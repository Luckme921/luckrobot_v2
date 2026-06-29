/**
 * @file chassis_node.cpp
 * @brief 工业级两轮差速底盘底层驱动节点 (生产环境定稿版 - 专为 Odom 与 Nav2 优化)
 * * * =========================================================================================
 * 【AI 开发与项目交接文档】(非常重要，后续开发 Odom / TF 节点请严格遵循此文档)
 * =========================================================================================
 * * * 1. 物理架构与运动学参数 (Kinematics Parameters):
 * - 轮距 (WHEEL_BASE_MM): 269.0 mm
 * - 轮径 (WHEEL_DIAMETER_MM): 100.0 mm
 * - 电机 ID 分配与极性:
 * - [右轮 / RIGHT_WHEEL]: ID = 0x03。🚨 注意：物理安装为镜像反装！代码在下发命令时已对其进行反转(取负)。
 * - [左轮 / LEFT_WHEEL]:  ID = 0x04。正常安装。
 * * * 2. 核心控制突破：软硬协同 Velocity Smoother & 防积分饱和 (Anti-Windup):
 * - 【硬件 Bug 回避】: 底层驱动固件在接收 `0 RPM` 指令时，会无视内部斜坡寄存器直接瞬间抱死。
 * - 【宏观软件接管】: 彻底弃用硬件减速器。在 ROS 2 层引入 100Hz 微积分速度平滑算法 (Velocity Smoother)。
 * 停车时，软件层以 `MAX_DECEL_X` 的减速度将指令 (如 40->38->...->0) 逐步下发。
 * - 【微观硬件插值】: 指令帧 `DATA[6] = 0x32 (5ms/RPM)`，仅用于让硬件填补软件两次 10ms 调度间的电信号阶梯缝隙。
 * - 【Anti-Windup 限幅】: 接收 `cmd_vel` 后，在平滑器介入前根据电机极速 (300 RPM) 进行硬性截断 (Clamp)，
 * 彻底杜绝上位机异常大数值指令导致的“积分饱和”与“刹车卡死/延迟”现象。
 * * * 3. RS485 通信协议 (115200, 8N1, 一问一答半双工):
 * - 下发指令帧 (10 字节): [ID, 0x64(速度环), RPM_H, RPM_L, 0x00, 0x00, 0x32(硬件插值), DATA[7](刹车), 0x00, CRC8]
 * - 软制动: `DATA[7] = 0x00` (日常行车、防断联软停，依靠软件下发递减的 RPM 实现平滑)
 * - 硬急刹: `DATA[7] = 0xFF` (物理切断瞬间抱死，仅在节点生命周期结束、Ctrl+C销毁时作最后安全防线)
 * * * 4. 里程计 (Odometry) 数据预处理基石:
 * - 电机反馈位置为无符号 16-bit 整数 (0~32767，代表 0~360度)。
 * - 引入 `unwrap_position()` 算法处理 360 度跨越溢出，将其解卷为“无极累加的绝对物理弧度 (Radian)”。
 * - 【🚨 里程计符号极性规范】: 确保机器人向正前方直行时，发布的左右两轮“累加弧度”与“RPM”**必须均为正数**。
 * * * 5. ROS 2 节点接口 (I/O):
 * - [Sub] `/cmd_vel` (geometry_msgs::msg::Twist): 监听上层规划的目标线速度与角速度。
 * - [Pub] `/wheel_feedback` (std_msgs::msg::Float64MultiArray): 
 * 输出格式 -> [左轮绝对累加弧度, 右轮绝对累加弧度, 左轮实时RPM, 右轮实时RPM]
 * - [Pub] `/wheel/odom` (nav_msgs::msg::Odometry): 专供 EKF 融合的纯轮式里程计数据
 * =========================================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
// ==================== [新增]: 里程计与四元数相关头文件 ====================
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
// =====================================================================
#include <mutex>
#include <cmath>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <csignal>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include <sstream>

// ==================== 1. 硬件连接配置 ====================
// 使用 udev 的 by-id 路径，防止 Ubuntu 重启后 /dev/ttyUSB* 乱序导致失联
const char* SERIAL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_ABADW8KY-if00-port0";
const int BAUDRATE = B115200;

// ==================== 2. 运动学静态参数 ====================
constexpr double WHEEL_DIAMETER_MM = 100.0;
constexpr double WHEEL_BASE_MM = 269.0; 
constexpr double WHEEL_CIRCUMFERENCE_CM = M_PI * (WHEEL_DIAMETER_MM / 10.0); // 31.4159 cm

const uint8_t RIGHT_WHEEL_ID = 0x03;
const uint8_t LEFT_WHEEL_ID = 0x04;

// ==================== 3. 极速限制与防积分饱和 (Anti-Windup) ====================
constexpr int RPM_LIMIT = 300; // 核心安全阈值: 限制电机最大 RPM

// [编译期推导] 自动计算物理最大线速度与角速度，用于拦截不合法的畸形 cmd_vel
constexpr double MAX_PHYSICAL_LIN_VEL = (RPM_LIMIT / 60.0) * (WHEEL_CIRCUMFERENCE_CM / 100.0); // 约 1.57 m/s
constexpr double MAX_PHYSICAL_ANG_VEL = MAX_PHYSICAL_LIN_VEL / ((WHEEL_BASE_MM / 1000.0) / 2.0); // 约 11.67 rad/s

// ==================== 4. 电机通信协议常量 ====================
const uint8_t FUNC_CODE = 0x64;       // 0x64 速度环模式
const uint8_t RUN_BYTE = 0x00;        // 解除刹车
const uint8_t BRAKE_BYTE = 0xFF;      // 硬件抱死
const int FRAME_LENGTH = 10;          
const uint8_t CRC8_POLY_REV = 0x8C;   // MAXIM CRC8 反转多项式

// ==================== 5. 软件平滑器 (Velocity Smoother) 参数 ====================
// 决定机器人的驾驶手感：起步猛如虎，刹车稳如狗
constexpr double MAX_ACCEL_X = 1.2;  // 线起步加速度 (m/s^2)
constexpr double MAX_DECEL_X = 0.25; // 线刹车减速度 (m/s^2) - 0.2m/s 刹停约需 0.8s，极其优雅
constexpr double MAX_ACCEL_Z = 3.0;  // 角起步加速度 (rad/s^2)
constexpr double MAX_DECEL_Z = 1.5;  // 角刹车减速度 (rad/s^2)

// ==================== 6. 节点调度高可用参数 ====================
constexpr int CONTROL_FREQ = 100;         // ROS2 核心循环频率 100Hz (dt = 0.01s)
constexpr double CMD_VEL_TIMEOUT = 0.2;   // 看门狗保护超时阈值 (秒)
constexpr int RESPONSE_TIMEOUT_MS = 10;   // 串口 select 非阻塞最长等待时长
constexpr int RS485_DELAY_US = 300;       // RS485 总线半双工读写切换物理死区时间
constexpr int RS485_CMD_RETRY = 2;        // 单帧校验失败或丢失时的重试次数
constexpr int MAX_COMM_FAILURES = 5;      // 连续通讯失败阈值，超限则彻底重启串口

// ==================== 7. 全局句柄 (确保信号杀死进程时的安全) ====================
std::atomic<bool> g_node_running(false);
std::atomic<bool> g_signal_handled(false);
int g_serial_fd = -1;
struct sigaction g_old_sigint;

// ==================== 底层工具与系统调用 ====================

/**
 * @brief CRC-8/MAXIM 校验算法实现
 */
static inline uint8_t crc8_maxim(const uint8_t* data, int len) {
    if (data == nullptr || len <= 0) return 0;
    uint8_t crc = 0x00;
    for (int i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x01) crc = (crc >> 1) ^ CRC8_POLY_REV;
            else crc = crc >> 1;
        }
    }
    return crc;
}

/**
 * @brief 清空 Linux 内核串口缓冲区。每次发送前调用，防上一轮脏数据干扰解析。
 */
void clear_serial_buffer() {
    if (g_serial_fd < 0) return;
    tcflush(g_serial_fd, TCIOFLUSH);
}

/**
 * @brief 全局信号捕获。拦截 Ctrl+C 等强杀指令，强制下发 0xFF 硬件急刹保证人身安全。
 */
void signal_handler(int signum) {
    if (g_signal_handled.load()) return;
    g_signal_handled.store(true);
    sigaction(SIGINT, &g_old_sigint, nullptr);

    if (g_serial_fd >= 0) {
        uint8_t brake_frame[FRAME_LENGTH] = {0};
        brake_frame[1] = FUNC_CODE;
        brake_frame[7] = BRAKE_BYTE; // 核心：下发 0xFF 抱死指令
        
        brake_frame[0] = LEFT_WHEEL_ID;
        brake_frame[9] = crc8_maxim(brake_frame, 9);
        write(g_serial_fd, brake_frame, FRAME_LENGTH);
        tcdrain(g_serial_fd);
        usleep(RS485_DELAY_US);
        
        brake_frame[0] = RIGHT_WHEEL_ID;
        brake_frame[9] = crc8_maxim(brake_frame, 9);
        write(g_serial_fd, brake_frame, FRAME_LENGTH);
        tcdrain(g_serial_fd);
    }
    
    g_node_running.store(false);
    if (g_serial_fd >= 0) {
        close(g_serial_fd);
        g_serial_fd = -1;
    }
    rclcpp::shutdown();
}

// ==================== 核心控制节点 ====================
class ChassisNode : public rclcpp::Node {
public:
    ChassisNode() : Node("chassis_driver_node"),
                    target_linear_x_(0.0), target_angular_z_(0.0), watchdog_tripped_(false),
                    cur_lin_x_(0.0), cur_ang_z_(0.0),
                    left_last_raw_pos_(0), right_last_raw_pos_(0),
                    left_accumulated_rad_(0.0), right_accumulated_rad_(0.0),
                    first_read_left_(true), first_read_right_(true), comm_failures_(0),
                    odom_x_(0.0), odom_y_(0.0), odom_th_(0.0), odom_initialized_(false) // 初始化里程计变量
    {
        g_node_running.store(true);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ChassisNode::cmd_vel_callback, this, std::placeholders::_1));

        feedback_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_feedback", 10);

        // ==================== [新增]: 注册里程计发布器 ====================
        // 注意：为配合 robot_localization EKF 融合，我们只发布话题，不发布 TF
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);
        // ==================================================================

        if (!serial_init()) {
            RCLCPP_WARN(this->get_logger(), "未检测到底盘串口，系统进入后台自愈重连模式...");
        } else {
            RCLCPP_INFO(this->get_logger(), "=== 工业级底盘驱动就绪 (纯软件平滑接管 + Odom 解算) ===");
            RCLCPP_INFO(this->get_logger(), "当前硬件极速限幅: RPM=±%d | V_max=%.2f m/s | W_max=%.2f rad/s", 
                        RPM_LIMIT, MAX_PHYSICAL_LIN_VEL, MAX_PHYSICAL_ANG_VEL);
        }

        last_cmd_vel_time_ = this->now();
        last_odom_time_ = this->now(); // 记录初次里程计时间
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / CONTROL_FREQ),
            std::bind(&ChassisNode::control_loop, this));
    }

    /**
     * @brief 析构函数双重保险。如果是被 launch 文件正常销毁，同样触发硬件急刹保护。
     */
    ~ChassisNode() {
        g_node_running.store(false);
        // 如果信号拦截器没跑（比如正常生命周期退出），在这里补刀急刹
        if (!g_signal_handled.load() && g_serial_fd >= 0) {
            uint8_t brake_frame[FRAME_LENGTH] = {0};
            brake_frame[1] = FUNC_CODE;
            brake_frame[7] = BRAKE_BYTE;
            
            brake_frame[0] = LEFT_WHEEL_ID;
            brake_frame[9] = crc8_maxim(brake_frame, 9);
            write(g_serial_fd, brake_frame, FRAME_LENGTH);
            tcdrain(g_serial_fd);
            usleep(RS485_DELAY_US);
            
            brake_frame[0] = RIGHT_WHEEL_ID;
            brake_frame[9] = crc8_maxim(brake_frame, 9);
            write(g_serial_fd, brake_frame, FRAME_LENGTH);
            tcdrain(g_serial_fd);
            
            close(g_serial_fd);
            g_serial_fd = -1;
        }
    }

private:
    /**
     * @brief Linux POSIX 串口初始化 (RAW模式, 无流控, 8N1)
     */
    bool serial_init() {
        if (g_serial_fd >= 0) {
            close(g_serial_fd);
            g_serial_fd = -1;
        }

        g_serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (g_serial_fd < 0) return false;

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(g_serial_fd, &tty) != 0) return false;

        cfsetispeed(&tty, BAUDRATE);
        cfsetospeed(&tty, BAUDRATE);

        tty.c_cflag |= (CLOCAL | CREAD | CS8);
        tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        
        // 配合 select 实现严格受控的超时阻塞
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(g_serial_fd, TCSANOW, &tty) != 0) return false;
        clear_serial_buffer();
        return true;
    }

    /**
     * @brief 获取控制指令。快速执行，无复杂耗时计算。
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        target_linear_x_ = msg->linear.x;
        target_angular_z_ = msg->angular.z;
        last_cmd_vel_time_ = this->now(); // 喂狗：重置看门狗定时器
        watchdog_tripped_ = false;
    }

    /**
     * @brief 逆运动学算法：底盘中心速度 (V, W) 解算至双轮物理 RPM
     */
    void calculate_wheel_rpm(double v_m_s, double w_rad_s, int& left_rpm, int& right_rpm) {
        double v_cm_s = v_m_s * 100.0;
        double angular_cm = w_rad_s * (WHEEL_BASE_MM / 20.0); 

        // 差速运动学推导
        double left_vel_cm = v_cm_s - angular_cm;
        double right_vel_cm = v_cm_s + angular_cm;
        
        // 🚨 物理约束：右侧电机反装，发令前必须翻转符号以保持整车向前推力！
        right_vel_cm = -right_vel_cm; 

        // 终极保护：即使上层经过了软限幅，底层浮点数计算后依旧通过 clamp 确保数据绝不越界
        left_rpm = std::clamp(static_cast<int>(round((left_vel_cm * 60.0) / WHEEL_CIRCUMFERENCE_CM)), -RPM_LIMIT, RPM_LIMIT);
        right_rpm = std::clamp(static_cast<int>(round((right_vel_cm * 60.0) / WHEEL_CIRCUMFERENCE_CM)), -RPM_LIMIT, RPM_LIMIT);
    }

    /**
     * @brief 有符号整数转 16-bit 补码传输字节 (高低位拆分)
     */
    std::pair<uint8_t, uint8_t> rpm_to_2bytes(int rpm) {
        uint16_t int16_val = (rpm < 0) ? (0x10000 + rpm) : static_cast<uint16_t>(rpm);
        return {static_cast<uint8_t>((int16_val >> 8) & 0xFF), static_cast<uint8_t>(int16_val & 0xFF)};
    }

    /**
     * @brief Odom 里程计命脉：16-bit 编码构循环位置解卷算法
     * @details 将 0-32767 的有界刻度，通过判断相邻两帧差值 (Delta) 是否大于半圈 (16384)，
     * 无缝拼接、积分为无限增长的绝对物理弧度 (Radian)。
     */
    double unwrap_position(uint16_t raw_pos, uint16_t& last_pos, double& accumulated_rad, bool& is_first) {
        if (is_first) {
            last_pos = raw_pos;
            is_first = false;
            return accumulated_rad;
        }
        int delta = static_cast<int>(raw_pos) - static_cast<int>(last_pos);
        
        // 检测过零点溢出：如果差值异常大，说明物理上跨越了 0 点界限
        if (delta > 16384) delta -= 32768;        // 反转变号跨越
        else if (delta < -16384) delta += 32768;  // 正向变号跨越

        last_pos = raw_pos;
        // 映射比例：32768 个编码器刻度代表电机轴旋转 360 度 (2.0 * PI 弧度)
        accumulated_rad += (static_cast<double>(delta) / 32768.0) * (2.0 * M_PI);
        return accumulated_rad;
    }

    /**
     * @brief 核心通信管线：请求指令与读取状态反馈 (带重试纠错能力)
     */
    bool send_and_read_motor(uint8_t id, int target_rpm, int16_t& feedback_rpm, uint16_t& feedback_pos) {
        if (g_serial_fd < 0) return false;

        uint8_t frame[FRAME_LENGTH] = {0};
        frame[0] = id;
        frame[1] = FUNC_CODE;
        // 【硬件微插值】: 0x32(十进制50) = 5ms/RPM. 
        // 恰好填补 100Hz 软件平滑器产生的 10ms 阶梯缝隙，杜绝电机高频嗡嗡声。
        frame[6] = 0x32;  
        // 软件接管宏观加减速，彻底弃用底层 0xFF 硬件抱死（防卡死 Bug）
        frame[7] = RUN_BYTE; 
        
        auto [high, low] = rpm_to_2bytes(target_rpm);
        frame[2] = high;
        frame[3] = low;
        
        frame[9] = crc8_maxim(frame, 9);

        // 带丢包/错包自动重发机制
        for (int retry = 0; retry < RS485_CMD_RETRY; ++retry) {
            clear_serial_buffer(); // 发送前严格清空缓冲区
            if (write(g_serial_fd, frame, FRAME_LENGTH) != FRAME_LENGTH) continue;
            tcdrain(g_serial_fd); // 阻塞直到串行缓冲中数据物理发出
            
            uint8_t resp[FRAME_LENGTH] = {0};
            fd_set read_fds;
            struct timeval timeout;
            FD_ZERO(&read_fds);
            FD_SET(g_serial_fd, &read_fds);
            timeout.tv_sec = 0;
            timeout.tv_usec = RESPONSE_TIMEOUT_MS * 1000;

            if (select(g_serial_fd + 1, &read_fds, nullptr, nullptr, &timeout) > 0) {
                if (read(g_serial_fd, resp, FRAME_LENGTH) == FRAME_LENGTH) {
                    if (resp[0] == id && resp[9] == crc8_maxim(resp, 9)) {
                        feedback_rpm = (static_cast<int16_t>(resp[4]) << 8) | resp[5];
                        feedback_pos = (static_cast<uint16_t>(resp[6]) << 8) | resp[7];
                        usleep(RS485_DELAY_US); // 让出半双工总线时间片，防碰撞
                        return true;
                    }
                }
            }
        }
        return false;
    }

    // ==================== [新增]: 里程计正向运动学解算 ====================
    /**
     * @brief 纯轮式里程计解算 (通过双轮物理弧度积分推导当前 X, Y, Yaw)
     * @note 专供 EKF 融合使用，因此仅发布 /wheel/odom 话题，严禁发布 TF (防冲突)
     */
    void publish_odometry(double current_left_rad, double current_right_rad) {
        rclcpp::Time current_time = this->now();

        // 第一次进来只记录初始状态，不计算位移
        if (!odom_initialized_) {
            last_odom_left_rad_ = current_left_rad;
            last_odom_right_rad_ = current_right_rad;
            last_odom_time_ = current_time;
            odom_initialized_ = true;
            return;
        }

        double dt = (current_time - last_odom_time_).seconds();
        if (dt <= 0.0) return;

        // 1. 获取两次调度间(10ms)轮子转过的增量弧度
        double d_left_rad = current_left_rad - last_odom_left_rad_;
        double d_right_rad = current_right_rad - last_odom_right_rad_;

        // 2. 将设计常量转为国际标准单位 (米)
        double R = WHEEL_DIAMETER_MM / 2000.0; // 轮半径 0.05 m
        double L = WHEEL_BASE_MM / 1000.0;     // 轮距 0.269 m

        // 3. 计算左右轮各自在地面滚过的物理距离 (m)
        double d_left_dist = d_left_rad * R;
        double d_right_dist = d_right_rad * R;

        // 4. 正向运动学 (Forward Kinematics) 求解底盘中心位移量
        double d_s = (d_right_dist + d_left_dist) / 2.0; // 底盘中心前进距离
        double d_th = (d_right_dist - d_left_dist) / L;  // 底盘中心旋转角度差

        // 5. 坐标系积分推演 (中点欧拉法，精度高于基本欧拉法)
        double dx = d_s * cos(odom_th_ + d_th / 2.0);
        double dy = d_s * sin(odom_th_ + d_th / 2.0);

        odom_x_ += dx;
        odom_y_ += dy;
        odom_th_ += d_th;

        // 6. 估算瞬时线速度与角速度
        double vx = d_s / dt;
        double vth = d_th / dt;

        // 7. 组装并发布 /wheel/odom 消息
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        
        odom.pose.pose.position.x = odom_x_;
        odom.pose.pose.position.y = odom_y_;
        odom.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_th_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.angular.z = vth;
        
        // 🚨 注入极其关键的协方差矩阵 (Covariance) 供 EKF 权重分配
        // 轮式里程计在 X 轴和 Y 轴极其可信 (0.01)，但在 Yaw 轴因为车轮打滑极不可信 (0.1)
        odom.pose.covariance[0] = 0.01;  
        odom.pose.covariance[7] = 0.01;  
        odom.pose.covariance[35] = 0.1;  
        odom.twist.covariance = odom.pose.covariance;
        
        odom_pub_->publish(odom);

        // 8. 刷新历史记录
        last_odom_left_rad_ = current_left_rad;
        last_odom_right_rad_ = current_right_rad;
        last_odom_time_ = current_time;
    }
    // =====================================================================

    /**
     * @brief 100Hz ROS 2 控制闭环守护线程
     */
    void control_loop() {
        if (!g_node_running.load() || !rclcpp::ok()) return;

        // 通信链路健康自愈维护
        if (g_serial_fd < 0) {
            if (serial_init()) {
                RCLCPP_INFO(this->get_logger(), "✅ 底盘串口重连成功");
                comm_failures_ = 0;
                first_read_left_ = true;  // 重新标定编码器解卷基准点
                first_read_right_ = true;
                odom_initialized_ = false; // [新增]: 断网重连后重置里程计积分起点
            } else {
                return;
            }
        }

        double time_since_cmd = (this->now() - last_cmd_vel_time_).seconds();
        bool is_timeout = (time_since_cmd > CMD_VEL_TIMEOUT);

        double target_x = 0.0;
        double target_z = 0.0;

        if (!is_timeout) {
            std::lock_guard<std::mutex> lock(vel_mutex_);
            // =========================================================================
            // 【极其重要：硬性截断积分饱和 (Anti-Windup)】
            // 必须在平滑器介入前，将规划器下发的不切实际目标截断在物理极速阈值内！
            // 防止系统累积了无法达到的虚假目标导致刹车失效 (如目标累积到了 10m/s，要刹车时等半天)。
            // =========================================================================
            target_x = std::clamp(target_linear_x_, -MAX_PHYSICAL_LIN_VEL, MAX_PHYSICAL_LIN_VEL);
            target_z = std::clamp(target_angular_z_, -MAX_PHYSICAL_ANG_VEL, MAX_PHYSICAL_ANG_VEL);
        } else {
            // 看门狗超时断联：将平滑目标切为 0
            if (!watchdog_tripped_) {
                RCLCPP_WARN(this->get_logger(), "指令心跳超时！底盘已启动自主滑行制动");
                watchdog_tripped_ = true;
            }
        }

        // =========================================================================
        // 【核心平滑中枢】：100Hz 软件微分速度限制器 (Velocity Smoother)
        // 抛弃硬件一刀切控制。通过微步长逼近目标速度，完美兼容起步猛烈与驻停轻柔。
        // =========================================================================
        double dt = 1.0 / CONTROL_FREQ;

        // 1. 线速度平滑推演
        if (target_x != cur_lin_x_) {
            // 同向且绝对值正在扩大定义为“加速”
            bool is_accel = (target_x * cur_lin_x_ >= 0) && (std::abs(target_x) > std::abs(cur_lin_x_));
            double step = (is_accel ? MAX_ACCEL_X : MAX_DECEL_X) * dt;
            
            if (target_x > cur_lin_x_) cur_lin_x_ = std::min(target_x, cur_lin_x_ + step);
            else cur_lin_x_ = std::max(target_x, cur_lin_x_ - step);
        }

        // 2. 角速度平滑推演
        if (target_z != cur_ang_z_) {
            bool is_accel = (target_z * cur_ang_z_ >= 0) && (std::abs(target_z) > std::abs(cur_ang_z_));
            double step = (is_accel ? MAX_ACCEL_Z : MAX_DECEL_Z) * dt;
            
            if (target_z > cur_ang_z_) cur_ang_z_ = std::min(target_z, cur_ang_z_ + step);
            else cur_ang_z_ = std::max(target_z, cur_ang_z_ - step);
        }

        // 映射至底层电机转速指令
        int left_rpm = 0, right_rpm = 0;
        calculate_wheel_rpm(cur_lin_x_, cur_ang_z_, left_rpm, right_rpm);

        int16_t fb_left_rpm = 0, fb_right_rpm = 0;
        uint16_t left_raw_pos = 0, right_raw_pos = 0;

        // 严格遵循半双工令牌环：顺序读取，避免总线碰撞
        bool left_ok = send_and_read_motor(LEFT_WHEEL_ID, left_rpm, fb_left_rpm, left_raw_pos);
        bool right_ok = send_and_read_motor(RIGHT_WHEEL_ID, right_rpm, fb_right_rpm, right_raw_pos);

        if (!left_ok || !right_ok) {
            comm_failures_++;
            if (comm_failures_ >= MAX_COMM_FAILURES) {
                RCLCPP_ERROR(this->get_logger(), "物理层通信严重丢帧，断网自愈逻辑触发...");
                close(g_serial_fd);
                g_serial_fd = -1;
                comm_failures_ = 0;
            }
        } else {
            comm_failures_ = 0;

            // 获取绝对解卷物理弧度
            double raw_left_rad = unwrap_position(left_raw_pos, left_last_raw_pos_, left_accumulated_rad_, first_read_left_);
            double raw_right_rad = unwrap_position(right_raw_pos, right_last_raw_pos_, right_accumulated_rad_, first_read_right_);

            // =========================================================================================
            // 【里程计反馈符号一致性强制对齐】
            // 物理规律还原：当机器人向正前方位移时...
            // - 左轮原始编码器值跑向负数 (与电机内部绕组相位有关)，故需取负修正为正。
            // - 右轮由于倒装下发指令强制为负，致使它的内部物理读数已经跑向正数，直接取值即可。
            // 最终目标：无论如何接线，发布给 Odom 节点的 `out_x_rad` 和 `out_x_rpm` 必须是正前方向均为正。
            // =========================================================================================
            double out_left_rad = -raw_left_rad;
            double out_right_rad = raw_right_rad; 
            
            // 速度反馈极性同理校准
            double out_left_rpm = static_cast<double>(fb_left_rpm);
            double out_right_rpm = -static_cast<double>(fb_right_rpm); 

            std_msgs::msg::Float64MultiArray feedback_msg;
            feedback_msg.data = {out_left_rad, out_right_rad, out_left_rpm, out_right_rpm};
            feedback_pub_->publish(feedback_msg);

            // ==================== [新增]: 触发纯轮式 Odom 推演 ====================
            publish_odometry(out_left_rad, out_right_rad);
            // =====================================================================
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feedback_pub_;
    // [新增] 里程计发布器
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::mutex vel_mutex_;
    double target_linear_x_, target_angular_z_;
    rclcpp::Time last_cmd_vel_time_;
    bool watchdog_tripped_;

    // 软件平滑器积分状态量 (历史状态记录)
    double cur_lin_x_;
    double cur_ang_z_;

    // 编码器推演核心变量 (无限解卷)
    uint16_t left_last_raw_pos_, right_last_raw_pos_;
    double left_accumulated_rad_, right_accumulated_rad_;
    bool first_read_left_, first_read_right_;
    int comm_failures_;

    // [新增] 运动学正向解算(Odom)核心坐标与时间锚点
    double odom_x_, odom_y_, odom_th_;
    double last_odom_left_rad_, last_odom_right_rad_;
    rclcpp::Time last_odom_time_;
    bool odom_initialized_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, &g_old_sigint);

    auto node = std::make_shared<ChassisNode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}