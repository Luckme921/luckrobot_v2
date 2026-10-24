# 2026-10-23 导航延迟、转向与安全链修复记录

## 状态与边界

已通过 SSH 部署并编译到 `/home/nvidia/ros2_ws`。本次没有启动底盘、电机、导航硬件，也没有发送实车目标。完成的是软件修复及隔离测试，不是实车碰撞安全验收。

保持导航最高线速度 0.30 m/s。下一轮先显式使用 `motion_speed_limit:=0.15`；实测通过后再恢复 0.30。不能仅凭 7.8 kg 质量推断刹停距离或转动惯量。

已知实车：直径 0.350 m、轮径 0.100 m、轮距 0.269 m；IMU 倒装，原始 IMU Z 角速度转到车体坐标系需要改变符号。历史存在底盘打滑，不把所有历史异常归因为软件。未改轮径、轮距、相机/雷达/IMU 外参、EKF 融合配置或电机协议寄存器。

## 证据与尚未证实的原因

主要数据：`/home/nvidia/custom_nav_20261023_110341.tar.gz`，录制 334.20 秒。另检查 Jetson 同次启动日志，而不是只依据截图。

- 控制器配置 10 Hz，速度平滑器 20 Hz。底盘虽按 100 Hz 定时，串口问答限制下轮速反馈约 31 Hz。提高定时频率不能直接消除串口耗时。
- 包中出现 12 次无安全局部轨迹、4 次底盘指令心跳超时、4 次控制器耐心超时。
- 相机安全扫描的“采集时间到录包接收时间”中位数约 0.092 秒，P95 约 0.301 秒，最大约 0.511 秒；规划扫描最大约 1.479 秒。
- TF 在 bag 内的接收延迟明显；部分日志几秒前产生却集中到达。**录包接收延迟不能直接等同于每个控制节点看到的数据延迟。** 全量 costmap 消息间隔也不能单独用于断定停更，另有增量更新。
- 录包退出约在 epoch `1792725014.388`，后续真实节点日志中 map TF 停在 `1792725014.390`，随后出现过期变换错误；之后 odom TF 也冻结。这是实际故障，而非单纯 RViz 问题。
- Fast DDS 原先使用自定义 SHM + UDP，存在残留 SHM 文件及录包退出相关异常。作为有证据支持的排查方向，改用限定于本导航启动环境的 UDP + 异步发布；**没有死锁堆栈，不能断言 SHM 就是唯一根因，也不能凭短时测试称长期故障已根治。**
- 电机响应延迟的简单相关拟合受无时间戳 Twist 和录包积压影响，不作为硬件响应时间结论。没有准确碰撞时间及原始深度/RGB，尚不能排除视野盲区、漏检、近距深度失效或机械响应问题。

## 本次改动

1. 新增最终数据新鲜度保护节点。链路：控制器 → 平滑器 `/cmd_vel` → Collision Monitor `/cmd_vel_safe` → 新保护 `/cmd_vel_drive` → 底盘。
   - 同时检查 map TF、odom TF、EKF、原始轮里程计、IMU、雷达、相机的采集时间和单调时钟接收间隔；重复/冻结的时间戳不能维持健康状态。
   - 数据缺失/过期或命令断流时输出零速度。恢复需连续健康 0.5 秒及新的速度命令，不重放缓存非零速度。
   - map TF 限值 0.75 秒；odom TF、EKF、轮反馈、IMU 为 0.30 秒；扫描 0.35 秒；命令 0.20 秒。
   - 线/角速度同比例限幅保留请求曲率。仅故障恢复时额外缓起步，正常左右转向切换不重复滤波或插入停车。碰撞模块的减速/归零不被该节点延迟。
   - **输出零速度不等于瞬间物理停车**：底盘仍有减速过程，需实测刹停距离。该软件保护不是安全认证急停。
2. 控制器拒绝过期位姿/变换，执行反馈采用测量时间戳而非回调收到时间。
3. 相机深度/确认输入采用最新帧队列；拒绝过期、乱序数据；最新 TF 回退限制在 0.20 秒时差内，不用冻结 TF 重新标记扫描时间。TF 失败时不伪造无障碍扫描。原有 5 帧、至少 7 束连续表面确认保留；即时安全扫描不等待这 5 帧。
4. 速度平滑器启用线/角速度耦合缩放，命令超时 1.0 → 0.25 秒。线加速度 0.20 m/s²、线减速度 0.50 m/s²保持。局部 costmap 更新 3 → 5 Hz。全局/局部膨胀半径均为 0.25 m；代价衰减系数仍分别为 15/20，并非所有参数完全相同。
5. Collision Monitor 变换等待 0.50 → 0.05 秒，源超时 1.0 → 0.35 秒，预测圆半径 0.19 → 0.22 m；实际 URDF 半径仍为 0.175 m。Humble 无效源可能被忽略，因此新保护节点补充缺失/过期时的归零。
6. 底盘命令队列深度 1，轮里程计队列缩短并限制消息寿命；看门狗使用单调时钟；拒绝非有限速度。未盲改电机帧的加速字段。
7. 新增 `tools/robot_env.sh` 统一 DDS 环境；只改变本导航启动环境，不覆盖系统旧 SHM 配置。录包采用观察者 best-effort QoS，记录最终命令、保护状态，并附带每秒 tegrastats 性能采样。打包压缩降低进程优先级。

## 验证

- Jetson 编译：chassis_driver、luckrobot_nav2_plugins、my_robot_navigation、my_robot_bringup 成功。
- 单元测试：新鲜度/速度策略 13 项，相机确认历史 8 项，全通过。
- ROS_DOMAIN_ID=191 隔离集成：22 项检查通过，包括旧数据、断流、恢复、左右转向不插入停车、3 次录包启停。只发布合成消息，无真实硬件。
- 结果：`/tmp/luckrobot_guard_test_2393ylpb/results.json`；注意 /tmp 不承诺跨重启保留。
- 20 个关键本地/远程源码及配置 SHA256 核对一致；关键 installed 文件与源码一致。
- `--show-args`、Python 测试及脚本语法通过；没有实车运动验收。

## 下次测试：MobaXterm 三个 SSH 终端

机器人落地放稳、人员离开行驶路径，操作者可随时物理急停。不要在建图期间抱起机器人；不先做长距离/狭窄通道/人员挡车测试。

终端 1（新建地图；关闭自动 RViz）：

```bash
source ~/ros2_ws/tools/robot_env.sh
ros2 daemon stop
ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping navigation_profile:=luck_full depth_obstacle_mode:=height \
  use_rviz2:=false use_rviz:=false motion_speed_limit:=0.15
```

终端 2（MobaXterm 会话需打开 X11 forwarding；此命令不修复已有 OpenGL 兼容性问题）：

```bash
source ~/ros2_ws/tools/robot_env.sh
rviz2 -d ~/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/rviz_config/rviz2_config1.rviz
```

终端 3：

```bash
source ~/ros2_ws/tools/robot_env.sh
bash ~/ros2_ws/tools/run_custom_nav_diag.sh 0
```

等“记录已开始”再发目标。先空旷直行 0.8–1.0 m，再分别左、右转弯；一次一目标。结束按一次 Ctrl+C，等待打包完成，提供路径和实车现象。若出现停顿，先不要不断重发目标，也不要放宽超时掩盖问题；可在另一个终端查看：

```bash
source ~/ros2_ws/tools/robot_env.sh
ros2 topic echo /motion_freshness_guard/status
```

相机障碍测试先静止确认障碍被安全扫描识别，再做低速验证；不能用人体或“碰上才算失败”作为测试方法。0.15 通过后另起一次 0.30 对照，验证最大速度下制动、转角误差与数据时延。该轮暂不叠加语义推理，之后再做导航+推理性能对照。

## 备份与参考

原版本备份：`/home/nvidia/ros2_ws/backups/20261023_latency/before.tar.gz`，SHA256 `790ea6b336594ff9c9d22caf3d46cc73d2446bbdb529f2257177afe5ef8eab9c`。回退前须停止硬件并按文件清单恢复，不直接覆盖整个工作区。本地根目录不是 Git 仓库，本轮没有声称完成 Git 提交或推送。

- [Humble Collision Monitor 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/src/collision_monitor_node.cpp)
- [Humble 速度平滑器源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_velocity_smoother/src/velocity_smoother.cpp)
- [Fast DDS 2.6 传输配置](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/xml_configuration/transports.html)
- [rmw_fastrtps 发布模式](https://github.com/ros2/rmw_fastrtps#change-publication-mode)
