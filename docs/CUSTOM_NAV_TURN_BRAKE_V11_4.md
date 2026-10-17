# LuckRobot V11.4：掉头提前制动与可见诊断

## 1. 本轮证据与判断

`custom_nav_20260908_092417` 使用 `navigation_profile:=luck_full`，三段导航均
成功，最长一段由 3.298 m 收敛至 0.037 m，横向误差最大 0.050 m，说明坐标
方向、LuckPlanner 路径和 LuckController 基本闭环没有反号。但是第三段角速度
出现 16 次稳定换向，前视航向误差出现 11 次稳定换向。因此“最后能到点”并
不能否定实物看到的掉头过冲；该段属于稳定但欠阻尼的转向瞬态。

根因集中在原地对准阶段：旧逻辑在大角度掉头时一直给出最高
`0.25 rad/s`，接近 `rotate_exit_angle=0.15 rad`（8.6°）后才开始明显收速。
其后还有 10 Hz `velocity_smoother`、底盘速度斜坡和实物惯性，实际角速度不能
瞬间消失，容易越过路径方向，再用反向角速度修正。控制方向本身是正确的，
所以本轮不翻转坐标、不更换里程计，也不降低全程转速。

## 2. V11.4 修改

LuckController 新增 `rotate_braking_decel=0.20 rad/s²`，只计算原地掉头末段的
保守制动包络：

```text
w_limit = max(0.08, sqrt(2 * rotate_braking_decel *
                        (|heading_error| - rotate_exit_angle)))
```

在当前最高原地转速 0.25 rad/s 下，它约从 17.5° 剩余误差开始收速，到
8.6° 退出原地转向；0.08 rad/s 下限避免电机停在退出阈值之外。退出后仍保持
原有的线速度加角速度连续路径跟踪，不增加
“停住对齐”步骤，也不修改直线速度、路径增益、局部避障、Collision Monitor
或 Goal Checker。终点最终朝向仍沿用已有的独立制动逻辑。

诊断脚本也已修复。旧脚本在第一句提示之前同步调用生命周期和两个
`ros2 param dump`；当 Jetson 的参数服务繁忙时，看起来会无限无响应。现在脚本
立即显示 `[1/5]` 至 `[5/5]`，所有服务调用都有 4～6 秒上限；某项参数超时只
写入记录，不再无限阻塞 rosbag。分析器新增传感器到包周期统计，并会把多次
稳定角度反向明确报告为 `UNDERDAMPED_TRANSIENT`。

## 3. 本轮明确不改膨胀半径

保留全局 `0.30 m`、局部 `0.25 m`。两者不要求相等：全局较大的软膨胀带用于
让长路径优先远离墙角，局部较小的软膨胀带用于保留实车在窄处的机动空间。
真正禁止中心进入的 253 内切代价与 `robot_radius=0.175 m` 有关，不会因为把
全局外圈从 0.30 m 改成 0.25 m 而消失。现在缩小全局半径反而会让 LuckPlanner
更愿意贴近拐角，与 V11.3 刚修复的卡角问题方向相反；等转向 A/B 通过后再用
独立窄通道试验决定，不与本轮控制修复混改。

## 4. VSLAM 状态

V12 安装的是 Gemini 双目观测启动文件、候选 EKF 配置和同步检查工具，不包含
Isaac ROS cuVSLAM 二进制或容器。执行
`vslam_observe.launch.py start_camera:=true start_vslam:=false` 只启动相机观测，
`start_vslam:=false` 明确表示不启动 VSLAM；Docker 检查无输出也表明 Isaac ROS
容器环境尚未安装。

本次预检结果已经证明双目输入满足接入前提：左右各约 29.964 Hz、640×400
mono8、时间差 P95 为 0、最大 0.001 ms，右目标定基线 0.095320 m，结果为
`PASS`。这只是“相机和标定可用于后续 cuVSLAM”的结论，不等于 cuVSLAM 已部署。
主导航和 TF 仍是轮速/IMU EKF 加 `slam_toolbox`。

## 5. Jetson 部署与单变量复测

先关闭旧的导航终端和诊断脚本，再部署：

```bash
cd ~/ros2_ws
sha256sum luckrobot_custom_nav_turn_brake_v11_4_20260908.zip
unzip -o luckrobot_custom_nav_turn_brake_v11_4_20260908.zip -d ~/ros2_ws
sha256sum -c docs/CUSTOM_NAV_TURN_BRAKE_V11_4_MANIFEST.sha256

source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select luckrobot_nav2_plugins my_robot_navigation my_robot_bringup \
  --allow-overriding luckrobot_nav2_plugins my_robot_navigation my_robot_bringup
source ~/ros2_ws/install/setup.bash

ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping navigation_profile:=luck_full
```

另开终端确认新参数，再开始记录：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 param get /controller_server LuckFollowPath.rotate_braking_decel
ros2 param get /controller_server LuckFollowPath.max_trajectory_center_cost
ros2 param get /planner_server LuckGrid.max_planning_cost

bash ~/ros2_ws/tools/run_custom_nav_diag.sh 120
```

必须看到 `[5/5] rosbag 已启动` 后再在 RViz 发送**一个**目标：在开阔区域选择
车后方约 1～1.5 m 的点，使它完成一次接近 180° 掉头后直行。不要在同一包中
再发第二个点、启动自动探索或改参数。完成后 Ctrl-C，上传新的
`~/custom_nav_*.tar.gz`。

第一轮通过标准是：运行参数为 0.20；掉头最后约 20°不再越过目标方向后明显
反打；没有持续左右摇摆；目标仍正常到达；分析器的稳定航向误差换向优先降到
不超过 2 次。若仍过冲，只调整 `rotate_braking_decel`，不同时改膨胀层。

## 6. 对当前告警的处置边界

EKF 设为 50 Hz，每周期预算正好 20 ms。偶发约 20 ms 的 missed-rate 不会直接
让行为树失败，但连续 20～63 ms、一次 343 ms，并同时出现 SLAM 扫描队列满和
参数服务超时，说明当时有真实的 CPU/调度阻塞，可能放大控制反馈滞后。它们
不是 cuVSLAM 负载造成的，因为 cuVSLAM 尚未运行。

本轮先用提前制动与新录包隔离控制问题，不同时降低 EKF 频率。若新包仍持续
出现大于 40 ms 的 EKF 周期和扫描丢弃，再单独审计传感器频率、CPU/GPU/温度，
之后才决定把 EKF 从 50 Hz 调到 40 Hz；不能把严重超时当作无关噪声，也不能
在没有时序证据时直接改融合架构。
