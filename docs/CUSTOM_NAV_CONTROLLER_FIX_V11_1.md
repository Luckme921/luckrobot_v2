# LuckRobot 自定义控制器终点死锁与摆动修复 V11.1

## 1. 两次实车记录的结论

本轮分析的是：

- `custom_nav_20260907_214005.tar.gz`：`SmacPlanner2D + LuckController`
- `custom_nav_20260907_214428.tar.gz`：`LuckPlanner + LuckController`

两组记录得出相同结论：问题主要在 V11.0 控制器的终点状态切换，不是
Collision Monitor，也不是全局规划器首先造成的。

| 指标 | 第一包 | 第二包 |
|---|---:|---:|
| 最后停留距离 | 0.091 / 0.102 m | 0.109~0.110 m |
| 0.20 m 内角速度稳定换向 | 12 / 8 次 | 20 次 |
| 横向误差最大值 | 0.057 m | 0.027 m |
| Collision Monitor 改写配对命令 | 0 次 | 0 次 |

V11.0 的 `goal_rotate_dist=0.12 m`，但 Nav2 的
`general_goal_checker.xy_goal_tolerance=0.08 m`。控制器先在 12 cm 内停止平移，
而 Nav2 要求进入 8 cm 才锁定 XY 并允许只调整朝向，因此机器人会停在
9~11 cm，永远无法满足到点条件，并在终点附近反复左右旋转。

第二包启用了 LuckPlanner，但同样卡在 0.11 m；两个规划器的路径横向误差都很小，
所以本轮不修改 LuckPlanner，也不修改稳定 RPP 基线。

## 2. V11.1 修改

1. LuckController 每个控制周期通过 Nav2 `GoalChecker::getTolerances()` 读取当前
   Goal Checker 的真实 XY/Yaw 容差；终点调姿距离永远不大于其 XY 容差。
2. 配置中的 `goal_rotate_dist` 从 0.12 m 改为 0.08 m，Yaw 容差与
   `general_goal_checker` 统一为 0.10 rad。
3. 终点角速度由比例控制和制动距离包络共同限制，`rotate_gain` 从 1.5 降为 1.0。
4. 关闭控制器内部第二重角加速度限幅，只保留下游 `velocity_smoother` 的
   `+0.30/-0.80 rad/s²`，避免越过目标朝向后命令反向滞后。
5. `bt_loop_duration` 设为 20 ms（50 Hz），避免实测持续出现 100 Hz BT 超期，
   同时不改变 10 Hz 控制频率。
6. `/luck_controller/tracking_error` 扩为 11 个数，新增目标 yaw 误差、是否处于
   最终调姿、当前 Goal Checker 的 XY 容差。
7. 诊断脚本每 15 秒显示剩余时间，避免录包阶段看起来“没有反应”。

## 3. Jetson 部署

终端 1、自动探索和所有 Nav2 进程均关闭后，在终端 2 执行：

```bash
cd ~/ros2_ws

sha256sum luckrobot_custom_nav_controller_fix_v11_1_20260907.zip

stamp=$(date +%Y%m%d_%H%M%S)
tar -czf ~/luckrobot_before_custom_nav_v11_1_${stamp}.tar.gz \
  src/luckrobot_nav2_plugins \
  src/my_robot_navigation \
  src/my_robot_bringup \
  tools

unzip -o luckrobot_custom_nav_controller_fix_v11_1_20260907.zip -d ~/ros2_ws

sha256sum -c docs/CUSTOM_NAV_CONTROLLER_FIX_V11_1_MANIFEST.sha256

source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select luckrobot_nav2_plugins my_robot_navigation my_robot_bringup \
  --allow-overriding luckrobot_nav2_plugins my_robot_navigation my_robot_bringup

source ~/ros2_ws/install/setup.bash
```

## 4. 单变量复测

本轮只测 `SmacPlanner2D + LuckController`，不要启动自动探索，也不要使用
`luck_full`。终端 1：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping \
  navigation_profile:=luck_controller
```

终端 2先确认运行参数并录制：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 param get /bt_navigator bt_loop_duration
ros2 param get /controller_server LuckFollowPath.goal_rotate_dist
ros2 param get /controller_server LuckFollowPath.goal_yaw_tolerance
ros2 param get /controller_server LuckFollowPath.rotate_gain
ros2 param get /controller_server LuckFollowPath.max_angular_accel
ros2 param get /controller_server LuckFollowPath.max_angular_decel

bash ~/ros2_ws/tools/run_custom_nav_diag.sh 90
```

预期依次为 `20, 0.08, 0.10, 1.0, 0.0, 0.8`。录制开始后在 RViz 只发送
一个 0.8~1.2 m 的开阔目标，让脚本自然结束。不要在同一包里连续发多个目标；
如果目标没有成功也不要用新目标抢占，保留故障现场直到录制结束。

验收条件：能进入 8 cm 范围；终点角速度不持续换向；导航返回成功；途中横向
误差不劣于 V11.0。通过后再单独测试原窄通道，最后才重新启用 `luck_full`。

随时可重启并回退到稳定基线：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping \
  navigation_profile:=stable
```
