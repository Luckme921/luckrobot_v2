# LuckRobot V12.12 物理转向失败与定位漂移隔离

## 结论

`custom_nav_20260922_204323` 中的最终漂移不是深度相机噪点或激光雷达断流引起的。
车体在物理上没有旋转，但电机编码器仍报告左右轮旋转；轮速角速度进入 EKF
后使 `odom -> base_footprint` 虚假旋转，然后 SLAM Toolbox 扫描匹配无法继续消除这个误差。

关键证据：

- 540–600 s 内，轮速里程计报告约 518.9 deg 旋转。
- 同时 IMU 只观测到约 0.2 deg 真实旋转。
- EKF 产生约 115.5 deg 虚假旋转。
- `map -> odom` 在约 482 s 后基本不再修正。
- 激光雷达、IMU、深度高度扫描与 TF 数据流仍在。
- V12.11 的 0.35 rad/s 破静摩擦指令已多次输出，但最后车体仍不转。

因此，电机低速死区可能是最初停顿的触发因素，但不能解释最后的长时间失败。
0.35 rad/s 对应单轮约 9 RPM；该指令仍不能让车体转动时，必须按执行器卡滞/空转处理，
不能继续增大最小速度或无限重试。

## V12.12 改动

1. 生产 EKF 与 VSLAM 候选 EKF 不再融合 `/wheel/odom` 的 `vyaw`。
   - 保留轮速 `vx`。
   - 角速度只由经 TF 变换的 `/imu/data` 提供。
   - 纯原地空转时，左右轮的平均线位移接近零，因此不会再把假旋转写入定位。

2. 破静摩擦策略保持不变。
   - 仍允许控制器在 IMU 连续 0.6 s 未观测到转动时，把原地转向提升至 0.35 rad/s。
   - 不全局抬高最小角速度，避免恢复终点摆动和转过头。

3. 物理转向失败提前为 3 s 判定。
   - 0.6 s 后已使用安全上限破静摩擦。
   - 再给予 2.4 s，IMU 仍无车体转动就不再继续累积假里程计。

4. 同一目标的物理转向故障锁定 30 s。
   - 锁定状态跨越 BT 的 Clear/Wait/FollowPath 重试。
   - 行为树可以进行清图与重规划，但控制器不再向同一卡死转向反复输出非零速度。
   - 目标终点变化超过 0.20 m 时可立即解锁；否则最多等待 30 s。

## 部署

```bash
cd ~/ros2_ws

sha256sum luckrobot_rotation_stall_localization_guard_v12_12_20260923.zip

unzip -o \
luckrobot_rotation_stall_localization_guard_v12_12_20260923.zip \
-d ~/ros2_ws

sha256sum -c \
docs/ROTATION_STALL_LOCALIZATION_GUARD_V12_12_20260923_MANIFEST.sha256

source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select \
  luckrobot_nav2_plugins \
  my_robot_navigation \
  my_robot_bringup \
  --allow-overriding \
  luckrobot_nav2_plugins \
  my_robot_navigation \
  my_robot_bringup

source ~/ros2_ws/install/setup.bash
```

## 验收

必须停止旧导航树后重新启动，因为 EKF 配置只在节点启动时读取。

```bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh

ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full \
  depth_obstacle_mode:=height
```

新终端：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 param get \
  /controller_server LuckFollowPath.execution_rotation_stall_timeout

ros2 param get \
  /controller_server LuckFollowPath.rotation_stall_latch_timeout

ros2 param get /ekf_filter_node odom0_config

python3 ~/ros2_ws/tools/check_ekf_input_health.py --duration 30
```

期望：

- `execution_rotation_stall_timeout = 3.0`
- `rotation_stall_latch_timeout = 30.0`
- `odom0_config` 中只有 `vx` 为 `true`，`vyaw` 为 `false`
- EKF 健康检查 `RESULT=PASS`

随后仅做一次 0.8–1.2 m 直线和一次 90 deg 转向。如果车体再次物理不转，预期结果是：

- 约 0.6 s 后尝试 0.35 rad/s 破静摩擦。
- 总计约 3 s 仍不转时终止本次 FollowPath。
- 同目标重试不再输出转向指令。
- RViz 中机器人和激光点云不应再随轮端假旋转漂移。

## 物理层结论

若 0.35 rad/s 破静摩擦后车体仍不转，软件应安全失败而不再加速。
后续硬件排查只需区分：

- 车轮/电机轴在转，车体不转：检查轮胎打滑、万向轮、机械干涉、顶部或底盘被卡。
- 车轮与电机都不转，但编码器位置仍变化：检查电机驱动器反馈字段、编码器安装或协议解析。

这两种情况都不应再造成定位漂移；V12.12 的目标就是将“底盘故障”与“地图定位”隔离。
