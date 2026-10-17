# LuckRobot 原地转向静摩擦与真实运动门控 V12.11

## 结论

`custom_nav_20260922_201509` 不是深度相机、代价地图或 Collision Monitor
把速度清零。故障段中 `/cmd_vel_nav`、`/cmd_vel`、`/cmd_vel_safe` 一致持续输出
约 `-0.25 rad/s`，轮端反馈和 `/wheel/odom` 也报告旋转；但 `/imu/data` 在前
两次目标的约 25.9 秒内角速度为零、积分转角为零。这说明低速原地差速命令没有
克服实车静摩擦，而轮编码器进展误导了原执行监控。

## 修复

- LuckController 新增 `/imu/data` 真实转动见证。
- 仅在“纯原地转向 + IMU 新鲜 + IMU 连续 0.6 秒仍静止”时，把本周期角速度
  提升到现有安全上限 `0.35 rad/s`。IMU 检测到真实旋转后立即恢复原有
  `0.25 rad/s` 转向和制动包络。
- 增益发生在轨迹碰撞检查之前，因此更高转速对应的扫掠 footprint 仍会被验证；
  不改变直线、混合 `v+w`、深度避障、Collision Monitor 和底盘固件。
- 纯原地转向的执行失速判定不再把轮编码器当作整车旋转证据；IMU 可用时必须由
  IMU 证明物理旋转。IMU 暂时缺失时才回退到融合里程计，避免传感器掉线误停。
- 诊断分析新增 `safe-command/IMU physical rotation`，可以直接识别“有转向命令、
  编码器在走、实车不转”的区间。

## 部署

```bash
cd ~/ros2_ws

sha256sum luckrobot_rotation_breakaway_guard_v12_11_20260922.zip

unzip -o \
luckrobot_rotation_breakaway_guard_v12_11_20260922.zip \
-d ~/ros2_ws

sha256sum -c \
docs/ROTATION_BREAKAWAY_GUARD_V12_11_20260922_MANIFEST.sha256

source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select \
  luckrobot_nav2_plugins \
  my_robot_navigation \
  --allow-overriding \
  luckrobot_nav2_plugins \
  my_robot_navigation
```

## 实车验证

先彻底关闭旧导航栈，再启动唯一一套导航：

```bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full \
  depth_obstacle_mode:=height
```

在第二个终端确认生效：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 param get \
  /controller_server LuckFollowPath.rotation_breakaway_delay

ros2 param get \
  /controller_server LuckFollowPath.rotation_breakaway_angular_vel

ros2 param get \
  /controller_server LuckFollowPath.execution_feedback_imu_topic
```

预期依次是 `0.6`、`0.35`、`/imu/data`。

在空旷地面先启动持续记录，再连续发布 8～12 个需要先原地转向的目标。故障一旦
出现或全部完成，只按一次 `Ctrl-C`：

```bash
bash ~/ros2_ws/tools/run_custom_nav_diag.sh 0
```

验收标准：每个目标均应在约 1～2 秒内出现可见车体转动；允许日志偶尔出现一次
`applying bounded rotation breakaway`，但不允许持续 6 秒后触发 execution stall，
也不允许 Collision Monitor 被绕过。

