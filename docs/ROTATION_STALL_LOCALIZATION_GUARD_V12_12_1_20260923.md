# LuckRobot V12.12.1 转向恢复能力修正

V12.12 的轮速角速度隔离方向正确，但将纯转向超时从 6 s 缩短到 3 s，
并加入了跨行为树重试的 30 s 强锁。实车日志证明这两项过于激进：

- 第二个导航目标在原地右转时出现起步停顿。
- 0.6 s 后已输出 0.35 rad/s 破静摩擦指令。
- 3 s 超时后强锁使所有 Clear/Wait/FollowPath 恢复都被立即拒绝。
- 锁过期后的新目标可以再次正常到达，证明导航树、深度扫描和底盘没有整体失效。

V12.12.1 因此：

1. 完全删除 30 s 转向故障强锁。
2. 恢复 V12.11 已验证的 6 s 纯转向超时。
3. 保留 0.6 s 后提升到 0.35 rad/s 的破静摩擦策略。
4. 仅保留 V12.12 中不影响运动能力的定位修正：生产 EKF 和 VSLAM 候选 EKF
   不再融合轮速 `vyaw`，转向角速度只由 IMU 提供。

这样在转向暂时不起步时，原有 BT 仍可清图、等待和重规划；即使轮端产生假旋转，
`odom -> base_footprint` 也不再因轮速角速度而漂移。

## 部署

```bash
cd ~/ros2_ws

sha256sum luckrobot_rotation_stall_localization_guard_v12_12_1_20260923.zip

unzip -o \
luckrobot_rotation_stall_localization_guard_v12_12_1_20260923.zip \
-d ~/ros2_ws

sha256sum -c \
docs/ROTATION_STALL_LOCALIZATION_GUARD_V12_12_1_20260923_MANIFEST.sha256

source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select luckrobot_nav2_plugins my_robot_navigation my_robot_bringup \
  --allow-overriding luckrobot_nav2_plugins my_robot_navigation my_robot_bringup
```

然后必须完全停止旧导航树并重新启动。

```bash
source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh

ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping navigation_profile:=luck_full depth_obstacle_mode:=height
```

验证：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 param get \
/controller_server LuckFollowPath.execution_rotation_stall_timeout

ros2 param get /ekf_filter_node odom0_config
```

应看到：

- `execution_rotation_stall_timeout = 6.0`
- `odom0_config` 只有第 7 项 `vx` 为 `True`，第 12 项 `vyaw` 为 `False`
- 不再存在 `LuckFollowPath.rotation_stall_latch_timeout` 参数

## RViz 中相机扫描为什么贴地

`/scan_camera_safety` 和 `/scan_camera` 是 `sensor_msgs/LaserScan`，不是原始三维点云。
`depth_height_scan` 先在三维相机坐标中按真实高度选出可能撞到车体的点，然后将它们投影成
`base_footprint` 平面上的二维扫描，供 Nav2 二维代价地图使用。因此 RViz 中这两个 LaserScan
显示在地面是正常的，不代表相机 TF 高度错误。

需要查看原始三维高度时，在 RViz 添加 `PointCloud2` 并选择相机的深度点云话题，
不要用 `/scan_camera*` 判断 Z 高度。
