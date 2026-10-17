# LuckRobot 统一启动入口

当前实车导航底座已经完成了可重复的主链：机器人 TF、A2M12、Gemini 深度障碍物、
轮速、IMU、主 EKF、slam_toolbox、Nav2 自定义规划/控制器、速度平滑器和
Collision Monitor。手动 RViz 目标的验收基线仍以 `luck_full` 为主；自动探索属于
下一阶段，不能与本阶段的底盘回归测试混在一起。

## 一条命令启动主系统

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh
ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping navigation_profile:=luck_full
```

这个入口内部只包含一个 `smart_navigation.launch.py` 实例，因此不会重复启动
底盘、雷达、相机、滤波器或 EKF。它默认启动的里程计链为：

```text
/wheel/odom + /imu/data -> /odometry/filtered -> Nav2 odom->base_footprint
```

## 可选开启双目和候选 EKF

cuVSLAM 安装在 Jetson 的 Isaac ROS Docker 容器中，不能从宿主机 launch 直接代替
容器启动。需要做 VSLAM A/B 时，主系统使用：

```bash
ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping navigation_profile:=luck_full \
  enable_stereo_ir:=true start_vslam_candidate:=true
```

随后在另一个终端启动现有 `start_vslam_container.sh`，容器内运行
`stereo_vslam_observe.launch.py`。候选 EKF 只发布 `/odometry/vslam_candidate`，
`publish_tf: false`，不会接管主 EKF、`map->odom` 或 Nav2；VSLAM 丢帧时主链仍可用。

## 验收检查

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /collision_monitor
ros2 topic hz /wheel/odom
ros2 topic hz /odometry/filtered
ros2 param get /ekf_filter_node publish_tf
```

只有主链四个生命周期节点均为 `active [3]` 后才发送 RViz 目标。启用候选 EKF 时另
行确认：

```bash
ros2 param get /ekf_filter_node_vslam_candidate publish_tf
ros2 topic hz /odometry/vslam_candidate
```

候选参数必须返回 `Boolean value is: false`；`/tf` 的“发布端点”存在不等于它在发送
TF，实际 TF 所有权仍由主 EKF 和 slam_toolbox 保持。

V12.6 起，候选链不能再直接融合原始 cuVSLAM 里程计。必须经过
`vslam_odom_guard`，由独立协方差、物理连续性和主里程计一致性共同判定后发布
`/visual_slam/tracking/odometry_validated`。完整说明见
`docs/VSLAM_HEALTH_GUARD_V12_6_20260914.md`。
