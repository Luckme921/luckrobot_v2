# LuckRobot VSLAM 健康门控 V12.6

## 为什么不能直接融合原始 cuVSLAM

`auto_diag_20260914_200046` 中，主轮速/IMU 导航累计运动约 9.88 m，Nav2 和
V10.7 探索均保持稳定；但原始 cuVSLAM 在约 57--59 s 失锁，最大单帧位置跳变
30.57 m，视觉轨迹累计为 64.05 m。候选 EKF 随后又产生 51.56 m 的跳变。

原因不是发布频率不足。原始视觉里程计约 29.96 Hz，真正的问题是：

- `/visual_slam/tracking/odometry` 的位姿协方差为零；
- 有效置信度单独发布在
  `/visual_slam/tracking/vo_pose_covariance`；
- 旧候选 EKF 直接融合原始里程计，并把零协方差替换为很小的数值，错误地
  高度信任失锁后的视觉位姿。

## V12.6 数据链

```text
cuVSLAM raw odometry -----------+
cuVSLAM pose covariance --------+--> vslam_odom_guard
production /odometry/filtered --+        |
                                         +--> /vslam/health
                                         +--> /visual_slam/tracking/odometry_validated
                                                        |
/wheel/odom + /imu/data + validated VSLAM --------------+
                                                        v
                                             /odometry/vslam_candidate
```

门控节点执行以下检查：

1. 原始位姿与独立协方差时间同步；
2. 位置标准差不超过 0.20 m，航向标准差不超过 10 度；
3. 单帧位移、旋转、线速度和角速度满足底盘物理上限；
4. 与当前生产 `/odometry/filtered` 的缓慢偏离不超过安全阈值；
5. 失锁后停止发布视觉测量，连续 60 帧健康才恢复；
6. 恢复时锚定到当前生产 odom，避免 cuVSLAM 新原点造成跳变。

门控和候选 EKF 都不发布 TF，生产 Nav2 仍使用原来的
`/odometry/filtered`。因此本阶段依然是影子测试，不会接管小车。

对故障 rosbag 离线重放门控判据的结果：

```text
visual_samples=9059
accepted=3685
rejected=1472
recoveries=17
max_contiguous_guarded_step=0.023369m
RESULT=PASS
```

原来 30.57 m 的视觉跳变未进入受保护输出。

## 部署与构建

```bash
cd ~/ros2_ws
sha256sum luckrobot_vslam_health_guard_v12_6_20260914.zip
unzip -o luckrobot_vslam_health_guard_v12_6_20260914.zip -d ~/ros2_ws

source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select my_robot_bringup \
  --allow-overriding my_robot_bringup
source ~/ros2_ws/install/setup.bash
```

## 启动顺序

宿主机启动完整导航、双目输入和影子融合；不会自动启动探索：

```bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh
ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full \
  use_rviz2:=true \
  enable_stereo_ir:=true \
  start_vslam_candidate:=true
```

第二个终端进入现有 Isaac ROS 容器并启动 cuVSLAM：

```bash
bash ~/ros2_ws/tools/start_vslam_container.sh
```

容器中：

```bash
export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch luckrobot_vslam stereo_vslam_observe.launch.py
```

第三个终端验收：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic echo /vslam/health --once
timeout -s INT 8s ros2 topic hz \
  /visual_slam/tracking/odometry_validated
ros2 param get /ekf_filter_node_vslam_candidate publish_tf
```

期望 `/vslam/health` 为 `true`，受保护输出接近 30 Hz，候选 EKF 的
`publish_tf` 为 `false`。如果健康状态为 `false`，不要为了通过测试放宽阈值，
应先检查图像、遮挡、曝光和红外投射模式。

## 动态验收

先只做一次 120 s 旁路记录，不启动自动探索：

```bash
bash ~/ros2_ws/tools/run_vslam_diag.sh 120
```

依次完成开阔直线、原地转向和历史拐角；结束后提供最新
`~/vslam_diag_*.tar.gz`。至少三次不同起点的长时测试均无视觉大跳变、门控恢复
不过度频繁，才允许讨论生产融合。

