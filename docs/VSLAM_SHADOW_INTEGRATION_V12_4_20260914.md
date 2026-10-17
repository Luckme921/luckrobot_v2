# VSLAM 影子融合与全屋探索验收 V12.4

## 阶段结论

LuckRobot 的 `luck_full` 自定义导航已经通过开阔直线、约 90 度转向、历史
拐角和一次 V10.7 全屋探索验收。V10.7 运行中出现过一次导航中止，但探索器
正确暂停该分支、切换到其他分支，并在连续 15 秒没有安全候选后正常结束。

cuVSLAM 的短程直线和原地转向尺度已经通过，但这不足以直接替换生产里程计。
本阶段采用影子融合：

```text
生产链（保持不变）
/wheel/odom + /imu/data -> /odometry/filtered -> Nav2

候选链（只观察）
/wheel/odom + /imu/data + /visual_slam/tracking/odometry
    -> /odometry/vslam_candidate
```

候选 EKF 必须保持 `publish_tf: false`。因此 cuVSLAM 初始化、短时遮挡、丢帧或
容器退出都不会接管 `odom -> base_footprint`，也不会改变当前导航和探索运动。

## 启动顺序

先结束旧导航和探索进程，再启动宿主机统一系统：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh

ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full \
  enable_stereo_ir:=true \
  start_vslam_candidate:=true
```

第二个终端进入持久化 Isaac ROS 容器：

```bash
bash ~/ros2_ws/tools/start_vslam_container.sh
```

进入容器后执行：

```bash
export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch luckrobot_vslam stereo_vslam_observe.launch.py
```

## 启动验收

第三个终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 param get /ekf_filter_node_vslam_candidate publish_tf
timeout -s INT 8s ros2 topic hz /visual_slam/tracking/odometry
timeout -s INT 8s ros2 topic hz /odometry/vslam_candidate
```

必须满足：

- `publish_tf` 为 `false`；
- 原始 cuVSLAM 约 30 Hz；
- 候选 EKF 约 50 Hz。

若任一项失败，不启动探索，保留生产链并检查容器输出。

## 300 秒全屋探索 A/B

确认上述三项后，在第三个终端执行：

```bash
bash ~/ros2_ws/tools/run_auto_diag.sh 300
```

脚本会自行启动 V10.7 探索。不要另外启动 `auto_explore.launch.py`，不要在 RViz
手动发送目标。V12.4 的诊断包额外记录以下轻量话题，不记录原始图像：

- `/visual_slam/tracking/odometry`；
- `/visual_slam/tracking/vo_pose_covariance`；
- `/odometry/vslam_candidate`；
- `/diagnostics`。

完成后提供最新的 `~/auto_diag_*.tar.gz`。下一步将检查全程输出间隙、视觉状态
协方差、与主 EKF 的轨迹差、停止后漂移以及遮挡恢复。只有多场景结果稳定优于
主 EKF，才会设计带健康门控和自动回退的生产融合，不能直接切换 Nav2。
