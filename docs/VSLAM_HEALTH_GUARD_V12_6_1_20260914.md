# LuckRobot VSLAM 健康门控 V12.6.1

本补丁修复 V12.6 在 Jetson 上构建成功、但 ROS 2 找不到
`vslam_odom_guard.py` 的安装入口问题。根因在可执行文件安装名与 launch
入口不够稳健；V12.6 的门控算法和阈值没有改变。

## 安全边界

- 当前仍是影子接入：Nav2 继续使用 `/odometry/filtered`。
- 健康门控只发布 `/visual_slam/tracking/odometry_validated`。
- 候选 EKF 只发布 `/odometry/vslam_candidate`，并保持 `publish_tf: false`。
- 在下面的可执行文件检查通过前，不启动导航，也不让机器人运动。

## 部署

```bash
cd ~/ros2_ws
sha256sum luckrobot_vslam_health_guard_v12_6_1_20260914.zip
unzip -o luckrobot_vslam_health_guard_v12_6_1_20260914.zip -d ~/ros2_ws
sha256sum -c docs/VSLAM_HEALTH_GUARD_V12_6_1_20260914_MANIFEST.sha256

chmod +x src/my_robot_bringup/scripts/vslam_odom_guard.py
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-clean-cache \
  --packages-select my_robot_bringup \
  --allow-overriding my_robot_bringup
source ~/ros2_ws/install/setup.bash
```

## 必须先通过的静态检查

```bash
ros2 pkg executables my_robot_bringup | grep -F vslam_odom_guard
test -x ~/ros2_ws/install/my_robot_bringup/lib/my_robot_bringup/vslam_odom_guard \
  && echo "VSLAM guard executable: OK"
python3 -m py_compile \
  ~/ros2_ws/src/my_robot_bringup/scripts/vslam_odom_guard.py \
  ~/ros2_ws/src/my_robot_bringup/launch/luckrobot_system.launch.py \
  ~/ros2_ws/src/my_robot_bringup/launch/vslam_candidate_ekf.launch.py
```

预期第一条列出：

```text
my_robot_bringup vslam_odom_guard
```

若没有这行，不继续启动，把三条检查的完整输出返回。

## 影子模式启动与验收

静态检查成功后，宿主机启动完整系统；不会自动启动探索：

```bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh
ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full \
  use_rviz2:=true \
  enable_stereo_ir:=true \
  start_vslam_candidate:=true
```

按既有方式在 Isaac ROS 容器内启动 cuVSLAM。另一个宿主机终端检查：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 node list | grep -E 'vslam_odom_guard|ekf_filter_node_vslam_candidate'
timeout -s INT 8s ros2 topic hz \
  /visual_slam/tracking/odometry_validated
ros2 param get /ekf_filter_node_vslam_candidate publish_tf
```

只有健康输出连续、候选 EKF 不发布 TF，并通过长时动态测试后，才讨论让
VSLAM 参与生产定位。健康失败时继续使用轮速和 IMU，不放宽门控阈值。
