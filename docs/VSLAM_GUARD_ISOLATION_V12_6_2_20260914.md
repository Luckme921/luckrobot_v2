# LuckRobot VSLAM 门控隔离 V12.6.2

## 本次故障结论

V12.6.1 已正确安装并启动门控程序，但启动约 7 秒后
`ekf_filter_node_vslam_candidate` 首先报告状态含 NaN；约 0.55 秒后生产
`ekf_filter_node` 也报告 NaN，随后 `odom -> base_footprint` 变成无效 TF，Nav2、
SLAM、RViz 和 Collision Monitor 只是重复报告同一条坏 TF。

这不是可忽略的日志，也不能通过放宽 VSLAM 阈值解决。V12.6.2 将验证阶段改为
真正的单向旁路：

- 默认只运行生产轮速/IMU EKF；
- VSLAM 健康门控可独立运行，但不发布 TF；
- 第二个候选 EKF 被单独的危险开关隔离，默认不启动；
- 旧参数 `start_vslam_candidate:=true` 仅作为门控别名，不再启动第二 EKF；
- 门控检查完整 6x6 协方差是否有限；
- 门控重新构造全有限的 Odometry，只保留 x/y/yaw 对角协方差，不复制原始
  VSLAM 消息中的未知字段。

## 故障恢复

先停止当前整栈，不要发送目标：

```bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh
```

## 部署

```bash
cd ~/ros2_ws
sha256sum luckrobot_vslam_guard_isolation_v12_6_2_20260914.zip
unzip -o luckrobot_vslam_guard_isolation_v12_6_2_20260914.zip -d ~/ros2_ws
sha256sum -c docs/VSLAM_GUARD_ISOLATION_V12_6_2_20260914_MANIFEST.sha256

chmod +x src/my_robot_bringup/scripts/vslam_odom_guard.py
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-clean-cache \
  --packages-select my_robot_bringup \
  --allow-overriding my_robot_bringup
source ~/ros2_ws/install/setup.bash
```

## 第一阶段：生产基线静止验收

不启动 VSLAM 门控和候选 EKF：

```bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh
ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full \
  use_rviz2:=true \
  enable_stereo_ir:=true \
  start_vslam_guard:=false \
  start_vslam_candidate:=false \
  start_vslam_candidate_ekf:=false
```

另一个宿主机终端让机器人静止并检查 30 秒：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/tools/check_ekf_input_health.py --duration 30
```

必须得到 `RESULT=PASS`，并且启动终端没有 `Critical Error` 或
`TF_NAN_INPUT`。否则问题在轮速/IMU输入或生产 EKF，与 VSLAM 无关；此时停止，
提供完整检查输出。

## 第二阶段：只接健康门控

第一阶段通过后重启，仍不启动候选 EKF：

```bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh
ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full \
  use_rviz2:=true \
  enable_stereo_ir:=true \
  start_vslam_guard:=true \
  start_vslam_candidate:=false \
  start_vslam_candidate_ekf:=false
```

然后在 Isaac ROS 容器中启动 cuVSLAM。宿主机确认隔离关系：

```bash
ros2 node list | grep -E 'vslam_odom_guard|ekf_filter_node_vslam_candidate'
python3 ~/ros2_ws/tools/check_ekf_input_health.py --duration 60
timeout -s INT 8s ros2 topic hz \
  /visual_slam/tracking/odometry_validated
```

预期只有 `/vslam_odom_guard`，不应出现
`/ekf_filter_node_vslam_candidate`。这一阶段先静止验收，再做低速短距离记录；
禁止设置 `start_vslam_candidate_ekf:=true`。
