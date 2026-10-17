# LuckRobot VSLAM 激光交替帧动态验收 V12.7

## 1. 先给结论

本轮不把 VSLAM 接入生产 EKF，也不修改已经通过实车测试的 Nav2、自定义规划器、
自定义控制器、Collision Monitor、底盘驱动、探索算法或相机外参。

`vslam_diag_20260914_214025` 是一次有效的 **180 秒动态否决记录**。虽然静止阶段
`/vslam/health=true`，但机器人运动后原始 cuVSLAM 位姿几乎不动：

- 轮式里程计在各段累计约 0.95 m、127°、91°、0.91 m、97° 等明显运动；
- 同期原始 cuVSLAM 全程位置范围仅约 1.2 cm × 0.9 cm，Yaw 范围约 0.22°；
- 视觉位置标准差多次升到 1 m，Yaw 标准差升到约 57.3°；
- 180 个健康状态样本中仅 81 个为 true，并反复进入
  `visual_position_covariance_high`；
- 没有 NaN，时间戳正常样本的匹配延迟也正常。因此 V12.6.3 的时序修复有效，
  但相机运动图像本身不适合当前跟踪配置。

此前独立低速测试并非假成功：

- `manual_vslam_linear_20260914_103606`：轮式约 0.496 m，视觉约 0.490 m；
- `manual_vslam_turn_20260914_103658`：轮式约 43.31°，视觉约 44.11°。

两个配置的 TF、基线和外参相同。关键差异是此前专用
`vslam_observe.launch.py` 明确设置 `enable_laser=false`，而统一系统启动沿用 Orbbec
默认值，启动日志显示 G300 laser 为 ON。相机投射的红外纹理随相机一起移动，可能
成为错误的“相机固定特征”，这与本次运动时冻结、高协方差和反复恢复现象一致。

## 2. V12.7 的相机档位

新增 `vslam_camera_profile`，不改变默认行为：

- `navigation`：原生产导航档位，激光开启；
- `passive`：激光完全关闭，只用于定位根因的 A/B 排障；
- `interleaved`：Orbbec 官方 Laser On-Off 交替模式。序列 0 激光开启，只送给
  深度流；序列 1 激光关闭，只送给左右 IR。预计深度与左右 IR 各约 15 Hz。

交替模式的目标是同时保留：

1. 激光增强的深度帧，用于 `/scan_camera`、局部代价地图和近场安全；
2. 无投影纹理的同步双目帧，用于 cuVSLAM。

Gemini 330 系列固件至少需要 1.4.00 才支持该模式。正式测试前必须从启动日志确认
设备接受 `Laser On-Off`、三个 SequenceIdFilter 均生效；若设备拒绝参数，立即回到
`navigation`，不尝试融合。

官方资料：

- Orbbec interleave AE：
  https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/configuration/interleave_ae_mode.html
- Orbbec ROS 2 Gemini 330 启动参数：
  https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/launch/gemini_330_series.launch.py
- NVIDIA Isaac ROS Visual SLAM：
  https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html

## 3. 部署

先关闭全部旧导航、相机和 cuVSLAM 终端，然后执行：

```bash
cd ~/ros2_ws

sha256sum luckrobot_vslam_interleaved_camera_v12_7_20260915.zip

stamp=$(date +%Y%m%d_%H%M%S)
tar -czf ~/luckrobot_before_vslam_v12_7_${stamp}.tar.gz \
  src/my_robot_bringup/launch/test_bringup.launch.py \
  src/my_robot_bringup/launch/smart_navigation.launch.py \
  src/my_robot_bringup/launch/luckrobot_system.launch.py \
  tools/check_vslam_stereo_sync.py \
  tools/start_vslam_container.sh \
  tools/run_vslam_diag.sh \
  tools/analyze_vslam_motion_bag.py

unzip -o luckrobot_vslam_interleaved_camera_v12_7_20260915.zip -d ~/ros2_ws
sha256sum -c docs/VSLAM_INTERLEAVED_CAMERA_V12_7_20260915_MANIFEST.sha256

chmod +x tools/start_vslam_container.sh tools/run_vslam_diag.sh \
  tools/check_vslam_stereo_sync.py \
  tools/analyze_vslam_motion_bag.py

source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select my_robot_bringup \
  --allow-overriding my_robot_bringup
source ~/ros2_ws/install/setup.bash
```

## 4. 唯一允许的首轮启动方式

宿主机终端 1：

```bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh

ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full \
  use_rviz2:=true \
  enable_stereo_ir:=true \
  vslam_camera_profile:=interleaved \
  start_vslam_guard:=true \
  start_vslam_candidate:=false \
  start_vslam_candidate_ekf:=false
```

确认终端 1 包含以下含义的日志后再继续：

- interleave mode 为 `laser`，frame interleave 为 true；
- depth SequenceIdFilter ID 为 0；
- left/right IR SequenceIdFilter ID 为 1；
- 深度、左 IR、右 IR 均持续发布；
- `/scan_camera` 持续发布且 RViz 能看到相机障碍层。

宿主机终端 2：

```bash
bash ~/ros2_ws/tools/start_vslam_container.sh
```

进入容器后：

```bash
export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch luckrobot_vslam stereo_vslam_observe.launch.py
```

**每次终端 1 重启相机后，终端 2 的 cuVSLAM 也必须 Ctrl-C 后重新启动。** 不允许
让旧 tracker 跨越相机断流继续运行。

先等待 `/vslam/health` 连续为 true 至少 10 秒，再开始动态记录；否则启动恢复时间会
混入健康比例。

## 5. 动态验收

宿主机终端 3 先开始记录：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/run_vslam_diag.sh 240
```

录制开始后依次执行，不启动自动探索：

1. 约 1 m 开阔直线；
2. 约 90° 原地转向；
3. 一个带拐角、总路程 2～4 m 的 Nav2 目标；
4. 静止 20 秒。

先观察桌沿/低矮障碍物是否稳定进入 `/scan_camera`。任何碰撞、深度大面积空洞、
VSLAM 容器退出或生产导航异常都立即停止。

打包完成后在 Jetson 本机运行：

```bash
latest=$(ls -dt ~/vslam_diag_*_bag | head -1)
python3 ~/ros2_ws/tools/analyze_vslam_motion_bag.py "$latest" \
  | tee ~/vslam_dynamic_result.txt
```

首轮通过条件：

- `RESULT=PASS`；
- 左右 IR 约 15 Hz、严格同步，无时间倒退；
- 原始视觉/轮式累计路径比和累计转角比均在 0.65～1.35；
- `/vslam/health` true 比例至少 90%（包含启动恢复时间）；
- `/scan_camera` 在移动期间持续存在，桌沿/障碍物可见；
- 没有 NaN、TF 冲突或候选 EKF 节点。

首轮通过也只代表根因修复候选成立。进入生产融合前需要三天/三类光照、至少 20 次
动态路线测试，其中至少 19 次全部通过，且任何一次都不能污染生产 TF。达到该门槛后，
再单独设计 VSLAM 融合权重与自动降级；本轮禁止开启
`start_vslam_candidate_ekf:=true`。

## 6. 回退

如果交替模式不被固件支持或深度安全层变差，停止全部终端，重新启动时改为：

```bash
enable_stereo_ir:=false vslam_camera_profile:=navigation \
start_vslam_guard:=false start_vslam_candidate_ekf:=false
```

这会恢复当前已经通过实车测试的轮速 + IMU + 2D SLAM/Nav2 主链。
