# LuckRobot VSLAM 双目单调门控 V12.7.1

## 1. 本轮结论

不要使用 V12.7 当前运行继续动态测试。两次 20 秒预检都稳定复现同一个错误：

- 左右 IR 各收到约 575～579 条消息，表面速率约 29.97 Hz；
- 左右各有约 287～289 次非单调时间戳，恰好接近总数的一半；
- 左右相机同一帧间仍严格同步，基线仍为 0.09532 m；
- `/scan_camera` 约 29 Hz，说明深度避障通路仍在运行；
- cuVSLAM 没有形成健康输出，`/vslam/health` 30/30 均为 false。

这说明 Orbbec Laser On-Off 交替配置与 SequenceIdFilter 已被驱动接受，但过滤掉的
交替帧仍以重复/回退的旧时间戳发布。cuVSLAM 实际只看到约 15 Hz 的新帧，因此输出
连续的约 66.7 ms 间隔警告，并且无法通过健康门控。

V12.7.1 新增主机侧 `vslam_stereo_monotonic_gate`：左右图像必须在 0.1 ms 内同步，
且两边时间戳都严格大于上一已发布帧才转发。相机原始话题、深度图、`/scan_camera`、
Collision Monitor、生产 EKF、Nav2 和 TF 所有权均不改变。

## 2. 部署

先关闭当前容器、导航和相机终端，然后执行：

```bash
cd ~/ros2_ws

sha256sum luckrobot_vslam_stereo_monotonic_gate_v12_7_1_20260915.zip

stamp=$(date +%Y%m%d_%H%M%S)
tar -czf ~/luckrobot_before_vslam_v12_7_1_${stamp}.tar.gz \
  src/my_robot_bringup/CMakeLists.txt \
  src/my_robot_bringup/package.xml \
  src/my_robot_bringup/launch/test_bringup.launch.py \
  src/luckrobot_vslam/launch/stereo_vslam_observe.launch.py \
  tools/check_vslam_stereo_sync.py \
  tools/run_vslam_diag.sh

unzip -o \
  luckrobot_vslam_stereo_monotonic_gate_v12_7_1_20260915.zip \
  -d ~/ros2_ws

sha256sum -c \
  docs/VSLAM_STEREO_MONOTONIC_GATE_V12_7_1_20260915_MANIFEST.sha256

chmod +x \
  src/my_robot_bringup/scripts/vslam_stereo_monotonic_gate.py \
  tools/check_vslam_stereo_sync.py \
  tools/run_vslam_diag.sh

source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select my_robot_bringup \
  --allow-overriding my_robot_bringup
source ~/ros2_ws/install/setup.bash
```

把更新后的容器 Launch 同步到 Isaac ROS 工作区：

```bash
mkdir -p ~/isaac_ros_ws/src/luckrobot_vslam/launch

cp \
  ~/ros2_ws/src/luckrobot_vslam/launch/stereo_vslam_observe.launch.py \
  ~/isaac_ros_ws/src/luckrobot_vslam/launch/stereo_vslam_observe.launch.py
```

## 3. 启动和静态准入

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

终端 1 不得出现 `TF_NAN_INPUT`、`Critical Error` 或节点退出。另开宿主机终端先检查
门控输出，此时不需要启动 cuVSLAM：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

python3 ~/ros2_ws/tools/check_vslam_stereo_sync.py \
  --duration 20 \
  --expected-rate 15
```

必须看到约 15 Hz、左右 `nonmonotonic=0`、`RESULT=PASS`。若仍失败，不启动容器，
直接提供输出。

宿主机终端 2：

```bash
bash ~/ros2_ws/tools/start_vslam_container.sh
```

进入容器后先构建同步过来的 Launch，再启动并保持终端运行：

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select luckrobot_vslam
source /workspaces/isaac_ros-dev/install/setup.bash

export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch luckrobot_vslam stereo_vslam_observe.launch.py
```

该 Launch 默认订阅 `/vslam/stereo/...`，并将 cuVSLAM 图像间隔警告阈值调整为
80 ms，以匹配约 66.7 ms 的 15 Hz 输入；这只修正告警阈值，不放宽健康门控。

宿主机终端 3 再执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

python3 ~/ros2_ws/tools/check_ekf_input_health.py \
  --duration 30 \
  --require-vslam

timeout -s INT 8s ros2 topic hz /scan_camera
ros2 node list | grep ekf_filter_node_vslam_candidate
```

只有健康检查 `RESULT=PASS`、`/vslam/health` 全程 true、`/scan_camera` 持续存在、
最后一条命令无输出，才进入动态测试。

## 4. 动态测试

```bash
bash ~/ros2_ws/tools/run_vslam_diag.sh 240
```

录制开始后依次完成约 1 m 直线、约 90° 原地旋转、2～4 m 带拐角路线和 20 秒
静止。不得启动自动探索，不得开启候选 EKF。完成后运行：

```bash
latest=$(ls -dt ~/vslam_diag_*_bag | head -1)
python3 ~/ros2_ws/tools/analyze_vslam_motion_bag.py "$latest" \
  | tee ~/vslam_dynamic_result.txt
```

上传诊断压缩包和 `~/vslam_dynamic_result.txt`。只有动态分析 `RESULT=PASS` 才继续
多场景 19/20 验收；本轮仍不把 VSLAM 融入生产 `/odometry/filtered`。

