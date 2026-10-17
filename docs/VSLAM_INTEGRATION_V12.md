# LuckRobot V2：VSLAM 分阶段接入方案（V12）

## 1. 本阶段只做导航与 VSLAM

Jetson Orin Nano Super 8GB 本阶段不部署 LocateAnything、YOLO 或其他大视觉模型。
本轮只建立可独立验收、可随时回退的视觉里程计链：

```text
Gemini 336L 左/右 IR ──> cuVSLAM ──> 视觉里程计 ──┐
轮速编码器 ────────────────────────────────────────┼─> EKF ─> odom->base_footprint
UART IMU ──────────────────────────────────────────┘

A2M12 + 深度障碍物 ─> slam_toolbox ─> map->odom + 2D 几何地图
```

因此本项目只使用 **Isaac ROS cuVSLAM** 做视觉里程计。语义地图和对话地点导航属于
后续独立里程碑，不在本次 Jetson 部署与测试范围内。

## 2. 本轮不可破坏的边界

1. `slam_toolbox` 继续独占 `map -> odom`。
2. 现有 `robot_localization` 继续独占 `odom -> base_footprint`。
3. cuVSLAM 必须设置：
   `publish_map_to_odom_tf:=false`、`publish_odom_to_base_tf:=false`。
4. cuVSLAM 第一阶段只发布 `/visual_slam/tracking/odometry`，不接 Nav2。
5. VSLAM 候选 EKF 发布 `/odometry/vslam_candidate` 且 `publish_tf: false`，只用于 A/B
   对比。
6. 只有旁路验收通过，才允许把视觉里程计接入主 EKF；出现跟踪丢失、跳变、延迟或
   相机断流时，主链仍能依靠轮速和 IMU 工作。

这不能保证任何环境中“绝对不失败”，但能保证单一视觉故障不会直接破坏底盘控制，
并把失败变成可检测、可降级、可恢复的状态。

## 3. 相机安装高度

当前深度光心约为 13.85 cm。夹具使整机相机向上平移约 23 cm 后，预计光心为：

```text
13.85 cm + 23 cm = 36.85 cm
```

**36.9 cm 可以使用**，位于建议的 35～45 cm 区间内。与理论最优 40 cm 相比只低约
3.2 cm，不值得为了这点差异牺牲支架刚度。最终应以左右 IR 镜头光心离地高度实测，
而不是以连接板或外壳底边为准。

安装要求：

- 光轴水平，建议 pitch 误差不超过 ±1°；
- 支架刚性优先，手推相机外壳不应出现肉眼可见摆动；
- 相机与底盘装好后再更新 URDF 和复核外参；
- 若机构确实只是保持原姿态整体抬高 0.23 m，
  `camera_color_screw_joint` 的 Z 初值可由 `0.13933` 改为 `0.36933` m；这只是 CAD
  初值，不能替代装配后的实测；
- 相机抬高后，当前深度图中央 10 行不能可靠覆盖低矮近场障碍。低位 A2M12 暂时继续
  承担安全底线，后续把深度避障升级为“点云高度带 + 地面剔除”，不要简单扩大
  `scan_height`。

## 4. 阶段 A：确认 Jetson 软件边界

先不要启动 Nav2。执行：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

echo "===== Jetson / JetPack ====="
cat /etc/nv_tegra_release
dpkg-query -W nvidia-jetpack 2>/dev/null || true
uname -m

echo "===== ROS / CUDA ====="
echo "$ROS_DISTRO"
nvcc --version 2>/dev/null || true

echo "===== storage / container runtime ====="
df -h /
df -h /mnt/nvme 2>/dev/null || true
docker --version 2>/dev/null || true
dpkg -l | grep -E 'nvidia-container|docker-buildx' || true

echo "===== Orbbec ====="
ros2 run orbbec_camera list_devices_node
```

本项目固定 ROS 2 Humble / JetPack 6.x 时，应使用 **Isaac ROS release-3.2**，不要把
面向 Jazzy / 新 JetPack 的最新分支混入当前工作空间。官方建议 Jetson 使用 NVMe/SSD，
为开发容器和构建预留至少 30 GB。若这里显示的不是 JetPack 6.1/6.2、aarch64，或磁盘
空间不足，先停在本阶段，不安装 VSLAM。

## 5. 阶段 B：建立官方 Isaac ROS release-3.2 容器环境

如果宿主机目前只安装了 ROS 2 Humble，**不要直接执行宿主机**
`apt install ros-humble-isaac-ros-visual-slam`，也不要向现有 `~/ros2_ws` 混入最新 Isaac
ROS 分支。NVIDIA 的 quickstart、教程和示例均以 Isaac ROS Dev Docker 为运行环境；
按 release-3.2 建立独立工作区：

```bash
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

cd ~/isaac_ros_ws
./src/isaac_ros_common/scripts/run_dev.sh -d ~/isaac_ros_ws
```

进入开发容器后，再按 release-3.2 官方 Debian quickstart 安装 VSLAM 包：

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-visual-slam
source /opt/ros/humble/setup.bash
ros2 pkg prefix isaac_ros_visual_slam
```

注意，官方开发容器退出后会被删除，不能把一次交互式 `apt install` 当作持久部署。
LuckRobot 使用 `docker/isaac_ros/Dockerfile.luckrobot_vslam` 固化软件层，并执行：

```bash
bash ~/ros2_ws/tools/install_vslam_container.sh
```

宿主相机与容器 VSLAM 通信时必须保持相同 `ROS_DOMAIN_ID` 和兼容的 RMW/网络配置。
禁止在容器里再次启动 `chassis_driver`、IMU、Nav2 或第二个相机驱动。V12 当前的
`vslam_observe.launch.py` 用于宿主机相机预检；容器侧使用独立的
`luckrobot_vslam/stereo_vslam_observe.launch.py`。该包已经在 2026-09-09 完成构建和
静态启动验证，不能在宿主缺少 Isaac ROS 包时强行 `start_vslam:=true`。

## 6. 阶段 C：只验收 Gemini 336L 双目输入

Gemini 336L 无内置 IMU，本轮 cuVSLAM 使用纯双目模式。先关闭终端 1 和所有相机
进程，不让机器人运动，然后运行：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch my_robot_bringup vslam_observe.launch.py \
  start_camera:=true start_vslam:=false
```

另开终端：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 tools/check_vslam_stereo_sync.py --duration 20
```

该测试模式关闭主动投射器，开启左右 IR 640×400@30 Hz 和硬件帧同步，不启动深度、
彩色、底盘或 Nav2。验收条件：

- 左右均为约 30 Hz，不能只有 15 Hz；
- 分辨率一致，编码为单通道图像；
- 时间戳不倒退，帧周期抖动 P95 不大于 2 ms，左右时间差 P95 不大于 0.1 ms；
- 两个 `camera_info` 的内参有效，右目投影矩阵能给出合理的非零双目基线；
- TF 中存在 `base_footprint -> ... -> camera_left_ir_optical_frame` 和右目光学帧。

若相机不支持该组合，先保留完整终端输出，不要随意改分辨率。用下列命令收集设备与
实际流配置：

```bash
ros2 topic list | grep -E '/camera/(left_ir|right_ir)'
ros2 param dump /camera/camera > ~/gemini336l_vslam_params.yaml
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo '{}'
```

## 7. 阶段 D：启动纯双目 cuVSLAM（仍不接导航）

确认阶段 C 合格且容器内 Isaac ROS 已安装后，宿主终端只启动相机与机器人描述：

```bash
ros2 launch my_robot_bringup vslam_observe.launch.py \
  start_camera:=true start_robot_description:=true start_vslam:=false
```

另一个宿主终端进入 Isaac ROS 容器：

```bash
bash ~/ros2_ws/tools/start_vslam_container.sh
```

容器内启动纯双目观察节点：

```bash
export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch luckrobot_vslam stereo_vslam_observe.launch.py
```

第二终端检查：

```bash
ros2 topic hz --window 100 /visual_slam/tracking/odometry
ros2 topic echo /visual_slam/status --once
ros2 topic echo /visual_slam/tracking/odometry --once
ros2 topic info /tf -v
```

观察启动文件使用 `base_footprint` 作为被估计刚体，使用 `odom` 作为消息坐标系，但
关闭 cuVSLAM 的两个 TF 广播器。因此，即使算法跳变，也不会改动当前 TF 树。

完成“静止 10 秒、人工缓慢直移约 1 m、缓慢原地转 90°、回到起点”后运行轻量记录：

```bash
bash ~/ros2_ws/tools/run_vslam_diag.sh 90
```

上传生成的 `~/vslam_diag_*.tar.gz`。该包不录原始图像，体积较小；双目同步统计由脚本
单独写入状态文件。

通过标准：

- 静止时视觉位姿不持续漂移或随机跳变；
- 平移方向与 `/wheel/odom` 一致；
- 左转时两者 yaw 符号一致；
- VSLAM 里程与轮式里程在短距离内没有明显比例错误；
- `visual_slam/status` 不频繁报告 tracking lost；
- VSLAM 退出后现有 TF、EKF 和 Nav2 完全不受影响。

## 8. 阶段 E：旁路 EKF A/B（不控制机器人）

只有阶段 D 通过后，在完整底盘系统已经运行、VSLAM 里程计也稳定时，第三终端启动：

```bash
ros2 launch my_robot_bringup vslam_candidate_ekf.launch.py
```

它把轮速、IMU 和视觉位姿融合为 `/odometry/vslam_candidate`，但不发布 TF，Nav2 仍然
使用原来的 `/odometry/filtered`。对比：

```bash
ros2 topic hz /odometry/filtered
ros2 topic hz /odometry/vslam_candidate
ros2 run tf2_ros tf2_echo odom base_footprint
```

候选配置将视觉 x/y/yaw 设为相对起点的绝对位姿，并设置马氏距离拒绝阈值。视觉速度
不再重复融合，以避免同一视觉信息作为 pose 和 twist 被重复计算。阈值只能依据实测
协方差和跳变数据调节，不能直接靠主观增大。

## 9. 阶段 F：正式融合与故障降级

阶段 E 至少完成以下样本后，才修改主 EKF：开阔直线 5 次、原地旋转 5 次、窄通道
3 次、纹理少的白墙区域 3 次、暗光 3 次、相机短时遮挡 3 次。

正式融合时需增加一个 VSLAM 健康门控节点：

- 输入：`visual_slam/status`、输出频率、协方差、连续位姿增量；
- 正常：转发到主 EKF；
- tracking lost、频率过低、时间戳异常、单帧跳变：停止转发视觉数据；
- 主 EKF 的 `sensor_timeout` 到期后自动只使用轮速 + IMU；
- 连续稳定一段时间后再恢复转发，恢复时使用相对坐标，避免把历史偏移硬注入主 EKF；
- 健康切换只影响里程计观测，不允许接管 `map -> odom` 或直接发布速度。

这一步完成后，再把 Nav2 的 `odom_topic` 从基线输出切换到经过验收的融合输出，并保留
一个启动参数用于立即回退到 wheel+IMU。

## 参考资料

- NVIDIA Isaac ROS Visual SLAM release-3.2：
  https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/index.html
- NVIDIA Isaac ROS Compute Setup release-3.2：
  https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/compute/index.html
- NVIDIA Isaac ROS Common / Dev Docker release-3.2：
  https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_common/index.html
- NVIDIA RGB-D / cuVSLAM 相机要求：
  https://nvidia-isaac-ros.github.io/v/release-3.2/reference_workflows/isaac_perceptor/run_perceptor_on_rgbd.html
- Orbbec ROS 2 launch 参数：
  https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/launch_parameters.html
- Orbbec Gemini 330 interleave 模式：
  https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/configuration/interleave_ae_mode.html
