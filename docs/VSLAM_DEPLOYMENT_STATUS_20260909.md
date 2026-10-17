# Isaac ROS cuVSLAM 部署状态（2026-09-09）

## 已完成

Jetson Orin Nano Super 8GB 已完成以下旁路部署，期间没有启动底盘或 Nav2：

- 宿主系统：Ubuntu 22.04 / L4T R36.4.3 / CUDA 12.6 / ROS 2 Humble；
- Docker CE 29.8.0、containerd、Buildx、Compose 已安装；
- NVIDIA Container Toolkit 1.16.2 已安装并配置；
- 已生成 `/etc/cdi/nvidia.yaml`，容器可见 `nvidia.com/gpu=0`；
- `~/isaac_ros_ws/src/isaac_ros_common` 固定在 `release-3.2`，提交 `fcf4d9e`；
- 官方 aarch64 Isaac ROS 开发镜像已拉取；
- 专用持久化镜像内安装 `ros-humble-isaac-ros-visual-slam` 3.2.6；
- `luckrobot_vslam` 独立包已在 `/workspaces/isaac_ros-dev` 构建；
- `VisualSlamNode` 组件成功加载，报告 `cuVSLAM version: 12.6`，GPU 预热成功；
- 真实左右 IR 输入下，视觉里程计稳定在约 29.96 Hz；
- `/visual_slam/status` 的 `vo_state=1`（接口定义为 Success），单帧跟踪约
  2.4 ms；
- 静止间隔 10 秒读取的两次位姿均保持零，日志未出现跟踪错误；
- 测试结束后相机、VSLAM、Nav2 和探索节点均已关闭。

此前 Gemini 336L 双目预检也已通过：左右 IR 为 640×400 mono8、约 29.964 Hz，
时间戳单调，左右时间差 P95 为 0，右目投影矩阵给出的基线约 0.09532 m。

这些结果表示“软件、GPU、组件、双目输入和静止跟踪链已经打通”，尚不表示视觉尺度、
方向、旋转和丢失恢复已经通过实机运动验收。

## 设计边界

第一阶段为纯双目观察模式：

- 不融合 IMU（Gemini 336L 没有内置 IMU）；
- 不发布 `map -> odom`；
- 不发布 `odom -> base_footprint`；
- 不接入主 EKF，也不控制 Nav2；
- `slam_toolbox` 和现有 `robot_localization` 继续拥有主 TF 链。

因此第一次 VSLAM 测试失败或退出，不会改变当前已经稳定的底盘导航。

## 下一次物理测试

先关闭自动探索，架好相机并保持机器人静止。宿主终端 1 只启动机器人描述和双目相机：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_bringup vslam_observe.launch.py \
  start_camera:=true start_robot_description:=true start_vslam:=false
```

宿主终端 2 进入已经部署好的持久化容器（不要直接调用裸 `run_dev.sh`，否则会使用
不含 cuVSLAM 的官方基础层）：

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

宿主终端 3 先检查输出，再录制旁路数据：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic hz --window 100 /visual_slam/tracking/odometry
timeout -s INT 8s ros2 topic echo /visual_slam/status --once
bash ~/ros2_ws/tools/run_vslam_diag.sh 90
```

`/visual_slam/status` 是 `VOLATILE` 状态话题，宿主机也没有安装容器内的
Isaac 消息接口。因此不得使用无超时的 `--once` 作为持续健康判据；
`/visual_slam/tracking/odometry` 持续约 30 Hz 才是主要存活指标。

需要记录轮速、IMU 与 VSLAM 的动态对照时，重启宿主观察启动文件并显式
打开运动参考模式：

```bash
ros2 launch my_robot_bringup vslam_observe.launch.py \
  start_camera:=true \
  start_robot_description:=true \
  start_vslam:=false \
  start_motion_reference:=true \
  motion_cmd_vel_topic:=/cmd_vel_vslam_test
```

该模式只启动底盘、IMU 和原有轮速/IMU EKF，不启动 Nav2、自动探索或
碰撞监控。只允许在空旷区域使用带自动停车的有限时长低速命令。

测试顺序为：静止 10 秒、人工缓慢直移约 1 m、缓慢原地左转约 90°、回到起点。
第一轮不要启动 Nav2 或自动探索。上传生成的 `~/vslam_diag_*.tar.gz` 后，再决定是否进入
旁路 EKF A/B；在此之前绝不修改主 EKF。

## 已知非阻断事项

容器执行 `apt-get update` 时 Yarn 第三方源报告缺少公钥
`62D54FD4003F6525`。Isaac ROS、ROS 2、Ubuntu 和 NVIDIA 源均可正常使用，cuVSLAM 已
安装成功；该警告与 VSLAM 无关，本轮不为修复无关源而改动系统。

官方开发镜像及其依赖较大。部署后根分区约 116 GB 中剩余约 28 GB，Docker 镜像合计
约 40.53 GB。后续不要再拉取其他 Isaac ROS/JetPack 版本；及时转移大型 rosbag，
并在空间低于 15 GB 时暂停录制和构建。
