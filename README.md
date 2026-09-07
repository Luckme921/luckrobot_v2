# LuckRobot V2

LuckRobot V2 是一台基于 Jetson Orin Nano 8GB、ROS 2 Humble、A2M12 2D 激光雷达、Orbbec Gemini 336L RGB-D 相机、双轮编码器和 IMU 的室内差速机器人。

当前仓库的优先目标是先建立一个可重复、可验收、具备安全兜底的导航底座，再接入 Isaac ROS VSLAM、语义视觉和 LLM。不要在基础导航尚未量化稳定前把 VSLAM、YOLO 和语音同时接入主 TF 链。

## 当前架构

```text
/wheel/odom --(Vx、当前配置也融合 Vyaw)--+
                                             +--> robot_localization --> odom -> base_footprint
/imu/data -------------------------(Vyaw)----+

A2M12 /scan --> BoxFilter --> /scan_filtered --> slam_toolbox + Nav2 + Collision Monitor
Gemini depth --> depthimage_to_laserscan --> SpeckleFilter --> /scan_camera --> Nav2 + Collision Monitor

slam_toolbox --> map -> odom
Nav2 Controller /cmd_vel_nav --> velocity_smoother --> /cmd_vel --> Collision Monitor --> /cmd_vel_safe --> chassis_driver
Nav2 Humble Behavior Server (Spin/BackUp 等) --> /cmd_vel --> Collision Monitor --> /cmd_vel_safe --> chassis_driver
```

Humble 官方 `navigation_launch.py` 不会把 `behavior_server` 重映射到
`/cmd_vel_nav`。恢复行为使用自身的速度/加速度约束并直接进入碰撞监控；
不要把 `/cmd_vel_nav` 在 Spin 测试期间没有数据误判为丢消息。

注意：交接说明声称 EKF 的轮式输入只使用 `Vx`，但仓库当前的 `ekf.yaml` 实际同时启用了轮式 `Vyaw`。这是需要用实车 bag 数据做 A/B 验证的物理参数，本轮没有擅自改变。

## Jetson 构建

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

首次使用近场安全层时确认已安装：

```bash
sudo apt update
sudo apt install ros-humble-nav2-collision-monitor
```

## 推荐启动方式

新建或继续开荒地图：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py mode:=mapping
```

另开终端启动前沿探索：

```bash
ros2 launch my_robot_mapping auto_explore.launch.py
```

当前 V10 探索器不会把灰色未知区的前沿质心直接发给 Nav2。它先对每条
前沿后方的 UNKNOWN 栅格做 1.5 m 有界连通洪泛，用真实待探索面积识别
大房间/宽通道并过滤小于 0.20 m² 的碎角；随后在 `/map` 白色已知区建立
拓扑 DFS，沿选中的 UNKNOWN basin 以最长 `0.85 m` 的安全局部目标推进。
侧分支的白区父分叉进入 LIFO 栈，当前 basin 确认结束后才切换分支。切换时
由 Nav2 从当前位置走最短安全路线，不再机械重放旧分叉坐标。每个
实际目标仍通过 Nav2 全局路径、实时代价和车体余量检查，灰区只提供信息，
绝不是运动终点。

V10.3 根据 `auto_diag_20260831_221003` 修复了一个明确的软膨胀自锁：
目标本身代价为 9，但动态路径跟踪中止后机器人停在代价 67，旧逻辑会把
所有大于 40 的软代价格都当成碰撞并原地等待。现在 41～98 只允许一个
最长 0.45 m、终点代价不高于 20 的规划脱困步；99/100 仍硬停止。窄通道
实车 A/B 采用全局/局部 0.30/0.25 m 的软膨胀半径，车体 0.175 m 和
Collision Monitor 0.19 m 均未缩小。详细证据见
[V10.3 软膨胀脱困说明](docs/SOFT_INFLATION_ESCAPE_V10_3.md)。

V10.6 根据 2026-09-07 新补充的“中间出发”和“一角出发”两次 300 秒录包，
只把可达白色观察点前方、且没有被墙/障碍遮挡的 UNKNOWN 栅格计为探索收益，
过滤封闭墙外灰区和已经完成分叉的低价值残片。目标几何余量提高到 `0.25 m`，
软代价脱困步缩短到 `0.30 m`；机器人若在代价 `>=99` 停留 15 秒则安全暂停，
不再出现探索节点运行但 180 秒不发目标。实测起步残余航向误差约 27～33°，
因此本版受控地把 RPP 对准阈值收紧为 `0.22 rad`，但不改速度、膨胀层、底盘
和 Collision Monitor。详见
[V10.6 可观测未知区与硬代价安全暂停](docs/OBSERVABLE_UNKNOWN_DFS_V10_6.md)。

V10.5 根据 2026-09-07 的“中间出发”和“一角出发”两次 300 秒录包，
修正同一走廊沿途反复写入伪分叉的问题。活动分支至少前进 `1.20 m` 才允许
新增一层宏观分叉；恢复记忆分支时，当前白区路径必须经过旧锚点并沿原出口
方向离开。无进展区域按 `0.85 m / 60 s` 冷却，普通规划安全段由 `0.85 m`
增至 `1.20 m`，减少完整减速/再启动次数；Nav2、底盘和安全链参数保持稳定
基线不变。详见
[V10.5 宏观分叉约束与长步连续探索](docs/MACRO_JUNCTION_DFS_V10_5.md)。

现有两次录包还完成了传感器时间与 TF 审计：TF 树唯一且连通，所有消息头单调；
相机/雷达扫描最近时间差中位约 17 ms，当前低速 2D 导航可用，但物理外参仍需在
最终机构上实测。VSLAM 相机建议做成光心 0.35～0.45 m 可调、首选 0.40 m、
下俯 5～8°。升高后必须重做外参，并把中央 10 行深度伪雷达升级为点云高度带避障，
不能只改 URDF。详见
[传感器对齐现状与 VSLAM 相机安装建议](docs/SENSOR_ALIGNMENT_AND_VSLAM_MOUNTING.md)。

自定义 Nav2 插件 V11.0 的两次实车包证明直线跟踪横向误差较小，但发现控制器
在 0.12 m 提前停止平移、Goal Checker 要求进入 0.08 m 的确定性死锁。V11.1
改为运行时读取所选 Goal Checker 的实际容差，统一终点阈值，并移除与
`velocity_smoother` 串联的第二重角加速度限幅；本轮仍只测试
`SmacPlanner2D + LuckController`。详见
[V11.1 自定义控制器终点修复](docs/CUSTOM_NAV_CONTROLLER_FIX_V11_1.md)。

V10.4 根据两次 2026-09-06 实车录包补上完成收敛：小于 `0.35 m²` 的
残余灰区不再生成目标，小于 `0.60 m²` 的碎片不写入 DFS 分支栈；完成区
只有新增至少 `0.75 m²` 未知面积才重新打开。分支末端确认由 6 秒缩短为
2.5 秒，所有有效分支清空后连续 15 秒无候选即发布探索完成。该版本没有
修改底盘、Nav2 控制器、膨胀层或 Collision Monitor 的稳定基线，详见
[V10.4 分支收敛与录包诊断](docs/EXPLORATION_COMPLETION_V10_4.md)。

RViz 中蓝/红点是前沿边界，绿色圆点是当前实际目标，黄色点是记住的白区
父分叉，紫色点是对应的待探索子分支方向。日志中的 `V10.4 precheck` 会打印
UNKNOWN gain，`V10.4 goal locked`
会打印步长、全局代价、分支编号和 DFS 栈深度。探索行为树使用独立的
路过点 goal checker，不再为每个 0.85 m 中间点精修最终 yaw；手动 RViz
导航仍保持 8 cm / 0.10 rad 精度。

Humble 官方默认行为树在 `FollowPath` 中不填写 goal checker 名称；同时
加载手动和探索两个 checker 后，这会使 RViz 手动导航直接失败并误触发
Spin/Wait/BackUp。当前默认单点/多点导航树已显式绑定
`general_goal_checker`，探索专用树显式绑定 `explore_goal_checker`，两条链
互不串用。修复依据见
[V10.1 Goal Checker 路由修复](docs/GOAL_CHECKER_ROUTING_FIX_V10_1.md)。

V9 的未知区分支设计见
[V9 UNKNOWN-basin DFS 探索设计](docs/UNKNOWN_BASIN_DFS_EXPLORATION_V9.md)，
实车诊断后的路径执行升级见
[V10 直接转场探索设计](docs/DIRECT_TRANSIT_EXPLORATION_V10.md)。

保存栅格地图和可继续使用的 posegraph（路径不要带后缀）：

```bash
mkdir -p ~/ros2_ws/maps
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/nvidia/ros2_ws/maps/my_room_map'}"
ros2 run nav2_map_server map_saver_cli -f /home/nvidia/ros2_ws/maps/my_room_map
```

日常导航使用 `slam_toolbox` posegraph 定位，不启动 AMCL：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=localization \
  posegraph:=/home/nvidia/ros2_ws/maps/my_room_map
```

需要把家具布局等长期变化永久并入旧地图时，使用 `lifelong`，结束后再次调用 `serialize_map`：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=lifelong \
  posegraph:=/home/nvidia/ros2_ws/maps/my_room_map
```

`localization` 的近期扫描缓存只用于定位，底层 posegraph 不会被永久修改；日常动态障碍由 Nav2 局部代价地图处理。不要为了一个临时纸箱频繁永久改图。

上述两个加载旧图的命令默认机器人从充电桩/posegraph 首节点启动。若机器人不在该位置，加上 `start_at_dock:=false`，然后在 RViz 中给出初始位姿。

若首次验证 Collision Monitor 时出现误停，可临时回退进行对比（只用于诊断）：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping use_collision_monitor:=false
```

旧的 `robot_bringup.launch.py` 和 `test_bringup.launch.py` 均保留；前者仍假设外部已经有 `map -> odom` 提供者。推荐日常只使用 `smart_navigation.launch.py`，避免重复启动 SLAM 或 AMCL。

## 上车前必须核对 DDS

当前 Jetson 的 `.bashrc` 实际是：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/ros2_ws/fastdds_shm.xml
```

这与交接说明中的“强制 CycloneDDS”不一致，而且仓库的 FastDDS XML 没有绑定 Wi-Fi 网卡。先记录实际基线，不要在同一次导航测试中顺便切 DDS：

```bash
echo "$RMW_IMPLEMENTATION"
echo "$FASTRTPS_DEFAULT_PROFILES_FILE"
echo "$CYCLONEDDS_URI"
ros2 doctor --report
```

DDS 切换要作为独立实验进行，并保证 Jetson、远程 RViz/控制端的 `ROS_DOMAIN_ID=100` 和 RMW 实现一致。

## Isaac ROS VSLAM 的版本边界

当前宿主是 ROS 2 Humble、JetPack 6.x、CUDA 12.6，不能直接照搬 Isaac ROS 最新版文档。最新版已经面向 ROS 2 Jazzy / JetPack 7.2；本机应固定在 [Isaac ROS release-3.2](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/index.html)，该版本的测试矩阵才是 ROS 2 Humble + JetPack 6.1/6.2。

Gemini 336L 是 95 mm 基线的双目机型，但不带 IMU。release-3.2 的 cuVSLAM 应先按双目模式验证左右 IR 图像、标定和硬件时间戳；官方要求目标图像频率 30 Hz、双目时间偏差在 100 微秒量级，而当前导航相机配置是 15 Hz，因此不能直接把现有 `/camera/depth/*` 流接进去。后续 VSLAM 应作为“里程计源”接入 EKF，并关闭它对全局 `map -> odom` 的竞争，继续由 `slam_toolbox` 负责 2D 全局地图。

## 验收

详细步骤见 [导航闭环验收清单](docs/NAVIGATION_ACCEPTANCE_TEST.md)。第一轮只验证：单一 TF 发布者、传感器频率、静止漂移、直行/原地旋转、近场制动、单点导航、动态绕障、恢复行为和自动探索结束。所有项目通过后再进入 Isaac ROS VSLAM。
