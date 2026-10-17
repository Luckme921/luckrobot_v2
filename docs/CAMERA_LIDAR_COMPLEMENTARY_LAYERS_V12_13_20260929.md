# LuckRobot V12.13 雷达/相机互补障碍层与路线稳定

## 问题结论

相机层当然可以作为二维雷达层的补充，而且两者应该共同参与 Nav2 规划；但它们
不能互相当成同一物理高度的清除传感器。二维雷达位于低位，可以从桌板、横梁等
悬空障碍物下方看见“空闲”；高度相机却会正确发现机器人顶部可能碰撞。如果二者
共享同一个 `ObstacleLayer`，低位清除射线、相机标记、全局清图和周期重规划共同
作用时，悬空障碍可能暂时消失，规划器便在 A/B/C 路线间反复切换。

V12.13 做了三项相互独立的修复：

1. 全局和局部代价地图分别建立 `lidar_obstacle_layer` 与
   `camera_obstacle_layer`，最终由主代价地图按最大代价合并。雷达仍负责低位、
   远距离和 360° 结构，相机负责前向、近距离、机器人高度包络内的补充障碍。
2. LuckGrid 增加路线滞回。对于同一个目标，只要当前剩余路径仍可通行，就继续
   执行这条路径；不会仅因为另一条路线暂时变短而切换。新障碍落到当前路径上时，
   当前路径立即失效并正常重新规划，因此不会牺牲动态避障。
3. 控制器执行失败的外层恢复不再清空全局代价地图。真正的规划失败仍保留原有
   一次全局清图恢复，不会完全丧失恢复能力。

相机障碍不会写进 SLAM Toolbox 的静态占据栅格，所以 RViz 中不一定变成静态地图
的黑色。这是预期设计：它会出现在局部/全局代价地图中并影响路线，但不会把人等
动态物体永久烧进地图。`/scan_camera_safety` 仍独立进入 Collision Monitor，
多帧确认和路线滞回只影响规划，不削弱即时停车。

## 部署

```bash
cd ~/ros2_ws

sha256sum luckrobot_camera_lidar_complementary_layers_v12_13_20260929.zip

unzip -o \
luckrobot_camera_lidar_complementary_layers_v12_13_20260929.zip \
-d ~/ros2_ws

sha256sum -c \
docs/CAMERA_LIDAR_COMPLEMENTARY_LAYERS_V12_13_20260929_MANIFEST.sha256

source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select luckrobot_nav2_plugins my_robot_navigation \
  --allow-overriding luckrobot_nav2_plugins my_robot_navigation

source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh
```

用干净终端重新启动：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full \
  depth_obstacle_mode:=height
```

## 启动后静态核对

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 param get \
/global_costmap/global_costmap plugins

ros2 param get \
/local_costmap/local_costmap plugins

ros2 param get \
/global_costmap/global_costmap camera_obstacle_layer.camera.topic

ros2 param get \
/global_costmap/global_costmap lidar_obstacle_layer.scan.topic

ros2 param get \
/planner_server LuckGrid.route_hysteresis_enabled

timeout -s INT 8s ros2 topic hz /scan_filtered
timeout -s INT 8s ros2 topic hz /scan_camera
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /collision_monitor
```

预期：

- 两张代价地图都列出独立的 `lidar_obstacle_layer` 与
  `camera_obstacle_layer`；
- 相机层话题为 `/scan_camera`，雷达层为 `/scan_filtered`；
- `LuckGrid.route_hysteresis_enabled` 为 `True`；
- 两路扫描都有稳定数据，三个生命周期节点均为 `active [3]`。

## 唯一需要的实车包

先启动记录，再只发送一次此前会出现 A/B/C 来回切换的长距离目标。不要混入短目标、
自动探索或人工新目标；让机器人至少经过两次原来容易切换的决策位置。发生碰撞风险
立即急停。完成或问题复现后只按一次 `Ctrl+C`，等待压缩包生成。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

bash ~/ros2_ws/tools/run_custom_nav_diag.sh 0
```

本次重点验收：

1. 首次看到 B/C 上的高位障碍后，规划切到 A；
2. 相机转离障碍后，仍保持 A，不再因 B/C 暂时更短而切回；
3. 把一个动态障碍放到当前 A 路线上时，旧路线仍能立即失效并重规划；
4. 机器人转弯时实际车体、IMU、轮速与 `/cmd_vel_safe` 一致，不再把真实底盘不响应
   和路线切换混为同一故障。

## RViz 显示说明

`/scan_filtered` 常按雷达自身 frame 显示在雷达安装高度；`/scan_camera` 是先按三维
高度筛选、再投影到 `base_footprint` 的二维 `LaserScan`，所以 RViz 中显示贴地。
贴地只是二维投影的显示形式，不代表相机 TF 或高度筛选错误。要看真实 Z 高度，应
添加相机原始 `PointCloud2`，而不是用 `/scan_camera` 判断高度。
