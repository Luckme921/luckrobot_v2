# LuckRobot V12.13.1 相机全局代价确认与转向稳定修正

## 诊断结论

本版依据 `custom_nav_20260929_155035`（322.13 s）而不是 RViz 颜色猜测：

- `/scan_camera` 与全局主代价地图可比较的 68,814 个点中，68,802 个（100.0%，
  四舍五入）落在致命代价；
- 其中 19,527 个相机点位于 SLAM `/map` 的空闲栅格，19,515 个（99.9%）仍被
  `/global_costmap/costmap` 标为致命障碍；
- 相机层已经正确作为雷达层的补充参与全局规划。它不写入 SLAM Toolbox 的
  静态 `/map`，因此不会把动态人或短时高位障碍永久烧成黑色；
- 同一导航目标内没有发生大尺度 A/B/C 路线切换，V12.13 路线迟滞有效；
- 但 1.46 m 初始路径只有一个同向转弯，控制器却出现 6 次稳定角速度换向；
  5.06 m 初始路径的 5 个拐角也全部同向，控制器却出现 21 次换向。这是跟踪
  欠阻尼，不是全局路径反复切换。

## 修正

1. 周期复用安全路线时，不再每次把当前机器人位置作为一个新点插到缓存路线
   前方。控制器本来就会投影到最近的剩余路径点；旧连接段会在机器人越过路径
   中心线时左右变化，放大为新的急弯信号。
2. `corner_lookahead_dist` 从 0.12 m 调整为 0.16 m。用包内真实 TF、全局路径和
   实际线速度离线重算，普通跟踪阶段的稳定角速度换向从 31 次降至约 25 次；
   固定缓存路线几何后可进一步降至约 21 次。
3. 不修改相机确认阈值、障碍持久时间、机器人半径、膨胀半径、速度上限、
   Collision Monitor 或静态地图输入。

## RViz 判断方法

红/紫扫描点是 LaserScan 显示，会盖在下面的代价栅格上。要核对相机是否进入
全局规划：

1. 保持 `Global Costmap` 显示开启，话题为 `/global_costmap/costmap`；
2. 临时关闭 `Camera Planning confirmed (/scan_camera)` 显示；
3. 相机确认过的障碍应在全局代价地图中保留，但不会改变 `Map (Static)` 的黑白图。

不要把 `/scan_camera` 接到 SLAM Toolbox 来追求“变黑”，否则动态障碍和高位
短时噪声会永久污染地图。

## 部署与构建

```bash
cd ~/ros2_ws

sha256sum luckrobot_camera_lidar_complementary_layers_v12_13_1_20260929.zip

unzip -o \
luckrobot_camera_lidar_complementary_layers_v12_13_1_20260929.zip \
-d ~/ros2_ws

sha256sum -c \
docs/CAMERA_LIDAR_COMPLEMENTARY_LAYERS_V12_13_1_20260929_MANIFEST.sha256

source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select luckrobot_nav2_plugins my_robot_navigation \
  --allow-overriding luckrobot_nav2_plugins my_robot_navigation

source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh
```

干净终端重启：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full \
  depth_obstacle_mode:=height
```

## 验收

先检查生效值：

```bash
ros2 param get /controller_server LuckFollowPath.corner_lookahead_dist
ros2 param get /planner_server LuckGrid.route_hysteresis_enabled
```

预期分别为 `0.16` 和 `True`。随后一次只发送一个目标：先做原来会来回修正的
1～1.5 m 转弯目标，再做原来的长距离同向多拐角目标。录包：

```bash
bash ~/ros2_ws/tools/run_custom_nav_diag.sh 0
```

若出现碰撞风险立即急停；测试结束只按一次 `Ctrl+C` 并等待压缩完成。
