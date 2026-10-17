# LuckRobot V12.14 连续弧线跟踪与 SLAM 错误匹配收紧

## 结论来自两份实车包

本版分析了：

- `custom_nav_20260929_210835`（约 292 s）；
- `custom_nav_20260929_212702`（约 410 s）。

结论不是单一的“控制器参数不顺滑”：

1. 第一包中 `map -> odom` 连续，最大单步修正约 2.9 cm / 1.8°。
   Collision Monitor 只把 3 个非零导航控制样本压成零；多数停顿来自 action、
   costmap clear 服务超时和控制器切入原地转向，不是前方人员安全停车。
2. 第二包在记录约 344～355 s 连续出现 23.6、25.9、26.3、27.8 cm 的
   `map -> odom` 单步修正，30 s 内累计约 1.16 m。这与截图中整批点云/代价图
   横向平移、出现多组重影一致，属于 SLAM 匹配失稳，不是单纯 RViz 显示残影。
3. 故障区间 345～360 s 中，轮速/命令约为 1.59 m，而 `map` 坐标中的机器人
   只前进约 0.7 m。包内无法仅凭软件数据百分之百区分“相似结构造成错误匹配”
   与“轮端打滑/编码器前进但车体少动”，因此本版不盲改编码器比例。
4. 控制器旧条件会在底盘低速且路径夹角仅约 29～38°时切换为原地转向，日志
   中对应多次 `v=0`、再起步。正常偏差本应保持线速度并叠加角速度。

## 本版修改

### 1. 中等角度保持连续 `v + w` 修正

- 删除“当前线速度低于 0.03 m/s 且角度超过 0.45 rad 就原地转向”的入口；
- 小于 `hard_rotate_angle=0.80 rad`（约 46°）的路径误差，无论刚起步还是行驶中，
  都由 Pure Pursuit 在前进速度上连续叠加角速度；
- `min_linear_vel` 从 0.03 提到 0.05 m/s，避开本底盘已经实测过的低速静摩擦区；
- 大于约 46°的起始误差、终点航向校正，以及所有前进弧线均碰撞时，仍允许
  受 footprint 检查的原地转向。不会为了顺滑硬顶障碍物。

`velocity_smoother` 继续限制加速度，所以这里不是瞬间“猛加反向角速度”，而是
用户提出的连续速度矢量修正。

### 2. 收紧 slam_toolbox 的错误约束

- `ceres_loss_function: HuberLoss`：降低离群图约束拉动整张图的权重；
- `link_scan_maximum_distance: 8.0 -> 1.5`：该参数是允许建立约束的扫描位姿
  距离，不是 A2M12 的量程。旧值 8 m 会放宽相隔很远的相似结构错误连接；
- `link_match_minimum_response_fine: 0.1 -> 0.2`；
- 回环最小连续链 `10 -> 15`，粗/细响应阈值提高到 `0.40 / 0.50`；
- `map_update_interval: 2.0 -> 3.0 s`，减少大地图重绘和 Nav2 action/service
  同时抢占 CPU；动态人与障碍仍由 Nav2 传感器层实时处理，不依赖 `/map` 刷新；
- `minimum_travel_distance: 0.20 -> 0.25 m`、TF 缓存 `3 -> 10 s`；
- 关闭当前未使用的 SLAM 交互编辑模式，降低 RViz/SLAM 额外负担。

这些值以 Slam Toolbox 的官方参数语义和默认量级为基线。它们不会把
`/scan_camera` 写入静态地图；2D SLAM 仍只使用 `/scan_filtered`。

### 3. 人突然进入车前方时

安全链没有被弱化：

- `/scan_camera_safety` 和主雷达继续直接进入 Collision Monitor；
- Collision Monitor 可以立即把 `/cmd_vel` 压成零；
- `/scan_camera` 的多点、多帧确认流继续供局部/全局代价地图规划；
- 人离开后，代价地图的既有 persistence 会短时间保留障碍，宁可暂时绕行，
  不会因本版“连续弧线”而穿过人。

若雷达视野的大部分都被移动人员遮挡，纯 2D SLAM 仍可能短时降质。HuberLoss
能抑制离群约束，但不能替代以后基于语义的人体动态点过滤。实车验收先用纸板
横向移入车道，不要用真人站在机器人正前方冒险。

## 部署

```bash
cd ~/ros2_ws

sha256sum luckrobot_smooth_tracking_slam_guard_v12_14_20260930.zip

unzip -o \
luckrobot_smooth_tracking_slam_guard_v12_14_20260930.zip \
-d ~/ros2_ws

sha256sum -c \
docs/SMOOTH_TRACKING_SLAM_MATCH_GUARD_V12_14_20260930_MANIFEST.sha256

chmod +x tools/run_custom_nav_diag.sh

source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select \
  luckrobot_nav2_plugins \
  my_robot_navigation \
  my_robot_mapping \
  my_robot_bringup \
  --allow-overriding \
  luckrobot_nav2_plugins \
  my_robot_navigation \
  my_robot_mapping \
  my_robot_bringup

source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh
```

关闭旧终端后，用一个干净终端启动：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full \
  depth_obstacle_mode:=height
```

## 生效检查

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 param get /controller_server LuckFollowPath.min_linear_vel
ros2 param get /controller_server LuckFollowPath.hard_rotate_angle

ros2 param get /slam_toolbox ceres_loss_function
ros2 param get /slam_toolbox link_scan_maximum_distance
ros2 param get /slam_toolbox link_match_minimum_response_fine
ros2 param get /slam_toolbox map_update_interval
ros2 param get /slam_toolbox enable_interactive_mode
```

预期依次为：`0.05`、`0.80`、`HuberLoss`、`1.5`、`0.2`、`3.0`、`False`。

## 一次完整验收

先启动记录，再发目标：

```bash
bash ~/ros2_ws/tools/run_custom_nav_diag.sh 0
```

按顺序、一次只测一个场景：

1. 1.0～1.5 m 开阔直线；
2. 起始方向误差约 25～40°的目标，应该以连续圆弧起步，不再先停住原地转；
3. 先用纸板横向进入前方安全区，确认机器人立即减速/停车，移走后再恢复或重规划；
4. 最后重走原来在约 344～355 s 出现整图横移的长距离回访路线。

出现整幅地图、墙线或点云共同横移时，不要继续追发目标。只按一次 `Ctrl+C`，
等待生成压缩包。新版诊断会额外保存 `/slam_toolbox` 参数，并尝试记录 SLAM pose/
图事件；上传新的 `custom_nav_*.tar.gz` 即可复核。

验收门槛：

- 正常中等角度转弯无持续 `v=0` 原地摆头；
- 人/纸板进入时允许并且应该安全停车，这不算“卡顿”；
- 单次 `map -> odom` 平移不应再出现连续 20～30 cm 级跳变；
- 若仍有大跳变，需要同步核对轮胎打滑、雷达安装刚度和轮编码器实位移，不能再
  仅靠控制器参数掩盖。
