# LuckRobot V12.14.1 紧急回退与动态地图边界

## 状态

V12.14 已撤回。不要继续使用
`luckrobot_smooth_tracking_slam_guard_v12_14_20260930.zip` 实车导航。

本回退包只撤销 V12.14 引入的控制器与 SLAM 实验改动，保留此前已经通过实车
验证的功能：

- 深度相机高度障碍扫描；
- `min_points_per_bin=5` 空间支持过滤；
- `/scan_camera_safety` 即时安全流；
- `/scan_camera` 多帧/多束确认规划流；
- 雷达与相机独立的局部、全局代价图层；
- 相机障碍局部 6 s、全局 16 s 的有限记忆；
- Collision Monitor；
- LuckPlanner 路线迟滞及 0.16 m 拐角前视。

## 三份 V12.14 实车包的证据

分析对象：

- `custom_nav_20260930_100232`；
- `custom_nav_20260930_100558`；
- `custom_nav_20260930_100845`。

### 第一次

- 控制器约 19 s 处于转向脱静摩擦状态；
- 多次持续下发 `w=-0.35 rad/s`，但 IMU确认车体没有对应转动；
- 用户推动后才恢复运动；
- Collision Monitor 只拦截 3 个非零样本，不是安全层导致长期不动。

### 第二次

- Collision Monitor 没有拦截非零命令；速度话题也没有中途断流；
- `map -> odom` 在 53.89、57.73、63.51、70.81 s 分别跳变约
  29.9、36.9、37.1、37.2 cm；
- 这与整张地图和点云共同横移完全一致，属于定位/建图失稳。

### 第三次

- 22.5 s 内，轮式里程计报告约 1.0 m 平移、25°偏航；
- 同期融合里程计/IMU偏航约 0°；
- `map -> odom` 在 17.09 和 21.35 s 跳变 31.2、36.6 cm，角度各约 5.8°；
- 说明左右轮打滑、接地/阻力不一致或编码器运动与车体真实运动不一致，随后
  SLAM被迫用大幅离散修正补偿。

因此故障不是深度相机障碍点写入 SLAM：`slam_toolbox` 的输入仍只有
`/scan_filtered`，相机只进入 Nav2 代价地图。

## 本回退具体做什么

1. LuckController 恢复 V12.12.1 已验证实现；
2. `min_linear_vel` 恢复为 `0.03 m/s`；
3. `nav2_params.yaml` 恢复到 V12.13.1 的文件校验值；
4. `mapper_params_online_async.yaml` 恢复 V12.14 前的运行参数；
5. 保留增强后的诊断脚本，它只录包，不参与控制。

这一步的目的不是宣称旧 SLAM 参数已经完美，而是先恢复唯一可比较的运动基线，
停止在机械打滑状态上继续叠加算法变量。

## 动态地图应该如何处理

### 动态人员和短时物体

不要写入静态 SLAM 地图。当前正确链路是：

```text
雷达 / 高度相机
        -> Nav2 obstacle layers（marking + clearing）
        -> Collision Monitor
        -> 临时绕行或停车
```

相机障碍会在局部图保留最多约 6 s、全局图保留最多约 16 s，然后由新观测和
raytrace clearing 清除。人突然出现时允许立即停车；人离开后短暂等待或绕行是
安全迟滞，不应永久写进墙体地图。

### 移动的家具或长期布局变化

`mode:=mapping` 会继续增加和优化 pose graph，但普通在线 mapping 不等于可靠的
“动态障碍自动遗忘”。旧扫描仍在 pose graph 中，移动后的家具可能留下重影。

推荐工作流：

1. 建图/自动探索阶段使用 `mode:=mapping`；
2. 地图验收后保存 posegraph；
3. 日常导航改用 `mode:=localization`，静态地图不再被人员和临时物体改写；
4. 家具长期变化时，单独进入受控的重新建图或 `lifelong` 维护任务，验收后保存
   新版本，不在日常人流中持续修改主地图。

Slam Toolbox 的 true lifelong/node decay 仍属于实验能力且计算开销更高，不适合
把每个走动的人当作“动态修图”来源。

## 部署回退

```bash
cd ~/ros2_ws

sha256sum luckrobot_v12_14_1_safe_rollback_20260930.zip

unzip -o \
luckrobot_v12_14_1_safe_rollback_20260930.zip \
-d ~/ros2_ws

sha256sum -c \
docs/V12_14_1_EMERGENCY_ROLLBACK_20260930_MANIFEST.sha256

chmod +x tools/run_custom_nav_diag.sh

source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select \
  luckrobot_nav2_plugins \
  my_robot_navigation \
  my_robot_mapping \
  --allow-overriding \
  luckrobot_nav2_plugins \
  my_robot_navigation \
  my_robot_mapping

source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh
```

必须关闭所有旧启动终端，再开一个干净终端：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full \
  depth_obstacle_mode:=height
```

检查回退值：

```bash
ros2 param get /controller_server LuckFollowPath.min_linear_vel
ros2 param get /slam_toolbox ceres_loss_function
ros2 param get /slam_toolbox link_scan_maximum_distance
ros2 param get /slam_toolbox link_match_minimum_response_fine
ros2 param get /slam_toolbox map_update_interval
ros2 param get /slam_toolbox enable_interactive_mode
```

预期依次为：`0.03`、`None`、`8.0`、`0.1`、`2.0`、`True`。

## 再次通电前的物理检查

第三包已经证明仅靠软件无法判断直线方向的真实滑移距离。先检查：

- 两个驱动轮是否都完整压地，轮胎是否沾灰、油或表面过滑；
- 万向轮是否卡滞、方向是否顶住；
- 抬高相机后重心是否导致一侧驱动轮载荷明显变小；
- 相机线、网线、电源线是否拖地或在转向时拉住车体；
- 轮毂、联轴器、紧固螺丝是否有空转；
- 不要在 Nav2 正在发命令时用手推动车体，这会同时破坏 odom 与 scan match。

## 最小复测

回退后先静止 30 s，确认墙线不自行横移。然后只测试一个与当前朝向基本一致、
距离 0.6～0.8 m 的直线目标，同时记录：

```bash
bash ~/ros2_ws/tools/run_custom_nav_diag.sh 60
```

本轮不要先测试大角度原地转向，不要推车，不要同时放置动态障碍。如果直线仍
出现左右轮空转、地图横移或整车实际距离明显小于里程计，应停止导航，优先处理
机械接地/传动问题；继续调 SLAM 参数只会掩盖根因。
