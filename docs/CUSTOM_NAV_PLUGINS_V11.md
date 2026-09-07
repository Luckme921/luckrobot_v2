# LuckRobot Nav2 自定义规划与控制架构 V11

> V11.0 首轮实车记录已发现终点状态切换死锁；不要继续部署本页旧参数。
> 修复和复测步骤见 `CUSTOM_NAV_CONTROLLER_FIX_V11_1.md`。

## 1. 本轮边界

本轮保持 Nav2 的生命周期、行为树、代价地图、速度平滑器、Collision
Monitor 和底盘驱动不变，只新增两个 Humble 插件：

- `luckrobot_nav2_plugins/LuckController`
- `luckrobot_nav2_plugins/LuckPlanner`

深度相机夹具高度及现有传感器外参也保持不变；本轮只隔离验证规划与控制算法。

现有 `SmacPlanner2D + RegulatedPurePursuitController` 仍是默认 `stable`
基线。自动探索仍显式选择 `GridBased + FollowPath`，本轮不会暗中改变探索。

三个可启动档位：

| 档位 | 全局规划器 | 控制器 | 用途 |
|---|---|---|---|
| `stable` | SmacPlanner2D | RPP | 已验收回退基线（默认） |
| `luck_controller` | SmacPlanner2D | LuckController | 第一阶段实车 A/B |
| `luck_full` | LuckPlanner | LuckController | 控制器通过后才进行第二阶段 |

## 2. 为什么不直接复用 V1 控制律

V1 项目证明了自定义 Nav2 插件架构可行，但旧控制器采用明显的“停车转向—
直线前进”状态切换，直行阶段没有连续角速度修正。这与 V2 已观察到的窄通道
左右摆动、动态重规划时反复进入原地旋转不匹配。因此 V11 只参考包结构和接口，
重新实现控制律。

## 3. LuckController V11.0

控制器以 `vx + wz` 连续修正为主：

1. 在局部代价地图坐标系中寻找最近路径点和 0.35 m 前视点；
2. 纯追踪曲率作为前馈，路径切线误差作为反馈；
3. 起步误差超过 0.45 rad 才进入原地转向，低于 0.15 rad 才退出；
4. 已经行驶时，小于 0.80 rad 的突发偏差优先边走边修正；
5. 按曲率、机器人所在膨胀代价、剩余路径和 0.35 m 到点距离连续降速；
6. 对输出圆弧做 1.0 s 车体轮廓前向碰撞检查；
7. 最终安全链仍是 `velocity_smoother -> collision_monitor -> chassis`。

调试话题 `/luck_controller/tracking_error` 的八个数依次为：

1. 到终点直线距离；
2. 前视点角度误差；
3. 横向误差；
4. 曲率；
5. 输出线速度；
6. 输出角速度；
7. 当前代价；
8. 是否处于原地转向状态。

## 4. LuckPlanner V11.0

规划器使用八邻域 A*，搜索状态包含进入栅格的方向，因此可对换向增加代价，
同时使用全局膨胀代价把路径推向通道中部。对结果仅在低代价已知区域执行
视线化简，再按 0.05 m 插值。禁止对角穿过两个障碍角点。

这不是宣称优于 Smac2D 的最终版本，而是可观测、可按实车数据迭代的 V2
基线。第一轮禁止直接启用 `luck_full`，避免无法区分规划与控制问题。

## 5. Jetson 部署与第一轮测试

关闭终端 1、自动探索以及所有残留 Nav2 后执行：

```bash
cd ~/ros2_ws

sha256sum luckrobot_custom_nav_plugins_v11_20260907.zip

stamp=$(date +%Y%m%d_%H%M%S)
tar -czf ~/luckrobot_before_custom_nav_v11_${stamp}.tar.gz \
  src/my_robot_navigation \
  src/my_robot_bringup \
  tools

unzip -o luckrobot_custom_nav_plugins_v11_20260907.zip -d ~/ros2_ws

sha256sum -c docs/CUSTOM_NAV_PLUGINS_V11_MANIFEST.sha256

source /opt/ros/humble/setup.bash
rosdep install --from-paths src/luckrobot_nav2_plugins --ignore-src -r -y

colcon build --symlink-install \
  --packages-select luckrobot_nav2_plugins my_robot_navigation my_robot_bringup \
  --allow-overriding luckrobot_nav2_plugins my_robot_navigation my_robot_bringup

source ~/ros2_ws/install/setup.bash
```

先进行无运动检查：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py --show-args

grep -n -E 'LuckGrid|LuckFollowPath|rotate_to_heading_min_angle' \
  ~/ros2_ws/install/my_robot_navigation/share/my_robot_navigation/config/nav2_params.yaml
```

应看到 `navigation_profile` 的三个选择，同时稳定 RPP 的
`rotate_to_heading_min_angle` 仍为 `0.35`。

终端 1 先启动控制器单变量档：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping \
  navigation_profile:=luck_controller
```

终端 2 录制，然后在 RViz 只发送一个 0.8~1.2 m 的开阔目标：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/run_custom_nav_diag.sh 90
```

首轮上传 `~/custom_nav_时间.tar.gz`。验收顺序是开阔直线、带约 20°起始
偏角、原窄通道；不启动自动探索，不在同一轮改变参数。

任何异常都可重启并回退：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping \
  navigation_profile:=stable
```

只有 `luck_controller` 在相同目标上达到“不碰边、角速度不频繁换向、终点不过冲”
后，下一轮才把档位改为 `luck_full` 测试规划器。

## 6. 清理旧部署包

清理脚本默认仅预览，且只处理 `~/ros2_ws` 根目录下已被替代的
`luckrobot_*.zip`。它不会删除 rosbag、诊断 tar 包、源码、build/install/log
或家目录备份：

```bash
bash ~/ros2_ws/tools/cleanup_old_deployment_zips.sh
bash ~/ros2_ws/tools/cleanup_old_deployment_zips.sh --apply
```
