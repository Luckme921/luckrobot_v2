# LuckRobot V11.6：最小拐角恢复

## 1. 新记录给出的确定结论

V11.5.1 没有复现 `free(): invalid next size`。启动检查在 `t=0` 确认五个必要
生命周期节点全部 active；`t=2` 时只有 `/planner_server` 的生命周期服务一次
没有及时返回。检查结束时 ROS 图中仍同时存在 `/controller_server` 与
`/planner_server`，随后控制器连续执行三个导航目标，因此旧脚本的 `RESULT=FAIL`
是服务查询瞬时超时造成的假失败，不是进程崩溃。

实车导航时间线为：

- `(0.00, 0.00) -> (0.82, -0.15)` 成功；
- `(0.75, -0.14) -> (1.67, -0.71)` 成功；
- `(1.63, -0.69) -> (0.76, -2.16)` 在转角进入局部死锁。

第三段在阻塞前先选到 17 条安全候选，再降为 8 条，最后所有正向候选均发生
预测 footprint 碰撞。控制器等待约 5 秒后正确终止 FollowPath，但行为树 Wait
后的新 FollowPath 继承了旧的 `blocked_since_seconds_`：日志直接从 6.4、7.8、
9.2 秒继续，因此每次只经过 Controller Server 的 0.3 秒容错便再次失败。清图和
重新规划确实发生了，但输入地图和机器人位置近似不变时，确定性 LuckPlanner
仍会给出近似相同的路线。

## 2. 控制修正

### 2.1 行驶时只使用当前几何前视点

Pure Pursuit 已通过前视点计算：

```text
curvature = 2 * target_y / lookahead_distance²
angular   = linear * curvature
```

旧代码又叠加未来路径线段的切向误差。在动态重规划拐角处，未来切向可能与当前
前视点位于机器人两侧，导致修正过头甚至反向打角。V11.6 将
`path_heading_gain` 固定为 0，并从控制公式中彻底移除该项。小于原地转向阈值
的误差仍由 10 Hz Pure Pursuit 以 `v+w` 连续修正，不会停下来对准。

### 2.2 全部前进弧线失败后的有界松角

当正常指令及 3×7 条正向速度弧线全部无效，控制器才按实际前视点方向尝试
`0.12 rad/s` 原地旋转。每个 0.1 秒控制周期都用完整 footprint 预测后续 1.5 秒，
不安全则尝试反方向，两边都不安全就输出零速度。该动作最多持续 2 秒，角运动
不计作路线进展；一旦出现安全正向弧线立即恢复前进，仍无解则交回行为树重规划。

这不是盲目 Spin，也没有加入后退。Collision Monitor 和底盘心跳制动仍处于
最终速度链中。

### 2.3 新 FollowPath 的计时边界

控制器记录最近一次 10 Hz 计算时间。普通 1 Hz 路径更新与最近控制周期只相差
约 0.1 秒，不会复位阻塞计时；行为树 Wait 后重新开始 FollowPath 的间隔超过
0.5 秒，会清空上一轮阻塞状态。因此新的尝试从 0 秒开始，同时固定障碍仍会在
有界时间内失败，避免无限等待和 0.3 秒快速失败循环。

## 3. 未在本版混入的改动

- 不降低车体半径、全局/局部膨胀半径或碰撞阈值；
- 不恢复 V11.5 的扩展调试数组、航向滤波器和参数组；
- 不加入倒车、随机转向或自动探索；
- 不实现失败走廊记忆。若有界松角后全局规划仍反复返回同一死路，需要新的
  转角 rosbag 才能设计跨节点的失败走廊/临时禁行代价，而不能继续猜参数。

## 4. 部署与验证

关闭旧导航终端后执行：

```bash
cd ~/ros2_ws

sha256sum luckrobot_custom_nav_minimal_corner_recovery_v11_6_20260908.zip

unzip -o \
luckrobot_custom_nav_minimal_corner_recovery_v11_6_20260908.zip \
-d ~/ros2_ws

sha256sum -c \
docs/CUSTOM_NAV_MINIMAL_CORNER_RECOVERY_V11_6_MANIFEST.sha256

source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --cmake-clean-cache \
  --packages-select luckrobot_nav2_plugins my_robot_navigation my_robot_bringup \
  --allow-overriding luckrobot_nav2_plugins my_robot_navigation my_robot_bringup

source ~/ros2_ws/install/setup.bash

ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping navigation_profile:=luck_full
```

另开终端先运行修正后的启动门禁：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/check_custom_nav_startup.sh 60
```

门禁通过后，必须在发送目标以前启动录包。先单独录一个开阔短目标：

```bash
bash ~/ros2_ws/tools/run_custom_nav_diag.sh 90
```

再单独录原转角目标：

```bash
bash ~/ros2_ws/tools/run_custom_nav_diag.sh 180
```

转角允许出现：

```text
LuckController all forward arcs blocked; bounded corner unwind ...
```

但不应再出现阻塞时间跨重试从 `6.4 s`、`7.8 s` 连续增长的快速失败。如果仍
卡住，不要重新发布另一个目标改变样本；卡住后按 Ctrl-C，让脚本打包并上传该
次 `~/custom_nav_*.tar.gz`。
