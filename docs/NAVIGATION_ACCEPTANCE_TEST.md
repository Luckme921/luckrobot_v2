# 导航闭环验收清单

本清单要求每项单独记录结果。首次运行 Collision Monitor、恢复旋转或自动探索时，先架空驱动轮，确认方向和急停，再落地低速测试。

## 0. 环境与版本快照

```bash
date
uname -a
cat /etc/nv_tegra_release
printenv | grep -E '^(ROS|RMW|FASTRTPS|CYCLONEDDS)'
ros2 doctor --report
```

记录 JetPack、ROS、RMW、网卡名和当前代码提交/压缩包日期。不同 DDS 的测试结果不要混在一组数据里。

## 1. 启动与生命周期

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py mode:=mapping
```

另一终端：

```bash
ros2 node list | sort
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /collision_monitor
```

期望四个生命周期节点均为 `active`，且没有第二个 `slam_toolbox`、`amcl` 或第二个 `map -> odom` 发布者。

## 2. 话题与频率

```bash
ros2 topic hz /scan
ros2 topic hz /scan_filtered
ros2 topic hz /scan_camera_raw
ros2 topic hz /scan_camera
ros2 topic hz /wheel/odom
ros2 topic hz /odometry/filtered
ros2 topic echo /scan_camera --once
```

期望所有过滤后话题持续有数据。若 `/scan_camera_raw` 有数据而 `/scan_camera` 没有，首先检查 `camera_scan_filter_chain` 是否成功加载参数。

## 3. TF 唯一性与时延

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link camera_depth_frame
```

期望 TF 链为 `map -> odom -> base_footprint -> base_link -> sensors`。终端不应反复出现 extrapolation、时间倒退或重复 authority 警告。

## 4. 里程计基础测试

依次测试静止 5 分钟、直行 1 m、原地旋转 360°、直行返回。记录 `/wheel/odom` 与 `/odometry/filtered` 的最终误差：

- 静止时速度应稳定回到精确 0，位姿不持续爬行。
- 直行时 Y 和 Yaw 不应明显发散。
- 原地旋转时 X/Y 不应出现大幅平移。
- 若旋转角误差明显，先录 bag 对比轮式 `Vyaw` 与 IMU `Vyaw`，再决定是否把 EKF 的轮式 `Vyaw` 关闭。

建议录制：

```bash
ros2 bag record /tf /tf_static /scan /scan_filtered /scan_camera \
  /wheel/odom /odometry/filtered /imu/data /cmd_vel /cmd_vel_safe \
  /map /map_updates
```

## 5. 近场安全层

本车体半径为 0.175 m。安全层使用半径 0.19 m、碰撞预测时域 2 s 的
`FootprintApproach`：朝障碍运动时按预计碰撞时间渐进限速；圆形底盘在
车体外有近障碍时仍可原地旋转或向远离障碍的方向脱困。旧的固定
`FrontStopZone` 会同时清零平移与旋转，已在自动探索中造成永久停车。

首次测试必须架空驱动轮。同时观察输入和安全输出：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_safe
```

按以下顺序验收：

1. 清空：无近场障碍时，`/cmd_vel_safe` 应跟随输入。
2. 接近：向正前方障碍运动时，安全输出应随预计碰撞时间缩短而平滑下降。
3. 停车：继续接近车体外缘时，安全输出必须在碰撞前降至 0。
4. 脱困：障碍仍位于圆形车体外时，原地旋转或远离障碍的指令不应被永久清零。
5. 障碍移除后，安全输出必须自动恢复，不得需要重启节点。

如果频繁误停，只调整 `collision_monitor.yaml` 的区域边界或点数阈值并重新执行全部五项测试，不要关闭 Nav2 自身碰撞检测。

## 6. Nav2 功能验收

Humble 在 controller_server 同时加载多个 goal checker 时，默认行为树不能
再把名称留空。启动后先确认手动单点/多点行为树都已加载本包版本：

```bash
ros2 param get /controller_server goal_checker_plugins
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
ros2 param get /bt_navigator default_nav_through_poses_bt_xml
```

期望 checker 列表同时包含 `general_goal_checker`、`explore_goal_checker`；
两个行为树路径均位于
`install/my_robot_navigation/share/my_robot_navigation/behavior_trees/`。
RViz 手动目标必须使用前者，自动探索树使用后者。若终端出现
`goal_checker name  ... does not exist`，其后的清图、Spin、Wait、BackUp
只是配置错误引发的恢复，必须停止测试，不能当作正常避障行为。

按顺序测试：

1. 开阔区 1 m 单点导航；
2. 90° 转弯和窄门通过；
3. 路径中临时放置纸箱，确认局部避障或安全重规划；
4. 人为堵死路径，确认清图、Spin、Wait、BackUp 恢复动作不会高速突变；
5. 连续发送 10 个目标，统计成功率、耗时、最小障碍距离和恢复次数。

合格基线建议：10 次中至少 9 次无需人工干预；任何碰撞、TF 断链、底盘失控均判定失败。

当前实车采用 `xy_goal_tolerance: 0.08` 和 `yaw_goal_tolerance: 0.10`
（约 5.7°）。A/B 测试中，旧的 `0.15 m / 0.15 rad` 在位置误差约
10.2 cm、航向误差约 8.0° 时提前判定成功；新容差将误差收敛到约
7.1 cm 和 4.85°，近目标修正平滑完成。后续只有在连续目标验收出现
明显反复摆动时才放宽容差。

进度检查使用 `nav2_controller::PoseProgressChecker`，平移超过 0.05 m
或旋转超过 0.20 rad（约 11.5°）都视为有效进展。旧的
`SimpleProgressChecker` 只检查 XY，180° 返回测试中机器人持续慢速转动，
仍在 35.01 s 时被误判为无进展并触发一次无必要的清图重试。

当前 Humble 栈的纯航向目标不要构造为“位置完全相同的
`NavigateToPose`”：这会形成零长度路径，实车测试中控制器可能持续等待。
纯原地转向应调用 Nav2 `Spin` Action；带平移的普通导航目标仍使用
`NavigateToPose`。

## 7. 自动探索与终身地图

本机使用面向实车清扫规律的 V10 UNKNOWN-basin 拓扑 DFS `explore_lite`。蓝/红色小点
仅表示白/灰边界，不是导航目标；绿色圆点是实际目标，黄色点是记住的
白区父分叉，紫色点是待探索子分支：

- 每条前沿后方只沿灰色未知栅格做 1.50 m 有界洪泛，再从可达白色观察点
  做前向视锥与遮挡检查，将可直接观察的 UNKNOWN 面积作为宏观分支收益；
  大房间/宽通道优先，小于 `0.50 m²` 的低价值灰区不生成目标；
- 只保留长度至少 `0.30 m`（约 6 个 5 cm 栅格）的连续白/灰边界，过滤
  零散未知噪点；该值是边界长度阈值，不是未知区域面积；
- 在前沿 `1.00 m` 范围的已知自由区搜索距未知区/占用区至少 `0.25 m`
  的安全落脚点；车体
  半径为 `0.175 m`，因此保留 75 mm 目标点余量，同时不会把 `0.30 m`
  的 inflation 软代价误当成不可通行硬边界；
- 再用 `/global_costmap/costmap` 检查落脚点，只接受代价不超过 `20` 的
  低代价白区。这样可排除 SLAM 地图显示为空白、但实时障碍物或膨胀层
  已认为贴近椅腿/墙面的点；
- 全局代价地图每秒发布完整最新状态，避免只收到增量更新时探索器继续
  使用旧图；调用 `/compute_path_to_pose` 后只沿路径的安全前缀前进，
  每次下发目标距当前位置不超过 `1.20 m`，且终点必须仍是已知白色、
  全局代价不超过 `20` 的栅格；前缀允许经过不超过 `80` 的膨胀软代价，
  但仍拒绝内切/致命碰撞区和未知区。RPP 碰撞检查与 collision monitor
  全程保持开启；
- 保留手动导航稳定基线的 `0.35 m` 规划器目标替代容差；探索层会检查
  Nav2 实际返回的路径终点和每个局部前缀，替代终点不会被误报成已经
  到达前沿；
- 距机器人不足 `0.25 m` 的落脚点不发送瞬间成功目标，但也不再永久判定
  整片分支已经覆盖；到达观察点后冷却该前沿快照 1.5 s，等待 SLAM 边界
  沿 UNKNOWN basin 向前推进；
- 机器人当前位置代价为 `41～98` 的软膨胀时，只允许最多 `0.30 m`、
  终点代价 `<=20` 的规划脱困步；未知、地图外或 `99/100` 内切/致命格
  仍暂停发目标；连续 15 秒没有恢复为安全白区便安全暂停整个探索，避免从
  真正无效的起点强行移动或节点数分钟假运行；
- 实际目标朝向沿规划路径末段，而不是强制朝向灰区。自动探索显式使用
  `explore_goal_checker`（0.12 m、忽略最终 yaw），避免把每个局部白区连接点
  当成需要精确停车定向的任务终点；手动 RViz 仍使用 `general_goal_checker`
  的 0.08 m / 0.10 rad；
- 探索使用无恢复循环的独立行为树。静态不可达、预检超过 `6 s` 时将该
  前沿冷却 `20 s`；无运动进展或跟踪中止时将整片 `0.85 m` 区域冷却
  `60 s`，不在椅子旁反复清图、Spin、Wait、BackUp；地图更新后仍可
  重试，不会一次性永久拉黑所有前沿；
- 导航期间连续 `15 s` 没有至少 `0.05 m` 平移且没有至少 `0.15 rad`
  转向进展，才取消目标。正常起步调头不会再被误判成卡死；
- V10 不再仅按前沿质心是否接近来猜测“同一分支”。它在当前白区上计算到
  各候选的保守连通路径，按公共路径前缀找真实分叉，把父分叉与未选子边
  存入 LIFO 栈。普通分支只接受当前路径航向前方 `70°` 锥内且有正向投影
  的候选；侧分支结束后从当前位置由 Nav2 直接规划到记忆分支的实时入口，
  真实窄通道仍会自然经过父分叉，开放区域不再逐点倒回旧坐标。已完成分支按分叉、
  方向、前沿位置和 UNKNOWN basin 联合去重；同一 basin 的可见边界可随
  建图平移，已完成区域若后来新增未知收益也允许重新打开；
- 当前分支暂时失败时冷却具体前沿并尝试栈内旁支，不让 Nav2 在椅子旁
  反复执行恢复动作。V10.4 会过滤小于 `0.35 m²` 的残余未知碎角，只把
  至少 `0.60 m²` 的宏观分支写入 DFS 栈；活动分支和记忆栈清空后连续
  `15 s` 没有安全候选才判定探索结束。V10.5 进一步要求沿活动分支至少
  前进 `1.20 m` 才能再记录一层分叉，且恢复分支的路径必须经过旧锚点
  并沿原出口离开，避免同一走廊沿途产生多个别名分支。V10.6 进一步只统计
  从可达白色观察点前方约 120° 视锥内、没有被墙/障碍遮挡的 UNKNOWN；
  即时目标至少需要 `0.50 m²`，写入兄弟分支栈至少需要 `0.75 m²`。

`planner_frequency: 2.0` 表示每 0.5 秒检查一次状态。目标已锁定或正在进行
路径预检时，定时器不会反复抢占 Nav2。到点地图刷新窗口为 `0.05 s`，
正常接续约 0.05~0.55 s；本轮不启用到点前抢占。普通安全段增至
`1.20 m`，以减少进入末端减速区的次数。

```bash
ros2 launch my_robot_mapping auto_explore.launch.py
```

终端日志应依次出现 `V10.6 precheck frontier`、
`Advancing one local branch step`（或 `Local path reaches`）以及
`V10.6 goal locked` 和 `Reached local connector step`。`V10.6 goal locked` 中的
实际目标步长必须不大于 `1.20 m`，全局终点代价必须不大于 `20`。
若从软膨胀区恢复，日志应带 `soft-cost escape`，该步长必须不大于
`0.30 m`；若出现 `hard stop >= 99`，机器人必须保持停止，且 15 秒后探索器
必须明确进入 safety-pausing，而不是无限等待。
发现分叉时应出现 `DFS remembered`；分支结束后应出现 `DFS branch ...
complete` 和 `DFS direct transit`。转场路径应由 Nav2 走当前最短安全路线，
不应机械重复先前的一串回退目标。
不可达区域应出现 `Deferring frontier`，
不应先进入行为树的多轮 Spin/Wait/BackUp。
探索结束后同时保存栅格地图和 posegraph。随后完全停止系统，以定位模式重启：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=localization posegraph:=/home/nvidia/ros2_ws/maps/my_room_map
```

从充电桩（首节点）启动时应直接定位；从其他位置启动时需用 RViz 给出初始位姿并验证重定位。完成这一闭环，才算真正具备可继续开发 VSLAM 与语义导航的基线。
