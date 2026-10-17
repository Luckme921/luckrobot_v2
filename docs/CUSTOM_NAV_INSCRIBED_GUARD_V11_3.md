# LuckRobot V11.3：拐角内切代价保护

## 1. 两次实车记录的结论

`custom_nav_20260908_091049` 使用 `luck_controller`，即 GridBased 全局规划器与
LuckController。前两个目标正常到达；第三段行驶约 0.94 m 后停在转角：

- Collision Monitor 没有修改 `/cmd_vel`，不是最终安全层误停；
- LuckController 多次选中最大 footprint 代价为 253 的局部速度弧线；
- 随后机器人所在全局栅格变为 253，GridBased 报告起点位于致命空间；
- 控制器再无局部安全轨迹，行为树只能重复清图和重规划。

Nav2 中机器人中心代价 252 是最后一个可规划值，253 是
`INSCRIBED_INFLATED_OBSTACLE`。V11.2 只做 footprint 的 254 致命碰撞检查，
没有单独检查预测轨迹的中心代价，因此能把中心驶入全局规划器不再接受的格子。

`custom_nav_20260908_092417` 使用 `luck_full`，即 LuckPlanner + LuckController。
三次目标均到达，最长目标由 3.298 m 收敛到 0.037 m；横向误差 P90/最大值为
0.022/0.050 m，Collision Monitor 对 1138 组配对速度指令均未做截断。
成功包中也有 444 个周期的 footprint 最大代价 253；这证明不能简单拒绝所有
footprint 253，否则会破坏已经成功的转弯。自定义 full 组合能够稳定工作，
也说明不应在本轮同时改动正常跟踪增益。

## 2. 修复内容

1. LuckController 新增 `max_trajectory_center_cost`，默认和 LuckPlanner 一致为 252。
2. 正常轨迹的任一预测中心代价大于 252 时，候选弧线立即失效；若起点已经
   位于 253，只允许最终真正退出到 `<=252` 的平移弧线，安全原地转向仍可用。
   footprint 继续按 254 致命碰撞阈值检查。
3. 无候选时保留 footprint 与中心两类最大代价到调试话题，便于区分“安全贴边”
   和“中心已经进入 253 内切区”。
4. 持续局部无解继续使用 Humble 提供的 `nav2_core::PlannerException`；
   Controller Server 会先按 `failure_tolerance` 输出零速，再把失败交给专用
   安静恢复树。不能使用新版本 Nav2 才有的 `NoValidControl`。
5. 诊断分析器补齐避障字段解析，并分别统计 footprint、当前中心和预测中心代价。

V11.3 调试数组在 V11.2 基础上新增：下标 13 为预测 footprint 最大代价，
下标 14 为预测机器人中心最大代价，下标 15 为连续阻塞时长。分析器同时兼容
旧的 15 项 V11.2 数组。

本轮没有修改线速度、角速度、转向增益、机器人半径、膨胀半径、里程计、
Collision Monitor 或探索算法。修复的预期不是让真实窄缝也强行通过，而是在
进入零余量区域之前停车并获得新路线。

## 3. 部署

必须先关闭旧的终端 1，再执行：

```bash
cd ~/ros2_ws
sha256sum luckrobot_custom_nav_inscribed_guard_v11_3_20260908.zip
unzip -o luckrobot_custom_nav_inscribed_guard_v11_3_20260908.zip -d ~/ros2_ws
sha256sum -c docs/CUSTOM_NAV_INSCRIBED_GUARD_V11_3_MANIFEST.sha256

source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select luckrobot_nav2_plugins my_robot_navigation my_robot_bringup \
  --allow-overriding luckrobot_nav2_plugins my_robot_navigation my_robot_bringup
source ~/ros2_ws/install/setup.bash

ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping navigation_profile:=luck_full
```

运行后另开终端确认：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 param get /controller_server LuckFollowPath.max_trajectory_center_cost
ros2 param get /planner_server LuckGrid.max_planning_cost
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
```

控制器中心代价和规划器最大代价必须均为 252；旧的
`LuckFollowPath.max_trajectory_cost` 不应存在。行为树路径必须以
`navigate_to_pose_luck_full.xml` 结尾。

## 4. 实车复测

把机器人放到已知白色空闲区，footprint 不得压住紫色/蓝色内切代价区；如果
上次已经物理卡在拐角，先断电并人工移回空闲区，不能依靠盲目倒车脱困。

```bash
source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/run_custom_nav_diag.sh 150
```

只发送一个需要经过原拐角的目标。验收条件：

- 正常路段继续跟踪，不能为了安全修复产生持续左右摇摆；
- 调试统计不得再出现选中轨迹**中心**代价 `>=253`；footprint 最大代价 253
  本身不等于碰撞；
- 转角无安全弧线时应在进入内切区前停车、等待或重规划；
- 若仍卡住，上传这次新的 `custom_nav_*.tar.gz`，不要用成功包代替故障包。

召回场景必须单独再录一次，因为本轮提供的 full 包在停止录制前没有复现召回
卡住，不能由这份成功记录推断其具体原因。
