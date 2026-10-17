# LuckRobot V11.2：局部速度采样与安静重规划

## 1. 本轮问题与结论

实车日志反复出现：

```text
LuckController projected footprint collision; commanding zero for replanning
Passing new path to controller.
```

V11.1 的碰撞预测只有“通过/停车”两个结果：名义速度弧线与局部代价地图中的
致命栅格相交后，线速度和角速度会同时清零。全局规划器虽然每秒刷新路径，
但若新路径的近端形状没有改变，控制器会再次得到相同结果，于是形成约 1 Hz 的
“传新路径—判碰撞—零速度”循环。它不是行为树崩溃，也不代表底盘方向错误，
而是控制器缺少局部可行速度搜索。

## 2. V11.2 状态机

本轮保留 Nav2 的 Lifecycle、Action、TF、全局/局部 Costmap、速度平滑器和
Collision Monitor，只替换算法插件和自定义行为树：

1. **名义轨迹安全**：继续使用 V11.1 已验证的前视、曲率、终点减速和转向控制，
   不改变正常直线与到点行为。
2. **名义轨迹被挡**：在 1.5 s 滚动窗口中评价 3 组线速度和 7 组角速度，
   对每条轨迹用机器人 footprint 做逐点碰撞检查。
3. **局部绕行**：从安全候选中综合评价前视点距离、航向误差、最大代价、
   速度损失、角速度连续性，并对短时间内左右换边增加惩罚。
4. **动态障碍等待**：所有候选均不安全时先输出零速度，等待行人离开；
   期间全局路径仍以 1 Hz 更新，一旦出现安全路径立即恢复。
5. **持续阻塞重规划**：5 s 后仍无安全候选，或虽然存在候选但里程计持续显示
   机器人未运动（例如最终安全层一直压制速度），控制器抛出 Humble 支持的
   `nav2_core::PlannerException`，让 FollowPath 正常失败。专用行为树只执行
   `清局部图 -> 等待 -> 清全局图 -> 重新规划`，不执行盲目 Spin 或 BackUp。
6. **最终安全层**：Collision Monitor 仍位于 `/cmd_vel` 与 `/cmd_vel_safe`
   之间，对任何上游算法拥有最终减速/停车权。

这套逻辑不会保证任意拥挤环境中都一定存在可行路径；它保证的是：有安全局部
绕行时主动选择，没有时停车等待，持续不可达时可解释地重规划/失败，不用乱转
或倒车掩盖问题。

## 3. 可观测接口

- `/luck_controller/local_trajectory`：本周期选中的安全预测轨迹。
- `/luck_controller/lookahead`：当前前视点。
- `/luck_controller/tracking_error`：数组含义如下：

| 下标 | 含义 |
|---:|---|
| 0 | 距离最终目标（m） |
| 1 | 前视点方向误差（rad） |
| 2 | 横向误差（m） |
| 3 | 名义曲率 |
| 4、5 | 最终输出线速度、角速度 |
| 6 | 机器人中心栅格代价 |
| 7 | 原地转向状态 |
| 8 | 最终航向误差 |
| 9 | 终点航向精修状态 |
| 10 | 当前 Goal Checker 位置容差 |
| 11 | 避障状态：0 名义、1 局部绕行、2 等待、3 请求重规划 |
| 12 | 本周期安全候选数量 |
| 13 | 选中轨迹最大 footprint 代价 |
| 14 | 连续阻塞时长（s） |

## 4. Jetson 部署与 A/B 测试

部署包解压到 `~/ros2_ws` 后执行：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select luckrobot_nav2_plugins my_robot_navigation my_robot_bringup \
  --allow-overriding luckrobot_nav2_plugins my_robot_navigation my_robot_bringup

source ~/ros2_ws/install/setup.bash
```

先仅验证新控制器，保持 SmacPlanner2D：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping navigation_profile:=luck_controller
```

同一场地、同一目标通过后，再验证自定义规划器与控制器：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping navigation_profile:=luck_full
```

另开终端录制单目标测试：

```bash
source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/run_custom_nav_diag.sh 120
```

测试顺序必须是：开阔直线目标、带转弯目标、纸箱静态封路、人员横穿后离开。
一次只发送一个 RViz 目标，两个导航档位使用相同起终点。人员测试不得代替
Collision Monitor 的纸箱/软障碍安全测试。

## 5. 验收标准

- 开阔目标不进入避障状态，保持 V11.1 的轨迹与终点表现。
- 人员横穿时，状态 1/2 可出现；人离开且代价地图清除后自动继续。
- 固定障碍完全封路时先停车，约 5 s 后出现一次可解释的控制失败并重规划。
- 日志中不得出现因本树主动触发的 `Running spin` 或 `Running backup`。
- 不允许连续左右换向、贴障强行通过或绕过 Collision Monitor。
- `luck_controller` 通过后才评价 `luck_full`，否则无法区分规划器与控制器问题。
