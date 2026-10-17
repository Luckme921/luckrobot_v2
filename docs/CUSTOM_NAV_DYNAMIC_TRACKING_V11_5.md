# LuckRobot V11.5：动态航向阻尼与拐角有界脱困

## 1. 本轮实测结论

分析 `custom_nav_20260908_105210` 后，第三个目标由 1.770 m 收敛到 0.332 m
后失败。Collision Monitor 对 828 组时间对齐的速度指令修改数为 0，因此不是
保护器误停。机器人中心当前代价最高为 153，候选轨迹中心最高为 173，均未进入
253 内切区；真正否决速度的是预测车体外轮廓碰到 254 致命障碍。控制器连续
40 次报告无安全局部轨迹，行为树已经重复清图和重规划，但地图没有出现新的
通路信息，规划器每秒返回近似相同的贴角路径，所以“已有动态重规划”不等于
一定会得到不同路线。

行驶摆动也有确定证据。在约 87.5 秒处，几何前视点仍在车体左侧约 15°，但
新全局路径的前方线段切向跳到右侧约 50°。V11.4 的
`linear * curvature + path_heading_gain * future_tangent_error` 被后半项主导，
把指令突然打到右侧 `-0.35 rad/s`。随后路径与控制反复换向，整包轮式里程计
仅走 3.11 m，却累计旋转约 699°。这不是坐标轴反号，也不是把控制周期改成
0.2/0.5 秒可以解决的问题。

## 2. 10 Hz 连续动态修正

LuckController 本来就在 10 Hz（每 0.1 秒）重新读取位姿、规划路径和当前速度。
V11.5 保留这个更快的闭环，并在当前线速度上连续叠加角速度，不增加停车对准：

```text
e_filtered = low_pass(angle_to_lookahead, tau=0.30 s)
e_active   = deadband(e_filtered, 0.04 rad)
w_pp       = v * curvature
w_cmd      = w_pp + clamp(0.35 * e_active, -0.10, 0.10) - yaw_rate_damping
```

关键变化是修正方向来自实际几何前视点，不再来自可能跨过拐角的未来线段切向。
0.30 秒低通吸收 1 Hz 重规划的瞬时跳变，约 2.3° 的死区不追逐栅格噪声，
`0.10 rad/s` 限幅防止附加项压过 Pure Pursuit 曲率项。里程计反馈的角速度只在
实测旋转速度超过当前同向请求时施加阻尼；它不会为了制动生成反方向指令，避免
再次出现“目标仍在左侧、控制器却向右打满”的现象。

## 3. 拐角有界脱困与重新规划

原局部采样器只有 3 组正线速度 × 7 组角速度。当 21 条前进弧线全部发生
footprint 碰撞时，即使机器人原地转动是安全的，也只能输出零速度。V11.5 只在
这一条件下增加 `±0.15 rad/s` 的原地旋转候选，并优先选择减小前视点误差的
方向。它不是正常行驶策略，也不会替代用户要求的边走边修正。

原地松角最多持续 2.5 秒：一旦出现安全的正向弧线就立即恢复前进；仍然无解则
主动结束本次 FollowPath，让现有行为树执行清局部代价地图、等待和全局重规划。
原地旋转不再被误判为“路径已有进展”，所以不会无限自转。行为树 Wait 后的新
FollowPath 获得一次新的有界尝试；普通 1 Hz 路径刷新不会反复清零阻塞计时。

## 4. 规划器拐角约束

保留全局/局部膨胀半径 `0.30/0.25 m`。两者表达不同策略，不要求相等：全局
较宽的软代价带让长路径远离墙角，局部较窄的带保留狭窄区机动空间。把全局半径
缩到 0.25 m 不会消除 254 footprint 碰撞，反而会让全局路径更愿意贴角。

本轮改的是 LuckPlanner 的路径选择而不是几何安全半径：`cost_penalty` 从 3.0
增至 5.0，促使 A* 在可通行走廊内更偏向中部；直视化简
`smoothing_max_cost` 从 180 收紧到 100，禁止长线段穿过高软代价拐角内侧。
`max_planning_cost=252` 不变，所以真正狭窄但中心仍可通行的 A* 栅格路径不会
被一刀切禁止，只是不再被激进拉直。

## 5. 当前真正生效的最小架构

只有启动命令显式带上 `navigation_profile:=luck_full` 时，单点导航才走完整的
自定义链：

```text
RViz NavigateToPose
  -> navigate_to_pose_luck_full.xml
  -> LuckGrid       (luck_planner.cpp/.hpp，生成全局路径)
  -> LuckFollowPath (luck_controller.cpp/.hpp，10 Hz 生成 cmd_vel_nav)
  -> velocity_smoother -> Collision Monitor -> chassis_driver
```

不写 `navigation_profile` 时默认是 `stable`，使用 SmacPlanner2D + 官方 RPP，
即使 YAML 同时加载了 Luck 插件，也不会被行为树选中。可以在运行时通过默认
行为树路径和启动日志确认，不能仅凭“插件已编译/已加载”判断。

核心算法确实集中在两个 CPP/HPP，但仍必须有三类胶水配置：

- `nav2_params.yaml`：插件 ID、控制频率、代价地图、Goal/Progress Checker 和
  算法参数；
- `navigate_to_pose_luck_full.xml`：决定何时规划、跟踪、清图、等待和重试；
- launch：只选择配置档和行为树绝对路径，不应放运动策略。

当前自定义行为树是我们改过的“安静恢复树”，没有 Spin/BackUp。早期看到的
突然自转/倒车不是行为树恐慌，而是 FollowPath 因空 Goal Checker 名称立即失败
后进入官方恢复动作；该路由错误已经修复。V11.5 仍不恢复盲目倒车：先让控制器
用完整 footprint 校验的短时原地松角；失败后树会清图并重新运行 LuckGrid。
若实车证明仅转向不足，再单独加入同样经过 footprint 预测、距离受限的后退候选，
不能在这次同时叠加，以免无法判断是哪一项改善或恶化。

重新规划并非“调用一次就必然换路”。LuckGrid 是确定性的；机器人未移动、障碍
地图未变化、代价参数未变化时，输入相同就会得到相同路径。本轮从三处打破死循环：
先避免生成贴角捷径，其次让机器人安全改变朝向以恢复正向弧线，最后用严格计时
把仍然无解的情况交回行为树。以后若仍存在“确有另一条路但反复选择同一死路”，
才需要增加临时禁行区/失败走廊记忆，而不是继续降低膨胀半径。

## 6. 部署与单场景验证

关闭旧导航和诊断终端后，在 Jetson 执行：

```bash
cd ~/ros2_ws
sha256sum luckrobot_custom_nav_dynamic_tracking_v11_5_20260908.zip
unzip -o luckrobot_custom_nav_dynamic_tracking_v11_5_20260908.zip -d ~/ros2_ws
sha256sum -c docs/CUSTOM_NAV_DYNAMIC_TRACKING_V11_5_MANIFEST.sha256

source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select luckrobot_nav2_plugins my_robot_navigation my_robot_bringup \
  --allow-overriding luckrobot_nav2_plugins my_robot_navigation my_robot_bringup
source ~/ros2_ws/install/setup.bash

ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping navigation_profile:=luck_full
```

另开终端确认参数并录包：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 param get /controller_server LuckFollowPath.path_heading_filter_tau
ros2 param get /controller_server LuckFollowPath.path_heading_correction_limit
ros2 param get /controller_server LuckFollowPath.in_place_avoidance_timeout
ros2 param get /planner_server LuckGrid.smoothing_max_cost
ros2 param get /planner_server LuckGrid.cost_penalty

bash ~/ros2_ws/tools/run_custom_nav_diag.sh 150
```

看到 `[5/5] rosbag 已启动` 后，只发送一个能够复现原拐角的目标，不启动自动
探索、不临时改膨胀半径。若该路径当前被人或物体完全堵死，允许机器人短时原地
松角、失败并安静重规划；不允许无限转、不允许持续重复立即失败。测试结束按
Ctrl-C，上传新的 `~/custom_nav_*.tar.gz`。

验收重点为：行驶中前视点误差与角速度不再频繁相反；稳定角速度换向显著少于
旧包的 16 次；拐角能够恢复前进，或在有界时间内明确失败重规划；Collision
Monitor 和 0.175 m 车体半径保持不变。先完成这一项，再调整探索算法。
