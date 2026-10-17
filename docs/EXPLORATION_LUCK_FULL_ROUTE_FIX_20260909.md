# 自动探索 Luck Full 路由修复（2026-09-09）

## 结论

`auto_diag_20260909_091555` 证明机器人不动不是前沿点选择错误，也不是
底盘忽略了正常运动指令。自动探索专用行为树仍显式调用旧的
`GridBased` 规划器和 `FollowPath`（RPP）控制器，绕过了已经完成实车验证的
`LuckGrid` 和 `LuckFollowPath`。

134.3 秒记录中共有 8 个导航目标，`/cmd_vel_nav` 的线速度始终为 0，角速度
始终约为 0.02 rad/s；轮速反馈为 0，地图新增已知面积为 0。LuckController
调试话题没有任何样本，进一步证明自定义控制器没有被调用。探索器每 15 秒
取消一次目标以及 BT 的 `Failed to get result ... in node halt` 均为无运动后的
结果，不是根因。

旧 RPP 在原地对准阶段根据近零里程计速度和 0.20 rad/s² 角加速度限制，每个
周期反复给出约 0.02 rad/s。该值低于实车可用转速区，底盘不转，反馈继续为
零，形成闭锁。

同一记录还有第二个独立安全问题：`/cmd_vel` 有 1257 帧，而
`/cmd_vel_safe` 为 0 帧，表明当次 Collision Monitor 没有实际转发控制指令。
生命周期服务超时不能替代端到端链路验证。

## 修改

1. 探索专用行为树改为 `LuckGrid + LuckFollowPath + explore_goal_checker`。
2. 保留“失败立即返回 explore_lite”的无恢复探索树，不改动已经冻结的 V11.8
   控制器参数，也不修改 V10.6 的前沿/DFS 选择参数。
3. 自动诊断脚本在放行探索前检查安装区行为树路由。
4. 自动诊断脚本向 `/cmd_vel` 发送一帧全零 Twist，并要求在
   `/cmd_vel_safe` 收到一帧输出。该检查不会驱动车轮；若安全链未工作则拒绝
   启动探索。

## 测试顺序

关闭全部旧进程后，只启动一套 `luck_full` 导航，再运行自动诊断。第一轮只测
120 秒并让人在车旁监护。通过标准是：

- 诊断预检显示 `/cmd_vel -> /cmd_vel_safe zero-command probe: PASS`；
- 探索开始后 `/luck_controller/tracking_error` 有数据；
- 起步指令不再长期固定为 `v=0, |w|=0.02`；
- 小车实际移动，地图新增已知面积大于 0；
- 若目标不可达，探索器可以取消并切换目标，但不应连续 8 次原地超时。

这轮只验证插件路由和安全链。确认机器人能稳定执行探索目标后，再依据新包
评估中心起步的往复、分支回退和完成判断，避免把控制链故障误当作探索决策
问题继续调参。

## VSLAM 当前状态

环境检查只做了审计，没有安装 cuVSLAM/Isaac ROS。Jetson 当前为 L4T R36.4.3、
CUDA 12.6、ROS 2 Humble，内存和系统盘空间足够；Docker、NVIDIA Container
Runtime 和 Isaac ROS 包均未安装。双目输入预检已经通过。导航与探索链稳定后
再部署 VSLAM，避免同时引入 GPU/容器负载和新的 TF/里程计变量。
