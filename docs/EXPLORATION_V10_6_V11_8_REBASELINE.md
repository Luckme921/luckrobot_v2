# V10.6 探索器与 V11.8 导航重新基线测试

## 结论边界

`custom_nav_20260908_223625` 的单拐角任务成功；
`custom_nav_20260908_224918` 中前四个完整导航任务均到达。成功任务的最终位置误差为
约 0.048～0.080 m，路径横向误差 P90 约 0.009～0.028 m。控制器停止后没有出现
“越过目标再返回”的距离反弹，也没有超过 1 秒的底盘执行停滞。

因此 V11.8 保持冻结，不根据 RViz 的轻微视觉变化继续调整速度、制动、膨胀半径或
底盘参数。

旧的自动探索数据主要使用旧控制器，中心起步时的长期停留混合了目标选择问题与旧
控制器拐角问题。下一轮只组合当前 `Observable-UNKNOWN ... V10.6` 与
`LuckGrid + LuckFollowPath V11.8`，不先修改探索评分。

## 自动探索单变量测试

1. 将机器人放在房间中部、周围至少保留安全转身空间；终端 1 使用
   `navigation_profile:=luck_full`。
2. 不从 RViz 发送手动目标，不运行第二个 `explore_node`。
3. 执行 `bash ~/ros2_ws/tools/run_auto_diag.sh 300`。脚本自己启动和停止
   `auto_explore.launch.py`。
4. 完整运行 5 分钟；只有碰撞风险时才急停。记录机器人是否重复返回已无灰色可见区、
   是否在目标交接时明显停顿，以及是否正常发布 `EXPLORATION_COMPLETE`。
5. 上传最新的 `~/auto_diag_*.tar.gz`。根据这一包再决定 V10.7 的目标连续交接、
   宏观分支评分和完成判断，避免同时改多个变量。

新版 `run_auto_diag.sh` 对生命周期服务偶发超时采用与手动诊断相同的规则：节点在
ROS 图中且没有明确返回 inactive/unconfigured 时继续录包；发现同名重复节点、缺少
必要节点或明确非 active 状态仍会拒绝启动。它新增记录轮速反馈、LuckController
前视点/候选轨迹/跟踪误差和行为树事件。

## VSLAM 并行准备边界

已完成的双目 640x400@30 Hz 同步预检是输入层 PASS，不代表 cuVSLAM 已安装。
先在关闭 Nav2、自动探索和机器人运动后执行：

```bash
bash ~/ros2_ws/tools/check_vslam_environment.sh
```

脚本只读取 JetPack、CUDA、Docker、磁盘、内存和 Isaac ROS 包状态，不安装软件。
上传 `~/vslam_environment_*.tar.gz` 后，再决定 Isaac ROS release-3.2 容器部署；正式
VSLAM 首先旁路运行，不发布 TF、不接管主 EKF，也不与自动探索同时测试。
