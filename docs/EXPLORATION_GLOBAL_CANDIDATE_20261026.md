# 全局区域探索：已接入的候选后端

本文件更新 `EXPLORATION_GLOBAL_ARCHITECTURE_20261026.md` 中“仅离线核心”的状态。
现在 C++ explore_lite 已接入候选后端；不是让 Python 原型接管运动。
默认保留 legacy，显式 `global_regions:=true` 才启用新后端。

## 实际实现

- 从全局 /map 的边界建立地图坐标区域身份，长边界按空间跨度分段（1.5 m）。
- 保留区域身份跨越短目标；边界分裂建立父子关系；优先子区域、当前区域、祖先
  的其他候选，最后按当前已知路径代价选其他根。车头方向不决定区域是否存在。
- 区域消失后，记录的全部边界证据必须在不同完整地图更新中连续三次变成已知，
  才标为 covered。无候选、预检失败、导航中止均不构成覆盖。
- 每个边界最多保留 8 个间隔 >=0.25 m 的观察点；预检失败暂缓该观察点 20 s，
  再试其他点。到达最终观察点也等待该视点的数据整合，不直接宣布区域完成。
- 全局边界附属未知面积的有界扩展用于筛掉小碎片，不再采用机器人当前位置的
  圆锥来否决远处分支。该面积是潜在未知边缘面积估计，不是未知房间真实面积。
- 已知路径长度 + 接近边界代价用于选点，避免只选最近的已知落脚点。
- LuckGrid 预检和 LuckFollowPath 执行仍沿用；每段 <=0.80 m，软膨胀区脱困
  仍 <=0.30 m，终点成本 <=20、路径成本 <=80、终点间隙 0.25 m 不变。
- 新后端额外检查路径前缀中心必须属于 /map 已知空闲格。
- 新后端暂不启用运动中滚动交接，等待当前短段结果，避免并发区域目标混用。
- 持续无安全候选 62.5 s 暂停；120 s 空闲面积无 >=0.25 m² 增长暂停并明确
  标为可能无效往返。超过 512 个区域身份暂停而不静默丢弃分支。
- 原运动控制、速度、传感器滤波和动态障碍清理不变。

## 部署与验证范围

源代码路径：
- src/m-explore-ros2/explore/include/explore/global_regions.hpp
- src/m-explore-ros2/explore/src/explore.cpp
- src/m-explore-ros2/explore/src/costmap_client.cpp（完整地图更新序号）
- src/my_robot_mapping/launch/auto_explore.launch.py
- tools/run_auto_diag.sh

测试/回放：
- tools/test_global_regions.cpp：身份保持、短步保留、暂时阻塞、覆盖证据、重新
  出现、边界分裂及子区域优先。
- tools/test_exploration_region_policy.py：原 Python 策略原型六项测试。
- tools/replay_global_exploration.py：仅在 ROS_DOMAIN_ID=193、localhost 模式运行。
  只回放 /map、代价地图、TF、时钟；不回放任何速度命令。动作桩拒绝所有请求。
  默认 shadow_only 模式不得发送动作；verify-rejection 模式只允许预检请求被拒绝，
  不应产生导航目标。日志/summary.json 保存到指定目录。

历史轨迹回放只说明候选选点/回调可以运行，不能证明新路线的闭环覆盖率，
也不能代表导航、深度、语义推理同时运行的资源消耗。尚无新后端实车记录。

最终版验证记录：
- `~/global_shadow_103651_final/summary.json`：170 次选点，0 次规划请求、0 次导航请求。
- `~/global_rejection_101050/summary.json`：模拟拒绝 69 次规划请求，0 次导航请求；
  28 次明确 pending 报告，没有把这些拒绝标成探索完成。
- 本地副本：`diagnostics/global_exploration_20261026/`。
- 七个部署源文件 SHA256 已逐个核对本地与 Jetson 相同；最终 C++ 包编译通过。

## 重要限制

这是 FUEL/TARE 分层思想的轻量适配候选，不是完整移植或论文性能复现。
空间分段/重叠关联是入口区域近似，不是完成了房间语义拓扑；大回环修正后的
身份关联和相邻入口合并仍需要验证。还未运行跨区域 TSP，全局最优不能承诺。
全局图上的最短路径必须由已知空间连通性/规划器提供，不能跨未知区域取直线。
本版没有放宽碰撞条件来换取探索率。

## 用户运行

先正常启动原有 mapping + luck_full 导航。另一个终端：

```bash
source ~/ros2_ws/tools/robot_env.sh
bash ~/ros2_ws/tools/run_auto_diag.sh 180 global
```

必须看到“探索后端：global”。录包脚本会启动探索，不要另外启动 explore 或发送
RViz 目标。停止一次 Ctrl+C 后等待打包。
原版对照：`bash ~/ros2_ws/tools/run_auto_diag.sh 180 legacy`。
脚本不写第二参数仍是 legacy，避免改变用户已有默认行为。

升级前备份：~/explore_before_global_regions.cpp、~/explore_before_global_regions.h。
部署日志：~/global_regions_build.log。
