# LuckRobot V11.7.4：执行链停滞看门狗、激光滤波节点与诊断门禁修复

## 1. 本轮实车结论

V11.6 的两份有效 rosbag 都是成功样本，并没有包含用户最后报告的“不动”
时段：

- `custom_nav_20260908_150516` 在 15:05:50--15:07:28 记录。三层速度
  `/cmd_vel_nav -> /cmd_vel -> /cmd_vel_safe` 完整一致，碰撞监视器没有修改
  774 组配对命令；轮式里程计实际累计 4.177 m。
- `custom_nav_20260908_150835` 在 15:08:53--15:12:21 记录。碰撞监视器只
  修改 1741 组配对命令中的 23 组；轮式里程计实际累计 8.759 m。前两个
  长目标均到达，最后一个目标也在 15:12:17 成功。
- 两包中把 `/cmd_vel_safe` 与双电机 RPM 按时间对齐后，分别只有 1/2343、
  4/5300 个瞬时样本出现“命令非零而轮速近零”，没有任何持续 1 秒的执行
  停滞。因此 V11.6 的正常输出、转角松角和底盘执行链已通过本轮测试，
  本版本不再修改速度、Pure Pursuit 曲率、膨胀半径或 Collision Monitor。

真正异常的长返回目标在 15:12:36 才发送，比第二包结束晚约 15 秒。终端记录
显示机器人位置从 `(-4.92,-0.88)` 出发后约 97 秒没有变化，期间仍每秒收到新
路径；15:13:08 换成近目标后仍无位移。日志没有出现“所有局部轨迹被挡”的
控制器告警。因此现有证据不能把它归因于规划路径或 V11.6 拐角碰撞，也不能
在没有故障期 `/cmd_vel_safe` 和电机 RPM 的情况下武断判定是哪一级归零。

同时，启动门禁明确发现了两个完全同名的 `/scan_to_scan_filter_chain`。一个
`smart_navigation.launch.py` 只会启动一个该节点，所以这是旧 launch 遗留的
孤儿进程。重复传感器节点会造成数据来源不唯一、SLAM 队列满、EKF/ROS 服务
超时，叠加启动不能作为有效测试条件。

## 2. V11.7.3 的最小修改

### 2.1 不改变正常运动的执行停滞看门狗

LuckController 新增独立的命令--里程计看门狗：

- 控制器输出 `|v| >= 0.03 m/s` 或 `|w| >= 0.10 rad/s`；
- `/odometry/filtered` 同时保持 `|v| < 0.01 m/s` 且
  `|w| < 0.03 rad/s`；
- 平移或混合运动连续 3.0 秒；纯原地转向连续 6.0 秒。

满足时，本次 FollowPath 会明确失败并交给已有的“清局部图、等待、全局
重规划”行为树，不再无限显示 `Passing new path to controller`。普通 0.5 Hz
同目标重规划不会清零这个计时器；行为树真正启动新的 FollowPath 时才给一次
新的有限尝试。该逻辑只观察故障，不改任何正常速度指令。

V11.7 实车日志中，首个目标先持续输出 `v=0,w=-0.25` 的原地转向，融合里程计
短时间仍为零，3 秒窗口触发了看门狗；随后后续目标又能正常运动。这种现象既
可能是启动时 EKF 延迟，也可能是转向执行链真正没有响应。V11.7.1 为纯转向增加
`execution_rotation_stall_timeout=6.0`，只消除这个已确认的启动误判风险，不关闭
看门狗；超过 6 秒仍没有角速度反馈时仍会触发安静重规划。

如果日志出现：

```text
LuckController commanded motion but odometry remained still ...
```

说明“控制器有非零意图，但执行反馈静止”。随后 rosbag 分析会继续用
`cmd_vel_safe` 与电机 RPM 区分：

- `cmd_vel_safe=0`：Collision Monitor/速度链上游制动；
- `cmd_vel_safe!=0` 且 RPM=0：底盘串口、电机驱动或物理执行层；
- RPM!=0 但地图位姿不动：车轮打滑或定位/TF 链问题。

### 2.2 降低无收益的全局路径替换

有效包中出现 3 次 FollowPath action 应答超时、1 次规划 action 应答超时，
并伴随地图扩容、EKF 超期和 SLAM 队列满。自定义控制器和局部避障仍以 10 Hz
运行，因此把全局重规划由 1.0 Hz 调到 0.5 Hz：动态障碍仍由局部层立即处理，
全局层每 2 秒更新一次，减少对一个正在成功执行的路径反复抢占。

### 2.3 拒绝重复节点并支持故障前一直录包

- `check_custom_nav_startup.sh` 和 `run_custom_nav_diag.sh` 发现完全同名节点时
  直接失败，不再把污染的 ROS 图判为 PASS。
- 新增 `stop_navigation_stack.sh`，只以 SIGINT 停止本机器人导航栈和可能遗留
  的子进程，保留节点正常制动、释放串口的机会。V11.7.2 在 SIGINT 后会列出
  仍存活的精确进程，只对本机器人栈发送 SIGTERM，并刷新 ROS CLI daemon；
  不再让孤立的激光滤波器污染下一次启动。
- `run_custom_nav_diag.sh 0` 会一直记录到 Ctrl+C；结束时自动补记最终 TF 并
  打包。分析器新增安全速度与双轮 RPM 的逐帧执行检查。

### 2.4 修复 Humble laser_filters 的同名内部节点

一次完全清理后的单次 launch 显示两个预期进程：主雷达过滤器和相机伪雷达
过滤器；但是主雷达的 `LaserScanBoxFilter` 还会在进程内创建辅助节点。Humble
版本会把 launch 传入的全局 `__node` 重映射同时施加给主节点和辅助节点，产生
两个 `/scan_to_scan_filter_chain`，并输出 `Publisher already registered`。

V11.7.3 从 `test_bringup.launch.py` 和 `robot_bringup.launch.py` 的主雷达过滤
Node 中移除显式 `name`。可执行文件的默认名称本来就是
`scan_to_scan_filter_chain`，仍能匹配 `box_filter.yaml`；相机过滤器继续使用
`camera_scan_filter_chain`。实机端点已证明 `/scan_filtered` 原本只有一个发布者，
所以此修改不是改变滤波算法或话题，而是消除同名节点、参数服务歧义和门禁误报。

### 2.5 V11.7.4 修正生命周期服务超时误判

V11.7.3 实车单次干净启动后，ROS 节点列表中只剩一个
`/scan_to_scan_filter_chain`，内部辅助节点已正确显示为
`/laser_scan_box_filter`，证明同名节点问题已经解决。生命周期管理器日志也明确
显示 `planner_server` 已激活、LuckGrid 已激活并完成 bond；但 Jetson 负载较高时，
独立执行 `ros2 lifecycle get` 仍可能收不到 `/get_state` 应答。旧门禁把连续三次
服务应答超时当成节点失活，因此给出假 `FAIL`。

V11.7.4 只修改两个诊断脚本：节点缺失、同名重复、明确返回
`inactive/unconfigured/finalized` 仍然失败；节点仍在 ROS 图中但生命周期服务暂时
无响应时返回 `PASS_WITH_WARNINGS`，并允许录包继续捕获故障。该修改不涉及
LuckPlanner、LuckController、Nav2 参数、代价地图、Collision Monitor、传感器
滤波或底盘驱动，对运动输出没有直接影响。

## 3. 下一轮唯一测试流程

关闭旧终端后，在终端 1 执行：

```bash
cd ~/ros2_ws
bash tools/stop_navigation_stack.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select luckrobot_nav2_plugins my_robot_navigation my_robot_bringup \
  --allow-overriding luckrobot_nav2_plugins my_robot_navigation my_robot_bringup
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping navigation_profile:=luck_full
```

导航全部 active 后，终端 2 执行门禁：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/check_custom_nav_startup.sh 40
```

`RESULT=PASS` 或 `RESULT=PASS_WITH_WARNINGS` 都可进入录包；后者表示 ROS 服务
响应偏慢，不等于节点失活。终端 2 随后执行：

```bash
bash ~/ros2_ws/tools/run_custom_nav_diag.sh 0
```

按顺序发送一个已验证的转角目标和一个长返回目标，每个目标等到成功或明确
失败后再发下一个。若再次不动，保持录包至少 10 秒；再发一个附近安全目标，
再等 10 秒，然后在诊断终端 Ctrl+C。不要先重启 launch，否则会丢失故障现场。

## 4. 本轮不做的修改

- 不缩小 0.30/0.25 m 的全局/局部膨胀半径；本轮成功包已通过同一拐角。
- 不增加倒车或盲目 Spin；未知故障时自动运动会降低安全性。
- 不继续改正常转向增益；用户反馈 V11.6 已基本达到理想状态。
- 不依据故障发生前结束的包宣称已定位到底盘或 Collision Monitor。V11.7
  先保证异常能自行退出当前 FollowPath，并保证下一包包含决定性证据。
