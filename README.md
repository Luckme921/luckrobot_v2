# LuckRobot V2

> 2026-09-29：V12.13.1 保留低位二维雷达/高度相机独立 Nav2 障碍层，修正
> 周期复用路线时反复插入“当前位置连接段”造成的左右修正，并把急弯最短前视
> 由 0.12 m 温和放宽为 0.16 m。322 s 实车包证明相机点在静态地图空闲区中
> 仍有 99.9% 被全局代价地图标记为致命障碍；它不会也不应写黑到 SLAM `/map`。
>
> 2026-09-29：V12.13 将低位二维雷达和高度相机拆成两个独立 Nav2 障碍层，
> 再由主代价地图按最大代价融合；低位雷达的清除射线不再覆盖只能由相机看见的
> 桌沿/横梁。同时 LuckGrid 对同一目标保留仍然安全的既有路线，禁止仅因相机
> 暂时离开视场就从安全 A 路切回更短的 B/C 路。新障碍仍会立即判废旧路线，
> `/scan_camera_safety` 即时制动链保持不变。
>
> 2026-09-22：V12.10.9 根据 300 秒动态包修复仍能进入规划层的持续窄假点。
> 即时安全流由 4 束提高为 6 束且相邻量程容差收紧到 0.08 m；规划确认提高为
> 5 帧，并在量程跳变处分割角度簇，防止“4束近距假点 + 远处墙面回波”被拼成
> 满足7束门槛的伪障碍。局部/全局6/16秒记忆保持不变，只记忆通过新确认的点。
>
> 2026-09-22：V12.10.8 根据动态绕障实测，将已确认相机障碍的局部/全局世界
> 坐标记忆由 3/8 秒加倍为 6/16 秒。机器人转向导致高位障碍离开前向视场后，
> 记忆覆盖完整绕行过程，避免车体尚未越过障碍就重新选择穿越路线。移动物体移开
> 后最多产生有限的保守绕行，不会永久写入静态地图。
>
> 2026-09-22：V12.10.7.1 冻结深度避障静态验收基线：最近表面支持数 5、
> 高度范围 0.12～0.50 m。空场规划流最近点 p05 为 0.843 m，原 0.472 m
> 回波只偶尔留在即时安全流；真实近障碍仍有 8～12 个支持点并稳定进入规划流。
> 高度跨越约 0.11 m 扫描平面的低位障碍继续由主 2D 雷达覆盖，下一阶段进入
> 分场景动态导航验收。
>
> 2026-09-22：V12.10.7 根据空旷/真实平板 A/B 实测，将最近表面支持阈值由
> 3 提高到 5。空旷假点稳定约 4 个同深度样本，真实近障碍约 48 个，具备明显
> 区分度。RViz 同时显示青色即时安全流 `/scan_camera_safety` 与紫色规划确认流
> `/scan_camera`，避免把只在安全流闪烁的点误认为已经进入代价地图。
>
> 2026-09-22：V12.10.6 修复持续近距飞点。旧高度投影按角度束统计所有深度点，
> 因此“1个近距飞点 + 多个远处墙面点”也能满足最小点数，而最近值会错误成为障碍。
> 现在每束只统计距最近表面3cm以内的支持点，至少3个同深度样本才发布；LaserScan
> intensity 同时携带支持数用于诊断。4帧世界坐标确认继续处理短时噪声。
>
> 2026-09-22：V12.10.5.2 修复确认扫描“有 15 Hz 但 RViz 无点”。Gemini 图像
> 时间戳可领先最新 `odom -> base_footprint` TF，精确时刻查询大量触发 future
> extrapolation，旧节点因而发布空扫描；现在优先精确 TF，失败时使用最新可用 TF。
> `/scan_camera` 发布 QoS 同时改为 Reliable，兼容现有 RViz 显示，并继续兼容
> Nav2 的 Best Effort 订阅。4 帧/0.50 秒确认阈值保持不变。
>
> 2026-09-22：V12.10.5.1 修复 `camera_scan_confirmation` 启动即退出：Humble
> `rclpy` 日志接口不接受 C 风格的额外位置参数，旧写法导致节点构造阶段抛出
> `TypeError`，表现为 `/scan_camera_safety` 正常而 `/scan_camera` 发布者为 0。
> 同时将规划确认提高为 4 帧/0.50 秒；安全制动流仍为单帧空间过滤。
>
> 2026-09-22：V12.10.5 将高度深度扫描拆成两路。`/scan_camera_safety`
> 经轻量空间滤波后直接供 Collision Monitor 使用；`/scan_camera` 只发布在
> `odom` 坐标中经过车体运动补偿、连续 4 帧确认且具有足够横向宽度的回波，供
> 局部/全局代价地图使用。这样不会把单帧 RGB-D 飞点用 3/8 秒观测记忆放大，
> 同时不牺牲真实近障碍物的即时制动能力。
>
> 2026-09-22：V12.10.4 为前向深度相机障碍增加有限世界坐标记忆：局部 3 秒、
> 全局 8 秒。机器人转向绕行导致障碍暂时离开相机视场时，不再立刻忘记障碍并
> 反复切回碰撞最短路；动态障碍移走后记忆会自动过期。
>
> 2026-09-22：V12.10.3 将高度感知 `/scan_camera` 以受限的 1.5 m 障碍
> 范围加入全局代价地图。局部控制器先处理新出现的高位障碍，随后 LuckGrid
> 根据动态更新的全局代价地图替换主路径；完整相机点云保持关闭以控制 Jetson
> 的计算与 DDS 带宽开销。

## VSLAM 长时动态验收结论（2026-09-21）

Gemini 336L 被动双目输入已达到约 30 Hz、单调时间戳和硬件同步要求，但 535 秒
实车动态包中原始 cuVSLAM 路径约为轮速路径的 4.26 倍，出现 126 m/s 非物理跳变，
健康门控仅 32.2% 时间为真。当前 VSLAM 因此不接入生产 EKF、Nav2 或主 TF，保留为
默认关闭的研究旁路。现有轮速 + IMU + 2D SLAM 导航基线继续作为生产方案；详见
[VSLAM 长时动态实车验收结论](docs/VSLAM_FIELD_VERDICT_20260921.md)。

## VSLAM 双目单调门控 V12.7.2（2026-09-15）

Orbbec Laser On-Off 交替模式在实机上仍以约 30 Hz 发布左右 IR，但约一半消息重复
上一有效帧时间戳，导致 cuVSLAM 只看到 15 Hz 有效帧、健康门控始终拒绝。新增
`vslam_stereo_monotonic_gate`，只转发严格同步且时间戳递增的左右图像；普通双目
30 Hz 透传，交替双目输出干净的约 15 Hz。生产 Nav2、EKF、深度避障和 TF 所有权
保持不变，VSLAM 候选 EKF 仍禁止启动，直到动态验收通过。

V12.7.2 修复 Humble `rclpy.Node.subscriptions` 为只读属性导致门控节点启动退出的
问题。该故障只造成 `/vslam/stereo/...` 零样本，没有修改原始相机话题、生产 EKF、
Nav2 或底盘控制。

LuckRobot V2 是一台基于 Jetson Orin Nano 8GB、ROS 2 Humble、A2M12 2D 激光雷达、Orbbec Gemini 336L RGB-D 相机、双轮编码器和 IMU 的室内差速机器人。

V12.7 对 VSLAM 仍采用旁路动态验收：Gemini 使用 Laser On-Off 交替帧，激光帧
保留给深度避障，无激光同步双目帧送入 cuVSLAM。生产 EKF 与 Nav2 暂不接收视觉
里程计；详见 `docs/VSLAM_INTERLEAVED_CAMERA_V12_7_20260915.md`。

当前仓库的优先目标是先建立一个可重复、可验收、具备安全兜底的导航底座，再单独接入 Isaac ROS cuVSLAM。Jetson Orin Nano Super 8GB 本阶段不部署 LocateAnything 或其他大视觉模型，也不把 VSLAM 直接接入主 TF 链。

2026-09-14 的 V12.5 修复确认：启用双目 IR 时 Gemini 336L 的深度必须同步为
640x400@30 Hz；混用深度 15 Hz 会让驱动反复重置，导致 `/scan_camera` 和原始
cuVSLAM 同时断流。相机升高后深度伪激光默认取 60 行以覆盖高位桌沿，并且自动
探索诊断会先验证主雷达、相机扫描及（若启用）原始 VSLAM 都有实时消息。
VSLAM 仍是候选旁路，未通过多场景健康门控前不会替换生产 EKF。

V12.6 根据 300 秒实车包修复了候选融合的安全缺口：cuVSLAM 在约第 59 秒
发生 30.57 m 单帧跳变，而旧候选 EKF 因原始 Odometry 协方差为零仍会错误接纳。
新增 `vslam_odom_guard` 同步独立的 `vo_pose_covariance`，检查协方差、物理
连续性及主里程计一致性；失锁时暂停视觉融合，连续 60 帧健康后重新锚定。
故障包离线重放后受保护输出最大连续步长为 0.0234 m。候选仍保持
`publish_tf: false`，不会影响现有 Nav2。详见
[V12.6 VSLAM 健康门控](docs/VSLAM_HEALTH_GUARD_V12_6_20260914.md)。

## 当前架构

```text
/wheel/odom --(Vx、当前配置也融合 Vyaw)--+
                                             +--> robot_localization --> odom -> base_footprint
/imu/data -------------------------(Vyaw)----+

A2M12 /scan --> BoxFilter --> /scan_filtered --> slam_toolbox + Nav2 + Collision Monitor
Gemini depth --> height-aware projection --> light filter --> /scan_camera_safety --> Collision Monitor
                                                     +--> world-frame 4-frame confirmation --> /scan_camera --> Nav2 costmaps

slam_toolbox --> map -> odom
Nav2 Controller /cmd_vel_nav --> velocity_smoother --> /cmd_vel --> Collision Monitor --> /cmd_vel_safe --> chassis_driver
Nav2 Humble Behavior Server (Spin/BackUp 等) --> /cmd_vel --> Collision Monitor --> /cmd_vel_safe --> chassis_driver
```

Humble 官方 `navigation_launch.py` 不会把 `behavior_server` 重映射到
`/cmd_vel_nav`。恢复行为使用自身的速度/加速度约束并直接进入碰撞监控；
不要把 `/cmd_vel_nav` 在 Spin 测试期间没有数据误判为丢消息。

注意：交接说明声称 EKF 的轮式输入只使用 `Vx`，但仓库当前的 `ekf.yaml` 实际同时启用了轮式 `Vyaw`。这是需要用实车 bag 数据做 A/B 验证的物理参数，本轮没有擅自改变。

## Jetson 构建

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

首次使用近场安全层时确认已安装：

```bash
sudo apt update
sudo apt install ros-humble-nav2-collision-monitor
```

## 推荐启动方式

新建或继续开荒地图：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py mode:=mapping
```

另开终端启动前沿探索：

```bash
ros2 launch my_robot_mapping auto_explore.launch.py
```

当前 V10 探索器不会把灰色未知区的前沿质心直接发给 Nav2。它先对每条
前沿后方的 UNKNOWN 栅格做 1.5 m 有界连通洪泛，用真实待探索面积识别
大房间/宽通道并过滤小于 0.20 m² 的碎角；随后在 `/map` 白色已知区建立
拓扑 DFS，沿选中的 UNKNOWN basin 以最长 `0.85 m` 的安全局部目标推进。
侧分支的白区父分叉进入 LIFO 栈，当前 basin 确认结束后才切换分支。切换时
由 Nav2 从当前位置走最短安全路线，不再机械重放旧分叉坐标。每个
实际目标仍通过 Nav2 全局路径、实时代价和车体余量检查，灰区只提供信息，
绝不是运动终点。

V10.3 根据 `auto_diag_20260831_221003` 修复了一个明确的软膨胀自锁：
目标本身代价为 9，但动态路径跟踪中止后机器人停在代价 67，旧逻辑会把
所有大于 40 的软代价格都当成碰撞并原地等待。现在 41～98 只允许一个
最长 0.45 m、终点代价不高于 20 的规划脱困步；99/100 仍硬停止。窄通道
实车 A/B 采用全局/局部 0.30/0.25 m 的软膨胀半径，车体 0.175 m 和
Collision Monitor 0.19 m 均未缩小。详细证据见
[V10.3 软膨胀脱困说明](docs/SOFT_INFLATION_ESCAPE_V10_3.md)。

V10.6 根据 2026-09-07 新补充的“中间出发”和“一角出发”两次 300 秒录包，
只把可达白色观察点前方、且没有被墙/障碍遮挡的 UNKNOWN 栅格计为探索收益，
过滤封闭墙外灰区和已经完成分叉的低价值残片。目标几何余量提高到 `0.25 m`，
软代价脱困步缩短到 `0.30 m`；机器人若在代价 `>=99` 停留 15 秒则安全暂停，
不再出现探索节点运行但 180 秒不发目标。实测起步残余航向误差约 27～33°，
因此本版受控地把 RPP 对准阈值收紧为 `0.22 rad`，但不改速度、膨胀层、底盘
和 Collision Monitor。详见
[V10.6 可观测未知区与硬代价安全暂停](docs/OBSERVABLE_UNKNOWN_DFS_V10_6.md)。

V10.7 根据 `auto_diag_20260909_123011` 增加受约束的滚动目标接力：同一 UNKNOWN
分支内，当机器人距离当前安全白色落脚点不超过 0.45 m，探索器会先用 `LuckGrid`
验证下一段，再更新 Nav2 目标，使直行探索不必每 0.6～1.2 m 完全停稳。跨分支和
DFS 回退仍等待当前 action 完成，接力失败也不会取消原目标。完成区过滤、兄弟分支
回退以及“栈清空后连续 15 秒无候选即结束”保持不变。同时，LuckController 的执行
看门狗增加 `/wheel/odom` 旁路确认，避免 IMU/EKF 短时延迟把真实原地转动误判成
底盘卡死；实际控制仍只使用 `/odometry/filtered`。详见
[V10.7 连续滚动目标与执行反馈看门狗](docs/ROLLING_HORIZON_EXPLORATION_V10_7_20260909.md)。

V10.5 根据 2026-09-07 的“中间出发”和“一角出发”两次 300 秒录包，
修正同一走廊沿途反复写入伪分叉的问题。活动分支至少前进 `1.20 m` 才允许
新增一层宏观分叉；恢复记忆分支时，当前白区路径必须经过旧锚点并沿原出口
方向离开。无进展区域按 `0.85 m / 60 s` 冷却，普通规划安全段由 `0.85 m`
增至 `1.20 m`，减少完整减速/再启动次数；Nav2、底盘和安全链参数保持稳定
基线不变。详见
[V10.5 宏观分叉约束与长步连续探索](docs/MACRO_JUNCTION_DFS_V10_5.md)。

现有两次录包还完成了传感器时间与 TF 审计：TF 树唯一且连通，所有消息头单调；
相机/雷达扫描最近时间差中位约 17 ms，当前低速 2D 导航可用，但物理外参仍需在
最终机构上实测。VSLAM 相机建议光心位于 0.35～0.45 m；现有水平夹具预计约
0.369 m，可以采用。升高后必须重做外参，并把中央 10 行深度伪雷达升级为点云高度带避障，
不能只改 URDF。详见
[传感器对齐现状与 VSLAM 相机安装建议](docs/SENSOR_ALIGNMENT_AND_VSLAM_MOUNTING.md)。

自定义 Nav2 插件 V11.0 的两次实车包证明直线跟踪横向误差较小，但发现控制器
在 0.12 m 提前停止平移、Goal Checker 要求进入 0.08 m 的确定性死锁。V11.1
改为运行时读取所选 Goal Checker 的实际容差，统一终点阈值，并移除与
`velocity_smoother` 串联的第二重角加速度限幅；本轮仍只测试
`SmacPlanner2D + LuckController`。详见
[V11.1 自定义控制器终点修复](docs/CUSTOM_NAV_CONTROLLER_FIX_V11_1.md)。

V11.2 针对“预测碰撞后持续零速度、每秒重复接收同形路径”的实车现象，
在 LuckController 内加入 3×7 组速度弧线的 footprint 滚动评价。正常轨迹安全时
完全沿用 V11.1；仅在异常时选择减速/同方向局部绕行，无候选时先等动态行人，
持续 5 秒才交给 `清图 + 等待 + 重规划` 的安静恢复树。自定义导航档不再执行
盲目 Spin/BackUp，Collision Monitor 仍保留最终安全裁决。详见
[V11.2 局部速度采样与安静重规划](docs/CUSTOM_NAV_ROBUST_REPLAN_V11_2.md)。

V11.3 根据 `custom_nav_20260908_091049` 的完整代价地图与控制数据修复一个
确定性边界错误：V11.2 只检查 footprint 是否碰到 `>=254`，没有约束预测轨迹
的机器人中心，因而可能把中心带入 Nav2 的 253 内切区。控制器随后位于全局
规划器认定的致命起点，造成拐角持续停车重规划。现在规划器和控制器统一只
中心规划代价 `<=252`；若起点已经是 253，则只接受能够真正退出内切区的弧线。
持续无解用 Humble 支持的 `PlannerException` 上报，由 Controller Server 的
`failure_tolerance` 限定耐心时间。footprint
仍按真实致命碰撞阈值 254 判断，不会误杀成功包中安全的贴边转弯。`luck_full` 实车包
`custom_nav_20260908_092417` 的三次目标均成功，最长 3.30 m，横向误差 P90
为 2.2 cm，因此本轮不改已验证的速度、转向增益、膨胀半径或安全层。详见
[V11.3 拐角内切代价保护](docs/CUSTOM_NAV_INSCRIBED_GUARD_V11_3.md)。

V11.4 重新审视 `custom_nav_20260908_092417` 的过程数据：虽然三个目标全部
成功且横向误差很小，最长一段仍出现 16 次稳定角速度换向和 11 次稳定航向
误差换向，符合实物看到的 180° 掉头末段过冲。LuckController 现在用独立的
`rotate_braking_decel=0.20 rad/s²` 在约 17.5° 时提前收掉原地转速，8.6° 后
仍边走边修正，不增加停顿。诊断脚本的生命周期/参数调用均增加超时并立即显示
进度，分析器会明确标记欠阻尼段。本轮保留全局/局部膨胀半径 0.30/0.25 m，
避免把转向与选路安全两个变量混在一次实车试验中。详见
[V11.4 掉头提前制动与可见诊断](docs/CUSTOM_NAV_TURN_BRAKE_V11_4.md)。

V11.5 根据 `custom_nav_20260908_105210` 分离修复行驶摆动和拐角停车。旧版在
前视点仍位于左侧约 15° 时，会被新路径未来线段约右侧 50° 的切向带成反方向
满角速度；现在控制器以 10 Hz 对实际前视点方向做 0.30 秒滤波，并在原线速度
上叠加带死区、限幅和角速度阻尼的连续转向量。拐角处只有当 21 条正向弧线
全部发生 footprint 碰撞时才允许最长 2.5 秒的原地松角，仍无解就明确交给行为树
重新规划，不把原地旋转误当成路线进展。LuckPlanner 同时提高软代价权重并限制
高代价区的路径拉直，避免重复生成切过内侧的长线段。全局/局部膨胀半径继续
保持 0.30/0.25 m，不通过缩安全边界掩盖控制和选路问题。详见
[V11.5 动态航向阻尼与拐角有界脱困](docs/CUSTOM_NAV_DYNAMIC_TRACKING_V11_5.md)。

V11.5 在 Jetson 首次部署后，`controller_server` 在未开始导航时报告
`free(): invalid next size (normal)` 并以 `SIGABRT(-6)` 退出；随后生命周期管理器
按设计关闭其余导航节点。因此 V11.5 控制器增量已被隔离，不能继续实车使用。
V11.5.1 启动恢复版保留 LuckPlanner 的居中选路参数，但将 LuckController 精确
恢复为已经过连续实车运行的 V11.4 实现，并增加 60 秒无运动启动门禁。只有门禁
通过后才允许一次短距离单目标测试；动态航向和拐角脱困将经独立内存检查后逐项
恢复。详见 [V11.5.1 控制器启动恢复](docs/CUSTOM_NAV_STARTUP_RECOVERY_V11_5_1.md)。

V11.6 根据 V11.5.1 的实车终端记录做最小修复。恢复版连续完成前两个目标且
`controller_server` 未再崩溃；第三个转角的安全前进候选由 17 降到 8 再降为 0。
控制器第一次有界失败后，旧阻塞时间跨越行为树 Wait 保留下来，令后续 FollowPath
仅约 0.3 秒就重复失败。现在行驶转向只使用几何前视点的 Pure Pursuit 曲率，完全
撤下可能在拐角反向的未来线段切向项；全部前进弧线失败时，仅以 0.12 rad/s 做
最多 2 秒、逐周期 footprint 校验的原地松角。行为树启动的新 FollowPath 会获得
新的有界计时，而普通 1 Hz 路径刷新不会清零。启动门禁也允许单次 ROS 服务超时，
避免把仍然存在且 active 的节点误报为崩溃。详见
[V11.6 最小拐角恢复](docs/CUSTOM_NAV_MINIMAL_CORNER_RECOVERY_V11_6.md)。

V11.7.3 分析了 `custom_nav_20260908_150516/150835`：两包的安全速度与双轮
RPM 一致，分别实际运动 4.18 m 和 8.76 m，且没有持续 1 秒的执行停滞；真正
不动的长返回目标发生在录包结束约 15 秒之后。控制器现增加 3 秒“非零命令但
融合里程计静止”看门狗，异常时主动交回行为树清图、等待并重规划，不让同目标
路径刷新无限掩盖停滞。全局重规划从 1 Hz 降为 0.5 Hz，保留 10 Hz 局部响应并
降低 action 应答超时。启动/录包门禁现在拒绝同名重复节点，新增干净停止脚本，
诊断支持 `run_custom_nav_diag.sh 0` 一直录到 Ctrl+C，并逐帧比较安全速度和电机
RPM。正常速度、转向、膨胀半径和安全层保持 V11.6。详见
[V11.7.3 执行链停滞看门狗](docs/CUSTOM_NAV_EXECUTION_WATCHDOG_V11_7.md)。纯原地转向
单独使用 6 秒窗口，避免起步时融合里程计延迟造成误触发；平移运动仍为 3 秒。
清理脚本现在还会定向结束 SIGINT 后残留的机器人进程并刷新 ROS CLI daemon。
主雷达过滤器不再设置进程级 `__node` 重映射，消除 Humble LaserScanBoxFilter
内部辅助节点与主节点同名的问题；滤波参数和 `/scan_filtered` 数据流保持不变。
V11.7.4 进一步修正诊断门禁：节点存在且未返回明确非 active 状态时，单纯的
`/get_state` 服务超时记为 `PASS_WITH_WARNINGS`，不再误报节点死亡，也不阻止
故障期 rosbag 录制。该补丁只影响诊断脚本，不改变任何运动参数或控制输出。

V11.8 根据 `custom_nav_20260908_195429` 的完整拐角故障包定位到控制器根因：
全局路线实际从代价 `99 -> 52 -> 35 -> 0` 正确离开墙边，但固定 0.35 m 前视
跨过近端急弯，把本应左转约 63° 的 0.12 m 路径误读成只需左转约 12.5°，
局部采样随后又选中使中心代价 `153 -> 173` 的错误一侧。现在直线仍使用
0.35 m，只有近/远路径方向明显分叉时才连续缩短到 0.12 m；候选评分显式优先
降低末端中心代价。正向弧线与扫掠原地转向都无解时，最多允许 2 秒、4 cm/s、
完整 footprint 验证且中心代价必须下降的安全短倒退。规划器、膨胀半径、底盘
和 Collision Monitor 均未修改。详见
[V11.8 拐角路径贴合与受控脱困](docs/CUSTOM_NAV_CORNER_ADHERENCE_V11_8.md)。

V10.4 根据两次 2026-09-06 实车录包补上完成收敛：小于 `0.35 m²` 的
残余灰区不再生成目标，小于 `0.60 m²` 的碎片不写入 DFS 分支栈；完成区
只有新增至少 `0.75 m²` 未知面积才重新打开。分支末端确认由 6 秒缩短为
2.5 秒，所有有效分支清空后连续 15 秒无候选即发布探索完成。该版本没有
修改底盘、Nav2 控制器、膨胀层或 Collision Monitor 的稳定基线，详见
[V10.4 分支收敛与录包诊断](docs/EXPLORATION_COMPLETION_V10_4.md)。

RViz 中蓝/红点是前沿边界，绿色圆点是当前实际目标，黄色点是记住的白区
父分叉，紫色点是对应的待探索子分支方向。日志中的 `V10.4 precheck` 会打印
UNKNOWN gain，`V10.4 goal locked`
会打印步长、全局代价、分支编号和 DFS 栈深度。探索行为树使用独立的
路过点 goal checker，不再为每个 0.85 m 中间点精修最终 yaw；手动 RViz
导航仍保持 8 cm / 0.10 rad 精度。

Humble 官方默认行为树在 `FollowPath` 中不填写 goal checker 名称；同时
加载手动和探索两个 checker 后，这会使 RViz 手动导航直接失败并误触发
Spin/Wait/BackUp。当前默认单点/多点导航树已显式绑定
`general_goal_checker`，探索专用树显式绑定 `explore_goal_checker`，两条链
互不串用。修复依据见
[V10.1 Goal Checker 路由修复](docs/GOAL_CHECKER_ROUTING_FIX_V10_1.md)。

V9 的未知区分支设计见
[V9 UNKNOWN-basin DFS 探索设计](docs/UNKNOWN_BASIN_DFS_EXPLORATION_V9.md)，
实车诊断后的路径执行升级见
[V10 直接转场探索设计](docs/DIRECT_TRANSIT_EXPLORATION_V10.md)。

保存栅格地图和可继续使用的 posegraph（路径不要带后缀）：

```bash
mkdir -p ~/ros2_ws/maps
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/nvidia/ros2_ws/maps/my_room_map'}"
ros2 run nav2_map_server map_saver_cli -f /home/nvidia/ros2_ws/maps/my_room_map
```

日常导航使用 `slam_toolbox` posegraph 定位，不启动 AMCL：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=localization \
  posegraph:=/home/nvidia/ros2_ws/maps/my_room_map
```

需要把家具布局等长期变化永久并入旧地图时，使用 `lifelong`，结束后再次调用 `serialize_map`：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=lifelong \
  posegraph:=/home/nvidia/ros2_ws/maps/my_room_map
```

`localization` 的近期扫描缓存只用于定位，底层 posegraph 不会被永久修改；日常动态障碍由 Nav2 局部代价地图处理。不要为了一个临时纸箱频繁永久改图。

上述两个加载旧图的命令默认机器人从充电桩/posegraph 首节点启动。若机器人不在该位置，加上 `start_at_dock:=false`，然后在 RViz 中给出初始位姿。

若首次验证 Collision Monitor 时出现误停，可临时回退进行对比（只用于诊断）：

```bash
ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping use_collision_monitor:=false
```

旧的 `robot_bringup.launch.py` 和 `test_bringup.launch.py` 均保留；前者仍假设外部已经有 `map -> odom` 提供者。推荐日常只使用 `smart_navigation.launch.py`，避免重复启动 SLAM 或 AMCL。

## 上车前必须核对 DDS

当前 Jetson 的 `.bashrc` 实际是：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/ros2_ws/fastdds_shm.xml
```

这与交接说明中的“强制 CycloneDDS”不一致，而且仓库的 FastDDS XML 没有绑定 Wi-Fi 网卡。先记录实际基线，不要在同一次导航测试中顺便切 DDS：

```bash
echo "$RMW_IMPLEMENTATION"
echo "$FASTRTPS_DEFAULT_PROFILES_FILE"
echo "$CYCLONEDDS_URI"
ros2 doctor --report
```

DDS 切换要作为独立实验进行，并保证 Jetson、远程 RViz/控制端的 `ROS_DOMAIN_ID=100` 和 RMW 实现一致。

## Isaac ROS VSLAM 的版本边界

当前宿主是 ROS 2 Humble、JetPack 6.x、CUDA 12.6，不能直接照搬 Isaac ROS 最新版文档。最新版已经面向 ROS 2 Jazzy / JetPack 7.2；本机应固定在 [Isaac ROS release-3.2](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/index.html)，该版本的测试矩阵才是 ROS 2 Humble + JetPack 6.1/6.2。

Gemini 336L 是 95 mm 基线的双目机型，但不带 IMU。release-3.2 的 cuVSLAM 应先按纯双目模式验证左右 IR 图像、标定和硬件时间戳；官方要求相机至少 30 Hz、帧周期抖动不超过 ±2 ms、左右采集时刻偏差不超过 ±100 us。当前导航相机配置是 15 Hz，因此不能直接把现有 `/camera/depth/*` 流接进去。后续 VSLAM 应作为“里程计源”接入 EKF，并关闭它对全局 `map -> odom` 的竞争，继续由 `slam_toolbox` 负责 2D 全局地图。

VSLAM 与旁路 EKF 的完整分阶段部署见
[VSLAM 分阶段接入方案](docs/VSLAM_INTEGRATION_V12.md)。当前主里程计和 TF 在旁路验收
完成前保持不变；LocateAnything 已从本机部署范围中移除。2026-09-09 已在 Jetson
完成 Docker/NVIDIA Runtime、Isaac ROS release-3.2、cuVSLAM 3.2.6 和独立观察节点的
安装构建，并通过 GPU 预热静态启动检查；下一步只差相机上车后的旁路跟踪实测。详见
[cuVSLAM 部署状态与测试命令](docs/VSLAM_DEPLOYMENT_STATUS_20260909.md)。

## 验收

详细步骤见 [导航闭环验收清单](docs/NAVIGATION_ACCEPTANCE_TEST.md)。第一轮只验证：单一 TF 发布者、传感器频率、静止漂移、直行/原地旋转、近场制动、单点导航、动态绕障、恢复行为和自动探索结束。所有项目通过后再进入 Isaac ROS VSLAM。
# V12.11 原地转向静摩擦修复（2026-09-22）

`custom_nav_20260922_201509` 证明：故障时安全速度链持续输出转向命令，轮编码器
也报告转动，但 IMU 在约 25.9 秒内确认车体完全未转。V12.11 为 LuckController
增加 IMU 真实运动见证与受限 `0.35 rad/s` 脱静摩擦增益，并修正纯原地转向的
执行失速判定。详见
`docs/ROTATION_BREAKAWAY_GUARD_V12_11_20260922.md`。

# V12.14 连续弧线跟踪与 SLAM 错误匹配收紧（2026-09-30）

**状态：已撤回，禁止继续实车使用。** 三份 2026-09-30 实车包显示 V12.14
未能阻止定位跳变，并放大了打滑后的不稳定表现。请使用 V12.14.1 回退包。

`custom_nav_20260929_210835/212702` 证明卡顿和地图漂移是两个问题：普通
29～38°误差被低速条件切成原地转向；另一次长距离回访在约 11 秒内发生多次
23～28 cm 的 `map -> odom` 跳变，累计约 1.16 m。V12.14 让小于约 46°的
路径偏差始终使用连续 `v+w` 圆弧修正，并将可执行线速度下限提高到 0.05 m/s；
同时把误当作雷达量程的 `link_scan_maximum_distance=8.0` 恢复到 1.5 m，启用
HuberLoss、提高扫描/回环响应门槛并降低静态地图重绘负担。Collision Monitor
和两路相机安全/规划链保持不变，人员进入时仍可立即停车。详见
[V12.14 连续弧线跟踪与 SLAM 错误匹配收紧](docs/SMOOTH_TRACKING_SLAM_MATCH_GUARD_V12_14_20260930.md)。

# V12.14.1 紧急回退与动态地图边界（2026-09-30）

`custom_nav_20260930_100232/100558/100845` 显示：第二、三次分别出现约
30～37 cm 的连续 `map -> odom` 跳变；第三次轮式里程计报告约 25°转动而
IMU/融合航向接近 0°，证明存在驱动轮打滑、接地/阻力不一致或轮端运动未转化为
真实车体运动。V12.14.1 完整撤销 V12.14 的控制器与 SLAM实验参数，恢复
V12.13.1/V12.12.1 运动基线，同时保留高度相机避障、相机/雷达独立代价层和
Collision Monitor。动态人员继续由可清除的代价地图处理，不写入静态 SLAM。
详见 [V12.14.1 紧急回退与动态地图边界](docs/V12_14_1_EMERGENCY_ROLLBACK_AND_DYNAMIC_MAP_20260930.md)。
