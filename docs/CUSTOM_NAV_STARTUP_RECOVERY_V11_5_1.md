# LuckRobot V11.5.1：控制器启动恢复与故障隔离

## 1. 故障定性

Jetson 已成功编译 V11.5，但 `controller_server` 在导航节点全部激活后、尚未发送
目标时输出：

```text
free(): invalid next size (normal)
process has died ... exit code -6
```

这表示进程检测到堆内存元数据损坏并主动 `SIGABRT`，不是目标点不可达、膨胀
半径过大、EKF 偶发超周期或参数查询超时。`controller_server` 死亡以后出现的
`Node not found`，以及 planner、BT、smoother 变为 inactive，都是生命周期管理器
安全联锁的后果。此时不得发送目标，也不能靠重新查询参数恢复，必须重启完整导航。

V11.5 相对已运行通过的 V11.4 同时加入了航向滤波状态、扩展调试数组、原地旋转
轨迹候选和 FollowPath 重试计时。仅凭一次 glibc 报错不能严谨地指定到其中某一行，
所以本版不猜测单一原因，而是先隔离整个控制器增量。

## 2. 本版改动边界

- `LuckController` 的 HPP/CPP 精确恢复为 V11.4；撤下 V11.5 新增状态和原地候选。
- 保留 V11.5 `LuckPlanner` 的 `cost_penalty=5.0` 和
  `smoothing_max_cost=100`，因为崩溃进程是 controller_server，且 planner_server
  已正常激活；这两项只影响全局路径居中和拐角拉直。
- 全局/局部膨胀半径继续为 `0.30/0.25 m`，车体半径和 Collision Monitor 不变。
- 行为树继续使用 `navigate_to_pose_luck_full.xml`，没有恢复盲目 Spin/BackUp。

这是一次故障二分和安全恢复，不代表放弃 10 Hz 连续航向修正。启动和短目标均
稳定后，再把动态航向、扩展诊断、原地脱困分成独立版本逐项加入；任何一步出现
崩溃即可直接定位，不再一次混入多组状态。

## 3. 部署

先关闭旧的导航、探索和诊断终端，不发送目标。Jetson 执行：

```bash
cd ~/ros2_ws

sha256sum luckrobot_custom_nav_startup_recovery_v11_5_1_20260908.zip

unzip -o \
luckrobot_custom_nav_startup_recovery_v11_5_1_20260908.zip \
-d ~/ros2_ws

sha256sum -c \
docs/CUSTOM_NAV_STARTUP_RECOVERY_V11_5_1_MANIFEST.sha256

source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --cmake-clean-cache \
  --packages-select \
  luckrobot_nav2_plugins \
  my_robot_navigation \
  my_robot_bringup \
  --allow-overriding \
  luckrobot_nav2_plugins \
  my_robot_navigation \
  my_robot_bringup

source ~/ros2_ws/install/setup.bash

ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full
```

## 4. 先做 60 秒无运动启动门禁

另开终端，保持机器人原地且不要在 RViz 发布目标：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

bash ~/ros2_ws/tools/check_custom_nav_startup.sh 60
```

脚本每两秒检查五个生命周期节点。预期输出 `RESULT=PASS`，并生成
`~/custom_nav_startup_*.tar.gz`。如果仍出现 `controller_server` 消失，立即停止，
不要测试运动；上传该包和终端 1 从 “Configured LuckFollowPath” 开始到崩溃的输出。

## 5. 门禁通过后的唯一运动测试

门禁通过后才运行：

```bash
bash ~/ros2_ws/tools/run_custom_nav_diag.sh 90
```

看到 `[5/5] rosbag 已启动` 后，只发布一个 0.8–1.2 m 的开阔直线目标。不要启动
自动探索，不测试窄拐角，不临时改参数。完成后上传最新的
`~/custom_nav_*.tar.gz`。本轮只回答两个问题：控制器是否还会崩溃、恢复版的基本
速度链是否完整；通过后再继续轨迹与脱困算法。

## 6. 与当前日志中其他提示的关系

- `EKF Failed to meet update rate`：说明 Jetson 有调度抖动，需要后续做资源优化，
  但 20–35 ms 的偶发超期不会解释 glibc 的堆损坏；343 ms 峰值应继续记录。
- RViz 的旧时间戳丢包：常见于启动阶段缓存，和 controller 堆崩溃不是同一故障。
- `get_parameters timeout`：节点忙或已经进入联锁关闭时的服务超时，是结果而非根因。
- CUVSLAM 尚未因为本包而安装；此前 VSLAM 预检只验证双目话题、标定与同步通过。
