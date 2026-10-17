# LuckRobot VSLAM 门控时序修正 V12.6.3

## 本次记录结论

V12.6.2 的安全隔离正确：运行节点中只有 `vslam_odom_guard`，没有候选 EKF；
轮速、IMU、生产 `/odometry/filtered` 和门控 VSLAM 输出均通过有限值检查，记录中
没有 NaN、`TF_NAN_INPUT` 或生产 TF 污染。

首次读取 `/vslam/health=false` 是在门控完成首次 60 个连续合格样本以前，不代表
后续没有输出。附件中门控于 `1789391588.277` 完成首次准入，随后持续发布约
30 Hz。

但静止测试中仍出现两次 `stale_reference_odometry` 和一次
`missing_synchronized_covariance`。这是参考里程计只保存“最新一帧”以及单帧
DDS/调度抖动立即触发完整恢复导致的误判，不是 VSLAM 位姿跳变。

## V12.6.3 修改

- 保存约 6 秒生产里程计历史，按消息时间戳为每帧 VSLAM 选择最近参考；
- 单帧参考/协方差传输错配只丢弃该帧，连续 3 帧才暂停门控；
- NaN、无效四元数、位姿跳变、速度异常、高协方差以及相对生产里程计偏差仍然
  立即熔断；
- 门控仍不发布 TF，候选 EKF 仍必须保持关闭；
- 健康检查新增 `--require-vslam`，会识别“准入后再次掉线”。

## 部署

先在所有终端停止当前导航、门控和 VSLAM 测试进程，再执行：

```bash
cd ~/ros2_ws
sha256sum luckrobot_vslam_guard_timing_v12_6_3_20260914.zip
unzip -o luckrobot_vslam_guard_timing_v12_6_3_20260914.zip -d ~/ros2_ws
sha256sum -c docs/VSLAM_GUARD_TIMING_V12_6_3_20260914_MANIFEST.sha256

chmod +x src/my_robot_bringup/scripts/vslam_odom_guard.py
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-clean-cache \
  --packages-select my_robot_bringup \
  --allow-overriding my_robot_bringup
source ~/ros2_ws/install/setup.bash
```

## 静止稳定性验收

宿主机终端 1：

```bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh
ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping \
  navigation_profile:=luck_full \
  use_rviz2:=true \
  enable_stereo_ir:=true \
  start_vslam_guard:=true \
  start_vslam_candidate:=false \
  start_vslam_candidate_ekf:=false
```

宿主机终端 2 按原方法进入 Isaac ROS 容器并启动：

```bash
ros2 launch luckrobot_vslam stereo_vslam_observe.launch.py
```

等待终端 1 出现 `VSLAM healthy for 60 samples` 后，宿主机终端 3 执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

python3 ~/ros2_ws/tools/check_ekf_input_health.py \
  --duration 120 \
  --require-vslam

timeout -s INT 5s ros2 topic echo /vslam/health --once
```

通过条件：总结果为 `RESULT=PASS`，健康状态为 `data: true`，启动终端没有
`VSLAM rejected`、NaN、`TF_NAN_INPUT` 或 `Critical Error`。通过前不要启动
`start_vslam_candidate_ekf`，也不要移动机器人。
