# LuckRobot V12.8：主里程计毛刺保护

## 故障证据

`vslam_diag_20260915_100901` 录包开始时，小车静止且
`/wheel/odom` 位姿约为 `(0.0002, 0.0)`，cuVSLAM 原始位姿为 `(0, 0)`，
但主 EKF `/odometry/filtered` 已错误地位于 `(72.15, 69.61)`。
Nav2 随后报告机器人和传感器原点越出代价地图，RViz 中机器人模型、局部/全局
代价地图消失，规划和探索均无法继续。

VSLAM Guard 当时为不健康，`/visual_slam/tracking/odometry_validated` 没有数据；
候选 VSLAM EKF 也未运行。因此该故障不是 VSLAM 融合造成的。

底盘驱动原先直接用编码器位置差除以 ROS 时间差生成 Twist，缺少物理上限校验。
编码器回包毛刺或极短时间差可产生巨大瞬时速度；轮式累计位置在下一帧可能恢复，
但主 EKF 已经把异常速度积分成巨大假位移。

## 修复

- 依据 300 RPM 电机物理上限检查左右编码器单帧增量。
- 拒绝非有限、时间差异常和超过底盘物理上限的 Twist。
- 被拒绝的样本不进入 `/wheel/odom`，并更新差分锚点，防止下一帧重复污染。
- 主 EKF 开启时间跳变复位和 `odom0_twist_rejection_threshold`，形成第二道保险。
- VSLAM 诊断改为先启动 rosbag，再做轻量状态检查；不再在导航运行期间订阅
  15 秒双目原始图像，也不会漏录检查期间出现的 TF/里程计异常。

## 部署后验收

1. 关闭旧导航栈并重新构建 `chassis_driver`、`my_robot_bringup`。
2. 启动统一系统 launch，静止 30 秒。
3. 确认 `/wheel/odom` 与 `/odometry/filtered` 的平面位置均接近原点，且差值不突变。
4. 依次测试开阔直线、90° 转向、此前拐角，再启动自动探索。
5. 若日志出现 `Rejecting implausible ...`，诊断保护已经生效；把当次诊断包提供出来，
   后续再针对串口回包质量做统计，不应取消保护。
