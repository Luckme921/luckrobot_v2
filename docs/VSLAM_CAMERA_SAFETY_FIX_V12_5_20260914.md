# cuVSLAM 输入与高位障碍安全修复 V12.5

## 本轮结论

`auto_diag_20260914_164354` 不能作为 VSLAM 稳定性通过证据。录包中
`/visual_slam/tracking/odometry` 和 `vo_pose_covariance` 均为 0 条，而
`/odometry/vslam_candidate` 约 49 Hz；候选 EKF 当时实际上只靠轮速和 IMU
继续输出。该候选输出不能代表 cuVSLAM 已经参与融合，生产导航仍保持
`/odometry/filtered`。

同一包的相机日志给出了明确原因：Gemini 336L 拒绝 `depth=640x400@15` 与
`left/right IR=640x400@30` 的混合 profile，随后不断 reset pipeline。因此
深度图没有消息，`/scan_camera` 也没有消息；Collision Monitor 只剩低位 2D
雷达，无法可靠看到桌沿、桌面下沿或高于雷达平面的线缆。

## 修复内容

- `test_bringup.launch.py` 在 `enable_stereo_ir:=true` 时自动把深度改为
  `640x400@30`，与左右 IR 完全一致；普通 Nav2 仍使用 15 Hz。
- 深度伪激光垂直采样从 10 行改为默认 60 行，并可用
  `depth_scan_height:=N` 覆盖。相机光心约 0.37 m、水平安装时，60 行约覆盖
  光心上下 5.5°，能看到只高出镜头几厘米的桌沿；3 m 范围内不会把地面大量
  当成障碍。它仍是二维安全投影，不等价于完整三维碰撞检测，线缆/桅杆必须
  保留物理余量。
- `run_auto_diag.sh` 在自动探索前必须收到新鲜的 `/scan_filtered` 和
  `/scan_camera` 消息；检测到候选 EKF 或 cuVSLAM 节点时，还必须收到原始
  `/visual_slam/tracking/odometry` 和候选输出，否则拒绝启动探索。这样不会
  再在传感器失效时让机器人运动。

## 验证顺序

1. 停止旧导航、探索和 VSLAM 容器，重新构建本包。
2. 先只启动系统并观察 `/scan_camera`，让机器人静止；确认相机日志不再出现
   “depth/ir streams must have the same resolution and frame rate”。
3. 在 RViz 检查相机扫描能看到桌沿，再用极低速、空旷区域做单点导航。不要
   直接把机器人开向桌沿测试。
4. 开启 `enable_stereo_ir:=true` 和候选 EKF，启动容器内 cuVSLAM；确认原始
   cuVSLAM 约 30 Hz 且有非零协方差，再运行 `run_auto_diag.sh`。
5. 只有长直线、闭环、遮挡恢复和低纹理场景均通过后，才考虑健康门控下的
   主 EKF A/B 切换；本版本不自动接管 Nav2 TF。

