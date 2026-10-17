# VSLAM 长时动态实车验收结论（2026-09-21）

## 结论

Gemini 336L 的被动双目输入链本身通过，但当前相机/驱动与 Isaac ROS cuVSLAM
组合未通过长时动态验收，不能接入生产 EKF、Nav2 或主 TF。当前量产式启动继续使用
轮速 + IMU 的 `/odometry/filtered`；VSLAM 保留为默认关闭的研究旁路。

这是一次明确的“不接入”结论，不再通过放宽健康阈值掩盖问题。

## 证据

`vslam_diag_20260921_183544` 覆盖约 535 秒、20 个运动段：

- 左右 IR 约 29.964 Hz，时间戳单调，99.8% 的双目对采集差小于 0.1 ms；
- `/scan_camera` 与四个 Nav2 生命周期节点正常；
- 轮速路径约 14.94 m，而原始视觉路径约 63.70 m，路径比 4.263；
- 原始视觉出现约 126.26 m/s 的非物理瞬时速度和约 21.17 m 的净位移；
- 视觉位置标准差最大达到 753/1018 m，Yaw 标准差最大约 2457°；
- 健康门控只有 32.2% 时间为真；受保护输出虽把航向差压到约 3.24°，
  但前向尺度仍比轮速低约 34.3%，且最长断流约 56.35 秒。

因此短距离直行/原地旋转曾经通过，只能证明初始化和局部跟踪可用，不能证明长时、
多转角和室内弱纹理场景可用于导航融合。本次长时数据是最终优先证据。

## 安全性

本轮 `start_vslam_candidate_ekf:=false`，健康门控不发布 TF；坏的视觉里程计没有进入
生产 EKF。Nav2 整体运动良好，说明现有隔离设计有效。严禁把
`/visual_slam/tracking/odometry` 或门控输出直接改接主 EKF。

## 独立发现

1. 首次系统启动的主 EKF 出现 NaN，与 VSLAM 融合无关。主 EKF只订阅轮速和 IMU，
   当时 VSLAM 候选 EKF关闭。IMU 驱动已增加非有限数值与非法四元数发布前拦截。
2. 最后一个拐角停住时，LuckController 多次报告所有前向局部轨迹均不安全，已经执行
   安静重规划、清代价地图、限时角度脱困与重试；新全局路径仍落在同一不可行局部。
   人工发布新的近目标改变路径拓扑后成功到达。该问题属于局部可行域/恢复策略，
   与 VSLAM 跳变无关。
3. 旧版 `run_vslam_diag.sh` 在 Ctrl-C 后可能无限等待 rosbag 退出，从而没有执行 tar。
   新版对 rosbag 采用有界 SIGINT/SIGTERM/SIGKILL 收尾，并保证中断后继续打包。

## 后续路线

- 生产版关闭 `enable_stereo_ir`、`start_vslam_guard`、`start_vslam_candidate` 和
  `start_vslam_candidate_ekf`，释放 USB、CPU/GPU 与相机带宽给深度避障和语义感知。
- 第一台 Jetson 继续负责底盘、轮速/IMU EKF、2D SLAM、Nav2、深度安全层和探索。
- 下一阶段优先做关键帧语义检测与语义地标入图；全量大模型、对话、语音放到第二台
  Jetson。两机通过带时间戳的 ROS 2 消息交换语义观测与导航任务。
- 若未来仍需 VSLAM，应更换为具备可靠硬件同步、稳定全局时间戳、适合室内弱纹理的
  双目/双目惯性传感器，再重新做同一套 10 分钟、多转角、遮挡和失锁恢复验收。

## 2026-10-17 新夹具后的静止复查

新夹具不改变相机、雷达、IMU 安装位姿。轮式角速度旁路 A/B 已完成，但没有显示出
值得接入主 EKF 的收益。随后仅在旁路启动 cuVSLAM：主 EKF、Nav2、TF 仍用原配置，
`start_vslam_candidate_ekf:=false`，cuVSLAM 的两个 TF 发布开关均为 `false`。

相机 `interleaved` 模式成功启动，深度 SequenceIdFilter=0、双目 IR=1；
`/scan_camera` 约 30 Hz，视觉原始里程计约 16–20 Hz。静止时健康门控持续拒绝视觉：
`state=visual_position_covariance_high`、位置标准差约 1.0 m、航向标准差约 57°，
诊断时已拒绝 2728 帧、接受 0 帧。容器约占 314 MiB，`docker stats` 显示约 12% CPU；
宿主可用内存由约 4.0 GiB 降至约 3.3 GiB。此为单次静止快照，不能代表动态峰值。
由于静止门槛未通过，本轮未进行运动对照，也未接入候选 EKF。停止旁路并恢复生产
相机配置后，`check_nav_tf_health.py --duration 10` 中 EKF、odom TF、map TF、相机
均为 OK。此前的“不接入生产定位”结论不变。
