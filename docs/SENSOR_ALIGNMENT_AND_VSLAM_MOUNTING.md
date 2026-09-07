# 传感器对齐现状与 VSLAM 相机安装建议

## 1. 现有两次录包的时间与 TF 结论

对 `auto_diag_20260907_133130` 和 `auto_diag_20260907_134030` 的消息头时间、
录包接收时间和 TF 所有权进行了离线核对：

| 数据源 | 实测频率 | 消息头单调性 | 典型接收滞后 |
|---|---:|---:|---:|
| IMU | 100.00 Hz | 无倒退 | 中位约 0.7 ms |
| 轮式里程计 | 约 30.1 Hz | 无倒退 | 中位约 0.4 ms |
| EKF 里程计 | 约 50.0 Hz | 无倒退 | 中位约 4.6～9.1 ms |
| 相机伪激光 `/scan_camera` | 14.98 Hz | 无倒退 | 中位约 33 ms |
| A2M12 过滤扫描 `/scan_filtered` | 约 12.4 Hz | 无倒退 | 中位约 81 ms |

雷达与相机扫描消息头的最近时间差中位约 16.6～16.8 ms、95% 约 31.8 ms。
IMU 经当前倒装 TF 转成车体 Z 轴后，与轮式角速度的相关系数为 0.845～0.917，
最佳相位约为 -30 ms。对当前 0.15 m/s、0.35 rad/s 的低速 2D Nav2/EKF，
这属于可用但仍可优化的软同步水平，不是本轮探索卡死的主因。

TF 树为单一连通链：`map -> odom -> base_footprint -> base_link -> sensors`。
11 个子坐标系均只有一个父节点，未发现多个父节点或运行中变化的静态 TF。
因此可确认“软件坐标树和时间戳基本一致”；但这不能证明相机、雷达和 IMU 的
物理外参已经通过标定靶实测。当前数值主要来自 CAD/SolidWorks 和手工精调，仍需
在最终机构上做外参验收。

## 2. 当前相机高度

URDF 中：

- `base_footprint -> base_link` 高 0.0135 m；
- `base_link -> camera_color_screw_frame` 高 0.13933 m，且相机倒装；
- 螺丝孔到 `camera_link` 的 Z 偏移为 0.014311 m，倒装后该项向下。

所以相机深度光心离地约为
`0.0135 + 0.13933 - 0.014311 = 0.1385 m`，即 **13.85 cm**。
雷达扫描平面约为 `0.0135 + 0.096 = 0.1095 m`。

## 3. VSLAM 机械建议

13～14 cm 并不会让 VSLAM 算法无法运行，但对室内移动机器人并不理想：视野容易
被底盘、椅腿和近地物体遮挡，画面中地面占比大，远处墙面、门框、家具边缘等稳定
特征不足。Gemini 336L 深度视场约为 90°×65°；水平安装在当前高度时，下边缘
约 0.22 m 外就与地面相交。

建议机构不要一次做死：

- 光心目标高度 **0.40 m**，保留 **0.35～0.45 m** 三档或滑槽调节；相对当前
  光心先上移约 **0.26 m**。
- 初始俯角采用 **5～8°向下**，并提供约 ±5°微调。这样仍能看到门框/家具等
  中高处特征，同时保持一定地面纹理和近场覆盖。
- 支架必须刚性高、无明显扭转和共振；相机光心相对 `base_link` 的轻微晃动会被
  VSLAM误认为机器人运动，通常比几厘米的高度选择更致命。
- 固定 VSLAM 相机不建议随对话对象自动俯仰；若未来需要“抬头看人”，优先增加
  独立云台 RGB 相机。若必须共用，动态关节角必须实时进入 TF，算法和标定复杂度
  明显增加。

重要安全约束：当前 `depthimage_to_laserscan` 只取深度图中央 10 行。把相机抬到
0.40 m 后，这条水平扫描会漏掉低矮障碍，不能仅改 URDF 高度就继续沿用。机械升高
应与下列二选一方案同步：

1. 保留约 0.11 m 高的 2D 雷达负责低位安全，并把深度避障改为点云高度带 + 地面
   剔除；或
2. 另保留低位近场深度/超声传感器。不要简单扩大 `scan_height`，否则地面会被
   大量误判为障碍。

## 4. VSLAM 数据要求与实施顺序

当前导航启动只开 640×400@15 Hz 深度和 640×480@15 Hz 彩色流，左右 IR 默认关闭。
这不能直接作为 cuVSLAM 双目输入。与本机 ROS 2 Humble / JetPack 6.x 对应的 Isaac
ROS release-3.2 要求目标图像频率至少 30 Hz、帧率抖动约 ±2 ms、同一双目对时间差
约 ±100 µs。Gemini 336L 没有内置 IMU，因此第一阶段应先做纯双目 VSLAM，不能把
当前 UART 接收时才打时间戳的 IMU 直接当作硬同步 VIO 输入。

推荐顺序：

1. 先完成可调高支架，暂定 0.40 m / 下俯 5～8°，不要立即修改稳定导航基线。
2. 单独建立 VSLAM 测试 launch：左右 IR 640×400 或驱动支持的匹配分辨率 @30 Hz，
   `enable_frame_sync:=true`，关闭会污染 IR 纹理的主动投射模式，并记录左右图像
   `camera_info` 与消息头时间差。
3. 验证双目内参、畸变、基线和左右光学 TF；这些应来自相机标定/驱动，不能用卷尺
   覆盖出厂值。
4. 机构最终锁紧后再标定 `base_link -> camera_link`。以 CAD/卷尺值为初值，用两个
   不同墙角和专用标定靶复核。现有二维 ICP 脚本只能在静止且两个扫描平面看到同一批
   竖直结构时做残差诊断，不能替代完整 3D 相机—雷达外参标定。
5. 分开估计时间偏差与空间外参；时间错位不能靠修改 XYZ/Yaw“补偿”。完成后在机器人
   原地旋转时叠加点云与激光，静态墙边不应分叉或拖影。
6. cuVSLAM 先只输出局部视觉里程计，作为 EKF 的候选里程计源；全局
   `map -> odom` 仍由 `slam_toolbox` 独占。完成 A/B 验收后再决定融合权重。

使用 `tools/run_sensor_alignment_diag.sh 30` 可生成不含原始图像的轻量诊断包。
它适合验收当前 Nav2 的时间与 TF；要验收 VSLAM 双目同步，必须先启用左右 IR，
并确认包中出现 `/camera/left_ir/camera_info` 和 `/camera/right_ir/camera_info`。

参考资料：

- [NVIDIA Isaac ROS Visual SLAM release-3.2](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/index.html)
- [NVIDIA RGB-D / cuVSLAM 相机时间与标定要求](https://nvidia-isaac-ros.github.io/v/release-3.2/reference_workflows/isaac_perceptor/run_perceptor_on_rgbd.html)
- [Orbbec Gemini 330 系列数据手册](https://orbbec.com/wp-content/uploads/2024/04/Gemini-330-series-Datasheet-V1.4_20250124.pdf)
- [Orbbec ROS 2 时间域与帧同步参数](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/launch_parameters.html)
