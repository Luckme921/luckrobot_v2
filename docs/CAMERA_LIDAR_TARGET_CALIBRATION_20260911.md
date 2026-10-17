# LuckRobot V2 相机—2D 雷达靶标外参标定

## 结论和边界

相机升高 203 mm 后，CAD 高度可作为初值，但装配带来的 yaw、pitch、roll 小偏差不能靠
肉眼验收。`ICP_Calibration/auto_icp_calibrator.py` 只匹配 `/scan_filtered` 与
`/scan_camera` 两条二维扫描；它只输出 X/Y/Yaw 残差，而且两条扫描位于不同高度，普通
家具和椅子会破坏对应关系。因此它只能在两者都看到同一片竖直墙面时作为复核工具，
不得直接把一次 ICP 结果写进 URDF。

本项目采用两阶段流程：

1. 用已知尺寸的竖直棋盘背板，同时采集彩色图、深度图、2D 雷达线和 TF；用多姿态
   靶标求 `laser -> camera` 外参，并以 CAD 的 Z 高度作为受约束初值。
2. 将候选外参写入测试分支后，再用不同墙角的二维 ICP、深度点云投影和原地旋转地图
   重影做独立验收。未通过验收不覆盖稳定导航基线。

相机内部标定、左右 IR 的双目基线不会因整机支架刚性平移而改变。需要重做的是
`base_link/laser -> camera_link` 的整机外参。纯双目 cuVSLAM 可以先观察运行；一旦把
VSLAM 位姿转换为底盘运动或融合进 EKF，错误外参会把相机旋转杆臂运动误认为底盘平移，
因此必须在融合前完成外参验收。

## 标定靶要求

推荐使用刚性、完全平整的竖直背板。当前实物为 **Q12-150-10**：

- 标称 12 × 9 是**方格数量**，求解器使用的内部角点为 **11 × 8**；单格标称
  10 mm，板约 150 × 150 mm。
- 用卡尺复测单格边长、整板宽高和厚度。求解时以实测值为准。
- “背板下边缘”指整块 150 × 150 mm 实体板的外轮廓下边，不是黑白棋盘格区域的
  下边。它与黑白格外围留白多少无关。
- 若让约 109.5 mm 高的 A2M12 扫描线直接穿过背板，应把实体板下边固定在扫描平面
  以下并准确记录高度；若实体板下边像本次实测约 200 mm，雷达实际看到的是背后的
  同一面平墙。此时求解器使用“相机棋盘正面 + 正面到墙面的距离补偿”模型，
  `board_bottom_mm` 只作记录，不进入外参方程，不要求毫米级测量。
- 墙面补偿距离不是只看板材标称厚度，而是从棋盘印刷正面到墙面的总距离；若背面有
  双面胶、垫片或局部悬空，必须计入。这个数值和单格实际边长会影响毫米级平移结果，
  但采完后可以通过求解器参数修正，无需重拍。
- 墙面模式要求背板紧贴同一片平墙、不能翘曲或晃动；板在墙面内旋转 10～20°不会
  破坏标定。若所谓 10～20°是板面从墙上翘起，则不能使用这组样本。
- 该棋盘较小，采集距离应为 **0.45～0.65 m**。超过约 0.7 m 后10 mm方格在
  640像素图像中太小，角点精度会明显下降。
- 背板不能弯曲或手持晃动。可制作带刻度的直角支架保证竖直，机器人全程不动，只
  移动棋盘支架。玻璃反光时调整环境照明，不要让高亮覆盖内角点。

## 12 组姿态

每组放稳后等待至少 2 秒再采集。推荐顺序如下：

| 组 | 距相机约 | 水平位置 | 背板相对机器人 |
|---:|---:|---|---|
| 1–3 | 0.45 m | 左、中、右 | 正对 |
| 4–6 | 0.55 m | 左、中、右 | 正对 |
| 7–8 | 0.55 m | 中 | 左转/右转约 15° |
| 9–10 | 0.65 m | 左、右 | 左转/右转约 20° |
| 11–12 | 0.50～0.60 m | 任意可见位置 | 作为独立验证组 |

“距离”和“角度”不要求手工量得很准，算法会从图像、深度和雷达估计；重要的是姿态要
有明显变化、整块棋盘在彩色与深度视野内，而且雷达确实击中背板。

## 采集命令

先启动当前 mapping 导航栈，但不要发目标、不要启动自动探索：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch my_robot_bringup smart_navigation.launch.py \
  mode:=mapping navigation_profile:=luck_full
```

在第二个终端确认原始数据存在：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

for topic in \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/depth/image_raw \
  /camera/depth/camera_info \
  /scan /scan_filtered /scan_camera
do
  printf '%-38s ' "$topic"
  timeout -s INT 4s ros2 topic hz "$topic" 2>&1 | tail -1
done
```

如果五个必要话题（彩色图、深度图、两份 CameraInfo、`/scan`）都存在，开始交互采集：

```bash
python3 -c 'import rclpy, numpy, cv2; print("calibration Python dependencies: OK")'
```

若只有 `cv2` 缺失，再安装 Ubuntu 仓库版本，不要使用 pip 覆盖 ROS 的 OpenCV：

```bash
sudo apt update
sudo apt install -y python3-opencv
```

然后开始交互采集：

```bash
python3 ~/ros2_ws/tools/capture_camera_lidar_calibration.py \
  --poses 12 \
  --checkerboard-cols 11 \
  --checkerboard-rows 8 \
  --square-size-mm 10 \
  --board-width-mm 150 \
  --board-height-mm 150 \
  --board-thickness-mm 实测厚度 \
  --board-bottom-mm 80
```

每移动一次背板并放稳，按一次 Enter。程序仅保存一组压缩快照，不会产生数 GB 的视频
rosbag。完成后上传：

```text
~/camera_lidar_calib_YYYYMMDD_HHMMSS.tar.gz
```

同时反馈以下文字和照片：

1. 棋盘内角点“列 × 行”；
2. 卡尺实测单格边长（mm）；
3. 背板宽 × 高（mm）以及背板底边离地高度；
4. 棋盘四边到背板四边的距离；
5. 正面拍摄整块背板的照片、机器人和背板同框的侧视照片；
6. 12 组中哪些是左/中/右、近/中/远、左偏/右偏。

## 墙面模式离线求解

当雷达扫描线低于实体背板时，运行以下命令。该工具只生成候选报告，不修改 URDF：

```bash
python3 ~/ros2_ws/tools/solve_camera_lidar_wall_calibration.py \
  ~/camera_lidar_calib_YYYYMMDD_HHMMSS \
  --square-size-mm 10.0 \
  --wall-offset-mm 3.0
```

默认将最后两组留作独立验证，并读取采集元数据中的背板厚度。输出
`wall_calibration_candidate.json` 和质量表；只有训练组和留出组误差都降低、外参改变量
符合装配误差量级，并通过投影复核，才允许生成 URDF 测试补丁。

收到数据后才进入求解、留出组验证和 URDF 候选补丁。不要先运行旧 ICP 并把它的残差
直接加到 URDF。
