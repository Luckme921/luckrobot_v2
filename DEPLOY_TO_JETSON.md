# Jetson 增量部署说明

本压缩包只包含本轮导航底座改动。它按 `~/ros2_ws` 为根目录保存相对路径；解压时不会删除未列出的文件。

## 1. 本轮文件清单

### 新增文件

- `README.md`
- `DEPLOY_TO_JETSON.md`
- `docs/NAVIGATION_ACCEPTANCE_TEST.md`
- `src/my_robot_bringup/config/box_filter.yaml`
- `src/my_robot_bringup/config/camera_filter.yaml`
- `src/my_robot_bringup/config/collision_monitor.yaml`
- `src/my_robot_bringup/launch/smart_navigation.launch.py`
- `src/my_robot_mapping/config/mapper_params_localization.yaml`

### 修改文件

- `order.txt`
- `ICP_Calibration/auto_icp_calibrator.py`
- `src/chassis_driver/src/chassis_node.cpp`
- `src/imu_driver/package.xml`
- `src/my_robot_description/package.xml`
- `src/my_robot_bringup/launch/robot_bringup.launch.py`
- `src/my_robot_bringup/launch/test_bringup.launch.py`
- `src/my_robot_bringup/package.xml`
- `src/nav_config/camera_filter.yaml`
- `src/my_robot_mapping/config/mapper_params_online_async.yaml`
- `src/my_robot_mapping/package.xml`
- `src/my_robot_navigation/CMakeLists.txt`
- `src/my_robot_navigation/package.xml`
- `src/my_robot_navigation/config/nav2_params.yaml`
- `src/my_robot_navigation/launch/navigation.launch.py`
- `src/my_robot_navigation/scripts/vacuum_manager.py`

## 2. 覆盖前备份

先停止所有 ROS 2 launch，确保底盘不会运动，然后在 Jetson 执行：

```bash
cd ~/ros2_ws
stamp=$(date +%Y%m%d_%H%M%S)
tar -czf ~/luckrobot_before_nav_update_${stamp}.tar.gz \
  src/chassis_driver \
  src/imu_driver \
  src/my_robot_description \
  src/my_robot_bringup \
  src/my_robot_mapping \
  src/my_robot_navigation \
  src/nav_config \
  ICP_Calibration \
  order.txt
```

备份保存在 `/home/nvidia/luckrobot_before_nav_update_时间.tar.gz`，不包含 `build/`、`install/` 和地图。

## 3. 上传、核对和覆盖

用 MobaXterm 把 `luckrobot_navigation_update_20260829.zip` 上传到 `/home/nvidia/ros2_ws/`，然后执行：

```bash
cd ~/ros2_ws
unzip -l luckrobot_navigation_update_20260829.zip
unzip -o luckrobot_navigation_update_20260829.zip -d ~/ros2_ws
```

## 4. 安装依赖并增量编译

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  chassis_driver \
  imu_driver \
  my_robot_description \
  my_robot_mapping \
  my_robot_navigation \
  my_robot_bringup
source install/setup.bash
```

若 `collision_monitor` 不存在，再安装：

```bash
sudo apt update
sudo apt install ros-humble-nav2-collision-monitor
```

不要在首轮同时清空整个 `build/`、`install/`、`log/`，也不要同时切换 DDS；先保留现有基线做增量验证。

## 5. 第一轮无运动检查

先不要让车轮落地，不发送 `/cmd_vel`：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_bringup smart_navigation.launch.py --show-args
```

确认能列出至少 `mode`、`posegraph`、`start_at_dock`、`use_collision_monitor`、`use_rviz2` 后，再进入 `docs/NAVIGATION_ACCEPTANCE_TEST.md` 的分阶段测试。
