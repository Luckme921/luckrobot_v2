# 深度扫描量程连续聚类 V12.10.9

## 问题结论

`custom_nav_20260922_193929` 共 302.1 秒。导航无 abort，完整的两个目标成功，
但空场仍反复出现约 0.408 m、4～5 束宽的近距回波。即时安全流允许4束连续点，
因此它可能短暂触发 Collision Monitor；规划确认只检查连续角度索引，近距窄点
与后方墙面相邻时可能合并为不少于7束的簇，继而被代价地图记忆6/16秒。

这不是 `min_points_per_bin=5` 未生效。该参数验证同一角度束中的纵向深度支持；
本次缺口发生在相邻角度束的横向聚类阶段。

## 修改

- `/scan_camera_safety`：SpeckleFilter 从4束提高到6束；相邻量程容差从0.12 m
  收紧到0.08 m。
- `/scan_camera`：世界坐标确认从4帧提高到5帧，0.50秒历史窗口不变。
- 规划扫描聚类新增0.08 m量程连续性：相邻束距离突变时必须拆成两个簇，不能让
  近距假点借用远处墙面束数。
- `min_points_per_bin=5`、高度0.12～0.50 m、局部/全局障碍记忆6/16秒不变。

## 部署

```bash
cd ~/ros2_ws
sha256sum luckrobot_depth_range_cluster_v12_10_9_20260922.zip
unzip -o luckrobot_depth_range_cluster_v12_10_9_20260922.zip -d ~/ros2_ws
sha256sum -c docs/DEPTH_RANGE_CLUSTER_FILTER_V12_10_9_20260922_MANIFEST.sha256

source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select my_robot_bringup \
  --allow-overriding my_robot_bringup
source ~/ros2_ws/install/setup.bash
```

关闭旧 launch 后重新启动：

```bash
bash ~/ros2_ws/tools/stop_navigation_stack.sh
ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping navigation_profile:=luck_full depth_obstacle_mode:=height
```

## 验收

```bash
ros2 param get /camera_scan_confirmation confirmation_frames
ros2 param get /camera_scan_confirmation cluster_range_tolerance
ros2 param get /camera_scan_confirmation min_cluster_beams

python3 ~/ros2_ws/tools/check_depth_height_scan.py \
  --duration 20 --topics /scan_camera_safety /scan_camera
```

预期为5帧、0.08 m、7束。RViz中的青色即时安全点允许偶发出现，但约0.408 m
的4～5束窄点不应继续进入紫色规划流，也不应生成新的膨胀障碍。随后分别测试一个
真实高位平板和一次正常导航，确认真障碍仍能进入两路扫描并触发绕行或减速。
