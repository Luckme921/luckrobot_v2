#!/usr/bin/env bash

# Stop one LuckRobot navigation/mapping stack before starting a fresh launch.
# This intentionally uses SIGINT only: each node can publish its final zero
# velocity / motor brake and release the serial device cleanly.

set -u

echo "正在停止 LuckRobot 导航、建图与传感器进程；小车将制动。"

echo "===== 清理前相关进程 ====="
pgrep -af -- \
  'smart_navigation.launch.py|robot_bringup.launch.py|navigation.launch.py|laser_filters/scan_to_scan_filter_chain' \
  2>/dev/null || true

# Stop launch parents first so they cannot keep owning or reporting children.
parent_patterns=(
  "ros2 launch my_robot_bringup smart_navigation.launch.py"
  "smart_navigation.launch.py"
  "robot_bringup.launch.py"
  "navigation.launch.py"
)
for pattern in "${parent_patterns[@]}"; do
  pkill -INT -f -- "$pattern" 2>/dev/null || true
done
sleep 2

# A terminal window can disappear without every child leaving.  Stop only the
# executables that belong to this robot stack; do not kill the ROS daemon or
# unrelated user programs.
child_patterns=(
  "/opt/ros/humble/lib/nav2_"
  "/opt/ros/humble/lib/slam_toolbox/"
  "/opt/ros/humble/lib/robot_localization/ekf_node"
  "/opt/ros/humble/lib/laser_filters/scan_to_scan_filter_chain"
  "/opt/ros/humble/lib/depthimage_to_laserscan/depthimage_to_laserscan_node"
  "/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher"
  "/opt/ros/humble/lib/rviz2/rviz2"
  "/home/nvidia/ros2_ws/install/chassis_driver/lib/chassis_driver/chassis_node"
  "/home/nvidia/ros2_ws/install/imu_driver/lib/imu_driver/imu_driver_node"
  "/home/nvidia/ros2_ws/install/rplidar_ros/lib/rplidar_ros/"
  "/home/nvidia/ros2_ws/install/orbbec_camera/lib/orbbec_camera/"
)
for pattern in "${child_patterns[@]}"; do
  pkill -INT -f -- "$pattern" 2>/dev/null || true
done

sleep 5

# Some laser-filter children have been observed surviving when their launch
# terminal is closed.  At this point every intended launch parent has already
# received SIGINT and had time to stop normally, so terminate only the exact
# robot-stack executables that are still alive.  Never use a broad ROS-wide
# kill here: the daemon and unrelated ROS applications remain untouched.
remaining_processes="$(pgrep -af -- \
  '/opt/ros/humble/lib/(laser_filters/scan_to_scan_filter_chain|nav2_|slam_toolbox/|robot_localization/ekf_node|depthimage_to_laserscan/depthimage_to_laserscan_node|robot_state_publisher/robot_state_publisher)|/home/nvidia/ros2_ws/install/(chassis_driver|imu_driver|rplidar_ros|orbbec_camera)/' \
  2>/dev/null || true)"
if [[ -n "$remaining_processes" ]]; then
  echo "===== SIGINT 后仍存活，定向发送 SIGTERM ====="
  echo "$remaining_processes"
  for pattern in "${child_patterns[@]}"; do
    pkill -TERM -f -- "$pattern" 2>/dev/null || true
  done
  sleep 4
fi

# ros2 CLI daemon may retain discovery entries after an uncleanly closed
# process.  Refresh only the CLI daemon after the robot stack has stopped.
timeout 5s ros2 daemon stop >/dev/null 2>&1 || true
sleep 2
timeout 8s ros2 daemon start >/dev/null 2>&1 || true
sleep 3

nodes="$(timeout -s INT 5s ros2 node list 2>/dev/null || true)"
robot_nodes="$(grep -E '^/(controller_server|planner_server|bt_navigator|velocity_smoother|collision_monitor|chassis_driver_node|scan_to_scan_filter_chain|slam_toolbox|ekf_filter_node|rplidar_node|rviz2)$' <<< "$nodes" || true)"
duplicate_nodes="$(grep '^/' <<< "$nodes" | sort | uniq -d || true)"

if [[ -n "$robot_nodes" || -n "$duplicate_nodes" ]]; then
  echo "仍检测到旧节点；不要叠加启动新的 launch："
  [[ -n "$robot_nodes" ]] && echo "$robot_nodes"
  [[ -n "$duplicate_nodes" ]] && {
    echo "同名重复节点："
    echo "$duplicate_nodes"
  }
  echo "===== 仍存活的相关进程 ====="
  pgrep -af -- \
    'smart_navigation.launch.py|robot_bringup.launch.py|navigation.launch.py|laser_filters/scan_to_scan_filter_chain' \
    2>/dev/null || true
  echo "不要启动新的 launch。再次运行本脚本；若仍存在，再重启 Jetson。"
  exit 3
fi

echo "旧导航栈已清理，可以启动一个新的 smart_navigation.launch.py。"
