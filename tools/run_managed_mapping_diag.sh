#!/usr/bin/env bash
# Records an existing manager; never launches/cancels exploration or Nav2.
set -eo pipefail
# ROS setup scripts intentionally inspect optional unset variables.
set +u
source "$HOME/ros2_ws/tools/robot_env.sh"
set -u
duration="${1:-0}"
[[ "$duration" =~ ^[0-9]+$ ]] || { echo '用法: bash run_managed_mapping_diag.sh [秒数，0为持续记录]'; exit 2; }
prefix="$HOME/managed_mapping_diag_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$prefix"
timeout 10s ros2 node list --no-daemon --spin-time 3 > "$prefix/nodes.txt" 2>&1 || true
if ! grep -Fxq '/map_session_manager' "$prefix/nodes.txt"; then
  echo '未发现 /map_session_manager，请先启动 managed_mapping.launch.py。'
  exit 1
fi
ros2 bag record --include-hidden-topics -o "$prefix/bag" \
  /mapping/status /mapping/navigation_ready /mapping/map_ready /explore/status \
  /map /tf /tf_static /odometry/filtered /wheel/odom /imu/data \
  /plan /local_plan /cmd_vel /cmd_vel_nav /cmd_vel_smoothed \
  /scan_filtered /scan_camera /scan_camera_safety /scan_camera_clear \
  /diagnostics/global_camera_obstacle_grid /map_updates \
  /global_costmap/costmap /global_costmap/costmap_updates \
  /local_costmap/costmap /local_costmap/costmap_updates \
  /navigate_to_pose/_action/status /rosout > "$prefix/recorder.log" 2>&1 &
recorder=$!
finish() {
  trap '' INT TERM
  echo '正在停止录包并打包；不会停止探索或导航。'
  kill -INT "$recorder" 2>/dev/null || true
  wait "$recorder" || true
  for name in session current checkpoint; do
    if [[ -f "$HOME/ros2_ws/map_store/$name.json" ]]; then
      cp "$HOME/ros2_ws/map_store/$name.json" "$prefix/$name.json"
    fi
  done
  tar -czf "$prefix.tar.gz" -C "$(dirname "$prefix")" "$(basename "$prefix")"
  echo "诊断包：$prefix.tar.gz"
  exit 0
}
trap finish INT TERM
sleep 2
kill -0 "$recorder" || { cat "$prefix/recorder.log"; exit 1; }
echo '记录已开始。结束测试后按一次 Ctrl+C；本脚本只录包。'
started=$SECONDS
while kill -0 "$recorder" 2>/dev/null; do
  if (( duration > 0 && SECONDS - started >= duration )); then finish; fi
  sleep 1
done
echo "录包进程提前退出，请查看 $prefix/recorder.log"
exit 1
