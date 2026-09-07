#!/usr/bin/env bash
set -u

duration="${1:-30}"
if ! [[ "$duration" =~ ^[1-9][0-9]*$ ]]; then
  echo "用法: bash tools/run_sensor_alignment_diag.sh [秒数]" >&2
  exit 2
fi

workspace="${LUCKROBOT_WS:-$HOME/ros2_ws}"
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$workspace/install/setup.bash" ]]; then
  echo "错误：未找到 ROS 2 Humble 或 $workspace/install/setup.bash" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"

stamp=$(date +%Y%m%d_%H%M%S)
prefix="$HOME/sensor_alignment_${stamp}"
bag_dir="${prefix}_bag"
state_log="${prefix}_state.log"
bag_log="${prefix}_record.log"

finish_recording()
{
  if [[ -n "${bag_pid:-}" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    kill -INT "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
  fi
}
trap finish_recording EXIT INT TERM

{
  echo "===== date ====="
  date --iso-8601=ns
  echo "===== relevant topics ====="
  ros2 topic list | grep -E '^/(tf|tf_static|imu/data|wheel/odom|odometry/filtered|scan_filtered|scan_camera|camera/(depth|left_ir|right_ir)/)' || true
  echo "===== camera node parameters ====="
  timeout -s INT 8s ros2 param get /camera/camera enable_frame_sync || true
  timeout -s INT 8s ros2 param get /camera/camera time_domain || true
  timeout -s INT 8s ros2 param get /camera/camera enable_sync_host_time || true
  timeout -s INT 8s ros2 param get /camera/camera enable_left_ir || true
  timeout -s INT 8s ros2 param get /camera/camera enable_right_ir || true
  timeout -s INT 8s ros2 param get /camera/camera left_ir_fps || true
  timeout -s INT 8s ros2 param get /camera/camera right_ir_fps || true
  echo "===== TF samples ====="
  timeout -s INT 5s ros2 run tf2_ros tf2_echo base_link laser || true
  timeout -s INT 5s ros2 run tf2_ros tf2_echo base_link camera_depth_frame || true
  timeout -s INT 5s ros2 run tf2_ros tf2_echo base_link imu_link || true
} > "$state_log" 2>&1

echo "传感器对齐诊断：${duration} 秒（只录小消息，不录原始图像）"
echo "请让机器人静止 5 秒、原地缓慢旋转约 90°、再静止 5 秒。"
echo "输出前缀：$prefix"

ros2 bag record -o "$bag_dir" \
  /tf \
  /tf_static \
  /imu/data \
  /wheel/odom \
  /odometry/filtered \
  /scan_filtered \
  /scan_camera \
  /camera/depth/camera_info \
  /camera/left_ir/camera_info \
  /camera/right_ir/camera_info \
  /rosout \
  < /dev/null > "$bag_log" 2>&1 &
bag_pid=$!

sleep 2
if ! kill -0 "$bag_pid" 2>/dev/null || [[ ! -d "$bag_dir" ]]; then
  echo "错误：rosbag 未启动。" >&2
  tail -n 30 "$bag_log" >&2 || true
  exit 3
fi

timeout -s INT "${duration}s" tail --pid="$bag_pid" -f /dev/null || true
finish_recording
bag_pid=""

tar -czf "${prefix}.tar.gz" \
  -C "$HOME" \
  "$(basename "$bag_dir")" \
  "$(basename "$state_log")" \
  "$(basename "$bag_log")"

echo "诊断包：${prefix}.tar.gz"
echo "注意：若 left_ir/right_ir 不存在，本包只能验收当前 Nav2，不能验收 VSLAM 双目同步。"
