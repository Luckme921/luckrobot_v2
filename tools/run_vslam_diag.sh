#!/usr/bin/env bash
set -u

duration="${1:-90}"
if ! [[ "$duration" =~ ^[1-9][0-9]*$ ]]; then
  echo "用法: bash tools/run_vslam_diag.sh [秒数]" >&2
  exit 2
fi

workspace="${LUCKROBOT_WS:-$HOME/ros2_ws}"
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$workspace/install/setup.bash" ]]; then
  echo "错误：未找到 ROS 2 Humble 或 $workspace/install/setup.bash" >&2
  exit 2
fi

# ROS 2 Humble setup files legitimately probe variables before defining them.
# Temporarily disable nounset so this script also works when launched directly.
set +u
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
set -u

stamp=$(date +%Y%m%d_%H%M%S)
prefix="$HOME/vslam_diag_${stamp}"
bag_dir="${prefix}_bag"
state_log="${prefix}_state.log"
record_log="${prefix}_record.log"

finish_recording()
{
  local state=""
  local attempt=0
  if [[ -n "${bag_pid:-}" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    kill -INT "$bag_pid" 2>/dev/null || true
    # rosbag2 may occasionally ignore SIGINT while closing sqlite.  Never let
    # the diagnostic wrapper block forever: allow 8 s, then TERM, then KILL.
    for attempt in $(seq 1 40); do
      state=$(ps -o stat= -p "$bag_pid" 2>/dev/null | tr -d ' ' || true)
      if [[ -z "$state" || "$state" == Z* ]]; then
        break
      fi
      sleep 0.2
    done
    state=$(ps -o stat= -p "$bag_pid" 2>/dev/null | tr -d ' ' || true)
    if [[ -n "$state" && "$state" != Z* ]]; then
      echo "rosbag 在 SIGINT 后未退出；发送 SIGTERM。" >&2
      kill -TERM "$bag_pid" 2>/dev/null || true
      sleep 1
    fi
    state=$(ps -o stat= -p "$bag_pid" 2>/dev/null | tr -d ' ' || true)
    if [[ -n "$state" && "$state" != Z* ]]; then
      echo "rosbag 仍未退出；发送 SIGKILL 以保证能够打包。" >&2
      kill -KILL "$bag_pid" 2>/dev/null || true
    fi
    wait "$bag_pid" 2>/dev/null || true
  fi
}

interrupted=0
handle_interrupt()
{
  interrupted=1
  echo
  echo "收到中断；正在安全停止 rosbag，随后仍会生成压缩包。"
  # A second Ctrl-C used to interrupt sqlite finalization and skip tar.  The
  # bounded cleanup below makes it safe to ignore further signals briefly.
  trap '' INT TERM
}
trap finish_recording EXIT
trap handle_interrupt INT TERM

echo "VSLAM 轻量诊断：${duration} 秒"
echo "可用 Nav2 连续测试多个目标；本脚本不发送速度，也不录原始图像。"
echo "必须先启动本脚本再运动；若重启相机，也必须重启容器内 cuVSLAM。"
echo "输出前缀：$prefix"

# 先录包，再做任何 ROS 图检查。旧版先检查约 90 秒，恰好漏掉了检查期间
# 发生的里程计/TF 故障；现在从脚本启动的第一刻开始保留证据。
ros2 bag record -o "$bag_dir" \
  /tf \
  /tf_static \
  /visual_slam/tracking/odometry \
  /visual_slam/tracking/vo_pose_covariance \
  /visual_slam/tracking/odometry_validated \
  /vslam/health \
  /visual_slam/tracking/vo_path \
  /diagnostics \
  /cmd_vel_nav \
  /cmd_vel \
  /cmd_vel_safe \
  /cmd_vel_vslam_test \
  /wheel_feedback \
  /wheel/odom \
  /imu/data \
  /odometry/filtered \
  /odometry/vslam_candidate \
  /rosout \
  < /dev/null > "$record_log" 2>&1 &
bag_pid=$!
record_start_epoch=$(date +%s)

sleep 2
if ! kill -0 "$bag_pid" 2>/dev/null || [[ ! -d "$bag_dir" ]]; then
  echo "错误：rosbag 未启动。" >&2
  tail -n 40 "$record_log" >&2 || true
  exit 3
fi

echo "rosbag 已启动；正在采集轻量状态，期间也可以按测试步骤运动。"

{
  echo "===== date ====="
  date --iso-8601=ns
  echo "===== installed packages ====="
  timeout -s INT 3s ros2 pkg prefix isaac_ros_visual_slam 2>&1 || true
  timeout -s INT 3s ros2 pkg prefix orbbec_camera 2>&1 || true
  echo "===== relevant topics ====="
  timeout -s INT 3s ros2 topic list | \
    grep -E '^/(visual_slam|vslam/stereo|diagnostics|wheel/odom|imu/data|odometry/|tf)' || true
  echo "===== stereo monotonic gate ====="
  timeout -s INT 3s ros2 node info /vslam_stereo_monotonic_gate 2>&1 || true
  echo "===== VSLAM endpoints ====="
  timeout -s INT 3s ros2 topic info /visual_slam/tracking/odometry -v 2>&1 || true
  timeout -s INT 3s ros2 topic info /visual_slam/tracking/odometry_validated -v 2>&1 || true
  timeout -s INT 3s ros2 topic echo /vslam/health --once 2>&1 || true
  echo "===== camera VSLAM frame mode ====="
  for parameter in \
    enable_laser \
    interleave_frame_enable \
    enable_sequence_id_filter \
    sequence_id_filter_id
  do
    timeout -s INT 2s ros2 param get /camera/camera "$parameter" 2>&1 || true
  done
  echo "===== TF ownership ====="
  timeout -s INT 3s ros2 topic info /tf -v 2>&1 || true
  echo "===== optical TF ====="
  timeout -s INT 3s ros2 run tf2_ros tf2_echo \
    base_footprint camera_left_ir_optical_frame 2>&1 || true
  timeout -s INT 3s ros2 run tf2_ros tf2_echo \
    base_footprint camera_right_ir_optical_frame 2>&1 || true
} > "$state_log"

elapsed=$(( $(date +%s) - record_start_epoch ))
remaining=$(( duration - elapsed ))
last_progress_epoch=$(date +%s)
while (( remaining > 0 )) && (( interrupted == 0 )); do
  sleep 1 || true
  now_epoch=$(date +%s)
  elapsed=$(( now_epoch - record_start_epoch ))
  remaining=$(( duration - elapsed ))
  if (( remaining > 0 && now_epoch - last_progress_epoch >= 15 )); then
    echo "仍在记录，剩余约 ${remaining} 秒……"
    last_progress_epoch=$now_epoch
  fi
done
finish_recording
bag_pid=""

tar -czf "${prefix}.tar.gz" \
  -C "$HOME" \
  "$(basename "$bag_dir")" \
  "$(basename "$state_log")" \
  "$(basename "$record_log")"

trap - EXIT INT TERM
echo "诊断包：${prefix}.tar.gz"
