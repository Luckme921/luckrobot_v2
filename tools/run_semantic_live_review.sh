#!/usr/bin/env bash
# Isolated passive camera + local-only review page; does not launch chassis or Nav2.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=100

cleanup() {
  if [[ -n "${rsp_pid:-}" ]]; then
    kill -INT "$rsp_pid" 2>/dev/null || true
    wait "$rsp_pid" 2>/dev/null || true
  fi
  if [[ -n "${camera_pid:-}" ]]; then
    kill -INT "$camera_pid" 2>/dev/null || true
    wait "$camera_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

timeout -s INT 1800s ros2 launch my_robot_description display_robot.launch.py \
  use_rviz:=false &
rsp_pid=$!

timeout -s INT 1800s ros2 launch orbbec_camera gemini_330_series.launch.py \
  enable_point_cloud:=false \
  color_width:=640 color_height:=480 color_fps:=15 \
  depth_width:=640 depth_height:=400 depth_fps:=15 \
  depth_registration:=true align_mode:=SW align_target_stream:=COLOR \
  device_preset:='AMR with IR-Pass' &
camera_pid=$!
sleep 8

timeout -s INT 1790s python3 /home/nvidia/ros2_ws/tools/semantic_live_review.py \
  --model /home/nvidia/semantic_experiments/yoloe-11s-seg.pt \
  --compare-seg /home/nvidia/semantic_experiments/yolo11s-seg.pt \
  --compare-box /home/nvidia/semantic_experiments/yolo11s.pt \
  --initial-model yolo11s-seg \
  --classes person chair table backpack box bottle cup book doll \
    'stuffed animal' 'teddy bear' 'potted plant' \
  --interval 0.25
