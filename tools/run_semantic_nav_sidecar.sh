#!/usr/bin/env bash
# Passive semantic observer for an already-running production navigation launch.
# It neither starts the camera/chassis nor publishes velocity or Nav2 goals.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=100

nodes="$(ros2 node list)"
if ! grep -qx '/camera/camera' <<< "$nodes"; then
  echo 'Production camera node is absent; refusing to start semantic sidecar.' >&2
  exit 2
fi

timeout -s INT 600s python3 /home/nvidia/ros2_ws/tools/semantic_live_review.py \
  --model /home/nvidia/semantic_experiments/yoloe-11s-seg.pt \
  --compare-seg /home/nvidia/semantic_experiments/yolo11s-seg.pt \
  --compare-box /home/nvidia/semantic_experiments/yolo11s.pt \
  --initial-model yolo11s-seg \
  --allow-raw-depth \
  --classes person chair table backpack box bottle cup book doll \
    'stuffed animal' 'teddy bear' 'potted plant' \
  --interval 0.5 --observation-interval 0.5
