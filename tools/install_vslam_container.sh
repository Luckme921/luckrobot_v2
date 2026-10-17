#!/usr/bin/env bash

# Build a persistent Isaac ROS release-3.2 image containing cuVSLAM.
# This script never starts the camera, Nav2, or the chassis.

set -euo pipefail

# NVIDIA's color helper treats a non-capable TERM as a fatal tput error.
if ! tput colors >/dev/null 2>&1; then
  export TERM=xterm-256color
fi

robot_ws="${HOME}/ros2_ws"
isaac_ws="${HOME}/isaac_ros_ws"
common_dir="${isaac_ws}/src/isaac_ros_common"

if [[ ! -x "${common_dir}/scripts/build_image_layers.sh" ]]; then
  echo "错误：未找到 ${common_dir}；请先克隆 isaac_ros_common release-3.2。" >&2
  exit 2
fi

if ros2 node list 2>/dev/null | grep -Eq \
  '^/(controller_server|explore_node|chassis_driver_node|chassis_node)$'; then
  echo "错误：导航、探索或底盘节点仍在运行；为避免资源竞争，本脚本不继续。" >&2
  exit 3
fi

install -m 0644 \
  "${robot_ws}/docker/isaac_ros/Dockerfile.luckrobot_vslam" \
  "${common_dir}/docker/Dockerfile.luckrobot_vslam"
install -m 0644 \
  "${robot_ws}/docker/isaac_ros/luckrobot.isaac_ros_common-config" \
  "${common_dir}/scripts/.isaac_ros_common-config"

echo "构建持久化 LuckRobot cuVSLAM 镜像；首次构建会下载约 29 MB ROS 包。"
"${common_dir}/scripts/build_image_layers.sh" \
  --image_key aarch64.ros2_humble.luckrobot_vslam \
  --image_name isaac_ros_dev-aarch64-luckrobot-vslam

docker image inspect isaac_ros_dev-aarch64-luckrobot-vslam >/dev/null
echo "VSLAM_IMAGE=isaac_ros_dev-aarch64-luckrobot-vslam"
echo "RESULT=PASS"
