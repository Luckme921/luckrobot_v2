#!/usr/bin/env bash

# Enter the persistent LuckRobot Isaac ROS development container.

set -euo pipefail

if [[ $(tput colors 2>/dev/null || echo 0) -lt 8 ]]; then
  export TERM=xterm-256color
fi

isaac_ws="${HOME}/isaac_ros_ws"
common_dir="${isaac_ws}/src/isaac_ros_common"

if [[ ! -f "${common_dir}/scripts/.isaac_ros_common-config" ]] ||
   [[ ! -f "${common_dir}/docker/Dockerfile.luckrobot_vslam" ]]; then
  echo "错误：LuckRobot cuVSLAM 镜像配置尚未安装。" >&2
  echo "先执行：bash ~/ros2_ws/tools/install_vslam_container.sh" >&2
  exit 2
fi

if ! docker image inspect isaac_ros_dev-aarch64-luckrobot-vslam >/dev/null 2>&1; then
  echo "错误：持久化 cuVSLAM 镜像不存在。" >&2
  echo "先执行：bash ~/ros2_ws/tools/install_vslam_container.sh" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# run_dev.sh checks LFS after changing into the workspace root, which is not a
# Git checkout in this installation. Point that check at isaac_ros_common while
# retaining the workspace root as the Docker mount.
export GIT_DIR="${common_dir}/.git"
export GIT_WORK_TREE="${common_dir}"
cd "${common_dir}"
exec "${common_dir}/scripts/run_dev.sh" -d "${isaac_ws}" --skip_image_build
