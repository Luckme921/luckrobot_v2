#!/usr/bin/env bash

# Read-only Isaac ROS cuVSLAM prerequisite audit for LuckRobot V2.
# This script installs nothing and starts no ROS nodes.

set -u
set -o pipefail

stamp="$(date +%Y%m%d_%H%M%S)"
report="$HOME/vslam_environment_${stamp}.txt"
archive="$HOME/vslam_environment_${stamp}.tar.gz"

run_section()
{
  local title="$1"
  shift
  echo "===== ${title} =====" | tee -a "$report"
  timeout -k 2s -s INT 12s "$@" 2>&1 | tee -a "$report" || true
}

: > "$report"
echo "LuckRobot VSLAM 环境只读检查；不会安装软件或启动节点。"

run_section "date" date --iso-8601=ns
run_section "Jetson release" cat /etc/nv_tegra_release
run_section "architecture" uname -a
run_section "JetPack package" dpkg-query -W nvidia-jetpack
run_section "CUDA compiler" nvcc --version
run_section "GPU runtime" nvidia-smi
run_section "memory" free -h
run_section "root storage" df -h /
run_section "NVMe storage" df -h /mnt/nvme
run_section "Docker" docker --version
run_section "NVIDIA container packages" dpkg-query -W 'nvidia-container*' 'docker*' 'containerd*'
run_section "Isaac ROS package" ros2 pkg prefix isaac_ros_visual_slam
run_section "Docker images" docker images --format '{{.Repository}}:{{.Tag}} {{.Size}}'

{
  echo "===== ROS environment ====="
  printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-}"
  printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-}"
  printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-}"
} | tee -a "$report"

tar -czf "$archive" -C "$HOME" "$(basename "$report")"
echo "环境报告：${archive}"
