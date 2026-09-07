#!/usr/bin/env bash

# Record one RViz/manual NavigateToPose test without starting explore_lite.
# Usage: bash ~/ros2_ws/tools/run_manual_nav_diag.sh [seconds]

set -u

duration="${1:-75}"
stamp="$(date +%Y%m%d_%H%M%S)"
diag="$HOME/manual_nav_${stamp}"
bag_pid=""
finished=0

finish_recording()
{
  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    kill -INT "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" || true
  fi

  if [[ "$finished" -eq 0 ]]; then
    finished=1
    if [[ -d "$diag" ]]; then
      tar -czf "${diag}.tar.gz" \
        -C "$HOME" \
        "$(basename "$diag")" \
        "$(basename "${diag}_state.log")" \
        "$(basename "${diag}_controller_params.yaml")" \
        "$(basename "${diag}_collision_params.yaml")" \
        "$(basename "${diag}_bag.log")"
      echo "诊断包：${diag}.tar.gz"
    else
      echo "错误：rosbag 目录未生成；请上传 ${diag}_bag.log"
    fi
  fi
}

trap 'finish_recording; exit 130' INT TERM

controller_state="$(ros2 lifecycle get /controller_server 2>&1 || true)"
if ! grep -q 'active' <<< "$controller_state"; then
  echo "拒绝记录：controller_server 尚未 active：$controller_state"
  exit 3
fi

if ros2 node list 2>/dev/null | grep -Fxq /explore_node; then
  echo "拒绝记录：检测到 /explore_node。关闭自动探索后再测试手动导航。"
  exit 4
fi

ros2 param dump /controller_server > "${diag}_controller_params.yaml" 2>&1 || true
if ros2 node list 2>/dev/null | grep -Fxq /collision_monitor; then
  ros2 param dump /collision_monitor > "${diag}_collision_params.yaml" 2>&1 || true
else
  echo "collision_monitor not running" > "${diag}_collision_params.yaml"
fi

{
  echo "===== date ====="
  date --iso-8601=ns
  echo "===== controller lifecycle ====="
  echo "$controller_state"
  echo "===== nodes ====="
  ros2 node list
  echo "===== velocity endpoints ====="
  for topic in /cmd_vel_nav /cmd_vel /cmd_vel_safe /odom /odometry/filtered; do
    echo "----- $topic -----"
    ros2 topic info "$topic" -v 2>&1 || true
  done
  echo "===== map pose before ====="
  timeout -s INT 5s ros2 run tf2_ros tf2_echo map base_footprint || true
} > "${diag}_state.log" 2>&1

ros2 bag record \
  --include-hidden-topics \
  -o "$diag" \
  /cmd_vel_nav \
  /cmd_vel \
  /cmd_vel_safe \
  /wheel_feedback \
  /wheel/odom \
  /odom \
  /odometry/filtered \
  /imu/data \
  /scan_filtered \
  /scan_camera \
  /tf \
  /tf_static \
  /map \
  /map_updates \
  /local_costmap/costmap \
  /local_costmap/costmap_updates \
  /global_costmap/costmap \
  /global_costmap/costmap_updates \
  /plan \
  /local_plan \
  /navigate_to_pose/_action/feedback \
  /navigate_to_pose/_action/status \
  /behavior_tree_log \
  /rosout \
  < /dev/null > "${diag}_bag.log" 2>&1 &

bag_pid=$!
sleep 3

if ! kill -0 "$bag_pid" 2>/dev/null; then
  echo "错误：rosbag 启动失败。"
  finish_recording
  exit 5
fi

echo "记录已开始。现在只在 RViz 发送一个本轮需要复现的目标；不要启动自动探索。"
echo "窄通道诊断请发送同一个长距离目标，基础运动诊断可发送约 0.30 m 正前方目标。"
echo "将继续记录 ${duration} 秒。"
sleep "$duration"

{
  echo "===== map pose after ====="
  timeout -s INT 5s ros2 run tf2_ros tf2_echo map base_footprint || true
  echo "===== date after ====="
  date --iso-8601=ns
} >> "${diag}_state.log" 2>&1

finish_recording
trap - INT TERM
