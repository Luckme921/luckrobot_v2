#!/usr/bin/env bash

# Record one RViz NavigateToPose A/B run for LuckController/LuckPlanner.
# Usage: bash ~/ros2_ws/tools/run_custom_nav_diag.sh [seconds]

set -u

duration="${1:-90}"
stamp="$(date +%Y%m%d_%H%M%S)"
diag="$HOME/custom_nav_${stamp}"
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
        "$(basename "${diag}_planner_params.yaml")" \
        "$(basename "${diag}_bag.log")"
      echo "诊断包：${diag}.tar.gz"
    else
      echo "错误：rosbag 目录未生成；请上传 ${diag}_bag.log"
    fi
  fi
}

trap 'finish_recording; exit 130' INT TERM

controller_state="$(ros2 lifecycle get /controller_server 2>&1 || true)"
planner_state="$(ros2 lifecycle get /planner_server 2>&1 || true)"
if ! grep -q 'active' <<< "$controller_state" || ! grep -q 'active' <<< "$planner_state"; then
  echo "拒绝记录：controller/planner 尚未同时 active。"
  echo "$controller_state"
  echo "$planner_state"
  exit 3
fi
if ros2 node list 2>/dev/null | grep -Fxq /explore_node; then
  echo "拒绝记录：先关闭 /explore_node，本轮只做手动单目标 A/B。"
  exit 4
fi

ros2 param dump /controller_server > "${diag}_controller_params.yaml" 2>&1 || true
ros2 param dump /planner_server > "${diag}_planner_params.yaml" 2>&1 || true

{
  echo "===== date ====="
  date --iso-8601=ns
  echo "===== lifecycle ====="
  echo "controller: $controller_state"
  echo "planner: $planner_state"
  echo "===== loaded algorithms ====="
  ros2 param get /controller_server controller_plugins 2>&1 || true
  ros2 param get /controller_server LuckFollowPath.plugin 2>&1 || true
  ros2 param get /planner_server planner_plugins 2>&1 || true
  ros2 param get /planner_server LuckGrid.plugin 2>&1 || true
  echo "===== default BT ====="
  ros2 param get /bt_navigator default_nav_to_pose_bt_xml 2>&1 || true
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
  /luck_controller/lookahead \
  /luck_controller/tracking_error \
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

echo "记录已开始。只发送一个目标；先测试 0.8~1.2 m 开阔直线，再测试原窄通道。"
echo "不要启动自动探索，不要同时修改参数。将记录 ${duration} 秒。"
remaining="$duration"
while (( remaining > 0 )); do
  step=15
  if (( remaining < step )); then
    step="$remaining"
  fi
  sleep "$step"
  remaining=$((remaining - step))
  if (( remaining > 0 )); then
    echo "仍在记录，剩余约 ${remaining} 秒……"
  fi
done

{
  echo "===== map pose after ====="
  timeout -s INT 5s ros2 run tf2_ros tf2_echo map base_footprint || true
  echo "===== date after ====="
  date --iso-8601=ns
} >> "${diag}_state.log" 2>&1

finish_recording
trap - INT TERM
