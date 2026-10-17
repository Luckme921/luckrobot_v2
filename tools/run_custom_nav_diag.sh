#!/usr/bin/env bash

# Record one RViz NavigateToPose A/B run for LuckController/LuckPlanner.
# Usage: bash ~/ros2_ws/tools/run_custom_nav_diag.sh [seconds]
# Use 0 to record until Ctrl-C.  That mode is preferred for an intermittent
# stall because the bag cannot expire immediately before the failure.

set -u

duration="${1:-90}"
if ! [[ "$duration" =~ ^[0-9]+$ ]]; then
  echo "用法：$0 [非负整数秒数]；0 表示一直记录到 Ctrl-C。"
  exit 2
fi
stamp="$(date +%Y%m%d_%H%M%S)"
diag="$HOME/custom_nav_${stamp}"
bag_pid=""
finished=0

ros_timeout()
{
  local seconds="$1"
  shift
  timeout -s INT "$seconds" "$@"
}

finish_recording()
{
  # One Ctrl-C is enough. Ignore additional terminal interrupts while rosbag
  # flushes and tar is being written, otherwise repeated Ctrl-C can leave only
  # *_bag.log without the requested diagnostic archive.
  trap '' INT TERM
  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    echo "收到停止请求；正在让 rosbag 落盘并打包，请不要重复按 Ctrl-C……"
    kill -INT "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" || true
  fi
  if [[ "$finished" -eq 0 ]]; then
    finished=1
    if [[ -d "$diag" ]]; then
      {
        echo "===== map pose after ====="
        ros_timeout 5s ros2 run tf2_ros tf2_echo map base_footprint || true
        echo "===== date after ====="
        date --iso-8601=ns
      } >> "${diag}_state.log" 2>&1
      tar -czf "${diag}.tar.gz" \
        -C "$HOME" \
        "$(basename "$diag")" \
        "$(basename "${diag}_state.log")" \
        "$(basename "${diag}_controller_params.yaml")" \
        "$(basename "${diag}_planner_params.yaml")" \
        "$(basename "${diag}_slam_params.yaml")" \
        "$(basename "${diag}_bag.log")"
      echo "诊断包：${diag}.tar.gz"
    else
      echo "错误：rosbag 目录未生成；请上传 ${diag}_bag.log"
    fi
  fi
}

trap 'finish_recording; exit 130' INT TERM

echo "自定义导航诊断：${duration} 秒"
echo "输出前缀：${diag}"
echo "每项 ROS 服务都有超时；若节点繁忙，不会再无限卡在参数读取。"
echo "[1/5] 检查必要节点（生命周期单项最多 3 秒）……"
node_list="$(ros_timeout 4s ros2 node list 2>/dev/null || true)"
duplicate_nodes="$(grep '^/' <<< "$node_list" | sort | uniq -d || true)"
if [[ -n "$duplicate_nodes" ]]; then
  echo "拒绝记录：ROS 图中存在同名重复节点，数据来源不唯一："
  echo "$duplicate_nodes"
  echo "请先关闭所有旧导航终端并运行 tools/stop_navigation_stack.sh。"
  exit 4
fi
required_nodes=(
  /controller_server
  /planner_server
  /velocity_smoother
  /bt_navigator
  /collision_monitor
)
for node in "${required_nodes[@]}"; do
  if ! grep -Fxq "$node" <<< "$node_list"; then
    echo "拒绝记录：ROS 图中缺少必要节点 ${node}。"
    exit 3
  fi
done
if grep -Fxq /explore_node <<< "$node_list"; then
  echo "拒绝记录：先关闭 /explore_node，本轮只做手动单目标 A/B。"
  exit 4
fi

controller_state="$(ros_timeout 3s ros2 lifecycle get /controller_server 2>&1 || true)"
planner_state="$(ros_timeout 3s ros2 lifecycle get /planner_server 2>&1 || true)"
smoother_state="$(ros_timeout 3s ros2 lifecycle get /velocity_smoother 2>&1 || true)"
bt_state="$(ros_timeout 3s ros2 lifecycle get /bt_navigator 2>&1 || true)"
collision_state="$(ros_timeout 3s ros2 lifecycle get /collision_monitor 2>&1 || true)"
lifecycle_acceptable()
{
  local node="$1"
  local state="$2"
  if grep -Eq '^active \[[0-9]+\]\r?$' <<< "$state"; then
    return 0
  fi
  if grep -Eq 'inactive|unconfigured|finalized|Node not found' <<< "$state"; then
    echo "拒绝记录：${node} 返回了明确的非 active 状态：${state}"
    return 1
  fi
  echo "警告：${node} 生命周期服务无响应，但节点仍在 ROS 图中；继续录包以捕获该异常。"
  return 0
}
if ! lifecycle_acceptable /controller_server "$controller_state" ||
  ! lifecycle_acceptable /planner_server "$planner_state" ||
  ! lifecycle_acceptable /velocity_smoother "$smoother_state" ||
  ! lifecycle_acceptable /bt_navigator "$bt_state" ||
  ! lifecycle_acceptable /collision_monitor "$collision_state"
then
  exit 3
fi

# A running safety stream alone is not enough: Nav2 costmaps intentionally
# subscribe to the confirmed planning stream. Refuse a misleading recording
# if either half of the V12.10.5 dual-channel chain has no publisher.
camera_safety_info="$(ros_timeout 4s ros2 topic info /scan_camera_safety 2>&1 || true)"
camera_plan_info="$(ros_timeout 4s ros2 topic info /scan_camera 2>&1 || true)"
if ! grep -Eq 'Publisher count: [1-9][0-9]*' <<< "$camera_safety_info"; then
  echo "拒绝记录：/scan_camera_safety 没有发布者；高度深度安全链未启动。"
  echo "$camera_safety_info"
  exit 3
fi
if ! grep -Eq 'Publisher count: [1-9][0-9]*' <<< "$camera_plan_info"; then
  echo "拒绝记录：/scan_camera 没有发布者；camera_scan_confirmation 未运行。"
  echo "请先检查启动终端中的 scan_confirmation_filter 异常。"
  echo "$camera_plan_info"
  exit 3
fi

# Topic publishers alone did not catch the 2026-10-17 failure: EKF stopped
# publishing while its process remained alive. The confirmation filter then
# stamped /scan_camera with a frozen TF, and the chassis heartbeat expired.
echo "检查 EKF、地图 TF 和规划扫描的实时性（最多约 5 秒）……"
if ! python3 "$HOME/ros2_ws/tools/check_nav_tf_health.py" --duration 5; then
  echo "拒绝记录：定位/TF/相机规划扫描过期。请停止导航并检查 EKF；不要发送目标。"
  exit 3
fi

echo "[2/5] 保存控制器与规划器参数（单项最多 6 秒）……"
ros_timeout 6s ros2 param dump /controller_server \
  > "${diag}_controller_params.yaml" 2>&1 || true
ros_timeout 6s ros2 param dump /planner_server \
  > "${diag}_planner_params.yaml" 2>&1 || true
ros_timeout 6s ros2 param dump /slam_toolbox \
  > "${diag}_slam_params.yaml" 2>&1 || true

echo "[3/5] 保存算法、行为树和起始位姿（单项最多 5 秒）……"
{
  echo "===== date ====="
  date --iso-8601=ns
  echo "===== lifecycle ====="
  echo "controller: $controller_state"
  echo "planner: $planner_state"
  echo "velocity_smoother: $smoother_state"
  echo "bt_navigator: $bt_state"
  echo "collision_monitor: $collision_state"
  echo "===== selected parameters from dumps ====="
  grep -E \
    'controller_plugins:|LuckFollowPath:|plugin:|rotate_braking_decel:|rotation_breakaway_|execution_feedback_imu_topic:|path_heading_|angular_damping_gain:|in_place_avoidance_|action_resume_gap:|max_trajectory_center_cost:' \
    "${diag}_controller_params.yaml" 2>&1 || true
  grep -E 'planner_plugins:|LuckGrid:|plugin:|max_planning_cost:|cost_penalty:|smoothing_max_cost:' \
    "${diag}_planner_params.yaml" 2>&1 || true
  echo "===== selected SLAM parameters ====="
  grep -E \
    'ceres_loss_function:|map_update_interval:|minimum_travel_distance:|link_match_minimum_response_fine:|link_scan_maximum_distance:|loop_match_minimum_|tf_buffer_duration:|enable_interactive_mode:' \
    "${diag}_slam_params.yaml" 2>&1 || true
  echo "===== default BT ====="
  ros_timeout 5s ros2 param get /bt_navigator default_nav_to_pose_bt_xml 2>&1 || true
  echo "===== velocity chain endpoints ====="
  for topic in /cmd_vel_nav /cmd_vel /cmd_vel_safe /wheel_feedback /wheel/odom /imu/data; do
    echo "--- $topic ---"
    ros_timeout 5s ros2 topic info "$topic" -v 2>&1 || true
  done
  echo "===== map pose before ====="
  ros_timeout 5s ros2 run tf2_ros tf2_echo map base_footprint || true
} > "${diag}_state.log" 2>&1

echo "[4/5] 启动 rosbag……"
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
  /scan_camera_safety \
  /scan_camera \
  /tf \
  /tf_static \
  /map \
  /map_updates \
  /slam_toolbox/pose \
  /slam_toolbox/new_node_event \
  /slam_toolbox/loop_closure_event \
  /local_costmap/costmap \
  /local_costmap/costmap_updates \
  /global_costmap/costmap \
  /global_costmap/costmap_updates \
  /plan \
  /local_plan \
  /luck_controller/lookahead \
  /luck_controller/local_trajectory \
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

echo "[5/5] rosbag 已启动。"
echo "记录已开始。一次只复现一个场景，不要在同一包里混合自动探索。"
echo "若测试召回：必须先启动本脚本，再发送召回目标，卡住后再 Ctrl-C 打包。"
echo "普通导航：先测试 0.8~1.2 m 开阔直线，再单独记录原转角/窄通道。"
if (( duration == 0 )); then
  echo "不要启动自动探索，不要同时修改参数。现在会一直记录；故障出现后按 Ctrl+C 打包。"
  while true; do
    sleep 15
    echo "仍在记录；故障出现后按 Ctrl+C……"
  done
else
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
fi

finish_recording
trap - INT TERM
