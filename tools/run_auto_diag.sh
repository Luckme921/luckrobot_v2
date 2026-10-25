#!/usr/bin/env bash

# Record an automatically synchronized Nav2 / explore_lite diagnostic run.
# Usage: bash ~/ros2_ws/tools/run_auto_diag.sh [seconds]

set -u
set -o pipefail

duration="${1:-300}"
if ! [[ "$duration" =~ ^[1-9][0-9]*$ ]]; then
  echo "用法：bash ~/ros2_ws/tools/run_auto_diag.sh [正整数秒数]" >&2
  exit 2
fi

stamp="$(date +%Y%m%d_%H%M%S)"
diag="$HOME/auto_diag_${stamp}"
bag_pid=""
packed=0
exploration_started=0

collision_params="${diag}_collision_params.yaml"
controller_params="${diag}_controller_params.yaml"
planner_params="${diag}_planner_params.yaml"
state_log="${diag}_state.log"
bag_log="${diag}_bag.log"
explore_log="${diag}_explore.log"

# Always create the text members first. Ctrl-C during preflight can therefore
# still produce a useful archive instead of failing because files are absent.
: > "$collision_params"
: > "$controller_params"
: > "$planner_params"
: > "$state_log"
: > "$bag_log"
: > "$explore_log"

pack_available_files()
{
  local members=()
  local path
  for path in \
    "$diag" \
    "$collision_params" \
    "$controller_params" \
    "$planner_params" \
    "$state_log" \
    "$bag_log" \
    "$explore_log"
  do
    if [[ -e "$path" ]]; then
      members+=("$(basename "$path")")
    fi
  done

  if [[ "${#members[@]}" -gt 0 ]]; then
    tar -czf "${diag}.tar.gz" -C "$HOME" "${members[@]}"
    echo "诊断包：${diag}.tar.gz"
  fi
}

finish_recording()
{
  # Avoid recursively invoking this handler while rosbag is finalized.
  trap - INT TERM

  # The exploration process may already have exited on timeout. Do not rely
  # on its destructor's asynchronous cancellation surviving ROS shutdown.
  if [[ "$exploration_started" -eq 1 ]]; then
    echo "取消本轮未完成的导航目标……"
    timeout -k 2s -s INT 6s ros2 service call \
      /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal \
      '{goal_info: {stamp: {sec: 0, nanosec: 0}, goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}}' \
      >> "$state_log" 2>&1 || echo "警告：取消目标请求未确认；请观察小车，必要时急停。" >&2
    exploration_started=0
  fi

  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    echo "[6/6] 正在停止 rosbag 并写入索引……"
    kill -INT "$bag_pid" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$bag_pid" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 "$bag_pid" 2>/dev/null; then
      echo "警告：rosbag 10 秒内未退出，发送 TERM 完成清理。" >&2
      kill -TERM "$bag_pid" 2>/dev/null || true
    fi
    wait "$bag_pid" 2>/dev/null || true
  fi

  if [[ "$packed" -eq 0 ]]; then
    packed=1
    echo "正在打包诊断文件……"
    pack_available_files
  fi
}

on_interrupt()
{
  echo
  echo "收到中断，保留已经采集的数据。"
  finish_recording
  exit 130
}

trap on_interrupt INT TERM

run_to_file()
{
  local label="$1"
  local output="$2"
  shift 2
  echo "$label"
  timeout -k 2s -s INT 8s "$@" > "$output" 2>&1
  local rc=$?
  if [[ "$rc" -ne 0 ]]; then
    echo "警告：${label}失败或超过 8 秒（状态 ${rc}），继续采集。" | tee -a "$output" >&2
  fi
}

append_state()
{
  local label="$1"
  shift
  echo "$label" | tee -a "$state_log"
  timeout -k 2s -s INT 8s "$@" 2>&1 | tee -a "$state_log"
  local rc=${PIPESTATUS[0]}
  if [[ "$rc" -ne 0 ]]; then
    echo "警告：该检查失败或超过 8 秒（状态 ${rc}），继续。" | tee -a "$state_log" >&2
  fi
}

# A topic appearing in `ros2 topic list` is not proof that its sensor is
# producing data.  In particular, the old camera profile could leave all
# nodes alive while the depth/IR pipeline repeatedly reset, so Collision
# Monitor silently received no camera scan.  Require a fresh message before
# allowing the robot to move.
require_fresh_sample()
{
  local topic="$1"
  local label="$2"
  local sample="${diag}_sensor_sample.tmp"
  local received=1
  : > "$sample"

  for _ in 1 2; do
    timeout -k 2s -s INT 6s ros2 topic echo \
      --qos-reliability best_effort \
      --qos-durability volatile \
      "$topic" --once > "$sample" 2>&1 || true
    if grep -Eq '^(header:|stamp:|ranges:|pose:)' "$sample"; then
      received=0
      break
    fi
  done

  {
    echo "===== ${label} (${topic}) ====="
    cat "$sample"
  } >> "$state_log"
  rm -f "$sample"

  if [[ "$received" -ne 0 ]]; then
    echo "错误：${label} ${topic} 在 12 秒内没有新消息；不启动自动探索。" >&2
    return 1
  fi
  return 0
}

echo "自动探索诊断：${duration} 秒"
echo "输出前缀：${diag}"
echo "脚本会显示每个阶段；运行阶段没有新日志时仍会持续录包。"

echo "[1/6] 读取运行参数（单项最多 8 秒）……"
run_to_file "  - collision_monitor 参数" "$collision_params" \
  ros2 param dump /collision_monitor
run_to_file "  - controller_server 参数" "$controller_params" \
  ros2 param dump /controller_server
run_to_file "  - planner_server 参数" "$planner_params" \
  ros2 param dump /planner_server

echo "[2/6] 检查生命周期、Goal Checker 路由和源码版本……"
{
  date
  echo "===== stable navigation source hashes ====="
  sha256sum \
    "$HOME/ros2_ws/src/luckrobot_nav2_plugins/src/luck_controller.cpp" \
    "$HOME/ros2_ws/src/luckrobot_nav2_plugins/src/luck_planner.cpp" \
    "$HOME/ros2_ws/src/chassis_driver/src/chassis_node.cpp" \
    "$HOME/ros2_ws/src/my_robot_navigation/config/nav2_params.yaml" \
    "$HOME/ros2_ws/src/my_robot_bringup/config/collision_monitor.yaml" \
    "$HOME/ros2_ws/src/my_robot_bringup/launch/smart_navigation.launch.py"
  echo "===== exploration and goal-routing source hashes ====="
  sha256sum \
    "$HOME/ros2_ws/src/m-explore-ros2/explore/src/explore.cpp" \
    "$HOME/ros2_ws/src/my_robot_mapping/launch/auto_explore.launch.py" \
    "$HOME/ros2_ws/src/my_robot_mapping/behavior_trees/explore_replan_no_recovery.xml" \
    "$HOME/ros2_ws/src/my_robot_navigation/launch/navigation.launch.py" \
    "$HOME/ros2_ws/src/my_robot_navigation/behavior_trees/navigate_to_pose_general_goal_checker.xml" \
    "$HOME/ros2_ws/src/my_robot_navigation/behavior_trees/navigate_through_poses_general_goal_checker.xml"
} >> "$state_log" 2>&1

node_list="$(timeout -k 2s -s INT 5s ros2 node list 2>/dev/null || true)"
duplicate_nodes="$(grep '^/' <<< "$node_list" | sort | uniq -d || true)"
if [[ -n "$duplicate_nodes" ]]; then
  echo "错误：ROS 图中存在同名重复节点，不启动自动探索：" >&2
  echo "$duplicate_nodes" >&2
  echo "请关闭旧导航终端并运行 tools/stop_navigation_stack.sh。" >&2
  finish_recording
  exit 3
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
    echo "错误：ROS 图中缺少必要节点 ${node}，不启动自动探索。" >&2
    finish_recording
    exit 3
  fi
done
if grep -Fxq /explore_node <<< "$node_list"; then
  echo "错误：已有 /explore_node；请先关闭旧自动探索终端。" >&2
  finish_recording
  exit 3
fi

controller_state="$(timeout -k 2s -s INT 3s ros2 lifecycle get /controller_server 2>&1 || true)"
planner_state="$(timeout -k 2s -s INT 3s ros2 lifecycle get /planner_server 2>&1 || true)"
smoother_state="$(timeout -k 2s -s INT 3s ros2 lifecycle get /velocity_smoother 2>&1 || true)"
bt_state="$(timeout -k 2s -s INT 3s ros2 lifecycle get /bt_navigator 2>&1 || true)"
collision_state="$(timeout -k 2s -s INT 3s ros2 lifecycle get /collision_monitor 2>&1 || true)"

lifecycle_acceptable()
{
  local node="$1"
  local state="$2"
  echo "===== ${node} lifecycle =====" >> "$state_log"
  echo "$state" >> "$state_log"
  if grep -Eq '^active \[[0-9]+\]\r?$' <<< "$state"; then
    return 0
  fi
  if grep -Eq 'inactive|unconfigured|finalized|Node not found' <<< "$state"; then
    echo "错误：${node} 返回明确的非 active 状态：${state}" >&2
    return 1
  fi
  echo "警告：${node} 生命周期服务无响应，但节点仍在 ROS 图中；继续录包捕获异常。"
  return 0
}

if ! lifecycle_acceptable /controller_server "$controller_state" ||
  ! lifecycle_acceptable /planner_server "$planner_state" ||
  ! lifecycle_acceptable /velocity_smoother "$smoother_state" ||
  ! lifecycle_acceptable /bt_navigator "$bt_state" ||
  ! lifecycle_acceptable /collision_monitor "$collision_state"
then
  finish_recording
  exit 3
fi

append_state "===== goal checker plugins =====" \
  ros2 param get /controller_server goal_checker_plugins
append_state "===== default NavigateToPose BT =====" \
  ros2 param get /bt_navigator default_nav_to_pose_bt_xml
append_state "===== default NavigateThroughPoses BT =====" \
  ros2 param get /bt_navigator default_nav_through_poses_bt_xml

# The exploration action supplies its own BT file, so checking only the BT
# navigator's default tree cannot detect a stale GridBased/FollowPath route.
# Refuse the run unless the installed exploration tree explicitly selects the
# same custom planner/controller pair that was accepted in manual luck_full
# tests.
installed_explore_bt="$HOME/ros2_ws/install/my_robot_mapping/share/my_robot_mapping/behavior_trees/explore_replan_no_recovery.xml"
if [[ ! -f "$installed_explore_bt" ]] ||
  ! grep -q 'planner_id="LuckGrid"' "$installed_explore_bt" ||
  ! grep -q 'controller_id="LuckFollowPath"' "$installed_explore_bt"
then
  echo "错误：安装区探索行为树没有同时使用 LuckGrid/LuckFollowPath；不启动自动探索。" >&2
  echo "请重新构建 my_robot_mapping 后再测试。" >&2
  finish_recording
  exit 3
fi

echo "[3/6] 验证避障传感器和（如启用）VSLAM 的实时消息……"
if ! require_fresh_sample /scan_filtered "主雷达扫描" ||
  ! require_fresh_sample /scan_camera "相机高度带扫描" ||
  ! require_fresh_sample /scan_camera_safety "相机安全扫描"; then
  finish_recording
  exit 3
fi

# When the candidate filter or the container-side cuVSLAM node is present,
# accepting an auto-explore run without raw visual odometry would produce a
# misleading 50 Hz candidate stream from wheel+IMU alone.  Keep this a hard
# preflight gate; production Nav2 still uses /odometry/filtered.
if grep -Fxq /visual_slam_node <<< "$node_list" ||
  grep -Fxq /ekf_filter_node_vslam_candidate <<< "$node_list"; then
  if ! require_fresh_sample /visual_slam/tracking/odometry "cuVSLAM 原始里程计" ||
    ! require_fresh_sample /visual_slam/tracking/odometry_validated "VSLAM 健康门控里程计" ||
    ! require_fresh_sample /odometry/vslam_candidate "VSLAM 候选融合里程计"; then
    echo "提示：VSLAM 原始输出存在，但健康门控尚未接纳；先修复视觉跟踪再做融合验收。" >&2
    echo "主轮速+IMU 导航不受本门控改变，关闭候选后仍可单独测试探索。" >&2
    finish_recording
    exit 3
  fi
fi

# A lifecycle service timeout is not enough to prove that Collision Monitor is
# forwarding commands.  Publish zero Twist frames for a few seconds instead of
# a single frame: under CPU/DDS load a one-shot publisher can exit before its
# endpoint is matched and cause a false negative.  If the live probe still
# misses, accept only an exact, inspectable /cmd_vel -> collision_monitor ->
# /cmd_vel_safe -> chassis_driver_node topology.  Explicit inactive lifecycle
# states have already been rejected above.
safe_probe="${diag}_safe_probe.tmp"
cmd_topic_info="${diag}_cmd_topic_info.tmp"
safe_topic_info="${diag}_safe_topic_info.tmp"
: > "$safe_probe"
: > "$cmd_topic_info"
: > "$safe_topic_info"
timeout -k 1s -s INT 7s ros2 topic echo \
  --qos-reliability reliable \
  --qos-durability volatile \
  /cmd_vel_safe --once > "$safe_probe" 2>&1 &
safe_probe_pid=$!
sleep 1.0
timeout -k 1s -s INT 4s ros2 topic pub \
  --rate 10 \
  --wait-matching-subscriptions 1 \
  /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
  > /dev/null 2>&1 || true
wait "$safe_probe_pid" 2>/dev/null
safe_probe_rc=$?

if [[ "$safe_probe_rc" -eq 0 ]] && grep -q '^linear:' "$safe_probe"; then
  echo "===== /cmd_vel -> /cmd_vel_safe multi-frame zero-command probe: PASS =====" \
    >> "$state_log"
else
  echo "===== zero-command probe did not receive a sample =====" >> "$state_log"
  cat "$safe_probe" >> "$state_log"

  timeout -k 2s -s INT 6s ros2 topic info /cmd_vel -v \
    > "$cmd_topic_info" 2>&1 || true
  timeout -k 2s -s INT 6s ros2 topic info /cmd_vel_safe -v \
    > "$safe_topic_info" 2>&1 || true
  {
    echo "===== /cmd_vel topology fallback ====="
    cat "$cmd_topic_info"
    echo "===== /cmd_vel_safe topology fallback ====="
    cat "$safe_topic_info"
  } >> "$state_log"

  if grep -q '^Node name: collision_monitor$' "$cmd_topic_info" &&
    grep -q '^Publisher count: 1$' "$safe_topic_info" &&
    grep -q '^Subscription count: 1$' "$safe_topic_info" &&
    grep -q '^Node name: collision_monitor$' "$safe_topic_info" &&
    grep -q '^Node name: chassis_driver_node$' "$safe_topic_info"
  then
    echo "警告：零速探针受 DDS/CPU 延迟影响未收到样本；安全链拓扑唯一且完整，继续录包。"
    echo "===== safety topology fallback: PASS =====" >> "$state_log"
  else
    echo "错误：无法确认唯一完整的 Collision Monitor 安全链；不启动自动探索。" >&2
    rm -f "$safe_probe" "$cmd_topic_info" "$safe_topic_info"
    finish_recording
    exit 3
  fi
fi
rm -f "$safe_probe" "$cmd_topic_info" "$safe_topic_info"

echo "[4/6] 启动 rosbag……"
# ros2 bag reads keyboard controls. Redirect stdin so a background recorder is
# not suspended by an interactive shell with SIGTTIN.
ros2 bag record \
  --qos-profile-overrides-path "$HOME/ros2_ws/tools/nav_diag_qos.yaml" \
  --include-hidden-topics \
  -o "$diag" \
  /cmd_vel_nav \
  /cmd_vel \
  /cmd_vel_safe \
  /cmd_vel_drive \
  /motion_freshness_guard/status \
  /wheel_feedback \
  /wheel/odom \
  /odometry/filtered \
  /visual_slam/tracking/odometry \
  /visual_slam/tracking/vo_pose_covariance \
  /visual_slam/tracking/odometry_validated \
  /vslam/health \
  /odometry/vslam_candidate \
  /imu/data \
  /scan_filtered \
  /scan_camera_safety \
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
  /luck_controller/local_trajectory \
  /luck_controller/tracking_error \
  /navigate_to_pose/_action/feedback \
  /navigate_to_pose/_action/status \
  /compute_path_to_pose/_action/feedback \
  /compute_path_to_pose/_action/status \
  /explore/frontiers \
  /explore/status \
  /diagnostics \
  /behavior_tree_log \
  /rosout \
  < /dev/null > "$bag_log" 2>&1 &

bag_pid=$!
sleep 3
if ! kill -0 "$bag_pid" 2>/dev/null || [[ ! -d "$diag" ]]; then
  echo "错误：rosbag 没有成功启动；不启动自动探索。日志如下：" >&2
  tail -n 30 "$bag_log" >&2 || true
  finish_recording
  exit 4
fi
echo "  rosbag 已启动（PID ${bag_pid}）。"

echo "[5/6] 开始自动探索；将在 ${duration} 秒后自动结束，也可按 Ctrl-C 提前结束。"
exploration_started=1
timeout -k 10s -s INT "${duration}s" \
  stdbuf -oL -eL ros2 launch my_robot_mapping auto_explore.launch.py \
  2>&1 | tee "$explore_log"
explore_rc=${PIPESTATUS[0]}

if [[ "$explore_rc" -ne 0 && "$explore_rc" -ne 124 && "$explore_rc" -ne 130 ]]; then
  echo "警告：自动探索进程状态为 ${explore_rc}；诊断数据仍会保存。" >&2
fi

finish_recording
trap - INT TERM
