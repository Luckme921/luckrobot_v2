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

collision_params="${diag}_collision_params.yaml"
controller_params="${diag}_controller_params.yaml"
state_log="${diag}_state.log"
bag_log="${diag}_bag.log"
explore_log="${diag}_explore.log"

# Always create the text members first. Ctrl-C during preflight can therefore
# still produce a useful archive instead of failing because files are absent.
: > "$collision_params"
: > "$controller_params"
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

  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    echo "[5/5] 正在停止 rosbag 并写入索引……"
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

echo "自动探索诊断：${duration} 秒"
echo "输出前缀：${diag}"
echo "脚本会显示每个阶段；运行阶段没有新日志时仍会持续录包。"

echo "[1/5] 读取运行参数（单项最多 8 秒）……"
run_to_file "  - collision_monitor 参数" "$collision_params" \
  ros2 param dump /collision_monitor
run_to_file "  - controller_server 参数" "$controller_params" \
  ros2 param dump /controller_server

echo "[2/5] 检查生命周期、Goal Checker 路由和源码版本……"
{
  date
  echo "===== stable navigation source hashes ====="
  sha256sum \
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

append_state "===== controller lifecycle =====" \
  ros2 lifecycle get /controller_server
append_state "===== planner lifecycle =====" \
  ros2 lifecycle get /planner_server
append_state "===== BT navigator lifecycle =====" \
  ros2 lifecycle get /bt_navigator
append_state "===== collision monitor lifecycle =====" \
  ros2 lifecycle get /collision_monitor
append_state "===== goal checker plugins =====" \
  ros2 param get /controller_server goal_checker_plugins
append_state "===== default NavigateToPose BT =====" \
  ros2 param get /bt_navigator default_nav_to_pose_bt_xml
append_state "===== default NavigateThroughPoses BT =====" \
  ros2 param get /bt_navigator default_nav_through_poses_bt_xml

active_count=$(grep -c '^active \[3\]$' "$state_log" || true)
if [[ "$active_count" -lt 4 ]]; then
  echo "错误：4 个必要生命周期节点中仅确认 ${active_count} 个 active；不启动小车。" >&2
  finish_recording
  exit 3
fi

echo "[3/5] 启动 rosbag……"
# ros2 bag reads keyboard controls. Redirect stdin so a background recorder is
# not suspended by an interactive shell with SIGTTIN.
ros2 bag record \
  -o "$diag" \
  /cmd_vel_nav \
  /cmd_vel \
  /cmd_vel_safe \
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
  /navigate_to_pose/_action/feedback \
  /navigate_to_pose/_action/status \
  /compute_path_to_pose/_action/feedback \
  /compute_path_to_pose/_action/status \
  /explore/frontiers \
  /explore/status \
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

echo "[4/5] 开始自动探索；将在 ${duration} 秒后自动结束，也可按 Ctrl-C 提前结束。"
timeout -k 10s -s INT "${duration}s" \
  stdbuf -oL -eL ros2 launch my_robot_mapping auto_explore.launch.py \
  2>&1 | tee "$explore_log"
explore_rc=${PIPESTATUS[0]}

if [[ "$explore_rc" -ne 0 && "$explore_rc" -ne 124 && "$explore_rc" -ne 130 ]]; then
  echo "警告：自动探索进程状态为 ${explore_rc}；诊断数据仍会保存。" >&2
fi

finish_recording
trap - INT TERM
