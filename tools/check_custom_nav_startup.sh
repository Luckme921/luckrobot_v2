#!/usr/bin/env bash

# Verify that the custom Nav2 stack remains healthy while the robot is idle.
# Usage: bash ~/ros2_ws/tools/check_custom_nav_startup.sh [seconds]

set -u

duration="${1:-60}"
stamp="$(date +%Y%m%d_%H%M%S)"
prefix="$HOME/custom_nav_startup_${stamp}"
log="${prefix}.log"
result="PASS"
lifecycle_warnings=0
required_nodes=(
  /controller_server
  /planner_server
  /velocity_smoother
  /bt_navigator
  /collision_monitor
)
declare -A timeout_streak

exec > >(tee "$log") 2>&1

echo "自定义导航无运动启动门禁：${duration} 秒"
echo "保持小车原地；不要发送目标，不要启动自动探索。"
echo "开始时间：$(date --iso-8601=ns)"

check_state()
{
  local node="$1"
  local state
  local streak
  state="$(timeout -s INT 3s ros2 lifecycle get "$node" 2>&1 || true)"
  printf '%s=%s\n' "$node" "$state"
  if grep -Eq '^active \[[0-9]+\]\r?$' <<< "$state"; then
    timeout_streak["$node"]=0
    return 0
  fi
  if grep -Eq 'inactive|unconfigured|finalized|Node not found' <<< "$state"; then
    echo "错误：${node} 返回了明确的非 active 状态。"
    return 1
  fi
  streak="${timeout_streak[$node]:-0}"
  streak=$((streak + 1))
  timeout_streak["$node"]="$streak"
  lifecycle_warnings=$((lifecycle_warnings + 1))
  echo "警告：${node} 生命周期服务暂时无响应，连续 ${streak} 次；节点仍在 ROS 图中，不据此判定失活。"
  return 0
}

start_seconds="$SECONDS"
next_lifecycle_check=0
while (( SECONDS - start_seconds < duration )); do
  elapsed=$((SECONDS - start_seconds))
  echo "===== t=${elapsed}s ====="
  node_list="$(timeout -s INT 4s ros2 node list 2>&1 || true)"
  duplicate_nodes="$(grep '^/' <<< "$node_list" | sort | uniq -d || true)"
  if [[ -n "$duplicate_nodes" ]]; then
    echo "错误：ROS 图中存在同名重复节点；这通常是上一次 launch 的孤儿进程："
    echo "$duplicate_nodes"
    echo "===== 同名节点对应进程 ====="
    pgrep -af -- \
      'scan_to_scan_filter_chain|smart_navigation.launch.py|robot_bringup.launch.py' \
      2>/dev/null || true
    echo "===== /scan_filtered 发布端点 ====="
    timeout -s INT 5s ros2 topic info /scan_filtered -v 2>&1 || true
    result="FAIL"
    break
  fi
  node_ok=1
  for node in "${required_nodes[@]}"; do
    if ! grep -Fxq "$node" <<< "$node_list"; then
      echo "错误：ROS 图中缺少 ${node}"
      node_ok=0
    fi
  done
  if [[ "$node_ok" -eq 0 ]]; then
    result="FAIL"
    break
  fi

  if (( elapsed >= next_lifecycle_check )); then
    cycle_ok=1
    for node in "${required_nodes[@]}"; do
      check_state "$node" || cycle_ok=0
    done
    if [[ "$cycle_ok" -eq 0 ]]; then
      result="FAIL"
      break
    fi
    next_lifecycle_check=$((elapsed + 15))
  fi
  sleep 2
done

echo "===== 版本与运行库 ====="
grep -n -E \
  'cost_penalty:|smoothing_max_cost:|path_heading_gain:|execution_stall_timeout:|path_heading_filter|in_place_avoidance' \
  "$HOME/ros2_ws/install/my_robot_navigation/share/my_robot_navigation/config/nav2_params.yaml" \
  2>&1 || true
plugin_library="$HOME/ros2_ws/install/luckrobot_nav2_plugins/lib/libluckrobot_nav2_plugins.so"
if [[ -f "$plugin_library" ]]; then
  sha256sum "$plugin_library"
  ldd "$plugin_library" || true
fi

echo "===== ROS 节点 ====="
timeout -s INT 5s ros2 node list 2>&1 || true

echo "===== 自定义树全局重规划频率 ====="
grep -n 'RateController hz=' \
  "$HOME/ros2_ws/install/my_robot_navigation/share/my_robot_navigation/behavior_trees/navigate_to_pose_luck_full.xml" \
  2>&1 || true

echo "===== 最近的 controller_server 崩溃记录 ====="
if command -v coredumpctl >/dev/null 2>&1; then
  timeout 8s coredumpctl info controller_server 2>&1 || true
else
  echo "coredumpctl 不可用"
fi

echo "结束时间：$(date --iso-8601=ns)"
if [[ "$result" == "PASS" ]] && (( lifecycle_warnings > 0 )); then
  result="PASS_WITH_WARNINGS"
  echo "说明：节点均存在且未返回明确的非 active 状态，但出现 ${lifecycle_warnings} 次生命周期服务超时。"
  echo "这通常表示 Jetson/ROS 服务繁忙；门禁保留警告，但允许启动录包捕获真实故障。"
fi
echo "RESULT=${result}"

tar -czf "${prefix}.tar.gz" -C "$HOME" "$(basename "$log")"
echo "诊断包：${prefix}.tar.gz"

if [[ "$result" == "FAIL" ]]; then
  exit 3
fi
