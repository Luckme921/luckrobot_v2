#!/usr/bin/env bash

# Isolated chassis-direction diagnostic for LuckRobot.
#
# Terminal 1 must run the hardware-only launch with:
#   ros2 launch my_robot_bringup test_bringup.launch.py \
#     use_rviz2:=false cmd_vel_topic:=/cmd_vel_test
#
# Usage:
#   bash ~/ros2_ws/tools/run_manual_motion_diag.sh linear
#   bash ~/ros2_ws/tools/run_manual_motion_diag.sh turn
#   MANUAL_CMD_VEL_TOPIC=/cmd_vel_vslam_test \
#     bash ~/ros2_ws/tools/run_manual_motion_diag.sh vslam_linear
#   MANUAL_CMD_VEL_TOPIC=/cmd_vel_vslam_test \
#     bash ~/ros2_ws/tools/run_manual_motion_diag.sh vslam_turn
#
# The two modes are deliberately separate so the robot never performs an
# unexpected second movement.  "linear" commands about 9 cm forward.
# "turn" commands about 21 degrees counter-clockwise.

set -u

mode="${1:-}"
cmd_vel_topic="${MANUAL_CMD_VEL_TOPIC:-/cmd_vel_test}"
case "$mode" in
  linear)
    command_seconds="1.5"
    message_count="30"
    publisher_timeout_seconds="10"
    twist="{linear: {x: 0.06, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    expectation="车体物理正前方约 9 cm；/wheel/odom 的 x 应增加"
    ;;
  turn)
    command_seconds="2.0"
    message_count="40"
    publisher_timeout_seconds="10"
    twist="{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.18}}"
    expectation="原地向左（逆时针）约 21 度；/wheel/odom 的 yaw 应增加"
    ;;
  vslam_linear)
    command_seconds="8.0"
    message_count="160"
    publisher_timeout_seconds="16"
    twist="{linear: {x: 0.07, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    expectation="VSLAM 动态验收：车体物理正前方约 0.56 m；必须在空旷区域执行"
    ;;
  vslam_turn)
    command_seconds="6.5"
    message_count="130"
    publisher_timeout_seconds="15"
    twist="{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.12}}"
    expectation="VSLAM 动态验收：原地向左约 45 度；必须在空旷区域执行"
    ;;
  *)
    echo "用法：bash ~/ros2_ws/tools/run_manual_motion_diag.sh {linear|turn|vslam_linear|vslam_turn}"
    exit 2
    ;;
esac

stamp="$(date +%Y%m%d_%H%M%S)"
diag="$HOME/manual_${mode}_${stamp}"
bag_pid=""
finished=0

publish_zero()
{
  ros2 topic pub --once "$cmd_vel_topic" geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
}

finish_recording()
{
  publish_zero

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
        "$(basename "${diag}_bag.log")"
      echo "诊断包：${diag}.tar.gz"
    else
      echo "错误：rosbag 目录未生成；请上传 ${diag}_bag.log"
    fi
  fi
}

trap 'finish_recording; exit 130' INT TERM

nodes="$(ros2 node list 2>/dev/null || true)"
for forbidden in /controller_server /velocity_smoother /collision_monitor /bt_navigator /explore_node; do
  if grep -Fxq "$forbidden" <<< "$nodes"; then
    echo "拒绝测试：检测到 $forbidden。先 Ctrl+C 关闭 Nav2/探索，再只启动 test_bringup。"
    exit 3
  fi
done

topic_info="$(ros2 topic info "$cmd_vel_topic" -v 2>&1 || true)"
if ! grep -q 'Subscription count: 1' <<< "$topic_info"; then
  echo "拒绝测试：${cmd_vel_topic} 不是恰好 1 个订阅者。"
  echo "$topic_info"
  exit 4
fi

{
  echo "===== mode ====="
  echo "$mode"
  echo "===== expected physical result ====="
  echo "$expectation"
  echo "===== date ====="
  date --iso-8601=ns
  echo "===== nodes ====="
  printf '%s\n' "$nodes"
  echo "===== ${cmd_vel_topic} endpoints ====="
  printf '%s\n' "$topic_info"
  echo "===== wheel odom before ====="
  timeout -s INT 3s ros2 topic echo /wheel/odom --once || true
  echo "===== filtered odom before ====="
  timeout -s INT 3s ros2 topic echo /odometry/filtered --once || true
  echo "===== imu before ====="
  timeout -s INT 3s ros2 topic echo /imu/data --once || true
  echo "===== VSLAM odometry before ====="
  timeout -s INT 3s ros2 topic echo /visual_slam/tracking/odometry --once || true
  echo "===== VSLAM candidate EKF before ====="
  timeout -s INT 3s ros2 topic echo /odometry/vslam_candidate --once || true
} > "${diag}_state.log" 2>&1

# ros2 bag reads keyboard controls. Redirect stdin so a background recorder is
# not suspended by the interactive shell with SIGTTIN.
ros2 bag record \
  -o "$diag" \
  /manual_diag_marker \
  "$cmd_vel_topic" \
  /wheel_feedback \
  /wheel/odom \
  /odometry/filtered \
  /imu/data \
  /visual_slam/tracking/odometry \
  /visual_slam/tracking/vo_pose_covariance \
  /visual_slam/tracking/vo_path \
  /odometry/vslam_candidate \
  /diagnostics \
  /scan_filtered \
  /scan_camera \
  /tf \
  /tf_static \
  /rosout \
  < /dev/null > "${diag}_bag.log" 2>&1 &

bag_pid=$!
sleep 3

if ! kill -0 "$bag_pid" 2>/dev/null; then
  echo "错误：rosbag 启动失败。"
  finish_recording
  exit 5
fi

ros2 topic pub --once /manual_diag_marker std_msgs/msg/String \
  "{data: '${mode}_command_start'}" >/dev/null 2>&1 || true

echo "开始 ${mode} 测试：${expectation}"
# Count only actual 20 Hz publications, not ros2 CLI/DDS discovery startup.
# A separate outer timeout still guarantees that a failed publisher cannot
# leave the test running indefinitely.
timeout -s INT "${publisher_timeout_seconds}s" \
  ros2 topic pub --rate 20 --times "$message_count" \
  --wait-matching-subscriptions 1 \
  "$cmd_vel_topic" geometry_msgs/msg/Twist "$twist" \
  >/dev/null 2>&1 || true

publish_zero

ros2 topic pub --once /manual_diag_marker std_msgs/msg/String \
  "{data: '${mode}_command_stop'}" >/dev/null 2>&1 || true

sleep 3

{
  echo "===== wheel odom after ====="
  timeout -s INT 3s ros2 topic echo /wheel/odom --once || true
  echo "===== filtered odom after ====="
  timeout -s INT 3s ros2 topic echo /odometry/filtered --once || true
  echo "===== imu after ====="
  timeout -s INT 3s ros2 topic echo /imu/data --once || true
  echo "===== VSLAM odometry after ====="
  timeout -s INT 3s ros2 topic echo /visual_slam/tracking/odometry --once || true
  echo "===== VSLAM candidate EKF after ====="
  timeout -s INT 3s ros2 topic echo /odometry/vslam_candidate --once || true
  echo "===== date after ====="
  date --iso-8601=ns
} >> "${diag}_state.log" 2>&1

finish_recording
trap - INT TERM
