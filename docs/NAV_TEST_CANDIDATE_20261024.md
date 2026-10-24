# 2026-10-24 candidate handoff: not yet high-speed validated

## Evidence and interpretation

Original 0.30 controller prediction / 0.15 guard execution mismatch is fixed through
one launch speed argument. Continuous segment projection reduces target-point jumps.
Persistent 4 MiB UDP caps survived reboot. Motor acceleration byte is now explicit
through all launch layers; default50 retained, candidate must pass motor_accel_time:=10.

DDSM115 byte50 versus byte10, same low-speed command, two repeats each:
forward first90% feedback 1.61–1.81 s versus .41–.51 s; differential turn .77–.88
versus .27–.31 s. No failed replies/faults, current maxima .167/.175 A on stand.
Loaded byte10 short straight/turn/arc completed; user observed basically normal.
This is evidence for faster actuator response, not proof of calibrated braking.

Three navigation bags audited:
- nav_motor10_20261024_125259
- nav_motor10_restart_20261024_164410
- nav_repeat_20261024_164629

Near-straight target R-minus-L mean RPM: .027, -.043, -.100.
Feedback R-minus-L mean RPM: 1.045, 1.065, 1.067.
These include transients; raw integer RPM means alone cannot establish torque
imbalance or justify permanent left-wheel trim. Compare same-time position
derivatives and body IMU in longer steady motion before fitting calibration.
One RPM at diameter .1 m means ~.00524 m/s; one-wheel differential step at track
.269 m means ~.0195 rad/s. Quantization can matter for tiny corrections; aggressive
RPM dithering/extra PI could add oscillation and is NOT deployed without validation.

User reports deviation followed by corrective yaw, no obvious slip. This supersedes
the misleading inference that small controller cross-track error means smooth motion.
Whole-bag IMU/wheel yaw integrals are NOT absolute heading error against a fixed
ground-truth path. SLAM corrections/terminal phase also affect apparent geometry.

The recent .30 m goals ran predominantly .07–.09 m/s due to approach regulation.
They do not validate sustained .15 or .30 speed. Earlier long high-speed runs still
show unacceptable overshoot; byte10 has not yet passed equivalent long/corner runs.

## Conservative test order (one bag per scenario)

Keep candidate motor10 and all safety checks enabled. Start .15 m/s limit.
1. Open straight, 1–1.2 m goal, clear extra runout and operator emergency stop.
2. Separate bag, open left turn; then separate right turn. No narrow aisles first.
3. Only after logs and physical results accepted: .20 then .30, each with separate
   stop and corner checks. Do not change multiple parameters, wheel trim, inflation,
   or controller at once. Stop on swing/overshoot; don't keep rerunning into danger.

Exact startup (new mapping, do not carry/rotate robot afterward):
```bash
source ~/ros2_ws/tools/robot_env.sh
ros2 launch my_robot_bringup luckrobot_system.launch.py \
  mode:=mapping navigation_profile:=luck_full depth_obstacle_mode:=height \
  use_rviz2:=false use_rviz:=false \
  motion_speed_limit:=0.15 motor_accel_time:=10
```
Second SSH terminal:
```bash
source ~/ros2_ws/tools/robot_env.sh
bash ~/ros2_ws/tools/run_custom_nav_diag.sh 0
```
Wait for recording-started message before goal. Ctrl+C once after completion and
wait for tar.gz. Recorder now includes wheel_response_debug and motor_status_debug,
plus actual driver/smoother parameters. Report package path and qualitative defect.

MobaXterm X11 terminal for RViz:
```bash
source ~/ros2_ws/tools/robot_env.sh
export QT_X11_NO_MITSHM=1
rviz2 -d ~/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/rviz_config/rviz2_config1.rviz
```
Prior X11 shader issues are not declared resolved. If map/robot not displayed
correctly, do not send blind goals; use the bounded straight-goal script instead:
```bash
source ~/ros2_ws/tools/robot_env.sh
python3 ~/ros2_ws/tools/short_nav_speed_test.py 1.0
```
Only run that command with at least 2 m clear space ahead in current heading.
Robot may stop within configured goal tolerance, not exactly at commanded distance.

For shutdown cancel any goal first; once stopped, Ctrl+C launch terminal. Do not
launch a second stack over the existing one. Historical low-speed stable reference
7a01ff7 archive preserved; no destructive Git reset performed.
