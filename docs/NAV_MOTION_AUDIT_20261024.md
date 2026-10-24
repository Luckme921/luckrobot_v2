# 2026-10-24 interrupted-work continuation

## Scope and status

Jetson rebooted. No hardware launch, motion goal, map reset, or safety bypass was
performed during this audit. The previous predictive controller was NOT validated
by the October 23 runs: user still reports overshoot and oscillation.

## Preserved evidence

- Local `diagnostics/20261023_handoff/custom_nav_20261023_164818.tar.gz`:
  SHA256 `01d774bcd35eaed70a1f30128a9d4b5572218926e31f8cb668177ad0c55f414c`.
- Recovered complete `custom_nav_20261023_155014.tar.gz`, 54695709 bytes:
  SHA256 `997261d00093099c56a35a3ed0220917155c6f41d109a1b14e23c832c53f4847`.
- Remote stable milestone `7a01ff7fa9228335ca46875309bc8989b09ed842`, October 17,
  desired/smoother speed 0.15 m/s. Archive `/home/nvidia/luckrobot_stable_7a01ff7.tar.gz`.
  This is a historical rollback reference, NOT permission to reset the dirty repo.
- Pre-edit remote files: `~/ros2_ws/backups/20261024_speed_authority/`.

## Findings (164818)

1. Controller desired speed and smoother limit 0.30 m/s; final freshness guard
   limited execution to 0.15 m/s. Prediction and execution were inconsistent.
2. Camera maximum arrival gap ~0.152 s, EKF ~0.057 s, IMU ~0.075 s. This run
   does not support attributing all observed oscillation to camera dropouts.
3. Approximate command/feedback lag minimizing RMSE: nav to smoother ~0.22–0.26 s;
   drive to wheel forward ~0.98 s, yaw ~0.70 s; drive to body IMU yaw ~0.66 s;
   IMU to filtered yaw ~0.04 s. Arrival-time correlation includes ramping,
   filtering and command amplitudes: these are NOT identified pure actuator delays.
   Do not blindly set response_delay=0.98 or crank feedback gain.
4. Long episode ~95 s, twelve command yaw sign changes, maximum reported lateral
   error ~0.213 m. Physical overshoot reported by user remains an acceptance failure.
5. Mapping uses slam_toolbox, not AMCL. Increasing AMCL frequency is not a fix.
6. Kernel socket caps reverted to 212992 after reboot. User executed the supplied
   installer; remote verified 4194304 for both caps and persistent sysctl file.

## Implemented and deployed

- `navigation.launch.py`: shared motion_speed_limit sets both controller desired
  velocities and smoother forward bound, typed YAML; guard receives the same launch
  argument through smart_navigation. Invalid/NaN/>0.30 limits reject launch.
- `LuckController::selectTrackingTarget`: nearest-segment projection replaces
  nearest discrete vertex, preserving continuous carrot progress and subtracting
  already traversed segment from remaining distance. Does not smooth a corner
  through an obstacle or change the planned route.
- Persistent UDP installer changes only rmem_max/wmem_max to 4 MiB; no IP/Wi-Fi edits.

## Verification

- Jetson launch substitution test passes 0.15/0.30, unchanged angular limit,
  rejects zero/negative/NaN/Inf/>0.30.
- Colcon built luckrobot_nav2_plugins successfully (55 s).
- CTest: motion_prediction and controller_prediction both pass. Added actual
  controller regression showing continuous carrot through nearest-vertex boundary.
- Camera memory regression: 39 origin/size changes preserve world location,
  cleared hits do not reappear, plugin loads.
- Tests use isolated ROS domain 191/localhost, synthetic costmap, no chassis.
- Launch install files resolve to modified source via symlink.

## Still unresolved / next acceptance gate

- Do not label overshoot, actuator lag or high-speed cornering solved.
- Need supervised response test to distinguish command ramp from physical motor
  response. Hold navigation/semantic workloads fixed; log all speed stages,
  actual transmitted RPM, wheel feedback and body IMU. Do not infer physical stop
  from encoder feedback alone.
- Inspect serial framing/partial reads and actual motor model before changing
  acceleration register. Source clears CSIZE after setting CS8; investigate/fix
  with bench verification, not a blind live driver change.
- Controller/planner frequency increases alone cannot remove downstream latency.
- Person in global costmap is not proof of permanent occupancy-map insertion.
  Separate SLAM map, lidar layer and camera layer. Out-of-view high obstacles must
  not be erased merely on a TTL; require valid clearing observation.
- Keep speed 0.15 for first supervised regression despite retaining requested
  0.30 ceiling. Progress toward 0.30 only after stopping/yaw response is measured.

## Primary reference designs reviewed

- Humble Nav2 RPP: velocity/curvature regulated tracking and collision checks:
  https://github.com/ros-navigation/navigation2/blob/humble/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp
- Humble velocity smoother, open/closed loop behavior:
  https://github.com/ros-navigation/navigation2/blob/humble/nav2_velocity_smoother/src/velocity_smoother.cpp
- Nav2 tuning guide (rolling; verify Humble support before adopting parameters):
  https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/
- Motor protocol comparison only; robot motor model not yet confirmed:
  https://www.waveshare.com/wiki/DDSM115

No claim that replacing the controller with a newer algorithm automatically fixes
execution latency. First make speed authority, measured response, collision
prediction and stopping envelope agree; then compare planner/controller candidates.

## Supervised raised-wheel test (user confirmed secure stand)

Executed isolated chassis only, /cmd_vel_bench, no Nav2/smoother/EKF/camera.
Sequence: zero 1 s, forward 0.06 m/s 2 s, zero 3 s, yaw command 0.18 rad/s
2 s, zero 3 s, final repeated zero. Outer process timeout and existing 0.2 s
driver watchdog retained. Driver stopped and absence of chassis process verified.
JSON: diagnostics/20261023_handoff/bench_wheel_response_20261024.json.

- Completed without feedback-stale abort. Startup watchdog message occurred before
  test publisher discovery; no later watchdog warnings in driver log.
- Encoder-difference velocity first 90% crossing: forward 1.607 s, yaw 0.791 s.
  First near-zero (<0.01) after stop: forward 0.323 s, yaw 0.242 s. These are noisy
  single-crossing metrics, not sustained settling times or physical stopping distance.
- Raw motor RPM during forward: at 0.5 s L=2/R=3; 1 s L=5/R=6;
  1.5 s L=7/R=9; 1.9 s L=10/R=12; 0.3 s after stop L=3/R=11.
  Therefore the slow response is also visible in raw motor feedback, not solely EKF.
- Raised-wheel data cannot identify loaded chassis inertia, traction, true yaw,
  or validate collision stopping distance. Left/right asymmetry needs repeatability
  checks and transmitted-command telemetry before changing motor gains/registers.

## Repeat with driver command telemetry

Added best-effort `/wheel_response_debug`: command age, effective target v/w,
software-ramped v/w, raw protocol target RPM L/R, feedback RPM L/R, reply-valid
flags L/R, combined bus seconds, watchdog flag. No motor register/gain changes.
Raw JSON `diagnostics/20261023_handoff/bench_repeat_20261024.json`.
Two repeats, each 3 s forward .06, 3 s zero, 3 s yaw .18, 3 s zero, initial 1 s zero.

- 829 driver samples; no failed replies, combined serial transaction mean .03189 s,
  maximum .04782 s. This is ~31 Hz effective bus throughput, not nominal 100 Hz.
- Forward target RPM 11/-11 reached in outgoing commands within .065/.111 s.
  Feedback first 90% crossing: left 1.821/1.835 s, right 1.565/1.643 s.
- Yaw target RPM -5/-5 sent by .074 s; feedback first 90% .776–.840 s.
- Thus slow response repeats downstream of the software ramp, independent of
  Nav2/EKF. Motor failure is NOT established: protocol interpretation, motor ramp,
  feedback semantics, load/power and physical response remain hypotheses.
- Driver overflow abort occurred on SIGINT after completed test and repeated zero.
  Code inspection found asynchronous serial close could make FD_SET use fd=-1.
  Changed SIGINT handler to request stop only; executor shuts ROS down and destructor
  sends brake frames/closes serial. Serial retry aborts on stop request.
  Three subsequent zero-command shutdown trials produced no overflow/abort.
  This fixes a concrete race; does not prove all possible crash causes eliminated.

Do not tune motor registers without exact model. User asked for mitigations:
retain low-speed validation, anticipate braking and reduce translation when yaw
cannot track, keep collision stops independent. Do not use stronger yaw gains,
minimum-speed boosts, or blind reverse escape as substitutes for response diagnosis.

## Confirmed motor model and static status

User supplied https://www.waveshare.net/wiki/DDSM115 as wheel documentation.
Reviewed official protocol: speed commands signed RPM; byte 6 acceleration time
is 0.1 ms per RPM (50 => 5 ms/RPM). This setting alone does not explain observed
~1.8 s to 11 RPM. Do not treat documentation ramp duration as measured plant response.
Protocol reply byte 1 is mode (not echoed 0x64), byte 8 fault mask. Query 0x74
returns winding temperature in byte 6, unlike drive reply which returns position.

Read-only exclusive-serial 0x74 queries, three each, with chassis process absent:
motor 4 mode=2, RPM=0, 31 C, faults=0; motor 3 mode=2, RPM=0, 30–31 C, faults=0.
This rules out a currently reported mode mismatch/latched fault, not intermittent
under-load faults, supply voltage issues, mechanical faults or feedback filtering.
No acceleration-register or PID changes made. Next A/B should capture raw position,
RPM, current and faults together, change one documented ramp value at a time,
low-speed on stand only, restore default after comparison. Never apply a fast
bench ramp to loaded navigation without separate stopping-distance verification.

## Acceleration byte A/B (supervised stand, same commands, two repeats each)

Parameterized `motor_accel_time` (integer 1..255, default remains 50). Added motor
mode/current/fault telemetry and test abort on mode !=2, nonzero fault, |current|>1.25 A.
Only parameter changed: 50 versus 10; software slopes/speeds/serial configuration unchanged.
JSON files preserved locally: `bench_accel_50.json`, `bench_accel_10.json`.

| First raw-feedback 90% crossing | byte 50 | byte 10 |
|---|---|---|
| Forward, left/right, two repeats | 1.607–1.812 s | 0.410–0.509 s |
| Differential rotation | 0.775–0.875 s | 0.272–0.307 s |
| Maximum reported absolute current | 0.167 A | 0.175 A |

Both completed, all replies valid, all observed modes 2 and fault masks zero.
Bus mean ~31.9 ms unchanged. One byte-10 watchdog sample was before command startup.
This controlled comparison identifies acceleration byte as a substantial contributor
to observed response delay. Earlier statement that nominal 5 ms/RPM arithmetic cannot
explain the full delay remains true mathematically, but MUST NOT be read as exonerating
the register: empirical A/B clearly demonstrates its influence. Firmware internal
behavior is not yet known. Do not claim motor hardware defective or PID misconfigured.

Restored byte 50 by zero-only default driver startup, then shut driver down.
Post-test 0x74 checks: both RPM0, mode2, fault0, 30–31 C. Candidate byte10 NOT made
navigation default. Need supervised loaded low-speed stopping/turning validation,
then intermediate speed progression; no automatic 0.30 m/s rollout.

## First loaded short straight, byte10

User confirmed robot on ground, 2 m clear forward space and emergency-stop supervision.
Only isolated chassis ran: 1 s zero, 2 s at .06 m/s, 4 s zero, repeated final zero.
No rotation or Nav2. Completed; driver stopped. File `floor_short_20261024.json`.
226 driver samples, bus mean .0320 s, no failed replies, mode2/fault0 throughout,
max reported current .457 A. Target11/-11 sent by .122 s; feedback first90%
left .506 s/right .474 s. Two watchdog samples during startup.
Wheel-odometry displacement ~.112 m; after first pose following zero command,
remaining integrated x ~.0086 m. Encoder-only estimate, NOT independent physical
stopping distance or proof of no slip. Await user qualitative physical observation.
Default remains50; byte10 only process override, no next movement automatically.

## Loaded small left turn, byte10

User confirmed surrounding clearance and supervision. One 2 s command at .18 rad/s,
zero translation, preceded by 1 s zero and followed by 4 s zero plus final zero.
File `floor_turn_20261024.json`; completed and driver process absent afterwards.
228 driver samples; no bad replies, mode2/fault0, max current .367 A.
First90% left .353 s/right .320 s. Five watchdog samples were pre-motion startup
(recording t=.094–.223 s; movement starts 1.180 s), not interruptions during turn.
Wheel-integrated yaw ~22.00 degrees, ~.93 degree after first pose following stop.
No IMU/independent yaw reference in this isolated test: this does NOT establish
physical yaw accuracy or final-goal control performance. Await qualitative observation.

## Loaded short left arc, byte10

User reported small turn normal, then authorized arc after forward/left clearance
request. Executed one 2 s command v=.06 m/s,w=.12 rad/s, zeros before/after.
File `floor_arc_20261024.json`, completed, driver shut down. 226 samples;
no failed replies, fault0/mode2, max current .455 A. Target protocol RPM 8/-15
sent by .076 s; first90% feedback left .491 s/right .523 s. Two startup watchdog
samples. This is an open-loop actuator check, not navigation path tracking validation.

## Integrated navigation candidate

Propagated motor_accel_time through luckrobot_system -> smart_navigation ->
test_bringup -> chassis, default50 preserved. Started fresh mapping (robot had
been moved), luck_full, height obstacles, no RViz, motion_speed_limit=.15,
motor_accel_time=10. Runtime reads confirmed motor10, controller desired .15,
smoother max .15 and guard .15. Controller/collision lifecycle active; TF/EKF/camera
health passed. Issued only .30 m forward goal from fresh map origin; action returned
status4 SUCCEEDED, controller logged Reached the goal. User physical report pending.
Bag `/home/nvidia/nav_motor10_20261024_125259`, launch log
`/home/nvidia/nav_candidate_20261024.log`, launch PID12382 remains running idle.
Do not infer .30 m actual displacement from goal distance (goal tolerance exists).

## Afternoon reboot continuation

User confirmed carried/rotated robot, now on ground with safe clearance and supervision.
Reboot preserved both 4 MiB UDP caps. Started NEW mapping, no old posegraph or map
overwrite, motor10/speed .15/no RViz; runtime parameters verified, controller and
collision monitor active; TF/EKF/camera health passed. Launch PID4658, log
`/home/nvidia/nav_motor10_restart_20261024_164255.log` remains idle after test.
One .30 m forward goal returned status4 SUCCEEDED; bag
`/home/nvidia/nav_motor10_restart_20261024_164410`. Physical observation pending.
Driver heartbeat timeout logged AFTER controller goal success (~.47 s later),
followed by guard waiting for input. Must inspect command timings before attributing
this to in-motion dropout. No additional motion issued. Goal test cancellation now
waits for terminal result; runner stops launch on script failure.

## Repeat after user reported mild left/right judder

First afternoon success was NOT accepted as smooth: user felt left/right judder.
Repeated same .30 m forward goal, no parameter changes; health/lifecycles checked.
Start map(.224,.001,yaw .003), target(.524,.002); action SUCCEEDED. Bag
`/home/nvidia/nav_repeat_20261024_164629` copied local, motion-chain JSON also saved.
92 tracking samples all mode0. Ordinary moving heading error max .01345 rad,
reported cross-track max .00491 m. These controller metrics do not disprove physical
wheel judder or short transient yaw: must reconcile with IMU/wheel/RPM quantization
and user's repeat observation. Navigation stays idle, no turn or speed increase.
