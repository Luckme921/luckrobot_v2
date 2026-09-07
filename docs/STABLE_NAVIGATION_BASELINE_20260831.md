# Stable navigation baseline — 2026-08-31

This checkpoint contains only the physical motion, Nav2 controller, collision
monitor, launch, and diagnostic files that have passed manual tests. Automatic
exploration remains a separate, not-yet-accepted layer.

## Accepted runtime behavior

- `controller_server.odom_topic`: `/odometry/filtered`
- maximum commanded speed: `0.15 m/s`, `0.35 rad/s`
- rotate-to-path mode switch: `0.35 rad` (about 20 degrees)
- approach slowdown distance: `0.35 m`
- goal tolerance: `0.08 m`, `0.10 rad`
- chassis linear deceleration: real elapsed-time integration at `0.50 m/s^2`
- collision monitor: lidar and depth enabled, circular approach model,
  `0.19 m` radius and `2.0 s` time-to-collision

## Physical evidence

- `manual_nav_20260831_104019`: fixed the slow initial rotation; angular speed
  reached about `0.25 rad/s`.
- `manual_nav_20260831_112014`: long goals completed with about `4.5 cm` and
  `2.1 cm` final map-frame position error; post-success wheel motion was at
  most `1 mm`.
- `manual_nav_20260831_121241`: a `2.00 m` goal stopped at map pose
  `(1.954, 0.017)`; the `0.35 m` approach setting reduced final rolling motion.
- `manual_nav_20260831_122529`: dynamic obstacle caused Nav2 to replan and the
  collision monitor to enter approach protection twice; removing the obstacle
  restored normal motion and the goal succeeded.

## Files in the checkpoint

- `src/chassis_driver/src/chassis_node.cpp`
- `src/my_robot_navigation/config/nav2_params.yaml`
- `src/my_robot_bringup/config/collision_monitor.yaml`
- `src/my_robot_bringup/launch/smart_navigation.launch.py`
- `tools/run_manual_nav_diag.sh`

Do not mix experimental exploration controller parameters into these files
without a separate physical regression test.
