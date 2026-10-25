# Curve planner candidate, 2026-10-25

User accepts current motion and dynamic obstacle clearing, requests smoother
variable-radius intermediate turns rather than grid zigzags. Jetson rebooted
during development; no hardware launch or wheel command was issued in this task.

## Implementation

LuckGrid now selects direction-state Dijkstra (zero heuristic) to find the
cost-aware corridor, followed by the installed Humble Nav2 constrained smoother
(Ceres). Existing local rounding remains the seed/fallback. Merely replacing A*
with Dijkstra does not smooth a path; continuous optimization provides that part.

Preferred turning radius 0.45 m is a soft curvature objective, not a guaranteed
minimum radius or G2 continuity guarantee. Optimizer nodes approximately 0.10 m,
upsampling 4, maximum 300 nodes, 50 iterations, 80 ms solver time budget. Total
planning includes search, solver overhead and validation and can exceed 80 ms.
Controller curvature preview and collision checks remain authoritative for speed.

Every accepted curve keeps endpoints, checks quarter-cell samples and connecting
grid segments, excludes unknown/lethal footprint cells and center costs above
the allowed seed corridor threshold (at most 252), limits displacement from seed
to 0.45 m and length to 1.25 times seed plus 0.05 m. Failed optimization or
validation uses the existing seed. Thus narrow/unknown corridors can retain
corners; this does not promise all paths become smooth.

Outline checking uses lethal threshold 254, matching the controller. Using 253
at both the center and outline double counted inflated robot clearance and
initially rejected 19/20 cases. Actual obstacle/unknown outline checks remain.

No changes to controller source (identical to 9fe0799), velocity limits, motor
parameters, camera/lidar clearing, EKF, BT or inflation settings in this change.

## Recorded-data validation

Bag: custom_nav_20261024_224959.tar.gz
SHA256: 0dc3e58a89c5f3f50a09917f27f50d3f663018f7b5f62400fcae1910aaf0efb4

tools/export_planner_replay.py reconstructed 20 map/start/goal cases from recorded
OccupancyGrid plus updates. OccupancyGrid inversion is quantized/lossy, so this is
offline regression evidence, not exact raw-costmap replay or physical validation.
Both baseline and candidate run against identical reconstructed cases.

16/20 candidate curves accepted; four retained seeds (three center-cost rejects,
one outline collision reject). Examples of largest adjacent-segment angle in
degrees: 40.94 -> 4.32; 43.30 -> 6.33; 46.11 -> 12.34;
78.38 -> 8.96. One accepted case still has 42.18 degrees, so sharp geometry is
not eliminated universally. Pose spacing differs: vertex angle alone is not a
curvature or tracking-error metric. Full candidate planning about 15–127 ms on
the Jetson without navigation load; loaded real-time behavior remains untested.

Five CTest tests passed in isolated candidate overlay: path rounding, actual
planner/optimizer integration, corner speed, motion prediction, controller
prediction. Added outline inflated/lethal/unknown distinction regression.

## Rollback and testing

Previous installed baseline commit: 6a2a519. To restore previous planning behavior
without changing controller, set LuckGrid.use_astar=true and
LuckGrid.constrained_curve_enabled=false in nav2_params.yaml, then restart launch.
Do not hot-switch during motion. Candidate is not yet physically accepted.

First physical comparison: same open L-shaped route, then S-shaped route, one
goal at a time with existing 0.40 m/s / 0.80 rad/s limits and diagnostics running.
Observe corner cutting, unnecessary stops and tracking error; keep current speed
limits until this planner comparison is accepted.

## References

- https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/smoother_plugins/constrained_smoother/configuring_constrained_smoother/
- https://github.com/ros-navigation/navigation2/blob/humble/nav2_constrained_smoother/src/constrained_smoother.cpp
- https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/

Hybrid A* remains A* and adds car-like curvature primitives. For this circular
differential-drive robot, retaining a flexible corridor search and bounded
continuous optimization avoids imposing one hard car-like turning radius.
