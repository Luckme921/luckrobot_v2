# 2026-10-24: cruise arcs, obstacle provenance and persistent semantic maps

## Evidence and deployment scope

Input custom_nav_20261024_202805.tar.gz SHA256:
4c3fe4ec1a9fb27c99569b466ede030be322a5b9104018475848a640037f56b5.
Copied to diagnostics/20261023_handoff, hashes match Jetson.
Baseline before this change: 9fe0799. No physical motion commanded.

Selected straight samples: 494, absolute cross-track median .00938 m,
p90 .01965 m, maximum .04175 m. These are filtered samples, not whole-run
accuracy or ground truth. Adjacent command sign changes above .03 rad/s: 1.
Cruise arc samples (distance > .8 m, normal tracking, v > .02):
curvature .2-.5: n224, median command v .29284;
.5-1: n322, .23685; 1-2: n182, .16019; 2-4: n50, .10525 m/s.
Yaw command versus instantaneous measured yaw p90 discrepancy ~.15-.20 rad/s;
this includes response lag and is not a calibrated motor steady-state error.
There are six planner abort log messages, including repeated no-path outcomes
near (0.22,-0.03), (0.21,-0.06), (0.32,-0.05), and one chassis heartbeat warning.
These are not proved resolved by a speed change.

## Small cruise-only candidate

Multiply curvature_speed_gain by 1-.15*blend, where
blend=clamp((remaining-approach_dist)/.25,0,1).
At the explicit .40/.80 launch, approach_dist is .75 m: unchanged <=.75 m,
smooth blend .75-1 m, full .85 gain beyond 1 m. In-place rotation unchanged.
At fixed curvature the raw gain change generally requests ~4-10% more velocity
in this bag; this is not replayed closed-loop performance. Every downstream
tracking/yaw/preview/braking/cost/trajectory constraint remains active.
Linear cap .40, yaw cap .80, model delay .25, linear brake .35, angular brake
.20, preview lateral bound .15 and smoother limits remain unchanged: more speed
must fit the measured model, not a fictitiously increased braking capability.
Steering is recomputed from the increased linear request to preserve arc shape.
No new terminal boost, lookahead shortening, rotation increase, or blind escape.
Physical validation remains required.

## Clearing and replanning

Both lidar obstacle layers already use clearing:true on /scan_filtered.
Global lidar raytrace max12 m; local3 m. Max-combining free obstacle-layer cells
does not erase occupied cells supplied by /map. Camera height obstacles cannot
be disproved by a low laser beam. Invalid or self-filtered beams are not evidence
of free space; do not blindly enable inf_is_valid for all such beams.
The black points may reside in SLAM history; exact world-cell attribution and
observed-free evidence remain outstanding. No blanket map deletion deployed.
Candidate future mechanism: separate recent laser free/occupied evidence,
time-aligned TF, multiple valid free traversals and confidence hysteresis;
reconcile only supported transient map cells, keeping camera obstacles separate.

luck_full BT replans .5 Hz. Cached remaining route is checked for traversability;
new occupied cells invalidate reuse and cause A*. Safe alternate routes can be
selected, but mapping artifacts can block both routes. Global replan is not a
2-second hard reaction guarantee; local safety reacts independently.

## Recommended lifecycle (design, not deployed this turn)

1. First environment: SLAM Toolbox mapping + independent YOLO11s-seg RGB-D
   object map, confidence >.70 plus multi-view association/depth validity.
2. Save occupancy map, serialized pose graph, object observations/IDs, map UUID,
   calibration/model versions and timestamps as one versioned dataset.
3. Routine known-map startup: map_server + AMCL + semantic prior, verify initial
   pose/convergence after manual relocation. Only one map->odom publisher.
4. Temporary people/objects update dynamic costmaps; semantic objects track
   last_seen, uncertainty and existence evidence. Out of view is not absent.
5. Explicit maintenance session: load SLAM Toolbox graph and continue mapping;
   do not simultaneously publish AMCL map->odom. Validate changes, reproject
   semantics using observation/keyframe anchors after nonrigid graph correction,
   save a new map version, then return to localization.
6. SLAM Toolbox localization is an alternative to AMCL for a controlled A/B test,
   not an obligatory extra stage before AMCL each boot.

On Orin Nano 8GB start with a lightweight 2D geometry + sparse 3D object graph,
not an unbenchmarked full dense Hydra/Kimera deployment. First improve collision-
checked path smoothing / compare Smac2D with the same controller, then benchmark
MPPI separately under navigation + perception load. Do not swap planner and
controller simultaneously or assume newer Rolling options exist in Humble.

## Primary references inspected

- https://github.com/SteveMacenski/slam_toolbox (mapping/localization/lifelong limitations)
- https://github.com/ros-navigation/navigation2 (Humble source for current behavior)
- https://arxiv.org/2305.20026 (use https://arxiv.org/abs/2305.20026; RPP, Autonomous Robots 2023)
- https://github.com/MIT-SPARK/Hydra and https://www.roboticsproceedings.org/rss18/p050.pdf
- https://arxiv.org/abs/2305.07154 (hierarchical persistent maps / loop-closure correction)
- https://arxiv.org/abs/2101.06894 (Kimera dynamic scene graphs)
- https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/
- https://github.com/ros-navigation/navigation2/blob/main/nav2_mppi_controller/README.md

Venue reputation is not proof of suitability, embedded runtime, or deletion
correctness. No impact-factor ranking or Orin runtime claim has been established.
