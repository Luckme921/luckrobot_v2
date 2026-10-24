# Planner geometry candidate after accepted-controller rollback

Baseline: 07a80fe; controller remains byte-identical to 9fe0799.
No motion commands, controller tuning or speed-ceiling changes in this task.

## Evidence

Downloaded custom_nav_20261024_223400.tar.gz, SHA256
ae448e79b0ea744e52fb9599aaaee605265cd5022b0498f9af322efa28d0dcc4.
Selected straight tracking: n607, absolute cross-track median .00843 m,
p90 .02078 m. Five chassis heartbeat warnings and three planner abort messages
remain independent investigation items; smoothing does not establish a fix.

Recorded path geometry, unique rounded-coordinate arrays (cached truncations
count separately, not independent routes):

| bag | arrays | arrays containing >30-degree vertex turn | largest vertex turn |
| --- | ---: | ---: | ---: |
| 202805 | 77 | 49 | 50.60 deg |
| 221927 | 142 | 71 | 52.13 deg |
| 223400 | 131 | 73 | 75.08 deg |

Different routes prevent causal comparisons. These figures demonstrate that
the existing line-of-sight simplification/resampling retains sharp vertices;
they do not demonstrate candidate real-world performance.

## Implementation

Keep direction-state, cost-aware A* search and its existing penalties. After
line-of-sight simplification, use tangent quadratic Bezier transitions at
eligible corners. Trim at most .25 m per side and .4 of each adjacent segment,
preventing overlap between neighboring corner blends. Angle <.10 rad remains
straight; >2.35 rad remains an explicit hairpin. Try at most four sizes by
halving; below .04 m fall back to original corner.

Every changed chord is supercover checked in the locked global costmap, using
the existing smoothing_max_cost=100 and rejecting unknown cells. Additionally
check the actual robot footprint at <=half-cell spacing along every accepted
chord/connector. Missing footprint disables the blend. Failed validation
retains the original route rather than forcing a corner cut or removing it.
Bezier derivative bound determines sampling density (<=quarter-cell target).
Result is resampled at <=half-cell spacing, preserves endpoint positions and
requested final yaw, and recomputes tangent headings.

This is tangent-continuous corner blending, NOT curvature-continuous trajectory
optimization, a guaranteed minimum-turn-radius planner, or guaranteed faster
motion. Tight corners can retain old geometry. Controller preview, collision
rollout, measured-response braking and speed regulation remain authoritative.

Cached route reuse now checks every segment (including diagonal supercover),
not only vertices. Existing path hysteresis still prevents gratuitous route
switching; a newly blocked segment invalidates reuse.

Parameters: LuckGrid.corner_rounding_enabled=true,
LuckGrid.corner_rounding_distance=.25 (allowed 0-.40 m). Set enabled=false in
configuration and restart to disable corner blending. This does not disable
the cache segment safety check or denser path sampling.

## Validation and next physical test

Jetson colcon build passed. Five CTest tests passed: path_rounding,
planner_rounding, corner_speed, motion_prediction, controller_prediction.
Tests include real plugin free-space smoothing, unchanged endpoints, exact
unknown-space fallback, blocked cached segment, rejected curve, hairpin,
duplicate points and all previous controller/braking regressions.
No physical validation yet. Compare the same left/right moving bends, then
continuous bends and a narrow turn at the accepted .40/.80 launch limits.
Do not raise speed in the same experiment. Record before sending each goal.
