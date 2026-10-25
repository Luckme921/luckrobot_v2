# Path speed envelope, first bounded rollout

Latest measured total mass from user: **9.5 kg**, supersedes earlier 7.8 kg
estimate. Track 0.269 m and wheel diameter 0.100 m remain project values.
Do not assign this total to an arbitrary URDF link or invent inertia/COM.
At 0.4 m/s, 0.45 m radius: lateral force 3.378 N, lateral acceleration
0.356 m/s^2. Ideal friction coefficient requirement still 0.0363 because mass
cancels. Motor torque and actual braking capability do not follow from that.
Existing measured-response-based delay/braking limits retained; no friction or
rollover guarantees inferred from mass alone.

## Sources and design

- RPP paper: https://arxiv.org/abs/2305.20026
- Humble project: https://api.nav2.org/nav2-humble/html/md_nav2_regulated_pure_pursuit_controller_README.html
- TOPPRA paper (IEEE TRO): https://arxiv.org/abs/1707.07239
- Implementation: https://github.com/hungpham2511/toppra

Retain existing pursuit steering, collision checking and recovery. Borrow
path-dependent velocity constraint/backward braking concepts; do not claim to
implement TOPPRA, time-optimal torque control, MPC or a full dynamic model.

New path_speed_envelope.hpp computes curvature using both 0.10 and 0.20 m
windows, sampled every 0.025 m. Limits by cruise, 80% yaw reserve and lateral
acceleration 0.20 m/s^2. Preview horizon is at least 1 m and covers modelled
stopping distance plus delay/window margin. Each future cap is propagated
backward with braking and delay. Hairpins receive tight limits, not zero
curvature. Existing downstream smoother handles acceleration; this is not a
full-path forward/backward time parameterization. Per-wheel torque and curvature
derivative constraints are not explicitly solved in this rollout.

New use_path_speed_envelope flag controls the feature. Before the terminal
approach, blend away from the old empirical request toward the feasible
envelope, capped at 25% uplift per control evaluation relative to that old
request (not compounded over time). Blend fades with cross-track error (5 cm)
and mismatch between measured yaw and measured linear speed times pursuit
curvature (0.20 rad/s). This is a tracking gate, not a slip detector. Existing
heading, obstacle cost, angular saturation, tracking-response, acceleration,
trajectory checks and collision monitor still apply after request generation.
Straight cruise remains capped at 0.4 m/s. Pure rotation and terminal approach
retain their former request logic; tighter geometric preview still applies.

Diagnostics append tracking_error indexes 28 legacy request, 29 envelope cap,
30 blended request; -1 means not evaluated. Existing fields unchanged. Final
command remains indexes 4 and 5; later safety reductions can lower it.

## Data and tests

110926 bag: 608.32 seconds, 5616 tracking samples, 239 unique path arrays,
36 with >30 degree adjacent angles, max 90. Cached truncations count separately.
Wheel feedback ~31.2 Hz, odometry ~50 Hz, IMU ~101 Hz. These arrivals do not
establish physical tracking accuracy; isolated angle sign changes do not prove
oscillation on replanned routes. Planner not declared problem-free.

Standalone C++ replay runs the actual envelope on exported path geometry:
- 110926: 239 paths, 4772 sampled locations, 3039 permit cruise geometrically,
  1625 have tighter preview than old 0.20 m-only geometry window.
- 104553: 109 paths, 2848 locations, 1552 permit cruise geometrically,
  1248 have tighter preview.
All caps finite and in [0,0.4], no old preview constraint lost. These are NOT
physical simulations, travel time gains or predictions of final commanded speed.
Five candidate CTest regressions pass, including mirrored/density-invariant
preview, sharp corner limit, bounded uplift and degraded-tracking gates.

Rollback: set use_path_speed_envelope=false and restart launch to restore
fd9f0f7 speed decision behavior. Keep comparison route and speed limits fixed.
No planner or obstacle-clearing changes in this rollout. Real motion untested.
