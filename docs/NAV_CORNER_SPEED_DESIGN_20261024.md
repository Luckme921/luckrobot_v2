# Corner-speed planning: baseline and next candidate design

User accepted motor10/.15 run custom_nav_20261024_170030 qualitatively, but finds
it slow. Original archive downloaded locally. Keep this version as regression baseline.
Ordinary moving cross-track absolute p50=.01218 m,p90=.03668,p99=.05462,max=.06108.
Heading error p50=.07535 rad,p90=.31632,p99=.58427,max=.62802.
Tracking modes: normal2722, sampled avoidance25, unwind5, reverse40, other2=6.
Do not compare these counts as success rates or assert equivalent routes to old bags.

## Verified current limits

Update: user requested both recovery windows increase to 3 s. Source constant,
header default and YAML now agree. Unwind nominal angle .36 rad (20.6 degrees),
reverse nominal distance .12 m; collision checks unchanged. Previous values below
describe the accepted recorded baseline, not the new recovery candidate.

Normal lookahead .35 m; adaptive corner .16 m. Smaller lookahead amplifies curvature
response for a given lateral displacement. Do not globally shorten as speed rises.
Unwind .12 rad/s, blocked elapsed window <2 s (nominal .24 rad/~13.8 degrees).
Reverse .04 m/s, cumulative command budget2 s per goal (~.08 m nominal), optional
yaw0/±.12; trajectory must be collision-valid and improve center cost by >=2.
These are time/command bounds, not measured-displacement guarantees. Actuator lag,
callback delays, repeated goals and actual odometry must be considered separately.

## Next implementation boundary

Do not alter accepted baseline or shrink footprint/clear unseen camera obstacles.
Add a separate future-path speed preview, distinct from steering lookahead:
1. Derive turn severity from distance-spaced geometry, not noisy adjacent points.
2. Determine feasible turn speed from curvature, yaw-rate and lateral-acceleration
   constraints; collision-check any geometrically rounded path before accepting it.
3. Back-propagate speed limits along arc length using conservative deceleration,
   including command/actuator delay and localization margin. Apply as upper bound
   to existing approach/obstacle/tracking limit, never a minimum-speed boost.
4. Keep present steering and downstream collision/freshness gates; don't add
   competing yaw PI or blindly preserve forward speed while yaw cannot track.
5. Keep rotate-in-place for tight corners/large orientation errors when no safe
   continuous arc exists. Never force a radius through obstacles for smoothness.

Model illustration only: d_stop=v*T+v²/(2a), T=.25 s,a=.35 m/s² gives .070 m
at .15, .107 m at .20, .204 m at .30. These parameters are model assumptions;
motor10 bench rise-time is not stopping delay, and these distances are NOT certified
safety clearances. Need loaded longer straight and corner datasets before tightening.

Candidate testing must include straight/no slowdown, mirrored corners, sparse/dense
equivalent paths, short-segment S turns, zero-length segments, replanning continuity,
terminal approach and footprint collision regression. Compare against this baseline
offline then .15/.20/.30 supervised stages. High-speed robustness pending.

## Implemented candidate (2026-10-24)

`corner_speed.hpp` samples a 1 m future path horizon every .025 m using arc-length
interpolation. A +/- .20 m geometry window estimates curvature, independent of
planner vertex density. The cap combines 80% of configured maximum yaw rate and
a provisional .15 m/s² lateral acceleration limit. It propagates the speed cap
backward with the existing .35 m/s² braking and .25 s response model, including
the geometry window and an additional .05 m margin. These are conservative design
assumptions, not identified plant dynamics or certified stopping distances.

Controller applies this cap before steering, cost regulation, sampled avoidance
and collision prediction. It cannot increase speed or override endpoint braking.
`LuckFollowPath.use_corner_speed_preview: true` enables it; false restores the
previous tracking logic without undoing the requested 3 s recovery limits.
Steering lookahead remains .35/.16 m. No geometric path rounding was added.
The existing continuous v+w tracking remains responsible for feasible arcs;
sharp corners may still require very low speeds or rotation.

Unit regression covers straight, left/right symmetry, sparse/dense equivalent
polylines, duplicates, hairpins, S turns and advance braking/release. Hardware
validation of this new candidate has not been performed. Recommended next
supervised test uses motion_speed_limit:=0.20 and motor_accel_time:=10;
do not interpret build/unit success as validation at .30 m/s.

Remote pre-change backup: backups/20261024_corner_preview/before.tar.gz.
