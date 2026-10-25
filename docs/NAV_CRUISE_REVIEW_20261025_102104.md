# Bounded cruise relaxation and noise review

Bag 20261025_102104: 64 distinct path arrays; 7 contain >30 degree
vertex angles, maximum 32.525 degrees. Different routes and cached truncations
prevent treating this as a controlled improvement percentage. No persistent
repeated heading sign reversals flagged by existing episode analysis. Seven
planning aborts, two controller patience aborts and four chassis heartbeat
timeouts remain. No claim that all navigation issues are solved.

## Friction interpretation

Mass 7.8 kg, track 0.269 m, wheel diameter 0.100 m from project context.
At v=0.4 m/s and R=0.45 m, lateral acceleration =0.356 m/s^2 and force
=2.773 N. Ideal flat-ground friction demand mu=v^2/(R*g)=0.0363;
mass cancels in this simplified inequality. It does not cancel in motor torque
or inertia, and support-wheel load sharing, tire compliance, combined braking,
surface contamination and robot center of mass are not identified by this model.

Primary experiment: https://www.tribology.rs/journals/2014/2014-1/2.pdf
reports a patterned rubber/smooth ceramic combination with static coefficient
about 0.25 dry (0.11 soap). Samples are footwear rubber, not DDSM115 tire tests.
Do not adopt a universal 0.6–0.8 wheel/tile coefficient. A hypothetical mu=0.3
is only a scenario estimate, not a measured or guaranteed lower bound.

## Limited controller change

use_tracking_gated_cruise enabled in current config; false restores old behavior.
Before approach_dist, relax empirical curvature slowdown by at most 15%,
continuously fading to zero extra request at cross-track error 3 cm or lookahead
bearing 0.20 rad. No terminal boost or pure-rotation change. This multiplier
affects requested speed before safety constraints, not final speed by 15%.

Preview lateral acceleration budget changes from 0.15 to 0.20 m/s^2 under the
same switch. This is a conservative control/comfort budget, NOT an estimate of
maximum tire grip. Retains yaw limit with 20% reserve, advance braking, heading
slowdown, acceleration/smoother limits, trajectory collision checks and collision
monitor. Tight arcs still slow: no promise of constant-speed arbitrary curves.
No planner or sensor filtering changes in this patch.

## Startup observation

First 60 seconds of bag, not necessarily device startup: camera planning scan
888 frames, 33 with returns <0.45 m, none with only 1–2 such beams, nearest
0.391 m; lidar 759 frames, 579 with returns <0.45 m, 10 with only 1–2 such
beams, nearest 0.191 m. Max arrival gaps camera 0.125 s, lidar 0.154 s.
This does not identify false positives or prove causality for planning failure.
Do not increase body-radius filtering or delete map obstacles based on this.

Validation: CTest including tracking-gate symmetry/bounds and preview bounds.
Candidate suite initially had planner_rounding cleanup timeout after its PASS
assertions; an isolated rerun passed in 0.21 s. Record this teardown flake rather
than describing the first suite as all passing.
Physical behavior with the new switch remains unvalidated. Record the same
route next; four heartbeat events need separate timeline correlation before
any larger speed relaxation.
