# Speed profile review: 20261025_104553

Read-only review, no further motion/configuration change.
Duration 281.73 s; 2580 tracking samples. 109 distinct path arrays, 46 with
adjacent turns >30 degrees; median maximum angle 4.157 degrees, p90/max 90.
Cached truncations count separately; these are not independent failed routes.
One tracking episode maximum cross-track error 0.135 m; automatic transient
flags are heuristics and do not establish oscillation independently of replans.
Four heartbeat warnings, three planner aborts. Controller parameter dump empty;
state log confirms smoother limits 0.4 m/s and 0.8 rad/s. Recorded /cmd_vel_nav
angular maximum 0.8, downstream maximum 0.571 does not prove a lower hard cap.

Proposal: replace empirical curvature slowdown with a path-indexed feasible
speed envelope, not a universal increase of velocity. At each s use minimum of
cruise, lateral-acceleration sqrt(a_lat/abs(k)), yaw omega_max/abs(k), per-wheel
speed bounds using track 0.269 m, obstacle/stopping and tracking bounds. Include
curvature derivative to limit angular acceleration (omega=v*k). Backward pass
enforces braking to slower future segments; forward pass enforces acceleration.
Include measured response delay and update when route/obstacles change.

With current preview budget a_lat=0.20 and 80% yaw reserve of 0.80 rad/s,
at R=0.45 m limits are sqrt(0.2*0.45)=0.300 and 0.64*0.45=0.288 m/s.
Thus 0.4 m/s is currently infeasible under those configured limits even if tire
friction is adequate. At R=1.0 m both limits permit 0.4 m/s. Separately current
empirical multiplier 1/(1+abs(k)) is ~0.31 at R=.45 and ~0.5 at R=1;
this creates additional slowdown before heading and other checks. These are
code calculations, not attribution of every low-speed sample to that limiter.

Do not remove slowdown at geometric discontinuities or on unvalidated narrow
paths. Ideal polygon corners have no finite continuous-travel curvature bound.
Mass and geometry alone do not establish rollover threshold without center of
mass/support polygon; tire coefficient alone does not establish trackability.
Next implementation should instrument each limiting stage and validate envelope
against recorded curves before replacing the current speed law. Keep fd9f0f7
as comparison baseline. No claim planner is already problem-free.
