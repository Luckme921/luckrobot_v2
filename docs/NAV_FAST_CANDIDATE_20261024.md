# Explicit 0.40 m/s candidate — NOT physically validated

Accepted baseline: da9bfdd913b211235ebe09eddea5fc3da1f51cd7,
tag nav-accepted-20261024-020. User accepted the 18:35 run at .20 m/s,
.35 rad/s tracking and .25 rad/s alignment; motor acceleration byte 10.
Bag SHA256: 786eedb0ba9195bb8d0df920e1dd42f8575ee7c8921c1143b698db5ee8a5ee7e.
Raw archive copied to the Windows diagnostics folder, checksum verified.

The new explicit arguments are motion_speed_limit:=0.40 and
motion_angular_limit:=0.70. This permits twice the recorded linear/tracking
yaw caps. Alignment yaw is .50; breakaway is .50 to satisfy controller
invariants, without raising it further to .70. Controller, smoother and guard
share the caps. Endpoint approach distance becomes .75 m above .30 m/s.
Defaults remain unchanged; previous .20 commands still reproduce the limits.

Keep acceleration .20 m/s² and .30 rad/s², smoother deceleration .50/.80,
predictor braking .35 m/s²/.20 rad/s² and response delay .25 s. These are model
settings, not measured brake certification. Do not double deceleration to make
a faster command appear safe in prediction. Nominal stop model changes from
.107 m at .20 to .329 m at .40, before sensor/pose uncertainty margins.

Corner preview remains 1 m with .20 m geometry window, .05 m margin and
.15 m/s² lateral limit. Higher yaw cap flows into its curvature speed bound;
the lateral limit can prevent a tight turn from doubling speed. Keep .35/.16
steering lookahead and existing angular unwind prediction. Recovery remains
.04 m/s reverse and .12 rad/s unwind, each bounded by the requested 3 s window.
Collision monitor keeps 2 s approach prediction, radius .22 m and sensor age
.35 s. At .40 m/s the 2 s prediction covers .80 m at constant speed; it is
not a guarantee against unseen obstacles. Controller rollout includes delay
and braking tail (up to 6 s); at .70 rad/s the .20 rad/s² model needs 3.5 s
to brake, so the rollout budget still covers the nominal 1.5 s command horizon,
.25 s delay and braking. No footprint, obstacle filtering or freshness limits
were weakened. Control frequencies were not raised without load evidence.

Validation: launch substitution tests, invalid bounds, recovery/braking
invariants, corner tests at .40/.70, actual controller configure and command
regressions. No physical fast test performed. First supervised test should be
an open straight, followed separately by a broad turn, with recording and an
operator able to stop. Do not start with narrow passages or dynamic obstacles.

Start with the existing mapping/luck_full/height command plus:
motion_speed_limit:=0.40 motion_angular_limit:=0.70 motor_accel_time:=10
For operational rollback (no git checkout required), use .20 and .35 instead.
