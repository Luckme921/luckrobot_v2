# Slightly faster turn candidate, not physically validated

Accepted predecessor: c95bdfc, tag nav-accepted-20261024-040; tested source
46d305f. Accepted bag 185427 SHA256 (local/Jetson match):
5aeedab35735173413b43e304321e8d9c4705e8a46c31d6086761269a771de96.

Use motion_speed_limit:=0.40 motion_angular_limit:=0.80 motor_accel_time:=10
with mapping/luck_full/height. This raises tracking yaw .70 -> .80 (+14.3%)
and alignment yaw .50 -> .57143 rad/s. Breakaway follows alignment, not full
tracking yaw. Controller, smoother +/- yaw bounds and final guard share the
argument. Corner preview receives the new yaw limit, while lateral acceleration
cap .15 m/s² can still constrain tight curves. Recovery .04 m/s/.12 rad/s and
3 s windows remain unchanged. Linear cap .40, approach .75 m, steering
lookahead .35/.16 m unchanged.

Keep angular acceleration .30 and model angular braking .20 rad/s²; no evidence
supports assuming stronger brakes. The stopping model at alignment cap grows
from .75 rad at .50 to about .959 rad at .57143, so it automatically starts
limiting rotation earlier. Complete nominal rollout at tracking cap .80 is
1.5 + .25 + .80/.20 + .05 = 5.80 s, within the 6 s bound. Launch now rejects
limits incompatible with the complete configured braking horizon. Measured
overspeed may still conservatively invalidate a rollout; the cap is not a
guarantee about physical yaw. No collision/freshness protections were loosened.

Validation: parameter substitution at old/new limits, rejected invalid caps,
controller configure invariants, corner geometry symmetry/density at .80 and
existing motion/controller regression tests. No wheel command sent. User
should test broad left/right turns and terminal alignment before narrow spaces.
Rollback to the accepted settings by using motion_angular_limit:=0.70.
