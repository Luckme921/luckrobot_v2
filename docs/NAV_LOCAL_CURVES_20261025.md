# Independent corner smoothing after constrained optimization fallback

Follow-up to b1d1f21 using existing 20261025_100055 and 20261024_224959
reconstructed costmap fixtures. No new motion commands or controller tuning.

Problem: the seed's older quadratic blends used outline cost threshold 253,
while the constrained optimizer/controller use 254 at outline and separate
center clearance. This double inflation rejected otherwise feasible local
blends. Whole-path optimization failure then exposed the remaining sharp seed.

Only when constrained optimization fails, fallback corner checks use lethal/unknown
outline exclusion, retain the existing strict center lineIsClear check, and
sample at quarter-cell spacing. The maximum trim distance starts at the larger
of configured corner trim and preferred radius (currently 0.45 m), still limited
to 40% of each adjacent segment. Each corner independently halves trim up to
four attempts, down to the existing 4 cm floor. Valid corners survive failure
elsewhere. This is a quadratic tangent blend, not a circular arc or G2 curve;
trim distance is not the actual turning radius. The first Ceres attempt and its
seed remain unchanged to preserve previously accepted solutions. An initial
experiment changing that seed regressed some cases and was not deployed.

Legacy mode constrained_curve_enabled=false retains its former trim and
outline threshold. No changes to inflation, velocity/controller, camera/lidar
clearing, EKF or motor settings. Physical tracking remains to be validated.

Added regression: two consecutive corners, first forbidden to round, second
free; preserve the first and smooth the second, verify all produced segments.
Existing lethal/unknown, exact endpoint, controller prediction tests retained.

Replay fixtures are lossy reconstructions of OccupancyGrid and are not physical
motion simulations. Adjacent vertex angle depends on sample spacing and is not
a measurement of actual wheel trajectory or guaranteed curvature continuity.

Final isolated Release build: five CTest tests passed; 13 latest and 20 previous
recorded snapshots replayed. Previous-bag case 10 fallback improved maximum
vertex angle 33.28 -> 3.80 degrees. Latest case 6 preserves additional rounded
corners (178 -> 216 sampled poses) but maximum angle remains 42.27; case 11
remains 45 degrees. Therefore this is a limited fallback improvement, not a
solution for all narrow turns. Latest total plan times approximately 1.7–13.4 ms
without concurrent robot navigation load. Do not compare Debug and Release
optimizer timings/outputs: solver time budget and convergence differ.

Earlier initial-seed experiment reduced case 6 to 13 degrees in Debug but
regressed previous cases and failed to reproduce that benefit in Release.
That experiment was discarded; do not count it as a deployed benefit.
