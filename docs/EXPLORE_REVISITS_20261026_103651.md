# 103651: repeated observation endpoints

Bag extracted on Jetson under `~/nav_analysis_auto_103651`.
Several goals succeeded. Different frontier identities selected endpoints near
(0.49,0.19) and (0.44,0.18), interspersed with connector backtracking. Later
branch #9 reached its viewpoint and selection waited with 24 unresolved
frontiers. This is selection starvation, not evidence of motor execution failure.

Confirmed code defect: UNKNOWN gain limited flood depth from frontier cells but
did not limit Euclidean range from the observation point. The advertised 1.5 m
observable radius was therefore not enforced. Added the observer-range bound.

Added a soft staging score penalty within 0.50 m of successful observation
endpoints: maximum 2.0 at the endpoint, linearly declining to zero. Only final
observation endpoints are remembered (deduplicated at 0.15 m, bounded to 256);
transit steps and explicit DFS return anchors are not recorded. No hard
blacklist, collision-threshold relaxation, or controller tuning was introduced.
This heuristic requires physical validation and is not a proof of coverage or
loop freedom. Smaller corrected gains may legitimately reject more frontiers.

0.80 m segment limit retained. Remaining limitations include choosing only one
best staging candidate per frontier and premature branch exhaustion when all
current candidates are filtered. Do not describe the exploration problem as
fully resolved merely because the package compiles.

Backups: `~/explore_before_103651.cpp`, `~/explore_before_103651.h`.
Build log: `~/explore_103651_build.log`.
