# Exploration 101050: bounded planning requests

Bag: `/home/nvidia/auto_diag_20261026_101050.tar.gz`.
Three dispatched goals (0.59, 0.90, 1.01 m reported displacement) reached.
Later candidate plans repeatedly failed prefix selection with cost -1
(unknown/unavailable global cost), while 17 frontiers remained. This evidence
does not establish a motor stall, nor prove that all frontiers are reachable.

Changes:
- Default and launch maximum segment reduced from 1.20 to 0.80 m.
- Before LuckGrid precheck, use the existing known-free topology route to pick
  a bounded intermediate endpoint with unchanged 0.25 m clearance and cost <=20.
  LuckGrid and existing prefix checks still validate the actual navigation step.
- Retain original branch destination for completion bookkeeping. Reaching an
  intermediate endpoint is not branch completion.
- Count robot-to-first-path-point displacement in prefix length.
- Handoff proximity reduced from 0.45 to 0.25 m for shorter segments.

If no suitable intermediate point exists, the existing full precheck remains;
its output is still bounded/rejected by prefix validation. This is not permission
to travel in unknown space. Single best staging candidate and repeated cooldown
cycles remain possible limitations. Physical validation is pending.

Backup on Jetson: `~/explore_before_101050.cpp`.
Build log: `~/explore_101050_build.log`.
