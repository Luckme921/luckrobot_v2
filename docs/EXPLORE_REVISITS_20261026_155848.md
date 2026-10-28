# Global exploration: 155848 diagnosis and candidate repair

## Evidence
Source: auto_diag_20261026_155848.tar.gz, global backend confirmed in state log.
18 dispatched navigation steps, 17 reached; median step 0.405 m, maximum 0.73 m; 16 steps below 0.6 m. Eight viewpoint rejections. Increasing the previous 0.8 m cap alone cannot explain or fix most short moves.
A connector toward (-0.58, 0.86) ended at (0.17, 0.11), followed by reselection of (0.12, -0.09). Several successfully visited viewpoints became eligible again after the 20-second cooldown while frontier evidence remained similar.

## Candidate changes
- Keep an observation destination across intermediate execution steps; revalidate known-space route and Nav2 path before each dispatch. Release it if the frontier disappears, a route is unavailable, a planner rejects it, navigation fails, or motion progress times out.
- Record successful final observation locations. Suppress nearby repeats within 0.30 m while frontier centroid remains within 0.40 m and attributed unknown area differs by less than 0.25 square meters. This is a geometric heuristic, not exact visibility or room topology; a blocked region remains pending. Region completion still requires map evidence.
- Global backend uses configurable global_max_goal_step=1.60 m. Legacy remains 0.80 m. This is an upper bound, not a forced length; planner and known-space/cost/clearance checks remain. Soft escape limits remain unchanged.
- Motion progress timeout now defers the global viewpoint without applying legacy branch completion.

## Scope and limitations
No wheel commands were issued during repair. The motion controller, collision thresholds, speed settings and mapping algorithm are unchanged.
The 120-second no-map-growth pause remains. Repeated inaccessible candidates can still cause a diagnostic pause; blocked does not mean explored.
Successful-view suppression uses frontier geometry and area, and does not guarantee semantic identity across SLAM corrections. The new destination latch and successful-arrival behavior still require a real closed-loop run. Reject-all replay tests only failure paths.

## Deployment and rollback
Jetson backups: ~/explore_before_155848_fix.cpp and ~/explore_before_155848_fix.h.
Sources: explore/src/explore.cpp and explore/include/explore/explore.h.
Rebuild package explore_lite after restoring these files if rollback is required.
Test with existing navigation launch and: bash ~/ros2_ws/tools/run_auto_diag.sh 180 global
Do not run another explorer or send manual navigation goals during that recording.

## Verification results
Jetson colcon build passed; C++ global region regression passed. Reject-all planner replay on this bag issued 160 planner checks and zero navigation requests; 3 pending reports. This verifies rejection handling only, not successful observation latching or real exploration efficiency. Local and remote SHA-256 match for both modified C++ files. UDP limits remain 4194304 bytes after reboot.

