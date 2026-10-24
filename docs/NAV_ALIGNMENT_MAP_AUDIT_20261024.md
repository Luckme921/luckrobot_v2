# Alignment handoff and map-layer audit: run 192816

Pre-change code 832cb8c. User reports good overall motion but initial heading
offset, small straight corrections and old obstacle evidence in lower-left RViz.
Bag SHA256: 5d7c2b5d51143092c051f59411bd911e9b256aafc9f07bc57b30c3ce147cf8ab.

Seven nonterminal rotate->translate transitions in this bag occurred at
7.98–8.50 deg remaining carrot bearing and |EKF yaw rate| .145–.202 rad/s.
Five immediately requested about .23 m/s (downstream acceleration limiting
still applies). Prior runs 185427 and 190648 have six and eight such transitions;
the relaxed handoff is not a new consequence of the .40 m lookahead.

Change rotate_exit_angle .15 -> .08 rad (4.58 deg). Add
rotate_exit_yaw_rate .12 rad/s. While in alignment, reaching the angle threshold
with excessive actual yaw commands zero angular velocity and holds translation;
once both conditions pass, normal v+w tracking resumes. Existing collision
prediction still checks coasting. Breakaway cannot override this zero-command
settling phase. Normal translating corner control, speed caps, .40/.16 m
lookahead, goal yaw tolerance and recovery remain unchanged. This can add a
short alignment pause; it does not require stopping at ordinary path corners.

Near-straight selection in the current bag: 445 samples, cross-track p50 .00674 m,
p90 .01958 m, yaw-command p90 .13839 rad/s, adjacent sign changes 18 (10 above
.01, 3 above .03 on both samples). Previous 190648: 741 samples, p50 .00827 m,
p90 .01897 m, 47 changes (28/7). Different routes and selections mean this is
not a controlled proof of improvement; do not keep tuning straight gains blindly.

Map facts: /scan_camera_clear has 3476 recorded messages. SLAM consumes
/scan_filtered, map_update_interval=2 s, minimum travel distance/heading=.2.
It is a pose-graph-derived historical occupancy map, not a latest-frame dynamic
obstacle grid. Source: https://github.com/SteveMacenski/slam_toolbox .
Finite new rays should clear observed free space in obstacle layers; unseen or
invalid depth should not. A low lidar ray cannot prove a high camera obstacle
absent. Camera clearing does not overwrite SLAM static occupancy.

Final combined costmaps have many high-cost cells over static free/unknown
cells, but inflation and sensor-height differences also produce this. Without
the event location/time, that overlap does NOT prove a ghost or identify its
layer. The screenshot is a composite, not isolated /map. No static-map erasure,
blind TTL, sensor mask loosening or SLAM parameter change is deployed here.

Add 1 Hz /diagnostics/global_camera_obstacle_grid from the camera layer before
inflation/master combination and include it in the recorder. It allows direct
comparison with /map and final costmaps in future events. This is observability,
not a claim that historical dynamic objects are solved. Remaining work: verify
whether the complained cell is static occupancy or remembered camera occupancy,
then evaluate observed-free updates with sensor/height and localization checks.

Validation: actual controller handoff tests hold at .20 rad/s residual yaw,
release at .05, and keep aligning at ~8 deg; prediction/corner regression;
camera layer ray clearing/resize tests; 12 camera-filter tests; launch bounds.
No physical movement commanded. Backup backups/20261024_alignment/before.tar.gz.
