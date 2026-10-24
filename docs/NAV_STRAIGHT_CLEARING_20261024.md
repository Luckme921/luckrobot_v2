# Small straight tracking and camera clearing update

Pre-change source 0138575; user reports good overall motion, mild straight
corrections and a person intermittently leaving a frequently turned corner.
Camera sometimes cannot see that position. Preserve the accepted speed caps.
Bag 190648 archived locally and on Jetson, SHA256:
6e99f78be126552f49f79bf72e887fd68fada2dbdd636915c82c7d996ae8d164.

Read-only selection: ordinary mode, not rotating, v>.10, lookahead bend<.08,
abs bearing<.20 rad. 741 samples: cross-track p50 .00827 m, p90 .01897 m,
max .02560 m. Adjacent command sign changes 47; 28 with both magnitudes>.01,
7 with both>.03 rad/s. This is not a count of visible physical oscillations.
Overall moving cross-track p50 .0120 m,p90 .0451 m,max .0816 m. Filters bound
the selected errors by design; do not generalize this to all straight motion.

Change normal LuckFollowPath lookahead .35 -> .40 m; corner minimum stays .16.
Geometric small-error response is reduced, without an extra delayed filter or
global yaw-gain reduction. No claim that offline data validates the new closed
loop. Keep speed .40, yaw .80, existing braking, preview and recovery limits.

Previously marking and clearing both used temporally confirmed /scan_camera.
New /scan_camera_clear contains latest spatially supported finite returns
(same cluster/range checks, fresh timestamp and bounded TF check), without
waiting for the five-frame marking vote. Both camera costmap layers consume
it with clearing=true, marking=false, inf_is_valid=false, persistence=0.
Unobserved/no-depth rays are never declared free. Empty supported observation
publishes empty clearing scan to replace the last usable scan. No blind TTL
or lidar-based clearing of high camera obstacles is introduced. Existing
confirmed marking still protects occupied endpoints. Recorder includes the
new channel for subsequent verification.

This only removes temporal-confirmation delay when a finite background surface
is visible. It does not clear unknown/out-of-view space, guarantee erasure of
every person trace, or override occupied SLAM static-map cells. Exact event
time/location is unavailable; do not attribute the user's trace to a specific
layer without evidence. Mapping currently includes moving objects; stable
prior-map localization is a separate operational mode, not changed here.

Tests: filter unit cases; both YAML sources explicitly clear-only and finite;
real obstacle-layer finite ray clears intersected hit but retains off-ray hit;
39 map origin/size changes retain unseen obstacle and do not resurrect cleared
hit; controller/corner/motion regression and launch substitution checks.
Physical testing is pending. Backup backups/20261024_straight_clear/before.tar.gz.
