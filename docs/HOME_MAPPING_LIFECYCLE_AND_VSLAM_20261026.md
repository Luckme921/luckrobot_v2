# Household map lifecycle and VSLAM review

## Verified existing implementation
/explore/status uses ExploreStatus (string status) and transient-local QoS depth 10. It publishes exploration_started, exploration_in_progress, exploration_paused, exploration_complete, returning_to_origin and returned_to_origin. Durable DDS history is not persistent storage across publisher exit or reboot. Pending global frontiers currently produce logs; do not interpret inactivity or paused as completion.
Last 205912 run ended with six pending frontier segments, not completion.
tools/save_slam_snapshot.py calls slam_toolbox serialize_map and save_map with response checks. Automatic transactional map management and localization handoff are not implemented by this review.

## Recommended lifecycle
MAPPING -> COMPLETE_CANDIDATE -> STOPPING -> SAVING -> MAP_READY -> LOCALIZING -> NAV_READY.
BLOCKED / PAUSED / ERROR are distinct non-complete states. Candidate completion requires stable absence of eligible remaining frontiers and explicit classification of residual unknown areas; blocked regions are not silently complete.
Save only after navigation cancellation/idle and measured stationary state, with fresh map/TF. Store occupancy YAML/image, serialized pose graph, semantic observations linked to map UUID/version, calibration/configuration metadata. Save to a new version, validate files and service responses, then atomically update the current-version manifest. Saving failure preserves previous good map. Do not overwrite a known good map on mere stopping.
Expose durable state plus session_id, reason, map_id/version/path and timestamp; publish map_ready only after validation. Existing status topic should remain compatible. Persist the manifest on disk; volatile DDS state is insufficient across reboot. Prevent replay of old completion from saving into a new session.
Mode switching must have one map->odom owner. Stop mapping owner before activating AMCL, initialize pose in saved-map coordinates, validate localization and TF freshness, then enable goal dispatch. Never run AMCL and SLAM Toolbox as competing map->odom publishers.

## Daily operation
Prefer saved occupancy map + AMCL + existing Nav2/controller for routine use. Dynamic people/temporary obstacles update costmaps, not permanent structure on every pass. Significant persistent environmental changes trigger controlled mapping-maintenance sessions loading the serialized graph and saving a new version. AMCL does not update maps. SLAM Toolbox localization is an alternative to AMCL, not an additional simultaneous TF owner. Lifelong mapping is an optional later experiment, not an automatic assumption that all transient objects will be removed.
Semantic mapping is separate from visual SLAM: RGB-D segmentation + depth + time-aligned TF can continue with lidar localization. Loop closures/map revisions require semantic landmark reprojection or reconciliation from stored keyframe observations, not blind copying of old coordinates.

## Live VSLAM audit
2026-10-26 SSH: L4T R36.4.3; 7.4 GiB total, about 5.8 GiB available at idle; no running navigation launch or VSLAM container found. Image isaac_ros_dev-aarch64-luckrobot-vslam:latest exists. Idle memory says nothing about concurrent peak load.
Historical verdict docs/VSLAM_FIELD_VERDICT_20260921.md: 535 seconds, wheel path 14.94 m vs visual 63.70 m, nonphysical spikes, prolonged rejected output. 2026-10-17 static guard accepted zero frames due to covariance. These cannot be dismissed solely as old wheel slip or insufficient AI TOPS.
Current stereo_vslam_observe.launch.py disables both TF publications, IMU fusion and localization/mapping; it is visual odometry observation, not a production saved visual map. Its interleaved-mode comment describes 15 Hz effective passive stereo and an 80 ms jitter-warning threshold. NVIDIA release-3.2 specifies >=30 Hz, +/-2 ms frame-period jitter, +/-100 microseconds stereo capture offset. A wider software matching/warning threshold does not satisfy these acquisition requirements.
Next: reproduce inputs with passive 30 Hz stereo; audit raw/rectified CameraInfo, stereo baseline and inverted optical TF, capture timing and dropped frames, projected laser interaction; then static and 10-minute motion/occlusion/relocalization trials. Run no-TF side channel first, retain wheel+IMU and lidar baseline. Evaluate CPU/GPU/memory/temperature and depth-safety timing concurrently before any guarded candidate fusion. Do not fuse the same IMU twice without accounting for correlation.
No container deployment, physical motion or automatic mode transition was performed in this review.

## Primary sources
https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html
https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html
https://github.com/SteveMacenski/slam_toolbox
https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/index.html
