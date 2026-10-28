# Exploration review: 20261026 204405

## Recorded evidence
Global backend dispatched 17 goals; 16 reached. Median step 0.52 m, 14 below 0.8 m, maximum 1.57 m. The 1.60 m execution cap was active.
Region 12 retained approximately 0.69 square meters while several observation destinations were visited; later the robot returned toward its earlier route. Region 22 returned to (-1.33,1.45) after its attributed unknown area changed from 1.11 to 0.54 square meters. The previous absolute-area-difference predicate incorrectly released old viewpoints when unknown space shrank.

## Changes
1. Successful-view suppression is directional: shrinking area no longer reopens the same viewpoint. Nearby views within 0.40 m remain suppressed while frontier centroid shift is below 0.50 m and attributed area has not increased by more than 0.25 square meters.
2. Each region has a bounded observation budget. Two completed final observation attempts without a 0.15-square-meter cumulative decrease in attributed unknown area cause deferral, not completion. Frontier movement over 0.50 m, area growth over 0.25 square meters, or meaningful area reduction reopens the budget. Intermediate connectors do not consume it. Other global branches remain eligible.
3. Viewpoint utility uses sampled map-ray visibility of frontier points, rejecting endpoints hidden behind occupied or intervening unknown cells. Up to 64 frontier samples are evaluated per candidate, with a conservative 2.5 m visibility-estimate radius. Global frontier identification remains full-map, without robot-local horizon filtering.
4. Score is (known-route length + 0.80 m equivalent per-goal overhead)/(0.10 + estimated visible frontier area) + 0.5*distance to frontier. These are heuristic utility weights, not a calibrated time or true information-gain model.
5. Keep 1.60 m execution cap, locked macro viewpoint, footprint/cost/Nav2 checks, collision protections and motion-controller parameters unchanged.

## Limitations
Frontier-fringe area is a bounded unknown-area estimate, not exact room coverage. Region identity changes, delayed mapping or geometric jitter may affect the budget. Deferral can leave a region pending until new map evidence arrives; the explorer must not claim completion from this. No physical navigation was issued during repair. Closed-loop efficiency and arrival behavior require user testing.

## Regression and deployment
New test tools/test_observation_progress.cpp reproduces the 1.11 -> 0.54 area-shrink regression, repeated unchanged frontier attempts, separate branch eligibility, and reopening on map evidence.
Backups on Jetson: ~/explore_before_204405.cpp and ~/explore_before_204405.h. Restore these two and rebuild explore_lite to roll back; the extra helper header may remain unused.
Use existing navigation launch, then source ~/ros2_ws/tools/robot_env.sh and bash ~/ros2_ws/tools/run_auto_diag.sh 180 global. Do not run a second explorer or manual goals concurrently.

Verification: Jetson build passed (40.6 s); regression passed; reject-all replay: 152 planner checks, zero navigation actions, four pending reports. This validates rejection behavior only. Three deployed source hashes match local files. Last sampled evaluation timings 71.9–103.0 ms.

