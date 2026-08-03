# AGENTS.md — obstacle_detector

Package-level rules for the detector-only `obstacle_detector` package. These rules inherit and
must not weaken the repository-root `AGENTS.md`.

## Scope

This package contains one C++ ROS 2 Jazzy runtime node:

- `obstacle_detector_node` (ROS node name `obstacle_detector`) consumes 2D LiDAR, the occupancy
  map, global waypoints, ego odometry, and TF. It publishes provisional/confirmed stationary
  objects on `/static_obs` and the nearest confirmed dynamic opponent on `/opp_obs`.

Path planning, overtaking, avoidance-waypoint generation, and driving-state arbitration are
explicitly outside this package. Do not add an overtake planner, `/state`, `/avoid_waypoints`,
`/overtake_waypoints`, or planner-specific parameters back into this package.

## Runtime and messages

- Runtime code must remain C++.
- Use `sensor_msgs/msg/LaserScan`, `nav_msgs/msg/OccupancyGrid`, and
  `nav_msgs/msg/Odometry` for standard inputs.
- Use `f110_msgs/msg/WpntArray` for `/global_waypoints`.
- Use `f110_msgs/msg/ObstacleArray` for both `/static_obs` and `/opp_obs`; do not create a new
  obstacle message.
- Use `visualization_msgs/msg/MarkerArray` only for the RViz mirrors. `/static_obs/markers` must
  mirror the final `/static_obs` Frenet envelopes, and `/opp_obs/markers` must mirror the final
  `/opp_obs` Frenet envelope. Do not build these markers from Cartesian AABB metadata.

## Detection pipeline

Keep the scan-driven pipeline ordered as follows:

1. Require a valid CLCS reference built from `/global_waypoints`.
2. Resolve the scan frame to `map` using the scan header.
3. Reject invalid/out-of-range beams and perform adaptive-breakpoint clustering.
4. Before tracking, merge nearby scan fragments only when their Cartesian AABBs and actual point
   sets are both within `cluster_merge_distance`, and keep the merged AABB within `max_obs_size`.
   Drop any result still smaller than `min_cluster_points`.
5. Reject clusters larger than `max_obs_size`.
6. Build the complete map-frame Cartesian AABB and project it with
   `aabb_frenet_projector`. Use the projected AABB centre as the detection `(s,d)`, preserve
   independent longitudinal and lateral extents, and use the closest race-line/AABB-face
   distance for the race-line-facing lateral bound.
7. Apply the viewing-window and track-boundary gates.
8. Remove clusters that belong to known occupied map structure.
9. Scale each detection's Kalman measurement covariance by its range, point sparsity, and fresh
   ego-odometry yaw rate. Bound the scale and ignore stale/non-finite motion data.
10. Track surviving detections with the constant-velocity Frenet Kalman state
    `[s, vs, d, vd]`. Associate with the existing physical Frenet hard gate, then the
    detection-specific Mahalanobis gate, then deterministic Frenet-distance-ordered greedy 1:1
    assignment. Mahalanobis is a gate, not the sorting cost.
11. Keep hits below `min_hits_confirm` in `Pending` and out of both obstacle topics. At the
    confirming hit, publish the same track/ID immediately as `ProvisionalStatic`. Promote it to
    `ConfirmedStatic` after consecutive low-relative-speed observations, or to `Dynamic` only
    after consecutive velocity-confident motion observations with fresh, bounded ego yaw rate.
12. Merge confirmed tracks only within the same static/dynamic layer.
13. Preserve each measured cluster's independent Frenet footprint and map-frame Cartesian AABB
    through Detection and Track. For each same-layer component, union only currently visible
    member AABBs, reproject that complete union once, and publish matching Cartesian and Frenet
    bounds. A predicted-only component keeps its last measured Frenet footprint around the
    predicted Kalman centre but must set `has_cartesian=false`; never expose a stale raw scan
    footprint as current geometry.
14. Publish all merged statics on `/static_obs` and at most one nearest-ahead dynamic object on
    `/opp_obs`.
15. Build the two RViz MarkerArrays from those final published arrays' Frenet bounds. Include
    predicted-only obstacles and distinguish them with lower alpha.
16. Keep perception diagnostics passive. Accumulate beam, cluster, rejection, and tracker event
    counters over the configured wall-clock interval, but report live track/classification counts
    as a current snapshot. Do not claim noise filtering or deskew statistics in this package;
    those belong to the upstream scan preprocessor that actually performs them.

Pre-tracking `cluster_merge` and post-tracking `layer_merge` are complementary. The first creates
one detection/track from LiDAR fragments; the second consolidates same-layer output tracks. Do not
replace one with the other.

`FrenetProjector` remains responsible for boundary lookup and wrap-aware `s` differences.
`aabb_frenet_projector` is the sole owner of Cartesian obstacle AABB-to-Frenet conversion. It must
use the exported `global_planning::ClcsFrenetConverter`; downstream planners must consume the
published Frenet bounds instead of reprojecting the Cartesian metadata.

## Layer semantics

- Layer 1 is the `/map` and corridor filter. It is never published.
- Layer 2 is every provisional or confirmed non-map stationary object, published with
  `is_static=true`.
- Layer 3 is the nearest confirmed dynamic object ahead of the ego, published with
  `is_static=false`.
- A fresh track must not be published until `min_hits_confirm` sets its `classified` flag. The
  first publishable state is `ProvisionalStatic`; promotion to `ConfirmedStatic` preserves the
  track ID and must not interrupt `/static_obs`.
- Merge tracks only within one layer. Never merge static and dynamic tracks together.
- Layer merge uses the independent Frenet longitudinal/lateral extents. When a visible Cartesian
  AABB union exists, reproject it so the published Frenet envelope describes exactly the same
  current footprint shown by the marker.
- Publish both layer topics every scan, including empty arrays, so downstream consumers receive a
  deterministic scan-rate tick.

## Map filtering

- `use_map_filter` should remain enabled for normal operation.
- A live map with obstacles baked into it would erase those obstacles from perception. Use the
  `detector_map_yaml` launch argument to provide an obstacle-free map on
  `/obstacle_detector/map`; do not solve this by disabling the map filter.
- Keep `max_obs_size` above the diagonal of the largest supported stationary obstacle.

## Parameters and launch

- All tunables belong in `config/obstacle_detector.yaml` and must be declared with safe defaults
  in the C++ node.
- Keep explanatory comments and Python docstrings in Korean. Preserve standard license text,
  identifiers, topic names, formulas, and external API terminology when translating comments.
- `diagnostics_enable` and `diagnostics_period_sec` control the passive INFO diagnostics; they must
  not change scan, detection, association, or Kalman state.
- `launch/obstacle_detector_node.launch.py` is the direct node launch.
- `launch/obstacle_detector.launch.py` is the package entry point and starts the same detector
  plus optional RViz. It must not launch a planner.
- Keep `simulator:=true` selecting `/ego_racecar/odom`; real mode uses `/pf/pose/odom`.
- `use_sim_time` is independent of simulator mode and should be true only when `/clock` exists.

## Layout

- `src/obstacle_detector_node.cpp` — ROS interface and scan-driven detection pipeline.
- `src/aabb_frenet_projector.cpp` — complete Cartesian AABB-to-Frenet footprint projection.
- `src/frenet_marker_builder.cpp` — final Frenet obstacle arrays to map-frame RViz boundaries.
- `src/obstacle_tracker.cpp` — Frenet Kalman tracking and static/dynamic classification.
- `src/frenet_projector.cpp` — boundary lookup, track-length wrapping, and marker-only map
  interpolation.
- `include/obstacle_detector/` — matching C++ headers.
- `config/obstacle_detector.yaml` — all detector parameters.
- `launch/obstacle_detector_node.launch.py` — detector-only launch.
- `launch/obstacle_detector.launch.py` — detector-only package entry point with optional RViz.
- `docs/obstacle_detector_node.md` — Korean operation documentation.
- `docs/sim_test_commands.md` — Korean detector test procedure.
- `test/synthetic_opponent_test.py` — synthetic layer-classification and merge harness.
- `test/test_aabb_frenet_projector.cpp` — independent AABB extent and curved-track projection
  tests.
- `test/test_frenet_marker_builder.cpp` — Frenet-to-map interpolation and final-envelope marker
  tests.
- `test/test_obstacle_tracker.cpp` — motion-state transition and ID-continuity unit tests.

## Verification

- Build with `colcon build --packages-up-to obstacle_detector`.
- Confirm that only `obstacle_detector_node` is installed by this package.
- Launch in an isolated `ROS_DOMAIN_ID` and verify clean startup and shutdown.
- Run `test/synthetic_opponent_test.py` against a fresh detector process.
- Confirm `/static_obs` contains provisional/confirmed stationary objects, a moving provisional
  object moves to `/opp_obs` with the same ID, and confirmed stationary objects never leak into
  `/opp_obs`.
- Confirm every published object has finite Frenet bounds with `d_right <= d_left`; a closed-track
  wrap may make `s_start > s_end`. Visible merged objects must also have a matching current
  Cartesian AABB.
- Keep the Korean node document, README pair, configuration comments, and launch examples in sync.
