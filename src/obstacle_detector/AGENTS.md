# AGENTS.md — obstacle_detector

Package-level rules for the detector-only `obstacle_detector` package. These rules inherit and
must not weaken the repository-root `AGENTS.md`.

## Scope

This package contains one C++ ROS 2 Jazzy runtime node:

- `obstacle_detector_node` (ROS node name `obstacle_detector`) consumes 2D LiDAR, the occupancy
  map, global waypoints, ego odometry, and TF. It publishes existence-confirmed Unknown/Static
  objects on `/static_obs`, a Static-only view on `/confirmed_static_obs`, and the nearest
  Dynamic opponent on `/opp_obs`.

Path planning, overtaking, avoidance-waypoint generation, and driving-state arbitration are
explicitly outside this package. Do not add an overtake planner, `/state`, `/avoid_waypoints`,
`/overtake_waypoints`, or planner-specific parameters back into this package.

## Runtime and messages

- Runtime code must remain C++.
- Use `sensor_msgs/msg/LaserScan`, `nav_msgs/msg/OccupancyGrid`, and
  `nav_msgs/msg/Odometry` for standard inputs.
- Use `f110_msgs/msg/WpntArray` for `/global_waypoints`.
- Use `f110_msgs/msg/ObstacleArray` for `/static_obs`, `/confirmed_static_obs`, and `/opp_obs`;
  do not create a new obstacle message. `/confirmed_static_obs` is the confirmed-only Layer-2
  interface for persistent map consumers; it must not change the existing `/static_obs` contract.
- Use `visualization_msgs/msg/MarkerArray` only for the RViz mirrors. `/static_obs/markers` must
  mirror the final `/static_obs` Frenet envelopes, and `/opp_obs/markers` must mirror the final
  `/opp_obs` Frenet envelope. Do not build these markers from Cartesian AABB metadata.

## Detection pipeline

Keep the scan-driven pipeline ordered as follows:

1. Require a valid CLCS reference built from `/global_waypoints`. The upstream global planner
   periodically republishes identical geometry; ignore those retransmissions so active tracks and
   physical-ID memory survive. Rebuild CLCS and clear tracker state only when `x/y/s/d_left/d_right`
   geometry actually changes.
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
7. Apply the viewing-window and track-boundary gates. Test the projected envelope's lateral
   EDGES against the corridor, not just its centre: wall-hugging fan scatter keeps the centre
   inside while its AABB edge already pokes into the wall.
8. Remove clusters that belong to known occupied map structure.
9. Scale each detection's Kalman measurement covariance by its range, point sparsity, and fresh
   ego-odometry yaw rate. Bound the scale and ignore stale/non-finite motion data.
10. Track surviving detections with the constant-velocity Frenet Kalman state
    `[s, vs, d, vd]`. Associate with the existing physical Frenet hard gate, then the
    detection-specific Mahalanobis gate, then deterministic Frenet-distance-ordered greedy 1:1
    assignment. Mahalanobis is a gate, not the sorting cost. Treat the public `Obstacle.id` as a
    physical-object ID, separate from the internal Kalman-track instance. After primary
    assignment, reconnect unmatched tracks whose Frenet AABB edge gaps still form the same
    configured spatial cluster regardless of `Unknown`/`Static`/`Dynamic` motion state. New split
    tracks in that cluster share the public ID, and confirmed IDs remain available for matching
    re-detections after track retirement. When a confirmed track first becomes statistically
    Static, freeze that measured Frenet footprint and map-frame AABB as its stable identity anchor;
    later viewpoint drift and false Dynamic evidence must not move it. Tracks that never become
    Static fall back to their last associated measurement, never a prediction-only Kalman
    position. Re-identify dormant IDs when either the Frenet envelopes or measured map-frame AABBs
    form the same configured spatial cluster. Clear this identity memory on CLCS rebuild. In parallel,
    maintain a classification-only map-frame CV Kalman state `[x, vx, y, vy]` from associated
    AABB centres.
11. Keep existence and motion state machines separate. Existence uses
    `Raw -> Tentative -> Confirmed` with measurement votes inside `confirmation_window`.
    `Raw/Tentative` never publish. Only `Confirmed` tracks collect map-velocity chi-square
    evidence and transition among `Unknown`, `Static`, and `Dynamic`. Motion status only selects
    the output layer; the same public physical-object ID must be used on `/static_obs`,
    `/confirmed_static_obs`, and `/opp_obs`.
12. Compute `Tv=vᵀPv⁻¹v` from the map KF velocity and covariance through a regularized Eigen LDLT
    solve. Never form an inverse. Vote only on associated measurements; prediction-only frames add
    neither motion evidence nor map-position history. Require map-position RMS persistence for
    Static, and use stricter observation, vote, and RMS gates for `Dynamic -> Static`.
13. A non-dynamic track enters `/static_obs` only while its envelope-stability streak reaches
    `envelope_stability_frames`: consecutive matched frames whose measured centre and extents stay
    within `envelope_stability_tolerance_m`. Fan-shaped morphing clusters never settle and stay
    unpublished; stable real obstacles pass at the same hit as existence confirmation.
14. Merge confirmed tracks only within the same static/dynamic layer.
15. Preserve each measured cluster's independent Frenet footprint and map-frame Cartesian AABB
    through Detection and Track. Smooth the Frenet extents per matched measurement with
    fast-grow/slow-shrink magnitude filtering (`extent_shrink_alpha`) so per-scan AABB flapping
    (square box vs real shape) does not flick the published envelope; expand immediately, relax
    gradually. For each same-layer component, union only currently visible
    member AABBs, reproject that complete union once, and publish matching Cartesian and Frenet
    bounds. A predicted-only component keeps its last measured Frenet footprint around the
    predicted Kalman centre but must set `has_cartesian=false`; never expose a stale raw scan
    footprint as current geometry. Drop a predicted-only component whose merged envelope exceeds
    `max_obs_size`: fan-scatter ghost blobs can grow until they span the corridor and
    false-block planners.
16. Publish all merged Unknown/Static objects on `/static_obs`, the Static subset on
    `/confirmed_static_obs`, and at most one nearest-ahead dynamic object on `/opp_obs`.
17. Build the two RViz MarkerArrays from those final published arrays' Frenet bounds. Include
    predicted-only obstacles and distinguish them with lower alpha.
18. Keep perception diagnostics passive. Accumulate beam, cluster, rejection, and tracker event
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
- Layer 2 is every existence-confirmed `Unknown` or `Static` non-map object, published with
  `is_static=true`. `Unknown` is the safety-preserving provisional state.
- `/confirmed_static_obs` is a same-scan, `Static`-only view of Layer 2. It uses the same
  envelope-stability gate and object-level merge as `/static_obs`.
- Layer 3 is the nearest existence-confirmed `Dynamic` object ahead of the ego, published with
  `is_static=false`.
- A fresh track must not be published until its `TrackStatus` is `Confirmed`. Motion promotion
  preserves the track ID and must not create an empty handoff frame between layers.
- Merge tracks only within one layer. Never merge static and dynamic tracks together.
- Layer merge uses the independent Frenet longitudinal/lateral extents. When a visible Cartesian
  AABB union exists, reproject it so the published Frenet envelope describes exactly the same
  current footprint shown by the marker.
- Publish `/static_obs`, `/confirmed_static_obs`, and `/opp_obs` every scan, including empty
  arrays, so downstream consumers receive a deterministic scan-rate tick. The only exception:
  `/opp_obs` (and its marker) is suppressed with a throttled warning while the ego odometry stamp
  is stale beyond `meas_motion_timeout`, because the ahead-ranking would be misplaced.

## Map filtering

- `use_map_filter` should remain enabled for normal operation.
- A live map with obstacles baked into it would erase those obstacles from perception. Use the
  `detector_map_yaml` launch argument to provide an obstacle-free map on
  `/obstacle_detector/map`; do not solve this by disabling the map filter.
- Keep `max_obs_size` above the diagonal of the largest supported stationary obstacle.

## Parameters and launch

- All tunables belong in `config/obstacle_detector.yaml` and must be declared with safe defaults
  in the C++ node.
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
- `src/obstacle_tracker.cpp` — Frenet Kalman association, map Kalman motion estimation, existence
  status, chi-square voting, map-position persistence, hysteresis, and confidence.
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
- `test/test_obstacle_tracker.cpp` — motion-state transition, spatial reassociation, retired-ID
  reuse, and ID-continuity unit tests.

## Verification

- Build with `colcon build --packages-up-to obstacle_detector`.
- Confirm that only `obstacle_detector_node` is installed by this package.
- Launch in an isolated `ROS_DOMAIN_ID` and verify clean startup and shutdown.
- Run `test/synthetic_opponent_test.py` against a fresh detector process.
- Confirm `/static_obs` contains confirmed-existence Unknown/Static objects,
  `/confirmed_static_obs` contains only Static objects, a moving Unknown object moves to
  `/opp_obs` with the same ID, and Static objects never leak into
  `/opp_obs`.
- Confirm every published object has finite Frenet bounds with `d_right <= d_left`; a closed-track
  wrap may make `s_start > s_end`. Visible merged objects must also have a matching current
  Cartesian AABB.
- Keep the Korean node document, README pair, configuration comments, and launch examples in sync.
