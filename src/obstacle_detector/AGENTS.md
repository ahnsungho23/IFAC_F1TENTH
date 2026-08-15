# AGENTS.md — obstacle_detector

Package-level rules for the detector-only `obstacle_detector` package. These rules inherit and
must not weaken the repository-root `AGENTS.md`.

## Scope

This package contains one C++ ROS 2 Jazzy runtime node:

- `obstacle_detector_node` (ROS node name `obstacle_detector`) consumes 2D LiDAR, the occupancy
  map, global waypoints, ego odometry, and TF. It publishes provisional/confirmed stationary
  objects on `/static_obs`, a confirmed-only view on `/confirmed_static_obs`, and the nearest
  confirmed dynamic opponent on `/opp_obs`.

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
- The selected `/opp_obs` obstacle must set `is_interfering` from the parameterized ego/opponent
  Frenet corridor and constant-velocity gap check. This annotation must not change detection,
  tracking, motion classification, or opponent selection.
- Use `visualization_msgs/msg/MarkerArray` only for the RViz mirrors. `/static_obs/markers` must
  mirror the final `/static_obs` Frenet envelopes, and `/opp_obs/markers` must mirror the final
  `/opp_obs` Frenet envelope. Do not build these markers from Cartesian AABB metadata.

## Detection pipeline

Keep the scan-driven pipeline ordered as follows:

1. Require a valid Frenet reference built from `/global_waypoints`.
2. Resolve the scan frame to `map` using the scan header.
3. Reject invalid/out-of-range beams, transform each surviving beam to a map-frame point, and
   drop every point within `wall_assoc_distance_m` of a linear wall structure extracted from the
   SLAM map (the structural wall filter, see "Map filtering" below). Only then perform
   adaptive-breakpoint clustering; a dropped point breaks cluster contiguity like an invalid
   beam.
4. Before tracking, merge nearby scan fragments only when their Cartesian AABBs and actual point
   sets are both within `cluster_merge_distance`, and keep the merged AABB within `max_obs_size`.
   Drop any result still smaller than `min_cluster_points` or whose AABB diagonal is below
   `min_obs_size` (LiDAR noise).
5. Reject clusters larger than `max_obs_size`.
6. Build the complete map-frame Cartesian AABB and project it with
   `aabb_frenet_projector`. Use the projected AABB centre as the detection `(s,d)`, preserve
   independent longitudinal and lateral extents, and use the closest race-line/AABB-face
   distance for the race-line-facing lateral bound.
7. Apply the viewing-window and track-boundary gates. Test the projected envelope's lateral
   EDGES against the corridor, not just its centre: wall-hugging fan scatter keeps the centre
   inside while its AABB edge already pokes into the wall.
8. Scale each detection's Kalman measurement covariance by its range, point sparsity, and fresh
   ego-odometry yaw rate. Bound the scale and ignore stale/non-finite motion data.
9. Track surviving detections with the constant-velocity Frenet Kalman state
    `[s, vs, d, vd]`. Associate with the existing physical Frenet hard gate, then the
    detection-specific Mahalanobis gate, then deterministic Frenet-distance-ordered greedy 1:1
    assignment. Mahalanobis is a gate, not the sorting cost.
10. Keep a track in `Pending` and out of both obstacle topics until it matches on enough
    CONSECUTIVE frames. The required streak interpolates from `confirm_frames_near` (range 0)
    to `confirm_frames_far` (at `max_range`) on the last measured cluster range, and a single
    missed frame resets it, so intermittent flicker never confirms. At the
    confirming hit, publish the same track/ID immediately as `ProvisionalStatic`. Promote it to
    `ConfirmedStatic` after consecutive low-relative-speed observations, or to `Dynamic` only
    after consecutive velocity-confident motion observations with fresh, bounded ego yaw rate.
    Additionally, a static track enters the published layer only while its envelope-stability
    streak reaches `envelope_stability_frames`: consecutive matched frames whose measured centre
    and extents stay within `envelope_stability_tolerance_m`. An unmatched (prediction-only)
    frame resets that streak too. Fan-shaped morphing clusters never
    settle and stay unpublished; stable real obstacles pass at the same hit as before.
11. Merge confirmed tracks only within the same static/dynamic layer.
12. Preserve each measured cluster's independent Frenet footprint and map-frame Cartesian AABB
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
13. Publish all merged statics on `/static_obs`, the confirmed-static subset on
    `/confirmed_static_obs`, and at most one nearest-ahead dynamic object on `/opp_obs`. Annotate
    that opponent with `is_interfering=true` only when its lateral envelope overlaps the ego
    corridor and its current or constant-velocity-predicted rear gap is within the configured
    interference distance. Latch that same opponent until its gap exceeds the distance plus
    `interference_distance_margin_ratio`. The controller target stays at
    `interference_distance_m`; this Schmitt trigger prevents state chatter around that target.
14. Build the two RViz MarkerArrays from those final published arrays' Frenet bounds. Include
    predicted-only obstacles (possible in the dynamic layer; static ghosts are withheld while
    `static_publish_requires_visible` is true) and distinguish them with lower alpha.
15. Keep perception diagnostics passive. Accumulate beam, cluster, rejection, and tracker event
    counters over the configured wall-clock interval, but report live track/classification counts
    as a current snapshot. Do not claim noise filtering or deskew statistics in this package;
    those belong to the upstream scan preprocessor that actually performs them.

Pre-tracking `cluster_merge` and post-tracking `layer_merge` are complementary. The first creates
one detection/track from LiDAR fragments; the second consolidates same-layer output tracks. Do not
replace one with the other.

`FrenetProjector` remains responsible for boundary lookup and wrap-aware `s` differences.
`aabb_frenet_projector` is the sole owner of Cartesian obstacle AABB-to-Frenet conversion. It must
use the exported `global_planning::ClcsFrenetConverter`; downstream planners must consume the
published Frenet bounds instead of reprojecting the Cartesian metadata. Obstacle points use the
stateless `convert()` (arbitrary positions), but the ego odometry projection must use
`convertTracked()` — `ego_s_` anchors the viewing-window gate and opponent ahead-ranking, and a
stateless ego fix can branch-flip at the hairpin, anchoring every gate on the wrong leg.

## Layer semantics

- Layer 1 is the wall-structure (`/map`) and corridor filter. It is never published.
- Layer 2 is every provisional or confirmed non-map stationary object, published with
  `is_static=true`.
- `/confirmed_static_obs` is a same-scan, confirmed-only view of Layer 2. It uses the same
  envelope-stability gate, visibility gate, and object-level merge as `/static_obs`.
- Layer 3 is the nearest confirmed dynamic object ahead of the ego, published with
  `is_static=false`.
- A fresh track must not be published until its range-scaled consecutive-frame confirmation
  (`confirm_frames_near`/`confirm_frames_far`) sets its `classified` flag. The
  first publishable state is `ProvisionalStatic`; promotion to `ConfirmedStatic` preserves the
  track ID and must not interrupt `/static_obs`. With `static_publish_requires_visible`
  (default true), a static track is published only on scans where it was actually matched;
  prediction-only (ghost) tracks persist internally until their TTL expires but are withheld
  from `/static_obs` and `/confirmed_static_obs`.
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
- Layer 1 is a STRUCTURAL wall filter, not a per-cell occupancy vote. On each received map,
  occupied cells (`>= map_occupied_thresh`) are grouped into 8-connected components (the grid
  equivalent of DBSCAN), each component is tested for linearity with a 2x2 PCA
  (`wall_linear_ratio` eigenvalue ratio, `wall_min_length_m` major-axis extent), and a distance
  transform from all wall cells answers per-beam queries in O(1). A scan point within
  `wall_assoc_distance_m` of a wall component is dropped before clustering.
- This tolerates a map that diverges from the current scan (post-collision environment changes,
  localization offset): wall returns slightly off the mapped wall still match via the association
  distance, and compact non-linear blobs (e.g., obstacles or debris baked into the map) are NOT
  walls, so points near them remain obstacle candidates. The `detector_map_yaml` launch argument
  for an obstacle-free map on `/obstacle_detector/map` remains available but is no longer
  mandatory for that case; do not solve map divergence by disabling the map filter.
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
- `src/obstacle_tracker.cpp` — Frenet Kalman tracking and static/dynamic classification.
- `src/frenet_projector.cpp` — boundary lookup, track-length wrapping, and marker-only map
  interpolation.
- `src/wall_distance_filter.cpp` — structural Layer-1 wall extraction (grid-DBSCAN components,
  PCA linearity test) and the wall distance transform.
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
- `test/test_wall_distance_filter.cpp` — wall-component linearity, distance-transform, and
  association-distance classification tests.

## Verification

- Build with `colcon build --packages-up-to obstacle_detector`.
- Confirm that only `obstacle_detector_node` is installed by this package.
- Launch in an isolated `ROS_DOMAIN_ID` and verify clean startup and shutdown.
- Run `test/synthetic_opponent_test.py` against a fresh detector process.
- Confirm `/static_obs` contains provisional/confirmed stationary objects,
  `/confirmed_static_obs` contains only confirmed stationary objects, a moving provisional object
  moves to `/opp_obs` with the same ID, and confirmed stationary objects never leak into
  `/opp_obs`.
- Confirm every published object has finite Frenet bounds with `d_right <= d_left`; a closed-track
  wrap may make `s_start > s_end`. Visible merged objects must also have a matching current
  Cartesian AABB.
- Keep the Korean node document, README pair, configuration comments, and launch examples in sync.
