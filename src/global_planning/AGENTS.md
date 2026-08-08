# AGENTS.md

This file applies to the `global_planning` package (directory `src/global_planning`).
The package name, C++ namespace (`namespace global_planning`), include prefix
(`include/global_planning/`), and node/executable names are all unified under `global_planning`.

## Package Scope

- Runtime ROS 2 nodes in this package must be C++.
- Keep node sources in `src/` and public declarations or shared helpers in `include/`.
- Keep package parameters in `config/global_planning.yaml` unless a node clearly needs a dedicated YAML file.
- Keep launch entry points in `launch/`; launch files must load the matching YAML parameters.
- Keep user-facing node documentation in `docs/`.

## Messages and Dependencies

- Prefer existing `f110_msgs` messages and ROS 2 standard messages.
- `frenet_odom_node` uses `nav_msgs/msg/Odometry`, `f110_msgs/msg/WpntArray`, and CommonRoad-CLCS C++ core.
- Do not add `tf2` to `frenet_odom_node`; yaw and heading-error handling must use local math helpers.
- `global_trajectory_publisher_node` builds RViz markers with `visualization_msgs/msg/MarkerArray`.

## Frenet Odom Node

- Convert map-frame vehicle XY to Frenet `s`, `d` through `geometry::CurvilinearCoordinateSystem`.
- Do not copy waypoint `d_m` into vehicle `d`; waypoint `d_m` may describe another lateral offset.
- Keep topic names, frame names, and loop mode configurable through YAML.
- For closed-loop tracks, close the reference path before building CLCS and wrap published `s` by CLCS path length.
- Skip zero-length or invalid segments and avoid publishing if fewer than two waypoints are available.
- The previous polyline-based implementation was removed; consult git history (commit `301a06e` and earlier) if the legacy `frenet_odom_node_legacy_polyline.cpp` reference is ever needed.
- `frenet_odom_node` publishes via `ClcsFrenetConverter::convertTracked()`
  (monotonic s-window): after the first fix only
  `[s_prev - backward_tolerance, s_prev + forward_window]` (mod track length for
  closed loops) is searched, which is what prevents hairpin opposite-leg flips
  (branch-proximity non-uniqueness). A window miss FAILS CLOSED — never add a
  silent global fallback; only after `reacquire_after_misses` consecutive misses
  does one loud global re-search run (`result.reacquired`, WARN in the node).
  Reset `ClcsContinuityState` whenever the converter is rebuilt.
- Keep `convert()` stateless: `obstacle_detector` projects arbitrary points with
  it, and tracking would corrupt those projections.
- The per-segment projection used by the windowed search is reimplemented in
  `clcs_frenet_converter.cpp` on public `geometry::Segment` getters because the
  vendored 3-arg `Segment::convertToCurvilinearCoords` overload is private; do
  not patch the vendored library.

## Reference Path Adapter (`reference_path_adapter.{hpp,cpp}`)

- C++ port of Alg. 1 in Würsching & Althoff, IEEE IV 2024 ("Robust and Efficient
  Curvilinear Coordinate Transformation with Guaranteed Map Coverage"), for closed
  loops. Guarantees, on success (`criterion_met`), that every corridor point has a
  unique curvilinear projection of the curvature-singularity type:
  `rho = |kappa| * (inner-bound distance + boundary_margin) < 1` at every point.
- Closed-loop deviations from the paper (documented in the header): bounds come
  from per-waypoint `d_left`/`d_right` instead of lanelets; anchors (middles of
  straight runs, `anchor_curvature_threshold`) replace partition boundaries and
  every bend is subdivided/resampled as an open segment with pinned ends — without
  pinned ends convex bends contract instead of flattening (verified by test);
  the per-partition constant cap kappa_Gm is replaced by the pointwise rho check.
- The adapter runs inside `frenet_odom_node` only, is disabled by default, and
  MUST stay disabled unless `obstacle_detector` (which builds its own CLCS from
  the raw `/global_waypoints`) applies the same preprocessing — otherwise ego and
  obstacle Frenet frames diverge. The node logs a WARN_ONCE when it modifies the
  reference.
- Known behavior on real data: the IQP raceline already satisfies rho < 1
  (`already_satisfied`, no-op). The ifac_track centerline hairpin is
  geometrically infeasible for the criterion (required osculating radius exceeds
  the corridor); the loop then reports `boundary_hit`/`max_iterations` honestly
  instead of crossing walls. `resample_step` controls the convergence rate
  (paper Sec. IV); larger steps on narrow tracks trigger the Fig. 6 boundary
  guard earlier.

## Global Trajectory Publisher Node

- Reads `global_waypoints.json` and republishes waypoints on latched topics.
- The offline generator writes empty marker arrays, so this node builds `/global_waypoints/markers`
  (speed-colored racing line) and `/trackbounds/markers` (left/right bounds from `d_left`/`d_right`
  and `psi_rad`) from the waypoints themselves in `generateMarkers()`.
- Generated markers only fill arrays the JSON left empty; keep marker frame and line widths in YAML.
- Load the shared visualization profile after `global_planning.yaml`; it may override only
  `publish_markers` and `publish_lattice`, never waypoint or Frenet data outputs.
- With `publish_markers=false`, do not construct generated MarkerArray data.

## Documentation

- Update `docs/<node_name>.md` when node behavior, parameters, topics, or run commands change.
- Keep step-by-step operator documentation in Korean.
- Update package-level pipeline docs when a node's public behavior changes.
