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

## Removed: Reference Path Adapter (2026-08-09)

- The IV'24 Alg.1 port (`reference_path_adapter.{hpp,cpp}`, curvature-singularity
  smoothing/reduction, 8 parameters incl. `enable_path_smoothing` /
  `enable_curvature_reduction` / `reference_resample_step`) was removed entirely.
  On the verified waypoint snapshot it was a no-op (`already_satisfied`,
  rho 0.92 < 1), and running it inside `frenet_odom_node` alone would desync the
  ego Frenet frame from `obstacle_detector`, which builds its own CLCS from the
  RAW `/global_waypoints`.
- To reintroduce adaptation, apply it once at the publisher
  (`global_trajectory_publisher_node`) so every consumer receives the same
  adapted waypoints, and recompute `d_left`/`d_right` for moved points — see
  commit `09073ff` and `docs/proposal_remove_reference_path_adapter.md`.
  Note the port deviated from the paper (closed-loop anchors, waypoint bounds),
  so the paper's formal guarantees do not transfer as-is. Code is recoverable
  from git history (`c060ad8`).

## Global Trajectory Publisher Node

- Reads `global_waypoints.json` and republishes waypoints on latched topics.
- The offline generator writes empty marker arrays, so this node builds `/global_waypoints/markers`
  (speed-colored racing line) and `/trackbounds/markers` (left/right bounds from `d_left`/`d_right`
  and `psi_rad`) from the waypoints themselves in `generateMarkers()`.
- Generated markers only fill arrays the JSON left empty; keep marker frame and line widths in YAML.

## Documentation

- Update `docs/<node_name>.md` when node behavior, parameters, topics, or run commands change.
- Keep step-by-step operator documentation in Korean.
- Update package-level pipeline docs when a node's public behavior changes.
