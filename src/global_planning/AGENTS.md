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
- Publish ego odometry with `ClcsFrenetConverter::convertTracked()`. After the
  first fix, search only the configured monotonic s-window. A window miss must
  fail closed; perform a global re-search only after the configured consecutive
  miss threshold and emit a warning when re-acquisition occurs.
- Reset `ClcsContinuityState` whenever the converter or reference path is rebuilt.
- Keep `ClcsFrenetConverter::convert()` stateless because `obstacle_detector`
  uses it to project unrelated points. Do not patch the vendored CommonRoad-CLCS
  library to implement tracked projection.
- The removed reference-path adapter parameters (`reference_resample_step`,
  `enable_path_smoothing`, and `enable_curvature_reduction`) must not be restored
  to YAML unless adaptation is implemented consistently for every Frenet consumer.

## Global Trajectory Publisher Node

- Reads `global_waypoints.json` and republishes waypoints on latched topics.
- Keep the baseline `map_name` source immutable. The reload service must validate
  `<output_base_dir>/<reload_map_name>/global_waypoints.json` before atomically switching
  the in-memory bundle and active source path; never overwrite the baseline JSON.
- The offline generator writes empty marker arrays, so this node builds `/global_waypoints/markers`
  (speed-colored racing line) and `/trackbounds/markers` (left/right bounds from `d_left`/`d_right`
  and `psi_rad`) from the waypoints themselves in `generateMarkers()`.
- Generated markers only fill arrays the JSON left empty; keep marker frame and line widths in YAML.

## Lap Counter Node

- Read Frenet `s` only from `/car_state/frenet/odom` (`nav_msgs/msg/Odometry`,
  `pose.pose.position.x`).
- Count a lap only on a configured finish-region to start-region wrap. Keep the
  finish/start thresholds and minimum lap interval in `config/global_planning.yaml`.
- Publish the current count as `std_msgs/msg/Int32` on `/lap_count` with
  reliable, transient-local QoS. Do not overload unrelated `Odometry` fields.
- Keep the wrap detector independent from ROS in
  `include/global_planning/frenet_lap_counter.hpp` and cover boundary behavior
  in `test/test_frenet_lap_counter.cpp`.
- Document operator setup and threshold tuning in `docs/lap_counter_node.md`.
- Launch composition: `lap_counter_node` is NOT started by `global_planning.launch.py`.
  It is included by `state_machine.launch.py` (via `lap_counter.launch.py`), which still
  loads its parameters from `config/global_planning.yaml`. Keep `lap_counter.launch.py`
  working standalone.

## Documentation

- Update `docs/<node_name>.md` when node behavior, parameters, topics, or run commands change.
- Keep step-by-step operator documentation in Korean.
- Update package-level pipeline docs when a node's public behavior changes.
- `global_planning.launch.py` includes `map_creator.launch.py`; keep their parameter files
  separate through the `map_creator_params_file` launch argument.
