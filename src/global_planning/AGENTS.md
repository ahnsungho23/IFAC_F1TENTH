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
- `global_trajectory_publisher_node` republishes `visualization_msgs/msg/MarkerArray` read from JSON; it must not build markers itself.

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
- There are TWO ways the active source changes, and both go through `swapTo()` so they
  share one read + `validBundle` + atomic-replace path. Keep it that way; do not let a
  new trigger bypass the validation.
  1. `/global_planning/reload_waypoints` (map_creator, ~lap 3) -> `reload_map_name`.
  2. `/lap_count >= lap_switch_count` (default 11) -> `lap_switch_map_name`, a bundle
     that must already exist on disk (the offline Forza raceline).
- Lap-switch invariants: `lap_switch_map_name` empty DISABLES the feature and is the code
  default, so an unconfigured node behaves exactly as before; the switch is one-shot
  (`/lap_count` is latched, so without the flag every redelivery would re-read the files);
  the comparison is `>=` so a dropped message cannot skip it; a failed attempt must NOT set
  the flag, so the next lap retries while the live line stays untouched.
- The lap-count subscription QoS must stay `KeepLast(1)` + reliable + **transient_local** to
  match `lap_counter_node`'s publisher. A volatile subscription silently misses the latched
  value and defers the switch by a whole lap.
- Nothing undoes the lap switch: map_creator's post-swap stage is terminal (see
  `src/map_creator/AGENTS.md`). If a re-swap path is ever reintroduced there, revisit the
  ordering here first.
- Switching the live line also moves the Frenet frame that `lap_counter_node` itself counts
  on (`frenet_odom_node` rebuilds the CLCS from `/global_waypoints`). A target line whose
  `s_max` is below `finish_s_min` stops lap counting outright — check that relationship
  whenever the target bundle or map changes.
- This node performs no geometry. RViz markers are baked into `global_waypoints.json` by the
  offline generator (`generate_global_trajectory`, `Args::emit_markers`); the node only republishes
  them. Do not reintroduce marker-building code or marker style parameters here — the generator
  constants are the single source of truth.
- The in-race regeneration path (`regenerate_obstacle_map`, map_creator) deliberately writes empty
  marker arrays. When a marker array is empty the node must publish one `action=DELETEALL` marker
  instead: RViz `MarkerArray` entries are persistent per ns+id, so publishing an empty array would
  leave the previous map's raceline on screen. Markers vanishing after a reload is intended.

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
