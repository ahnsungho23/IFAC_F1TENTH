# AGENTS.md — static_obstacle_map

Package-level rules for the persistent confirmed-static obstacle map. These rules inherit and must
not weaken the repository-root `AGENTS.md`.

## Scope

- `static_obstacle_map_node` is a ROS 2 Jazzy C++ node.
- It combines a clean Layer-1 `nav_msgs/msg/OccupancyGrid` with confirmed static objects from
  `f110_msgs/msg/ObstacleArray`.
- It publishes a separate augmented map for adaptive global planning. Never publish the augmented
  map back onto the detector's base-map topic.
- Detection, motion classification, local avoidance, global path generation, and vehicle control
  are outside this package.

## Memory and geometry

- Store obstacle geometry in map-frame Cartesian coordinates, not Frenet coordinates, so global
  waypoint/CLCS updates do not move or clear the persistent map.
- Accept only confirmed-topic objects with `is_static=true`, `is_visible=true`, and a valid current
  Cartesian AABB.
- Missing or empty confirmed messages must not erase stored obstacles.
- Preserve the earliest confirmed footprint and grow it with a bounded AABB union across associated
  observations. Never shrink or replace an established edge with a later partial rear/side scan.
- Accept a new outward edge only after `edge_confirm_frames` consecutive observations agree within
  `edge_match_tolerance_m`; a one-scan outlier must not become persistent geometry.
- Clamp an oversized first observation to `max_obstacle_diagonal_m`. For an existing obstacle,
  reject a confirmed union expansion that would exceed this limit without moving or shrinking the
  previously stored footprint.
- Recompose every output from the immutable latest base map plus the current memory. Do not paint
  updates incrementally onto the previous output because that leaves stale occupied cells.
- A same-track dynamic reclassification may retract a stored object when
  `remove_reclassified_dynamic` is enabled.
- Clear memory on the reset service, and optionally when the base-map geometry changes.

## Interfaces and parameters

- Inputs: `nav_msgs/msg/OccupancyGrid` base map and `f110_msgs/msg/ObstacleArray` confirmed/dynamic
  objects.
- Outputs: the augmented `nav_msgs/msg/OccupancyGrid` and its persistent-obstacle
  `visualization_msgs/msg/MarkerArray` RViz mirror, both with Reliable + Transient Local QoS.
- Keep topic names, association/update settings, maximum size, inflation, occupied value, and reset
  behavior configurable in `config/static_obstacle_map.yaml`.
- The RViz mirror must use one map-frame `CUBE` per stored obstacle and prepend `DELETEALL` so
  reset, reclassification, and association updates never leave stale markers. Do not duplicate
  every base-map occupied cell as a marker; RViz can render the OccupancyGrid directly.
- Update memory on every confirmed callback, but bound full OccupancyGrid recomposition with
  `publish_period_ms`. New insertions, base-map updates, dynamic retractions, and reset may publish
  immediately.
- Keep the matching safe defaults declared in C++.

## Layout and documentation

- Public memory/composition logic belongs in `include/static_obstacle_map/` and is unit tested
  independently of ROS graph timing.
- Node implementation belongs in `src/static_obstacle_map_node.cpp`.
- Keep `launch/static_obstacle_map.launch.py` loading the package YAML.
- Keep step-by-step Korean documentation in `docs/static_obstacle_map_node.md`.

## Verification

- Build with `colcon build --packages-up-to static_obstacle_map obstacle_detector`.
- Run package tests and verify insertion, consecutive-edge confirmation, front-footprint
  preservation, one-scan outlier rejection, maximum-size enforcement without stored-footprint
  movement, persistence across empty messages, dynamic retraction, reset, and base-map
  recomposition.
- Launch the node and verify that late subscribers receive `/adaptive_obstacle_map` and
  `/adaptive_obstacle_map/markers`.
