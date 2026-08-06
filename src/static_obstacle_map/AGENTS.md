# AGENTS.md — static_obstacle_map

Package-level rules for persistent confirmed-static obstacle memory. These rules inherit and must
not weaken the repository-root `AGENTS.md`.

## Scope

- `static_obstacle_map_node` is a ROS 2 Jazzy C++ node.
- It stores confirmed static objects from `f110_msgs/msg/ObstacleArray` in map-frame Cartesian
  coordinates and publishes an RViz `visualization_msgs/msg/MarkerArray` view.
- It must not subscribe to, modify, compose, or publish `nav_msgs/msg/OccupancyGrid` maps.
- Detection, motion classification, planning, and vehicle control are outside this package.

## Memory and geometry

- Accept only confirmed-topic objects with `is_static=true`, `is_visible=true`, and a valid current
  Cartesian AABB.
- Missing or empty confirmed messages must not erase stored obstacles.
- Preserve the earliest confirmed footprint and grow it with a bounded AABB union across associated
  observations. Never shrink an established edge with a later partial scan.
- Accept an outward edge only after `edge_confirm_frames` observations agree within
  `edge_match_tolerance_m`.
- Clamp oversized first observations to `max_obstacle_diagonal_m` and reject later unions that
  exceed the limit without moving the stored footprint.
- A same-track dynamic reclassification may retract a stored object when
  `remove_reclassified_dynamic` is enabled. The reset service clears all memory.

## Interfaces and parameters

- Inputs: confirmed and dynamic `f110_msgs/msg/ObstacleArray` topics.
- Output: a Reliable + Transient Local `visualization_msgs/msg/MarkerArray`.
- Use one map-frame `CUBE` per stored obstacle and prepend `DELETEALL` to prevent stale markers.
- Keep topic names, frame, association settings, size limit, marker style, and reset behavior in
  `config/static_obstacle_map.yaml`, with matching safe defaults in C++.

## Layout and documentation

- Public memory logic belongs in `include/static_obstacle_map/` and is unit tested independently of
  ROS graph timing.
- Node implementation belongs in `src/static_obstacle_map_node.cpp`.
- Keep `launch/static_obstacle_map.launch.py` loading the package YAML.
- Keep step-by-step Korean documentation in `docs/static_obstacle_map_node.md`.

## Verification

- Build with `colcon build --packages-up-to static_obstacle_map obstacle_detector`.
- Test insertion, edge confirmation, outlier rejection, maximum-size enforcement, persistence
  across empty messages, dynamic retraction, marker output, and reset behavior.
- Launch the node and verify that late subscribers receive `/static_obstacle_map/markers` and that
  no OccupancyGrid output topic exists.
