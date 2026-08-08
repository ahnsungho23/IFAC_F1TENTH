# AGENTS.md — static_obstacle_map

Package-level rules for persistent confirmed-static obstacle memory. These rules inherit and must
not weaken the repository-root `AGENTS.md`.

## Scope

- `static_obstacle_map_node` is a ROS 2 Jazzy C++ node.
- It stores confirmed static objects received as `f110_msgs/msg/ObstacleArray`.
- It publishes the complete persistent snapshot as `f110_msgs/msg/ObstacleArray`; each element is
  an `f110_msgs/msg/Obstacle` with valid map-frame Cartesian geometry.
- It must not subscribe to, compose, or publish `nav_msgs/msg/OccupancyGrid`.
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
- Use stable memory IDs in the output `ObstacleArray` and publish the entire stored snapshot after
  insertion, geometry expansion, reset, or an enabled dynamic retraction.
- Keep `remove_reclassified_dynamic` disabled by default so short static/dynamic classifier
  oscillations cannot retract confirmed persistent geometry. A same-track dynamic reclassification
  may retract a stored object only when this option is explicitly enabled for a trusted signal.
- Clear memory only on the reset service or an explicitly enabled dynamic retraction.

## Interfaces and parameters

- Inputs: confirmed and optional dynamic `f110_msgs/msg/ObstacleArray` objects.
- Outputs: the persistent `f110_msgs/msg/ObstacleArray` and its
  `visualization_msgs/msg/MarkerArray` RViz mirror, both with Reliable + Transient Local QoS.
- Keep topic names, output frame, association/update settings, maximum size, marker style, and
  reset behavior configurable in `config/static_obstacle_map.yaml`.
- The RViz mirror must use one map-frame `CUBE` per stored obstacle and prepend `DELETEALL` so
  reset, reclassification, and association updates never leave stale markers.
- Publish an initial empty snapshot so late subscribers can distinguish an empty memory from a node
  that has not started.
- Keep the matching safe defaults declared in C++.

## Layout and documentation

- Public memory/message-building logic belongs in `include/static_obstacle_map/` and is unit tested
  independently of ROS graph timing.
- Node implementation belongs in `src/static_obstacle_map_node.cpp`.
- Keep `launch/static_obstacle_map.launch.py` loading the package YAML.
- Load the shared `f1tenth_control/config/runtime_visualization.yaml` after the package YAML so
  `publish_visualization=false` skips only the RViz mirror.
- Keep step-by-step Korean documentation in `docs/static_obstacle_map_node.md`.

## Verification

- Build with `colcon build --packages-up-to static_obstacle_map obstacle_detector`.
- Run package tests and verify insertion, consecutive-edge confirmation, front-footprint
  preservation, one-scan outlier rejection, maximum-size enforcement without stored-footprint
  movement, persistence across empty messages, `ObstacleArray` output, dynamic retraction, and
  reset.
- Launch the node and verify that late subscribers receive `/adaptive_obstacle_map` and
  `/adaptive_obstacle_map/markers`, and that no OccupancyGrid interface exists.
