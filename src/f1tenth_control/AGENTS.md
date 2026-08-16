# AGENTS.md

Package-specific rules for `src/f1tenth_control`.

## Scope

- `control_map_node` owns final path tracking, speed ramping, steering LUT application, and
  `/drive_autonomous` publication.
- `cruise_controller_node` may only compute a longitudinal speed limit from the selected forward
  opponent. It must not generate or modify waypoints and must not publish Ackermann commands.
- `drive_source_selector` remains the only package component forwarding `/drive_autonomous` to
  `/drive`.

## Cruise Interfaces

- Subscribe to `/opp_obs` with `f110_msgs/msg/ObstacleArray`.
- Subscribe to `/state` with transient-local QoS. Apply an opponent speed cap only in
  `STATE_CRUISE`; GLOBAL and AVOID must receive the unrestricted maximum cap.
- Subscribe to `/car_state/frenet/odom` with `nav_msgs/msg/Odometry`.
- Subscribe to `/global_waypoints` with `f110_msgs/msg/WpntArray` and transient-local QoS.
- Publish `/cruise_speed_limit` with `std_msgs/msg/Float64`.
- Publish `/cruise/gap_data` with the existing `f110_msgs/msg/GapData` for diagnostics.
- A cruise speed limit is a cap only. Steering and path-source selection must remain unchanged.
- Keep the default fixed-distance `trailing_gap` aligned with the detector's
  `interference_distance_m`; the detector margin is only a state-retention band, not the control
  target.

## Parameters and Launch

- Cruise parameters live in `config/cruise_controller.yaml` and must be loaded by both control
  launch entrypoints.
- Keep real/simulation topic differences in launch files. Do not hard-code environment-specific
  odometry topics in the controller implementation.
- A stale active opponent or stale CRUISE heartbeat must fail safe to the configured blind speed.
  A cruise node outside STATE_CRUISE must not alter GLOBAL/AVOID driving behavior.

## Documentation and Verification

- Keep `docs/cruise_controller_node.md` synchronized with behavior and interfaces.
- Build with `cb --packages-select f1tenth_control` after sourcing ROS 2 Jazzy.
- Run unit tests and a ROS topic-level runtime check for clear, following, emergency-stop, and
  stale-input behavior.
