# AGENTS.md

This package owns the simulator-only opponent vehicle controller.

## Scope

- Keep runtime code in C++/rclcpp.
- Follow `/centerline_waypoints` with the simulated opponent pose from `/opp_racecar/odom`.
- Publish only the opponent command `/opp_drive`; never publish the ego `/drive` or
  `/drive_autonomous` topics.
- Keep this package independent from the ego state machine and cruise controller. It creates the
  moving test target; it does not detect opponents or control ego following distance.

## Parameters and Launch

- Operational parameters live in `config/opponent_simulator.yaml`.
- `launch/opponent_simulator.launch.py` must load that YAML and expose the waypoint topic and speed
  scale used by `sim/launch_cruise.zsh`.
- Topic names, controller geometry, rates, limits, timeouts, and speed scaling must remain
  configurable.

## Documentation and Verification

- Keep `docs/opponent_drive_controller.md` synchronized with topics, parameters, and launch usage.
- Build with ROS 2 Jazzy and verify that one node is the sole publisher of `/opp_drive`.
- Test with a small waypoint array and opponent odometry before running the full two-car simulator.
