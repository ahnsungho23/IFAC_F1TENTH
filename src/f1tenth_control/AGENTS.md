# AGENTS.md for f1tenth_control

## Package scope

- Runtime nodes are C++17 ROS 2 Jazzy nodes.
- `control_map_node` consumes global/local waypoints, odometry, and IMU data and publishes
  `/drive_autonomous`.
- `drive_source_selector` only forwards `/drive_autonomous` to `/drive`; safety multiplexing for
  the real vehicle remains owned by the external `f1tenth_stack`.
- `sim_imu_bridge_node` is simulation-only. Do not launch it on the real vehicle.
- `odom_calib_node` is an optional observation tool and must not publish drive commands.

## Visualization and parameters

- Keep driving parameters in launch arguments or matching package configuration.
- `publish_l1_markers` controls the RViz-only `/debug/l1_lookahead` output. When false, do not
  create the publisher or construct MarkerArray messages.
- The shared profile is `config/runtime_visualization.yaml`. It may override only visualization,
  debug-output, and redundant map-publication switches; do not move algorithm tuning into it.
- Both `control_sim.launch.py` and `control_real.launch.py` must load the shared profile after
  their normal parameters so the profile wins only for listed switches.

## Layout and documentation

- Runtime sources: `control_code/`.
- Launch files: `launch/`.
- Shared visualization profile: `config/runtime_visualization.yaml`.
- Korean operating guide: `docs/runtime_visualization.md`.
- Update the guide and this file when public topics, parameters, or launch commands change.
