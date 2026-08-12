# AGENTS.md

Package rules for `src/f1tenth_control`.

## Scope and interfaces

- Runtime nodes are ROS 2 Jazzy C++17 nodes.
- `control_map_node` consumes `/local_waypoints`, localization odometry, and optional IMU/state
  inputs, then publishes `/drive_autonomous`.
- `drive_source_selector` is the simulation-side autonomous command forwarder from
  `/drive_autonomous` to `/drive`; it must not add planner or perception decisions.
- Keep command timestamps intact enough to trace the input command across the selector. Do not
  encode diagnostic identity by altering drive values.

## Parameters and launches

- Keep configurable topic names, feature toggles, and controller values in launch arguments or
  package configuration, with safe C++ defaults.
- `launch/control_sim.launch.py` and `launch/control_real.launch.py` share definitions through
  `launch/_control_common.py`; add common controller parameters there.
- Simulation-only selector parameters must also be passed by `control_sim.launch.py`.

## Tuning diagnostics

- `/cma_timing/events` is a default-off companion diagnostic interface. It must never become a
  controller input or change control output values.
- T5 is the first control cycle that consumes an avoidance `/local_waypoints` message published
  after STATE_AVOID. T6 is that cycle's `/drive_autonomous` publication. T7 is the matching
  selector `/drive` publication.
- Capture steady-clock timestamps at the event being measured and publish the drive command before
  its companion diagnostic event so instrumentation does not delay the command path.
- Keep production defaults with `timing_diagnostics_enable=false`.

## Deterministic lockstep

- `lockstep_mode` is CMA-only and must remain false by default. It disables the 20 ms wall timer and
  runs one existing controller cycle only when `/local_waypoints` and ego odometry have the same
  timestamp.
- Use the fixed `lockstep_period_sec` for controller integration and the odometry snapshot's yaw
  rate in place of asynchronously paired IMU data. Do not change the production IMU path.
- Preserve the computed `/drive_autonomous` values exactly; the lockstep coordinator applies that
  command to the next backend physics step.

## Documentation and verification

- Keep `docs/cma_timing_diagnostics.md` current when diagnostic events, parameters, or launch
  wiring change.
- Build both `control_map_node` and `drive_source_selector` through the package build and verify
  the integrated simulation command path after changes.
