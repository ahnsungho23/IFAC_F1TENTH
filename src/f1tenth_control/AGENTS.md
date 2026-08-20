# AGENTS.md
f1tenth_control package rules. These instructions apply to `src/f1tenth_control`.

## Normative Documents

- `CLAUDE.md` in this package is the normative controller document (algorithm,
  tuning history, per-parameter rationale with bag evidence). Read the relevant
  section BEFORE changing control behaviour, and update it in the same change.
- `LAUNCH_FULLSTACK.md` covers full-stack bring-up and the trajectory-generator
  argument cross-check table. `README.md` is the short operator guide.
- Keep Korean prose in these documents (they are read on the car during runs).

## Package Layout

- Nodes live in `control_code/` (not `src/`): `control_map_node.cpp` (main
  controller), `drive_source_selector.cpp`, `odom_calib_node.cpp`,
  `sim_imu_bridge_node.cpp`. Headers in `include/f1tenth_control/`.
- `scripts/sector_learner.py` is the only Python runtime node (sector scale table
  publisher). New runtime logic goes to C++.
- `f110_msgs/` here is a nested copy of the message package for standalone Jetson
  builds. Do not diverge it from the workspace-level `f110_msgs`.

## Parameters and Launch

- `control_map_node` has NO YAML parameter file by design. Values that change per
  environment live as `DeclareLaunchArgument` defaults in
  `launch/control_real.launch.py` / `launch/control_sim.launch.py`; everything else
  lives in `launch/_control_common.py`. The launch defaults, not the
  `declare_parameter` defaults, are what actually runs — keep the CLAUDE.md
  parameter table in sync with the launch files.
- `max_speed`, `max_lateral_accel`, `base_max_accel` are declared per-launch-file
  (real vs sim differ). Do not move them into `_control_common.py`.
- `config/sectors.yaml` is the only YAML, read by `sector_learner.py`.

## Runtime Parameter Contract (max_speed only)

- `control_map_node` accepts runtime `set_parameters` for `max_speed` ONLY, via a
  single `add_on_set_parameters_callback` registered in the constructor. Accept
  positive finite doubles; reject anything else with a `result.reason` string —
  `map_creator` logs that reason on failure.
- The producer is `map_creator_node`, which calls `/control_map_node/set_parameters`
  right after a successful obstacle-line swap (`swap_max_speed_mps`) or a baseline
  rollback (`rollback_max_speed_mps` when > 0). Renaming or namespacing this node in
  the launch files silently breaks that link — `map_creator.yaml`'s
  `control_node_name` must be updated together.
- The launch default of `max_speed` is the PRE-swap (baseline line) cap and must stay
  at or below `swap_max_speed_mps`, so the swap raises the cap rather than lowering it.
- Every other parameter is read once in the constructor. Do not add per-parameter
  runtime handling without a stated need — `CLAUDE.md` ②-u documents why.
- The node runs on `rclcpp::spin()` (single-threaded executor), so the parameter
  callback and the 20 ms control timer share a thread and need no locking. If that
  ever changes, add synchronisation for `max_speed_`.

## Change Checklist

- Build with `colcon build --packages-select f1tenth_control` and, for parameter or
  gate changes, verify at runtime (`ros2 param set /control_map_node max_speed ...`).
- Install rules in `CMakeLists.txt` must cover new executables, launch files, config
  files, and scripts.
- Record behaviour changes with their evidence (bag name, measured numbers) in
  `CLAUDE.md`; a tuning value changed without evidence in the document is treated as
  stale on the next run.
