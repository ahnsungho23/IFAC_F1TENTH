# AGENTS.md

This package is a simulation/tuning-only localization adapter.

- Keep runtime code in C++ and keep all topics, frames, QoS depth, and validation toggles configurable in `config/gt_localization_bridge.yaml`.
- The only allowed runtime input is simulator ego odometry. Do not add scenario manifests, baked-map geometry, obstacle ground truth, or obstacle topics as inputs.
- The node must not publish TF. In f1tenth_gym simulation, `gym_bridge` owns `map -> ego_racecar/base_link -> ego_racecar/laser`; publishing either edge here would create duplicate TF authorities.
- Preserve the source header timestamp and pose exactly. The default `mcl_compatible` twist mode may copy only `linear.x`, matching `/pf/pose/odom` from `particle_filter_cpp`.
- Keep `launch/gt_localization_bridge.launch.py`, the YAML configuration, tests, and `docs/gt_localization_bridge.md` synchronized with code changes.
- This package must never be included in the normal MCL/real-vehicle launch path.

