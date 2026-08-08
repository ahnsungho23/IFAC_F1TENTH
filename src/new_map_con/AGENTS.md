# AGENTS.md

This package owns the ROS 2 `new_map_con` controller.

Rules:
- Runtime nodes in this package must be C++/rclcpp.
- Keep parameter defaults declared in the node and operational values in `config/config.yaml`.
- Add or update launch files under `launch/` whenever node parameters are required.
- Launch files should prefer source-tree config/resource paths when the workspace source package is available, so config and map CSV edits apply without rebuilding after `--symlink-install`.
- Install `config/`, `launch/`, `maps/`, and `docs/` from `CMakeLists.txt`.
- Prefer `f110_msgs/WpntArray` for waypoint topics.
- Keep standalone/offline trajectory generation outside `/src`; this package should consume its CSV output through parameters.
- Keep the packaged default waypoint CSV aligned with the simulator map used by `f1sim`; if `global_waypoints_csv` changes to a different map, update docs and any simulator run instructions in the same change.
- User-facing package documentation must be written in Korean under `docs/`.
