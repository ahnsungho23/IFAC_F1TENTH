# AGENTS.md for lap_timer

- This ROS 2 Python node is an optional timing/HUD tool, not part of vehicle control.
- Keep `nav_msgs/msg/Odometry` and standard scalar messages for its public interfaces.
- Keep runtime parameters in `config/params.yaml` and load them from `launch/lap_timer.launch.py`.
- `rviz_text=false` must suppress the 3D text marker while preserving lap-time calculations.
- `publish_hud=false` must suppress only the overlay and HUD timer; keep numeric speed/steer topics.
- RViz must be optional through the `use_rviz` launch argument and default off for embedded use.
- Load `f1tenth_control/config/runtime_visualization.yaml` after node parameters.
- Keep Korean documentation in `docs/lap_timer_node.md` current.
