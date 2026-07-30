# lap_timer Agent Guide

These instructions extend the repository-level `AGENTS.md`.

## Package Rules

- Target Ubuntu 24.04 and ROS 2 Jazzy with Python 3.12.
- `lap_timer/lap_timer_node.py` is a legacy Python ROS 2 node. Keep changes focused; new runtime
  nodes in this repository must follow the root C++ policy.
- Use ROS 2 standard messages for odometry, drive status, timing, and visualization.
- Keep operating values in `config/params.yaml`; `launch/lap_timer.launch.py` must load that file.

## Layout and Documentation

- Runtime module: `lap_timer/lap_timer_node.py`
- Parameters: `config/params.yaml`
- Launch: `launch/lap_timer.launch.py`
- RViz: `rviz/lap_hud.rviz`
- Documentation: `docs/lap_timer_node.md`

Install every runtime resource through `setup.py`. Update the node document when topics, parameters,
lap detection, or HUD behavior changes.

## Verification

- Build with the repository Jazzy `cb` alias.
- Run the package pytest suite and validate launch parsing.
- A live timing check requires `/car_state/frenet/odom`; do not claim lap detection without that
  input.
