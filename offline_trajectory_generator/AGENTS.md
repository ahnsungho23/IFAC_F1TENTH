# AGENTS.md

This directory contains a standalone, offline trajectory generation tool.

Rules:
- Do not add ROS 2 runtime dependencies here.
- Keep the CLI runnable with plain `python3` after the workspace Python dependencies are available.
- Inputs are ROS map-server compatible SLAM map YAML/image files.
- Outputs should stay compatible with the existing `f110_msgs/Wpnt` field names when practical.
- Put user-facing operation docs in Korean.
- Prefer configurable CLI arguments over hard-coded map, speed, width, or smoothing values.
- When adding or changing generator parameters, update the GUI defaults, saved parameter YAML, and Korean README in the same change.
