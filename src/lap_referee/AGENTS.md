# AGENTS.md — lap_referee

Node-level rules for the `lap_referee` package. Inherits and must not weaken the
root `CLAUDE.md` / `AGENTS.md`.

## Purpose

A C++ ROS 2 node that judges and records one closed-loop rollout in
`f1tenth_gym_ros`. It is the measurement layer for the top-level
`dl_speed_optimizer` DL tool: one process == one rollout == one
`*_summary.json` + `*_trace.csv`.

## Rules

- Runtime node is C++ (ROS 2 Jazzy). Do not add a Python runtime node here.
- Use ROS standard messages: `nav_msgs/Odometry`, `sensor_msgs/LaserScan`,
  `ackermann_msgs/AckermannDriveStamped`. Do not introduce a new message type
  for rollout results; results are files (`summary.json` + `trace.csv`) so the
  non-ROS Python optimizer can read them without `rclpy`.
- Collision state is NOT available on a ROS topic from the gym bridge. Detect
  termination only from observable signals (scan min range, odom speed/pose,
  commanded speed, lateral error vs. reference). Keep all thresholds in
  `config/lap_referee.yaml`; do not hard-code them in the node.
- The output JSON schema is a contract with `dl_speed_optimizer`
  (`dl_speed_opt/evaluators.py`, `SimRunner._parse_outputs`). When you add/rename
  a summary field, update that parser, this package's docs, and the optimizer
  README in the same change.
- Keep the node single-rollout and self-terminating (`shutdown_on_terminate`),
  so the orchestrator can detect completion via process exit.

## Layout

- `src/lap_referee_node.cpp` — the node.
- `config/lap_referee.yaml` — all tunables.
- `launch/lap_referee.launch.py` — manual launch (requires `waypoints_csv:=`).
- `docs/lap_referee_node.md` — Korean operation doc.

## Checklist before finishing a change

- Builds with `colcon build --packages-select lap_referee`.
- Thresholds live in YAML, declared with safe defaults in the node.
- `docs/lap_referee_node.md` and the summary-field table stay in sync with code.
- If the JSON contract changed, `dl_speed_optimizer` parser + README updated too.
