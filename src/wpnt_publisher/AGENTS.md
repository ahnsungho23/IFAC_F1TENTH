# AGENTS.md

Package-specific rules for `src/wpnt_publisher`.

- Runtime code is C++17 on ROS 2 Jazzy.
- `wpnt_publisher` is the single selector that publishes `/local_waypoints` and its Path mirror.
- Preserve the dynamic-opponent contract: `STATE_OVERTAKE` selects `/overtake_waypoints` without
  changing the opponent planner's semantics.
- `STATE_AVOID` selects only a fresh `/avoid_waypoints` path.
- `STATE_AVOID` uses only a fresh planner-provided avoidance path. If no such path is available,
  the publisher falls back to the normal global waypoint selection; this package does not create
  or publish a safety-stop path.
- Runtime values and topic names live in `config/wpnt_publisher.yaml` and are loaded by
  `launch/wpnt_publisher.launch.py`.
- Keep Korean operational documentation in `docs/wpnt_publisher.md` current.
