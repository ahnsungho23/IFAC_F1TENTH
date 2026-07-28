# AGENTS.md for local_planning

## Package purpose

- This package handles static-obstacle avoidance only.
- Its geometric invariant is race-line locking: every published moving path must preserve the
  ordered `/global_waypoints` samples and modify only their local Frenet `d(s)` offset.
- Never add Cartesian nearest-path search, free-space graph search, or a map shortcut that can jump
  to a different geometric track branch. This invariant is especially important on non-convex
  snake sections.
- Dynamic-opponent planning is outside this package. `obstacle_detector` publishes dynamic
  perception separately on `/opp_obs`.

## Algorithm rules

- Runtime code is C++17 for ROS 2 Humble.
- Use `f110_msgs/msg/ObstacleArray`, `WpntArray`, and `OTWpntArray`; do not create a new message.
- Consume map-frame Cartesian obstacle centers and enclosing-circle radii from
  `obstacle_detector` Layer 2 on `/static_obs`. Project each center through the shared CLCS converter,
  then use the same radius as its longitudinal and lateral Frenet extent before selecting the
  nearest blocking obstacle cluster and evaluating both sides.
- Derive each target `d` from the obstacle lateral bound plus configured clearance. Reject targets
  outside the per-waypoint `d_left`/`d_right` track widths.
- Fit the local offset in unwrapped global Frenet `s`, clip cubic overshoot to the control-point
  extrema, and convert each selected global waypoint with its own normal. Preserve `s_m` and order.
- Validate lateral slope, recomputed Cartesian curvature, curvature rate, obstacle clearance, and
  track-bound clearance before publishing.
- Prefer a committed side while it remains feasible. Keep a validated commitment until its tail
  merges at `d=0`, even if perception drops the passed obstacle.
- Append a speed-aware ordered global `d=0` tail after the spline merge. After geometric merge,
  publish one rotated full global loop with ego at the start of the final state-machine tail ratio.
  Continue that non-empty handoff path until `/state` has entered `STATE_AVOID` for the commitment
  and subsequently confirms `STATE_GLOBAL`. Do not modify state-machine behavior for this handoff.
- If neither side is safe, publish only a collision-checked gradual-stop prefix before the obstacle.
  Never publish an unvalidated avoidance path merely to keep `/avoid_waypoints` non-empty.
- Recompute heading, curvature, velocity, and acceleration after applying `d(s)`.
- Handle closed-track `s` wrap explicitly. Never encode a waypoint index in Frenet odometry fields.

## Interfaces

- Subscribe: `/global_waypoints` (`f110_msgs/msg/WpntArray`).
- Subscribe: `/static_obs` (`f110_msgs/msg/ObstacleArray`); each obstacle
  must set `has_cartesian=true` and provide `x_center`, `y_center`, and a positive `radius`.
- Subscribe: `/car_state/frenet/odom` (`nav_msgs/msg/Odometry`), with `position.x=s` and
  `position.y=d`.
- Subscribe: `/state` (`f110_msgs/msg/StateMachine`) for explicit AVOID-to-GLOBAL handoff
  acknowledgement.
- Publish: `/avoid_waypoints` (`f110_msgs/msg/OTWpntArray`) as an ego-to-merge segment with
  map-frame Cartesian `x_m/y_m` populated for every waypoint.
- Publish debug: `/local_planning/path`, `/local_path`, `/local_planning/markers`.
- Standalone `/local_waypoints` publication stays disabled by default because `wpnt_publisher`
  selects `/avoid_waypoints` according to `/state`.

## Package layout and maintenance

- Node declaration: `include/local_planning/local_planner_node.hpp`.
- Algorithm declaration: `include/local_planning/raceline_spline_planner.hpp`.
- C++ sources: `src/local_planner_node.cpp`, `src/raceline_spline_planner.cpp`.
- Runtime parameters: `config/local_planning.yaml`.
- Launch entrypoint: `launch/local_planning.launch.py`.
- Korean node documentation: `docs/local_planner.md`.
- Algorithm tests: `test/test_raceline_spline.cpp`, including the wrong-branch snake regression.
- Manual Cartesian contract harness: `test/cartesian_static_pipeline_test.py`; run it against a
  fresh `local_planner_node` with a `global_waypoints.csv` path.
- End-to-end detector harness: `test/static_obs_pipeline_test.py`; run it while
  `obstacle_detector_node` and `local_planner_node` are active to verify
  `/scan -> /static_obs -> /avoid_waypoints`.
- Keep all runtime values configurable in YAML and load that YAML from the launch file.
- Update this file and the Korean documentation when behavior, topics, parameters, or launch usage
  changes.
