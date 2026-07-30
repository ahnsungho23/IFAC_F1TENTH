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

- Runtime code is C++17 for ROS 2 Jazzy.
- Use `f110_msgs/msg/ObstacleArray`, `WpntArray`, and `OTWpntArray`; do not create a new message.
- Consume map-frame Cartesian AABBs from `obstacle_detector` Layer 2 on `/static_obs`. Project each
  AABB center through the shared CLCS converter to lock the correct track branch. Compute the exact
  shortest distance from the AABB faces to the ordered global-race-line polyline only inside that
  branch's local longitudinal window; use it for the race-line-facing Frenet bound and blocking
  decision. Keep the centre-tangent four-corner envelope for the far bound and longitudinal extent
  used by avoidance construction.
- Derive each target `d` from the obstacle lateral bound plus configured clearance and the small
  commitment reserve. Reject targets outside the per-waypoint `d_left`/`d_right` track widths.
- Fit the local offset in unwrapped global Frenet `s`, clip cubic overshoot to the control-point
  extrema, and convert each selected global waypoint with its own normal. Preserve `s_m` and order.
- Validate lateral slope, recomputed Cartesian curvature, curvature rate, obstacle clearance, and
  track-bound clearance before publishing.
- Before the first lateral commitment, publish `ot_line=raceline_static_prepare` with a validated
  braking prefix while collecting the nearest cluster's IDs and conservative Cartesian AABB union.
  Reset the stabilization timer on a new cluster ID or meaningful envelope expansion, but use the
  configured maximum wait as an upper bound. An obstacle already inside the stop buffer bypasses
  this wait and enters safe-stop immediately.
- Freeze committed path geometry while its remaining forward portion is still valid against the
  latest AABBs. Replan only after that validation fails. A side may be reselected before the
  configured lateral/longitudinal engagement threshold, then it is locked for the rest of the
  maneuver. Keep the commitment until its tail merges at `d=0`, even if perception drops the passed
  obstacle.
- Append a speed-aware ordered global `d=0` tail after the spline merge. After geometric merge,
  publish a full global loop with `ot_line=raceline_global_handoff`. Continue that non-empty
  handoff path until `/state` has entered `STATE_AVOID` for the commitment and subsequently
  confirms `STATE_GLOBAL`.
- Treat a blocking cluster whose expanded front face begins after the active merge as the next
  maneuver, so it cannot invalidate the active spline merely through its post-merge controller
  tail. At merge completion or during global handoff, preempt handoff when such a cluster is
  present, retire the completed cluster IDs, release its side lock, and run the normal preparation
  stabilization before committing the next maneuver. Keep `/avoid_waypoints` non-empty throughout
  the chain and hand off to GLOBAL only when no uncompleted blocking cluster remains.
- If neither side is safe, publish only a collision-checked gradual-stop prefix before the obstacle.
  Latch safe-stop immediately and release it only after the configured number of consecutive safe
  plans. If no forward stop prefix exists, publish a zero-speed hold path rather than an empty path
  that would fall back to global. Never publish an unvalidated moving avoidance path merely to keep
  `/avoid_waypoints` non-empty.
- Recompute heading, curvature, velocity, and acceleration after applying `d(s)`.
- Handle closed-track `s` wrap explicitly. Never encode a waypoint index in Frenet odometry fields.

## Interfaces

- Subscribe: `/global_waypoints` (`f110_msgs/msg/WpntArray`).
- Subscribe: `/static_obs` (`f110_msgs/msg/ObstacleArray`); each obstacle
  must set `has_cartesian=true` and provide finite, ordered `x_min`, `x_max`, `y_min`, and `y_max`
  fields forming a non-point AABB. The enclosing `radius` remains message metadata and is not used
  as the planner geometry.
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
- AABB projection tests: `test/test_aabb_frenet_projector.cpp`, including independent longitudinal
  and lateral extents, a rotated track frame, and curved-race-line closest-face distance.
- Manual Cartesian contract harness: `test/cartesian_static_pipeline_test.py`; run it against a
  fresh `local_planner_node` with a `global_waypoints.csv` path.
- Initial-cluster harness: `test/initial_cluster_stabilization_pipeline_test.py`; run it against a
  fresh `local_planner_node` to verify that a late adjacent ID resets stabilization and affects the
  first committed side.
- Pre-engagement switch harness: `test/pre_engagement_side_switch_pipeline_test.py`; keep ego before
  both lock thresholds and verify that an invalidated side is replaced directly by the other side.
- End-to-end detector harness: `test/static_obs_pipeline_test.py`; run it while
  `obstacle_detector_node` and `local_planner_node` are active to verify
  `/scan -> /static_obs -> /avoid_waypoints`.
- Safe-stop/state harness: `test/safe_stop_latch_pipeline_test.py`; run it with
  `local_planner_node`, `state_machine_node`, and `wpnt_publisher` to verify the same-ID
  avoidance-to-stop latch, delayed release, and `/local_waypoints` forwarding contract.
- Sequential-obstacle harness: `test/sequential_obstacle_handoff_pipeline_test.py`; run it against
  a fresh `local_planner_node` to verify left-to-right maneuver chaining without an intermediate
  empty path or global handoff. Run it again with `--during-handoff` to verify handoff preemption.
- Keep all runtime values configurable in YAML and load that YAML from the launch file.
- Update this file and the Korean documentation when behavior, topics, parameters, or launch usage
  changes.
