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
- Consume the detector-owned Frenet footprint on `/static_obs` without any Cartesian-to-Frenet
  conversion. Treat `s_start/s_end/d_right/d_left` as the authoritative obstacle geometry.
  Cartesian AABB fields are optional current-observation metadata used only for RViz markers.
- Derive each target `d` from the obstacle lateral bound plus configured clearance and the small
  commitment reserve. Reject targets outside the per-waypoint `d_left`/`d_right` track widths.
- Fit the local offset in unwrapped global Frenet `s`, clip cubic overshoot to the control-point
  extrema, and convert each selected global waypoint with its own normal. Preserve `s_m` and order.
- Validate lateral slope, recomputed Cartesian curvature, curvature rate, obstacle clearance, and
  track-bound clearance before publishing.
- Before the first lateral commitment, publish `ot_line=raceline_static_prepare` with a validated
  braking prefix while collecting the nearest cluster's IDs and conservative Frenet-envelope union.
  Count distinct `/static_obs` messages, not planning ticks, and require the configured number of
  observations for every cluster ID unless the maximum wait is reached. Expand the final union by
  `k*sqrt(s_var/d_var)` plus fixed longitudinal/lateral extent-noise floors and freeze that
  uncertainty Guard with the commitment. An obstacle already inside the stop buffer bypasses this
  wait and enters safe-stop immediately.
- For a committed same-ID obstacle, replace the live envelope with the frozen Guard whenever the
  complete live uncertainty envelope remains contained in it. Never slide the Guard from one
  measurement to the next. A Guard breach must still validate the frozen path against the live
  envelope; rebuild only when that validation fails. Keep physical obstacle clearance separate
  from the uncertainty and AABB-extent margins.
- Freeze committed path geometry while its remaining forward portion is still valid against the
  latest Frenet envelopes. Replan only after that validation fails. A side may be reselected before the
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
- Recompute heading, curvature, and longitudinal acceleration after applying `d(s)`. Preserve the
  global waypoint velocity profile for moving avoidance; only safe-stop paths may reduce velocity.
- Handle closed-track `s` wrap explicitly. Never encode a waypoint index in Frenet odometry fields.

## Interfaces

- Subscribe: `/global_waypoints` (`f110_msgs/msg/WpntArray`).
- Subscribe: `/static_obs` (`f110_msgs/msg/ObstacleArray`); each obstacle must provide finite
  `s_start/s_end/d_right/d_left` fields forming a non-point Frenet footprint, with
  `d_right <= d_left`. `s_start > s_end` is valid across the closed-track wrap. Cartesian fields
  and `radius` are optional metadata and are not used as planner geometry.
- Subscribe: `/car_state/frenet/odom` (`nav_msgs/msg/Odometry`), with `position.x=s` and
  `position.y=d`.
- Subscribe: `/state` (`f110_msgs/msg/StateMachine`) for explicit AVOID-to-GLOBAL handoff
  acknowledgement.
- Publish: `/avoid_waypoints` (`f110_msgs/msg/OTWpntArray`) as an ego-to-merge segment with
  map-frame Cartesian `x_m/y_m` populated for every waypoint.
- Publish debug: `/local_planning/path`, `/local_path`, `/local_planning/markers`.
- Never publish `/local_waypoints`; `wpnt_publisher` exclusively selects and publishes it according
  to `/state`.

## Package layout and maintenance

- Node declaration: `include/local_planning/local_planner_node.hpp`.
- Algorithm declaration: `include/local_planning/raceline_spline_planner.hpp`.
- Uncertainty Guard declaration: `include/local_planning/obstacle_guard.hpp`.
- C++ sources: `src/local_planner_node.cpp`, `src/obstacle_guard.cpp`,
  `src/raceline_spline_planner.cpp`.
- Runtime parameters: `config/local_planning.yaml`.
- Launch entrypoint: `launch/local_planning.launch.py`.
- Korean node documentation: `docs/local_planner.md`.
- Algorithm tests: `test/test_raceline_spline.cpp`, including the wrong-branch snake regression.
- Guard tests: `test/test_obstacle_guard.cpp`, including variance inflation, frozen-envelope
  containment, accumulated drift rejection, invalid-variance fallback, and closed-track wrap.
- AABB projection belongs to `obstacle_detector` and must not be reintroduced here.
- Manual Frenet contract harness: `test/frenet_static_pipeline_test.py`; run it against a fresh
  `local_planner_node` with a `global_waypoints.csv` path.
- Initial-cluster harness: `test/initial_cluster_stabilization_pipeline_test.py`; run it against a
  fresh `local_planner_node` to verify that each late adjacent ID receives the configured number of
  real topic observations and affects the first committed side.
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
