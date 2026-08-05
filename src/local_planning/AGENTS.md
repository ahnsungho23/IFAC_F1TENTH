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
  Cartesian AABB fields are optional metadata and are not consumed as planner geometry.
- Derive each target `d` from the obstacle lateral bound plus configured clearance and the small
  commitment reserve. Reject targets outside the per-waypoint `d_left`/`d_right` track widths.
  A raceline-centred obstacle demands the full obstacle width plus margins on BOTH sides at once;
  when both sides are rejected with the full `obstacle_clearance_m`, retry the whole selection
  once with the reduced `minimum_avoidance_clearance_m` (never below vehicle half-width plus the
  hard collision margin) before declaring a safe stop.
- When left/right candidate scores tie within `side_tie_epsilon_m`, select the side with more
  reference-width headroom across the obstacle span. Reference widths carry no perception jitter,
  so centred-obstacle side choices cannot flap between replans.
- Before fitting a side's spline, reject its target when it cannot fit the waypoint track widths
  across the expanded obstacle-cluster span. This is only an early pruning gate; every surviving
  sampled spline must still pass the full transition, wall, obstacle, slope, and curvature checks.
- Build entry and exit offsets in unwrapped global Frenet `s` with a monotone quintic smoothstep.
  Keep `d`, `dd/ds`, and `d2d/ds2` continuous at the ego/target/global-line joins, clamp the
  profile to the ego/target extrema, and convert each selected global waypoint with its own normal.
  Preserve `s_m` and order. Select entry and exit scales independently: try entry scales from
  longest to shortest so a distant obstacle uses the available approach distance, and try exit
  scales from shortest to longest so the maneuver releases promptly. Fall back only when the full
  candidate validation fails.
- Validate lateral slope, recomputed Cartesian curvature, curvature rate, obstacle clearance, and
  track-bound clearance before publishing.
- Before the first lateral commitment, publish `ot_line=raceline_static_prepare` with a validated
  braking prefix while collecting the nearest cluster's IDs and conservative Frenet-envelope union.
  Count distinct `/static_obs` messages, not planning ticks, and require the configured number of
  observations for every cluster ID and the configured minimum stabilization duration unless the
  maximum wait is reached. Expand the final union by
  `k*sqrt(s_var/d_var)` plus fixed longitudinal/lateral extent-noise floors, capping the lateral
  margin at `uncertainty_max_lateral_margin_m` so fresh-detection variance cannot inflate a
  centred obstacle's Guard beyond what either side can clear, and freeze that
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
  maneuver. After one pre-engagement side switch, switching back is disabled until engagement or
  the next maneuver so a centred-obstacle tie cannot weave the car. Keep the commitment until its
  tail merges at `d=0`, even if perception drops the passed
  obstacle.
- After at least one valid `/static_obs` message, treat input older than
  `obstacle_stale_timeout_sec` as degraded perception, not as proof that the track is clear. Retain
  the frozen commitment and the last valid obstacle snapshot, complete the odometry-based merge and
  GLOBAL handoff normally, and reuse the snapshot when planning on a later lap. A fresh valid
  obstacle array, including an explicitly empty array, replaces that memory. Rejecting a wrong-frame
  array must not erase it. Do not wait for repeated observations when replanning solely from retained
  stale memory because no new samples can arrive.
- Separate commitment violations into hard physical collisions and soft uncertainty/clearance
  collisions. Test hard collisions against detector bounds plus vehicle half-width and the
  configured hard margin, and replan immediately. Require the configured consecutive planning
  cycles before acting on a soft-only collision, clearing the count as soon as the frozen path is
  valid again. Never debounce track-bound, path-exhaustion, or geometry failures. Log the offending
  obstacle ID, waypoint `s/d`, obstacle `s/d` bounds, and applied clearance.
- Append a speed-aware ordered global `d=0` tail after the spline merge. After geometric merge,
  publish a full global loop with `ot_line=raceline_global_handoff`. Continue that non-empty
  handoff path until `/state` has entered `STATE_AVOID` for the commitment and subsequently
  confirms `STATE_GLOBAL`.
- Stabilize every non-active blocking cluster from the current ego state concurrently while the
  active maneuver runs; do not use the old `merge_s` as the next-cluster observation origin.
  Once the active Guard rear plus `chain_release_margin_m` is behind ego, allow a feasible next
  spline anchored at the current `ego.d` to preempt the old merge. Retire the completed IDs and
  release their side lock, but keep `/avoid_waypoints` non-empty and `STATE_AVOID` active. Continue
  validating the current commitment against obstacles that lie before its merge until a validated
  chained path replaces it. A post-merge controller-tail obstacle must not make the current
  maneuver fail. Hand off to GLOBAL only after no unfinished blocking cluster remains.
- If neither side is safe, publish only a collision-checked gradual-stop prefix before the obstacle.
  During an active avoidance, derive that prefix from the remaining committed geometry so stopping
  never forces an immediate return to `d=0`. Without a usable committed prefix, keep the current
  `ego.d`; if no forward stop prefix exists, publish a zero-speed current-`d` hold rather than an
  empty path that would fall back to global. Latch safe-stop immediately and release it only after
  the configured number of consecutive safe plans. Never publish an unvalidated moving avoidance
  path merely to keep `/avoid_waypoints` non-empty.
- Treat stale Frenet odometry as a worst-case localization failure: publish a zero-speed hold at the
  last known pose without erasing a previously validated commitment. Resume ordinary validation and
  planning only after fresh odometry returns.
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
- Publish debug: `/local_planning/path` (`nav_msgs/msg/Path`) only.
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
  fresh `local_planner_node` to verify the minimum stabilization time, that each late adjacent ID
  receives the configured number of real topic observations, and that it affects the first
  committed side.
- Soft-violation harness: `test/soft_violation_confirmation_pipeline_test.py`; run it against a
  fresh `local_planner_node` to verify a transient uncertainty-only collision keeps the commitment
  and a persistent soft collision replans only after the configured planning-cycle count.
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
- Post-merge-tail harness: `test/post_merge_tail_chaining_pipeline_test.py`; run it against a fresh
  `local_planner_node` to verify that a stabilized obstacle in the first path's controller-only tail
  preempts from nonzero `ego.d` without preparation, safe-stop, empty output, or GLOBAL handoff.
- Stale-perception harness: `test/stale_obstacle_memory_pipeline_test.py`; run it against a fresh
  `local_planner_node` to verify frozen-path retention beyond the stale timeout, GLOBAL handoff
  completion, and last-snapshot reuse on the next lap while `/static_obs` remains silent.
- Keep all runtime values configurable in YAML and load that YAML from the launch file.
- Update this file and the Korean documentation when behavior, topics, parameters, or launch usage
  changes.
