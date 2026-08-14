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
- Obstacle bounds are raw detector geometry. Compute the tracking-error tube by bilinear
  interpolation of the configured speed-by-absolute-curvature LUT, falling back to
  `tracking_error_reserve_m` only when the LUT is intentionally empty. For obstacle target
  generation, use the maximum LUT value across the obstacle reference span. Limit the reference
  or candidate speed first with the configured jazzy_main-derived speed/lateral-acceleration table
  so `v^2 * abs(kappa) <= a_lat_max(v)`. For sampled-path, frozen-path, and raw hard-collision
  validation, use each candidate waypoint's limited speed and recomputed curvature. Obstacle
  clearance is `vehicle_half_width_m + safety_margin_m + tube`.
  The lateral-acceleration table only binds in curves, so an obstacle on a straight would otherwise
  be planned at race-line speed and reserve the widest tube in the LUT, closing gaps the car could
  hold at a lower speed. Cap each waypoint that is passing an obstacle by the gap as well: the tube
  it may spend is the lateral room left between the path and the obstacle face, and speed is
  lowered only until the tube fits, never below `avoidance_minimum_speed_mps`. Below that floor the
  maneuver is infeasible and safe-stop decides; never crawl through on a tube the car cannot hold.
  A waypoint passing no obstacle is never slowed. Keep this inversion of the LUT strict: a reduced
  speed whose tube overshoots the available room by the validator's own tolerance spends room the
  path does not have, and the validator then rejects the candidate the cap existed to enable.
  Global waypoint `d_left/d_right` are reference-to-physical-boundary distances. Validate every
  candidate's 0.56 m x 0.287 m, base-link-centred rectangular corners using candidate x/y/yaw and
  widths interpolated on the matching local reference segment. Subtract only
  `wall_safety_margin_m`, exactly once. Never add the tracking tube, obstacle margin, simulator TTC
  sweep, scan-noise guard, or another boundary/commitment/fallback margin.
- P3 lifecycle is continuation-first: an active recorded maneuver is continued (frozen path,
  revalidated every callback via guard containment + raw fallback) BEFORE any fresh M1
  selection, and fresh selection runs only with no active maneuver or in the same callback in
  which continuation invalidated. Do not restore fresh-first ordering: it re-shapes the
  published path every callback while the obstacle envelope is still being resolved.
- Committed-path retention band (`commitment_retention_reserve_fraction`, default 0.5): when
  re-validating an ALREADY COMMITTED path (P3 continuation raw fallback, P0 commitment hard
  check), the tracking-error reserve portion of the obstacle clearance is scaled by this
  fraction; the physical base clearance is never reduced and fresh planning always validates
  with the full reserve. A broken uncertainty guard or a full-margin violation inside the
  retention band holds the frozen geometry indefinitely (the old N-cycle soft-confirmation
  expiry is removed — it re-planned a nearly identical path on every progressive reveal, the
  dominant visible churn); invalidation/replacement requires an actual retention-margin
  violation, a non-obstacle failure, or completion. Do not reinstate the expiry and do not let
  fresh candidates validate with a scaled reserve.
- Localization reserve (`localization_reserve_m`, default 0.06): a constant floor added INSIDE
  `RacelineSplineParameters::trackingErrorReserve()` — the single choke point — so envelope
  expansion, hard validation, and the gap-limited speed inversion all see the same total. Do
  not add it separately at call sites (double counting) and do not remove it from the
  inversion path (the planner would pick speeds whose clearance the validator then rejects).
  Sized from sustained sim GT-vs-MCL error (per-speed-bin P95, 2026-08-13 probe); transient
  single-cycle MCL correction spikes are absorbed by the retention band, NOT this margin —
  do not resize it to the spike maximum (closes corridors). Keep it OUT of the CMA parameter
  whitelist: under GT localization the optimizer would drive it to zero. Re-measure on the
  real car (tools/mcl_gt_error_probe.py needs GT, so use MCL covariance/particle spread
  logging there) before real-car obstacle runs. On P3 completion, hand back through `activateGlobalHandoff` (the same
  closed global loop P0 uses) — never a frozen post-obstacle tail, which falls behind the ego
  and starves the FSM merge confirmation. The completion branch must re-register the completed
  maneuver's obstacle ids into `completed_obstacle_ids_` across the `clearCommitment()` wipe
  (P0-completion semantics): the detector's static occlusion hold keeps the passed track
  published, and without the exclusion the just-passed obstacle re-enters
  `buildNextManeuverInput` while ego is inside its padded span — the chained cluster then
  starts at ego and the safe-stop latch stalls the car inside its own danger region.
  Keep `maximum_exit_length_m` disabled (0) unless a
  single isolated obstacle is guaranteed: the long exit deliberately carries the offset over
  closely-following obstacles, and capping it makes the committed path dive into the next
  obstacle's inflated box the moment the detector reveals it, at a range where no fresh plan
  exists yet.
- The tracking-error LUT must stay measured, not guessed: obstacle-free laps on the clean map
  (`tools/cmaes_tuning/clean_lap_lut_traces.sh` in sim, or the real-car procedure in
  `docs/local_planner.md`) are the only valid source, and low-speed rows need dedicated
  speed-capped laps because racing never visits those cells. A monotone-filled cell silently
  becomes the span-maximum reserve and can block gaps the car could actually hold.
- `maximum_curvature_radpm` equals the vehicle's full-lock curvature (tan(0.41)/0.33 = 1.317),
  which is also the race line's own maximum. Treat it as physics, not a tunable: an obstacle
  inside or just after a maximum-curvature corner is unavoidable on both sides (the shifted
  line must exceed full lock), and safe-stop there is the correct verdict. Do not "fix" such a
  scenario by raising this limit.
- For every permitted side, sample `target_d_candidate_count` targets between the minimum
  obstacle-clearance offset and the maximum track-bound/`maximum_target_offset_m` offset. Generate
  every target/entry/exit combination before selecting; never return the first valid candidate.
  Hard-reject candidates with the existing transition, wall, obstacle, slope, curvature, and
  curvature-rate checks. Rank feasible candidates lexicographically by maximum minimum normalized
  wall/obstacle/curvature/curvature-rate slack, then minimum speed loss, then minimum global-line
  deviation. Do not replace this with a weighted sum.
- Reject a side before spline fitting when even its minimum-clearance target cannot fit the waypoint
  track widths across the expanded obstacle-cluster span. Use the remaining track-bound interval as
  the target sampling range; full sampled-path validation still applies before and after the span.
  That track-bound interval is for the vehicle CENTRE, so subtract `vehicle_half_width_m` alongside
  `wall_safety_margin_m`: the hard validator checks the rotated rectangle, and a gate that omits
  the half width offers targets the footprint check can never accept. When the race-line-speed gate
  does not fit, retry once against the gate the pass would need at `avoidance_minimum_speed_mps`
  before declaring the side blocked — a gap that is merely slower must not read as unreachable.
  This widens what is considered, never what is accepted; every candidate is still validated at the
  speed it actually ends up with.
- Build entry and exit offsets in unwrapped global Frenet `s` with a monotone quintic smoothstep.
  Keep `d`, `dd/ds`, and `d2d/ds2` continuous at the ego/target/global-line joins, clamp the
  profile to the ego/target extrema, and convert each selected global waypoint with its own normal.
  Preserve `s_m` and order. Keep entry and exit parameter families separate. Convert each positive
  `entry_transition_fraction` and nominal `pre_apex_distances_m[0]` into an effective length with
  `available * pre_apex_far / detection_lookahead * entry_fraction`, where `available` is
  ego-to-cluster distance; never clamp multiple entry values to the same ego start. Require
  `pre_apex_far <= detection_lookahead`. Preserve those configured candidates in order and append
  exactly one non-tunable candidate with effective length equal to the complete available
  ego-to-cluster distance. Treat `transition_distance_scales` and
  `outside_line_transition_scale` as exit-only compatibility parameters. Generate all combinations.
- Validate lateral slope, recomputed Cartesian curvature, curvature rate, obstacle clearance, and
  yaw-aware rectangular-footprint track-bound clearance before publishing. Keep corner projection
  on the candidate waypoint's local track branch to avoid nearby snake-track branch aliasing. Keep
  the legacy
  centerline headroom as the existing ranking metric, but hard-reject any footprint violation with
  `footprint_track_bound`. Track-bound validation uses `wall_safety_margin_m` exactly once. Do not
  add any further boundary, commitment, hard, or fallback margin.
- Before the first lateral commitment, publish `ot_line=raceline_static_prepare` with a validated
  braking prefix while collecting the nearest cluster's IDs and conservative Frenet-envelope union.
  Count distinct `/static_obs` messages, not planning ticks, and require the configured number of
  observations for every cluster ID and the configured minimum stabilization duration unless the
  maximum wait is reached. Expand the final union longitudinally by `k*sqrt(s_var)` plus the fixed
  longitudinal floor, but preserve the union's detector-owned `d_right/d_left` without lateral
  covariance inflation. Freeze that Guard with the commitment. An obstacle already inside the stop
  buffer bypasses this wait and enters safe-stop immediately.
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
- Separate commitment violations into hard physical collisions and soft uncertainty-envelope
  collisions. Both checks use the same unified physical clearance. Test hard collisions against
  raw detector bounds and replan immediately; test soft collisions against uncertainty Guards.
  Require the configured consecutive planning
  cycles before acting on a soft-only collision, clearing the count as soon as the frozen path is
  valid again. Never debounce track-bound, path-exhaustion, or geometry failures. Log the offending
  obstacle ID, waypoint `s/d`, obstacle `s/d` bounds, and applied clearance.
- Append a speed-aware ordered global `d=0` tail after the spline merge. After geometric merge,
  publish a full global loop with `ot_line=raceline_global_handoff`. Continue that non-empty
  handoff path until `/state` has entered `STATE_AVOID` for the commitment and subsequently
  confirms `STATE_GLOBAL`.
- Merge ramp (ego d → 0 smoothstep grafted onto the global handoff loop, 2026-08-13) is
  implemented but DEFAULT-OFF (`merge_ramp_min_length_m`/`merge_ramp_time_sec` = 0). A bare d=0
  loop delegates the return to the controller's natural convergence (real car: 0.055 m/m), but
  enabling the ramp with the current waypoint d_left/d_right shaved the sim wall pinch minimum
  from 0.117 to 0.082 m — the wall clamp cannot bind because those bounds are optimistic by a
  measured 0.16-0.23 m. Enable ONLY after the boundary data is calibrated (control team's
  per-sector lidar wall-clearance table), and re-run the lockstep baseline before adopting.
- Stabilize every non-active blocking cluster from the current ego state concurrently while the
  active maneuver runs; do not use the old `merge_s` as the next-cluster observation origin.
  Once the active Guard rear plus `chain_release_distance_m` is behind ego, allow a feasible next
  spline anchored at the current `ego.d` to preempt the old merge. Retire the completed IDs and
  release their side lock, but keep `/avoid_waypoints` non-empty and `STATE_AVOID` active. Continue
  validating the current commitment against obstacles that lie before its merge until a validated
  chained path replaces it. A post-merge controller-tail obstacle must not make the current
  maneuver fail. Hand off to GLOBAL only after no unfinished blocking cluster remains.
- If neither side is safe, publish only a collision-checked gradual-stop prefix before the obstacle.
  During an active avoidance, derive that prefix from the remaining committed geometry so stopping
  never forces an immediate return to `d=0`. Without a usable committed prefix, keep the current
  `ego.d`; if no forward stop prefix exists, publish a zero-speed current-`d` hold rather than an
  empty path that would fall back to global. Latch safe-stop and its obstacle IDs/sequence, danger
  `s` range, stop target, activation ego `s`, and timestamp immediately. An empty `/static_obs`
  array or an empty-cycle count is never a release condition. Release only after ego passes the
  latched danger range with margin, a hard-valid path for the same obstacle is consecutively
  confirmed and selectable in `STATE_AVOID`, or a stopped vehicle sees a persistently and
  explicitly clear forward corridor in fresh detector-health frames. While the danger remains
  logically ahead, reject global-raceline handoff. Never publish an unvalidated moving avoidance
  path merely to keep `/avoid_waypoints` non-empty.
- Treat stale Frenet odometry as a worst-case localization failure: publish a zero-speed hold at the
  last known pose without erasing a previously validated commitment. Resume ordinary validation and
  planning only after fresh odometry returns.
- Recompute heading and curvature after applying `d(s)`, cap moving-avoidance waypoint speed with
  the configured velocity-limit table, then recompute longitudinal acceleration. Do not apply this
  cap to global handoff geometry; safe-stop keeps its separate braking profile.
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
- Never publish `/local_waypoints`; `state_machine_node` exclusively selects and publishes it
  according to its committed state.
- Tuning-only timing diagnostics are default-off companion messages. T0 is the first relevant
  non-empty `/static_obs` accepted by the node and T1 is the first actual non-empty
  `/avoid_waypoints` publication. Capture the event's steady-clock time at the source operation and
  never feed `/cma_timing/events` back into perception or planning.
- Tuning-only record/replay diagnostics are also default-off. When enabled, publish passive JSON on
  `/cma_replay/planner_events` at initial-stabilization start/readiness and commitment. Include the
  source obstacle stamp and one `PLAN_CANDIDATE` record for every generated candidate: side, target,
  requested/effective entry, exit, centerline and rectangular-footprint wall clearance, footprint
  violation side/index/pose/relative heading/corner protrusion, obstacle clearance, peak
  curvature/rate, speed loss, rejection reason, and final rank. Do not introduce a decision clock
  or feed diagnostics back.
- `lockstep_mode` is CMA-only and default-off. It must suppress the wall planning timer and invoke
  the existing planner exactly once for each identical-stamp `/static_obs` and Frenet odometry
  pair. Before processing step k after the first step, require the state-machine output from step
  k-1 so DDS arrival order cannot select a different cached state.

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
- Keep `timing_diagnostics_enable=false` in the operational YAML.
- Keep `replay_diagnostics_enable=false` in the operational YAML.
- Keep `lockstep_mode=false` in the operational YAML.
- `p3_mode=TEST_ACTIVE` is the operational default (2026-08-12, user decision: P3 is the primary
  driving mode with P0 as backup). `SHADOW` must never publish a P3 path or mutate the P0
  commitment/safe-stop state. Every callback must identify `P3_M1`, `P3_COMMITTED_SUFFIX`, or
  `P0_BACKUP_ONLY` ownership.
- Safe-stop escalation is graded; do not collapse the ladder: (1) a margin-only blocking cluster
  (every member's RAW envelope + `vehicle_half_width_m + safety_margin_m` stays clear of d=0)
  degrades to the capped-speed lane hold (`margin_pass_speed_cap_mps`, `margin_pass=true`
  commitment that skips margin-based validators and is instead re-checked physically every
  cycle); (2) a physically blocking cluster brakes on the collision-free stop prefix; (3) with
  no stop prefix, brake along `last_valid_guidance_path_` (`buildCommittedPathStop`, then
  `buildLastPathBrake`); (4) the in-place zero-speed emergency hold is the last resort only
  when no valid guidance path was ever published.
- Never publish a slow section as a flat step from the ego position
  (`approach_feasibility_decel_mps2`, 2026-08-14): the margin pass ramps down from the MEASURED
  ego speed, and the avoidance spline carries a backward braking ramp into the obstacle-span
  speed. A flat step saturates the service brake, slips past the friction limit and cost us
  steering authority on the real car (run_0814_111210 wall crash). Obstacle-span speeds
  themselves are reserve-backed — never raise them.
- P3/M1 is production-owned C++ in this package. External CMA/evaluator executables are parity
  oracles only and must never supply runtime local paths.
- Run P3/M1 candidate generation immediately for every authoritative non-empty snapshot. Guard
  readiness is diagnostic provenance, not a standalone ownership veto: before the observation
  count/time Guard is complete, grant initial ownership only when the exact validator proves the
  selected path hard-valid against both the accumulated conservative geometry and the same
  callback's raw obstacle geometry. Do not add a second P3-only waiting parameter. After commit,
  never grow the frozen Guard by permanent lifetime union; use exact-ID containment and the
  guarded-then-raw exact-validation contract for each fresh live envelope.
- Preserve a selected P3 maneuver's immutable original geometry. Lifecycle continuation may trim
  only its passed prefix and must exact-revalidate the current suffix; it must complete after the
  expanded obstacle region is passed before a short suffix reaches the validator minimum.
- In `TEST_ACTIVE`, if current raw-obstacle validation discards a committed P3 suffix, run the
  unchanged P3/M1 planner at most once more in that callback using the exact same immutable
  snapshot. Publish only a fresh exact-hard-valid result; otherwise use the existing
  `P0_BACKUP_ONLY`/safe-stop fallback. Never retain or publish the rejected suffix.
- Update this file and the Korean documentation when behavior, topics, parameters, or launch usage
  changes.
