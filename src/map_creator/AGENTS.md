# AGENTS.md
map_creator package rules. These instructions apply to `src/map_creator`.

## Package Scope

- `map_creator_node` implements the lap-transition obstacle_map pipeline
  (`learning_adaptive_globalpath/MAP_CREATOR_PROPOSAL.md` is the normative design):
  laps-1-and-2 `/adaptive_obstacle_map` Cartesian snapshots projected onto the immutable P0
  reference into a ledger → side decision at the lap 2-to-3 transition
  → blocked-side painting →
  offline regeneration → gated swap via `/global_planning/reload_waypoints`.
- The left/right side decision MUST go through
  `local_planning::RacelineSplinePlanner::plan` (linked via the exported
  `local_planning::raceline_planner` target). Never replicate the decision
  logic in this package or in Python. (`evaluateObstacleScenario` was removed from
  local_planning in the sungho_main sync `70c5a9d`; plan() gates on
  isBlockingRaceline, so `SidePlannerAdapter::decide` maps kNoObstacle to the side
  the race line already passes on via the obstacle's Frenet d sign.)
- The painting target is the trajectory-generator input map only (gui_params
  `map_yaml`). Never paint the MCL map or the local_planning wall-only reference map.
- Every paint session starts from the pristine base map (immutable baseline).
  Never repaint on top of a previously painted obstacle_map.
- Regeneration runs `offline_trajectory_generator/regenerate_obstacle_map.py`
  (loads gui_params.yaml through the same `load_gui_params` the GUI uses).
  Do not invoke `trajectory_gui.py` as a process.
- The first regeneration pass overrides smoothing with `initial_smooth_sigma` and the free-mask
  cleanup with `initial_morph_kernel`. The single retry keeps `retry_safety_width` and overrides
  with `retry_smooth_sigma` / `retry_morph_kernel`. A morph value <= 0 keeps the gui_params
  morph_kernel; 1 makes open/close an identity so painted shapes stay intact. All pass-specific
  values belong in `config/map_creator.yaml`.
- Any failure at any stage keeps the previous global line (local avoidance keeps
  covering). No partial bakes: either all decided obstacles are painted or none.
- After painting, verify both `obstacle_map.png` and `obstacle_map.yaml` are regular files before
  starting regeneration. A missing artifact must abort the pipeline without invoking the optimizer.

## Package Layout

- ROS-free modules in `include/map_creator/` + `src/`:
  `frenet_aabb` (Cartesian AABB -> P0 Frenet projection via the exported
  `global_planning` CLCS converter; centre-tangent corner rotation),
  `obstacle_ledger` (authoritative persistent snapshots, wrap-aware P0-projected-Frenet
  geometry matching, absence-driven removal hysteresis; transport silence is not absence),
  `side_planner_adapter` (tuned-parameter planner instance + fixed ego protocol),
  `map_painter` (OpenCV painting, `<250` non-drivable predicate, paint value 0,
  wall-connected polygons — obstacle-only painting is forbidden: 16 px < cleanup
  speckle protection 25 px), `regeneration_manager` (single-flight subprocess).
- ROS coupling only in `src/map_creator_node.cpp` (FSM: IDLE → GENERATING → ARMED
  → MONITORING / ABORTED).
- Runtime parameters in `config/map_creator.yaml` (the `decision.*` block is the
  full side-decision snapshot; unlisted planner params must equal the deployed
  `local_planning.yaml` values, and provenance of tuned values must be commented).
- Launch from the workspace root (`~/2026_IFAC`) — relative paths in the YAML
  resolve from the launch working directory, same convention as global_planning.
- `map_creator.launch.py` also starts `static_obstacle_map`; do not start a duplicate
  persistent-map node when using the default launch composition. That include MUST stay
  wrapped in `GroupAction(..., scoped=True)`: `IncludeLaunchDescription.launch_arguments`
  is unscoped, so an unwrapped include overwrites the parent `params_file` and
  `map_creator_node` silently receives `static_obstacle_map.yaml` instead of its own YAML
  (every parameter falls back to the C++ default, `base_map_yaml` becomes empty, and the
  gui_params fallback path aborts the pipeline). See `docs/handoff_params_leak.md`.
  Verify after any launch edit with `ros2 param get /map_creator_node base_map_yaml`.
- The `/adaptive_obstacle_map` contract is the Cartesian AABB (`has_cartesian` +
  `x_min/x_max/y_min/y_max`); the producer never fills the Frenet fields. map_creator projects
  each AABB onto the immutable P0 reference itself, through the CLCS converter built atomically
  with the side-decision adapter from the FIRST `/global_waypoints` message (`frenet_aabb.cpp`:
  centre globally projected, corners rotated into the centre tangent frame so no corner can
  branch-flip). Incoming Frenet fields are ignored.
- Snapshot ingestion is all-or-nothing: if any static obstacle in a snapshot fails projection
  (or the frame/AABB is invalid), reject the WHOLE snapshot and keep the previous ledger.
  Dropping a single failed obstacle would read as a real disappearance and start the
  `removal_miss_laps` hysteresis. An empty array is a valid snapshot. A trigger lap reached
  while the last snapshot stands rejected aborts instead of freezing a stale ledger.
- P0 immutability holds for the node's lifetime only: if map_creator alone restarts after a
  swap, the latched `/global_waypoints` already carries the obstacle line and is captured as
  the new reference. Projection, side decision, and painting stay mutually consistent (they
  all use the captured reference), but the pipeline assumes map_creator starts before the
  first baseline publish and is not restarted alone mid-session.
- Tests in `test/` are gtest, ROS-free (ledger matching/removal, painter pixels).
- Korean operator documentation in `docs/map_creator_node.md`.

## Swap Preconditions

1. Driver completed with current gui_params values AND its physical gates passed
   (closed loop, off_map=0, strictly increasing s, |kappa| <= 3.2, obstacle
   clearance >= `min_obstacle_clearance_after_m`).
2. `/lap_count` advanced past the lap in which generation completed.
3. Reload-service unavailability may defer the request, bounded by
   `max_swap_deferral_laps`.

## Post-Swap Avoid Gate

- After a SUCCESSFUL swap (and only then — never at kArmed, where the old line
  through the obstacles is still live), the node sets the state_machine's
  `allow_avoid_transition` parameter via its `set_parameters` service:
  obstacle-line swap → `false` (the live line now clears the obstacles);
  baseline-rollback swap (`frozen_` empty) → `true` (local avoidance must cover
  again). Gated by `disable_avoid_after_swap`; target node name in
  `state_machine_node_name`. If the parameter service is not ready, the update
  stays pending and `tick()` retries; a rejected set is logged and dropped.

## Runtime Code

- Runtime code must be C++ for ROS 2 Jazzy. The Python driver is an offline
  helper script (allowed by the root CLAUDE.md policy), spawned as a subprocess.
- Prefer `f110_msgs`/`std_msgs`/`std_srvs` types; the reload interface is an
  argument-less `std_srvs/Trigger` by design. The global publisher keeps the baseline
  `map` source until this gated call switches it to the configured `obstacle_map` source.
- Post-swap side effects fire ONLY from the reload-success callback, via per-target
  AsyncParametersClient sends retried each tick until the service is ready:
  state_machine `allow_avoid_transition` (false on obstacle swap / true on rollback) and
  control `max_speed` (`swap_max_speed_mps` on obstacle swap, `rollback_max_speed_mps`
  on rollback when > 0). control_map_node accepts runtime updates for max_speed only;
  other parameters are warned about, not rejected, so `ros2 param get` can disagree with
  what control actually uses. Whether `swap_max_speed_mps` raises or lowers the cap
  depends on control's launch default (`control_real.launch.py max_speed`, currently 5.0,
  so 7.0 raises it) — do not describe the direction without checking that value.
- Update this AGENTS.md and `docs/map_creator_node.md` when behavior, topics,
  parameters, or launch usage change.
