# AGENTS.md
map_creator package rules. These instructions apply to `src/map_creator`.

## Package Scope

- `map_creator_node` implements the lap-transition obstacle_map pipeline
  (`learning_adaptive_globalpath/MAP_CREATOR_PROPOSAL.md` is the normative design):
  laps-1-and-2 `/adaptive_obstacle_map` ledger → side decision at the lap 2-to-3 transition
  → blocked-side painting →
  offline regeneration → gated swap via `/global_planning/reload_waypoints`.
- The left/right side decision MUST go through
  `local_planning::RacelineSplinePlanner::evaluateObstacleScenario` (linked via the
  exported `local_planning::raceline_planner` target). Never replicate the decision
  logic in this package or in Python.
- The painting target is the trajectory-generator input map only (gui_params
  `map_yaml`). Never paint the MCL map or the local_planning wall-only reference map.
- Every paint session starts from the pristine base map (immutable baseline).
  Never repaint on top of a previously painted obstacle_map.
- Regeneration runs `offline_trajectory_generator/regenerate_obstacle_map.py`
  (loads gui_params.yaml through the same `load_gui_params` the GUI uses).
  Do not invoke `trajectory_gui.py` as a process.
- The first regeneration pass overrides smoothing with `initial_smooth_sigma`. The single retry
  keeps `retry_safety_width` and overrides smoothing with `retry_smooth_sigma`; all pass-specific
  values belong in `config/map_creator.yaml`.
- Any failure at any stage keeps the previous global line (local avoidance keeps
  covering). No partial bakes: either all decided obstacles are painted or none.
- After painting, verify both `obstacle_map.png` and `obstacle_map.yaml` are regular files before
  starting regeneration. A missing artifact must abort the pipeline without invoking the optimizer.

## Package Layout

- ROS-free modules in `include/map_creator/` + `src/`:
  `obstacle_ledger` (authoritative persistent snapshots, wrap-aware supplied-Frenet geometry
  matching, absence-driven removal hysteresis; transport silence is not absence),
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
  persistent-map node when using the default launch composition.
- Map creator must not perform Cartesian-to-Frenet conversion. Treat the `s`/`d` geometry in
  `/adaptive_obstacle_map` as the perception contract and only compare or consume those fields.
- Tests in `test/` are gtest, ROS-free (ledger matching/removal, painter pixels).
- Korean operator documentation in `docs/map_creator_node.md`.

## Swap Preconditions

1. Driver completed with current gui_params values AND its physical gates passed
   (closed loop, off_map=0, strictly increasing s, |kappa| <= 3.2, obstacle
   clearance >= `min_obstacle_clearance_after_m`).
2. `/lap_count` advanced past the lap in which generation completed.
3. Reload-service unavailability may defer the request, bounded by
   `max_swap_deferral_laps`.

## Runtime Code

- Runtime code must be C++ for ROS 2 Jazzy. The Python driver is an offline
  helper script (allowed by the root CLAUDE.md policy), spawned as a subprocess.
- Prefer `f110_msgs`/`std_msgs`/`std_srvs` types; the reload interface is an
  argument-less `std_srvs/Trigger` by design. The global publisher keeps the baseline
  `map` source until this gated call switches it to the configured `obstacle_map` source.
- Update this AGENTS.md and `docs/map_creator_node.md` when behavior, topics,
  parameters, or launch usage change.
