# AGENTS.md
State machine package rules. These instructions apply to `src/state_machine`.

## Package Scope

- Keep this package focused on behavior-state decisions and final waypoint-source selection.
- `state_machine_node` publishes `f110_msgs/msg/StateMachine`, the selected
  `f110_msgs/msg/WpntArray`, and its `nav_msgs/msg/Path` visualization mirror.
- `state_machine_node` consumes `/car_state/frenet/odom`, `/global_waypoints`,
  `/avoid_waypoints`, and `/opp_obs`; it must not subscribe to `/scan`.
- This package is the only normal publisher of `/local_waypoints`. Keep
  `local_planning.publish_standalone_local` disabled during integrated operation.
- Do not reintroduce a separate state-subscriber waypoint relay. State decisions and waypoint
  selection must use the same cached inputs and validity rules in this node.

## Package Layout

- Public node declarations live in `include/state_machine/`.
- C++ node implementations live in `src/`.
- Runtime parameters live in `config/state_machine.yaml`.
- Launch entrypoints live in `launch/`.
- Node documentation lives in `docs/state_machine_node.md`, the repo `README.md`, and this `AGENTS.md`.

## Runtime Code

- Runtime code must be C++ for ROS 2 Jazzy.
- Use existing `f110_msgs` message types for project-specific interfaces.
- Transition logic is a `committed_state_`-based FSM:
  - `local_path_confirmed()` / `can_enter_avoid()`: at least M of the latest N avoid messages must
    contain a non-empty local path (default 3-of-5). AVOID has priority from GLOBAL and CRUISE.
  - GLOBAL enters CRUISE when a fresh `/opp_obs` contains a dynamic obstacle with
    `is_interfering=true`.
  - CRUISE selects GLOBAL geometry and returns to GLOBAL on false, empty, or stale `/opp_obs`.
  - `enter_to_global()` judges AVOID merge-back. It returns to CRUISE if interference still exists,
    otherwise GLOBAL.
  - `evaluate_avoid_path_exhausted()` is the deadlock escape for a withdrawn or stale avoid path.
    It may release AVOID only after the ego reaches the retained last non-empty path tail and the
    condition persists for `avoid_path_exhaustion_sec`; clear the M-of-N history on this release.
- `resolve_requested_state()`: the FSM 1-step. It reads/updates `committed_state_` and returns the state to publish. No dwell, no separate safety fallback — the state changes only when an entry or merge-back condition fires.
- When AVOID's terminal waypoint is a zero-speed hold and the normal merge-back is therefore
  intentionally rejected, the guarded release may use `/static_obs`: the ego-front corridor must
  remain clear for `stopped_path_clear_sec`, and a stale detector stream is never treated as clear.
  This release forces GLOBAL and latches avoid re-entry until a new obstacle or positive-speed path
  is observed.
- `enter_to_global()` assumes the **segment publishing convention**: `/avoid_waypoints` is an
  ego→merge segment in global-raceline Frenet coordinates (`s_m`/`d_m`), tail converging to d→0.
- `/state` is timer-driven at `publish_rate_hz`; `/local_waypoints` and
  `/local_waypoints/path` are emitted only by fresh Frenet odometry callbacks.
- Global waypoints are static validated data with no use-blocking TTL. The current avoidance path
  becomes invalid on an empty message, while the last non-empty path is retained only for guarded
  exhaustion detection. Opponent interference is valid only for `opponent_stale_timeout_sec`.
- `invalid_local_path_policy=global_fallback` is the only supported first-stage policy. Do not
  invent a stop path in this selector.
- Keep refinements (dynamic obstacle prediction, score-based hysteresis) as TODOs.

## Parameters, Launch, Docs

- All topic names, frame names, publish rates, stale/diagnostic timeouts, hold durations, waypoint
  counts, and default states must be parameters with YAML defaults.
- Keep `launch/state_machine.launch.py` loading `config/state_machine.yaml`.
- Keep Korean operational documentation in `docs/state_machine_node.md` current, including both
  state and selected-waypoint interfaces.
- Update this `AGENTS.md` (and `README.md` if run/launch changes) when changing node behavior, topics, parameters, or launch usage.
