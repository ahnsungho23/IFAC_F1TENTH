# AGENTS.md
State machine package rules. These instructions apply to `src/state_machine`.

## Package Scope

- Keep this package focused on behavior-state decisions and final waypoint-source selection.
- `state_machine_node` publishes `f110_msgs/msg/StateMachine`, the selected
  `f110_msgs/msg/WpntArray`, and its `nav_msgs/msg/Path` visualization mirror.
- `state_machine_node` consumes `/car_state/frenet/odom`, `/global_waypoints`,
  `/avoid_waypoints`, and `/overtake_waypoints`; it must not subscribe to `/scan`.
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
- Transition logic is a `committed_state_`-based FSM (dwell and safety fallback were removed 2026-07-13):
  - `local_path_confirmed()` / `can_enter_avoid()` / `can_enter_overtake()`: GLOBAL→AVOID /
    GLOBAL→OVERTAKE entry gate. At least M of the latest N messages must contain a non-empty
    local path (default 3-of-5); stale freshness is not an entry gate. AVOID has priority.
    `allow_avoid_transition` / `allow_overtake_transition` (code default true, YAML currently false)
    short-circuit these gates before the M-of-N check, so entry is fully blocked. They gate
    entry only — AVOID/OVERTAKE→GLOBAL merge-back ignores them. Read once at startup (no dynamic
    reconfigure callback); `publish_state_cycle()` also drops disabled sources from the input warning.
  - `enter_to_global()` (+ `evaluate_enter_to_global()` entry point): AVOID/OVERTAKE→GLOBAL graceful merge-back judgment.
  - `resolve_requested_state()`: the FSM 1-step. It reads/updates `committed_state_` and returns the state to publish. No dwell, no separate safety fallback — the state changes only when an entry or merge-back condition fires.
- `enter_to_global()` assumes the **segment publishing convention**: local paths (`/avoid_waypoints`, `/overtake_waypoints`) are ego→merge segments in global-raceline frenet coordinates (`s_m`/`d_m`), tail converging to d→0. As of 2026-07-13 `local_planning` still publishes a full-loop copy — until it is converted, the avoid-side merge judgment is inaccurate.
- `/state` is timer-driven at `publish_rate_hz`; `/local_waypoints` and
  `/local_waypoints/path` are emitted only by fresh Frenet odometry callbacks.
- The only exception is default-off `lockstep_mode` for CMA evaluation. It disables the wall timer,
  evaluates the FSM once for an identical-stamp Frenet/avoid-path pair, and then emits one selected
  local path with that logical timestamp. Production continues to use the 10 Hz timer.
- Tuning-only timing diagnostics are default-off companion messages. T2 is captured in the
  `/avoid_waypoints` callback at the first exact M-of-N satisfaction, T3 at the committed
  GLOBAL-to-AVOID change, and T4 at the first subsequent avoidance `/local_waypoints` publication.
  `tuning_publish_rate_hz_override` may change the FSM timer only while diagnostics are enabled;
  the production `publish_rate_hz=10` default must remain unchanged.
- Global waypoints are static validated data with no use-blocking TTL. Avoidance waypoints latch
  until an empty message arrives. Overtake waypoints use the configured receive-event hold period
  and are invalidated only by an empty message received after that period.
- `invalid_local_path_policy=global_fallback` is the only supported first-stage policy. Do not
  invent a stop path in this selector.
- Keep refinements (dynamic obstacle prediction, score-based hysteresis) as TODOs.

## Parameters, Launch, Docs

- All topic names, frame names, publish rates, stale/diagnostic timeouts, hold durations, waypoint
  counts, and default states must be parameters with YAML defaults. Keep timing diagnostics
  disabled in the operational YAML.
- Keep `lockstep_mode=false` in the operational YAML and expose it only through the launch argument.
- Keep `launch/state_machine.launch.py` loading `config/state_machine.yaml`.
- Never let a zero-tail-speed (or <3-waypoint) local path satisfy the AVOID/OVERTAKE ->
  GLOBAL merge check in `enter_to_global`. Safe-stop hold paths end at ego with vx 0, so
  without that guard the merge conditions pass "because the car is parked", the FSM flaps
  AVOID<->GLOBAL, and the brief GLOBAL ticks forward an obstacle-piercing global line +
  speed command that creeps the car into the obstacle (2026-08-13 real-car incident).
- Keep Korean operational documentation in `docs/state_machine_node.md` current, including both
  state and selected-waypoint interfaces.
- Update this `AGENTS.md` (and `README.md` if run/launch changes) when changing node behavior, topics, parameters, or launch usage.
