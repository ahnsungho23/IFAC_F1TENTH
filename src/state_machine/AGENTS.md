# AGENTS.md
State machine package rules. These instructions apply to `src/state_machine`.

## Package Scope

- Keep this package focused on behavior state publication.
- `state_machine_node` must publish only `f110_msgs/msg/StateMachine` on the configured state topic.
- `state_machine_node` consumes `/car_state/frenet/odom`, `/global_waypoints`, `/avoid_waypoints`, `/overtake_waypoints` and must not subscribe to `/scan`.
- Do not publish `/local_waypoints` from this package.
- Do not move waypoint source selection into this package; that belongs to downstream waypoint selection logic.

## Package Layout

- Public node declarations live in `include/state_machine/`.
- C++ node implementations live in `src/`.
- Runtime parameters live in `config/state_machine.yaml`.
- Launch entrypoints live in `launch/`.
- Node documentation currently lives in the repo `README.md` and this `AGENTS.md` (the package `docs/` was removed 2026-07-13).

## Runtime Code

- Runtime code must be C++ for ROS 2 Humble.
- Use existing `f110_msgs` message types for project-specific interfaces.
- Transition logic is a `committed_state_`-based FSM (dwell and safety fallback were removed 2026-07-13):
  - `local_path_confirmed()` / `can_enter_avoid()` / `can_enter_overtake()`: GLOBAL→AVOID /
    GLOBAL→OVERTAKE entry gate. At least M of the latest N messages must contain a non-empty
    local path (default 3-of-5); stale freshness is not an entry gate. AVOID has priority.
  - `enter_to_global()` (+ `evaluate_enter_to_global()` entry point): AVOID/OVERTAKE→GLOBAL graceful merge-back judgment.
  - `resolve_requested_state()`: the FSM 1-step. It reads/updates `committed_state_` and returns the state to publish. No dwell, no separate safety fallback — the state changes only when an entry or merge-back condition fires.
- `enter_to_global()` assumes the **segment publishing convention**: local paths (`/avoid_waypoints`, `/overtake_waypoints`) are ego→merge segments in global-raceline frenet coordinates (`s_m`/`d_m`), tail converging to d→0. As of 2026-07-13 `local_planning` still publishes a full-loop copy — until it is converted, the avoid-side merge judgment is inaccurate.
- Keep refinements (dynamic obstacle prediction, score-based hysteresis) as TODOs.

## Parameters, Launch, Docs

- All topic names, frame names, publish rates, stale timeouts, and default states must be parameters with YAML defaults.
- Keep `launch/state_machine.launch.py` loading `config/state_machine.yaml`.
- Update this `AGENTS.md` (and `README.md` if run/launch changes) when changing node behavior, topics, parameters, or launch usage.
