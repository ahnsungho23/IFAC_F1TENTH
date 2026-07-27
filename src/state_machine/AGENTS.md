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
  - `is_not_null_ptr()` / `can_enter_avoid()` / `can_enter_overtake()`: GLOBAL→AVOID / GLOBAL→OVERTAKE entry gate. The local path must stay non-empty longer than its stale timeout before entry; AVOID has priority when both gates hold.
  - `enter_to_global()` (+ `evaluate_enter_to_global()` entry point): AVOID/OVERTAKE→GLOBAL graceful merge-back judgment.
  - `resolve_requested_state()`: the FSM 1-step. It reads/updates `committed_state_` and returns the state to publish. No dwell, no separate safety fallback — the state changes only when an entry or merge-back condition fires.
- Known gap: `is_not_null_ptr()` checks only the stored message and its non-empty streak start, not `last_*_time_` freshness. A stale-but-non-empty last message keeps `can_enter_*` true; combined with the removed safety fallback, entry/exit can latch. Fix only when requested.
- `enter_to_global()` assumes the **segment publishing convention**: local paths (`/avoid_waypoints`, `/overtake_waypoints`) are ego→merge segments in global-raceline frenet coordinates (`s_m`/`d_m`), tail converging to d→0. As of 2026-07-13 `local_planning` still publishes a full-loop copy — until it is converted, the avoid-side merge judgment is inaccurate.
- Keep refinements (dynamic obstacle prediction, score-based hysteresis) as TODOs.

## Parameters, Launch, Docs

- All topic names, frame names, publish rates, stale timeouts, and default states must be parameters with YAML defaults.
- Keep `launch/state_machine.launch.py` loading `config/state_machine.yaml`.
- Update this `AGENTS.md` (and `README.md` if run/launch changes) when changing node behavior, topics, parameters, or launch usage.
