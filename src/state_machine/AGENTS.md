# AGENTS.md

State machine package rules. These instructions apply to `src/state_machine`.

## Package Scope

- Keep this package focused on behavior state publication.
- `state_machine_node` must publish only `f110_msgs/msg/StateMachine` on the configured state topic.
- `state_machine_node` consumes `/car_state/frenet/odom`, `/global_waypoints`, `/avoid_waypoints`,
  `/overtake_waypoints`, and `/perception/obstacles`; it must not subscribe to `/scan`.
- Do not publish `/local_waypoints` from this package.
- Do not move waypoint source selection into this package; that belongs to downstream waypoint selection logic.

## Package Layout

- Public node declarations live in `include/state_machine/`.
- C++ node implementations live in `src/`.
- Runtime parameters live in `config/state_machine.yaml`.
- Launch entrypoints live in `launch/`.
- Korean node documentation lives in `docs/state_machine_node.md`.

## Runtime Code

- Runtime code must be C++ for ROS 2 Humble.
- Use existing `f110_msgs` message types for project-specific interfaces.
- Static-obstacle transitions are `GLOBAL -> BLOCKED -> AVOID / SAFE_STOP`:
  - `global_corridor_blocked()` uses confirmed static obstacles, forward horizon, vehicle width,
    margin, and lateral covariance.
  - `can_enter_avoid()` requires `avoid_path_confirm_count` consecutive fresh, geometrically
    valid, collision-free segments.
  - `AVOID -> GLOBAL` requires merge completion, ego alignment, and a confirmed clear global
    corridor. A missing/stale path goes to `SAFE_STOP`, never directly to `GLOBAL`.
  - A last valid static path may be used only within `avoid_path_ttl_sec`.
- Preserve the existing dynamic-opponent `OVERTAKE` gate and merge-back behavior. Do not couple
  static obstacle changes into `opponent_detector`.
- `enter_to_global()` assumes local paths are ego-to-merge segments in global-raceline Frenet
  coordinates (`s_m`/`d_m`) with a tail converging to `d=0`.
- Keep refinements (dynamic obstacle prediction, score-based hysteresis) as TODOs.

## Parameters, Launch, Docs

- All topic names, frame names, rates, widths, margins, confirmation times, stale timeouts, and
  default states must be parameters with YAML defaults.
- Keep `launch/state_machine.launch.py` loading `config/state_machine.yaml`.
- Update this `AGENTS.md` (and `README.md` if run/launch changes) when changing node behavior, topics, parameters, or launch usage.
