# AGENTS.md

State machine package rules. These instructions apply to `planning/state_machine`.

## Package Scope

- Keep this package focused on behavior state publication.
- `state_machine_node` must publish only `f110_msgs/msg/StateMachine` on the configured state topic.
- `state_machine_node` must consume perception obstacle arrays and must not subscribe to `/scan`.
- Do not publish `/local_waypoints` from this package.
- Do not move waypoint source selection into this package; that belongs to downstream waypoint selection logic.

## Package Layout

- Public node declarations live in `include/state_machine/`.
- C++ node implementations live in `src/`.
- Runtime parameters live in `config/state_machine.yaml`.
- Launch entrypoints live in `launch/`.
- Node documentation lives in `docs/state_machine_node.md`.

## Runtime Code

- Runtime code must be C++ for ROS 2 Humble.
- Use existing `f110_msgs` message types for project-specific interfaces.
- Use `f110_msgs/msg/ObstacleArray` for perception obstacle evidence.
- Keep detailed obstacle and overtake decision algorithms as TODOs until they are explicitly requested.
- Transition gating layers live in this node: `evaluate_local_path()` (published local path quality gate) and `apply_transition_stability()` (dwell + N-tick debounce, immediate safety fallback to GLOBAL). Keep refinements (dynamic obstacle prediction, score-based hysteresis) as TODOs.

## Parameters, Launch, Docs

- All topic names, frame names, publish rates, stale timeouts, and default states must be parameters with YAML defaults.
- Keep `launch/state_machine.launch.py` loading `config/state_machine.yaml`.
- Update `docs/state_machine_node.md` when changing node behavior, topics, parameters, or launch usage.
