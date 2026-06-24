# AGENTS.md

State machine package rules. These instructions apply to `planning/state_machine`.

## Package Scope

- Keep this package focused on behavior state publication.
- `state_machine_node` must publish only `f110_msgs/msg/StateMachine` on the configured state topic.
- Do not publish `/local_waypoints` from this package.
- Do not move waypoint source selection into this package; that belongs to downstream waypoint selection logic.

## Runtime Code

- Runtime code must be C++ for ROS 2 Humble.
- Use existing `f110_msgs` message types for project-specific interfaces.
- Keep detailed obstacle and overtake decision algorithms as TODOs until they are explicitly requested.

## Parameters, Launch, Docs

- All topic names, frame names, publish rates, stale timeouts, and default states must be parameters with YAML defaults.
- Keep `launch/state_machine.launch.py` loading `config/state_machine.yaml`.
- Update `docs/state_machine_node.md` when changing node behavior, topics, parameters, or launch usage.
