# AGENTS.md

This file defines the working rules for AI coding agents in this repository. These rules apply to the entire repository unless a more specific `AGENTS.md` exists in a subdirectory.

## Communication

- All user-facing replies must be written in Korean unless the user explicitly requests another language.
- Keep repository guidance in English when practical to reduce token usage and make agent instructions compact.

## Target Environment

- Target platform: ROS 2 Humble.
- New ROS 2 runtime code must be written in C++.
- Use Python only where ROS 2 conventionally requires it, such as `launch.py` files or build/config helper scripts.
- Actively check and use relevant aliases from `~/.zshrc` when running build, test, launch, or debugging commands.

## Message Policy

- Prefer existing `f110_msgs` message types for inter-node communication.
- Prefer ROS 2 standard `std_msgs` types when a standard type is sufficient.
- Before creating a new message type, first verify that the requirement cannot be cleanly represented with `f110_msgs` or `std_msgs`.

## Parameter Policy

- Check every hard-coded value before adding it to source code.
- Move configurable values into YAML parameter files whenever practical.
- Configurable values include topic names, frame names, loop rates, thresholds, gains, file paths, modes, algorithm tuning values, and feature toggles.
- Nodes that require parameters must provide a matching YAML parameter file.
- Declare parameters and safe defaults clearly in the C++ node, while keeping operational values adjustable through YAML.

## Launch Policy

- Any node that requires parameters must provide a `launch.py` file.
- The `launch.py` file must load the matching YAML parameter file.
- Ensure `CMakeLists.txt` installs launch files and parameter files so the node can run with:
  `ros2 launch <package> <launch_file>.py`

## Node-Level AGENTS.md Policy

- Whenever creating a new node, create a node-level or package-level `AGENTS.md` in the most relevant directory for that node.
- Whenever modifying an existing node, check for the nearest applicable `AGENTS.md`. If it is missing, create one. If it exists but is outdated, update it.
- Node-level `AGENTS.md` files must describe the node-specific rules, package layout, message choices, parameter files, launch files, and documentation expectations.
- Node-level instructions may add constraints, but must not weaken or contradict this root `AGENTS.md`.
- Always follow the closest applicable `AGENTS.md` before editing a node.
- You must update repo using git before modifing.

## Node Documentation

- When adding a new node or significantly changing an existing node, write or update Markdown documentation inside the relevant package.
- The document must include:
  - Node purpose
  - Operating principle
  - Subscribed topics and published topics
  - Message types
  - Main parameters and YAML file location
  - How to run the node
  - `ros2 launch` example
- Use a clear filename that includes the node name, such as `docs/<node_name>.md`, unless the package already has a better local convention. Make sure to write documents step-by-step using Korean.

## Implementation Checklist

Before finishing any ROS 2 node change, verify:

- Runtime code is implemented in C++.
- `f110_msgs` or `std_msgs` were preferred for message usage.
- Configurable values were moved to YAML where practical.
- Nodes requiring parameters include a `launch.py` file.
- `CMakeLists.txt` installs binaries, launch files, parameter files, and documentation as needed.
- The nearest node-level or package-level `AGENTS.md` exists and is current.
- Node Markdown documentation explains operation and execution.
- The implemented code was built and run after completion.
- Runtime behavior was checked, and any launch/build/runtime issues were debugged before reporting completion.
