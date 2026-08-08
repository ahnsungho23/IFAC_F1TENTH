# steering_lookup Agent Guide

These instructions extend the repository-level `AGENTS.md`.

## Scope

- Target Ubuntu 24.04, ROS 2 Jazzy, and Python 3.12.
- This package is an importable support library, not a ROS 2 node.
- Keep the public `LookupSteerAngle.lookup_steer_angle(accel, vel)` API compatible.
- Resolve lookup CSV files through `ament_index_python`; retain direct-source fallback for offline
  tools.

## Layout and Messages

- `steering_lookup/`: installed Python library.
- `cfg/*.csv`: installed lookup resources.
- `cfg/analyse_tires.py`: legacy offline calibration analysis; it is not installed as runtime code.
- The package does not publish or subscribe to ROS messages.

## Verification

- Build with the repository Jazzy `cb` alias.
- Run the package pytest suite on the installed library and setup files.
- Validate at least one lookup using a packaged CSV after sourcing the install space.
