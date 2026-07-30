# f1tenth_control Agent Guide

These instructions extend the repository-level `AGENTS.md`.

## Target and Runtime

- Target Ubuntu 24.04 and ROS 2 Jazzy.
- Keep ROS 2 runtime nodes in C++17. Python is limited to launch files and offline tools.
- The CPU MPPI solver must remain available when CUDA is not installed. Treat the CUDA solver as an
  optional build acceleration path.
- Do not bypass `joy_teleop_monitor` or the existing drive-source selection and emergency-stop path.

## Package Layout

- `control_code/`: control, mux, dashboard, calibration, and simulator bridge nodes.
- `include/f1tenth_control/`: shared C++ interfaces.
- `launch/`: real-car, simulator, dashboard, and LUT calibration entry points.
- `tools/`: offline bag analysis, calibration, and test helpers.
- `docs/`: node and operator documentation.
- `control_code/NUC6_glc_pacejka_lookup_table.csv`: installed steering lookup resource.

## Messages and Parameters

- Prefer existing `f110_msgs` interfaces and ROS 2 standard message types.
- Keep topic names, gains, limits, rates, modes, and safety thresholds configurable.
- Shared launch construction belongs in `launch/_control_common.py`; keep real and simulator entry
  points separate because their hardware and safety dependencies differ.
- Document any new parameter and provide it through the applicable YAML or launch configuration.

## Launch and Documentation

- `control_sim.launch.py` must run without VESC packages and use the simulator topic profile.
- `control_real.launch.py` may depend on the separately built Jazzy `f1tenth_ws` for
  `vesc_ackermann`.
- Install new launch files and runtime resources in `CMakeLists.txt`.
- Update the relevant Markdown file in `docs/`, `CLAUDE.md`, or `WORKLOG.md` when behavior changes.

## Verification

- Build with the repository `cb` alias or its equivalent Jazzy colcon command.
- Test the CPU-only build path even when CUDA is available.
- Validate launch parsing for both real and simulator entry points. Hardware behavior requires the
  external sensor/VESC stack and must not be claimed without a vehicle test.
