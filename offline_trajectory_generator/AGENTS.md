# AGENTS.md

This directory contains a standalone, offline trajectory generation tool.

Rules:
- Do not add ROS 2 runtime dependencies here.
- Keep the CLI runnable with plain `python3` after the workspace Python dependencies are available.
- Inputs are ROS map-server compatible SLAM map YAML/image files.
- Outputs should stay compatible with the existing `f110_msgs/Wpnt` field names when practical.
- Put user-facing operation docs in Korean.
- Prefer configurable CLI arguments over hard-coded map, speed, width, or smoothing values.
- When adding or changing generator parameters, update the GUI defaults, saved parameter YAML, and Korean README in the same change.

## Layout

- `generate_global_trajectory.py` — map -> centerline -> raceline -> speed profile pipeline (CLI).
  The standard speed profile reads `config/velocity_limits.csv` with columns
  `[speed_mps, max_accel_mps2, max_decel_mps2, max_lateral_accel_mps2]`; keep the CLI, GUI, and
  saved YAML on the same table semantics. Only `centerline` and `mincurv` are supported.
  Extraction robustness invariants (keep when refactoring): centerline candidates are ranked by
  ENCLOSED contour area (not arc length — noise scribbles are long but enclose nothing); skeleton
  pixels in corridors narrower than `min_track_width` are dropped; the Zhang-Suen fallback keeps
  the skeleton loop connected without opencv-contrib; map cleanup must NEVER erase measured walls
  (`cleanup_free_mask(occupied_mask=...)` stamps raw occupied pixels back after morphology so a
  big morph kernel cannot swallow a thin interior wall); `count_off_map_waypoints` checks densely
  (between-waypoint segments too) and must keep warning when the raceline leaves free space.
  The `--max-curvature` penalty in the mincurv objective must stay MEAN-based; a stiff sum-based
  term destabilizes L-BFGS-B numerical gradients. `--raceline-smooth-sigma` removes phantom
  per-vertex curvature spikes after resampling. `limit_curvature_spikes` may only touch isolated
  runs (<= 3 waypoints), and final curvature-limit validation must keep warning on violations.
- `trajectory_gui.py` — tkinter GUI over the same args (`NUMERIC_SPECS` must cover new params).
  It supports only `centerline` and `mincurv`. Keep `write_outputs` filtering runtime-only
  callables from the metadata JSON.
- `generate_adaptive_overlays.py` — batch orchestrator for the C++ local-planner side evaluator,
  black obstacle-to-wall map editing, the standard mincurv generator, outline fallback, and
  timestamp-labelled PNG/report output. Keep the default 142 waypoints x 11 lateral offsets equal
  to exactly 1,562 scenarios. Do not reimplement left/right planning rules in Python.
- `optimize_adaptive_parameters.py` — feasibility-first CMA-ES orchestration over the same C++
  evaluator. `safe_stop` count must rank before every secondary objective; among equally feasible
  candidates, maximize `minimum_avoidance_clearance_m`. Never let the optimizer lower its physical
  floor below vehicle half-width plus `hard_collision_margin_m`, and do not overwrite the runtime
  local-planner YAML automatically.
- `config/adaptive_cmaes.yaml` — current CMA-ES initial point and bounded search space. Keep ordered
  transition scales represented as a positive first value plus two positive gaps.
- `test_adaptive_overlay_generator.py` — signed filename, scenario-count, CSV conversion, and
  dense off-map regression tests for the adaptive batch pipeline.
- `test_gui_params.py` — compatibility regression for removed optimizer values in persisted YAML.
- `config/velocity_limits.csv` — speed-dependent net acceleration limits for the standard GUI/CLI.
