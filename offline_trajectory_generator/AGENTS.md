# AGENTS.md

This directory contains a standalone, offline trajectory generation tool.
The generation engine is C++ (no ROS dependencies); only the tkinter GUI is Python.

Rules:
- Do not add ROS 2 runtime dependencies here.
- The engine must build with plain CMake (`cmake -B build && cmake --build build`)
  on both x86_64 and arm64/Jetson — no arch-specific compiler flags.
- Inputs are ROS map-server compatible SLAM map YAML/image files.
- Outputs should stay compatible with the existing `f110_msgs/Wpnt` field names when practical.
- Put user-facing operation docs in Korean.
- Prefer configurable CLI arguments over hard-coded map, speed, width, or smoothing values.
- When adding or changing generator parameters, update the CLI parser (`src/generate_main.cpp`),
  the gui_params reader (`src/regenerate_main.cpp`), the GUI defaults/`NUMERIC_SPECS`/
  `build_cli_args`, the saved parameter YAML, and the Korean README in the same change.

## Layout

- `src/trajectory_core.{hpp,cpp}` — map -> centerline -> raceline -> speed profile pipeline
  (C++ port of the former `generate_global_trajectory.py`; behavior-verified against it:
  identical centerline to machine precision, identical widths, same gate decisions).
  Optimizers: `centerline` (keep the skeleton line), `mincurv` (L-BFGS-B minimum curvature via
  vendored header-only `third_party/LBFGSpp` with numerical gradients, like scipy did), and
  `d_ratio` (fixed-ratio lateral offset; see `docs/proposal_d_ratio_raceline.md`).
  The speed profile reads `config/velocity_limits.csv` with columns
  `[speed_mps, max_accel_mps2, max_decel_mps2, max_lateral_accel_mps2]`; keep the CLI, GUI,
  and saved YAML on the same table semantics.
  Pixel rounding MUST stay half-to-even (`py_round`/`std::nearbyint`) — half-away rounding
  shifts raycast wall hits by one map pixel at exact .5 boundaries.
  Extraction robustness invariants (keep when refactoring): centerline candidates are ranked by
  ENCLOSED contour area (not arc length — noise scribbles are long but enclose nothing); skeleton
  pixels in corridors narrower than `min_track_width` are dropped; the Zhang-Suen fallback
  (compiled when OpenCV lacks ximgproc) keeps the skeleton loop connected; map cleanup must NEVER
  erase measured walls (`cleanup_free_mask` stamps raw occupied pixels back after morphology so a
  big morph kernel cannot swallow a thin interior wall); `count_off_map_waypoints` checks densely
  (between-waypoint segments too) and must keep warning when the raceline leaves free space.
  Steering-limit (`--max-curvature`) invariants: the speed model alone never rejects an
  undrivable kink (v_min floors it), so the kappa-excess penalty must stay in the mincurv
  objective, and it must stay MEAN-based (a stiff sum-based term destabilizes L-BFGS-B's
  numerical gradients — measured); `--raceline-smooth-sigma` (final-resample smoothing) removes
  the phantom per-vertex curvature spikes of the piecewise-linear optimizer output;
  `limit_curvature_spikes` may only touch ISOLATED runs (<= 3 wpts) — blending a sustained
  corner just moves the kink to the run boundary and amplifies it (6 -> 17 rad/m measured);
  the final validation warning (kappa_violations/max_abs_kappa) must keep firing when the
  delivered raceline exceeds the limit.
  `d_ratio` invariants: positive ratio moves toward `d_right` (the OPPOSITE sign of the
  mincurv alpha / Frenet d convention — user spec, do not "fix"); the ratio scales the usable
  half-width `max(d_side - clearance, 0)`, never the raw wall distance; optional smoothing
  applies to the alpha array and MUST be followed by a clip back to the RAW per-point corridor
  (smoothing the width arrays instead inflates narrow sections and overshoots the wall);
  the fold guard (cap |alpha| below the local curvature radius when moving toward the curvature
  center) must stay LAST and must only ever shrink |alpha| — without it a hairpin inside-offset
  folds into a self-loop (observed at d_ratio=-0.5 on map.yaml; widening boundary_margin does
  NOT help, it is a curvature problem). `report_min_clearance` (dense distance-transform audit
  of the final raceline) must keep running for every optimizer — off-map counting alone cannot
  see car-half-width wall clipping.
- `src/generate_main.cpp` — CLI (`bin/generate_global_trajectory`). Same flags as the old
  Python argparse (`--waypoint-step`, `--optimizer`, ...). Writes `off_map_wpnts` /
  `kappa_violations` / `max_abs_kappa` into metadata.json (the GUI status bar reads them).
  Sets `Args::emit_markers = true`: this is the ONLY path that bakes RViz MarkerArrays into
  global_waypoints.json. `global_trajectory_publisher_node` publishes them verbatim and builds
  nothing at runtime, so marker geometry/style changes must be made here and the map regenerated.
- `src/regenerate_main.cpp` — headless map_creator driver (`bin/regenerate_obstacle_map`).
  Leaves `Args::emit_markers` at `false` on purpose — the in-race regeneration is a pure
  trajectory job and writes no visualization. Consequence: after the lap-2 reload the
  republisher node emits `DELETEALL` and RViz shows no raceline/trackbound lines. That is
  intended; do not "fix" it by enabling emit_markers here without an explicit decision.
  Loads gui_params.yaml (GUI defaults + overrides, sanitizes a removed optimizer name to
  mincurv), applies the physical swap gates from
  `learning_adaptive_globalpath/MAP_CREATOR_PROPOSAL.md` 3.4, writes gate_report.json.
  Exit codes: 0 gates passed, 1 gate failure / generation error, 2 bad invocation.
  map_creator invokes it directly (its `python_executable` param is empty by default).
- `CMakeLists.txt` — standalone (NOT a colcon package; the repo root `src/` build does not
  pick it up). Binaries land in `bin/` (gitignored) — the GUI and map_creator expect them there.
- `third_party/LBFGSpp/` — vendored header-only L-BFGS-B (MIT).
- `trajectory_gui.py` — tkinter GUI. It runs `bin/generate_global_trajectory` as a subprocess
  into a temp work dir and renders the preview from the output files (`NUMERIC_SPECS` and
  `build_cli_args` must cover new params). "Save outputs" copies the work-dir files (and fixes
  metadata's `output_dir`). The saved `gui_params.yaml` may hold values from removed features
  (e.g. an old optimizer name); loading must sanitize such values instead of crashing.
- `config/velocity_limits.csv` — speed-dependent net acceleration limits for the GUI/CLI.
