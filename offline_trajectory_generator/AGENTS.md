# AGENTS.md

This directory contains standalone, offline trajectory generation tools.
There are TWO independent generators here; know which one you are touching:

- the **C++ generator** (`src/`, `bin/generate_global_trajectory`, `trajectory_gui.py`,
  `gui_params.yaml`) — the default pipeline, described in the rest of this file;
- the **Forza generator** (`forza_*.py`, `forza_gui_params.yaml`, `config/forza/`,
  `vendor/`) — a pure-Python port of ForzaETH race_stack's planner. See the
  "Forza generator" section at the bottom before editing any `forza_*` file.

They share no source file, no parameter YAML and no output directory. Do not make
one import from the other.

Rules:
- Do not add ROS 2 runtime dependencies here.
- The engine must build with plain CMake (`cmake -B build && cmake --build build`)
  on both x86_64 and arm64/Jetson — no arch-specific compiler flags.
- Inputs are ROS map-server compatible SLAM map YAML/image files.
- Outputs should stay compatible with the existing `f110_msgs/Wpnt` field names when practical.
- Put user-facing operation docs in Korean.
- Prefer configurable CLI arguments over hard-coded map, speed, width, or smoothing values.
- When adding or changing generator parameters, update the CLI parser (`src/generate_main.cpp`),
  the GUI defaults/`NUMERIC_SPECS`/`build_cli_args`, the saved parameter YAML, and the Korean
  README in the same change.

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
- `CMakeLists.txt` — standalone (NOT a colcon package; the repo root `src/` build does not
  pick it up). The generator binary lands in `bin/` (gitignored) for GUI and CLI use.
- `third_party/LBFGSpp/` — vendored header-only L-BFGS-B (MIT).
- `trajectory_gui.py` — tkinter GUI. It runs `bin/generate_global_trajectory` as a subprocess
  into a temp work dir and renders the preview from the output files (`NUMERIC_SPECS` and
  `build_cli_args` must cover new params). "Save outputs" copies the work-dir files (and fixes
  metadata's `output_dir`). The saved `gui_params.yaml` may hold values from removed features
  (e.g. an old optimizer name); loading must sanitize such values instead of crashing.
- `config/velocity_limits.csv` — speed-dependent net acceleration limits for the GUI/CLI.

## Forza generator (pure Python, independent of everything above)

Ported from `edge_test@95843f7a`; runs with no ROS 2, no colcon and no CMake.
Operation doc (Korean): `docs/forza_generator.md`.

- `forza_trajectory_gui.py` — tkinter GUI **and** the full pipeline (map -> Lee
  thinning -> shortest closed contour -> Savitzky-Golay -> watershed track bounds
  -> `tph.iqp_handler` minimum-curvature QP -> `tph.calc_vel_profile`). Unlike
  `trajectory_gui.py` it spawns no subprocess. `--render-test <png>` runs the whole
  thing headless and writes the output bundle, which is the way to verify a change.
- `forza_common.py` — verbatim copy of the deleted `generate_global_trajectory.py`
  (map loading, CSV/JSON writers, `Trajectory`/`MapInfo`/`GenerationResult`), plus
  the marker builders. Parts of it are dead code for the Forza path (the `laptime`
  and shortest-path optimizers); that is deliberate, so the copy stays comparable
  with the original. The C++ pipeline never imports it.
- `forza_preview.py` — `draw_polyline` / `render_preview_rgb` / `rgb_to_photoimage`,
  byte-identical copies of `trajectory_gui.py`'s. Duplicated ON PURPOSE: the two
  GUIs must share no file. `draw_polyline` is not optional — `render_preview_rgb`
  calls it. Do not "deduplicate" these back into `trajectory_gui.py`.
- `forza_gui_params.yaml` — Forza parameters. Separate from `gui_params.yaml`.
  Relative `optimizer_config_dir` resolves from THIS directory, not the workspace
  (`forza_trajectory_gui.py` has no `REPO_ROOT`; keep it that way so the tool runs
  outside a checkout).
- `config/forza/` — `racecar_f110.ini` + `veh_dyn_info/`. `ggv.csv` is an untuned
  stub (all 12.0) and `longitudinal_accel_scale` 3.7 multiplies it; the speed model
  is NOT trustworthy for real driving. Do not cite Forza lap times against C++ ones.
- `vendor/` — TUM `trajectory_planning_helpers` (19 of 31 modules) and
  `helper_funcs_glob` (`prep_track`, `interp_track`), LGPL-3.0, pinned to
  `b1b38ecf`. Read `vendor/README.md` before touching it: the ONLY upstream
  modification is the trimmed `__init__.py`, and it cannot be deleted because
  `create_raceline`/`iqp_handler`/`spline_approximation`/`prep_track` reach for
  `tph.<submodule>`. `helper_funcs_glob/` intentionally has no `__init__.py`.
  Adding a module means adding it to `__init__.py` and to `vendor/README.md`.

Invariants:

- `write_outputs()` in `forza_common.py` MUST emit the three marker arrays. The
  original hard-coded them empty, and `global_trajectory_publisher_node` answers an
  empty array with a `DELETEALL` — generation "succeeds" while RViz goes blank and
  the previous map's markers are wiped. Marker geometry/style is a 1:1 port of
  `src/trajectory_core.cpp:1125~1230`; keep the two in sync and keep the style as
  fixed constants (parameterizing them would duplicate the node's YAML).
  `global_traj_markers_sp` stays empty: Forza builds no shortest path and the JSON
  reader never looks at the `*_sp` keys.
- `expected_centerline_length` is compared against the RAW skeleton contour
  perimeter, not the final `s_max`. It must be retuned per map (and after any
  `occupancy_grid_threshold` / `filter_kernel_size` change) with the procedure in
  `docs/forza_generator.md` 4.1, or generation fails outright.
- Default output is `output/<map>/forza/`, never `output/<map>/` — that one belongs
  to the C++ generator and is what the publisher node reads by default.
- Separation check (the `forza_*` files must import nothing from the C++ GUI):
  `grep -nE "^\s*(from|import)\s+trajectory_gui\b" forza_*.py` and
  `grep -n "REPO_ROOT" forza_trajectory_gui.py` must both return nothing. Match on
  the import statement, not the bare name — the module docstring mentions
  `trajectory_gui.py` in prose on purpose.
- Regression check for any `forza_*` change: run `--render-test` before and after
  and diff `global_waypoints.csv`, `centerline.csv` and the preview PNG. They are
  bit-identical to `edge_test@95843f7a`'s generator today; only the marker arrays
  in `global_waypoints.json` differ, and that is the intended difference.
