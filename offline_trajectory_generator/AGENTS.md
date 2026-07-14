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
  Extraction robustness invariants (keep when refactoring): centerline candidates are ranked by
  ENCLOSED contour area (not arc length — noise scribbles are long but enclose nothing); skeleton
  pixels in corridors narrower than `min_track_width` are dropped; the Zhang-Suen fallback keeps
  the skeleton loop connected without opencv-contrib; map cleanup must NEVER erase measured walls
  (`cleanup_free_mask(occupied_mask=...)` stamps raw occupied pixels back after morphology so a
  big morph kernel cannot swallow a thin interior wall); `count_off_map_waypoints` checks densely
  (between-waypoint segments too) and must keep warning when the raceline leaves free space.
- `optimize_laptime.py` — `--optimizer laptime`: differentiable lap-time gradient descent.
  torch (CUDA) / MLX (Apple Metal) are OPTIONAL deps, imported lazily only when this optimizer
  is selected; the base pipeline must keep working with numpy/scipy alone. Both backends share
  the loss math through the `_Ops` adapter — change the model in `_lap_time_terms` only, never
  fork per-backend physics. Its differentiable speed model must stay consistent with
  `velocity_profile` (v_min/v_max clipping + accel/decel limits), otherwise the optimizer
  minimizes a different lap time than the one the tool reports.
  `--optimizer ai` (`optimize_lap_time_ai`, GUI "AI Optimize" button, `--ai-epochs`): per-epoch
  GD portfolio (per-restart smooth weights) -> exact rescoring -> ES polish, warm-restarting the
  incumbent each epoch. Its `evaluate` callback (built in `optimize_raceline`) MUST mirror the
  pipeline's post-processing (spike filter + optimizer_step and waypoint_step resamples) before
  calling `velocity_profile` — scoring the raw coarse line rewards sampling artifacts the final
  resample later exposes as curvature spikes, so the search would rank candidates by the wrong
  lap time (this exact bug shipped once: coarse-scored "12.5 s" lines were 14.8 s after resample).
  Steering-limit (`--max-curvature`) invariants: the speed model alone never rejects an
  undrivable kink (v_min floors it), so the kappa-excess penalty must stay in BOTH the
  differentiable loss (`_lap_time_terms`) and the ai `evaluate` callback; the mincurv objective's
  penalty must stay MEAN-based (a stiff sum-based term destabilizes L-BFGS-B's numerical
  gradients — measured); `--raceline-smooth-sigma` (final-resample smoothing) removes the phantom
  per-vertex curvature spikes of the piecewise-linear optimizer output and must be mirrored in
  the ai `evaluate`; `limit_curvature_spikes` may only touch ISOLATED runs (<= 3 wpts) — blending
  a sustained corner just moves the kink to the run boundary and amplifies it (6 -> 17 rad/m
  measured); the final validation warning (kappa_violations/max_abs_kappa) must keep firing when
  the delivered raceline exceeds the limit.
  ai search invariants: `_ai_inits` must stay ANCHORED on the mincurv warm start / incumbent
  (3/4 smooth perturbations around anchors, 1/4 random) — all-random inits converge to wavy
  local optima unrelated to the min-curvature line; the ES de-wiggle smoothing proposals must
  stay a POST-convergence pass (interleaving them during the random search greedily drags it
  into a smoother-but-slower basin, measured 14.3 s -> 15.3 s).
- `trajectory_gui.py` — tkinter GUI over the same args (`NUMERIC_SPECS` must cover new params).
  The GUI injects `args.progress_log` into the generation namespace; `optimize_raceline` routes
  optimizer logs through `getattr(args, "progress_log", print)` so long laptime/ai GPU runs show
  live progress in the status bar instead of a frozen "Generating...". Keep that wiring.
  Because of it, `vars(args)` can contain callables: `write_outputs`' metadata dump must keep
  filtering non-JSON-safe values (a raw `json.dumps(vars(args))` broke the GUI Save button once).
