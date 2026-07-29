# Offline Trajectory Generator

> 🌐 [한국어](README.md) · **English**

This tool takes the ROS map YAML/image produced after SLAM as input and generates a global waypoint file once, without running ROS 2.

## 1. Purpose

- Input: `map.yaml` + `map.pgm` or `map.png`
- Processing: free-space extraction, centerline generation, waypoint resampling, curvature-based speed profile calculation
- Output:
  - `global_waypoints.json`
  - `global_waypoints.csv`
  - `centerline.csv`
  - `metadata.json`
  - Optional: `debug_overlay.png`

`global_waypoints.json` uses the same structure as the existing `f110_msgs/Wpnt` field names.

## 2. How to Run

Run from the repository root.

To check while adjusting parameters with the GUI, use the following command.

```bash
python3 offline_trajectory_generator/trajectory_gui.py
```

On the first run, a map YAML selection window opens. For example, you can select `monte_carlo_localization/maps/slam_map.yaml`.

When creating a trajectory for use in f1sim, you must select the same map YAML as `map_path` in f1sim's `config/sim.yaml`. The current f1sim default map is `src/monte_carlo_localization/maps/fuck_f1.yaml`, and the `new_map_con` default CSV also uses `src/new_map_con/maps/fuck_f1.csv` in this map's coordinate frame.

If you want to open a specific map from the start, pass it as follows.

```bash
python3 offline_trajectory_generator/trajectory_gui.py \
  --map-yaml monte_carlo_localization/maps/slam_map.yaml
```

The left side of the screen shows the map path, save location, and trajectory parameters. The numeric parameters are grouped under the `Sampling`, `Track & safety`, `Speed profile`, `Map cleanup`, `Centerline`, `Min-curvature`, and `Straightening` subheadings so the item you want is easy to find. The right side shows the centerline and RT lane together over the map image. When you change a slider or checkbox, it automatically recomputes after a short moment and the overlay is refreshed.

Pressing the `AI Optimize` button switches the optimizer to `ai` and automatically searches for the fastest line by repeating a GPU gradient-descent portfolio + exact rescoring + evolution-strategy (ES) polish for `AI epochs` rounds (Optimizer tab; see the `--optimizer ai` description in section 3 for details). torch or MLX must be installed. The optimizer progress log (`[laptime] iter ...`, `[ai] epoch ...`) streams into the status bar during the run, so a large iteration count taking tens of seconds is not a hang.

Each numeric parameter shows its current value in the input box next to its name, and you can also type a value directly into that box. The value is applied when you press `Enter` in the input box or move to another box. A directly entered value is rounded to the parameter's unit and clamped to the minimum if it is below the minimum, but a value larger than the slider's maximum range can still be used as is. In that case the slider stays at the end of its quick-adjustment range, while the actually applied value is kept in the input box and the saved YAML. If a parameter is at the bottom of the screen, move there with the mouse wheel or scrollbar over the left panel.

When you press `Save` in the GUI, the trajectory currently shown on screen is saved as `global_waypoints.json`, `global_waypoints.csv`, `centerline.csv`, and `metadata.json`.

GUI parameter values are automatically saved to the following YAML immediately upon change, without any separate operation.

```text
offline_trajectory_generator/gui_params.yaml
```

On the next run, the values saved in this YAML are loaded again. If you want to use a different settings file, pass `--params-yaml`.

```bash
python3 offline_trajectory_generator/trajectory_gui.py \
  --params-yaml /tmp/my_trajectory_gui_params.yaml
```

To generate only the files directly via the CLI, use the following command.

```bash
python3 offline_trajectory_generator/generate_global_trajectory.py \
  --map-yaml monte_carlo_localization/maps/slam_map.yaml \
  --output-dir /tmp/offline_traj_slam_map \
  --debug-image
```

If you omit the output directory, the default is as follows.

```text
offline_trajectory_generator/output/<map_yaml_file_name>/
```

## 3. Main Options

- `--waypoint-step`: The final waypoint spacing. The default is `0.1` m.
- `--optimizer-step`: The internal centerline/RT lane calculation spacing. This spacing determines the number of variables in the minimum-curvature optimization, so setting it too small (e.g. `0.05`) inflates the variable count into the hundreds or thousands, which makes the optimizer hit its iteration limit and run slowly. The default is `0.2` m, and `0.15`–`0.25` is usually enough.
- `--safety-width`: The width including the vehicle width and safety margin.
- `--boundary-margin`: The additional distance to keep away from the wall.
- `--max-speed`: The waypoint speed upper limit. The default is `4.0` m/s.
- `--min-speed`: The waypoint speed lower limit. The default is `1.0` m/s.
- `--max-lateral-accel`: The lateral acceleration limit used for curvature-based speed calculation.
- `--max-curvature`: The vehicle steering limit as a maximum path curvature [rad/m]
  (= `tan(max_steer)/wheelbase`, ~`1.2` for F1TENTH). The speed model only slows down for sharp
  bends — it does not know the car *cannot steer through them at all* — so without this limit the
  optimizers cut corners with kinks (turn radius of a few cm) the car can never track. It enters
  the mincurv/laptime/ai losses as a penalty, and the final waypoints are validated against it
  with a warning on violation (GUI status-bar ⚠ plus a `max |κ|` metric). `0` disables.
- `--raceline-smooth-sigma`: Gaussian smoothing (in samples, default `1.0`) applied to the
  raceline right after the final waypoint resample. The optimizers emit piecewise-linear lines
  with vertices at `optimizer-step` spacing; linearly resampling them at the finer
  `waypoint-step` creates a phantom curvature spike at every vertex, causing steering-limit
  violations and over-braking. ~1 sample of smoothing removes the kinks without changing the
  geometry (measured on oct_28: lap time 12.2 s → 10.5 s once the phantom-spike braking is gone).
  `0` disables.
- `--median-kernel`: The median filter size for removing small dot noise in the SLAM map. Used with an odd value.
- `--morph-kernel`: The pixel kernel size used for map cleanup. Increase it for noisy maps, but if it is too large, narrow passages may disappear.
- `--morph-open-iterations`: The number of iterations for removing small free-space dot noise.
- `--morph-close-iterations`: The number of iterations for filling in broken free-space.
- `--skeleton-prune-iterations`: The number of times to iteratively trim dead-end branches of the skeleton.
- `--min-skeleton-component-area`: The minimum number of pixels for discarding skeleton fragments that are too small.
- `--min-track-width`: Drops skeleton pixels where the free space is narrower than this [m] (default `0.3`).
  Blocks centerline candidates inside scan-noise regions (scratches, wall leaks) that the car could
  never physically drive through. `0` disables.
- `--min-centerline-angle`: The minimum progress angle for removing pin shapes where the path turns back at one point.
- `--spike-filter-iterations`: The number of times to repeat the pin removal filter.
- `--reverse`: Flips the driving direction of the generated waypoints to the opposite.
- `--debug-image`: Saves a PNG with the centerline and global trajectory drawn over the map image.
- `--optimizer centerline`: The default. Works stably on SLAM maps.
- `--optimizer mincurv`: Attempts scipy-based minimum curvature correction. May require tuning depending on map quality.
- `--optimizer laptime`: **Gradient-descends the lap time itself as a loss function on the GPU**
  (`optimize_laptime.py`). Instead of indirectly reducing time by hand-tuning curvature/smooth/length
  weights like mincurv, it directly minimizes a differentiable lap-time model (curvature-limited speed
  `v=√(a_lat/|κ|)` plus accel/decel limits). Track bounds are hard-guaranteed by a sigmoid
  reparameterization. `--laptime-restarts` candidate lines are optimized **simultaneously** on the GPU
  (multi-start) and the fastest wins; one candidate is warm-started from the mincurv solution
  (disable with `--laptime-no-warm-start`). Requires torch (CUDA) or Apple MLX (Metal);
  `--laptime-backend auto` picks whichever is installed. Measured: on the fuck_f1 map (198 wpts),
  24 lines × 3000 steps take 3.4 s on Apple M-series, lap time 6.70 s → 5.90 s (−12%).
- `--laptime-iters` / `--laptime-restarts` / `--laptime-lr` / `--laptime-smooth-weight`:
  iterations / parallel candidates / learning rate (3-stage decay) / lateral-offset smoothness weight
  of the laptime optimizer. More candidates raise the chance of escaping local minima at almost no
  extra cost on a GPU.
- `--optimizer ai`: **automatic search that alternates several techniques per epoch to cut the lap
  time further** (same as the GUI's `AI Optimize` button). The search is **anchored on the mincurv
  solution**: the mincurv warm-start line is scored as the baseline, and most (3/4) of the initial
  candidates are smooth perturbations around the mincurv/incumbent lines with only 1/4 fully
  random for global exploration. One epoch consists of ① a GPU gradient-descent portfolio — each
  candidate in the restart batch gets a **different smoothness weight** (x0.25/x1/x4), exploring
  several regularization settings in one batched run — ② **exact rescoring** — every candidate is
  re-scored with the true `velocity_profile` lap time plus the steering-limit penalty ("score")
  after the same post-processing the pipeline applies (spike filter + resampling), not the
  differentiable surrogate, to pick the real winner — ③ an **evolution-strategy (ES) polish** — a
  (1+λ) ES that directly minimizes the non-differentiable exact score, followed by a de-wiggle
  sweep (flattening leftover wiggles that don't buy lap time). The next epoch
  warm-restarts from the incumbent (basin hopping). Uses the same `--laptime-*` settings. The
  exact rescoring includes the steering-limit penalty, so **undrivable candidates lose the
  ranking** — measured (fuck_f1, same settings): the laptime winner keeps a κ=2.2 rad/m corner
  (warning printed) while the ai winner has max |κ|=1.19 with zero steering-limit violations.
- `--ai-epochs`: number of ai search epochs. Default `3`. Each epoch reshuffles the random restart
  seeds, so more epochs give more chances to escape local minima at proportionally more runtime.
- `--max-optimizer-iter`: The maximum number of iterations for the minimum-curvature optimization (L-BFGS-B). The default is `200`. With `optimizer-step` in a sensible range the optimizer normally converges within this limit. It only reaches the limit on hard maps or large variable counts, and even then the best raceline found so far is used directly, so no warning is printed. If the warning shows up often, raising `optimizer-step` is the proper fix.
- `--no-straighten-straights`: Turns off the post-processing that corrects straight candidate sections into straight lines.
- `--straight-kappa-threshold`: Absolute curvature smaller than this value is regarded as a straight candidate. The default is `0.2` rad/m.
- `--straight-min-length`: The minimum section length to be recognized as a straight candidate.
- `--straight-clearance-margin`: The additional wall clearance distance required for straight correction validation.
- `--straight-blend-length`: The length over which the original path and the straight line are smoothly blended at both ends of the straight correction section.

The trinary gray unknown region of the ROS map is excluded from the drivable area by default. Turn on the GUI's `Unknown as free` only when you want to use even the gray unknown as free-space.

Straight correction pulls the internal waypoints of low-curvature sections onto the endpoint straight line, then applies it only when the entire straight line satisfies the free-space and clearance conditions according to the distance transform. Therefore, when the curvature of a straight section fluctuates due to SLAM noise, the speed profile is set higher. If the path is pulled too much, lower `straight_kappa_threshold`; if the straightening is insufficient, raise the value.

## 4. Generation Procedure

1. Save the map YAML and image files with SLAM.
2. Create `global_waypoints.json` with the run command above.
3. Open `debug_overlay.png` to check that the path follows the center of the track.
4. If the direction is reversed, add `--reverse` to the same command and regenerate.
5. If the speed is too high or too low, adjust `--max-speed`, `--min-speed`, and `--max-lateral-accel`.

When using the GUI, you can check steps 3-5 directly on screen and then just press `Save`.

## 5. Output File Description

`global_waypoints.csv` columns:

```text
id,s,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,d_left,d_right
```

The CSV format uses the same 10-column structure as `src/new_map_con/maps/fuck_f1.csv`. `d_left`/`d_right` are the distances [m] from each waypoint to the left/right track boundary, computed according to `--width-mode`:

- `hybrid` (default, recommended): directional robust raycast measures left/right boundaries independently. It requires at least 2 consecutive wall pixels so single-pixel noise/holes are ignored, and if a ray leaks through a gap and reaches the max distance it falls back to the distance-transform nearest-wall value. Left/right asymmetry is preserved when the raceline hugs the inside of a curve.
- `distance`: distance-transform based. Fast, but returns a single nearest-wall value so `d_left == d_right` (no left/right distinction).
- `raycast`: directional raycast only (with the robust gate); no gap-leak correction.

`d_left`/`d_right` feed directly into the `local_planning` avoidance safety-margin computation and the `new_map_con` path-boundary check, so avoidance logic that needs left/right asymmetry must use `hybrid`. `--max-width-distance` is the raycast upper bound; set it to the track width (3.0 m recommended for indoor F1TENTH tracks). The remaining ROS waypoint fields `d_m`, `s_m`, etc. are kept inside `global_waypoints.json`.

The CSV's `x_m` and `y_m` are map frame coordinates with the `resolution` and `origin` of the selected ROS map YAML applied. If the map and path are misaligned in RViz, first check whether the map YAML put into the generator and the map YAML loaded by f1sim/map server are the same file.

`global_waypoints.json` main fields:

- `global_traj_wpnts_iqp`: The waypoint array to be used for actual driving
- `centerline_waypoints`: The extracted centerline waypoint array
- `est_lap_time`: The expected lap time based on the speed profile
- `map_info_str`: A summary of the input map and generation settings

## 6. Dependencies

In the current environment, `numpy`, `opencv`, `PyYAML`, and `scipy` are used.

```bash
python3 -m pip install -r offline_trajectory_generator/requirements.txt
```

ROS 2, `rclpy`, `quadprog`, and `skimage` are not required.

Only `--optimizer laptime` and `--optimizer ai` additionally need an ML framework (either one):

- `torch` — Ubuntu / car PC. Uses CUDA automatically when available (`--laptime-device cuda` to force).
- `mlx` — Apple Silicon macOS. Uses the Metal GPU automatically.

The centerline extraction is robust against map noise: ① skeleton pixels in corridors narrower than
`--min-track-width` are treated as noise and removed, ② among candidate loops the closed contour with
the **largest enclosed area** (= the actual track loop) is selected — not the longest one, so long
thin noise scribbles cannot win — ③ when opencv contrib (`ximgproc.thinning`) is unavailable, a
built-in Zhang-Suen thinning keeps the skeleton loop connected without extra installs, and ④ **map
cleanup (median/morph close) can never erase measured walls** — after morphology the raw occupied
pixels are stamped back in (except sub-speckle dot noise), so a large morph kernel cannot swallow a
thin interior wall and let the raceline cut straight through it. The final raceline is validated
densely (~2 px spacing, including between-waypoint segments); if it leaves free space a warning with
the off-map waypoint count is printed (the GUI status bar shows a ⚠ as well).

The GUI uses `tkinter`, a Python standard library. If `tkinter` is missing on Ubuntu, install the following package.

```bash
sudo apt install python3-tk
```
