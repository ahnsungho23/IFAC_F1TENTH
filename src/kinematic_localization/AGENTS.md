# Package-level AGENTS.md for kinematic_localization

This document defines package-specific rules for `kinematic_localization`. The root
`/home/parkm/2026_IFAC/AGENTS.md` applies in full; entries below only add package detail.

## Launch / parameter loading (2026-08-19)

**Both launch files resolve `config/*.yaml` with `get_package_share_directory` and raise if the
file is missing.** Do not go back to `PathJoinSubstitution` + `FindPackageShare` for the
`parameters=[...]` entry: when that path does not exist, `launch_ros` **silently drops it** and
the node starts with every value at its code default. On 2026-08-19 the Jetson checkout had
`config/kinematic_localization.yaml` deleted; `ros2 launch` started with no error and the whole
test session ran with `gate_enable=false`, `smoothing_alpha_rot=-1` (→ 0.5) and
`source_voxel_size=-1` (→ 1.0 m), which cut the ICP correspondences from 54 to 15 and doubled
the pose noise. See `docs/rosbag_2026-08-19_2023_analysis.md`.

The constructor logs a **parameter summary** and warns when the values look like code defaults.
Keep both. When adding a parameter that materially changes registration quality, add it to that
summary line.

Quick field check (expected values in parentheses):

```bash
ros2 param get /kinematic_localization gate_enable          # true
ros2 param get /kinematic_localization smoothing_alpha_rot  # 0.12
ros2 topic echo /kinematic_localization/diagnostics --once   # num_source_points 40~60
```

## Package Purpose

Kinematic-ICP based map localization. Scans are deskewed and aligned (ICP with a wheel-odometry
prior) against a **frozen** voxel map (`.kissmap`), so the estimate cannot diverge the way pure
odometry does (e.g. in reverse driving). Interface is drop-in compatible with
`particle_filter_cpp` (MCL): it publishes `/pf/pose/odom` and the `map -> odom` TF.

- `localization_node` (C++): online localization, waits for `/initialpose` like MCL.
  With `slam_mode: true` the same node runs as online SLAM (freeze_local_map = false,
  no frozen map, auto-init at identity on the first scan, accumulated map saved via the
  `~/save_map` Trigger service or on shutdown, published on `~/map_points`).
  Output smoothing (complementary filter: wheel-odom prediction + ICP correction with a
  velocity-adaptive alpha, `smoothing_*` params) is applied to the published pose and
  `map -> odom` TF; SLAM map accumulation always uses the raw ICP pose. Rotation is
  pulled with its own fixed gain (`smoothing_alpha_rot`, YAML 0.12; < 0 = follow the
  translation alpha): the ICP yaw measurement is ~10x noisier than the wheel-odom yaw
  prediction while cornering (docs §8), so a uniform alpha turns corners into yaw jitter. The node also
  serves `/map` (`nav_msgs/OccupancyGrid`, transient_local) rasterized from the map
  points, so RViz 2D Pose Estimate works without a separate map_server. A non-empty
  `map_name` that fails to load is FATAL (fail fast — no silent pure-odometry); the
  cached grid is re-published every `map_publish_period_sec` so late/restarted
  subscribers always converge.
  Robustness features (kicp_robustness_plan, see `docs/kinematic_localization.md` §7):
  always-on NaN guard (core + node rollback), scan-gap watchdog (odom dead reckoning,
  `watchdog_*`), registration diagnostics on `~/diagnostics` (`diagnostics_*`),
  map-based pose validity check (`pose_check_*`), Mahalanobis gate (`gate_*`,
  default OFF until bag-measured), soft lateral DoF (`lateral_*`, default OFF).
  Watchdog/pose-check/gate/lateral are forced off in `slam_mode`. The node runs on a
  single-threaded executor by design — the shared state is lock-free; do not move
  callbacks into other callback groups or a MultiThreadedExecutor.
- `mapping_node` (C++): offline `.kissmap` builder from a rosbag (message-count based; bag
  send/receive timestamps are never used — sqlite3 bags report `send_timestamp == 0`).
- `scripts/pgm_to_kissmap.py`: converts a map_server occupancy map (pgm/png + yaml) to
  `.kissmap`. Point density is set by `--downsample` (2D grid step, default 0.1 m);
  `--voxel-size` is header metadata only (a mismatch with the node's `voxel_size`
  just WARNs at load). Keep `--downsample` <= ~0.14 m so a wall line stays under
  `max_points_per_voxel` (20) per 1 m voxel.
- `scripts/wall_rate.py`: wall-alignment metric used to validate localization quality.
- `scripts/compare_trajectory.py`: TUM trajectory comparison helper (from the KICP evaluation).

## Vendored Core

The ICP core lives in `third_party/kinematic-icp` (MIT, PRBonn kinematic-icp + kiss-icp).
kiss-icp v1.2.0, Sophus 1.22.10 (with `sophus.patch` pre-applied), and tsl-robin-map v1.2.1
are vendored in `third_party/kiss-icp|sophus|robin-map` — `kiss-icp.cmake` points
`FETCHCONTENT_SOURCE_DIR_*` at them, so builds need **no internet** (Jetson-safe). The
vendored dirs carry `COLCON_IGNORE` so colcon does not treat them as packages. If a vendored
dir is deleted, the build falls back to downloading the upstream tarballs. Local patches are
marked with `Localization patch (2026_IFAC)` comments:

- `Config::freeze_local_map` (KinematicICP.hpp/.cpp) — frozen-map localization mode.
- `USE_SYSTEM_SOPHUS/OFF`, `USE_SYSTEM_TSL-ROBIN-MAP/OFF` defaults (CMake).
- **NaN guards** (Registration.cpp `ComputeRobotMotion`): `correspondences.empty()` →
  return the odom prediction before the regularization lambda, and `break` after the
  in-loop re-association. Without them a 0-correspondence frame produces 0/0 = NaN and
  `Sophus::SE3d::exp(NaN)` **aborts the process** (SOPHUS_ENSURE) — this was the
  "/initialpose kills the node" bug. Never remove; they are deliberately toggle-free.
- **Registration diagnostics** (Registration.hpp `Diagnostics`, `diagnostics()`,
  `KinematicICP::registrationDiagnostics()`): per-frame inlier/residual/JTJ snapshot,
  reset at the top of every `ComputeRobotMotion` call. `residual_rms`/`JTJ` are one
  iteration stale by design (recorded at final-iteration entry) — consumers must widen
  trust via `converged`/`final_dx_norm` instead of re-associating.
- **Soft lateral DoF** (Registration.cpp `lateral_dof_enable_` branch, Config
  `lateral_dof_enable|lateral_regularization_scale|lateral_regularization_floor_tau2`):
  3-DoF perturbation with `beta_lat = max(floor/tau², scale·beta)`. The 2-DoF arc path
  is preserved verbatim and must stay bit-identical when the toggle is off.
- **Source-only downsample** (KinematicICP.hpp `Config::source_voxel_size`,
  KinematicICP.cpp `RegisterFrame` Voxelize call): > 0 replaces `voxel_size` in the
  source 0.5x/1.5x cascade only; map hash grid, correspondence search reach and
  adaptive-threshold floor keep using `voxel_size` so the convergence basin is
  unchanged. <= 0 (default) = upstream behavior.

Do not upgrade the vendored core without re-applying these patches. Sophus and
tsl-robin-map are **not** available system-wide (no sudo/apt) — never flip those options ON.

## Chassis-tilt (z-axis) compensation

- `utils::LevelScan` is the ONE implementation; `localization_node` and `mapping_node` both
  call it. Never fork it — a map built with a different leveling than the runtime scans is
  worse than no leveling at all.
- Roll must be derived from `roll_gradient × a_lat` with `a_lat = v·ω` taken from the wheel
  odometry KICP already consumes. NEVER estimate it from the accelerometer: gravity and
  centripetal acceleration share the lateral axis, and a naive complementary filter on the
  2026-08-19 23:38 bag reported a fake -13.8° ± 10.4° roll while driving (-25° in corners).
  The gradient itself is measured by regressing IMU `ay`[g] on `a_c`[g]: the slope is -1.000
  only if the body does not lean, and the deficit IS the roll (measured -0.731 -> 0.0274
  rad/(m/s²) = 1.57°/(m/s²) = 9.4° at 6 m/s²). See docs §11-4.
- `tilt_compensation_enable` must stay the SAME in `kinematic_localization.yaml` and
  `mapping.yaml`. The frozen map carries the same roll distortion (same LiDAR, same chassis),
  so enabling only the runtime side makes the compensation fight the map — measured on the
  23:38 bag: cornering lateral bias improves 6.2 -> 4.8 cm but the map-fit residual degrades
  0.0766 -> 0.0804 m at |a_lat| 6-12. Default is false for exactly this reason; flip it only
  after re-recording the map with it on and re-running the docs §11-6 table.
- Level the points, then FLATTEN z to 0 before registration — the map is a z=0 planar scan,
  so handing KISS a tilted cloud makes correspondences worse, not better. z survives only as
  the `tilt_max_point_height_m` out-of-plane reject test.

## .kissmap Format (defined by this package)

Binary: magic `KISSMAP1` (8 bytes), `double voxel_size`, `double max_range`, `uint64 count`,
then `double x,y,z` per point (map frame). Maps live in `maps/<map_name>.kissmap` and are
selected with the `map_name` parameter (an absolute path is used as-is).

## Message / Topic Policy

- Subscribed: `/scan` (`sensor_msgs/LaserScan`, sensor-data QoS), `/tf`, `/tf_static`,
  `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`, map frame — same UX as MCL).
- Published: `/pf/pose/odom` (`nav_msgs/Odometry`, reliable depth 10, frame `map`, child
  `base_link`), TF `map -> odom` computed as `T_map_base * T_odom_base^-1` (same convention
  as MCL in `particle_filter.cpp::publish_tf`), `/map` (`nav_msgs/OccupancyGrid`,
  transient_local depth 1, rasterized from the map points).
- SLAM mode only: publishes `~/map_points` (`sensor_msgs/PointCloud2`, transient_local) and
  serves `~/save_map` (`std_srvs/Trigger`, saves the accumulated map to `map_output_file`;
  the map is also auto-saved on clean shutdown).
- Diagnostics: `~/diagnostics` (`diagnostic_msgs/DiagnosticArray`, `diagnostics_enable`) —
  registration quality (inlier_ratio, residual_rms, tau, beta, convergence, gate state,
  dead_reckoning_sec). Standard type on purpose (root policy: no custom msg needed).
- No new custom message types; standard `sensor_msgs`/`nav_msgs`/`geometry_msgs`/
  `diagnostic_msgs` only.

## Parameter Policy

- `config/kinematic_localization.yaml`: all topics, frames, KICP tuning, covariances,
  the output-smoothing block (`smoothing_enable`, `smoothing_alpha`,
  `smoothing_alpha_gain`, `smoothing_velocity_full_mps`, `smoothing_alpha_max` —
  starting values for real-car tuning), the built-in `/map` server block (`map_topic`,
  `map_grid_resolution`, `map_point_dilation_m`), the SLAM-mode block (`slam_mode`,
  `map_output_file`, `map_publish_period_sec`), and the robustness blocks
  (`watchdog_*`, `diagnostics_*`, `pose_check_*`/`permissible_radius_m`, `gate_*`,
  `lateral_*`). Defaults policy: watchdog/diagnostics/pose-check ON (safe direction
  only). Gate and lateral DoF are ON since 2026-08-18 (docs §9): the cornering
  slip-hiding symptom was field-measured, §3 nominal distributions were measured
  (residual_rms p99 0.21, inlier p1 1.00), and the gate at chi2 11.34 rejected
  0/8401 nominal frames. Rollback: lateral_dof_enable:=false gate_enable:=false
  (then gate_chi2 9.21 for 2-DoF).
  Verified tuning: `voxel_size 1.0` — do NOT shrink it: it also sets the
  correspondence search reach and the adaptive-threshold floor, and 0.25
  collapsed the convergence basin (run_0818 replay diverged meters, inlier
  ratio 0.26). The indoor-scan source collapse (9-25 points at voxel 1.0) is
  fixed by the source-only `source_voxel_size 0.25` core patch instead (docs §8). `max_num_iterations 30`,
  `max_range 12.0` (30.0 -> 12.0 on 2026-08-19: it scales `sigma_odom` and therefore tau, so a 30 m reach on a ~17 m
  track inflated tau 3.31 -> 2.31 and stretched the heading-error tail; raise it again for a larger map — docs §10),
  `min_range 0.1`,
  `deskew true`, adaptive threshold/regularization on.
- `config/mapping.yaml`: same KICP block plus bag/output paths for `mapping_node`.
- Launch files must not hard-code tuning values; they only select the YAML and pass
  runtime selections (`map_name`, `bag_path`, `output_path`, `use_sim_time`).

## Launch / Run

- Localization: `ros2 launch kinematic_localization kinematic_localization.launch.py map_name:=<name>`
  then publish `/initialpose` once (RViz 2D Pose Estimate or `ros2 topic pub --once`).
- Mapping: `ros2 launch kinematic_localization mapping.launch.py bag_path:=<bag> output_path:=<file>`

## Documentation

Korean node documentation: `docs/kinematic_localization.md`. Keep it in sync when topics,
parameters, or the run procedure change.

## Replay A/B harness — two traps that invalidate results

Both were hit on 2026-08-19 and cost 8 replays. Check them before trusting any replay number.

1. **Seed `/initialpose` AFTER playback starts**, roughly 2 s into bag time. Publishing it first leaves the odom buffer
   empty, so `OdomAt()` has no prior for the first scan and the pose jumps several metres — every downstream metric is
   then garbage.
2. **Match the map to the bag.** `map.kissmap` (2026-08-15, 451 pts) sits a median 5.33 m away from the
   `rosbag2_2026_08_19-*` trajectories; `map_0818_track.kissmap` (2970 pts) is 0.70 m. A wrong map does not fail
   loudly — it just produces plausible-looking, meaningless numbers (`ifac_track` gives p50 8.33 deg on the same bag).

Also note `run_real.sh` defaults to `F1_MAP_NAME=map`, i.e. the stale 08-15 map. Verify the map the car actually loaded
with `grep "Loaded frozen map" ~/.ros/log/<session>/...` before drawing conclusions from live data.
