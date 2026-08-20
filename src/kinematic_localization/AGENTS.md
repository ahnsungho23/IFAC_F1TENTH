# Package-level AGENTS.md for kinematic_localization

This document defines package-specific rules for `kinematic_localization`. The root
`/home/parkm/2026_IFAC/AGENTS.md` applies in full; entries below only add package detail.

## Launch and parameter loading

- Both launch files must resolve their YAML to an absolute installed-share path and fail before
  node creation when the file is absent or its symlink is broken. Do not pass an unchecked
  `PathJoinSubstitution` as the parameter-file entry: `launch_ros` can silently omit a missing
  file and start the node with C++ defaults.
- `config_schema_version` is mandatory in both YAML files. Both C++ nodes must compare it with
  their compiled expectation and fail fast on missing, stale, or wrong YAML. Keep the two C++
  constants and both YAML values equal; `test/test_config_guard.py` locks this contract.
- Keep the startup parameter-summary logs. They are the rosbag-visible evidence of the effective
  gate, smoothing, source-downsample, voxel, and mapping settings used in a field session.

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
  mandatory `config_schema_version`,
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
  fixed by the source-only `source_voxel_size 0.25` core patch instead
  (docs §8). `max_num_iterations 30`, `max_range 30.0`, `min_range 0.1`,
  `deskew true`, adaptive threshold/regularization on.
- `config/mapping.yaml`: mandatory `config_schema_version`, the same KICP block, and bag/output
  paths for `mapping_node`.
- Launch files must not hard-code tuning values; they only select the YAML and pass
  runtime selections (`map_name`, `bag_path`, `output_path`, `use_sim_time`).

## Launch / Run

- Localization: `ros2 launch kinematic_localization kinematic_localization.launch.py map_name:=<name>`
  then publish `/initialpose` once (RViz 2D Pose Estimate or `ros2 topic pub --once`).
- Mapping: `ros2 launch kinematic_localization mapping.launch.py bag_path:=<bag> output_path:=<file>`

## Documentation

Korean node documentation: `docs/kinematic_localization.md`. Keep it in sync when topics,
parameters, or the run procedure change.
