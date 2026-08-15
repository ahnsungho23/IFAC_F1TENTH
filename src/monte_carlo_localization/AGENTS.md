# Package-level AGENTS.md for monte_carlo_localization (particle_filter_cpp)

This document defines package-specific developer rules and guidelines for `monte_carlo_localization` (ROS 2 package name: `particle_filter_cpp`).

## Package Purpose

`particle_filter_cpp` implements a Monte Carlo Localization (MCL) particle filter in C++ for 2D map-based localization of F1TENTH racecars. It subscribes to LiDAR scans, odometry, and global path waypoints, and publishes inferred vehicle poses and TF transforms (`map -> odom`).

## Message Policy

- Prefer standard ROS 2 messages (`sensor_msgs/msg/LaserScan`, `nav_msgs/msg/Odometry`, `geometry_msgs/msg/PoseWithCovarianceStamped`) for standard sensor/pose interfaces.
- Prefer `f110_msgs/msg/WpntArray` for global trajectory waypoints (`/global_waypoints`) used in auto-initialization.

## Key Features & Refactoring (2026-08-15)

- **MultiThreadedExecutor & Callback Groups**: 40 Hz MCL loop is isolated from map/visualization timers (`map_timer_` interval changed to 2000 ms).
- **ESS Gate Resampling**: Resampling occurs only when `ESS/N < ess_threshold` (default `0.5`). Sensor likelihoods accumulate across frames when not resampled.
- **Cluster Pose Estimation**: `expected_pose()` uses a primary cluster (radius `0.5m`, yaw tolerance `0.5rad`) around the highest weight particle to prevent bimodal split pose distortion.
- **Health Topic (`/pf/health`)**: Publishes JSON diagnostics at 1 Hz including cycle times, publish gaps, ESS ratio, particle dispersion ($\sigma_{pos}, \sigma_{yaw}$), and bimodality ratio.

## Parameter Policy & YAML Location

- Key configuration YAML: `config/mcl_config.yaml` (real/bag), `config/mcl_config_sim.yaml` (sim 전용, mod:=sim일 때 launch가 선택).
- `mcl_launch.py`는 파라미터 **값을 오버라이드하지 않는다** — 모드 배선(토픽/프레임/TF 플래그)과 설정 파일 선택만 담당. 튜닝값 변경은 반드시 YAML에서만 할 것 (2026-08-03, launch else 분기가 YAML을 묵살하던 배선 버그 제거).
- Key parameters:
  - `auto_init_from_waypoints` (bool, default: `true`): Automatically initializes particles around the first waypoint of `/global_waypoints`.
  - `ess_threshold` (double, default `0.5`): Threshold for ESS ratio to trigger resampling.
  - `cluster_radius` (double, default `0.5`): Position radius for primary cluster pose averaging (m).
  - `cluster_yaw_thres` (double, default `0.5`): Yaw tolerance for primary cluster pose averaging (rad).
  - `scan_topic` (string, default: `/scan`): LiDAR scan input topic.
  - `odom_topic` (string, default: `/odom`): Odometry input topic.
  - `publish_map_odom_tf` (bool, default: `true`): Controls publication of `map -> odom` TF.
  - `publish_extrapolation_sec` (double, default `0.0` in code; real YAML `0.09`, sim YAML `0.0`):
    forward-extrapolates the PUBLISHED `/pf/pose/odom` position along heading by `v × this`
    to compensate the measured ~90 ms end-to-end output lag.

## Launch Policy

- Launch file: `launch/mcl_launch.py`
- Command example: `ros2 launch particle_filter_cpp mcl_launch.py mod:=real map_name:=map`
- `use_sim_time` is an explicit launch argument. Keep it `false` for
  `f1tenth_gym_ros`, which has no `/clock`; bag playback defaults to `true`.
- Ensure `CMakeLists.txt` installs `launch`, `config`, `maps`, and documentation directories.

## Documentation Expectations

- Maintain package documentation under `docs/particle_filter.md` in Korean.
