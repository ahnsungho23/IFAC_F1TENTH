# Package-level AGENTS.md for monte_carlo_localization (particle_filter_cpp)

This document defines package-specific developer rules and guidelines for `monte_carlo_localization` (ROS 2 package name: `particle_filter_cpp`).

## Package Purpose

`particle_filter_cpp` implements a Monte Carlo Localization (MCL) particle filter in C++ for 2D map-based localization of F1TENTH racecars. It subscribes to LiDAR scans, odometry, and global path waypoints, and publishes inferred vehicle poses and TF transforms (`map -> odom`).

## Message Policy

- Prefer standard ROS 2 messages (`sensor_msgs/msg/LaserScan`, `nav_msgs/msg/Odometry`, `geometry_msgs/msg/PoseWithCovarianceStamped`) for standard sensor/pose interfaces.
- Prefer `f110_msgs/msg/WpntArray` for global trajectory waypoints (`/global_waypoints`) used in auto-initialization.

## Parameter Policy & YAML Location

- Key configuration YAML: `config/mcl_config.yaml` (real/bag), `config/mcl_config_sim.yaml` (sim 전용, mod:=sim일 때 launch가 선택).
- `mcl_launch.py`는 파라미터 **값을 오버라이드하지 않는다** — 모드 배선(토픽/프레임/TF 플래그)과 설정 파일 선택만 담당. 튜닝값 변경은 반드시 YAML에서만 할 것 (2026-08-03, launch else 분기가 YAML을 묵살하던 배선 버그 제거).
- Key parameters:
  - `auto_init_from_waypoints` (bool, default: `true`): Automatically initializes particles around the first waypoint of `/global_waypoints`.
  - `scan_topic` (string, default: `/scan`): LiDAR scan input topic.
  - `odom_topic` (string, default: `/odom`): Odometry input topic.
  - `publish_map_odom_tf` (bool, default: `true`): Controls publication of `map -> odom` TF.
  - `publish_extrapolation_sec` (double, default `0.0` in code; real YAML `0.09`, sim YAML `0.0`):
    forward-extrapolates the PUBLISHED `/pf/pose/odom` position along heading by `v × this`
    to compensate the measured ~90 ms end-to-end output lag (2026-08-14 diagnosis: corner
    p95 0.101→0.000 with 90 ms; yaw has NO lag so only translation is shifted; filter/EKF/TF
    state untouched; clamped to [0, 0.15]). ⚠️ Keep the sim value at 0.0 — CMA lockstep
    bit-determinism depends on it, and sim has no hardware pipeline lag to compensate.

## Launch Policy

- Launch file: `launch/mcl_launch.py`
- Command example: `ros2 launch particle_filter_cpp mcl_launch.py mod:=real map_name:=map`
- `use_sim_time` is an explicit launch argument. Keep it `false` for
  `f1tenth_gym_ros`, which has no `/clock`; bag playback defaults to `true`.
- Ensure `CMakeLists.txt` installs `launch`, `config`, `maps`, and documentation directories.

## Documentation Expectations

- Maintain package documentation under `docs/particle_filter.md` in Korean.
