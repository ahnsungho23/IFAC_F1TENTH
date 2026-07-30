# Package-level AGENTS.md for monte_carlo_localization (particle_filter_cpp)

This document defines package-specific developer rules and guidelines for `monte_carlo_localization` (ROS 2 package name: `particle_filter_cpp`).

## Package Purpose

`particle_filter_cpp` implements a Monte Carlo Localization (MCL) particle filter in C++ for 2D map-based localization of F1TENTH racecars. It subscribes to LiDAR scans, odometry, and global path waypoints, and publishes inferred vehicle poses and TF transforms (`map -> odom`).

## Message Policy

- Prefer standard ROS 2 messages (`sensor_msgs/msg/LaserScan`, `nav_msgs/msg/Odometry`, `geometry_msgs/msg/PoseWithCovarianceStamped`) for standard sensor/pose interfaces.
- Prefer `f110_msgs/msg/WpntArray` for global trajectory waypoints (`/global_waypoints`) used in auto-initialization.

## Parameter Policy & YAML Location

- Key configuration YAML: `config/mcl_config.yaml`
- Key parameters:
  - `auto_init_from_waypoints` (bool, default: `true`): Automatically initializes particles around the first waypoint of `/global_waypoints`.
  - `scan_topic` (string, default: `/scan`): LiDAR scan input topic.
  - `odom_topic` (string, default: `/odom`): Odometry input topic.
  - `publish_map_odom_tf` (bool, default: `true`): Controls publication of `map -> odom` TF.

## Launch Policy

- Launch file: `launch/mcl_launch.py`
- Command example: `ros2 launch particle_filter_cpp mcl_launch.py mod:=real map_name:=ifac_track`
- Ensure `CMakeLists.txt` installs `launch`, `config`, `maps`, and documentation directories.

## Documentation Expectations

- Maintain package documentation under `docs/particle_filter.md` in Korean.
