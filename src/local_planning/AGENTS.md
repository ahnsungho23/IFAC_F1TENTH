# AGENTS.md for local_planning

## 1. Package Purpose
- Provides real-time obstacle interference checking against global waypoints and computes a least-squares polynomial spline avoidance trajectory.
- Publishes the avoidance spline as `/avoid_waypoints` (OTWpntArray). `wpnt_publisher` subscribes to it and merges it onto the global line into `/local_waypoints` (→ control); `state_machine` uses the same topic for avoid decisions. Standalone `/local_waypoints` publishing is disabled by default (`publish_standalone_local: false`), routing control delivery solely through `wpnt_publisher`.

## 2. Key Rules & Conventions
- Written in modern C++17 for ROS 2 Humble.
- Uses Eigen3 (`Eigen::colPivHouseholderQr`) for robust least-squares polynomial solving ($A^T A c = A^T b$).
- Follows parameter rules: all runtime settings are loaded from `config/local_planning.yaml` via `launch/local_planning.launch.py`.

## 3. Interfaces
- Subscribes: `/global_waypoints`, `/map`, `/car_state/frenet/odom`.
- Publishes: `/avoid_waypoints`, `/local_waypoints`, `/local_planning/path`, `/local_planning/markers`.
