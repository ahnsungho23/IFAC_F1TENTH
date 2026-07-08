# AGENTS.md for local_planning

## 1. Package Purpose
- Provides real-time obstacle interference checking against global waypoints and computes a least-squares polynomial spline avoidance trajectory.
- Interoperates seamlessly with `wpnt_publisher` by publishing to `/planner/avoidance/otwpnts`.

## 2. Key Rules & Conventions
- Written in modern C++17 for ROS 2 Humble.
- Uses Eigen3 (`Eigen::colPivHouseholderQr`) for robust least-squares polynomial solving ($A^T A c = A^T b$).
- Follows parameter rules: all runtime settings are loaded from `config/local_planning.yaml` via `launch/local_planning.launch.py`.

## 3. Interfaces
- Subscribes: `/global_waypoints`, `/map`, `/car_state/frenet/odom`.
- Publishes: `/planner/avoidance/otwpnts`, `/local_waypoints`, `/local_planning/path`, `/local_planning/markers`.
