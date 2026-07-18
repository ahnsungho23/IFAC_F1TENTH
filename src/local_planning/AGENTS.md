# AGENTS.md for local_planning

## 1. Package Overview
The `local_planning` package provides two distinct local planning nodes for F1TENTH autonomous racing:
1. **`local_planner_node`**: Occupancy grid map (`/map`) based obstacle checking and least-squares polynomial spline avoidance trajectory generator.
2. **`static_obstacle_avoidance_node`**: Comception LiDAR-based obstacle (`/obstacles`) checking, Frenet-frame 5th-order polynomial (Quintic Spline) approach & return segment generator with lateral acceleration-bounded speed profiles and cost-function based best path selection.

## 2. Key Rules & Conventions
- Written in modern C++17 for ROS 2 Humble.
- Uses Eigen3 for matrix equation solving (`colPivHouseholderQr`).
- All runtime settings are configured via `config/local_planning.yaml`.

## 3. Node Details & Interfaces

### A. `static_obstacle_avoidance_node`
- **Purpose**: Generates local static obstacle avoidance waypoint segments in Frenet frame using quintic polynomial interpolation (`QuinticPolynomial`), converging cleanly back to the global path (`d=0, d'=0, d''=0`), adjusting throttle by lateral acceleration limits (`max_lat_accel`), and dynamically determining whether avoidance is needed based on corridor blocking.
- **Subscribed Topics**:
  - `/global_waypoints` (`f110_msgs/msg/WpntArray`): Global reference waypoints
  - `/car_state/frenet/odom` (`nav_msgs/msg/Odometry`): Vehicle pose & velocity in Frenet coordinates
  - `/obstacles` (`f110_msgs/msg/ObstacleArray`): Detected obstacles from Comception module
- **Published Topics**:
  - `/planner/avoidance/otwpnts` (`f110_msgs/msg/OTWpntArray`): Local avoidance path segment for `wpnt_publisher`
  - `/local_planning/avoidance_path` (`nav_msgs/msg/Path`): RViz visualization path
  - `/local_planning/candidate_paths` (`visualization_msgs/msg/MarkerArray`): RViz visualization of evaluated candidate paths
- **Launch Command**: `ros2 launch local_planning static_obstacle_avoidance.launch.py`

### B. `local_planner_node`
- **Subscribed Topics**: `/global_waypoints`, `/map`, `/car_state/frenet/odom`
- **Published Topics**: `/avoid_waypoints` (`f110_msgs/msg/OTWpntArray`), `/local_waypoints`, `/local_planning/path`, `/local_path`, `/local_planning/markers`
- **Launch Command**: `ros2 launch local_planning local_planning.launch.py`
