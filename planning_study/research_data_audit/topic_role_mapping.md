# Topic role mapping

Schema: `rosbag_research_inventory/1`. Mapping uses exact names/types found in the 137 metadata files; the current planner contract is corroborated by `src/local_planning/src/local_planner_node.cpp` and its YAML. A count is the number of bags with at least one message.

| Role | Exact topic | Type | Bags | Authority note |
|---|---|---|---:|---|
| `confirmed_static_obstacles` | `/confirmed_static_obs` | `f110_msgs/msg/ObstacleArray` | 54 | metadata + current source/config |
| `historical_static_obstacles` | `/perception/obstacles` | `f110_msgs/msg/ObstacleArray` | 2 | metadata; historical/remap input, not current confirmed-only default |
| `historical_static_obstacles` | `/static_obs` | `f110_msgs/msg/ObstacleArray` | 59 | metadata; historical/remap input, not current confirmed-only default |
| `local_avoidance_path` | `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` | 67 | metadata + current source/config |
| `selected_local_path` | `/local_waypoints` | `f110_msgs/msg/WpntArray` | 86 | metadata + current source/config |
| `global_reference_waypoints` | `/global_waypoints` | `f110_msgs/msg/WpntArray` | 90 | metadata + current source/config |
| `localization_pose` | `/pf/pose/odom` | `nav_msgs/msg/Odometry` | 98 | metadata + kinematic_localization output contract |
| `frenet_odometry` | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | 73 | metadata + current source/config |
| `vehicle_odometry` | `/car_state/odom` | `nav_msgs/msg/Odometry` | 4 | metadata + current source/config |
| `vehicle_odometry` | `/odom` | `nav_msgs/msg/Odometry` | 87 | metadata + current source/config |
| `simulator_ground_truth_pose` | `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | 25 | metadata + simulator naming; GT interpretation is an explicit inference |
| `imu` | `/imu/data` | `sensor_msgs/msg/Imu` | 18 | metadata + current source/config |
| `imu` | `/sensors/imu` | `vesc_msgs/msg/VescImuStamped` | 20 | metadata role; current extractor uses the recorded standard raw IMU companion |
| `imu` | `/sensors/imu/raw` | `sensor_msgs/msg/Imu` | 87 | metadata + current source/config |
| `scan` | `/scan` | `sensor_msgs/msg/LaserScan` | 103 | metadata + current source/config |
| `scan` | `/scan_slow` | `sensor_msgs/msg/LaserScan` | 10 | metadata + current source/config |
| `scan` | `/slow_scan` | `sensor_msgs/msg/LaserScan` | 1 | metadata + current source/config |
| `tf` | `/tf` | `tf2_msgs/msg/TFMessage` | 97 | metadata + current source/config |
| `tf_static` | `/tf_static` | `tf2_msgs/msg/TFMessage` | 90 | metadata + current source/config |
| `particle_cloud` | `/pf/viz/particles` | `geometry_msgs/msg/PoseArray` | 29 | metadata + current source/config |
| `drive_command` | `/ackermann_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | 42 | metadata + current source/config |
| `drive_command` | `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | 93 | metadata + current source/config |
| `drive_command` | `/drive_autonomous` | `ackermann_msgs/msg/AckermannDriveStamped` | 88 | metadata + current source/config |
| `steering` | `/ackermann_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | 42 | metadata + current source/config |
| `steering` | `/commands/servo/position` | `std_msgs/msg/Float64` | 63 | metadata + current source/config |
| `steering` | `/sensors/servo_position_command` | `std_msgs/msg/Float64` | 35 | metadata + current source/config |

`/pf/viz/particles` is a `PoseArray`; metadata does not expose particle weights, so ESS is unavailable unless a future weighted message type is recorded. Odometry covariance is read from `/pf/pose/odom.pose.covariance`, not from a separate topic.
