# new_map_con

> 🌐 [한국어](README.md) · **English**

A waypoint-following controller package. The `map_controller` (C++) node follows global/local waypoints with
pure-pursuit and publishes `/drive`. The **`opponent_simulator`** node simulates a virtual opponent vehicle for
`obstacle_detector` testing by publishing `/opponent_racecar/odom`. It supports both the vehicle and simulator topic profiles.

For a detailed explanation of the node operation, see [`docs/map_controller_node.md`](docs/map_controller_node.md).

## 1. Operating Principle

1. At startup it reads `global_waypoints_csv` (default `maps/fuck_f1.csv`, 8 columns) and publishes it latched to `/global_waypoints`.
2. On every control cycle (`control_rate_hz`) it finds the nearest waypoint to the current pose and computes the
   pure-pursuit steering toward the point ahead by a speed-based lookahead distance.
3. The target speed commands the waypoint `vx_mps` (at the speed-lookahead point), corrected by lateral error and curvature.
4. If `use_local_waypoints` is enabled and `/local_waypoints` is fresh, it uses that; otherwise it falls back to global.
5. The steering is scaled by acceleration/speed and limited by rate-of-change and maximum angle.

## 1.1 Detailed Pipeline

```
/pf/pose/odom ──┐
  (or /ego_racecar/odom)
                │
/local_waypoints ──┐
  (f110_msgs/WpntArray) │
                │      │         ┌─────────────────────────────┐
/odom ──────────┼──────┼────────▶│ map_controller              │
                │      │         │ (new_map_con)               │
/state ─────────┼──────┼────────▶│                             │
                ▼      ▼         │  pure-pursuit + speed ctrl  │
                ┌──────┐         │                             │
                │ pose │         └──────┬──────────────────────┘
                │+speed│                │
                └──────┘                ├──▶ /drive (AckermannDriveStamped)
                                        ├──▶ /global_waypoints (WpntArray, latched)
                                        └──▶ steering/lookahead markers (viz)
```

At startup the node loads the raceline CSV and publishes it as a latched `/global_waypoints` topic. This topic is consumed by global_planning, wpnt_publisher, and obstacle_detector.

## 2. Subscribed / Published Topics

The `simulator` parameter switches the topic profile (the topics below are the default-profile topics).

| Direction | Topic (sim/vehicle) | Message type | Description |
|------|------------------|-------------|------|
| Subscribe | `/local_waypoints` | `f110_msgs/WpntArray` | Local path (preferred if present) |
| Subscribe | `/ego_racecar/odom` / `/pf/pose/odom` | `nav_msgs/Odometry` | pose |
| Subscribe | `/ego_racecar/odom` / `/odom` | `nav_msgs/Odometry` | speed |
| Subscribe | `/state` | `std_msgs/String` | State machine (optional) |
| Subscribe | (off) / `/sensors/imu/raw` | `sensor_msgs/Imu` | Acceleration (steering scale, optional) |
| Publish | `/drive` | `ackermann_msgs/AckermannDriveStamped` | Drive command |
| Publish | `/global_waypoints` | `f110_msgs/WpntArray` | Global path (latched) |
| Publish | `steering`, `lookahead_point`, `my_waypoints`, `l1_distance` | `visualization_msgs/*`, `geometry_msgs/Point` | Debug markers |

## 3. Main Parameters

Full list and defaults: [`config/config.yaml`](config/config.yaml). The node declares safe defaults.

| Parameter | Meaning | Default |
|----------|------|--------|
| `global_waypoints_csv` | Path of the raceline CSV to follow | `maps/fuck_f1.csv` |
| `simulator` | Use the simulator topic profile | `false` |
| `control_rate_hz` | Control cycle | `40.0` |
| `min/max_lookahead_distance`, `lookahead_gain`, `lookahead_speed_gain` | Lookahead computation | `1.5`/`5.0`/`0.5`/`0.3` |
| `wheelbase`, `max_steering_angle`, `max_steering_delta` | Steering geometry/limits | `0.33`/`0.42`/`0.4` |
| `lateral_error_coeff`, `curvature_scale` | Speed correction | `1.0`/`0.8` |
| `use_local_waypoints`, `fallback_to_global_waypoints` | Path selection | `true`/`true` |

## 4. Build and Run

```bash
cd ~/2026_IFAC
colcon build --packages-select new_map_con
source install/setup.zsh

# Vehicle (default profile)
ros2 launch new_map_con new_map_con.launch.py

# Simulator profile (first run f1sim in another terminal)
ros2 launch new_map_con new_map_con.launch.py simulator:=true
```

Run with a different raceline CSV (e.g. dl_speed_optimizer output):

```bash
ros2 run new_map_con map_controller --ros-args \
  --params-file src/new_map_con/config/config.yaml \
  -p simulator:=true \
  -p global_waypoints_csv:=<optimized CSV absolute path> \
  -p package_resource_root:=src/new_map_con
```

## 5. opponent_simulator (Virtual Opponent)

The `opponent_simulator` node simulates a virtual opponent vehicle that follows `/global_waypoints`.
For `obstacle_detector` testing, it drives at a slower speed (default 0.8x) than ego and publishes `/opponent_racecar/odom`.

### Usage

```bash
# Default (0.8x speed, start 5m ahead)
ros2 launch new_map_con opponent_simulator.launch.py

# 0.7x speed, start 10m ahead
ros2 launch new_map_con opponent_simulator.launch.py speed_scale:=0.7 start_offset:=10.0

# Disable
ros2 launch new_map_con opponent_simulator.launch.py enabled:=false
```

### Main Parameters

| Parameter | Meaning | Default |
|----------|------|--------|
| `speed_scale` | Waypoint speed multiplier (0.8 = 80% speed) | `0.8` |
| `start_offset_m` | Initial arc-length offset [m] (ahead of ego) | `5.0` |
| `odom_topic` | Odometry topic to publish | `/opponent_racecar/odom` |
| `publish_tf` | Publish TF (map → opponent_base_link) | `true` |
| `enabled` | Enable the node | `true` |

### Use with obstacle_detector

```bash
# Terminal 1: Simulator (f1sim or real vehicle)
f1sim

# Terminal 2: new_map_con (ego control)
ros2 launch new_map_con new_map_con.launch.py simulator:=true

# Terminal 3: Virtual opponent
ros2 launch new_map_con opponent_simulator.launch.py speed_scale:=0.8 start_offset:=5.0

# Terminal 4: obstacle_detector (detect opponent)
ros2 launch obstacle_detector obstacle_detector.launch.py simulator:=true
```

The opponent circulates along the raceline, and `obstacle_detector` detects it via LiDAR and publishes it on `/opp_obs`.

## 6. References

- node-level rules: [`AGENTS.md`](AGENTS.md)
- Operating-principle document: [`docs/map_controller_node.md`](docs/map_controller_node.md)
- The raceline this controller follows is generated by `offline_trajectory_generator`/`dl_speed_optimizer`.
