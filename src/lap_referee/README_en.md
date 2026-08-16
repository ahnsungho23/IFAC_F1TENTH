# lap_referee

> 🌐 [한국어](README.md) · **English**

A C++ node package that **referees and records a single run (rollout)** in the f1tenth gym simulator.
It judges collision/lap-completion/off-track/stuck/timeout, and writes the lap time and trajectory to files.
It is used when the ROS backend of `dl_speed_optimizer` automatically evaluates candidate speed profiles.

Detailed operation: [`docs/lap_referee_node.md`](docs/lap_referee_node.md).

## 1. Operating Principle

Since the bridge does not report collisions over a ROS topic, the result is reconstructed **using only observable signals**.

1. Compute the track length and the `s` of each point from the reference raceline CSV (`waypoints_csv`).
2. Subscribe to `odom`/`scan`/`drive`, update state at 50 Hz, and take the moment the speed exceeds the threshold as t0.
3. Accumulate the forward progress of the nearest waypoint index to obtain the traveled distance.
4. Termination: `lap_complete` (traveled distance ≥ track × `lap_fraction`) / `collision` (minimum LiDAR distance < threshold) /
   `off_track` (lateral error exceeded) / `stuck` (command is high but stopped) / `timeout` / `no_start`.
5. On termination, write the result atomically to a file (temp → rename), publish a 0-speed `/drive`, then terminate the process.

One process = one rollout. The orchestrator detects completion from process termination.

## 1.1 Detailed Pipeline

```
/ego_racecar/odom ─────────┐
  (nav_msgs/Odometry)       │     ┌─────────────────────────────┐
                             ├────▶│ lap_referee                  │
/scan ───────────────────────┤     │                             │
  (sensor_msgs/LaserScan)    │     │ Rollout judge: collision/   │
                             │     │ complete/off_track/stuck/    │
/drive ──────────────────────┘     │ timeout/no_start            │
  (AckermannDriveStamped)          └──────┬──────────────────────┘
                                          │
                                          ├──▶ result JSON + trace CSV (file)
                                          └──▶ /drive (0-speed stop, then exit)
```

One process = one rollout. dl_speed_optimizer uses this as eval backend.

## 2. Subscribed / Published Topics

| Direction | Topic | Type | Default |
|------|------|------|--------|
| Subscribe | `odom_topic` | `nav_msgs/Odometry` | `/ego_racecar/odom` |
| Subscribe | `scan_topic` | `sensor_msgs/LaserScan` | `/scan` (the gym bridge publishes the ego scan as `/scan`) |
| Subscribe | `drive_topic` | `ackermann_msgs/AckermannDriveStamped` | `/drive` |
| Publish | `drive_topic` | `ackermann_msgs/AckermannDriveStamped` | 0-speed stop on termination (optional) |

## 3. Main Parameters

Full set: [`config/lap_referee.yaml`](config/lap_referee.yaml).

| Parameter | Meaning | Default |
|----------|------|--------|
| `waypoints_csv` | reference raceline CSV (required) | `''` |
| `output_dir` / `output_prefix` | result file location / prefix | `/tmp/lap_referee` / `rollout` |
| `collision_scan_threshold` | minimum LiDAR for collision judgment [m] | `0.13` |
| `off_track_threshold` | lateral error for off-track judgment [m] | `1.5` |
| `stuck_speed_threshold` / `stuck_cmd_threshold` / `stuck_time_sec` | stuck judgment | `0.2`/`0.8`/`0.7` |
| `lap_fraction` / `max_episode_time_sec` | completion ratio / maximum time | `0.97` / `60.0` |

## 4. Output Files

- `<output_dir>/<prefix>_summary.json` — rollout summary (`terminated`, `collided`, `lap_time_s`, `crash_s`, etc.).
- `<output_dir>/<prefix>_trace.csv` — `t,x,y,yaw,v,cmd_v,cmd_steer,min_scan,nearest_idx,lat_err,s`.

The JSON schema is a contract with `dl_speed_optimizer` (`SimRunner._parse_outputs` in `dl_speed_opt/evaluators.py`).

## 5. Build and Run

```bash
cd ~/2026_IFAC
colcon build --packages-select lap_referee
source install/setup.zsh

# Standalone run (the launch must always pass waypoints_csv)
ros2 launch lap_referee lap_referee.launch.py \
  waypoints_csv:=$HOME/2026_IFAC/offline_trajectory_generator/output/ifac_track/global_waypoints.csv \
  output_dir:=/tmp/lap_referee output_prefix:=rollout
```

The full closed loop (sim + controller + referee) is automatically bundled and run by `dl_speed_optimizer`'s `--backend ros`.

## 6. References

- node-level rules: [`AGENTS.md`](AGENTS.md)
- detailed document: [`docs/lap_referee_node.md`](docs/lap_referee_node.md)
