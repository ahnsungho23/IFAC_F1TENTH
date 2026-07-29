# obstacle_detector

> [한국어](README.md) · English

A ROS 2 Jazzy C++ package that detects non-map obstacles from 2D LiDAR, tracks them in the
raceline Frenet frame, and separates stationary objects from the nearest dynamic opponent.

This package is **perception only**. It does not plan paths, generate avoidance/overtake
waypoints, or publish driving state.

## Data flow

```text
/scan + /global_waypoints + /map + ego odometry + TF
  → adaptive-breakpoint clustering
  → pre-tracking LiDAR-fragment merge
  → size, viewing-window, track-boundary, and occupancy-map filters
  → range-, sparsity-, and yaw-rate-adaptive measurement covariance
  → constant-velocity Frenet Kalman tracking
  → static/dynamic classification
  → same-layer object merge
  → visible-track Cartesian AABB union, centre, and enclosing radius
  → one-second accumulated perception diagnostics
  ├─ /static_obs
  ├─ /opp_obs
  └─ /perception/obstacles/markers
```

- Layer 1 removes walls and known map structure and is not published.
- Layer 2 publishes every confirmed non-map stationary object on `/static_obs`.
- Layer 3 publishes at most one nearest-ahead confirmed dynamic object on `/opp_obs`.

Both layer topics use `f110_msgs/msg/ObstacleArray` and are published on every scan, including
empty arrays.

Every visible object sets `has_cartesian=true` and provides its map-frame AABB, AABB centre, and
enclosing-circle radius. A predicted-only object keeps its Frenet state but sets
`has_cartesian=false`, because Kalman prediction does not move the last raw scan footprint.

The pre-tracking fragment merge keeps small scan fragments temporarily and joins them only when
their Cartesian AABBs and actual point sets are close and the merged object remains within
`max_obs_size`. It creates one detection/track; the later same-layer merge only consolidates
already tracked output objects.

Association retains a physical Frenet-distance hard gate, adds a detection-specific Mahalanobis
gate, and then performs the existing Frenet-distance-ordered greedy one-to-one assignment.
Mahalanobis distance is not used as the sorting cost, so high-uncertainty detections do not gain
priority merely from their larger covariance.

## Build

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-up-to obstacle_detector
source install/setup.zsh
```

## Run

```bash
# Real vehicle
ros2 launch obstacle_detector obstacle_detector.launch.py

# Simulator
ros2 launch obstacle_detector obstacle_detector.launch.py simulator:=true

# Optional RViz
ros2 launch obstacle_detector obstacle_detector.launch.py rviz:=true
```

If the live map contains baked-in obstacles, give Layer 1 an obstacle-free map:

```bash
ros2 launch obstacle_detector obstacle_detector.launch.py \
  simulator:=true \
  detector_map_yaml:=/absolute/path/to/clean_map.yaml
```

All runtime parameters are in
[`config/obstacle_detector.yaml`](config/obstacle_detector.yaml). See
[`docs/obstacle_detector_node.md`](docs/obstacle_detector_node.md) for the detailed pipeline and
[`docs/sim_test_commands.md`](docs/sim_test_commands.md) for the test procedure.

With `diagnostics_enable=true`, the node reports accumulated beam, clustering, Layer-1 rejection,
association, track-lifecycle, and classification statistics every `diagnostics_period_sec`.
Noise-filter and deskew statistics are intentionally absent because this detector performs neither
operation; an upstream scan preprocessor must report them.
