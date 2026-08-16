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
  → complete Cartesian AABB-to-Frenet footprint projection
  → size, viewing-window, track-boundary, and occupancy-map filters
  → range-, sparsity-, and yaw-rate-adaptive measurement covariance
  → constant-velocity Frenet Kalman association and map-frame Kalman motion estimate
  → Raw / Tentative / Confirmed track existence
  → Unknown / Static / Dynamic motion classification
  → same-layer object merge
  → matching visible-track Cartesian AABB union and Frenet bounds
  → one-second accumulated perception diagnostics
  ├─ /static_obs
  ├─ /confirmed_static_obs
  ├─ /opp_obs
  ├─ /static_obs/markers
  └─ /opp_obs/markers
```

- Layer 1 removes walls and known map structure and is not published.
- Layer 2 publishes every existence-confirmed Unknown or Static object on `/static_obs`.
- `/confirmed_static_obs` publishes only existence-confirmed Static objects.
- Layer 3 publishes at most one nearest-ahead confirmed dynamic object on `/opp_obs`, including an
  `is_interfering` decision based on ego-corridor overlap and current/predicted rear gap. The
  operational defaults enter at 1.0 m and release the same opponent ID at 1.2 m.

All three layer views use `f110_msgs/msg/ObstacleArray` and are published on every scan, including
empty arrays.

Existence confirmation uses three associated measurements inside the latest five scans. Motion is
estimated by a supplemental map-frame `[x,vx,y,vy]` Kalman filter. The classifier votes on the
velocity statistic `Tv=vᵀPv⁻¹v` and map-position RMS without adding prediction-only frames. With
the defaults, three Dynamic evidence samples in the latest five move the same ID to `/opp_obs`;
Static requires ten Static evidence samples in the latest fifteen plus persistent map position.

The published `Obstacle.id` is a physical-object ID separated from the internal Kalman-track
instance. If the primary Frenet/Kalman association breaks, an unmatched track is reconnected when
its AABB and the detection still belong to the same spatial cluster, regardless of motion state.
Split tracks in one scan share that public ID. When a confirmed object first becomes statistically
Static, its measured Frenet footprint and map-frame AABB are frozen as a stable identity anchor;
later viewpoint drift or false Dynamic evidence cannot move it. Objects that never become Static
fall back to their last measured footprint. The anchor and ID are retained for 30 seconds after
retirement, and either a matching Frenet envelope or map-frame AABB can recover the ID.
Prediction-only Kalman positions never move this dormant identity. The ID therefore remains
stable when `Unknown`, `Static`, and `Dynamic` transitions move the object among `/static_obs`,
`/confirmed_static_obs`, and `/opp_obs`.
Periodic republishes of identical `/global_waypoints` preserve the CLCS, tracks, and ID memory.
They are reset only when the `x/y/s/d_left/d_right` reference geometry actually changes.

Every visible object publishes authoritative `s_start/s_end/d_right/d_left` bounds projected from
its complete map-frame AABB. It also sets `has_cartesian=true` and provides that same footprint's
AABB centre and enclosing-circle radius. A predicted-only object keeps the last measured Frenet
extent around its predicted centre but sets `has_cartesian=false`, because Kalman prediction does
not move the last raw scan footprint.

`/static_obs/markers` and `/opp_obs/markers` convert the final arrays'
`s_start/s_end/d_right/d_left` envelopes back into map-frame boundary lines. The static marker
topic therefore mirrors the exact geometry consumed by the local planner. Predicted-only objects
remain visible with lower alpha.

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
