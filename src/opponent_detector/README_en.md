# opponent_detector

> 🌐 [한국어](README.md) · **English**

A C++ node package that uses **2D LiDAR as its primary measurement** to detect and track a dynamic
opponent (no camera) **and plans a committed local overtaking path around it**. Optional IMU/odom
motion compensation deskews the scan before the node estimates the opponent's Frenet
position/velocity and publishes a **local spline overtaking line** (covering
only the overtaking segment, not the whole track) as an `OTWpntArray`. Detection and the overtake
planner are merged into one node, and every tunable — spline parameters included — lives in
a **single YAML** (`config/opponent_detector.yaml`). The overtaking line flows
`wpnt_publisher` → `/local_waypoints` → `new_map_con`, which the controller follows automatically.

It adapts the opponent-detection idea of [`2603.27207v1.pdf`](2603.27207v1.pdf) (Cihlar et al.,
*Autonomous overtaking trajectory optimization…*) to this camera-less repo. That paper uses a
depth camera + YOLO to **select which LiDAR cluster is the opponent**; we have no camera, so that
selection role is replaced by **relative velocity in the raceline (Frenet) frame** — static
structure stands still in that frame, the opponent moves.

Detailed operation: [`docs/opponent_detector_node.md`](docs/opponent_detector_node.md) (Korean).
Sim/harness test command cheat-sheet: [`docs/sim_test_commands.md`](docs/sim_test_commands.md) (Korean).

## 1. Operating Principle

The static/dynamic split happens in the Frenet frame, where ego motion is already removed via the
ego pose.

1. Deskew each `/scan` beam to the scan-start pose using IMU yaw rate, with odometry as fallback.
   Translation compensation and raw-noise filtering are optional until validated.
2. **Adaptive Breakpoint** clustering splits points using a range-dependent threshold, then merges
   nearby fragments only while the combined box remains obstacle-sized.
3. Transform deskewed points into the map frame via TF and fit AABB boxes.
4. **Map + track-boundary filter** — drop points outside the drivable corridor (`d_left/d_right`
   from `/global_waypoints`) and clusters sitting on known static structure (**walls**) in the SLAM `/map`.
5. Project each center to **Frenet (s,d)** using **`global_planning`'s CLCS converter**
   (`ClcsFrenetConverter`, vendored CommonRoad CLCS); drop points outside the projection domain (far
   from the track). The lightweight `FrenetProjector` is kept only for track-boundary
   (`d_left/d_right`) lookup and s-wrap, which CLCS does not provide.
6. Associate within a hard Euclidean safety gate using **Mahalanobis distance**, then apply a
   **constant-velocity Kalman filter** (state `[s, vs, d, vd]`) whose measurement covariance grows
   with range, sparsity, and yaw rate.
7. **Static/dynamic classification relative to the static-field velocity.** Walls (removed by the
   map filter) and stationary obstacles share one *static-field velocity* (≈0 in the map frame). Each
   track is judged by its speed **relative to that reference** — "same velocity as the walls → static,
   clearly different → dynamic opponent." The reference is the mean of the slow tracks (below
   `static_ref_gate`), which cancels common ego-localization drift and excludes the opponent.
   (Hysteresis + relative-`vs<reset` backstop; a `std` positional classifier is also a toggle.)
8. Publish tracked obstacles as `ObstacleArray` (stationary obstacles as `is_static=true`), and the
   dynamic opponent as Frenet `ProjOppTraj`. Perception repeatedly re-detects and publishes static
   obstacles; `ttl_static` only bridges brief sensor misses. Long-term memory and avoidance
   decisions belong to the local planner.
9. **Overtake planner (state machine + PCHIP spline)** — an `Idle → Committed → Cooldown` state
   machine decides whether an overtaking maneuver exists at all.
   - **Commit gates (ALL must hold):** the dynamic opponent is ahead inside
     `[trigger_min_ds, trigger_max_ds]` and blocks the ego corridor (`avoid_block_margin`); the
     **relative speed suffices to pass** (`ot_min_rel_vel`, predicted catch time ≤
     `ot_max_catch_time`); **no sharp corner** anywhere on the maneuver (raceline
     `|κ| ≤ ot_max_kappa`); and lateral room exists at the **predicted pass location** — the apex
     is placed where the pass will actually happen, not at the opponent's current position.
   - **Path shape:** starts at the CURRENT ego `(s,d)` (fine even off the raceline) → ramps to the
     apex offset → holds it through the pass zone → merges back onto the global raceline, as a
     **shape-preserving PCHIP spline** `d(s)` (no overshoot). The path covers **only the local
     overtaking segment**, never the whole track. Both the spline slope `|dd/ds|` (`ot_max_d_slope`)
     and its curvature `|d2d/ds2|` (the steering angle → anti-spin, `ot_max_path_kappa`) are limited
     (ramps/apex sized to them), and the speed is **curvature-limited** (`v = min(raceline_vx,
     √(a_lat/κ))`) then **longitudinally smoothed** (`ot_max_long_accel`, forward+backward) so the
     car BRAKES BEFORE a tight section, and **blended at both boundaries** (`ot_speed_blend_s`) so
     the handoff has no hard accel/brake.
   - **Continuous validity while COMMITTED (the opponent keeps moving):** completion fires only
     when the opponent was actually OBSERVED behind the ego (a lost track never counts as
     "complete" — losing it before the pass aborts). While still approaching (not yet alongside)
     ALL commit gates are re-run every cycle; if the environment becomes unsuitable the maneuver
     side-switches, else **aborts back to the global raceline** (also when the opponent leaves
     the ego corridor). Opponent intrudes on the committed path (`path_intrusion_margin`) → side
     switch, else abort; relative speed collapses (`ot_abort_rel_vel` hysteresis) → abort (follow
     instead); opponent drifts past `replan_ds/dd_threshold` → replan from the current ego pose.
     Once alongside, feasibility aborts are suppressed (dropping to global would steer into the
     opponent).
   - **Publish policy:** the OT is published only while COMMITTED. On completion/abort ONE empty
     OT is published (so `wpnt_publisher` falls back to global) and then the topic goes **silent**
     (re-commit possible after `ot_cooldown_s`). Mirrored as `nav_msgs/Path`
     (`/planner/avoidance/path`, **green** in RViz).
   - **Trailing (follow) mode:** when the opponent blocks the corridor but an overtake is NOT
     possible (gates reject / abort cooldown), the planner publishes a decelerate-and-follow local
     path (`ot_line="trail"`) instead of going silent — it tracks the raceline and brakes ahead of
     time (fwd/bwd accel pass) to match the opponent speed at `ot_trail_gap` behind it. Ends with
     one empty OT when the opponent clears/is passed; upgrades seamlessly to a commit when an
     overtake becomes feasible.
   - **Runway extension (anti trail-lock):** trailing tightly at matched speed puts the predicted
     catch point too close for any slope/curvature-limited ramp, which used to reject every commit
     and trap the planner in trailing. The apex is now pushed out to the minimum feasible runway —
     the ego **swings out first, then passes** — and while still laterally un-clear inside
     `ot_trail_gap` the speed profile is capped at the opponent speed so the ego never closes on a
     still-centered opponent mid-ramp.
   - **Weird-path / crash recovery guards:** the committed path is also monitored against the
     EGO — lateral deviation beyond `ot_ego_dev_replan` replans from the current pose, beyond 2x
     it aborts (the controller must never chase a line the car is no longer on). When the ego
     CLCS projection stays failed for `ego_pose_grace_s` (e.g. pushed off-track by a crash), the
     active path is cleared with ONE empty OT and the planner goes silent (never plans from a
     frozen pre-crash pose). The trail return ramp is never clipped below the curvature limit —
     the path extent is extended instead. Downstream, `wpnt_publisher` expires a silent OT feed
     after `ot_timeout_sec` and falls back to global.

Replacing the camera's "opponent-cluster selection" with relative velocity is the core of this node.
**Stationary obstacles (non-wall, e.g. cones) are also car-sized (≤0.5×0.5 m)**, so size cannot tell
them from the opponent — only velocity does (`max_obs_size` is 0.8 so the 0.707 m diagonal passes).

### Inter-node Data Flow

```
                        ┌─────── /scan (LaserScan)
                        │
                        ├─────── /global_waypoints (WpntArray, latched)
                        │           from: new_map_con
      External          │
      Inputs ───────────┼─────── /map (OccupancyGrid, latched)
                        │           from: MCL
                        │
                        ├─────── /pf/pose/odom (Odometry)
                        │           from: MCL / sim
                        ├─────── /sensors/imu/raw (Imu, optional)
                        │           real-car scan deskew
                        │
                        └─────── TF: map → laser frame
                                          │
                               ┌──────────▼──────────────────────┐
                               │   opponent_detector_node         │
                               │                                  │
                               │  detection → tracking → planner  │
                               └──────────┬──────────────────────┘
                                          │
      External                            ├──▶ /overtake_waypoints (OTWpntArray)
      Outputs ────────────────────────────│       to: wpnt_publisher
                                          ├──▶ /perception/obstacles (ObstacleArray)
                                          ├──▶ /proj_opponent_trajectory (ProjOppTraj)
                                          └──▶ /planner/avoidance/path (Path, viz)
```

**Inputs:** the node subscribes to `/map` (SLAM occupancy grid), `/pf/pose/odom` (ego pose), and
`TF` (`map→laser`) provided by MCL (`monte_carlo_localization`); `/global_waypoints` (global
raceline + track boundaries) published by `new_map_con`; the raw `/scan`; and optional
`/sensors/imu/raw` for scan deskew.

**Outputs:** the committed overtaking path `/overtake_waypoints` is consumed by
`wpnt_publisher`, which merges it into `/local_waypoints` for the `new_map_con` pure-pursuit
controller to follow. `/perception/obstacles` and `/proj_opponent_trajectory` expose opponent
information for downstream consumers, and `/planner/avoidance/path` is published solely for
RViz visualization.

## 2. Subscribed / Published Topics

| Dir | Topic (param) | Type | Default |
|-----|------|------|--------|
| Sub | `scan_topic` | `sensor_msgs/LaserScan` | `/scan` |
| Sub | `global_waypoints_topic` | `f110_msgs/WpntArray` (latched) | `/global_waypoints` (from `new_map_con`) |
| Sub | `map_topic` | `nav_msgs/OccupancyGrid` (latched) | `/map` (from `monte_carlo_localization`) |
| Sub | `ego_odom_topic` | `nav_msgs/Odometry` | `/pf/pose/odom` (sim: `/ego_racecar/odom`) |
| Sub | `imu_topic` | `sensor_msgs/Imu` | `/sensors/imu/raw` (optional; odom fallback) |
| Sub | TF | `map → <scan frame>` | provided by MCL / simulator |
| Pub | `obstacles_topic` | `f110_msgs/ObstacleArray` | `/perception/obstacles` |
| Pub | `raw_obstacles_topic` | `f110_msgs/ObstacleArray` | `/perception/detection/raw_obstacles` |
| Pub | `proj_opp_traj_topic` | `f110_msgs/ProjOppTraj` | `/proj_opponent_trajectory` |
| Pub | `avoidance_ot_topic` | `f110_msgs/OTWpntArray` | `/overtake_waypoints` (→ `wpnt_publisher`; only while committed, one empty OT on finish then silent) |
| Pub | `avoidance_path_topic` | `nav_msgs/Path` | `/planner/avoidance/path` (overtaking line, RViz green) |
| Pub | `opponent_path_topic` | `nav_msgs/Path` | `/perception/opponent/path` (opponent track, RViz orange) |
| Pub | `markers_topic` | `visualization_msgs/MarkerArray` | `/perception/obstacles/markers` (opp=red/static=blue/avoid=green) |

## 3. Main Parameters

Full list: [`config/opponent_detector.yaml`](config/opponent_detector.yaml).

| Parameter | Meaning | Default |
|----------|------|--------|
| `simulator` | Sim profile (ego odom from `/ego_racecar/odom`) | `false` |
| `lambda_deg` / `cluster_sigma` | Adaptive-breakpoint angle / noise | `10.0` / `0.03` |
| `min_cluster_points` / `max_obs_size` | Min cluster points / max size [m] (>0.707 so 0.5×0.5 passes) | `5` / `0.8` |
| `deskew_enable` / `deskew_source` | per-beam motion compensation / `auto`\|`imu`\|`odom` | `true` / `auto` |
| `deskew_translation_enable` / `deskew_sensor_timeout` | translational deskew (off until validated) / freshness [s] | `false` / `0.15` |
| `imu_angular_scale` | IMU yaw-rate unit conversion (current VESC deg/s→rad/s) | `π/180` |
| `noise_filter_enable` / `scan_median_window` | optional raw neighbour filter / median window (1=off) | `false` / `1` |
| `cluster_merge_enable` / `cluster_merge_distance` | merge nearby fragments / AABB gap [m] | `true` / `0.12` |
| `meas_range_var_scale` / `meas_sparse_var_scale` / `meas_yaw_rate_var_scale` | Kalman measurement-variance growth from range/sparsity/yaw | `2.0` / `1.5` / `0.25` |
| `max_viewing_distance` | Forward viewing distance [m] | `9.0` |
| `boundaries_inflation` / `fallback_track_halfwidth` | Corridor shrink [m] / half-width when bounds unset [m] | `0.1` / `1.5` |
| `use_map_filter` / `map_point_reject_ratio` | SLAM-map wall removal / cluster reject ratio | `true` / `0.6` |
| `classifier_mode` | `velocity` \| `std` \| `both` | `velocity` |
| `dyn_vel_enter` / `dyn_vel_exit` / `dyn_min_frames` | Enter/exit speed vs static field [m/s] / sustain frames | `0.5` / `0.25` / `3` |
| `static_ref_gate` | Track speed below which it defines the static-field velocity [m/s] | `0.3` |
| `vs_reset` | Low relative-speed backstop (force static) [m/s] | `0.1` |
| `assoc_gate` / `ttl_dynamic` / `ttl_static` | Association gate [m] / dynamic·static TTL | `0.5` / `40` / `3` |
| `assoc_use_mahalanobis` / `assoc_mahalanobis_gate` | covariance-aware association / 2-DoF χ² gate | `true` / `9.21` |
| `process_var_vs` / `process_var_vd` | Kalman process noise (s/d axis) | `2.0` / `8.0` |
| **`avoidance_enabled`** | overtake planner on/off | `true` |
| `avoid_trigger_min_ds` / `avoid_trigger_max_ds` | forward s-window considered for overtaking [m] | `0.5` / `8.0` |
| `avoid_block_margin` | consider overtaking only when the opponent is on the ego path [m] | `0.15` |
| **`ot_min_rel_vel`** | min relative speed (attainable ego v − opp vs) to commit [m/s] | `0.5` |
| **`ot_abort_rel_vel`** | abort a committed pass below this rel. speed (hysteresis) [m/s] | `0.2` |
| **`ot_max_catch_time`** | commit only when predicted catch time ≤ this [s] | `6.0` |
| **`ot_max_kappa`** | max raceline \|κ\| anywhere on the maneuver (corner gate) [1/m] | `1.0` |
| `avoid_pre_distance` / `avoid_post_distance` | min ramp to apex / return length [m] | `2.0` / `2.0` |
| **`ot_pass_clearance_s`** | apex hold zone around the predicted pass point [m] | `1.0` |
| `avoid_spline_resolution` | overtaking-line sample spacing [m] | `0.15` |
| `avoid_lateral_clearance` / `avoid_boundary_margin` | apex clearance / keep-inside-edge margin [m] | `0.30` / `0.35` |
| `ot_max_d_slope` | steering limit: max spline `\|dd/ds\|` (ramps/apex fitted to it) [-] | `0.40` |
| `ot_max_path_kappa` | curvature limit: max `\|d2d/ds2\|` the spline may add (steering angle → anti-spin) [1/m] | `0.5` |
| `ot_map_clearance` | free-space ring in the LIVE `/map` around each path sample (blocks walls/obstacles absent from CSV bounds) [m] | `0.20` |
| `ego_half_width` / `opponent_half_width` | ego / opponent half-width [m] | `0.15` / `0.25` |
| `avoid_side_mode` | side (`auto`\|`left`\|`right`) | `auto` |
| `avoid_max_lat_accel` | **floor** lateral-accel budget [m/s²] (per-sample capacity is `max(this, raceline vx²·\|κ_r\|)` — the raceline's own grip is trusted) | `4.0` |
| `ot_max_long_accel` | longitudinal-accel budget [m/s²]: fwd/bwd smoothing brakes BEFORE a tight section | `4.0` |
| `avoid_speed_scale` / `avoid_v_floor` / `avoid_v_ceiling` | speed scale / floor·ceiling [m/s] | `1.5` / `6.0` / `15.0` |
| `ot_speed_blend_s` | blend start=current ego speed, end=raceline speed (no handoff speed step) [m] | `1.5` |
| `path_intrusion_margin` | opponent within this distance of the committed path → replan/abort [m] | `0.5` |
| `replan_ds_threshold` / `replan_dd_threshold` | opponent s/d drift that forces a replan [m] | `0.5` / `0.2` |
| **`ot_ego_dev_replan`** | EGO lateral deviation from the committed path → replan from the current pose; 2x → abort [m] | `0.4` |
| **`ego_pose_grace_s`** | max age of the last successful ego CLCS projection; older → clear path + silence (global fallback) [s] | `0.3` |
| **`ot_completion_margin`** | completion: ego must pass the opponent by this much [m] | `1.0` |
| **`ot_max_duration_s`** | hard cap on a single committed maneuver [s] | `10.0` |
| **`ot_cooldown_s`** | silent time after completion/abort before a new commit [s] | `2.0` |
| **`ot_trail_enabled`** | publish a decelerate-and-follow path when the overtake is not possible | `true` |
| **`ot_trail_gap`** | following distance held behind the opponent while trailing [m] | `1.5` |

## 4. Build & Run

```bash
cd ~/2026_IFAC
colcon build --packages-up-to opponent_detector    # also builds global_planning (CLCS); all: cb
source install/setup.zsh                            # alias: sc

# real vehicle
ros2 launch opponent_detector opponent_detector.launch.py

# simulator (f110_gym): ego pose from /ego_racecar/odom
ros2 launch opponent_detector opponent_detector.launch.py simulator:=true
```

Prerequisite: `/global_waypoints` (from `new_map_con`) and `/map` + TF (from
`monte_carlo_localization` or the simulator) must be publishing.

## 5. Watch it in RViz

Pass `rviz:=true` to the launch file to bring up **RViz2** alongside the node (a map/scan/marker
preset: [`rviz/opponent_detector.rviz`](rviz/opponent_detector.rviz), top-down, fixed frame=map).
Stationary obstacles show as **blue** cylinders, the dynamic opponent as a **red** cylinder.

**Path A — synthetic harness (no simulator, fastest):**

```bash
# terminal 1: node + RViz
ros2 launch opponent_detector opponent_detector.launch.py rviz:=true
# terminal 2: injects a fake raceline/map/moving opponent/static object (prints PASS/FAIL)
python3 src/opponent_detector/test/synthetic_opponent_test.py
```

You should see a **red cylinder sliding along +s (the opponent)** and a **blue cylinder standing
still beside it (the static obstacle)**. The harness exits after 6 s.

**Path B — f110_gym 2-agent closed loop (real sim + a moving opponent):** set the sim to
`num_agent:2`, drive ego with `new_map_con`, and drive the opponent with `~/gap_follow`
(`f1tenth_control`). The opponent's drive **must** be remapped to `/opp_drive`. Full 4-terminal
sequence is in [`docs/opponent_detector_node.md` §3-1](docs/opponent_detector_node.md) (Korean). Gist:

```bash
# (1) sim   (2) ego                (3) opponent = gap_follow               (4) detector
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 launch new_map_con new_map_con.launch.py simulator:=true
ros2 launch f1tenth_control gap_follow.launch.py \
    scan_topic:=/opp_scan odom_topic:=/opp_racecar/odom drive_topic:=/opp_drive \
    is_simulation:=true force_autonomous:=true enable_aeb:=false
ros2 launch opponent_detector opponent_detector.launch.py simulator:=true rviz:=true
```

> **Two must-dos** — run the opponent with `enable_aeb:=false` (else its LiDAR AEB latches, the opponent
> stops mid-track, and the full-speed ego rear-ends it → sim freezes), and keep the detector's
> `use_sim_time` at its default false (this bridge does not publish `/clock` = wall clock). Troubleshooting:
> [`docs/opponent_detector_node.md` §3-1](docs/opponent_detector_node.md).

> To open RViz separately: `rviz2 -d install/opponent_detector/share/opponent_detector/rviz/opponent_detector.rviz`.

## 6. Verification (automated check)

PASS/FAIL only, no RViz:

```bash
# terminal A
ros2 run opponent_detector opponent_detector_node --ros-args \
  --params-file src/opponent_detector/config/opponent_detector.yaml
# terminal B
python3 src/opponent_detector/test/synthetic_opponent_test.py   # prints PASS/FAIL
```

## 7. Out of Scope (Follow-on)

- The GP-smoothed opponent line (`f110_msgs/OpponentTrajectory`) is a separate learning node in
  ForzaETH. If needed, add it as a new node consuming `/proj_opponent_trajectory`.

## 8. References

- Node-level rules: [`AGENTS.md`](AGENTS.md)
- Detailed doc: [`docs/opponent_detector_node.md`](docs/opponent_detector_node.md)
- Source paper: [`2603.27207v1.pdf`](2603.27207v1.pdf)
