# AGENTS.md for local_planning

<<<<<<< HEAD
## Package purpose

- This package handles static-obstacle avoidance only.
- Its geometric invariant is race-line locking: every published moving path must preserve the
  ordered `/global_waypoints` samples and modify only their local Frenet `d(s)` offset.
- Never add Cartesian nearest-path search, free-space graph search, or a map shortcut that can jump
  to a different geometric track branch. This invariant is especially important on non-convex
  snake sections.
- Dynamic opponents remain the responsibility of `opponent_detector` and `/overtake_waypoints`.

## Algorithm rules
=======
## 1. Package Purpose
- Provides real-time obstacle interference checking against global waypoints and computes a multi-candidate Frenet lattice avoidance trajectory. No legacy generator may bypass lattice dynamic-feasibility validation.
- Publishes an ego-to-merge avoidance segment as `/avoid_waypoints` (OTWpntArray). If the primary lattice is infeasible, run the denser recovery lattice without relaxing collision clearance. If recovery also fails, publish a collision-checked gradual-braking segment before the obstacle; publish an empty array only when even that segment cannot be formed. `wpnt_publisher` routes this segment into `/local_waypoints`; standalone publishing remains disabled by default.

## 2. Key Rules & Conventions
- Written in modern C++17 for ROS 2 Humble.
- Generate the primary lattice by sampling both lateral target offsets and longitudinal transition lengths, then select the minimum-cost feasible candidate.
- Plan the nearest longitudinal obstacle cluster first. Prefer one lateral target from the full-span corridor intersection. If the primary lattice fails, guide recovery across the complete entry-obstacle-merge horizon with a bounded beam search over safe-corridor knots; enforce the selected side through the obstacle span and hard-check every connecting segment. Obstacle-only guided profiles are a last-resort compatibility fallback, not the first recovery stage.
- Identify a committed collision cluster in global-waypoint coordinates. As the moving horizon clips its passed prefix, treat a stable nearby cluster endpoint as the same cluster and reuse the validated commitment instead of rebuilding the lattice every timer cycle.
- Run the node with a two-thread `MultiThreadedExecutor`. Keep map, obstacle, and planning callbacks in one mutually-exclusive group, and receive Frenet odometry in a separate group. The odometry callback writes a mutex-protected latest-input buffer; each planning cycle copies one immutable pose snapshot before heavy lattice work so planning cannot make its own odometry stale or race the pose used by candidate evaluation.
- Recompute heading, curvature, velocity, and acceleration fields from each candidate's modified map-frame geometry before scoring or publishing.
- Build quintic lateral transitions against actual reference-line arc length `s`, not waypoint index. Use shape-preserving PCHIP slopes at internal corridor knots so piecewise segments are tangent-continuous without overshooting a lateral extremum. Score feasible candidates using the closed-form spatial lateral jerk integral, maneuver length, full-map mean clearance, a non-dilutable minimum-clearance deficit, raceline deviation, curvature, distance-normalized curvature rate, added path length, speed loss, and a small preferred-side bias.
- Commit the first validated avoidance path through its geometric merge. Do not clear it merely because the passed obstacle leaves the detection window. Revalidate remaining point and segment clearances every cycle, and replan only if the committed path becomes invalid or another obstacle overlaps the horizon.
- Detect blocking against both the global centerline and the currently committed path's lateral
  profile. A new obstacle that leaves `d=0` open but intersects the shifted local route must trigger
  next-cluster replanning before the current merge.
- Freeze the associated static-obstacle boxes used to validate a new commitment until its merge completes. Rebuild the grid from that snapshot plus unmatched newly detected obstacles, so detector jitter cannot invalidate the selected path while genuinely new obstacles still trigger validation and replanning.
- When another obstacle cluster is visible near the committed merge, proactively build its replacement from the current ego position. Replace the commitment only after full validation and a bounded start-gap check; otherwise retain the current path unchanged.
- Treat the committed avoidance side as hard hysteresis: select the opposite side only when every candidate on the committed side fails safety validation.
- Preserve the actual merge index for state-machine completion, but append a configurable collision-checked global continuation to every published avoidance segment so the controller retains adequate lookahead near the merge. Keep publishing that continuation for a bounded waypoint distance when the geometric merge has passed but the measured ego `d` has not settled inside the configured tolerance.
- Gate startup planning until consecutive, finite, track-bounded Frenet odometry samples are stable. Tolerate only the configured number of transient invalid samples, and hold the last collision-checked avoidance or braking segment for a bounded timeout during short stale-input gaps. Clear it when the map/global path changes or the hold timeout expires.
- Reject non-finite or non-monotonic global paths. A changed global path must clear odometry readiness and every avoidance commitment; an identical republish may retain state.
- Follows parameter rules: all runtime settings are loaded from `config/local_planning.yaml` via `launch/local_planning.launch.py`.
- Treat `/car_state/frenet/odom.pose.pose.position.x` as Frenet `s`; never treat CLCS `child_frame_id` as a global waypoint index.
- Respect OccupancyGrid origin rotation for all world/grid conversions.
- Build the planning obstacle mask from the complete OccupancyGrid. Never use connected-component area classification to suppress planning triggers; wall-attached, thin, and large obstacles must remain detectable. Use connected components only for grouping, visualization, and logging.
- Build an explicit wrapped-s Frenet safe corridor over the planning horizon. Shrink track bounds and inflate occupied/unknown intervals by the configured footprint, localization margin, and safety margin. Keep blocked and left/right feasible intervals explicit.
- Default to rejecting a complete quintic candidate when any sampled `d(s)` leaves the safe corridor. Pointwise clamping is legacy behavior and must remain disabled unless explicitly configured.
- Validate an oriented-rectangle footprint plus the existing conservative circle at every resampled path pose. Occupancy, unknown-policy, map-boundary, and track-boundary violations are hard rejections. Include the same segment-interior samples in the soft clearance term.
- Cache the map-content signature and rebuild full occupancy, clearance, and visualization component data only when occupancy data changes.
- Cache each waypoint's safe-corridor slice while the map and global path remain unchanged. Reassemble the moving horizon from cached slices without weakening the original occupancy or footprint rules.
- Validate every committed-path point and swept segment at least once on the current map. Cache successful checks only for that exact commitment and invalidate them whenever the commitment or map changes.
- Keep planning and safety checks at the configured timer rate, but allow debug MarkerArray publication to use a lower configurable rate.
- Include map dimensions, resolution, origin pose, and occupancy data in the map signature; reject mismatched OccupancyGrid data sizes.
- Validate both avoidance sides and every segment between waypoints against the complete occupancy grid with footprint inflation before publishing.
- Reject candidates above configured curvature, distance-normalized curvature-rate, and lateral-acceleration limits; cap candidate velocity from the recomputed curvature.
- Do not restore legacy least-squares or independent smoothstep avoidance fallbacks. Quintic smoothstep is permitted only as the lateral transition inside a fully validated lattice candidate.
- If primary and recovery lattice candidates fail during a sequential-obstacle handoff, first rebuild a braking prefix from the last validated full avoidance path. Recheck its remaining track boundary, point footprint, and swept-segment clearance from the current position, and use it only within `lattice_replan_brake_timeout_sec`. Then try the global-path safe stop. During a short replanning gap, retain only an already checked braking segment for the configured bounded hold time. When no moving or braking segment remains, publish a collision-checked two-point zero-speed hold at the current Frenet pose and keep replanning. Never force an occupancy-colliding candidate merely to keep the topic non-empty.
- Treat an obstacle as blocking only when its lateral extent overlaps the vehicle envelope around the global raceline. Expand that envelope by the configured tracking margin as curvature rises so tight-corner tracking error is covered.
- Never allow the candidate-generation wall margin to be smaller than `vehicle_radius + path_clearance_margin`.
- Publish only collision-checked paths on `/local_planning/path`. Keep failed candidates on the orange debug marker and publish an empty Path instead of relabeling the global path as local avoidance.
- Red RViz spheres represent grouped obstacle-component centroids, not the complete detection mask. Safe-corridor boundaries, blocked/left/right intervals, inflated cells, and rejection counters must remain available on the debug MarkerArray.
- Keep detection and clearing hysteresis configurable.
- Keep the static-map response proactive: the default `ifac_track` profile detects about 12 m
  ahead and begins the avoidance transition about 4 m before the obstacle. Preserve the
  relationship `lookahead_wpnt_num >= detection_lookahead_wpnt_num` when tuning it.

## 3. Interfaces
- Subscribes: `/global_waypoints`, `/map`, `/perception/obstacles`, `/car_state/frenet/odom`.
- Publishes: `/avoid_waypoints`, `/local_waypoints`, `/local_planning/path`, `/local_planning/markers`.
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970

- Runtime code is C++17 for ROS 2 Humble.
- Use `f110_msgs/msg/ObstacleArray`, `WpntArray`, and `OTWpntArray`; do not create a new message.
- Consume map-frame Cartesian obstacle centers and enclosing-circle radii from
  `/perception/static_obstacles/cartesian`. Project each center through the shared CLCS converter,
  then use the same radius as its longitudinal and lateral Frenet extent before selecting the
  nearest blocking obstacle cluster and evaluating both sides.
- Derive each target `d` from the obstacle lateral bound plus configured clearance. Reject targets
  outside the per-waypoint `d_left`/`d_right` track widths.
- Fit the local offset in unwrapped global Frenet `s`, clip cubic overshoot to the control-point
  extrema, and convert each selected global waypoint with its own normal. Preserve `s_m` and order.
- Validate lateral slope, recomputed Cartesian curvature, curvature rate, obstacle clearance, and
  track-bound clearance before publishing.
- Prefer a committed side while it remains feasible. Keep a validated commitment until its tail
  merges at `d=0`, even if perception drops the passed obstacle.
- If neither side is safe, publish only a collision-checked gradual-stop prefix before the obstacle.
  Never publish an unvalidated avoidance path merely to keep `/avoid_waypoints` non-empty.
- Recompute heading, curvature, velocity, and acceleration after applying `d(s)`.
- Handle closed-track `s` wrap explicitly. Never encode a waypoint index in Frenet odometry fields.

## Interfaces

- Subscribe: `/global_waypoints` (`f110_msgs/msg/WpntArray`).
- Subscribe: `/perception/static_obstacles/cartesian` (`f110_msgs/msg/ObstacleArray`); each obstacle
  must set `has_cartesian=true` and provide `x_center`, `y_center`, and a positive `radius`.
- Subscribe: `/car_state/frenet/odom` (`nav_msgs/msg/Odometry`), with `position.x=s` and
  `position.y=d`.
- Publish: `/avoid_waypoints` (`f110_msgs/msg/OTWpntArray`) as an ego-to-merge segment with
  map-frame Cartesian `x_m/y_m` populated for every waypoint.
- Publish debug: `/local_planning/path`, `/local_path`, `/local_planning/markers`.
- Standalone `/local_waypoints` publication stays disabled by default because `wpnt_publisher`
  selects `/avoid_waypoints` according to `/state`.

## Package layout and maintenance

- Node declaration: `include/local_planning/local_planner_node.hpp`.
- Algorithm declaration: `include/local_planning/raceline_spline_planner.hpp`.
- C++ sources: `src/local_planner_node.cpp`, `src/raceline_spline_planner.cpp`.
- Runtime parameters: `config/local_planning.yaml`.
- Launch entrypoint: `launch/local_planning.launch.py`.
- Korean node documentation: `docs/local_planner.md`.
- Algorithm tests: `test/test_raceline_spline.cpp`, including the wrong-branch snake regression.
- Manual Cartesian contract harness: `test/cartesian_static_pipeline_test.py`; run it against a
  fresh `local_planner_node` with a `global_waypoints.csv` path.
- Keep all runtime values configurable in YAML and load that YAML from the launch file.
- Update this file and the Korean documentation when behavior, topics, parameters, or launch usage
  changes.
