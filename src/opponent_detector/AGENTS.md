# AGENTS.md — opponent_detector

Node-level rules for the `opponent_detector` package. Inherits and must not weaken the
root `CLAUDE.md` / `AGENTS.md`.

## Purpose

A C++ ROS 2 node that uses **2D LiDAR as the primary obstacle measurement** and optional IMU/odom
motion compensation to detect and track a dynamic opponent (no camera), for the overtaking pipeline.
It adapts the opponent-detection idea of the paper
`2603.27207v1.pdf` (Cihlar et al., "Autonomous overtaking trajectory optimization…"): that paper
uses a depth camera + YOLO to pick which LiDAR cluster is the opponent. We have no camera, so the
camera's cluster-selection role is replaced by **relative velocity in the raceline (Frenet) frame** —
static structure sits still in that frame, the opponent moves. Output is ForzaETH-compatible
`f110_msgs` so a downstream spliner/planner can consume it.

## Rules

- Runtime node is C++ (ROS 2 Humble). Do not add a Python runtime node here. The `test/` script is
  a manual integration harness only (allowed as a test helper), not a runtime dependency.
- Frenet projection uses **`global_planning`'s CLCS converter** (`global_planning::ClcsFrenetConverter`),
  linked as an exported library — `opponent_detector` depends on `global_planning` (build it first). Do
  NOT revert to the Python `frenet_converter` (Python-only, unlinkable from C++). The lightweight
  `FrenetProjector` remains ONLY for track-boundary (`d_left/d_right`) lookup and s-wrap, which CLCS
  does not provide; don't use it for projection. If `global_planning`'s exported lib changes, keep the
  `find_package(global_planning)` + link in `CMakeLists.txt` working.
- Prefer existing `f110_msgs`: publish `ObstacleArray` (`/perception/obstacles`), a static-only
  Cartesian `ObstacleArray` (`/perception/static_obstacles`), and `ProjOppTraj`
  (`/proj_opponent_trajectory`). Populate each `Obstacle` with Cartesian `x_center/y_center`,
  Frenet `s_center/d_center`, and a conservative enclosing-circle `radius` equal to half the
  Cartesian AABB diagonal. Set `has_cartesian=true`; do NOT invent a new message type.
- Inputs: `/scan` (ego LiDAR), `/global_waypoints` (latched, from `new_map_con`), `/map` (latched,
  from `monte_carlo_localization`), ego pose odom (`/pf/pose/odom`, sim `/ego_racecar/odom`), and
  optional `/sensors/imu/raw` for scan deskew. The
  laser→map transform uses TF2 with the scan's own `frame_id`, so `laser` vs `ego_racecar/laser`
  needs no manual switch.
- All tunables live in `config/opponent_detector.yaml`, declared with safe defaults in the node.
  Do not hard-code thresholds. Keep the `simulator` param + `simulator:=true` launch path working.
- The static/dynamic split is the core idea. Keep `classifier_mode` (`velocity`/`std`/`both`)
  configurable; `velocity` is the default (this project's chosen approach), `std` is the ForzaETH
  positional-spread fallback.
- Static obstacles matter: after the SLAM `/map` filter removes walls, non-wall stationary obstacles
  remain and are car-sized (≤0.5×0.5 m), so size cannot separate them from the opponent. Classify by
  **velocity relative to the static-field reference** (mean of tracks below `static_ref_gate`) — same
  velocity as the walls → static, deviating → dynamic. Do NOT reintroduce an absolute-zero velocity
  test, and keep `max_obs_size` above the 0.5×0.5 diagonal (0.707 m) so those obstacles are detected.
- Perception must repeatedly publish confirmed static detections. Its `ttl_static` only bridges
  brief measurement dropouts; long-term static-obstacle memory and avoidance decisions belong to
  the downstream local planner. Static detections are published as map-frame Cartesian centers
  plus an enclosing-circle radius on
  `/perception/static_obstacles`. Do not add a persistent static map to this node.
- Keep the front end ordered as: optional per-beam IMU/odom deskew → optional raw-noise filtering
  → adaptive-breakpoint clustering → size-safe fragment merge → CLCS projection. Keep raw filtering
  off by default until f1sim_C and rosbag sweeps show that it does not erase small/far obstacles.
- Tracker association keeps `assoc_gate` as a hard geometric bound, then optionally applies the
  covariance-aware Mahalanobis gate. Detection covariance must remain parameterized and may grow
  with range, sparsity, and yaw rate.
- **Overtake planner is merged into this node** (not a separate package): detection AND the committed
  spline overtaking line live in `opponent_detector_node`. It publishes `f110_msgs/OTWpntArray` on
  `/overtake_waypoints`; do NOT invent a new message. Keep the downstream contract intact —
  `wpnt_publisher` overrides `/local_waypoints` with the OT and `new_map_con` follows it, so the OT
  `wpnts` must carry valid map-frame `x_m,y_m` + `vx_mps` + `s_m`.
- The planner is a **state machine** (`OvertakePlanner`, `overtake_planner.{hpp,cpp}`):
  `Idle -> Committed -> Cooldown -> Idle`. Publish policy: while COMMITTED the local path is
  published every cycle; on completion/abort ONE empty OT is published (wpnt_publisher falls back to
  the global raceline) and then the topic goes SILENT. Do NOT go back to publishing empty OTs every
  scan — "no maneuver" means "no messages".
- **Trailing (follow) mode** (`ot_trail_enabled`/`ot_trail_gap`, `buildTrailPath`): when the
  opponent BLOCKS the corridor but the overtake gates reject (or during the abort cooldown), the
  planner publishes a decelerate-and-follow path (`ot_line="trail"`, `ot_side="trail"`) instead of
  going silent — silence would hand control back to the full-speed global raceline and rear-end
  the opponent. The trail path follows the raceline (ego d ramps to 0 under the same
  slope/curvature limits), and its speed target is capped at the OPPONENT speed from `ot_trail_gap`
  behind it; the shared fwd/bwd accel pass brakes ahead of that point. No `v_floor` in trail mode
  (following a slow car must be allowed to be slow). The return ramp from the current ego offset
  to the raceline respects the SAME slope/curvature limits: when the ramp needs more room than
  the opponent gap, the path extent is EXTENDED (`t_total = max(ds_opp, min_len, ramp/0.7)`) —
  do NOT clip the ramp to the gap (the old behaviour yanked a mid-swing-aborted ego back to
  center at several times the curvature limit = a violently hooked local path). Geometry past
  the opponent's s is safe: the speed profile caps at the opponent speed from `t_follow` onward.
  It is rebuilt every cycle (never committed),
  ends with ONE empty OT (`trail end`), and upgrades seamlessly to a commit (no empty in between)
  the moment an overtake becomes feasible. Do NOT remove the blocking/window gates from
  buildTrailPath — trailing a non-blocking or far opponent would pin the ego to slow speeds.
- **Published paths must relate to where the car actually IS** (the "weird local path" edge
  cases; see `test/offpath_recovery_test.py`):
  - *Ego-deviation guard* (`ot_ego_dev_replan`, `egoPathDeviation`): while COMMITTED the plan is
    otherwise only refreshed when the OPPONENT drifts — the ego leaving the line (bump, slide,
    wall contact, tracking failure) must ALSO be handled. Deviation beyond the threshold →
    same-side replan from the CURRENT pose (`replan (ego off path)`); beyond 2x → abort even
    alongside (the "never abort alongside" rule assumes the ego is ON the swing line). Do NOT
    remove this guard.
  - *Ego-Frenet freshness gate* (`ego_pose_grace_s`, node-side): a failed CLCS projection (ego
    off the domain, e.g. pushed off-track by a crash) keeps the last (s,d) but NOT the freshness
    stamp; once stale the planner receives `ego.s = -1`, clears any active path ONCE and stays
    silent. NEVER let the planner keep planning from a frozen pre-crash pose.
  - *Downstream staleness net*: `wpnt_publisher` expires an OT feed that stops without the
    clearing empty OT (`ot_timeout_sec`, see `src/wpnt_publisher/AGENTS.md`) — do not rely on it
    as the primary mechanism, but do not break the every-cycle publish contract it assumes.
- **Runway extension + pre-apex gap hold** (`buildPath`): when trailing tightly at matched speed
  the predicted catch point is closer than the shortest slope/curvature-limited ramp that reaches
  a clearing apex — without the extension every commit rejects and the planner is TRAPPED in
  trailing forever (the "never overtakes" bug). If `t_apex0 < ramp_len(amp_max)` the apex is
  pushed out to that minimum runway and `ds_pass`/`t_apex1` are re-derived from it: the ego swings
  out FIRST, then passes. Safety: in the committed speed profile's forward pass, samples not yet
  laterally clear of the opponent (`|d-opp_d| < half_sum + 0.5*lateral_clearance`) with predicted
  longitudinal gap inside `ot_trail_gap` are capped at the OPPONENT speed, so the ego never closes
  on a still-centered opponent mid-ramp. Do NOT remove either half — the extension without the
  gap hold rams the opponent; the gap hold without the extension re-creates the trap.
- Commit gates (ALL must hold; keep them): opponent dynamic + ahead in the trigger window + blocks
  the ego corridor (`|opp_d| < ego_half+opp_half+block_margin`) + **catchable** (attainable rel.
  speed >= `ot_min_rel_vel`, catch time <= `ot_max_catch_time`) + **no sharp corner** on the whole
  maneuver (raceline `|kappa| <= ot_max_kappa`) + lateral room at the **predicted pass location**
  (the apex is placed where the pass will happen, not at the opponent's current position).
- While COMMITTED the opponent keeps moving, so validity is re-judged every cycle: completion
  requires the pass to be OBSERVED (opponent seen behind the ego by `ot_completion_margin` at
  least once — a lost track must NEVER read as "overtake complete"); opponent lost before the
  pass -> abort; while still approaching (not yet alongside, `ds_opp > max(1, pass_clearance)`)
  ALL commit gates are re-run every cycle and if neither side is feasible (corner, no room inside
  the wall margin, uncatchable, opponent left the corridor) the maneuver ABORTS back to the
  global raceline; opponent intrudes on the passing gap -> side switch else abort; relative speed
  collapses (`ot_abort_rel_vel` hysteresis) -> abort; opponent drifted past
  `replan_ds/dd_threshold` -> replan from the CURRENT ego (s,d). Once ALONGSIDE, never abort for
  feasibility (dropping to global mid-pass steers into the opponent). A hard `ot_max_duration_s`
  timeout and an `ot_cooldown_s` re-commit cooldown bound every maneuver.
- The local path covers ONLY the overtaking segment (ego -> predicted pass -> merge + short tail),
  never the whole track. The spline starts at the CURRENT ego `d` (not assumed 0). ALL tunables
  stay in the single `config/opponent_detector.yaml` (do not add a second YAML).
- The overtaking line is a **shape-preserving PCHIP spline** `d(s)` (monotone cubic Hermite),
  sampled to map frame via the raceline (`x=x_r-d*sin psi, y=y_r+d*cos psi`). `d` is
  left-of-travel-positive (matches FrenetProjector/CLCS). Do NOT switch back to a natural cubic —
  it overshoots (swerves the wrong way before the apex).
- Wall margin is enforced by **apex fitting**, NOT posterior sample clamping: the apex is
  iteratively shrunk until the whole spline stays within `d_left/right - boundary_margin -
  ego_half_width` at every sample; if the shrunken apex can no longer clear the opponent the side
  is rejected. Do NOT reintroduce per-sample clamping — it flattens the line along the wall and
  breaks the smooth (min-curvature-like) spline shape.
- The overtake line respects BOTH a **slope limit** `ot_max_d_slope` (max `|dd/ds|` = heading
  offset) AND a **curvature limit** `ot_max_path_kappa` (max `|d2d/ds2|` = the steering ANGLE — a
  curvature spike spins the car even at modest slope, which is the tight/short-ramp edge case).
  Entry/exit ramps are LENGTHENED to hold both (`ramp_len = max(1.5*A/slope, sqrt(6*A/kappa))`),
  the reachable apex is CAPPED by them, and the fitted spline's actual max slope AND max curvature
  are verified (apex shrinks on the worse violation). Do NOT hardcode fixed ramp lengths that
  ignore the apex magnitude, and do NOT limit only the slope. The steering-extended merge can push
  the maneuver past the minimal corner-gate extent, so the corner gate is re-checked over the
  extension inside the fit.
- Overtake speed is **curvature-limited AND longitudinally smoothed**: the per-sample ceiling is
  `v_phys = sqrt(a_cap / |kappa_total|)` with `a_cap = max(avoid_max_lat_accel,
  raceline_vx^2*|kappa_r_smoothed|)` (kappa_total = raceline + smoothed spline d''; the raceline
  kappa in a_cap gets the SAME 3-pt smoothing so no-added-curvature samples give v_phys == vr
  exactly). The raceline speed is
  feasible by construction, so its own grip usage is trusted as LOCAL capacity — a fixed a_lat
  budget below it would drag the car under the raceline speed even where the spline adds no
  curvature (that was the "brakes way too hard" regression). `avoid_max_lat_accel` is only the
  floor grip for spline curvature on straights. The target is then passed through a
  **forward+backward longitudinal-accel pass** (`ot_max_long_accel`) so the car BRAKES BEFORE a
  tight section instead of at it (a per-sample cap slows too late -> too fast into the wall). Do
  NOT drop the fwd/bwd pass back to a per-sample cap, do NOT let `v_floor`/`speed_scale` exceed
  `v_phys`, and do NOT remove the raceline-grip term from a_cap. The profile is anchored to the
  current ego speed at the start and eased into the raceline speed at the merge end
  (`ot_speed_blend_s`) so the local<->global handoff never STEPS the setpoint (no hard accel
  entering / brake exiting).
- Every candidate spline is ALSO validated against the **live /map** (node passes a
  `pathPointCollides` checker via `setCollisionChecker`; `ot_map_clearance` free-space ring per
  sample). The CSV `d_left/d_right` raycast can miss structure that only exists in the loaded map
  (obstacle-baked maps like `fuck_f1_obs`) — Frenet bounds alone let the path cut through such
  walls. On a map hit the apex shrinks and refits; if the blockage is not apex-driven the side is
  rejected. Never publish a path through occupied cells.
- Do NOT hardcode a slow constant speed. The intended slowdown is ONLY the physics one (braking
  for the added spline curvature, per the rule above) — no arbitrary global de-rating. On the
  downstream side `wpnt_publisher`'s OT speed factor is param `ot_vx_scale` (default 1.0, no halving).
- Keep `avoidance_enabled` a toggle. RViz mirrors: `nav_msgs/Path` `/planner/avoidance/path` (green) and
  `/perception/opponent/path` (orange) + obstacle MarkerArray.
- The GP-smoothed `OpponentTrajectory` line is intentionally out of scope (it is a separate learning
  node in ForzaETH). If added later, do it as a new node consuming `/proj_opponent_trajectory`.

## Layout

- `src/opponent_detector_node.cpp` — node, clustering, filtering, pipeline, publishing (incl. OT).
- `src/frenet_projector.{hpp,cpp}` — C++ Cartesian→Frenet projection over global waypoints.
- `src/obstacle_tracker.{hpp,cpp}` — CV Kalman tracking + static/dynamic classification (Frenet).
- `src/overtake_planner.{hpp,cpp}` — committed overtaking state machine + PCHIP spline (Frenet→map)
  → OTWpntArray wpnts.
- `config/opponent_detector.yaml` — all tunables.
- `launch/opponent_detector.launch.py` — `simulator:=true|false`, `rviz:=true|false` (loads the preset).
- `rviz/opponent_detector.rviz` — RViz2 preset (top-down, map/scan/markers); installed by CMake.
- `docs/opponent_detector_node.md` — Korean operation doc.
- `docs/sim_test_commands.md` — Korean cheat-sheet of harness/f1sim test commands (keep in sync
  with the harness list and the launch procedure in the node doc).
- `test/synthetic_opponent_test.py` — synthetic moving-opponent integration check.
- `test/commit_lock_test.py` — overtake state-machine check (commit → sustained replans past the
  opponent, NO trail fallback mid-approach → ONE completion empty → silence). The harness ego
  FOLLOWS the published local path laterally (like the real controller) — the ego-deviation
  guard replans whenever the car is off the committed line, so a laterally frozen ego would keep
  resetting the swing-out.
- `test/trail_to_overtake_test.py` — trail→overtake escalation check (ego tight-trailing a
  matched-speed opponent must still COMMIT via the runway extension: apex reached, pre-apex gap
  hold near opponent speed, acceleration once laterally clear).
- `test/offpath_recovery_test.py` — off-path recovery check (ego knocked off the committed path
  → every subsequent path re-anchors near the NEW ego pose; ego off the CLCS projection domain
  → ONE clearing empty OT after `ego_pose_grace_s`, then silence).
  Run ALL harnesses against a fresh node in a clean
  `ROS_DOMAIN_ID`, launched WITHOUT `simulator:=true` (they publish ego odom on `/pf/pose/odom`).
  RESTART the node between harnesses — leftover tracker ghosts / stale-ego state from a previous
  scenario contaminate the next one.
- `2603.27207v1.pdf` — the reference paper this node adapts.

## Checklist before finishing a change

- Builds with `colcon build --packages-select opponent_detector`.
- Thresholds live in YAML, declared with safe defaults in the node.
- Node starts and runs both real (`/pf/pose/odom`) and `simulator:=true` (`/ego_racecar/odom`).
- `test/synthetic_opponent_test.py` still PASSes (dynamic detected, static kept static, ProjOppTraj populated).
- `docs/opponent_detector_node.md`, the topic/param tables, and the README pair stay in sync with code.
- Repo indexes updated (`README.md`/`README_en.md`, `src/README*.md`, `CLAUDE.md` component table).
