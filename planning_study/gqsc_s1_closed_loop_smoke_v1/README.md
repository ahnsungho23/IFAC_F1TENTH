# GQSC-S1 pre-smoke checkpoint and low-speed closed-loop smoke v1

## Result

`GQSC_S1_LOW_SPEED_SMOKE_PASS`

The frozen S1 generator completed the four counted, deterministic 2.0 m/s scenarios without
collision, footprint off-track, planner failure, safe-stop, or non-routine fallback. Every counted
maneuver reached `COMPLETE` and returned to the global line. This is a system-integration smoke
result, not final real-time qualification or a large closed-loop benchmark.

## Immutable pre-smoke checkpoint

- Branch: `planning_r3_rt_bounded_proposal`
- Commit: `534e5e63fdc19146c24887f697ed6b9f869d2347`
- Commit message: `Prepare frozen GQSC-S1 for closed-loop smoke`
- Annotated tag: `gqsc_s1_pre_smoke_v1`
- Tag object: `ba82d24c26112e557ac3ba6565058427f5260196`
- Tag target: `534e5e63fdc19146c24887f697ed6b9f869d2347`
- Frozen S1 SHA-256: `670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776`
- Staged/committed checkpoint: 96 files, 2,325,435 bytes
- Branch and tag were pushed to `origin`; the remote branch and peeled tag were verified against
  the local commit. The checkpoint worktree was clean.

Before the commit, the clean rebuild succeeded, all applicable local-planning tests passed, and
`git diff --cached --check` passed. The two known parameter/control mismatches and two known
production-parity cases remained unchanged. No build, install, log, cache, temporary benchmark,
or raw bag artifact was committed.

## Runtime setup and evidence boundary

- Simulator physics map: existing
  `src/kinematic_localization/maps/map_kissmap_render.pgm`
  (`60f63b93168b7b3c94e5412355b804ec5c918cae7bc608aa8e8bbc49b7984109`).
- Global path: existing `offline_trajectory_generator/config/output/map/global_waypoints.json`
  (`bc2191cd1ddf32c8019bb57b4fcfecc5c04e44b10ff139aa7bc4ef845422f640`).
- Speed cap: 2.0 m/s.
- Simulator seed/noise: 12345 / 0.01 m.
- Planner: checkpoint commit above, research instrumentation ON, `p3_diagnostics_detail=FULL`.
- Obstacle source: deterministic direct publication to the planner's authoritative
  `/confirmed_static_obs` input. This smoke therefore covers the planner/controller/lifecycle
  chain but does not evaluate obstacle detection.
- The simulator publishes odometry as `/ego_racecar/odom`; KICP normally subscribes to `/odom`.
  The run supplied the identical simulator odometry through the node's existing `odom_topic`
  parameter and an identity `ego_racecar/base_link -> base_link` TF adapter. No source or YAML was
  changed. Auto-initialization was accepted with residual RMS median 0.143 m.

The raw temporary MCAP lasted 231.733 s. Quantitative smoke statistics use only the two intervals
with commanded speed greater than 0.2 m/s: 75.780 s and 31.700 s. Planner JSONL has
`ros_time_ns=0` because the simulator did not provide `/clock`; the join therefore uses the
non-zero obstacle source stamp, checked against MCAP record time. This limitation is explicit and
the run is not used as a clock-synchronization validation.

## Counted scenarios

| Scenario | Geometry | Selected side | Lifecycle | Safety result |
|---|---|---:|---|---|
| S01 | centered obstacle at straight `s=42.0 m` | right | fresh -> committed -> continuing -> complete | pass |
| S02 | centered obstacle at curved `s=24.0 m` | left | two pre-commit selections, then stable continuation -> complete | pass |
| S03 | obstacle biased to the right at straight `s=17.0 m`, requiring a clear left maneuver | left | fresh -> committed -> continuing -> complete | pass |
| S04 | centered obstacle at `s=30.0 m`, followed through completion and raceline return | right | fresh -> committed -> continuing -> complete | pass |

The first S02 scheduler attempt at the lap interface removed the obstacle at wrap before the car
reached its station. The already committed path still completed, but that attempt is marked
`PROTOCOL_EXCLUDED`. S02 was repeated once in the same immutable run at another curved section
with identical lateral/longitudinal obstacle dimensions. No GQSC parameter or code changed.

## Closed-loop observations

- Active planning callbacks: 4,300.
- Fresh S1 evaluations per callback: 0 for 883 callbacks, 1 for 3,417 callbacks, and never 2 or 3.
- Full 128-factor/12-path/12-validator construction occurred 16 times; observed maxima remained
  `128/12/12`.
- Collision: 0 true samples out of 26,870 active-drive collision samples.
- Minimum vehicle-footprint-to-track margin: 0.298 m; no off-track sample.
- Safe-stop entries: 0. Non-routine planner fallbacks and invalidations during active drive: 0.
- Local waypoint publication gap p95/p99/max: 6.973/7.453/11.394 ms.
- Path tracking error p50/p95/p99/max: 0.056/0.237/0.380/0.521 m. The maximum occurred in the
  curved S02 maneuver, while its minimum footprint-track margin remained positive at 0.426 m.
- Within every counted committed maneuver, side switches were zero. S01/S03/S04 retained one
  candidate identity; S02 had one back-to-back pre-commit identity replacement and then retained
  the committed identity.
- Path digests changed during continuation because the committed suffix was prefix-trimmed. This
  is not candidate switching: candidate identity stayed fixed after commitment.
- Chaining was not triggered. Each counted maneuver completed and the state machine returned from
  `STATE_AVOID` to `STATE_GLOBAL`.

## Runtime interpretation

Across all active-drive callbacks, p50/p90/p95/p99/max was
0.431/1.276/1.492/1.562/30.854 ms because most callbacks reused a committed result or exited before
constructing paths. For the 16 callbacks that actually constructed and validated 12 paths:

- S1 evaluator: 22.896/29.103/29.939/30.020/30.040 ms.
- Full callback: 25.690/32.276/33.421/33.476/33.489 ms.

Thus normal continuation comfortably met 25 ms, but fresh-generation tail did not. These numbers
were collected with full research instrumentation enabled and only 16 constructed samples, so
they neither prove deployable no-instrumentation latency nor final real-time qualification. The
overruns caused no missed path publication, collision, or planner failure in this low-speed smoke;
they remain a required gate for the later real-time/large-benchmark stage.

## Files

- `smoke_scenarios.csv`: frozen scenario geometry and protocol disposition.
- `closed_loop_results.csv`: per-counted-maneuver lifecycle, tracking, and safety results.
- `temporal_stability.csv`: identity, digest, side, and lateral-path continuity diagnostics.
- `runtime.csv`: live callback/evaluator timing distributions.
- `evaluations_per_callback.csv`: actual callback-global evaluation counts.
- `failure_diagnosis.md`: excluded setup attempts, scheduler deviation, and final classification.

No S1 method, B128/K12 bound, geometry operator, ranking, validator, lifecycle, controller,
perception, localization source, or vehicle parameter was modified. No final holdout or large
randomized benchmark was accessed or run.

