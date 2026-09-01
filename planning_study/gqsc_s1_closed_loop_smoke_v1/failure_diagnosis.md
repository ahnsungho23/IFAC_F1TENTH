# Failure diagnosis

## Counted smoke result

No counted scenario produced a GQSC method, integration, lifecycle, controller-tracking,
simulator, sensor/state-interface, or other terminal failure. Collision, off-track, safe-stop,
planner invalidation, non-routine fallback, and path-publication discontinuity counts were zero.
The counted result is therefore `GQSC_S1_LOW_SPEED_SMOKE_PASS`.

## Excluded setup attempts

### Attempt A: `map_current` without simulator odometry remap

- Observed: KICP subscribed to `/odom`, which had zero publishers; the simulator published
  `/ego_racecar/odom`.
- Consequence: `/pf/pose/odom` stayed at the initial pose while the physical car moved. Frenet and
  the local path froze, the controller reported path-direction mismatch, and the simulator latched
  collision with zero obstacles.
- Classification: `SENSOR/STATE_INTERFACE`.
- Disposition: excluded before any S1 obstacle scenario.

### Attempt B: `map_current` with odometry remap

- Observed: auto-initialization was rejected because residual RMS median was 0.520 m, above the
  existing 0.350 m gate. A manual initial pose then drifted from `(2.13, 15.41)` to approximately
  `(-1.40, 3.46)` while stationary.
- Cause supported by evidence: `map_current` and installed `map.kissmap` are not the same geometry.
- Classification: `SIMULATOR` plus `SENSOR/STATE_INTERFACE` map contract mismatch.
- Disposition: excluded; localization was not bypassed or retuned.

### Accepted setup

The existing `map_kissmap_render.pgm`, installed `map.kissmap`, and the existing `map` global
waypoint bundle were used together. With the simulator odometry topic supplied through KICP's
existing `odom_topic` parameter, auto-initialization passed at residual RMS median 0.143 m. A clear
reference lap then completed without collision or path-direction warnings.

## Scenario scheduler deviation

The initial S02 rule changed to empty immediately after the Frenet lap wrap, before the vehicle
reached `s=6.0 m`. The already committed path continued and completed, but the obstacle was not
present through passage, so that attempt is `PROTOCOL_EXCLUDED`. S02 was run once more at curved
`s=24.0 m` with the same 0.44 m longitudinal extent and `[-0.18, +0.18] m` lateral bounds. No
planner code, parameter, ranking, bound, or geometry operator changed.

## Temporal and runtime diagnostics

The four counted maneuvers had zero committed-phase side switches. S02 had one back-to-back
pre-commit identity replacement; after commitment its identity remained stable. Repeated digest
changes reflect prefix trimming of the same committed candidate, not repeated candidate selection.

While S02 was staged with the controller stopped, provisional selections changed repeatedly during
initial acquisition and again before motion. This is retained as a zero-speed lifecycle diagnostic,
not counted as moving path chatter. It needs explicit stop-and-wait coverage in a later benchmark.

Fresh constructed callbacks exceeded the nominal 25 ms period in this instrumentation-ON smoke:
p50/p95/max was 25.690/33.421/33.489 ms. The overall active callback p95/p99 was only
1.492/1.562 ms because continuation dominated. No runtime overrun caused a visible closed-loop
failure, so it is not classified as a smoke runtime blocker; nevertheless final real-time
qualification is explicitly not claimed.

## Classification summary

| Evidence | Classification | Counted? |
|---|---|---:|
| Missing `/odom` publisher in first setup | `SENSOR/STATE_INTERFACE` | no |
| `map_current` vs `map.kissmap` residual/drift | `SIMULATOR` / `SENSOR/STATE_INTERFACE` | no |
| S02 wrap scheduler early clear | `OTHER` (research harness) | no |
| Four accepted obstacle maneuvers | no terminal failure | yes |
| Fresh-generation timing above 25 ms | runtime qualification warning | yes, nonblocking for this smoke |

No large benchmark was started, and no final holdout data was inspected.

