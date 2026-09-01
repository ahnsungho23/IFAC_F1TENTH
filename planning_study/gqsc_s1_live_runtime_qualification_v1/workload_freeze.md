# Immutable workload freeze

## Common production identity

All three rows are bound to the following identity. A mismatch is a stop condition, not a reason
to rebuild or substitute an artifact during a run.

| Item | Frozen value |
|---|---|
| repository checkpoint | `55c61e54540fc07d57e6041655e0e59ddb8e17e8` |
| method | `LEX8_GLOBAL_DISJOINT_COVERAGE4` |
| method SHA-256 | `670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776` |
| reference-v3 SHA-256 | `965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780` |
| pair/lexicographic/coverage | `128/8/4` |
| reconstruction/validator maxima | `12/12` |
| planning period | `25 ms` |
| production mode | `TEST_ACTIVE` |
| frozen-contract source SHA-256 | `21e653cf9b063c9c60843c6ebaeb06fda918046bfe6d78a7d4ba6b957bdddd40` |
| planner configuration SHA-256 | `4fe480351a80135ff2a6e4592f661ff8a5670c032e12554d85065339d16ea960` |
| path-digest implementation SHA-256 | source `cdd8548969b685a399d0789b0bb8343986cc6f40370ff4e621fac884a6f83f60`; header `60c163ba6e88e3b7e49541dd3e48dc5dfa32d1d899df2ba316f1accae45adafa` |

The current standalone executable SHA-256 is
`5428854081403031bd7fd3d037eae11258b0fb9bd6e48d3f9a59ebbfc49a6ede` and its ELF build ID is
`8d11477a6b1c8e9ffb806b723f6f60f7dfa0a6a9`. The current ROS node SHA-256 is
`3059d3c51d563fc2f4102289508a0a7578ddc059543d97022d451744f295d595`. These identify the present
pre-run binaries; the empty `CMAKE_BUILD_TYPE` is recorded rather than silently repaired.

## Candidate audit

No candidate was ranked using a latency column.

| Candidate | Original purpose and retained identity | Runtime semantics | Repetition/parity assessment | Decision |
|---|---|---|---|---|
| `SCE018` | allowed success-control event from `rosbag2_2026_08_25-09_59_22`; canonical event bytes retained | standalone harness, no simulator/executor in timed region | direct harness creates a new generation and lifecycle per iteration; 128/12/12 and digest retained | selected for W1 |
| `ROS_NODE_ISOLATED/SCE018` | earlier environment diagnosis; exact publisher command and message-level artifact not retained | actual node/executor, no simulator, but `SHADOW`; callback total includes unchanged P0 work | 40 historical samples; eager SHADOW evaluation is not equivalent to production TEST_ACTIVE | rejected as a frozen W2 procedure; SCE018 geometry remains the sole W2 target |
| `LIVE221808` | one prior live callback; reconstructed ego/reference/geometry and selected digest documented | prior full live callback plus reconstructed standalone query | only one live sample; compact event did not preserve all ROS obstacle fields and no canonical event file is retained | rejected |
| `STRAIGHT_201` | one small live requalification callback (`1955`) | simulator plus ROS, production-like | only one full-bound callback; no retained exact scenario driver or replay input | rejected |
| previous live-smoke cases | closed-loop safety/runtime evidence | simulator plus ROS | raw MCAP and exact launch/scheduler procedure are not retained in the directory | rejected as immutable inputs |
| `S01`–`S04` | four documented deterministic obstacle geometries; S01 is the simplest straight centered case | historical simulator plus ROS with direct authoritative obstacle publication | geometry and summary retained, but the old path bytes and scheduler/initialization commands are absent | S01 designated for W3 but blocked |

All chosen/designated inputs are pre-existing allowed development or live-smoke research evidence.
No row depends on the prohibited data split.

## W1: `W1_SCE018_STANDALONE`

Status: `FROZEN_READY`.

- Canonical source:
  `planning_study/p3_geometry_conditioned_method_v1/success_control_inputs/SCE018.event`
- Source SHA-256: `572adb59ea24f3f06bed7502eb870e57a33a5416e8630106b67bc2b1d0b405bf`
- Format: `P3_ORACLE_EVENT_V1`
- Provenance: event `SCE018`, bag `rosbag2_2026_08_25-09_59_22`, row/index `14769/1`,
  purpose `PLAN_PRIMARY`, allowed class
  `PILOT_SEEN_DEVELOPMENT_DATA_SUCCESS_CONTROL_AUXILIARY`
- Ego: `s=11.801691151003425`, `d=0.074708912484302642`, `speed=0`
- Recorded source stamp: `1787619957628360127`
- Reference: 185 waypoints embedded in the event; no external map is read
- Obstacle: id `231`, `s_center=20.930181948017037`,
  `s_start=20.857921486561061`, `s_end=21.002442409473012`,
  `d_right=-0.39155248802935838`, `d_left=-0.072975715874275343`,
  size `0.3498248946488427`, static and visible
- Expected counts: pair proxies/reconstructions/validators `128/12/12`; hard/usable `9/9`
- Expected selected path digest: `c7b2c19bf2af9350`
- Expected failure classification: `NONE`; expected timed outcome: `FRESH_SELECTED`

The harness excludes event-file parsing and planner construction from `callback_wall_us`. Each
timed iteration uses generation `300 + repeat` and a newly constructed lifecycle, so the fixed
geometry can be evaluated freshly without changing production algorithm semantics.

## W2: `W2_SCE018_ROS_NODE_TEST_ACTIVE`

Status: `BLOCKED_NO_TEST_ACTIVE_FRESH_REPLAY`.

The sole designated geometry is the exact W1 SCE018 event and expected selected digest above. W2
must run the actual `local_planning` node and executor, with no simulator, using production mode
`TEST_ACTIVE`. It cannot inherit the historical SHADOW command or describe SHADOW as production
parity.

The smallest neutral construction required is an external, retained event-to-ROS driver that:

1. converts all 185 reference waypoints, the exact obstacle fields, and the exact ego state into
   the production topic types without changing planner source or YAML;
2. gives every accepted cycle an explicit monotonic source identity and records the emitted bytes;
3. creates a documented fresh lifecycle between samples without changing `p3_mode` or obstacle
   geometry;
4. captures `/local_planning/live_runtime_profile` and the corresponding cycle diagnostic by
   callback sequence; and
5. proves 20 discarded warm-ups followed by 200 valid fresh callbacks in a non-performance smoke.

Whether that fresh lifecycle is created by process restart or by a deterministic external state
sequence is itself a scientific choice and is not resolved by retained evidence. No command is
claimed until that choice is made and validated.

## W3: `W3_S01_FULL_SIM_TEST_ACTIVE`

Status: `BLOCKED_MISSING_HISTORICAL_PATH_AND_DRIVER`.

The sole designated scenario is `S01_STRAIGHT_101` from
`planning_study/gqsc_s1_closed_loop_smoke_v1/smoke_scenarios.csv` (file SHA-256
`6b970bc319a774904156087a7b76f71ef584df59671d4e9b3927c3e4af082a92`). Its obstacle is centered
at `s=42.0`, spans `s=[41.78,42.22]`, `d=[-0.15,0.15]`, and historically selected RIGHT.

- Physics-map bytes retained:
  `src/kinematic_localization/maps/map_kissmap_render.pgm`, SHA-256
  `60f63b93168b7b3c94e5412355b804ec5c918cae7bc608aa8e8bbc49b7984109`
- Historical global path: `offline_trajectory_generator/config/output/map/global_waypoints.json`,
  documented SHA-256
  `bc2191cd1ddf32c8019bb57b4fcfecc5c04e44b10ff139aa7bc4ef845422f640`
- Current-state finding: the historical global-path file is not present and is not present in the
  recovered checkpoint object. Current
  `src/global_planning/data/ifac_track/global_waypoints.json` has SHA-256
  `b9cdd21fb7067ebd32b9ce2f4d2ffbd8affc96c5d2193eaba02892a772a16fad` and must not be substituted.
- Historical seed/noise: `12345/0.01 m`; historical speed cap: `2.0 m/s`
- Stack launcher identity: `sim/run.sh`, SHA-256
  `f7b5d1bafb011c30b51e74eb11179ac06d15fb29b090dcb9c4eec2a1a4a06f6d`

The old report records 332 callbacks and 39 fresh evaluations for S01, not 200. Across all four
smoke scenarios it records many fresh evaluations, but only 16 callbacks saturated 128/12/12.
Neither fact supplies the missing exact repeat driver. To unblock W3, the historical path bytes or
an independently authorized replacement lineage, exact simulator configuration/initial pose/TF
adapter, deterministic obstacle scheduler, collection-start predicate, and 200-fresh feasibility
smoke must all be retained and hashed before execution.
