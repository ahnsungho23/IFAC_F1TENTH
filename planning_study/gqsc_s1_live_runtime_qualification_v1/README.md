# GQSC-S1 live runtime qualification v1: immutable workload protocol

## Decision

`WORKLOAD_PROTOCOL_BLOCKED`

This directory defines the future runtime experiment without running it. W1 has an exact,
pre-existing input and an executable standalone procedure. W2 and W3 each have one designated
workload, but neither has the complete pre-existing procedure required to call it immutable.
The A/B experiment must not start until both blocking gaps are removed and this decision is
re-audited.

No planner source, production configuration, commit, CPU policy, or simulator state was changed.
No new latency dataset or latency comparison was collected. Workloads were chosen from semantics,
provenance, determinism, and reproducibility only; historical latency values were not used for
selection. The prohibited data split was not inspected.

## Verified checkpoint

- Branch: `research`
- HEAD: `55c61e54540fc07d57e6041655e0e59ddb8e17e8`
- Method: `LEX8_GLOBAL_DISJOINT_COVERAGE4`
- Frozen method SHA-256: `670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776`
- Bounds: pair proxies `128`, lexicographic `8`, disjoint coverage `4`, reconstruction `12`,
  validator `12`
- Planning period: `25 ms`
- Production mode: `TEST_ACTIVE`
- Planner configuration SHA-256:
  `4fe480351a80135ff2a6e4592f661ff8a5670c032e12554d85065339d16ea960`

The recovered S1/P3/R3 core source blobs match checkpoint
`ca621fd9d7c7aa0e3d30ee9c0e43a1599b98da79`. The current build is discoverable after sourcing the
overlays, but `build/local_planning/CMakeCache.txt` has an empty `CMAKE_BUILD_TYPE`; this task did
not rebuild or label that build as Release.

## Runtime layers and designated rows

| Workload | Layer | Designated input | State |
|---|---|---|---|
| `W1_SCE018_STANDALONE` | algorithm isolated; no ROS executor or simulator in timed work | canonical `SCE018.event` | `FROZEN_READY` |
| `W2_SCE018_ROS_NODE_TEST_ACTIVE` | production ROS node/executor; no simulator | SCE018-equivalent ROS messages | `BLOCKED` |
| `W3_S01_FULL_SIM_TEST_ACTIVE` | simulator plus required production ROS stack | historical S01 straight obstacle | `BLOCKED` |

Historical `ROS_NODE_ISOLATED/SCE018` means a SHADOW-mode run and is not the W2 definition above.
Historical closed-loop labels mean simulator-loaded runs and map to W3 only when their full input
and startup lineage is retained.

## Exact blockers

1. W2 has no retained event-to-ROS replay command or message artifact. More importantly, the
   retained 40-sample run used `SHADOW`, which evaluates eagerly. The frozen `TEST_ACTIVE` node
   holds a hard-valid committed suffix without invoking the evaluator, so publishing the same
   SCE018 input repeatedly does not establish 200 fresh callbacks. A neutral external lifecycle
   reset/replay construction and its callback-to-digest join must be implemented and validated.
2. W3's historical S01 geometry and physics map remain documented, but its global path
   `offline_trajectory_generator/config/output/map/global_waypoints.json` is absent from the current
   tree and absent at the recovered checkpoint. Only its historical SHA-256 is retained. The exact
   obstacle scheduler, initial-pose/TF adapter commands, collection-start command, and raw replay
   artifact are also absent. The current `ifac_track` path is not an authorized substitute.

## Unblock gate

The decision can change to `WORKLOAD_PROTOCOL_FROZEN` only after both blocked rows have an exact,
reviewed command, retained input bytes with hashes, a demonstrated `TEST_ACTIVE` fresh-callback
mechanism capable of 20 warm-up plus 200 valid samples per repeat, and a callback-linked parity
record. A tiny wiring smoke may then be run, but it must not be summarized as performance and must
not influence workload selection.

The binding details are in `workload_freeze.md`, `execution_protocol.md`,
`fresh_callback_contract.md`, `parity_contract.md`, `workload_manifest.csv`, and
`run_schedule.csv`.
