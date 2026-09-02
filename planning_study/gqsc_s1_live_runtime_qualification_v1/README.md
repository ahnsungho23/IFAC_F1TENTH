# GQSC-S1 live runtime qualification v1

## Decision

`WORKLOAD_PROTOCOL_FROZEN`

Protocol revision 1 freezes one causal, same-input design before any latency A/B run:

| Workload | Timed runtime layer | Immutable planner input | State |
|---|---|---|---|
| `W1_SCE018_STANDALONE` | standalone algorithm; no ROS executor/simulator | canonical `SCE018.event` | `FROZEN_READY` |
| `W2_SCE018_ROS_NODE_TEST_ACTIVE` | actual production node/executor; no simulator | exact SCE018 ROS projection | `FROZEN_READY` |
| `W3_SCE018_FULL_STACK_CONTENTION` | the same production node/executor while the normal simulator/ROS stack runs | the same private SCE018 ROS projection | `FROZEN_READY` |

This decision means the workloads and future commands are exact and runnable. It is not an
authorization record for a completed experiment. No 5 x 200 A/B run was executed, no new p95/p99
was calculated, and no smoke timing value was retained or interpreted. The prohibited split was
not inspected.

## Revision history

- Revision 0, committed at authoring HEAD `99dcb24664a43f28bd04f7889071e6b966a76097`, recorded
  W1 ready and W2/W3 blocked. Its blockers were the absent `TEST_ACTIVE` fresh replay/lifecycle
  and the non-reproducible historical S01 full-stack driver. That blocked decision is historical
  evidence and is not erased by revision 1.
- Revision 1 uses the already frozen SCE018 input in all three layers. An external C++17 ROS
  replay/collector exercises the production public source-restart lifecycle and joins existing
  profile/diagnostic topics. W3 uses supported remapping so the full stack remains active without
  becoming a competing planner-input source.

## Frozen production identity

- Algorithm checkpoint: `55c61e54540fc07d57e6041655e0e59ddb8e17e8`; revision-1 authoring
  HEAD: `99dcb24664a43f28bd04f7889071e6b966a76097`
- Method: `LEX8_GLOBAL_DISJOINT_COVERAGE4`
- Method SHA-256: `670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776`
- Bounds: pair/lexicographic/coverage/reconstruction/validator `128/8/4/12/12`
- Planning period: `25 ms`; production mode: `TEST_ACTIVE`
- Planner configuration SHA-256:
  `4fe480351a80135ff2a6e4592f661ff8a5670c032e12554d85065339d16ea960`
- SCE018 SHA-256:
  `572adb59ea24f3f06bed7502eb870e57a33a5416e8630106b67bc2b1d0b405bf`
- Expected selected path digest: `c7b2c19bf2af9350`

No planner source, production YAML, algorithm parameter, controller, Oracle, or dataset label was
modified. Only protocol documents and external tools below this experiment directory changed.

## Non-performance validation

The revision-1 replay tool was built in Release mode in its ignored private build tree. Fixed
four-callback smokes passed for both W2 and W3. Every joined row had a distinct source epoch,
`TEST_ACTIVE`, `r3_invoked=true`, `fresh_evaluation_count=1`, `128/12/12`, candidate count 12,
candidate `GQSC_S1_MAIN_COVERAGE_LEFT_c7b2c19bf2af9350`, digest `c7b2c19bf2af9350`,
`FRESH_SELECTED/FRESH_HARD_VALID_P3_M1`, owner `P3_M1`, and no safe stop.

- W2 redacted join SHA-256:
  `6342393165476f3d7b32482d9ede867061045b4a0eff6386a972c77cf29e5f4b`
- W2 graph SHA-256:
  `96f58425bf21fe930d834ad26da14da492a1d42af5189b5f6d2711e13074edba`
- W3 redacted join SHA-256:
  `1e37cbcfd44db4cb9fe93fa152603fdd0d2a318a754113d65c65745fbcd32610`
- W3 graph SHA-256:
  `bc612bf449dff3fa1a62cfdcd70ad37d84c017d73834974e430fab374ea445e2`

The retained validation files contain identities, counts, outcomes, and graph endpoints only; they
contain no numeric latency result. Binding details are in `workload_freeze.md`,
`execution_protocol.md`, `fresh_callback_contract.md`, `parity_contract.md`,
`workload_manifest.csv`, and `run_schedule.csv`.
