# GQSC-S1 live runtime qualification v1

## Decision

`WORKLOAD_PROTOCOL_FROZEN`

Binary provenance revision 2: `RUNTIME_BINARY_FROZEN`.

Protocol revision 2 gate: `PROTOCOL_R2_FROZEN`.

Operational revision 3 gate: `PRE_RESULT_EXECUTABLE_REPAIR_FROZEN`.

Protocol revision 2 freezes the pre-result decision, quantile, validity, affinity-evidence, and
system-state contract in `decision_contract.md`. The revision-2 execution attempt correctly stopped
before its first timing sample because the directly invoked W1 runner had Git mode `100644`.
Operational revision 3 changes no science: it repairs that mode to `100755`, adds a deterministic
filesystem/frozen-tree executable preflight, and freezes `executions/r3/` as the result namespace.

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
- Binary provenance revision 2, built from clean `research` commit
  `dc33b875a938fdb7730d35fe61de43ee863b04c0`, rejects the recovered unoptimized normal binaries
  and freezes a separate `-O3 -DNDEBUG` Release overlay for all three workloads. Workload identity,
  callback/sample/retry contracts, A/B order, parity rules, and threshold are unchanged.
- Execution attempt 0 started no scheduled run and collected zero timing callbacks because the
  exact final decision mapping and W1/W3 actual-affinity evidence were missing.
- Protocol revision 2 freezes the exact decision table, nearest-rank quantiles, negative W1/W2
  timing-scope-equivalence conclusion, run-validity precedence, actual PID/affinity evidence,
  AC/powersave state, and a synthetic-tested analysis script before timing.
- The revision-2 timing attempt stopped as `PRE_RUN_GATE_FAILED_BEFORE_TIMING`: direct W1 runner
  execution returned `permission denied`; no scheduled run, warm-up callback, measurement callback,
  or latency statistic occurred. Revision 3 preserves that evidence separately, repairs only the
  W1 executable mode, adds executable-mode preflight, and reserves the R3 raw/result namespace.

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
- Frozen W1 Release harness SHA-256:
  `49503d48d96cf408ad47691a69b683e5f2a0c02947a73283994f57b2697c0fd2`
- Frozen W2/W3 installed Release node SHA-256:
  `54019a86e13f4dc25771628f7a3d385be8e2c6ce2a657448b3a5939823ab878a`

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
`workload_manifest.csv`, `run_schedule.csv`, and `binary_provenance.md`.

Release-overlay validation additionally passed for W1, W2, and W3. The Release redacted W2/W3 join
SHA-256 values are `686b1556...02e8b` and `2fd8fe7e...32813`; their graph hashes are
`27100475...e1a4a` and `e647e3df...4272d`. No scientific latency output was retained.
