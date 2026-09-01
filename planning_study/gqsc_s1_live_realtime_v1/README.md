# GQSC-S1 live fresh-generation real-time qualification v1

## Result

`GQSC_S1_LIVE_RUNTIME_ENVIRONMENT_LIMITED`

The frozen `LEX8_GLOBAL_DISJOINT_COVERAGE4` method was not changed. Profiling shows that the
same bounded 128/12/12 query can run below 25 ms in a warm Release harness and above 25 ms when
scheduled on an efficiency core or inside the ROS/simulator workload. The requested nominal live
fresh-generation p95 qualification is therefore **not** claimed. No algorithm optimization was
applied to compensate for host scheduling or contention.

## Frozen scope

- Frozen S1 SHA-256: `670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776`
- Reference v3 SHA-256: `965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780`
- Pre-smoke checkpoint: `534e5e63fdc19146c24887f697ed6b9f869d2347`
- Frozen method serialization and selection/reconstruction/validator source are unchanged.
- Final holdout data was not accessed.

The only production-source additions are default-OFF research timing observation and a harness
stage-reporting mode. When enabled, the profiler emits a companion `std_msgs/String` record after
the measured callback interval. Research JSON/string serialization is excluded from `O_total` and
reported separately.

## A-O timing contract

| Stage | Measured work |
|---|---|
| A | callback/state snapshot preparation |
| B | obstacle ordering/conversion |
| C | corridor/free-space and immutable reference geometry |
| D | lateral proposal generation |
| E | transition proposal generation |
| F | bounded pair materialization/proxy evaluation |
| G | lexicographic 8 plus disjoint coverage 4 selection and dedup |
| H | P3 reconstruction |
| I | exact validation |
| J | final rank and lifecycle ownership outside evaluator wall time |
| K | ROS output-message construction |
| L | publication calls |
| M | measured planner mutex wait |
| N | residual callback work not assigned above |
| O | callback wall time before profiler serialization/publication |

`live_stage_runtime.csv` records the full A-O distributions for 40 stable node-isolated samples
and the one new closed-loop full-construction callback. The SHADOW node-isolated `O_total` also
contains the unchanged P0 safety-cycle work; `evaluator_wall` and C-I isolate GQSC.

## Profile-first finding

The strongest controlled comparison is an actual previous live-smoke query, callback 221808:

- live input: ego `(s,d,v)=(34.8393,-0.23648,0)`, one centered obstacle at `s=42.0 m`;
- work: 444 reference samples, 62 lateral proposals, 7 transition proposals, 128 pair proxies,
  12 P3 reconstructions, 12 validators, 1,398 reconstructed points;
- selected path digest: `d18331e97cdba596` in both live execution and the reconstructed
  standalone query;
- standalone warm Release, 100 repeats: wall p50/p95/p99/max
  `16.856/17.006/17.073/17.113 ms`;
- recorded live evaluator/callback: `27.802/30.904 ms`.

The reconstructed event reproduces the reference snapshot digest, ego snapshot digest, candidate
set outcome, and selected path digest. Its compact oracle-event format does not preserve every ROS
obstacle-message field, so its obstacle snapshot identifier differs; this is a geometry/output
exact timing comparison, not a claim of full message-lineage identity.

In the live evaluator, pair proxy, reconstruction, and exact validation consumed
`5.013 + 5.209 + 13.309 = 23.531 ms` (84.6% of evaluator wall). The inexpensive stages did not
explain the gap. The 128/12/12 counts were identical, so the slow case did not materialize extra
candidates or validators.

## CPU-placement diagnostic

The same frozen SCE018 query was repeated 100 times without changing the scheduler globally:

- unpinned: p95/p99/max `22.228/23.339/23.549 ms`;
- diagnostic performance-core CPU 8: `22.008/22.418/22.769 ms`;
- diagnostic efficiency-core CPU 22: `30.952/31.089/31.233 ms`.

All candidate counts and decisions remained exact. Pair/reconstruction/validation medians changed
from `2.947/5.088/11.816 ms` on CPU 8 to `5.017/7.102/16.799 ms` on CPU 22. During the independent
ROS-node run, `local_planner_node` was observed on logical CPU 22. That node produced callback p95
`30.824 ms` even without the simulator, confirming that simulator computation is not required for
the tail to appear. The host is a hybrid i7-14650HX: logical CPUs 0-15 advertise 5.0-5.2 GHz
maximum frequency, while 16-23 advertise 3.7 GHz.

This diagnostic used `taskset` only to establish causality. No affinity, realtime priority,
scheduler policy, or OS setting was changed for the planner deployment.

## Instrumentation overhead

Across 1,040 allowed-seen warm evaluations, turning the existing research instrumentation on
changed evaluator p50/p95 from `11.149/20.023 ms` to `11.298/20.125 ms`. This approximately
`0.15/0.10 ms` difference is far smaller than the 8-11 ms environment gap. The new live profiler
is default OFF; its own serialization starts after `O_total`.

## Optimization decision

No GQSC hot-path optimization was applied. The profile-supported bottlenecks are the frozen
bounded pair/P3/validator computations themselves running more slowly under core placement and
contention, not duplicated proposal semantics, locking, ROS conversion, or research formatting.
The task contract explicitly forbids changing GQSC to compensate when the same query is below
25 ms natively and above 25 ms only in the execution environment.

## Small live re-qualification

A single deterministic 2.0 m/s straight-obstacle maneuver was run with research instrumentation
OFF and the default-OFF A-O profiler explicitly ON:

- one full S1 generation: evaluator `20.829 ms`, callback `22.387 ms`;
- complexity: 1 obstacle, 480 stations, 62 lateral, 7 transition, 128 pair, 12 reconstruction,
  12 validator, 1,571 reconstructed points, selected size 141;
- selected fresh maneuver committed, continued, completed, and returned to `STATE_GLOBAL`;
- collision was false throughout the measured maneuver; no safe-stop entry was observed;
- existing non-hard publish-feasibility acceleration warnings were retained and were not used to
  tune the method.

After control shutdown, before the simulator process was stopped, the simulator later latched a
collision on the stale last drive command. That teardown-order event is excluded from the measured
maneuver and is not presented as a planner pass/fail observation.

One fresh sample below 25 ms cannot qualify p95 or p99. The immutable preceding 16-callback live
fresh-construction sample remains the tail evidence: callback p50/p95/p99/max
`25.690/33.421/33.476/33.489 ms`. Across all 4,300 active callbacks in that smoke, continuation
kept p50/p95/p99/max at `0.431/1.492/1.562/30.854 ms`.

## Parity gates

- allowed-seen exact parity: 104/104;
- hard/usable: 62/54;
- sequential lifecycle replay: 62/62;
- raceline parameterized tests: 88/88;
- production scenarios: 10/12, with the same known LayoutB/pinch coverage limits;
- pair/reconstruction/validator maxima: 128/12/12;
- hidden legacy seeds: 0;
- two complete parity runs had identical SHA-256 output
  `17510e5fe76583251218b6d2ffba998a7c0a53ba2d750c338c0d056693922415`.

All applicable tests outside the two known parameter/control mismatches and the two known
production-coverage cases passed. The isolated test initially could not write under `~/.ros` in
the sandbox; it passed after setting `ROS_LOG_DIR` under `/tmp`. No test logic was changed.

## Interpretation

The fresh S1 method is functionally bounded and deterministic, but live tail latency is sensitive
to heterogeneous-core scheduling and host GUI/simulator/ROS contention. A one-off live callback
can meet 25 ms, while p95/p99 cannot be guaranteed on this host without an execution-environment
qualification step. That future step may evaluate deployment CPU isolation or scheduler design,
but those system changes are outside this task and were not performed here.

