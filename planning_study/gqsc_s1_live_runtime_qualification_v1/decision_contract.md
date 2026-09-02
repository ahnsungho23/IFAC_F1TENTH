# Protocol revision 2 — final decision and execution-validity contract

## Status and scope

This is a pre-result protocol revision. Scientific timing callbacks collected before this revision:
`0`; scheduled qualification runs executed before this revision: `0`. The immutable revision-1 tag
`gqsc_s1_live_runtime_qualification_v1` remains historical and must not move.

Revision 2 changes no workload, binary, source, configuration, method, event, lifecycle, sample
count, schedule, retry limit, affinity target, deadline, or parity value. It freezes previously
missing timing-scope interpretation, quantiles, decisions, run-validity precedence, affinity
evidence, system state, and analysis code.

History remains explicit:

- revision 0: W1 ready; W2/W3 blocked;
- revision 1: same-input W1/W2/W3 workload protocol frozen;
- binary provenance gate: common Release-equivalent overlay frozen;
- execution attempt 0: stopped before timing because the decision mapping and W1/W3 actual-affinity
  evidence were incomplete;
- revision 2: this decision, quantile, validity, affinity, and system-state contract.

## Timer-scope audit

All three timers use `std::chrono::steady_clock`, a monotonic wall-runtime clock independent of ROS
or simulator time. Values are emitted in microseconds. Source inspection found that the frozen
binaries serialize `std::chrono::duration<double, std::micro>` values, not integer storage. The
analysis therefore parses the emitted numeric token exactly as a decimal microsecond value; it
does not round, truncate, interpolate, or coerce to integer. Presentation alone uses
`latency_ms = latency_us / 1000.0`.

### W1 primary diagnostic interval

Authoritative field: `callback_wall_us`, field 10 under 1-based TSV indexing in each
`GQSC_MAIN_TIMING` row (C++ zero-based field index 9 after splitting the complete row).

- clock: `std::chrono::steady_clock`;
- start: immediately before `evaluateP3Shadow` in
  `test/p3_r3_k12_integration_harness.cpp`;
- end: after either fresh `selectFresh` lifecycle selection or the fallback call;
- included: one SCE018 P3 evaluator invocation plus its fresh lifecycle selection; the fallback
  branch is included if the evaluator does not recover;
- excluded: event-file reading, planner construction, reference setup, ROS/executor dispatch,
  production snapshot preparation, obstacle ingress/conversion, ROS message construction and
  publication, live-profile serialization/publication, and process startup;
- warm-up: 20 identical evaluations before 200 emitted measurement rows;
- units/source: decimal microseconds from monotonic steady wall time.

### W2/W3 primary production interval

Authoritative field: `stage_us.O_total`, retained by the replay as `o_total_us`.

- clock: `std::chrono::steady_clock`;
- start: the first statement on entry to `LocalPlannerNode::onPlanningTimer`;
- end: callback-scope cleanup after the selected planning branch returns;
- included: live-profile setup overhead, obstacle-ingress drain/conversion, snapshot capture and
  locks, evaluator, lifecycle/rank, result/path message construction, and publications performed by
  the planning callback, including production diagnostics reached before callback return;
- excluded: scheduler/executor waiting before callback entry, event/replay preparation outside the
  node, live-profile JSON construction/publication performed after `O_total` is sampled, and work
  outside the planning callback;
- units/source: decimal microseconds from monotonic steady wall time, identical in W2 and W3.

### Equivalence conclusion

`W1_W2_TIMING_SCOPE_EQUIVALENT = FALSE`.

W1 remains algorithm-isolated diagnostic evidence. C1/C2 numerical differences may be reported as
descriptive cross-layer observations, but they must not be called pure ROS node/executor overhead,
must not be used as cases D/E of `RUNTIME_ENVIRONMENT_CONFIRMED`, and must not contribute a strong
effect to `RUNTIME_ENVIRONMENT_PARTIAL`. W2→W3 and within-workload A→B retain matched metric scope.

## Exact descriptive statistics

Only valid `phase=MEASUREMENT` callbacks are used. Every repeat must have exactly `n=200`; each
workload-condition pool must have exactly `n=1000` across its five repeats.

Sort values ascending and use the nearest-rank empirical quantile with no interpolation:

`Q(p) = x[ceil(p*n)]`, with one-based indexing.

- repeat `n=200`: p90=`x[180]`, p95=`x[190]`, p99=`x[198]`;
- pooled `n=1000`: p90=`x[900]`, p95=`x[950]`, p99=`x[990]`.

For even `n`, median is the arithmetic mean of the two center observations. Mean is the arithmetic
mean. `std` is frozen as population standard deviation,
`sqrt(sum((x-mean)^2)/n)`, because these values describe the complete retained empirical run/pool,
not an estimator with `n-1`. Tail counts use strict `>` comparisons at 25, 30, and 40 ms; fractions
are the count divided by the applicable `n`.

## Six workload-condition deadline decisions

For each of W1-A, W1-B, W2-A, W2-B, W3-A, and W3-B:

`DEADLINE_PASS = TRUE` iff both:

1. pooled nearest-rank p95 is `<= 25.000 ms`; and
2. at least 4 of 5 repeat nearest-rank p95 values are `<= 25.000 ms`.

Otherwise `DEADLINE_PASS = FALSE`. There is no grey zone.

## Repeat direction, crossing, and strong effect

For matched repeat numbers R1..R5:

- `REPEATABLE_IMPROVEMENT(X->Y)`: `p95(Y) < p95(X)` in at least 4/5 repeats;
- `REPEATABLE_WORSENING(X->Y)`: `p95(Y) > p95(X)` in at least 4/5 repeats;
- equality counts toward neither.

A repeatable PASS→FAIL or FAIL→PASS deadline crossing is decision-relevant by its applicable exact
rule below. When there is no deadline crossing, an effect is `STRONG` iff direction is repeatable
and the pooled p95 relative change is at least `10.0%`, using X as denominator. This is descriptive,
not a significance test. No other effect threshold is permitted.

The only predeclared contrasts are:

- C1 W1→W2 under A — descriptive only because timing scope is not equivalent;
- C2 W1→W2 under B — descriptive only because timing scope is not equivalent;
- C3 W2→W3 under A;
- C4 W2→W3 under B;
- C5 W1 A→B;
- C6 W2 A→B;
- C7 W3 A→B.

## Production deployment decision

`PRODUCTION_RUNTIME_PASS` iff the entire validity/parity/provenance/affinity/isolation gate passes
and W3-B `DEADLINE_PASS` is true. Otherwise `PRODUCTION_RUNTIME_FAIL`. W3-A is reported separately;
W1/W2 do not gate the absolute W3-B production decision.

## Final environment classification and precedence

Apply exactly this precedence.

### 1. `EXPERIMENT_INCONCLUSIVE`

Use if any of these occurs:

- not all 30 scheduled identities obtain one valid final run;
- an allowed single infrastructure retry also fails, or more than one retry would be needed;
- exact callback count is not reached;
- mandatory parity fails for any otherwise eligible measurement callback;
- binary/source/config/method/event identity changes;
- actual affinity cannot be verified;
- input isolation fails;
- timing metric/scope is violated;
- protocol changes after timing begins;
- a scientific sample must be discarded for a non-preregistered reason;
- result integrity cannot be established.

Preserve all raw data and issue no CONFIRMED/PARTIAL/NOT_SUPPORTED category.

### 2. `RUNTIME_ENVIRONMENT_CONFIRMED`

With a valid experiment, use if at least one of these exact cases holds:

- CPU placement recovery: W3-A FAIL, W3-B PASS, and C7 repeatable improvement;
- full-stack failure under A: W2-A PASS, W3-A FAIL, and C3 repeatable worsening;
- full-stack failure under B: W2-B PASS, W3-B FAIL, and C4 repeatable worsening.

The W1/W2 scope audit was negative, so the optional ROS cases D/E are disabled. This classification
and the separate production PASS/FAIL decision must both be reported.

### 3. `RUNTIME_ENVIRONMENT_PARTIAL`

If no CONFIRMED rule matches, use when at least one classification-eligible predeclared contrast
has no deadline crossing but has a STRONG effect: at least 10.0% pooled p95 change with the same
direction in at least 4/5 matched repeats. C1/C2 are not classification-eligible.

### 4. `RUNTIME_ENVIRONMENT_NOT_SUPPORTED`

With a valid experiment, use when neither CONFIRMED nor PARTIAL matches. This includes all relevant
conditions already passing, or failures without a preregistered repeatable qualifying crossing or
eligible strong effect.

## Parity and infrastructure failure handling

Any otherwise eligible measured callback that violates mandatory SCE018 parity is retained, marked,
and stops the complete experiment immediately as `EXPERIMENT_INCONCLUSIVE`. It is never discarded.
Stale, duplicate, continuation-only, or non-fresh callbacks remain ineligible input and are counted
separately rather than treated as parity failures.

An infrastructure-invalid attempt includes launch/node/graph/replay failure, timeout before exact
count, wrong affinity, input-isolation failure, or power-source change. Preserve its directory and
reason. At most `RERUN1` is allowed for that same scheduled identity and condition. Latency, an
outlier, or poor p95 never permits retry. A failed retry stops the experiment as inconclusive. The
revision-1 workload-level cap is preserved: if more than one repeat in the same workload would
require `RERUN1`, stop that workload and classify the complete experiment inconclusive.

## Run-level affinity evidence

Every final run must retain `affinity.txt` before accepting measurement callbacks. It records actual
PID, expected and actual executable, command line, `/proc/<PID>/status` CPU fields,
`taskset -pc`, and `ps -o pid,psr,comm,args`.

- A planner/harness allowed list must be exactly `0-23`;
- B planner/harness allowed list must be exactly `8`;
- W1 starts the future harness behind a stopped pre-exec gate, verifies the same PID's inherited
  affinity before `exec`, then records the actual frozen harness executable identity;
- W2 resolves and records the actual installed-node child PID before starting replay; it separately
  verifies the replay child on `0-7,10-23`;
- W3 resolves the actual installed-node PID and records every current member of each experiment
  background process group on `0-7,10-23` before starting replay; it separately verifies the replay
  child on that same background mask.

W2/W3 replay has a fixed three-second preflight hold, so planner and replay affinity evidence is
accepted before deterministic measurement inputs begin. Any evidence failure invalidates the run.

## System-state freeze

Frozen host state before timing:

- CPU: Intel Core i7-14650HX;
- kernel: `6.8.0-138-generic`;
- CPU 8 governor: `powersave`;
- `isolcpus`/`nohz_full`: absent;
- power source: `AC`, verified pre-result by `on_ac_power` exit status 0;
- topology: CPU 8/core 4, SMT sibling CPU 9, parent mask `0-23`.

Every run records start/end UTC time, `on_ac_power`, governor, `/proc/loadavg`, kernel, relevant
processes, and affinities in `system_context.txt`. AC and `powersave` must remain unchanged. A power
source change invalidates the run under the single-retry rule. Governor, power policy, realtime
priority, kernel parameters, SMT, and topology must not be changed. Temperature is descriptive only
and never a retry/cherry-pick reason without an explicit hardware failure.

## Frozen analysis and manifest

Analysis entry point: `tools/analyze_qualification.py`.

It consumes a result `run_manifest.csv` with this exact column order:

`workload_id,repeat_id,condition,attempt_id,final_attempt,run_valid,parity_pass,affinity_pass,input_isolation_pass,binary_identity_pass,metric_contract_pass,result_integrity_pass,invalid_rejected_count,raw_dir,retry_reason`

Every failed attempt remains a row. Each scheduled identity has ATTEMPT0 and at most one RERUN1,
with exactly one declared final attempt. Rows must follow the exact 30-row `run_schedule.csv` order;
an allowed RERUN1 immediately follows its failed ATTEMPT0. The script reads W1 `timing.tsv` and W2/W3
`joined_callbacks.jsonl` directly, validates exact counts/order/parity fields, and produces callback,
repeat, pooled, contrast, and decision artifacts. Any missing identity, failed gate, malformed raw
file, or incomplete cell has validity precedence and yields `EXPERIMENT_INCONCLUSIVE`.

The script implements nearest-rank quantiles, even median, population std, six deadline decisions,
matched-repeat direction, no-crossing 10% effects, production PASS/FAIL, and final precedence. Its
synthetic-only self-test covers quantile indices, the 4/5 rule, all final classification families,
and the validity override. Historical runtime samples are not used.

Analysis script SHA-256: `92321d50542004236de9154a9098e5e0ebc4897d3c61531d7fd4c73160e455db`.
