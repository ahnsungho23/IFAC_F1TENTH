# GQSC production-ladder subsumption and unified architecture study v1

## Scope and contracts

This is an offline/shadow study of exactly 104 already-seen independent evaluator
snapshots: 86 frozen production failures and 18 exact-lineage legacy-success controls. It uses one
warm Release process, research instrumentation OFF, 3 warmups and 10 measured repetitions per
snapshot/architecture. FINAL_HOLDOUT_UNSEEN contents, outcomes, paths, and statistics are not read
or used. Production `plan()`, validator, ranking, lifecycle, fallback, and parameters are unchanged.

`USABLE_VALID` means exact hard-valid, no next-obstacle exit conflict, and braking deficit <=
1e-9 m. The tables distinguish selected-usable from existence of any usable candidate.

## Main result

Hard-valid coverage matrix over all seen snapshots:

- LEGACY_SUCCESS / GQSC_SUCCESS: 18
- LEGACY_SUCCESS / GQSC_FAIL: 0
- LEGACY_FAIL / GQSC_SUCCESS: 44
- LEGACY_FAIL / GQSC_FAIL: 42

The current forced GQSC shadow is not a true standalone generator: it still obtains seed factors
from the legacy strict ladder. Thus coverage can establish candidate-family subsumption on the seen
snapshots, but cannot establish deployable GQSC-only latency.

## Legacy ladder and the 40-70 ms tail

Fresh strict/relaxed passes are timed separately in [legacy_stage_runtime.csv](legacy_stage_runtime.csv).
The largest stage p99 is `RELAXED / M1` at
23.847 ms (max 25.256 ms). Independent `.event` snapshots
contain no active maneuver record, current raw snapshot distinct from the evaluator input, or node
fallback state. Therefore active continuation, guarded/raw continuation revalidation, and node
safety/fallback bookkeeping are explicitly `NOT_PRESENT_IN_STATELESS_EVALUATOR_CORPUS`; no zero-ms
runtime is falsely claimed for them.

The 40-70 ms full-evaluator tail is compound rather than a single GQSC stage: legacy-only p99 is
39.591 ms, the isolated GQSC core p99 is
21.733 ms, and sequential p99 is
55.103 ms. Within the legacy passes, M1 is the largest
tail stage; strict and relaxed passes can both execute before GQSC. The measured B sequential
full-evaluator wall is p50/p95/p99/max
11.075/
42.074/
55.103/
76.738 ms. The table separates this from the isolated GQSC
core and from the current seed-extraction wall.

## Architecture comparison

| architecture | hard success | usable exists | legacy hard regressions | legacy-failure hard recoveries | p95 ms | p99 ms | executable meaning |
|---|---:|---:|---:|---:|---:|---:|---|
| A legacy only | 18 | 14 | 0 | 0 | 31.430 | 39.591 | measured |
| B legacy then GQSC | 62 | 50 | 0 | 44 | 42.074 | 55.103 | measured |
| C GQSC-only outcome | 62 | 51 | 0 | 44 | 41.926 | 54.831 | wall includes legacy seeds |
| C isolated GQSC core | - | - | - | - | 18.266 | 21.733 | not standalone executable |
| D M0-V1 then GQSC | 34 | 31 | 6 | 22 | 19.113 | 22.804 | measured research shadow |

Among 18 legacy hard successes, the first selected stage is M0-V1/M0-V2/M1 =
9/7/2. Forced GQSC finds a
hard-valid path in all 18, but selects the exact same path digest in only 0/18;
subsumption here means validity coverage, not identical geometry or ranking. D loses
6 legacy hard successes and retains
only 22 of B's
44 legacy-failure recoveries.

The 25 ms target is a soft planner-cycle target. A row passes p95/p99 only when the measured wall
column is below 25 ms; no hard real-time guarantee follows from this offline process.

## Subsumption failure diagnosis

[gqsc_subsumption_failures.csv](gqsc_subsumption_failures.csv) contains 0 contract-level
failure rows and checks exact production lateral, transition, B128 pair, Top-12 placement, and path
digest reconstruction. Lifecycle-specific causes are not inferable from independent snapshots.

## Recommendation

`GQSC_BOUNDED_UNIFICATION_NEEDED`

This is the single recommendation for this study. It is not a production integration decision.
