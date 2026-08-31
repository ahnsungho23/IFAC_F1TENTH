# R3-K12 exact Top-K enumeration optimization v2

## Outcome

Frozen `R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12` output remains exact on all combined-seen snapshots:
ordered feasibility Top-10, coverage Top-2, final Top-12, path digests, validator verdicts, hard/usable
counts, final selected digest, and downstream plan are 86/86 identical with instrumentation OFF and
ON. Production-success non-interference is 18/18, and construction/validator maxima remain 12/12.

The retained v2 changes reduce instrumentation-OFF R3 p95 from `132.73 ms` to `77.25 ms` and max
from `263.35 ms` to `166.17 ms`. Whole-evaluator p95 is `115.39 ms`. The 25 ms goal is not met:

**EXACT_FROZEN_R3_NOT_REALTIME_FEASIBLE**

This classification applies to the current exact implementation and frozen factor space on the
measured host/corpus. It does not claim that every conceivable future exact implementation is
mathematically impossible; it states that no additional safe lazy pruning was proved, and the
observed unique-profile tail remains far above the cycle budget.

## What was optimized

- Exact five-station basis reuse replaces target/index keyed duplication.
- Exact profile-equivalence reuse scores
  `(side,d_target,d_mid,stations[0..4])` once while retaining every transition row for coverage.
- Feasibility selection uses the proven-equivalent shape representative plus exact partial Top-10.
- Coverage comparison caches the current incumbent priority but preserves greedy order.
- The 71k-row hot pool is numeric/compact; source and Python-compatible strings are materialized only
  for selected rows.
- Corridor/reference invariants are cached once per station basis.
- Proxy work remains deterministic with at most four contiguous workers.

Across 86 events, full proxy evaluations fell from 776,193 to 498,506, a 35.78% exact reduction.
The pool itself remains 776,193 configurations because coverage semantics require distinct
entry/exit rows even when their reconstructed profile is identical.

## Why an exact global lazy Top-K was not used

The lateral catalog order is based on source priority, while leading rank fields are joint quintic
corridor/slope/curvature measurements. They are not monotone in lateral or transition index.
Coverage additionally uses a global base-score minimum followed by a selected-dependent diversity
term. The available universal bounds are too weak to exclude an unevaluated region.

The maximum event VUE019 is decisive: `4,471 x 16 = 71,536` raw combinations, 30 production
configurations excluded, 71,506 remaining configurations, and 71,506 unique profiles. Therefore
the exact equivalence rule skips zero full proxies there. Approximate cutoffs or factor reduction
would change the scientific method and were not used.

See [rank_structure.md](rank_structure.md) for the exact tuples and
[exact_pruning_proof.md](exact_pruning_proof.md) for the accepted/rejected proofs.

## Performance

Measurements use the same 86 immutable combined-seen failure events, Release harness, one process
per event, and linear-interpolated percentiles as v1. Instrumentation-OFF is production-equivalent;
this is deterministic offline replay, not a ROS executor/DDS/closed-loop benchmark.

| Scope | p50 | p90 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|
| R3 core, OFF | 13.82 ms | 53.76 ms | 77.25 ms | 119.65 ms | 166.17 ms |
| Whole evaluator, OFF | 25.71 ms | 83.31 ms | 115.39 ms | 146.88 ms | 206.48 ms |
| Pair proxy, OFF | 4.71 ms | 27.45 ms | 50.31 ms | 85.07 ms | 116.06 ms |
| Exact validation, OFF | 1.98 ms | 5.08 ms | 5.97 ms | 8.08 ms | 12.06 ms |
| R3 core, instrumentation ON | 13.91 ms | 53.58 ms | 76.41 ms | 120.54 ms | 162.25 ms |

The tail is caused by profile count, not validation: factor-pool p95 is 35,783.25 and unique-profile
p95 is 27,518.25. Exact validator p95 is only 5.97 ms.

[before_after_runtime.csv](before_after_runtime.csv) contains all phase percentiles,
[factor_pool_complexity.csv](factor_pool_complexity.csv) contains all 86 per-event counts,
[worst_case_breakdown.csv](worst_case_breakdown.csv) contains the 12 largest pools, and
[factor_evaluations_before_after.csv](factor_evaluations_before_after.csv) records total reductions.

## Frozen contract and verification

- Method SHA-256:
  `7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc`
- The frozen JSON was re-hashed and matches that value.
- `src/local_planning/config` has zero diff from `p3_r3_k12_functional_integration_v1`.
- Package tests are 25/25 pass after sourcing the ROS/project overlay, excluding only the two frozen
  known mismatch checks (`safety margin 0.05 vs 0.08`, `right steering 0.361 vs 0.41`).
- No method, quota, K, tie, dedup, validator, P3 geometry, final rank, lifecycle, fallback, or vehicle
  parameter changed.
- The eight-worker and basis-grouping trials were parity-correct but performance-rejected and are
  absent from the final implementation.
- No large closed-loop benchmark was run. No commit or push was made.

## Reproduction

```bash
python3 src/local_planning/test/check_p3_r3_k12_parity.py \
  --harness <release-build>/p3_r3_k12_integration_harness --jobs 4

LOCAL_PLANNING_RESEARCH_PARITY=1 \
python3 src/local_planning/test/check_p3_r3_k12_parity.py \
  --harness <release-build>/p3_r3_k12_integration_harness --jobs 4

python3 src/local_planning/test/profile_p3_r3_k12_runtime.py \
  --harness <release-build>/p3_r3_k12_integration_harness \
  --output /tmp/r3_k12_v2_profile.csv

python3 planning_study/p3_r3_k12_exact_topk_optimization_v2/generate_tables.py \
  --before /tmp/r3_k12_profile_v1.csv \
  --after /tmp/r3_k12_v2_profile.csv \
  --output planning_study/p3_r3_k12_exact_topk_optimization_v2
```
