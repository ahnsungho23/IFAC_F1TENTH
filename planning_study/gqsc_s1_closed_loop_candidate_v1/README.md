# GQSC S1 closed-loop candidate freeze and integration

## Final result

`GQSC_S1_RUNTIME_BLOCKER`

The frozen S1 selection policy is integrated exactly and satisfies the stateless, lifecycle,
determinism, no-hidden-legacy-seed, and B128/K12/validator-12 gates. It also reproduces the locked
allowed-seen result of 62 hard-valid and 54 usable-valid events. The full preserved callback,
however, has warm Release p95 43.208 ms, so it does not meet the approximately 25 ms planning-cycle
gate. The conditional low-speed closed-loop smoke was therefore not run.

## Frozen identity

| Item | Value |
|---|---|
| Method | `LEX8_GLOBAL_DISJOINT_COVERAGE4` |
| S1 canonical SHA-256 | `670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776` |
| Immutable v3 base SHA-256 | `965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780` |
| Pair proxy / reconstruction / validator cap | `128 / 12 / 12` per evaluation |
| Ordered streams | lexicographic 8, global disjoint coverage 4 |

The composed serialization is `gqsc_s1_method.json`; `gqsc_s1_method.sha256`, CMake configure,
the compile-time header, and `test_gqsc_s1_frozen_contract.py` pin the same bytes. The immutable v3
serialization was not overwritten.

## What changed

Only the clean S1 selection refinement was adopted: the first stream is shortened from 10 to 8,
and the existing global disjoint coverage selector accumulates four shapes in one call. No B1/B2
materialization, validator-steered bisection, operator, event constant, path family, validator,
rank, or dedup rule was introduced.

Two pre-integration contract defects were repaired separately:

1. nonblocking evaluator results retain the actual no-obstacle reason instead of the wrapper-entry
   label;
2. cheap GQSC geometry reads the effective evaluator parameters and uses the already-existing
   minimum-speed envelope only when the strict LUT envelope closes its midpoint.

The parameterized tight-gap and nonblocking-reason tests both pass. This interface repair is not a
GQSC operator or data-driven tuning change.

## Exact parity and bounds

The reproducible script enumerates only 9 pilot-seen, 40 development, 37
`VALIDATION_SEEN_AFTER_V1`, and 18 success-control snapshots: 104 total. On all 104:

- lateral, transition, and pair proposal order matched the frozen S1 research call;
- ordered 8+4 Top-12 matched;
- reconstructed path digests and validator verdicts matched;
- selected path and downstream `plan()` selected digest matched;
- method SHA and exact snapshot lineage matched;
- hidden legacy seed count was zero;
- observed maxima were 128 pair proxies, 12 reconstructions, and 12 validators.

Aggregate hard/usable event counts are 62/54, matching the pre-integration S1 lock. Detailed rows
are in `frozen_parity.csv` and `bounded_contract.csv`.

## Lifecycle and evaluations per callback

All 62 hard-valid snapshots passed fresh ownership, immutable continuation, obstacle dropout,
forward trim, completion, and owned-interval blocker invalidation. See
`lifecycle_transition_report.md` and `sequential_replay.csv`.

Default-OFF research instrumentation now reports `gqsc_s1_evaluation_count`. In the callback audit,
62 events used one S1 evaluation, 40 used two, and two used three. Additional evaluations are
distinct safe-stop/fallback input snapshots; same-input reuse remains cached. B128/K12/12 is an
evaluation-local bound, not a hidden callback-global claim.

## Warm Release runtime

Research logging was disabled. After two warmups, 104 snapshots were measured seven times each
(728 samples).

| Scope | p50 | p90 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|
| S1 evaluator | 11.092 ms | 17.368 ms | 19.742 ms | 20.758 ms | 21.023 ms |
| Full callback model | 16.861 ms | 39.239 ms | 43.208 ms | 55.006 ms | 55.968 ms |

The evaluator meets 25 ms in this harness; the full callback does not. This task does not optimize
fallback/safe-stop or change the frozen method to hide the tail.

## Regression interpretation

The S1-specific contract, selector, raceline parameter, instrumentation, and lifecycle tests pass.
The historical production-scenario suite improves from v3's 9/12 to 10/12 tests, but
`LayoutBReplanRecovers` and `SamePinchGeometryAtTwoDistances` remain frozen method-coverage misses.
They are not research/integrated parity failures and were not tuned away.

FINAL_HOLDOUT contents were not inspected. No large benchmark, closed-loop smoke, commit, or push
was performed.

## Reproduction

```bash
colcon build --packages-select local_planning --cmake-args -DCMAKE_BUILD_TYPE=Release
python3 planning_study/gqsc_s1_closed_loop_candidate_v1/run_s1_integration_study.py
ROS_LOG_DIR=/tmp/gqsc_s1_tests ctest --test-dir build/local_planning --output-on-failure \
  -R 'gqsc_(v3|s1)_frozen_contract|test_p3_r3_k12|test_p3_maneuver_lifecycle|test_raceline_spline|test_research_instrumentation'
```
