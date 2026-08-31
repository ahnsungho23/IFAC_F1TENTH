# GQSC standalone bounded direct-seed generator study v1

## Decision

`GQSC_STANDALONE_COVERAGE_INSUFFICIENT`

The prototype is genuinely standalone and bounded, and its usable result is promising, but it does
not meet the minimum coverage contract: legacy hard-success coverage is 17/18,
not 18/18, and it misses 5 teacher-hard events. It must not be frozen or connected
to production decisions.

## Scope and protocol

This is a seen-only research/shadow study over 86 frozen production-failure snapshots plus 18
legacy-success controls. Dataset roles are exactly `['DEVELOPMENT', 'PILOT_SEEN_DEVELOPMENT_DATA', 'VALIDATION_SEEN_AFTER_V1']` plus the existing
success-control role. FINAL_HOLDOUT paths, contents, outcomes, and statistics were not accessed.
The Release C++ harness ran all 104 inputs in one warm process with research instrumentation OFF,
2 warmups and 10 measured repeats per snapshot. Production `plan()` never calls the standalone
method.

## What the legacy-seeded teacher actually consumes

The audit contains all 748 legacy-derived seed tuples: stage counts {'M0_V1': 337, 'M1': 303, 'M0_V2': 108}, root
counts {'ACTIVE_OUTER': 467, 'ZERO_INTERFACE': 270, 'ALL_INACTIVE': 11}, and semantic counts {'M0_V1_ANALYTIC_PROBE_ROOT': 337, 'ZERO_INTERFACE_DIRECT_ANCHOR': 270, 'ANALYTIC_ACTIVE_OUTER_ROOT': 130, 'ANALYTIC_ALL_INACTIVE_ROOT': 11}. Source inspection plus the recorded
dataflow establishes a narrower dependency than the legacy template labels suggest:

- GQSC reads only each seed's `d_target`, `entry_scale`, and `exit_scale`.
- It does **not** read legacy `d_mid`, `s_probe`, `d_probe`, root value/index/branch, template hard
  verdict, or validator diagnostic.
- Therefore analytic-root and zero-interface labels are historical provenance, not required numeric
  inputs to B128. The required numeric semantics reduce to the side-domain/grid and component target
  anchors plus the bounded transition-range anchors reconstructed in
  [direct_operator_definitions.md](direct_operator_definitions.md).

The legacy-seeded teacher remains the existing reference: 62/104 hard and 51/104 usable, including
18/18 legacy hard controls and 44/86 legacy-failure hard recoveries. Its 18 success-control selected
digests match the preceding unified-architecture artifact exactly (18/18), so the standalone work did
not alter teacher semantics.

[legacy_seed_dependency.csv](legacy_seed_dependency.csv) preserves every tuple and marks whether its
target/transition value supports observed hard or usable teacher candidates. Those flags are
descriptive support, not a claim that an individual duplicate seed is causally unique.

## Coverage comparison

| corpus/contract | legacy | legacy-seeded teacher | standalone direct seed |
|---|---:|---:|---:|
| all 104 hard | 18 | 62 | 60 |
| all 104 usable | 14 | 51 | 54 |
| 18 legacy controls hard | 18 | 18 | 17 |
| 18 legacy controls usable | 14 | 15 | 15 |
| 86 legacy failures hard recovered | 0 | 44 | 43 |
| 86 legacy failures usable recovered | 0 | 36 | 39 |

All 14/14 legacy-usable control paths remain covered by at least one
standalone usable path. Selected digest agreement against the legacy selection is
teacher 0/18 and standalone 0/18; coverage is therefore
not path-identity preservation.

Among 57 events where both methods return hard-valid paths, selected digest agreement is
23/57. Standalone hard gains beyond the teacher occur at `DVE003;DVE022;DVE028`;
teacher-hard losses occur at `DVE023;DVE039;VUE009;VUE014;SCE008`. These gains do not compensate for the
18/18 non-interference failure. Full per-event hard/usable set intersections are in
[teacher_vs_standalone.csv](teacher_vs_standalone.csv).

## Miss diagnosis

Teacher-valid/standalone-invalid contract rows: 6; taxonomy {'TOP12_RANKING_MISS': 4, 'LATERAL_FACTOR_MISS': 2}. The main
remaining failure is not an unbounded search: direct factors are either absent or displaced by the
changed standalone B128/Top-12 pool. See [miss_taxonomy.csv](miss_taxonomy.csv). This v1 deliberately
stops rather than tuning more operators on the same seen outcomes.

## Bounds and runtime

All 208 per-event/method bound rows pass: lateral <=64, transition families <=7,
pair proxies <=128, reconstruction <=12, validator <=12. Standalone rows consume zero legacy seeds
and call no legacy builder/validator internally.

Standalone executable wall p50/p90/p95/p99/max is
10.898/17.817/20.754/
21.878/24.152 ms. The current legacy-seeded teacher's isolated
GQSC core p50/p95/p99/max in the same run is 7.811/
18.505/21.417/23.656 ms; its executable
wall additionally includes legacy seed construction/validation. Full breakdown is in
[native_runtime.csv](native_runtime.csv).

## Static path-quality interpretation

[path_quality_shadow.csv](path_quality_shadow.csv) records every hard-valid shadow candidate's track
and obstacle margins, braking deficit, normalized slack, mean lateral deviation, maximum curvature,
curvature rate, point count, and maneuver station length. These are static exact-validator/shadow
diagnostics only; no closed-loop robustness claim is made.

## Conclusion

The direct geometry operators prove that legacy path construction and analytic root harvesting are
not intrinsically required to execute bounded GQSC. However, v1 fails the frozen-method candidate
coverage gate, so the correct outcome is `GQSC_STANDALONE_COVERAGE_INSUFFICIENT`. No production decision path, validator,
ranking, P3 geometry, parameter, lifecycle, or fallback was changed, and no commit/push was made.
