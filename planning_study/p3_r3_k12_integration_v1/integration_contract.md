# Frozen R3-K12 integration contract

## Authority

- Method: `R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12`
- Frozen method SHA-256:
  `7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc`
- Authoritative offline specification:
  `../p3_geometry_factor_ranking_v2/selected_v2_method_spec.json`
- Integrated selector: `src/local_planning/src/p3_r3_k12.cpp`
- Recovery wiring and exact reconstruction/validation:
  `src/local_planning/src/p3_shadow.cpp`

## Runtime decision flow

```text
strict production P3 ladder
  |-- hard-valid success --> return the unchanged production result
  `-- failure
       relaxed production P3 ladder
         |-- hard-valid success --> return the unchanged production result
         `-- failure
              frozen R3 factor selection
                |-- lexicographic stream: at most 10 constructed paths
                `-- coverage-reserve stream: at most 2 constructed paths
                      |
                      exact existing validator once per unique path digest
                      |-- >= 1 hard-valid --> existing downstream rank/lifecycle
                      `-- 0 hard-valid --> unchanged pre-R3 fallback/safe-stop
```

Production success returns before R3. It therefore performs zero R3 path reconstruction and zero
additional R3 validator calls.

## Frozen selector and budget

The integrated selector preserves the offline expanded lateral-factor definitions, transition
factor definitions, factor ordering, lexicographic proxy rank, coverage ordering, exact floating
point ties and final tie-break strings. The selector receives only online-generatable geometry and
the production candidate factors; it does not receive oracle labels or FINAL_HOLDOUT outcomes.

The feasibility-ranked stream quota is 10, the coverage-reserve quota is 2 and the total selected
factor budget is 12. A construction guard failure does not consume a quota slot. A successfully
constructed path does consume a slot. A repeated path digest also consumes that selected-factor
slot, but the already-known exact verdict is reused and the exact validator is not called again.
Consequently:

```text
R3 constructed paths <= 12
R3 exact-validator calls <= unique constructed digests <= 12
```

## Acceptance and downstream ownership

R3 reconstructs the existing five-knot P3 spline family with the selected
`(side, d_target, d_mid, entry_scale, exit_scale)` factor. It does not alter corridor extraction,
vehicle parameters, speed shaping, collision geometry or validator thresholds. Every unique path
is measured and judged by the existing exact `validateCandidate()` authority.

Hard-valid R3 paths are ordered by the frozen seven-key post-validation tuple. The selected path is
then exposed through the same `P3ShadowResult` fields consumed by the existing planner/lifecycle.
The existing downstream comparator remains unchanged. R3's frozen ordering is represented by its
stable `final_rank` audit tie-break so equal-within-production-epsilon candidates retain frozen
order without changing M0/M1 production ranking.

`USABLE_VALID_P3` remains a research diagnostic: hard-valid, no next-obstacle exit conflict and
non-positive braking deficit within the existing rank epsilon. It does not weaken or replace the
hard validator.

## Instrumentation contract

Research instrumentation remains default OFF. When enabled, `R3_RECOVERY` evaluation records
method/SHA, factor-pool counts, 10+2 stream selections, construction and digest-dedup counts,
validator/hard-valid/usable-valid counts, selected factor lineage, runtime and fallback after R3
failure. These fields are write-only observations and are not planner inputs.
