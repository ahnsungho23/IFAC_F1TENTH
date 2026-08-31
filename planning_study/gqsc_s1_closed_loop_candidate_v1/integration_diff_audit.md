# S1 integration diff audit

## Adopted method delta

The only candidate-selection change from immutable v3 is:

```text
lexicographic quota: 10 -> 8
coverage reserve:    side-balanced 2 -> one global disjoint call selecting 4
total K:             12 -> 12
```

The S1 options select the pre-existing `DISJOINT_COVERAGE` implementation. The v3 lateral
operators, seven transition families, proposal order, wave-based B128 materialization, proxy
ranking, floating-point ties, exact shape/path dedup, P3 reconstruction, exact validator, and final
hard-valid rank are inherited by the v3 serialization SHA and are not redefined.

The effective-parameter binding is a separate interface repair. It replaces hard-coded
`0.15/0.08/0.04` cheap-corridor dimensions with the evaluator's active parameter object. When the
LUT-speed envelope closes the cheap midpoint, the same already-existing minimum-speed envelope
used by the side-domain contract is used for cheap geometry; every constructed path still goes
through the unchanged exact validator. Canonical seen inputs have the original values and produced
the frozen aggregate 62 hard / 54 usable exactly.

## Explicit exclusions verified

- No B1/B2 pair materialization rule enters the production selector.
- No legacy exact-validator-steered transition bisection executes.
- No new lateral or transition operator was added.
- `COMPONENT_HALF_FAR002_INWARD015` remains only as an older research option and S1 sets it false;
  its unit and 104-snapshot traces show no execution.
- M0/M1 and exact-R3 research adapters remain explicit only; observed production legacy seed max
  is zero.
- Pair/reconstruction/validator maxima are 128/12/12.
- `src/local_planning/config`, controller, perception, localization, message/topic interfaces,
  validator, lifecycle, fallback, and safe-stop parameters have no S1-specific change.

## Result/reason and parameter contract repairs

For a nonblocking input, the GQSC wrapper now preserves
`no static obstacle blocks the global race line` rather than leaking the frozen-entry lineage
label. The property test already treats this as a successful no-planning result. The tight-gap
parameterized test now uses the same effective geometry contract in side selection and cheap GQSC
geometry and passes without changing its parameters or weakening validation.

## Evidence boundary

Only the four explicitly enumerated seen corpus directories in `run_s1_integration_study.py` were
used. There is no discovery fallback. FINAL_HOLDOUT contents and the large closed-loop benchmark
were not accessed or executed.
