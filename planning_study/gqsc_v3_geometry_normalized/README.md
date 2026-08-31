# GQSC v3 geometry-normalized direct-operator redesign

## Result

`GQSC_V3_CLEAN_BUT_COVERAGE_TRADEOFF`

## Immutable closed-loop candidate freeze

`V3_SIDE_BALANCED_DISJOINT` is frozen as the geometry-general candidate that may proceed to a
separate closed-loop evaluation. This is not a claim of final paper-method superiority, and the
17/18 hard success-control result must not be repaired by tuning this frozen candidate.

- canonical method serialization: `gqsc_v3_geometry_general_method.json`
- canonical method SHA-256: `965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780`
- prevalidation lock SHA-256: `ea30a441695732e87d43d6863e1a31992dbfdd45749d324047495b6c96824185`
- DEVELOPMENT hard/usable: 24/49, 21/49
- VALIDATION_SEEN one-shot hard/usable: 20/37, 18/37
- success controls hard/usable: 17/18, 15/18
- native p95/p99/max: 16.598/20.431/20.470 ms
- pair/reconstruction/validator caps: 128/12/12

The canonical serialization includes every inherited lateral/transition operator in source order,
pair ordering, the side-balanced disjoint reserve, proxy and final ranking, tie breaks, exact-shape
and path-digest deduplication, P3 reconstruction, and exact-validator contract/source pins. It also
records that `COMPONENT_HALF_FAR002_INWARD015` and `MIN_OUTWARD` are excluded from this candidate.

The selected prevalidation-locked policy is `V3_SIDE_BALANCED_DISJOINT` with SHA-256 `ea30a441695732e87d43d6863e1a31992dbfdd45749d324047495b6c96824185`.
Selection used only 49 DEVELOPMENT events. Only after serialization as
`V3_PREVALIDATION_LOCKED` were the 37 VALIDATION_SEEN episodes and 18 success controls evaluated
once. No FINAL_HOLDOUT path was accessed.

## Why v2 was not carried forward

The rejected v2's `0.02 m` far offset, `0.15 m` inward offset, and MIN_OUTWARD choice were not
renamed or normalized. The Development deficiency audit found only one event (DVE039) whose entire
teacher hard/usable lateral set was absent, with no repeated normalized mechanism in another event.
Consequently v3 adds no lateral trajectory operator. MIN_OUTWARD's only hard distinction from MAX
was DVE023 and it added no usable event, so it was excluded as effectively event-specific.

## Protocol and outcomes

Four policies were predeclared in [v3_operator_candidates.md](v3_operator_candidates.md). The
Development comparison selected `V3_SIDE_BALANCED_DISJOINT` at 24/49 hard and
21/49 usable. The immutable one-shot result is 20/37 hard and
18/37 usable on VALIDATION_SEEN, plus 17/18 hard and
15/18 usable success controls. Frozen production-success baseline digest parity is
18/18.

All observed rows obeyed lateral<=64, transition<=7, B128, reconstruction<=12, and validator<=12.
Release standalone wall runtime p50/p90/p95/p99/max was
10.705/16.150/
16.598/20.431/
20.470 ms with research instrumentation disabled.

The Release unit suite passed 3/3, including frozen production identity/budgets and deterministic,
bounded standalone selection. The selected standalone callable consumes side geometry directly;
it receives no legacy candidate/seed and invokes no legacy builder or validator. Its policy enum is
handled only inside the research-only standalone selector, not the production
`selectP3R3K12Factors` decision path.

## Interpretation

The v3-specific selector is dimensionless and map-scale invariant, but its clean coverage must be
reported against the rejected v2 Development reference (27/49 hard, 23/49 usable).
`final_freeze_gate.md` records whether that tradeoff and the success-control result permit a final
freeze. Validation outcomes were not used for repair. Production `plan()`, P3 construction,
validator, ranking, lifecycle, fallback, and parameters were not changed by this research-only
study, and no closed-loop claim is made.
