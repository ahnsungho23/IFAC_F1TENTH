# Corrected property/result contract

## Generator-independent result semantics

The property oracle asks whether a conservative one-dimensional reachable corridor exists.  It
does not prove that a five-knot P3 exists (`test_rule_property.cpp`, `CorridorOracle::corridorExists`).
The planner-side predicate must therefore distinguish a planning result from candidate-generator
coverage:

```text
planner_ok := would_recover
           OR result == NO_OBSTACLE / "no static obstacle blocks the global race line"
```

`buildP3ShadowPlanningContext()` returns that reason when `nearestCluster()` is empty.  The
production result contract is `SplinePlanKind::kNoObstacle` with the same reason.  Hence the 101
cases in which the global line is not blocked require no local-avoidance candidate and are not
GQSC failures.  The observed wrapper reason `GQSC_V3_FROZEN_ENTRY` is a result-lineage/semantic
regression, not evidence that an operator, B128, or K12 missed a path.  This study does not change
that wrapper or weaken the test.

## Exact-positive contract

A case enters the method-coverage denominator only when an exact P3 is positively established by
one of the following:

1. frozen GQSC reconstructs a path and the unchanged exact hard validator accepts it; or
2. an audit-only reconstruction of an existing P3 factor tuple is accepted by that same validator.

The 83 corridor-only misses are labeled exactly `NO_EXACT_P3_WITNESS_CONFIRMED`.  Absence of a
witness is not a proof of P3-family impossibility, but those cases cannot be positive coverage
targets for this study.

## No-valid-side invariant

`RP2_51`, `RP2_238`, and `RP3_291` have no valid constant lateral side domain under the exact
geometry contract.  With no valid side, the direct transition set is empty by construction and
no factor pair may be materialized.  The independent 1D corridor oracle does not impose the same
constant-side/P3 constraint.  These three cases are therefore invariant exclusions, not requests
to manufacture an unsafe path.

## Denominators

- Confirmed-positive property denominator: `338 + 13 = 351`; frozen v3 covers `338/351`.
- Exact property failure-research denominator: `13` (Top-12 9, B128 2, lateral 2).
- Combined exact method-policy failure denominator: `19` = property 13 + production/raceline
  Top-12 1 + transition 5.
- `raceline_tight_gap` is an interface/parameter-contract case and is reported separately.

These are evidence-conditioned denominators, not a claim that the excluded 83 can never be
represented by P3.
