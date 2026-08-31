# Predeclared fixed-B128 materialization candidates

This document predates execution of the B128 ablation.  Both exact-proven B128 misses require an
already-existing `LONG_ENTRY_SHORT_EXIT` transition paired with an already-existing lateral
factor.  In each case the pair is excluded solely because the frozen transition-wave traversal
reaches the B128 cap first.  No new transition value, lateral operator, factor score, or budget is
introduced.

## B0 — `FROZEN_OPERATOR_PREFERRED_WAVES_B128`

The immutable v3 reference: build each lateral factor's operator-specific transition order, then
materialize wave 0 for all laterals, wave 1 for all laterals, and so on until B128.

## B1 — `PRIMARY_THEN_LONG_SHORT_STRIPE_B128`

1. Materialize each lateral factor's first operator-preferred transition in the frozen lateral
   order.
2. Materialize `LONG_ENTRY_SHORT_EXIT` once for each lateral in the same order, skipping exact
   configuration duplicates.
3. Resume the frozen operator-preferred wave traversal, skipping configurations already emitted,
   until exactly B128 or pool exhaustion.

The named transition is an existing geometry-derived regime, not a copied numeric transition.
The evidence is repeated across both independent B128 misses.  `128` is the frozen
`COMPUTATIONAL` budget.

## B2 — `SHORT_SHORT_THEN_LONG_SHORT_STRIPES_B128`

1. Materialize `SHORT_ENTRY_SHORT_EXIT` for each lateral in frozen lateral order.
2. Materialize `LONG_ENTRY_SHORT_EXIT` for each lateral in the same order.
3. Resume frozen operator-preferred waves with exact-configuration deduplication until B128.

This is a stronger categorical stratification control.  It discards the operator-specific primary
wave only where that primary is not the short-short regime, so it must pass exact non-regression
and runtime gates.  It does not prescribe an entry/exit number or use validator outcomes.

Neither policy raises B128.  If neither recovers the two exact misses without regression, this
study will report that fixed-B materialization was not cleanly repaired; it will not increase B.
