# Predeclared bounded selection-policy candidates

## Lock boundary

This document was written after diagnosing the ten exact-proven Top-12 displacement cases and
before executing the selection-policy ablation.  The policies receive only the frozen B128 factor
records and their existing cheap proxy fields.  They do not receive exact-validator outcomes,
case identifiers, dataset roles, witness values, or path digests.  All policies preserve exact
shape deduplication, the frozen proxy comparator and `K=12`.

The diagnosis motivating the candidates is structural: the ten witnesses occupy proxy ranks well
below the frozen lexicographic quota, seven use a transition family absent from the selected set,
and the frozen side-balanced reserve degenerates on one-sided geometry into two independent
one-item coverage calls.  Each one-item call resets its local diversity state.  The alternatives
therefore study reserve allocation and transition-family coverage; they do not encode any named
event.

## P0 — `V3_FROZEN_LEX10_SIDE2`

Unmodified frozen v3 reference: ten shape-deduplicated lexicographic factors followed by one
right-side and one left-side coverage choice where available, then general fill.  This is an
immutable comparator, not a v4 candidate.

## S1 — `LEX8_GLOBAL_DISJOINT_COVERAGE4`

1. Take the first eight shape-deduplicated factors in frozen lexicographic order.
2. Exclude those shapes.
3. Invoke the existing disjoint coverage objective once over all remaining factors with quota
   four.  Its local max-distance state therefore spans all four reserve choices.
4. Fill any unavailable slot by frozen lexicographic order.

The `8+4=12` split is `COMPUTATIONAL`: it reallocates the fixed K12 budget symmetrically from the
frozen `10+2`; it is not a metric threshold or an event-derived value.

## S2 — `LEX8_TRANSITION_STRATIFIED_COVERAGE4`

1. Take the first eight shape-deduplicated factors in frozen lexicographic order.
2. For each of four reserve slots, first restrict eligibility to transition families not yet
   represented in the selected set, when such a family exists.
3. Within the eligible set, use the existing disjoint coverage objective and frozen stable tie
   order.
4. If every available transition family is already represented, remove the restriction and
   continue the same coverage objective.

There is no prescribed transition-family name or transition value.  The only new numeric value is
the `COMPUTATIONAL` four-slot reserve inherited from the `8+4=K12` partition.

## S3 — `LEX6_GLOBAL_DISJOINT_COVERAGE6`

The same single global disjoint-coverage call as S1, with a symmetric `6+6=12` split.  This tests
whether the failure is irreducibly caused by too little reserve rather than by side splitting.
The split is `COMPUTATIONAL`; it is deliberately more coverage-heavy and must pass non-regression
and runtime gates rather than being assumed preferable.

## Rejection gate

A policy is rejected if it exceeds B128/K12/12 validators, changes any reconstructed factor or
validator contract, loses an exact hard/usable success relative to P0 on the allowed seen controls,
or requires event-conditioned behavior.  No policy is frozen by this study.
