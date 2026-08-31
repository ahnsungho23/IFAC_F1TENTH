# Unified bounded-generator feasibility

## Current dependency

Native GQSC B128 is not presently an independent generator. It consumes lateral and transition
seeds collected while the strict legacy M0/M1 ladder is constructed and exact-validated. Therefore
the row named `C_GQSC_B128_ONLY` is an outcome counterfactual; its executable wall-time row still
includes legacy seed extraction. The isolated core timing is measured, but the missing direct seed
producer has not been implemented.

## Seen-only subsumption gaps

- LEGACY_SUCCESS/GQSC_FAIL diagnostic rows: 0
- taxonomy: {}
- M0-V1-prefix legacy-success regressions: 6
- M0-V1-prefix usable regressions: 5
- idealized GQSC core p95: 18.266 ms

## Fixed proposal-operator feasibility

No operator is implemented in this study. No fixed proposal operator is justified by full-GQSC subsumption on this seen corpus. For each missing-lateral case, one deterministic
production-selected-equivalent lateral anchor would cost at most seven extra pair proxies (the
fixed transition set size). A missing-transition case would cost at most the retained lateral count
in pair proxies. Any future operator must enter the same bounded proxy ranking and replace, not add
to, the existing Top-12 reconstruction/validator quota; the hard caps remain 12/12. B128 truncation
or ranking-displacement cases should be addressed by bounded ordering/coverage slots rather than by
raising K. These estimates are feasibility bounds, not validated method changes.

## Recommendation

`GQSC_BOUNDED_UNIFICATION_NEEDED`
