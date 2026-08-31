# GQSC v3 final freeze gate

Decision: `GQSC_V3_CLEAN_BUT_COVERAGE_TRADEOFF`

- prevalidation lock: `V3_PREVALIDATION_LOCKED`
- lock SHA-256: `ea30a441695732e87d43d6863e1a31992dbfdd45749d324047495b6c96824185`
- selected policy: `V3_SIDE_BALANCED_DISJOINT`
- DEVELOPMENT hard/usable: 24/49, 21/49
- rejected v2 DEVELOPMENT hard/usable reference: 27/49, 23/49
- VALIDATION_SEEN one-shot hard/usable: 20/37, 18/37
- success controls hard/usable: 17/18, 15/18
- frozen production-success baseline digest reconstruction parity: 18/18
- all B128/K12/validator-12 bounds pass: true
- native p95 < 25 ms gate: true (16.598 ms)
- native p99 < 25 ms preference: true (20.431 ms)
- deterministic selector/bound test: PASS (`test_p3_r3_k12`, 3/3 tests)
- hidden legacy execution in standalone selector: none; the selected callable accepts frozen side
  geometry rather than legacy seeds, and its detail stream contains no standalone seed lineage or
  legacy builder/validator call
- production decision-path gate: PASS; v3 policy branches are confined to the research-only
  `selectP3R3RTStandaloneFactors()` call path and the frozen identity/budget unit test passed
- recommend immutable method freeze: false

The post-lock validation/control result was not used to repair or reselect the policy. The clean v3
rule introduces no event/map-specific offset, but it is not promoted to a final frozen method when
it gives up the rejected v2's Development coverage or fails the full 18/18 hard control gate.
