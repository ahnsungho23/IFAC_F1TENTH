# Parameter/interface reconciliation

## What the values mean

- `vehicle_half_width_m`: physical half-width of the rectangular vehicle footprint.
- `safety_margin_m`: additional obstacle lateral clearance; it is not vehicle width.
- `wall_safety_margin_m`: additional footprint-to-track-wall clearance.

The frozen GQSC v3 geometry contract serializes `0.15 / 0.08 / 0.04`.  The canonical operational
`local_planning.yaml` also sets `vehicle_half_width_m=0.15`, `safety_margin_m=0.08`, and
`wall_safety_margin_m=0.04`; therefore the main operational YAML is consistent with the frozen
numbers.  The node declaration default for safety margin is `0.05`, but the canonical launch YAML
overrides it.  `local_planning_sim.yaml` is a distinct configuration (`0.15 / 0.014789... / 0.10`).

`testParameters()` in `test_raceline_spline.cpp` intentionally constructs a different validator
contract (`0.12 / 0.03`) and enables the tracking-reserve mechanism.  `raceline_tight_gap` passes
under those test parameters in the legacy planner, while hard-coded frozen geometry produces no
side geometry.  The test is therefore useful evidence of a parameter-interface violation; it
should not be “fixed” by tuning an operator or by changing the test to the frozen numbers.

## Recommendation (not implemented)

Keep frozen v3 immutable.  A future method interface should accept the evaluator's effective
`RacelineSplineParameters` (including wall margin and reserve mode) when constructing geometry,
and its canonical serialization should specify this parameter binding rather than silently embed
one operational profile.  Add explicit parity tests for canonical YAML, node defaults, and the
simulation profile.  This is an interface/configuration recommendation, not a v4 method result.
