# Legacy bisected-transition derivation

## Source trace

The five production/raceline witnesses come from the baseline M0-V1 entry search in
`P3ShadowEvaluator::runQuery()` (`src/local_planning/src/p3_shadow.cpp`), not from a hidden
geometry formula and not from GQSC.

1. `entryScales()` starts with `entry_transition_fractions` and appends
   `detection_lookahead_m / pre_apex_distances_m.front()`.
2. `exitScales()` starts with `transition_distance_scales`; its extended maximum is obtained by
   requesting 1.5x and 2.0x the current effective post-apex length, clipping by
   `track_length - cluster_end - max(post_merge_lookahead, |v| post_merge_min_time) - 1e-3`, and
   converting the retained length back to a scale.
3. M0-V1 tests the shortest and longest entry scales.  Only if the short candidate fails with
   curvature/rate/slope (`NEEDS_LONGER_RAMP`) and the long candidate fails with track/footprint
   bounds (`NEEDS_SHORTER_RAMP`) does it establish a bracket.
4. It fixes exit to the shortest of three exit samples and evaluates
   `e_mid = (e_low + e_high)/2`.  The exact validator's first failure reason moves the lower or
   upper bound.  A hard-valid result ends the bisection; an unrelated failure ends it without a
   solution.

Thus “bisected” means **validator-steered interval bisection of entry scale**.  It does not mean
half of obstacle length or half of available longitudinal distance.

## Equations and observed witnesses

For these inputs:

```text
e_min = 0.51458109301505117
e_max = 15.0 / 11.442220427651225 = 1.310934367577036
e(q)  = e_min + q (e_max - e_min)

x_probe = x_min + (1/64) (x_max - x_min)
x_min   = 0.49716841625749453
```

| case | accepted q | entry scale | reconstructed x_max | exit scale |
|---|---:|---:|---:|---:|
| layoutB_failing_f0 | 1/2 | 0.91275773029604357 | 5.610479179325978 | 0.57706389693043958 |
| layoutB_failing_f1 | 7/16 | 0.86298565063591948 | 5.620145938712355 | 0.57721494004585172 |
| layoutB_failing_f2 | 1/2 | 0.91275773029604357 | 5.603035970386543 | 0.57694759679076091 |
| pinch_failure_f0 | 1/2 | 0.91275773029604357 | 5.618653251557724 | 0.57719161680906061 |
| pinch_failure_f1 | 1/2 | 0.91275773029604357 | 5.613234121212045 | 0.57710694289740938 |

The near-equal exit numbers are outputs of the general low-biased exit sample, not copied
constants: `x_max` changes with the side/horizon geometry.  The entry values are dyadic bisection
points of the same configured interval.  Which dyadic point is accepted is decided by exact
candidate construction and validation against the reference, corridor, footprint, obstacles,
curvature, rate, and slope constraints.

## What geometry does and does not determine

Obstacle front/rear and the side domain determine the P3 station geometry; cluster end, speed,
track length, post-merge tail, and outside multiplier also affect the exit interval.  However,
source does not provide a closed-form geometry-only map from those quantities to the accepted
entry dyadic point.  The missing information is the validator oracle used inside the loop.

`[INFERENCE]` A learned or analytic surrogate might predict the active constraint boundary, but
that would be a new method assumption.  It is not derivable from the current source contract.
