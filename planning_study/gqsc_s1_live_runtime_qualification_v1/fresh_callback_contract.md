# Fresh-callback contract

## Authoritative definitions

The node schema is `gqsc_s1_live_fresh_runtime/1`. A node callback is eligible only when its
profile has `fresh_evaluation_count > 0`; the observer increments this count only when the result
reports `r3_invoked`. The authoritative callback identity is the cycle diagnostic tuple:

`(source_epoch, source_stamp_ns, obstacle_sequence, global_reference_generation, evaluation_sequence, callback_sequence)`.

`callback_sequence` joins the cycle diagnostic to the live profile. A profile without a unique
matching cycle record is invalid. Source identity, not arrival order in a collector file, decides
duplicate/stale status.

The ROS-node primary latency is `stage_us.O_total`: steady-clock time from entry to
`onPlanningTimer()` through callback work and publication calls, ending before profiler
serialization/publication. `evaluator_wall_us` is a secondary compute-only diagnostic and must not
replace `O_total` in the primary qualification.

## New generation, duplicate, and stale rules

- A new generation is an accepted evaluator invocation for a new cycle identity, not merely a new
  timer tick or a republished message.
- Exact repeated source stamps are not a new generation. In lockstep mode, obstacle and odometry
  stamps must match and be greater than the last processed stamp.
- Regressing/stale source input is rejected and does not count. A callback that only continues a
  committed hard-valid suffix has no fresh evaluation and does not count.
- Reusing the same geometry is allowed only when the workload's frozen driver creates a valid new
  identity and the production lifecycle actually invokes the evaluator.
- A collector must never turn duplicate/rejected callbacks into retries by renumbering them.

## W1

The harness is not a ROS callback, so its equivalent fresh identity is the fixed event hash plus
the harness generation `300 + repeat`. Each loop creates a fresh evaluation and, on recovery, a
new lifecycle before `selectFresh`. Warm-up generations are `280` through `299`; measured
generations are `300` through `499`.

The W1 primary interval starts immediately before `evaluateP3Shadow` and ends after the fresh
lifecycle selection or fallback completes. It is emitted as `callback_wall_us`. Input parsing,
planner construction, and stdout formatting are outside the interval.

## W2

W2 must use the node definition above in `TEST_ACTIVE`. The historical SHADOW sequence is not an
eligible substitute because SHADOW eagerly evaluates the snapshot even when production authority
would continue an existing maneuver. The missing deterministic mechanism for producing 220
eligible fixed-SCE018 TEST_ACTIVE callbacks per repeat is a blocking gap.

## W3

W3 must use the same node definition and tuple. Simulator timer callbacks with
`fresh_evaluation_count == 0`, startup/not-ready callbacks, and committed-suffix continuation are
not samples. Because ego state evolves, geometry/digest parity must be joined to the exact cycle
identity rather than inferred from callback number. The missing exact scheduler and source lineage
are blocking gaps.

## Validity and failure inclusion

An eligible sample is valid only if all of the following hold:

1. workload/config/method/binary preflight identities match;
2. the record is uniquely joined to the authoritative generation tuple;
3. `fresh_evaluation_count == 1` for the selected workload protocol;
4. primary latency is finite and non-negative;
5. complexity counters are within `128/12/12` and satisfy the workload-specific expectation;
6. no duplicate, stale, startup, collector-loss, schema, or timeout fault occurred; and
7. the complete raw record and success/failure classification are retained.

A planner failure with a valid invoked fresh evaluation is included in latency and separately
classified. Only instrumentation/input/protocol invalidity excludes a sample. Invalid samples and
all reruns remain in the audit trail. No outlier rule, latency threshold, or post-hoc trimming rule
is permitted.
