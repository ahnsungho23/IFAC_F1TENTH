# Fresh-callback contract — revision 1

## Authoritative identity and join

The production schemas are `gqsc_s1_live_fresh_runtime/1` and
`local_planning_p3_cycle/1`. A ROS callback is eligible only when the profile has
`fresh_evaluation_count == 1`, which production increments only for `r3_invoked`. The authoritative
identity fields are:

`(source_epoch, source_stamp_ns, obstacle_sequence, global_reference_generation, callback_sequence)`.

`callback_sequence` must uniquely join one live-profile record to one cycle diagnostic. The cycle
diagnostic does not expose the internal evaluation sequence. The external collector therefore emits
`derived_evaluation_sequence` as the strictly increasing ordinal of uniquely joined callbacks after
requiring `fresh_evaluation_count == 1`; it is explicitly not represented as an internal node
counter.

Each joined row also carries `r3_invoked`, `stage_us.O_total` presence/value according to output
mode, pair/reconstruction/validator counts, candidate count and provenance, selected digest,
`fresh_hard_valid`, lifecycle state/reason, owner, safe-stop state, and the exact ego/obstacle
scalars. The existing outputs are sufficient; no planner instrumentation was added.

## Production-faithful new generation

In `TEST_ACTIVE`, the planner tries `continueCurrent` first. A valid committed suffix can therefore
produce no fresh evaluation even after another identical obstacle arrives. Revision 1 uses the
smallest verified public recovery transition already implemented by `acceptObstacles`:

1. publish the exact SCE018 obstacle with a higher source stamp;
2. publish the same geometry with a stamp 1 second lower;
3. production recognizes a regression of at least 0.5 second as source restart/bag-loop recovery,
   advances `source_epoch`, resets the lifecycle/selection envelope, and accepts the message;
4. the next qualifying planning callback uses the unchanged SCE018 reference, ego, and obstacle.

All generated stamps are unique: for restart index `i`, the high stamp is
`canonical + (i+1)*2 s`, followed by `high-1 s`. The driver never sets source epoch, obstacle
sequence, reference generation, callback sequence, selected candidate, or algorithm state. A normal
small timestamp regression that does not cross the production restart threshold is not eligible.

## W1

The standalone harness equivalent identity is fixed event SHA-256 plus harness generation. Exactly
20 warm-up evaluations precede 200 measurement evaluations. Each iteration constructs the existing
fresh lifecycle without altering geometry. Expected values are `128/12/12`, hard/usable `9/9`,
`FRESH_SELECTED`, and digest `c7b2c19bf2af9350`.

## W2 and W3

The replay collector requires exactly 220 unique joins and 220 distinct observed source epochs:

- joined ordinals 1–20: `phase=WARMUP`, retained but excluded from measurement;
- joined ordinals 21–220: `phase=MEASUREMENT`, indices 1–200;
- any additional eligible callback is a protocol violation;
- profile/diagnostic loss, duplicate join, missing field, parity failure, timeout, or nonzero process
  exit fails the attempt.

W2 and W3 use the same driver, topics, QoS, event bytes, source-stamp schedule, and join. W3 changes
only the concurrent process environment. Simulator timer callbacks and normal perception messages
cannot become samples because the qualified planner inputs are private remaps with one driver
publisher each.

## Output modes

- Smoke: `smoke_mode=true`; numeric timing and raw profile/diagnostic JSON are not emitted. The row
  records only that `O_total` exists and sets `timing_redacted=true`.
- Future qualification: `smoke_mode=false`; the raw joined record retains `O_total` and both source
  JSON records. Revision 1 froze but did not execute this mode.

Only protocol/input/instrumentation invalidity excludes a sample. A valid invoked planner failure
remains a measured and classified outcome. No latency cutoff, outlier deletion, or post-hoc sample
selection is allowed.
