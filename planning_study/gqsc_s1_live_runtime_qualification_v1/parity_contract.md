# A/B and cross-layer parity contract — revision 1, binary provenance revision 2

## Common invariants

Between A/B, all five repeats, and W1/W2/W3, hold constant the canonical event bytes, frozen method
and configuration, `128/8/4/12/12` policy, costs, guards, safety thresholds, `TEST_ACTIVE` mode,
planning period, warm-up/sample/retry rules, and success/failure classification. Only the runtime
layer differs W1→W2→W3, and only planner affinity differs A→B within one workload.

Binary provenance revision 2 binds W1 to Release harness SHA-256 `49503d48...0fd2` and binds both
W2 and W3 to the same installed Release node SHA-256 `54019a86...878a`. The harness translation
unit, node main, and shared `local_planner_core` all use `-O3 -DNDEBUG`. This adds build identity
without changing any functional parity rule.

The host is x86_64. `local_planning::pathDigest` hashes the raw IEEE-754 bytes of the ordered selected
waypoints and emits 16 lower-case hexadecimal digits. Because the replay creates the exact `map`
reference/ego/obstacle fields and performs no coordinate conversion, the binding cross-layer digest
is exactly `c7b2c19bf2af9350`; no tolerance rule is used.

## W1

Every measured row requires `128/12/12` and `FRESH_SELECTED`. Untimed parity before and after each
A/B pair requires digest `c7b2c19bf2af9350`, hard/usable `9/9`, failure `NONE`, matching integrated
and downstream digest, and unchanged selected provenance.

## W2 and W3

Every qualifying joined callback must match all of:

- SCE018 ego and obstacle scalar fields exactly within parser round-trip tolerance `1e-12`, and the
  same embedded 185-waypoint reference;
- `TEST_ACTIVE`, `r3_invoked=true`, `fresh_evaluation_count=1`;
- pair/reconstruction/validator `128/12/12`, candidate count 12;
- selected candidate `GQSC_S1_MAIN_COVERAGE_LEFT_c7b2c19bf2af9350`, source
  `LEX8_GLOBAL_DISJOINT_COVERAGE4`, source cell `COVERAGE`;
- selected digest `c7b2c19bf2af9350`;
- `fresh_hard_valid=true`, `FRESH_SELECTED`, reason `FRESH_HARD_VALID_P3_M1`, owner `P3_M1`, and
  `safe_stop_active=false`.

The public node output exposes hard validity as a boolean rather than W1's internal hard/usable
counts. The exact public node state above is therefore the frozen ROS-layer equivalent: hard-valid
and actually selected/usable, not an invented `9/9` claim. Adding instrumentation solely to expose
those internal counts is forbidden in this protocol.

W3 uses the same frozen planner snapshot even while simulator ego/perception topics evolve. Exact
digest parity is therefore required for every W3 sample, not weakened to a moving-trajectory
tolerance. A mismatch invalidates the complete A/B pair before latency analysis.

## CPU parity

CPU 8 is logical thread 8 of physical core 4; CPU 9 is its SMT sibling. All non-planner
experiment-owned processes use `0-7,10-23` in both A and B. Condition A leaves the planner
unrestricted on the verified `0-23` parent mask. Condition B alone changes the planner to
`taskset -c 8`. This common background mask prevents an experiment-owned sibling competitor from
being introduced only in B. Kernel command line has no `isolcpus`/`nohz_full`, so the contract calls
this affinity control, not hard OS isolation; normal host services are not silently claimed absent.
