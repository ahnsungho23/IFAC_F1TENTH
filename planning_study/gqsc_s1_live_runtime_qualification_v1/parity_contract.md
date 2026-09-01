# A/B parity contract

## Common invariants

Between A and B, and across all five repeats, the following must be identical:

- repository checkpoint, frozen contract source, planner configuration, binary, method and
  method SHA;
- `128/8/4/12/12` bounds, candidate semantics, validator, safety guards, mode, and planning period;
- workload input bytes and allowed provenance;
- environment bootstrap, ROS parameters except the explicitly frozen profiling switch, topic QoS,
  warm-up/sample counts, timeout, retry rule, and schedule;
- host, governor, background-service policy, and experiment-owned processes; only planner-process
  CPU affinity differs;
- success/failure classification and selected provenance for equivalent deterministic input.

Any parity mismatch invalidates the complete A/B pair before latency comparison. It must not be
repaired by selecting a nearby run.

## Digest implementation

`local_planning::pathDigest` uses a 64-bit FNV-style byte hash initialized with
`1469598103934665603`. It hashes waypoint count followed, for every waypoint in order, by raw
IEEE-754 bytes of `s_m`, `d_m`, `x_m`, `y_m`, `psi_rad`, `kappa_radpm`, `vx_mps`, and `ax_mps2`,
then emits 16 lower-case hexadecimal digits. The implementation source hashes are frozen in
`workload_freeze.md`. Cross-architecture/endian reconstruction is not assumed; the host
architecture is x86_64.

## W1

For every measured timing row, require pair/reconstruction/validator counts `128/12/12` and
outcome `FRESH_SELECTED`. Before and after every A/B pair, the untimed parity command must report:

- selected path digest `c7b2c19bf2af9350`;
- hard/usable counts `9/9`;
- failure classification `NONE`;
- matching integrated and downstream selected digest; and
- unchanged selected candidate provenance.

The timed `GQSC_MAIN_TIMING` schema does not carry a digest. Therefore the bracketing parity rows
are mandatory, and their absence cannot be replaced by assuming digest stability from counts.

## W2

The future replay must produce the exact W1 reference/ego/obstacle fields and bind the live profile
to a cycle diagnostic with the same callback sequence. A and B must match on the full generation
identity pattern, complexity counts, selected candidate identity/provenance, selected path digest
`c7b2c19bf2af9350`, output state, failure classification, and every safety guard. If operational
node conversion legitimately changes the path from W1, that new ROS-node digest must be established
by a pre-performance functional validation and frozen here before either A or B is run. The current
historical note `reference/lateral NOT_RETAINED` is insufficient; W2 remains blocked.

## W3

Exact path-digest equality across a moving simulator trajectory is not required between unrelated
cycle times. The strongest predeclared equivalence for W3 is instead:

1. byte-identical simulator/map/path/config/scheduler inputs and seed/noise;
2. identical initial pose and collection-start state within fixed tolerances established before
   performance data;
3. the same ordered scenario phase and source-identity rules;
4. for A/B cycles matched by scenario phase and source identity, identical selected side,
   candidate family/provenance, success/failure/lifecycle state, safety-guard state, and bounded
   counts; and
5. digest equality where the matched reference, ego, and obstacle snapshots are byte-identical.

The exact initial-state tolerances and deterministic phase join cannot be filled from the retained
S01 artifacts. They must be validated and frozen before execution; therefore W3 remains blocked.
