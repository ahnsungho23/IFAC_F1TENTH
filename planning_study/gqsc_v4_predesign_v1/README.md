# GQSC v4 pre-design: bounded selection and geometry-derived transition

## Conclusion

`GQSC_V4_GENERAL_TRANSITION_NOT_DERIVABLE`

The corrected evidence supports one clean **partial** refinement—S1
`LEX8_GLOBAL_DISJOINT_COVERAGE4`—but not a complete v4 design.  S1 recovers 3/10 exact Top-12
misses with zero hard/usable loss on allowed seen controls and stays within B128/K12/12 validators.
The two fixed-B128 alternatives recover both pair truncations but regress seen exact successes.
Most importantly, the five production transition witnesses are outputs of exact-validator-steered
entry bisection, not a source-defined geometry-only formula.  Under the task constraints a direct
equivalent cannot be honestly derived, so candidate D was not created or evaluated.

Frozen v3 remains immutable.  No planner decision/configuration, P3 family, validator, ranking,
operator, B128/K12 bound, canonical method serialization, or frozen SHA was changed.

## 1. Corrected property contract

The 200 historical property “misses” divide into 101 nonblocking result-semantics cases, 83
`NO_EXACT_P3_WITNESS_CONFIRMED` cases, three no-valid-side invariants, and 13 exact same-P3
coverage misses.  The confirmed-positive property denominator is therefore 351 = 338 frozen
successes + 13 exact misses; frozen v3 covers 338/351.  For failure-mechanism research the property
denominator is 13.  Adding six method-policy production/raceline misses gives the corrected combined
denominator 19.  `raceline_tight_gap` is excluded from that denominator as an interface mismatch.

No safety assertion was weakened.  A conservative 1D corridor remains useful, but it is not by
itself a five-knot P3 existence proof.

## 2. Top-12 displacement mechanism

The ten hard-valid witnesses have proxy ranks 24–67.  Every case is one-sided at selection time,
so the frozen side-balanced two-slot reserve cannot provide cross-side information; its separate
one-item coverage calls also do not accumulate a four-item diversity state.  Exact shape dedup is
working (12/12 unique in every selected set), yet near-equivalent normalized shapes remain heavily
clustered—the minimum selected pair distance ranges from 0.000579 to 0.022097.

Seven witnesses use a transition family completely absent from their selected 12.  In the other
three, the transition is present but the required lateral operator/shape is displaced.  The first
decisive frozen lexicographic fields are curvature proxy (5), next-obstacle exit conflict (3), and
maximum corridor violation (2).  No analytic-root or zero-interface seed is involved: these are
standalone direct-geometry factor pairs.

Policy ablation:

- S1 (`8+4` global disjoint reserve): 3/10 recovery, no hard/usable loss; preferred partial policy.
- S2 (`8+4` transition-stratified): 7/10, but one hard and two usable losses; rejected.
- S3 (`6+6` global): same 3/10 as S1 and one fewer property usable pass; not preferred.

These policies were predeclared from the structural diagnosis and never read case names, exact
verdicts, digests, or witness values online.

## 3. B128 materialization

The frozen traversal sorts seven transitions per lateral and emits wave 0 for every lateral, then
wave 1, and so on.  `RP2_22` has 32 laterals; the witness laterals 8/9/10 receive transitions
0,1,3,5 in the four complete waves, while required transition 2 remains later.  `RP3_220` has 63
laterals; two waves consume 126 slots and lateral 20 never reaches transition 2.

B1 and B2 both place the existing `LONG_ENTRY_SHORT_EXIT` pair inside B128 and S1 then reconstructs
both witnesses.  This is not a clean fix: B1+S1 loses `VUE025`; B2+S1 loses `DVE028` and `VUE025`.
The budget was not increased and neither policy is recommended.

## 4. Legacy transition provenance

The repeated entry values are dyadic points of
`[0.51458109301505117, 1.310934367577036]`.  Four frames accept the first midpoint
`0.91275773029604357`; `layoutB_failing_f1` accepts the later 7/16 point
`0.86298565063591948`.  The exact validator directs each bisection step from its first failure
category.  The exit is the fixed low-biased sample
`x_min + (1/64)(x_max-x_min)`; changing horizon-derived `x_max` explains the observed
0.57695–0.57721 spread.

This provenance proves a general **search procedure**, not a general direct geometry equation.
Replacing its validator oracle with a proxy would be a new method hypothesis; copying the five
outputs would be event tuning.  Both are outside this task, so no direct transition candidate was
added.

## 5. Lateral and interface findings

The two lateral misses do not share one operator: `RP3_90` is exact in-domain midpoint equality,
whereas `RP3_296` is an equal target/mid below the frozen side-domain lower bound.  Lateral
expansion is deferred.

For `raceline_tight_gap`, `0.15/0.08` are frozen vehicle half-width/obstacle safety margin, while
the unit test supplies `0.12/0.03`.  Canonical `local_planning.yaml` operationally matches frozen
`0.15/0.08` (wall `0.04`), but alternative configs and parameterized tests do not.  Therefore the
test is not an operator target: future geometry construction should bind to effective evaluator
parameters.  That recommendation was not implemented.

## 6. Controlled comparison and runtime

On the combined 19 exact method-policy misses, A/B/C recover 0/3/4 respectively; C's extra B128
coverage is rejected because it loses an existing seen success.  Transition 0/5 and lateral 0/2
remain unrecovered for every admissible candidate.  Candidate D is not applicable.

Warm Release standalone-native timing used 104 allowed seen snapshots, two discarded warmups and
10 measured repetitions each; audit digest/string output was disabled.  All measured policies
respect pair proxies <=128, reconstructions <=12, and validator calls <=12.

| candidate | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|
| frozen A | 4.790 ms | 18.867 ms | 21.411 ms | 22.947 ms |
| selection-only B | 11.107 ms | 19.946 ms | 20.921 ms | 21.048 ms |
| rejected B1 combination C | 9.994 ms | 19.594 ms | 21.027 ms | 21.797 ms |

The p95 and p99 targets are met in this offline native harness.  This is not a closed-loop or
full-callback qualification.

## Evidence limits and protocol

- Exact ablations use the existing P3 constructor and exact validator through temporary
  research-only audit adapters.  Those adapters and candidate-policy hooks were removed after
  collecting compact results; only this study directory remains from the task.
- Frozen method SHA expected and rechecked: `965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780`.
- FINAL_HOLDOUT contents were not opened, parsed, or executed.  No closed-loop run occurred.
- No v4 method is frozen, and no production behavior is changed by this design study.
