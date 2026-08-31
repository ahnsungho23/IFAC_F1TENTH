# R3-RT bounded proposal method v1

## Status and authority

This is a **new seen-data design**, not an exact implementation of frozen R3. The
offline teacher is `R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12` with method SHA
`7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc`.
No oracle coordinate or validator outcome enters proposal generation or ranking.

## Characterization-first evidence

The machine-readable characterization was written before the bounded sweep. Its
rows preserve all 86 x 12 teacher selections, identify feasibility Top-10 versus
coverage Top-2, direct lateral/transition sources, geometry descriptors, hard and
usable verdicts, and the final hard-ranker selection. R3 is a direct
`(d_target,d_mid,entry_scale,exit_scale)` factor method; `probe/root source` is
therefore explicitly `NOT_APPLICABLE_DIRECT_DT_DM_FACTOR`.

## Hard-bounded generator

For every strict side, at most 32 direct lateral operator outputs are formed. The
global deterministic interleave has an absolute cap of 64. Operators are paired
recipes, not a target x mid grid:

- exact production target with equal mid and `mid=target-0.02 m`;
- component C0 anchors at fractions `0, 1/32, 1/8, 3/8, 1/2, 2/3`, each with one
  fixed equal/offset/corridor-interpolation recipe;
- characterization-supported production offsets (`-0.01,+0.01,-0.06,+0.06,
  -0.10,-0.02 m`) with one associated mid recipe;
- two corridor centers and a bounded tail of 1/8, 1/4 center interpolation and
  `-0.10,+0.05 m` mid corrections.

The transition operator reads only the already-bounded exact production transition
set. Seven normalized geometry-independent S/M/L anchors are projected to the
nearest exact transition tuple: short/medium/long entry crossed only with the seven
predeclared short/medium/long combinations. Exact duplicates are removed, so the
actual transition count is at most seven.

Pairs are emitted by a source-conditioned diagonal lazy traversal. The traversal
stops immediately at B and never materializes the lateral x transition Cartesian
product. Exact production configurations are skipped before proxy work. The full
frozen-R3 proxy tuple is evaluated exactly once for each emitted pair.

## Selection and reserve ablation

Per-side and full small reserves were evaluated at B64 inside the same B budget.
They did not improve usable recovery, while the no-reserve diagonal already retained
diverse selected-family signatures. The selected v1 sweep therefore uses no fixed
proposal reserve. This is an evidence-based rejection, not an unbounded fallback.
From the bounded pool, frozen R3's lexicographic order supplies 10 configurations
and its coverage order supplies 2. At most 12 requests reach the unchanged P3
constructor and exact validator. Path-digest duplicates are removed before the
logical final-ranker view.

## Computation contract

`lateral operator outputs <= 64`, `transition proposals <= 7`,
`pair proxy evaluations <= B`, `P3 reconstructions <= 12`, and
`exact validator executions <= 12`. The Python study reports actual counts and
fails if a bound is exceeded.

## Data boundary

Only `PILOT_SEEN_DEVELOPMENT_DATA`, `DEVELOPMENT`, and
`VALIDATION_SEEN_AFTER_V1` enter this script. The final holdout is neither listed,
parsed, nor used for tuning. This offline prototype does not modify production
planner behavior.
