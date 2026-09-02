FIRST TASK AFTER A1:
preregister and execute an independent R4 SCE018 runtime replication.
Do not modify the algorithm before that replication.

# Resume after A1

Follow this order. Do not skip directly to algorithm changes or holdout evaluation.

## NEXT 1 — Independent R4 replication

Preregister a separate SCE018 execution using the frozen R4 method, binaries, workload, parity,
schedule, affinity, validity, and decision contracts. Use no R4 scientific callback again.

Primary questions:

- Does W3-B still pass 25 ms?
- Does the W3 A->B improvement direction replicate?
- Does exact parity remain intact?

Do not change the algorithm for this replication.

## NEXT 2 — Runtime geometry generalization

After replication, preregister a DEVELOPMENT/SYNTHETIC workload set spanning distinct compute
mechanisms:

- short-entry;
- high-curvature;
- probe/root-heavy;
- `d_mid`-sensitive;
- factor-space-heavy; and
- candidate-budget-heavy.

Select no case using `FINAL_HOLDOUT_UNSEEN`. Determine whether the 25 ms result generalizes beyond
SCE018 and identify worst-case compute geometry.

## NEXT 3 — Remaining selector/search coverage audit

Audit the prior H4-A failure mechanisms with mechanism-specific development/synthetic
reproductions:

- probe/root/`d_mid` coverage;
- multi-parameter/factor-space coverage; and
- candidate-budget truncation.

Do not tune against `VALIDATION_SEEN_AFTER_V1` identities.

## NEXT 4 — Minimal algorithm change, only if justified

Change the algorithm only after an independent failure mechanism is demonstrated. Modify only the
proven bottleneck: search coverage, selector logic, parameter coverage, budget, or family
expressiveness. Do not introduce case-specific patches.

## NEXT 5 — Regression and closed loop

Run, in order:

`synthetic/mechanism tests -> development regression -> VALIDATION_SEEN_AFTER_V1 confirmation -> closed-loop validation`

Validation-seen results remain confirmation evidence, never a new model-selection set.

## NEXT 6 — Final freeze and holdout

Freeze the method, runtime protocol, coverage contract, experiment protocol, and closed-loop
evidence first. Only then may `FINAL_HOLDOUT_UNSEEN` be evaluated once through the immutable final
pipeline.
