# Callback-global computation bound

## Evaluation-local frozen contract

Every fresh S1 evaluation remains bounded by:

```text
pair proxies <= 128
P3 reconstructions <= 12
S1 exact-validator executions <= 12
```

## TEST_ACTIVE control-flow bound

Let

```text
R = min(safe_stop_escape_max_retreats, 12)
P = 2 + R
```

`P` is the maximum requested-stop + ego-endpoint + bisection probes in one `buildSafeStop()`.
At most two non-escape S1 entries and two safe-stop constructions are reachable in the conservative
initial-stabilization/fallback path. Therefore:

```text
fresh_S1_per_callback <= 2 + 2P
```

With the current YAML `safe_stop_escape_max_retreats=8`:

```text
fresh S1 <= 22
pair proxies <= 2,816
P3 reconstructions <= 264
S1 evaluator validators <= 264
callback-global validateCandidate executions <= 266
```

The final `+2` is the two independently constructed braking paths. Other lifecycle revalidation
branches are mutually exclusive with the path that realizes this dominating bound. With the
source-level clamp `R<=12`, the parameter-independent hard envelope is 30 / 3,840 / 360 / 362.

These are structural upper bounds, not observed costs. After the optimization the 104-callback
observed maxima were 2 fresh S1 evaluations, 256 pair proxies, 24 reconstructions, 24 S1 evaluator
validators, and 25 callback-global `validateCandidate()` executions.

## Proposed enforceable contract

The defensible contract is hierarchical:

```text
normal path-selection fresh S1 <= 1
exact-equivalent input fresh S1 = 0 (certificate reuse)
distinct safe-stop escape states <= 2 + min(configured_retreats, 12) per stop construction
stop constructions <= 2 per callback
total fresh S1 <= 2 + 2*(2 + min(configured_retreats, 12))
```

It follows the existing control flow and safety semantics rather than selecting a number to meet a
timing target. A stronger unconditional `fresh S1 <= 1` is feasible for normal planning and active
continuation, but not for current safe-stop location selection: a distinct `v=0` future state is a
different planning problem. Imposing a smaller failure-path cap requires a redesigned safe-stop
predicate or a proof that deferred probing preserves the same stopping and release decisions.
