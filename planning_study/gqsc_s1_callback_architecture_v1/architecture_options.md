# Behavior-preserving architecture options

| option | status | reason |
|---|---|---|
| Exact same-input result reuse | **implemented** | Exact ego scalars, ordered obstacles, and planner revision prove equivalent input; 31 escape evaluations removed. |
| Reuse S1 exact failure/existence certificate | **implemented** | The same exact validator already decided every returned Top-12 path; duplicate verdict work cannot change the existential result. |
| Keep `measureCandidate()` for failed plan audits, skip duplicate validator | **implemented** | Preserves candidate audit metrics/reasons while removing a mathematically identical rejection. |
| Reuse primary result at a different speed or station | rejected | Transition geometry, braking deficit, corridor window, and validity may change. |
| Validate only the primary failed path at the stop state | rejected | Does not prove that another S1 candidate becomes feasible from the hypothetical state. |
| Replace escape S1 with braking-distance calculation | rejected | Braking feasibility and post-stop avoidance existence are different predicates. |
| Deterministic emergency path instead of S1 escape | deferred | Could preserve immediate stopping but not the existing resumability/stop-location contract. |
| Defer distinct-state escape search to next 40 Hz callback | deferred | Can change the selected stop point after braking has already begun; needs a temporal safety proof. |
| Hard `fresh S1<=1` cap | rejected for failure path | Would silently skip current safe-stop evidence. Valid only for normal path-selection callbacks. |
| Share immutable corridor/reference context across distinct states | future option | Potentially behavior-preserving, but exact floating-point/proposal parity and ownership need separate proof. |

The implemented changes do not alter frozen S1 serialization or SHA, proposal materialization,
Top-12, P3 geometry, exact validator, final ranker, selected path, lifecycle, or fallback. The p95
target is met, so no unproven safe-stop redesign was introduced to chase p99.
