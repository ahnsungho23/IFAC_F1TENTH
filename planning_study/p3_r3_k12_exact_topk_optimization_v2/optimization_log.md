# Exact Top-K optimization log

All retained steps preserve frozen factor generation, ordering, 10+2 quotas, K=12, tie-breaking,
shape/path deduplication, P3 geometry, exact validation, final rank, and selected path. Each material
step passed the combined-seen 86-event parity gate.

1. Reconstructed the exact lateral/transition Cartesian space, feasibility tuple, coverage score,
   diversity distance, and shape dedup order. No code optimization preceded this analysis.
2. Changed the profile-basis cache key from `(d_target,transition_index)` to the exact five-station
   bit pattern. This computes identical reference powers once for identical geometry.
3. Added exact profile key `H=(side,d_target,d_mid,stations[0..4])`. Proxy metrics are evaluated once
   per `H` and copied bit-for-bit, while each transition retains its own normalized entry/exit for
   coverage. Across 86 events this removed 277,687 of 776,193 full proxy evaluations (35.78%).
4. Replaced repeated global-min feasibility selection with shape-representative exact partial Top-10.
   The representative is the stable-tie minimum member of each identical-metric shape class.
5. Replaced tuple/string-heavy comparators with field-by-field comparisons that preserve the tuple's
   strict weak ordering, including non-finite equivalence behavior. Coverage reuses the incumbent's
   already-computed adjusted score within each scan.
6. Replaced the hot 71k-row public/string factor pool with a compact numeric rank row. Source strings
   and Python-compatible configuration/shape strings are still created for the selected factors in
   the same form.
7. Cached sample-local corridor bounds, center/later-obstacle flags, width denominator, and exact
   station differences in the station basis. Candidate floating sums and divisions retain their
   original order.
8. Tested eight fixed metric workers. It passed 86/86 exact parity but increased large-event runtime
   and CPU pressure on this host, so the fixed four-worker bound was restored.
9. Tested basis-grouped scheduling. It passed 86/86 parity but worsened tail balance because basis
   sample lengths differ, so original contiguous deterministic chunks were restored.
10. Rejected best-first Cartesian enumeration, k-way merge, score cutoffs, and dominance pruning:
    no sufficiently tight order-preserving bound exists for the frozen joint proxy and
    selected-dependent coverage score. VUE019 has 71,506 unique profiles and admits zero proven
    full-proxy skips.

Final instrumentation-OFF R3 p50/p95/p99/max is
`13.82/77.25/119.65/166.17 ms`, versus v1
`17.19/132.73/196.07/263.35 ms`. Exact validation p95 remains `5.97 ms`; pair proxy p95 is still
`50.31 ms` and is the remaining tail bottleneck.

No large closed-loop benchmark, method tuning, parameter change, commit, or push was performed.
