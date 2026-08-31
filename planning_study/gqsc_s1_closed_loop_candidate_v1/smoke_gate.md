# Closed-loop smoke gate

The small known-map low-speed closed-loop smoke was **not run**.

Stateless parity, lifecycle ownership, deterministic selection, no-hidden-seed, and bounded-work
gates pass. The production-equivalent evaluator itself has p95 19.742 ms, but the preserved full
callback model has p95 43.208 ms and p99 55.006 ms. Because the task required the smoke only when
the runtime gate passes, launching the simulator would violate the stated gate order.

This is a runtime blocker, not evidence of an S1 integration mismatch. The tail includes preserved
fallback and safe-stop hypothetical evaluations; callback-level GQSC evaluation counts were 1 for
62 events, 2 for 40, and 3 for 2.
