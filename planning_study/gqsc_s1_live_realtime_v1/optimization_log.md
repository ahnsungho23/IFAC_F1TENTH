# Optimization log

## Profile gate

Profiling was completed before any optimization decision. It showed:

1. A real prior-live query reproduced the same selected digest in standalone Release at p95
   17.006 ms, versus 27.802 ms live evaluator time.
2. Identical SCE018 work changed from p95 22.008 ms on a performance core to 30.952 ms on an
   efficiency core.
3. Pair proxies, P3 reconstruction, and exact validation dominate; obstacle conversion,
   Top-12 ordering, ROS message construction, publication, and lock wait do not.
4. Research instrumentation overhead is approximately 0.10 ms at p95, not the observed 8-11 ms
   gap.

## Applied changes

- Added a research-only, default-OFF A-O callback profiler.
- Added an evaluator observer carrying already-computed stage/count data to the node profiler.
- Added a standalone harness mode that prints existing stage counters for reproducibility.
- Ensured profiler string serialization/publication starts after `O_total`.

These are measurement changes, not GQSC optimizations. With profiling disabled, the observer is
empty and no profiler publisher is created.

## Rejected optimization candidates

No behavior-preserving GQSC hot-path optimization was applied:

- shared geometry/corridor caching: no duplicated callback-level computation was demonstrated as
  the source of the live/native gap;
- fewer pair proxies or validators: would change frozen B128/K12/validator semantics;
- numerical/tolerance changes: prohibited and unnecessary for the diagnosis;
- message/locking optimization: measured costs are too small to recover the tail;
- research formatting removal: already outside `O_total`, and ordinary research instrumentation
  was OFF in the new live run;
- parallel reconstruction/validation: changes deployment resource/scheduling behavior and was
  not justified after the core-placement result.

The requested environmental-limit rule applies, so changing GQSC to compensate would violate the
study contract.

