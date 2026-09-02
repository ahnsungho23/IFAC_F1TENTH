# Revision-3 execution result: inconclusive

Revision 3 is permanently classified `EXPERIMENT_INCONCLUSIVE`. Its ten valid W1 final runs contain
2,000 measured callbacks, but W2 R1-A `ATTEMPT0` and its single allowed `RERUN1` both failed before
replay because planner PID/affinity evidence could not be established. W2 and W3 measured callback
counts are zero.

The R3 W1 timing is failed-execution provenance, not part of the final qualification dataset. It
must not be reused in revision 4, mixed with revision-4 data, or used to choose a model, protocol,
threshold, schedule, or operational repair. No R3 latency quantile or A/B performance analysis was
performed before the revision-4 repair.

The untracked raw directory remains in place and byte-unchanged. Its verified preservation record
before the R4 repair is:

- W1 sorted per-file SHA-256 listing aggregate:
  `4a9d7d42f4ce9bab83748959eb2d199decde83ae94765a847027df4cc53ae96f`
- W2 failure sorted per-file SHA-256 listing aggregate:
  `802d4113f6e00c95035d18de93784daed8db5fb809f9265eacd9790a31ccfd45`
- complete R3 raw sorted per-file SHA-256 listing aggregate:
  `3c7a9ac0fb52b0ac71b64d53edb2f3d01adafddffbbb0edc31428fcbcaebba8b`
- file count: `54`
- byte count: `443897`
