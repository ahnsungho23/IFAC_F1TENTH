# Safe-stop contract audit

## Operations are separate

1. **Avoidance existence at a hypothetical stop**

   `anyFeasibleCandidateFrom(at_stop)` runs frozen S1 on the hypothetical `v=0` ego and asks whether
   any Top-12 path passes the unchanged exact validator. It does not publish or rank a stop path.

2. **Stop-location choice**

   `requested_stop_at = max(0, blocking.start - safe_stop_buffer_m)`. If that point is escapable it
   is kept. Otherwise the ego endpoint is tested; only if the endpoint is escapable does bounded
   bisection retreat the stop to the latest verified point. If neither endpoint works, the requested
   stop is restored and an unescapable warning is recorded.

3. **Braking geometry and feasibility**

   The planner independently constructs a deterministic race-line-offset braking prefix with
   `v(s) <= sqrt(2*a_stop*(stop_at-s))`, forces the last speed to zero, recomputes geometry, and
   executes `validateCandidate()` on the braking path. If no collision-free prefix exists, the
   unchanged committed/last-guidance/emergency-hold ladder owns the fallback.

4. **Latched release**

   Once latched, release still requires obstacle passage, repeated selectable hard-valid avoidance,
   stopped+corridor-clear evidence, or the existing blind-timeout contract. This audit did not
   modify those predicates.

## Is a full B128/K12 search required?

For the current contract, the answer is “yes for a **distinct hypothetical ego**”: resumability is
defined as existence of a path from the same frozen S1 family under the exact validator. A braking
distance calculation or validation of the already-failed primary path cannot prove that existential
claim. However, a second `measureCandidate()+validateCandidate()` pass over the same S1 result was
not required; the evaluator's exact certificate is equivalent and is now reused.

An explicit deterministic emergency trajectory could replace the *braking path builder*, but it
would not replace the *future escape existence* predicate. Replacing the latter with corridor width,
curvature proxy, or a hard callback cap needs a new safety/liveness contract and proof; it was not
done here.

## Counterfactual result on the seen corpus

- 31/44 logical probes were exact duplicates and are `C_DUPLICATE_OR_REUSABLE`.
- 13/44 used distinct hypothetical states. They are
  `E_FALLBACK_CONFIRMATION_ONLY` **for these recorded callbacks** because skipping them would not
  change the observed final path/kind; one changed only escape metadata.
- There were no observed A/B/D/F cases and no observed bisection-retreated final stop.

This empirical classification must not be generalized into permission to delete distinct-state
probes. Source semantics allow such a probe to move `stop_at`, which can decide whether the vehicle
can resume without reverse motion.
