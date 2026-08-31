# Exact S1 callback call graph

현재 operational mode는 YAML의 `p3_mode: TEST_ACTIVE`이다. 아래에서 “fresh”는 frozen S1
`evaluateP3Shadow()` 한 번을 뜻한다.

```text
onPlanningTimer(TEST_ACTIVE)
  capture immutable callback snapshot
  advanceP3Lifecycle(lazy evaluate)
    active suffix hard-valid --------------------------> publish; fresh S1 = 0
    no active suffix / continuation invalid
      evaluateP3Snapshot(TEST_ACTIVE_PRIMARY) ---------> fresh S1 <= 1
      hard-valid --------------------------------------> selectFresh + raw revalidation + publish
      no hard-valid
        runSafetyPlanningCycle(snapshot, same_input_p3)
          safe-stop latch active
            handleSafeStopLatch
              plan(current ego/current guarded obstacles)
          initial stabilization
            buildPreparationStop
              buildSafeStop -> SAFE_STOP_ESCAPE probes
            if unstable: plan(conservative obstacles, same_input_p3)
            if stable: fall through to ordinary plan
          ordinary plan
            generateP3Candidates(..., same_input_p3)
              exact-equivalent input ------------------> cached result
              different input -------------------------> fresh S1 <= 1
            no hard-valid
              buildSafeStop(..., same_input_p3)
                requested stop state (s_stop, d_ego, 0)
                ego endpoint state (s_ego, d_ego, 0), if needed
                bounded bisection states, if needed
```

## Invocation sites

| site | trigger | ego / obstacles | lifecycle context | result consumer |
|---|---|---|---|---|
| `evaluateP3Snapshot` | no held suffix, or continuation invalidated | frozen callback ego + guarded selection snapshot | fresh ownership | primary path selection |
| `generateP3Candidates` inside `plan()` | fallback/initial stabilization/latch replan requests a path | caller ego + caller's ordered obstacles | side lock and fallback context | primary fallback path selection |
| requested-stop `escapable_at(stop_at)` | all ordinary S1 candidates failed | `(wrap(s+stop_at), d, 0)` + raw obstacles re-expanded at that ego | before safe-stop latch | safe-stop location/resumability |
| ego-endpoint `escapable_at(0)` | requested stop failed and `stop_at>0` | `(s, d, 0)` + same raw obstacles re-expanded | before safe-stop latch | determine whether retreat can help |
| bisection `escapable_at(mid)` | requested stop failed, ego endpoint passed | bounded distinct hypothetical states | before safe-stop latch | latest escapable stop location |
| `handleSafeStopLatch::plan` | safe-stop is already latched | live ego + current maneuver/guarded obstacles | release ladder | hard-valid avoidance release or continued stop |

The same-input key is exact scalar `s,d,speed`, exact ordered obstacle-vector equality, and planner
reference/parameter revision. Snapshot timestamps are lineage metadata, not geometry; they do not
force recomputation when the actual planning input is identical.

## Observed 104-callback patterns

| pre-change pattern | count | explanation | post-change fresh count |
|---|---:|---|---:|
| primary only | 62 | primary S1 hard-valid | 1 |
| primary + equivalent escape | 29 | stop state exactly equaled primary input | 1 |
| primary + distinct stop escape | 11 | speed and/or station differed | 2 |
| primary + distinct stop + equivalent ego endpoint | 2 | requested stop failed; endpoint equaled primary | 2 |

`plan()`'s same-input fallback reuse already existed. This audit extended that exact equivalence to
safe-stop escape and removed duplicate validation of evaluator-certified failures. `OFF` and
`SHADOW` remain explicit research/rollback modes; the callback bound in `callback_global_bound.md`
is for the operational `TEST_ACTIVE` path.
