# GQSC v3 lifecycle / horizon reconciliation v1

## 최종 판정

`GQSC_V3_TRUE_COVERAGE_REGRESSION`

integration/lifecycle mismatch 자체는 해소됐다. 다섯 fresh rejection은 모두 frozen GQSC의 짧은 path
문제가 아니라 legacy lifecycle의 next-obstacle horizon overreach였고, explicit frozen ownership
certificate로 61/61 lifecycle ownership을 회복했다. 동일 callback의 exact same-input primary 재평가도
43회 cache reuse로 바뀌어 같은 입력의 중복 새 평가는 0이 됐다.

그러나 frozen method는 production scenario 3/12, raceline 8/88, deterministic corridor property
200/538에서 실제 coverage 회귀를 남긴다. 이 작업은 GQSC를 튜닝할 수 없고 이 실패는 horizon/cache
fix로 사라지지 않으므로, closed-loop smoke 준비 완료나 runtime-only blocker로 분류하지 않는다.

## 핵심 결과

| gate | pre-fix | post-fix |
|---|---:|---:|
| frozen stateless exact parity | 104/104 | 104/104 |
| GQSC hard-valid events | 61 | 61 |
| fresh lifecycle ownership | 56/61 | 61/61 |
| evaluator/lifecycle horizon exact | 56/61 | 61/61 |
| continuation/dropout/trim/completion | 56/56 | 61/61 |
| owned-interval new-blocker invalidation | 56/56 | 61/61 |
| hidden legacy seeds | 0 | 0 |
| max pair/reconstruction/validator | 128/12/12 | 128/12/12 |
| same-input duplicate new evaluations | 43 counterfactual | 0 |

## 다섯 lifecycle rejection

`DVE004`, `DVE006`, `DVE007`, `VUE011`, `VUE013`의 evaluator horizon은 1.899–3.147 m,
pre-fix lifecycle nominal horizon은 5.109–5.658 m였다. evaluator는 다음 expanded cluster의 front에서
현재 maneuver 책임을 끝냈지만 lifecycle은 `cluster_end + 5 m`까지 넓혀 다음 cluster를 현재 path의
실패로 판단했다. 다섯 path는 전자에서 5/5 hard-valid, 후자에서 0/5였다.

따라서 분류는 전부 `LIFECYCLE_REVALIDATION_OVERREACH`다. 수식, obstacle ID, sample 수와 역사적
설계 근거는 [horizon_semantics.md](horizon_semantics.md), raw 행은
[five_event_diagnosis.csv](five_event_diagnosis.csv)에 있다.

## ownership와 duplicate invocation

현재 maneuver가 소유하는 obstacle interval은 선택 시의
`maneuverScopeEnd = min(cluster_end + post_merge_lookahead, next expanded obstacle front)`이며 자기
cluster rear보다 앞에서 끝날 수 없다. absolute boundary를 record에 고정하고 active progress만 뺀다.
existing next cluster는 chaining 대상이지만, 선택 후 owned interval 안에 나타난 blocker는 그대로
현재 path를 invalidation한다. 전체 path의 track/footprint/curvature 검사는 horizon 밖에서도 유지된다.

104개 invocation audit에서 새 GQSC 평가는 callback당 1회 61건, 2회 41건, 3회 2건이었다. 후자의
추가 평가는 hypothetical safe-stop ego를 쓰는 진짜 새 계산이다. 동일 primary snapshot은 43번
재사용됐다. 현재 callback-global cap은 없고, 운영 설정의 정적 control-flow 상한은 44회, 직접 관측
상한은 3회다. 자세한 분기/구조 상한은 [callback_callgraph.md](callback_callgraph.md), event별 계수는
[duplicate_invocation_analysis.csv](duplicate_invocation_analysis.csv)에 있다.

## Warm Release runtime

research instrumentation OFF-equivalent harness, warmup 2회 뒤 snapshot당 7회(총 728)를 측정했다.

| stage (ms) | p50 | p90 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|
| GQSC generation | 6.484 | 9.164 | 9.372 | 9.713 | 9.918 |
| exact validation | 4.658 | 10.022 | 10.965 | 12.076 | 12.882 |
| frozen evaluation wall | 10.941 | 19.155 | 20.054 | 21.569 | 22.615 |
| lifecycle | 0.258 | 1.203 | 1.245 | 1.292 | 1.342 |
| fallback | 0.000 | 25.557 | 28.053 | 42.833 | 42.980 |
| callback model | 17.243 | 40.346 | 43.724 | 56.009 | 56.514 |

이전 callback p95 60.228 ms에서 43.724 ms로 낮아졌지만 같은 실행환경의 paired benchmark가 아니므로
약 27%를 확정 speedup으로 해석하지 않는다. 단일 evaluator p95는 20.140→20.054 ms로 사실상 같다.
전체 callback p95는 여전히 25 ms 주기를 초과하며 safe-stop hypothetical probe가 주된 tail이다.
따라서 coverage 외에 **secondary runtime risk**도 남는다. raw 측정은
[callback_runtime.csv](callback_runtime.csv)에 있다.

## 회귀 분류

최신 Release test 결과는 다음과 같다.

- `test_p3_maneuver_lifecycle`: 20/20 PASS
- production scenario: 9/12; 실패 3개 모두 D
- raceline: 80/88; B 1개, C 6개, D 1개
- corridor property: 1/2; oracle-feasible 538개 중 miss 200, false pass 0, D

총 실패 분류는 B 1, C 6, D 5다. B/C test도 삭제하지 않았고, 각 행에 generator-independent
replacement invariant를 먼저 제시했다. safety/lifecycle 의미를 직접 검사하는 mandatory replacement는
유지해야 한다. 전체 표는 [regression_test_classification.csv](regression_test_classification.csv)다.

전체 package 검사에서는 이 분류 외에도 이미 알려진 비-method 항목이 남았다. operational YAML과
harness의 `safety_margin_m` 0.08/0.05, control/planner의 right steering 0.41/0.361 mismatch는 요청대로
변경하지 않았다. overlay 없이 처음 실행한 node test의 shared-library 오류는 overlay를 source한 뒤
PASS했다. 새 reconciliation 파일은 copyright/cpplint/uncrustify를 통과했고, 전체 lint에는 frozen
selector source와 기존 R3 test의 두 formatter divergence 및 기존 R3 test blank-line 1건만 남았다.

## 산출물과 재현

- [ownership_contract.md](ownership_contract.md)
- [proposed_integration_fix.md](proposed_integration_fix.md)
- [post_fix_lifecycle_results.csv](post_fix_lifecycle_results.csv)
- [frozen_sha_verification.md](frozen_sha_verification.md)

```bash
cmake --build build/local_planning -j4
python3 planning_study/gqsc_v3_lifecycle_reconciliation_v1/run_reconciliation_study.py
ROS_LOG_DIR=/tmp/gqsc_v3_reconciliation \
  ctest --test-dir build/local_planning --output-on-failure \
  -R 'test_p3_maneuver_lifecycle|test_p3_production_parity|test_raceline_spline|test_rule_property'
```

FINAL_HOLDOUT content 접근, large closed-loop, GQSC tuning, commit/push는 수행하지 않았다.
