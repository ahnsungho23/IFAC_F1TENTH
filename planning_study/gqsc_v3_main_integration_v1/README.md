# GQSC v3 frozen main-planner integration v1

## 결론

`GQSC_V3_LIFECYCLE_BLOCKER`

Frozen GQSC v3의 stateless production adapter는 seen-only 104개 snapshot에서 연구 구현과
exact parity를 만족했습니다. 그러나 그중 GQSC hard-valid path가 있던 61건 중 5건은 기존
fresh lifecycle이 evaluator보다 긴 obstacle collision horizon으로 raw revalidation을 수행해
ownership을 거부했습니다. 메서드와 lifecycle을 동결한 조건에서는 closed-loop smoke 준비 완료로
판정할 수 없습니다.

이 작업은 큰 closed-loop benchmark를 실행하지 않았고, GQSC operator·B128/K12·rank·tie-break·
dedup·validator·차량/config·lifecycle을 튜닝하지 않았습니다.

## 동결 계약

| 항목 | 값 |
|---|---|
| Method | `V3_SIDE_BALANCED_DISJOINT` |
| Canonical method SHA-256 | `965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780` |
| Prevalidation lock SHA-256 | `ea30a441695732e87d43d6863e1a31992dbfdd45749d324047495b6c96824185` |
| Pair proxy cap | 128 |
| P3 reconstruction cap | 12 |
| Exact-validator cap | 12 |
| Lexicographic / coverage reserve | 10 / 2 |

CMake configure와 단위 테스트가 canonical JSON을 직접 SHA-256으로 검증합니다. 노드는 시작 때
method/lock SHA와 세 상한을 로그로 남깁니다. production entry는 frozen standalone selector를
직접 한 번 호출하며 legacy M0/M1 ladder나 exact R3를 먼저 실행하거나 seed로 사용하지 않습니다.
명시적 legacy/R3 API는 연구 baseline과 rollback 비교용으로만 남겼습니다.

## 구현 범위

- fresh candidate entry를 frozen GQSC v3로 교체했습니다.
- GQSC 결과의 exact final rank를 기존 downstream comparator의 마지막 tie 순서로 전달했습니다.
- continuation-first lazy evaluation, immutable suffix, guarded/raw revalidation, speed shaping,
  fallback/safe-stop, publication 및 controller message contract는 유지했습니다.
- 연구 계측은 parity/sequence/timing harness에만 추가했고 production 기본은 OFF입니다.

## Stateless exact parity

허용된 seen-only corpus를 경로로 명시해 총 104개를 평가했습니다. final holdout 경로를 검색하는
fallback은 스크립트에 없습니다.

| Dataset role | Snapshot | GQSC hard-valid 존재 |
|---|---:|---:|
| `PILOT_SEEN_DEVELOPMENT_DATA` | 9 | 4 |
| `DEVELOPMENT` | 40 | 20 |
| `VALIDATION_SEEN_AFTER_V1` | 37 | 20 |
| `SUCCESS_CONTROL_SEEN` | 18 | 17 |
| 합계 | 104 | 61 |

104/104 모두 다음 항목이 exact였습니다.

- lateral/transition/pair proposal order
- lexicographic 10 + disjoint coverage 2의 ordered Top-12
- reconstructed path digest와 validator verdict
- selected path digest
- 실제 `plan()`이 반환한 selected path digest
- snapshot lineage와 canonical method SHA
- pair/reconstruction/validator 상한

상세 행은 [frozen_parity.csv](frozen_parity.csv), 상한 집계는
[bounded_contract.csv](bounded_contract.csv)에 있습니다.

## Sequential/lifecycle replay

GQSC hard-valid 61건 중 fresh ownership에 성공한 것은 56건입니다. 이 56건은 다음 인공 연속
callback 계약을 모두 통과했습니다.

| 단계 | 결과 |
|---|---:|
| 동일 입력 active suffix 유지 | 56/56 |
| Guard 확장: raw fallback 또는 현재 hard-valid continuation | 56/56 |
| obstacle disappearance/dropout에서 frozen Guard 유지 | 56/56 |
| ego 진행에 따른 prefix-only trim | 56/56 |
| expanded cluster 뒤 completion | 56/56 |
| 새 full-width blocker에서 invalidation | 56/56 |

Guard 확장 단계는 37건이 guarded collision 뒤 raw hard-valid fallback을 사용했고, 19건은 확장된
Guard에서도 그대로 hard-valid였습니다. 새 blocker 뒤 기존 fallback 결과는 56/56
`NO_SAFE_PATH`였습니다. 이는 합성 blocker가 경로 바로 위에 놓인 결과이며 fallback/safe-stop
정책을 변경한 것이 아닙니다.

fresh ownership을 거부한 5건은 `DVE004`, `DVE006`, `DVE007`, `VUE011`, `VUE013`입니다.
모두 `FRESH_RAW_EXACT_HARD_INVALID:d-offset intersects an inflated static-obstacle box`였습니다.
원인과 horizon 수치는 [lifecycle_transition_report.md](lifecycle_transition_report.md)에 분리했습니다.

## Warm Release runtime

`CMAKE_BUILD_TYPE=Release`, research instrumentation OFF, warmup 2회 후 snapshot당 7회로 총
728회를 측정했습니다. 단위는 ms입니다.

| 구간 | p50 | p90 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|
| GQSC generation | 6.460 | 9.205 | 9.552 | 10.329 | 11.425 |
| Exact validation | 4.680 | 10.066 | 10.995 | 12.795 | 13.279 |
| Final ranking | 0.005 | 0.008 | 0.009 | 0.013 | 0.024 |
| Frozen evaluation wall | 10.938 | 18.965 | 20.140 | 22.663 | 23.182 |
| Lifecycle | 0.266 | 0.802 | 1.217 | 1.272 | 1.452 |
| Preserved fallback | 0.000 | 41.557 | 44.460 | 56.629 | 62.077 |
| Total callback model | 20.128 | 56.523 | 60.228 | 72.377 | 80.296 |

단일 frozen evaluation은 p95가 25 ms 안이지만 전체 callback은 그렇지 않습니다. frozen evaluation
실패 시 보존된 `plan()` fallback이 동일 primary generator를 다시 호출하기 때문에 tail이 커집니다.
이는 성능을 숨기지 않기 위해 실제 callback model에 포함했으며, 이 작업에서는 lifecycle/fallback을
바꾸지 않았습니다. 원시 반복값은 [callback_runtime.csv](callback_runtime.csv)에 있습니다.

## 회귀 및 범위 감사

- canonical JSON SHA 일치, selector source `p3_r3_k12.cpp` diff 0
- `src/local_planning/config`, launch, controller, perception, localization diff 0
- message/topic 및 vehicle parameter 변경 0
- frozen selector, lifecycle, safe-stop 단위 테스트 통과
- 기존 production scenario suite는 12개 중 9개 통과, 3개 실패:
  `PassingScenariosKeepRecoveringOnEveryLayout`, `LayoutBReplanRecovers`,
  `SamePinchGeometryAtTwoDistances`
- broader `test_raceline_spline`은 current-generator 의존 assertion 88개 중 8개 실패
- deterministic corridor property는 oracle-feasible 538개 중 planner failure 200개로 기존 ceiling
  99를 초과

마지막 세 실패는 frozen V3가 과거 legacy ladder의 recovery coverage를 모두 보존하지 않는다는 이미
동결된 `GQSC_V3_CLEAN_BUT_COVERAGE_TRADEOFF` 성격과 일치합니다. 이 통합에서 method를 고쳐
테스트를 통과시키지 않았습니다. broader suite에는 selection/committed-side/slow-gap recovery와
custom test parameter에서 transition proposal이 비는 method-interface 사례가 포함됩니다. 따라서
validator/lifecycle/safe-stop 구현이 unchanged인 것과 기존 generator-dependent behavior가
non-regressed인 것은 같은 주장이 아니며, 후자는 성립하지 않습니다. 자세한 변경 경계와 알려진
기존 config/control 검사 불일치는
[integration_diff_audit.md](integration_diff_audit.md)에 있습니다.

## 재현

```bash
cmake --build build/local_planning -j4
python3 planning_study/gqsc_v3_main_integration_v1/run_integration_study.py
ROS_LOG_DIR=/tmp/gqsc_v3_main_integration_tests \
  ctest --test-dir build/local_planning --output-on-failure
```

스크립트가 여는 corpus directory는 네 개의 seen-only 경로로 고정되어 있습니다. 이 작업에서
FINAL_HOLDOUT event, manifest, result contents는 읽거나 실행하지 않았습니다. 초기 repository 파일
inventory가 해당 directory의 경로명만 노출했으며 내용 접근은 없었습니다.
