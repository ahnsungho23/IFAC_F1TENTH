# Callback별 GQSC 호출 감사

## TEST_ACTIVE 주 경로

```text
onPlanningTimer
  capture immutable callback snapshot
  advanceP3Lifecycle
    active suffix hard-valid -> publish                         [GQSC 0]
    otherwise lazy evaluateP3Snapshot                          [GQSC 1]
      fresh hard-valid -> selectFresh -> publish               [추가 0]
      no output -> runSafetyPlanningCycle(snapshot, evaluation)
        plan(same input) -> exact witness match -> reuse        [primary 추가 0]
        buildSafeStop escape probes at hypothetical stop ego    [각 probe GQSC 1]
```

동일 callback의 primary 재사용 조건은 hash 추정이 아니라 다음 모두의 exact equality다.

1. evaluator가 실제 호출됐고 frozen GQSC 결과를 보유함
2. ego `s`, `d`, `speed` bit-exact equality
3. ordered obstacle message vector 전체 equality
4. planner parameter/reference revision equality

하나라도 다르면 `plan()`은 정상 evaluator를 새로 호출한다. preferred side/side-switch는 이미 생성된
후보를 downstream에서 filtering하는 입력이므로 witness에 포함할 필요가 없고, filter와 final rank는
cache 뒤에도 그대로 실행된다.

## 새 계산이 필요한 분기

- safe-stop escape probe는 ego station과 speed를 후보 stop point/0 m/s로 바꾸므로 primary snapshot과
  동일하지 않다. `anyFeasibleCandidateFrom()`은 그 hypothetical ego에서 실제로 탈출 가능한지를 묻기
  때문에 재사용할 수 없다.
- `tryEarlyChainedManeuver()`와 `beginChainedManeuverIfNeeded()`는 다음 obstacle set과 현재 ego로
  계획하므로 새 계산이다.
- obstacle/reference/parameter가 같은 callback 안에서라도 달라지면 exact witness 또는 revision이
  mismatch되어 새 계산한다.
- active suffix가 hard-valid하면 continuation-first lazy contract 때문에 fresh GQSC 자체가 0회다.

## 호출 수 상한

`buildSafeStop()` 한 번은 stop point 1회, 필요하면 ego point 1회, 그 사이 최대
`safe_stop_escape_max_retreats`회 이분탐색을 한다. 운영값 8에서는 최대 10회, 코드 내부 clamp 12를
적용한 구조 상한은 14회다. `plan()` 한 번은 primary 1회와 실패 시 위 probe를 합쳐 운영 최대 11회,
구조 최대 15회다.

현재 코드에는 callback-global GQSC cap이 없다. control-flow상 가장 긴 보수적 경로는

```text
lazy TEST_ACTIVE primary
+ failed early-chain plan
+ failed completion-chain plan
+ preparation-stop probes
+ final/early plan
```

이므로 운영 설정의 정적 상한은 `1 + 11 + 11 + 10 + 11 = 44`, 내부 clamp 기준 구조 상한은
`1 + 15 + 15 + 14 + 15 = 60`이다. 이 값은 control-flow upper bound이지 관측값이 아니다. branch
predicate가 일반 주행에서 동시에 성립하지 않아도 이를 막는 callback-global budget은 존재하지 않는다.

seen-only 104 snapshot의 직접 invocation audit에서는 다음만 관측됐다.

| 새 GQSC 평가/callback | callback 수 |
|---:|---:|
| 1 | 61 |
| 2 | 41 |
| 3 | 2 (`DVE039`, `V2E04`) |

동일-input primary cache hit는 43회였고, 수정 후 같은 입력을 다시 계산한 횟수는 0이다. 41개 callback은
safe-stop hypothetical probe 1회, 2개 callback은 2회를 수행해 관측 최대 새 평가는 3회였다. 수정 전
counterfactual은 1/3/4회 분포였으므로 43개의 동일 primary 평가를 제거했다. 상세 행은
[duplicate_invocation_analysis.csv](duplicate_invocation_analysis.csv)에 있다.
