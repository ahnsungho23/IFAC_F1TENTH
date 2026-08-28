# Local planner behavior contract

이 문서는 현재 코드를 더 안전하거나 더 좋은 코드라고 선언하지 않는다. Cleanup 리팩터링으로
변하면 안 되는 관측값과, 현재 자동화가 실제로 비교할 수 있는 범위를 구분한다.

## Observable behavior matrix

| 관측 항목 | 현재 존재 위치 | baseline 방법 | 자동 비교 수준 |
|---|---|---|---|
| Selected path identity | `P3ShadowResult::selected_candidate_identity` | 대표 하니스 TSV summary | 정확 비교 가능 |
| Selected path digest | `selected_path_digest` | 대표 하니스 TSV summary 및 전체 출력 hash | 정확 비교 가능 |
| `/avoid_waypoints` geometry | `publishResult()`의 outgoing `OTWpntArray` | offline selected path digest와 geometry unit test | 부분 비교만 가능 |
| Waypoint count/order | path digest가 count와 vector 순서를 포함, ordering gTest 존재 | digest 및 `ShiftsOnlyOrderedGlobalRaceLineSamples` | 핵심 경로 비교 가능 |
| Candidate count | `candidate_count`, M0/M1 count | TSV summary | 정확 비교 가능 |
| Candidate ordering | `result.candidates` 출력 순서 | TSV candidate row 순서와 전체 출력 hash | 정확 비교 가능 |
| Selected candidate | selected identity/digest | TSV summary와 candidate rows | 정확 비교 가능 |
| Rejection reason | candidate `rejection_reason`, result failure classification | TSV `validator_result` | 정확 비교 가능 |
| Safe-stop/fallback decision | `SplinePlanKind`, `SafeStopLifecycleDecision` | planner/safe-stop gTest | 판정 계약 비교 가능, end-to-end 미지원 |
| Commitment decision | `P3ManeuverLifecycleDecision` | lifecycle gTest 19개 | synthetic lifecycle 비교 가능 |
| Major diagnostic JSON | P3 cycle 및 replay diagnostic publishers | node test의 일부 stable field | 전체 payload 자동 비교 불가 |

## Path digest가 고정하는 것

현재 path digest는 waypoint vector의 크기를 먼저 hash하고 각 waypoint를 현재 순서대로 순회해
다음 double bit pattern을 hash한다.

1. `s_m`
2. `d_m`
3. `x_m`
4. `y_m`
5. `psi_rad`
6. `kappa_radpm`
7. `vx_mps`
8. `ax_mps2`

따라서 위 필드의 값, waypoint 수, 순서가 하나라도 바뀌면 digest가 달라진다. 다음은 digest에
포함되지 않는다.

- waypoint `id`
- waypoint `d_left`, `d_right`
- message header/frame/stamp
- `OTWpntArray.ot_side`, `ot_line`, `side_switch`, `last_switch_time`
- publish 직전 raw slowdown 이외의 외부 node 상태

즉 digest equality는 강한 offline path 계약이지만 `/avoid_waypoints` 전체 메시지 equality는
아니다.

## Representative set

| 요구 성격 | 선택한 기존 자산 | 보장 범위 | 부족한 범위 |
|---|---|---|---|
| Obstacle 없는 정상 주행 | `IgnoresObstacleWithEnoughRawRacelineClearance`, `ExplicitlyEmptyObstacleArrayIsStillAccepted` | no-blocking 결과와 explicit-empty 입력 수용 | 완전 빈 frozen stream 및 실제 GLOBAL 주행 출력 없음 |
| 단일 confirmed static obstacle | `pinch_success.stream` | 1 frame, 1 obstacle, left candidate 선택과 digest | 현재 live confirmed topic replay가 아님 |
| 좌측/우측 회피 | `pinch_success` left, `straight_margin_crawl` frame 0 right/frame 1 left | 양쪽 selected identity/digest | 차량 폐루프 추종 없음 |
| Candidate rejection | `failing_cluster1.stream` | 24개 후보의 순서와 개별 rejection, 최종 `NO_HARD_VALID_M1_CANDIDATE` | node safe-stop publish까지 연결되지 않음 |
| Committed continuation | `test_p3_maneuver_lifecycle` | immutable identity/digest, suffix trim, containment, invalidation | `onPlanningTimer()`의 continuation-first 호출 순서 전체는 직접 dump하지 않음 |
| Safe-stop/fallback | `BuildsCollisionFreeStopWhenBothSidesAreClosed`, `test_safe_stop_lifecycle` | stop path 성질과 release ladder | state machine을 포함한 end-to-end fallback 없음 |

새 scenario는 만들지 않았다. 위 부족분은 이번 baseline의 명시적 coverage gap이다.

## Candidate and ranking invariants

Cleanup 리팩터링 후 다음이 모두 같아야 한다.

- 후보 생성 순서와 generation index
- 후보 identity와 logical/source identity
- M0/M1 invocation 여부와 각 candidate count
- hard validator 호출 수
- 후보별 hard-valid와 rejection reason
- selected identity와 selected digest
- exit-next-obstacle 강등, braking deficit, velocity loss, safety slack, deviation,
  generation-order tie break의 기존 우선순위

대표 TSV 전체 출력 hash가 이를 한 번에 감시한다. Hash가 달라졌다면 의도하지 않은 formatting
변경일 수도 있으므로, candidate row를 직접 diff해 원인을 분류한다.

## Continuation and collision invariants

- Active immutable maneuver가 있으면 fresh evaluation보다 continuation을 먼저 수행한다.
- Frozen suffix가 hard-valid하면 evaluator를 호출하지 않는 lazy 동작을 유지한다.
- Fresh selection과 committed suffix revalidation은 동일한 maneuver collision horizon을 사용한다.
- Guard containment, guarded validation, raw validation의 순서를 유지한다.
- Current raw collision은 기존 confirmation/즉시 invalidation 규칙을 그대로 따른다.
- Completion은 기존 expanded cluster end와 progress 규칙을 유지한다.

Lifecycle gTest는 이 판정의 상당 부분을 고정하지만 node timer의 실제 call trace를 직렬화하지는
않는다.

## Safe-stop/fallback invariants

- Both-side rejection에서 기존 safe-stop/fallback 종류를 유지한다.
- Safe-stop 경로의 waypoint 순서와 zero-speed stop 성질을 유지한다.
- Release condition A/B/C 및 blind timeout 우선순위를 유지한다.
- Repeated/stale empty frame을 clear evidence로 승격하지 않는다.
- Valid avoidance release debounce와 state selectability 조건을 유지한다.
- Safe-stop latch가 활성일 때 다른 경로가 교대로 발행되지 않도록 한다.

현재 baseline은 lifecycle 단위 판정을 검증한다. `/state`, `/avoid_waypoints`, controller까지 포함한
통합 순서는 자동 비교하지 못한다.

## Diagnostic JSON contract

현재 코드에 실제로 존재하는 주요 P3 cycle 안정 필드는 다음과 같다.

- schema, mode, source epoch/stamp, Frenet stamp, reference generation
- obstacle sequence와 ingress sequence
- snapshot readiness/rejection
- obstacle 및 cluster identity/envelope
- candidate/M0/M1 count
- selected source/cell/branch/candidate identity/path digest
- fresh hard-valid/rejection
- lifecycle state/reason/output digest
- path owner와 backup-only 여부

Replay candidate audit에는 generation index, feasible/selected, side, target/entry/exit,
clearance/curvature/rate/slack, braking deficit, rejection reason, final rank가 존재한다.

하지만 timestamp, callback sequence, heartbeat/preroll과 subscriber 존재 여부가 payload를
바꾸므로 전체 JSON 문자열은 현재 결정론적 artifact로 동결하지 못했다. 리팩터링 중에는 key
삭제·이름 변경·의미 변경을 금지하고, exact payload comparator는 별도 작업으로 다룬다.

## Existing known failures

두 contract test 실패는 baseline에 이미 존재한다.

- `stuck_case_harness.safety_margin_m=0.05`, 운영 YAML `0.08`
- planner real `control_max_steering_right_rad=0.361`, control launch `0.41`

Cleanup 리팩터링에서 이 값을 고치거나 실패를 숨기지 않는다. 별도 parameter/control 계약
작업으로 분리해야 한다.
