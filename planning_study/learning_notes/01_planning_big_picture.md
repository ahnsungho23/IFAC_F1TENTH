# Local planning 큰 그림

## 1. 왜 local planner가 필요한가

[GENERAL THEORY] global planner는 트랙 전체를 도는 기준 경로를 준다. 그러나 경기 중 기준 경로 위에 정적 장애물이 생기면 “트랙 전체 최적화”를 다시 하기보다, 자차 앞의 짧은 구간에서 기준 경로를 조금 옆으로 옮기고 다시 합류하는 편이 빠르고 결정적이다.

[CURRENT IMPLEMENTATION] 이 저장소의 local planner는 ordered global waypoint를 유지하면서 Frenet 횡오프셋 `d(s)`를 바꾸는 geometry-first planner다. 시간에 따른 차량 상태를 적분하는 완전한 kinodynamic optimizer는 아니다. 입력과 출력 계약은 [`local_planner_node.cpp:776`](../../src/local_planning/src/local_planner_node.cpp#L776) 부근과 운영 YAML의 topic 설정([`local_planning.yaml:814`](../../src/local_planning/config/local_planning.yaml#L814))에서 확인된다.

## 2. 한 장짜리 흐름도

```text
/global_waypoints ───────────────┐
/car_state/frenet/odom ─────────┤
/confirmed_static_obs ──────────┤
/static_obs (raw speed hint) ───┤
/state ─────────────────────────┘
                 │
                 v
        snapshot / freshness 확인
                 │
        active maneuver가 있는가?
          ┌──────┴──────┐
         yes            no
          │              │
 committed suffix       P3 좌/우 corridor
 현재 상태로 재검증      + M0/M1 후보 생성
          │              │
          └──────┬───────┘
                 v
          속도 성형 + hard validator
                 │
          hard-valid 후보만 ranking
                 │
       ┌─────────┴─────────┐
    회피 경로            후보 없음/입력 이상
       │                  │
 commitment         slow pass / safe-stop /
       │             emergency hold / handoff
       └─────────┬─────────┘
                 v
          /avoid_waypoints
```

## 3. `runSafetyPlanningCycle()`의 역할

[CURRENT IMPLEMENTATION] `runSafetyPlanningCycle()`은 이름 그대로 한 가지 후보 생성기만 실행하는 함수가 아니다. Frenet odometry의 유효성·staleness, perception staleness, 기존 commitment, initial stabilization, P3를 사용하는 `planner_.plan()`, chained maneuver, safe-stop latch, raw slowdown, global handoff까지 연결한다. 시작은 [`local_planner_node.cpp:3775`](../../src/local_planning/src/local_planner_node.cpp#L3775)이고, planning timer가 P3 mode에 따라 이 안전 사이클 또는 P3 lifecycle을 우선 호출하는 구조는 [`local_planner_node.cpp:3507`](../../src/local_planning/src/local_planner_node.cpp#L3507)에 있다.

초보자 관점에서 이 함수는 다음 질문에 순서대로 답하는 “교통정리 담당”이다.

1. 지금 계획할 입력이 준비됐는가?
2. 즉시 정지해야 할 만큼 입력이 오래됐는가?
3. 이미 약속한 회피 경로를 계속 써도 되는가?
4. 새 장애물에 대해 회피 후보를 만들어야 하는가?
5. 회피가 없으면 어떤 보수적 fallback을 발행해야 하는가?
6. 빈 경로를 내도 되는가, 아니면 FSM 복귀용 handoff를 내야 하는가?

## 4. 현재 P3의 위치

[CURRENT IMPLEMENTATION] 운영 YAML과 launch 기본값은 `p3_mode: TEST_ACTIVE`다([`local_planning.yaml:842`](../../src/local_planning/config/local_planning.yaml#L842), [`local_planning.launch.py:142`](../../src/local_planning/launch/local_planning.launch.py#L142)). 이 모드에서는 active committed suffix가 먼저 검증되며, 그것이 유효하면 P3 evaluator 자체를 호출하지 않는다. continuation이 실패하거나 active maneuver가 없을 때 P3를 평가한다([`local_planner_node.cpp:3203`](../../src/local_planning/src/local_planner_node.cpp#L3203)).

[CURRENT IMPLEMENTATION] 새 회피 형상을 만드는 production candidate generator는 P3 하나다. 옛 P0 quintic grid는 제거되었고 `RacelineSplinePlanner::plan()`도 내부에서 `generateP3Candidates()`를 호출한다([`raceline_spline_planner.cpp:3104`](../../src/local_planning/src/raceline_spline_planner.cpp#L3104), [`raceline_spline_planner.cpp:3480`](../../src/local_planning/src/raceline_spline_planner.cpp#L3480)). 진단의 `P0_*` 문자열과 `P0_BACKUP_ONLY`는 호환 계약 또는 기존 안전 사다리의 이름이지, 별도 P0 candidate grid가 살아 있다는 뜻이 아니다.

## 5. 입력과 출력의 의미

| 정보 | planner가 묻는 질문 |
|---|---|
| global waypoint | 기준 위치, 순서, 속도, heading, curvature, 좌우 track width는 무엇인가? |
| Frenet odometry | 자차는 기준선의 어디 `s`에 있고 얼마나 옆 `d`에 있으며 얼마나 빠른가? |
| confirmed static obstacle | 경로 형상을 바꿀 권한이 있는 정적 상자는 어디인가? |
| raw static obstacle | 승격 전 위험 때문에 속도만 미리 낮춰야 하는가? |
| state machine | AVOID에서 GLOBAL로 넘길 준비가 되었는가? |

[CURRENT IMPLEMENTATION] 출력 `/avoid_waypoints`는 단순한 선이 아니라 `x,y,s,d,psi,kappa,vx,ax`를 가진 ordered waypoint 배열이다. candidate는 global waypoint 순서를 복사하고 `d`, `x`, `y`를 바꾼 뒤 heading·curvature·acceleration을 다시 계산한다([`p3_shadow.cpp:1045`](../../src/local_planning/src/p3_shadow.cpp#L1045), [`raceline_spline_planner.cpp:2663`](../../src/local_planning/src/raceline_spline_planner.cpp#L2663)).

## 6. 한 프레임과 여러 프레임

[GENERAL THEORY] 한 프레임 최적화는 “지금 보이는 가장 좋은 경로”를 고른다. 실제 차량은 이전 명령의 영향을 받고 perception 상자는 조금씩 흔들리므로, 매 프레임 경로를 다시 고르면 좌우 또는 곡률이 흔들릴 수 있다.

[CURRENT IMPLEMENTATION] 그래서 현재 planner는 fresh selection 후 경로를 immutable maneuver로 기록하고, 다음 콜백에는 현재 자차 위치 앞의 suffix만 잘라 exact validator로 다시 검사한다([`p3_maneuver_lifecycle.cpp:119`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L119), [`p3_maneuver_lifecycle.cpp:272`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L272)). 유효하면 새 후보가 더 좋아 보이는지를 계산조차 하지 않는다.

## 7. 숫자 예제

[GENERAL THEORY] 기준선 앞 8 m에 장애물이 있고, 왼쪽으로 `0.6 m` 옮겼다가 5 m 뒤에 복귀한다고 하자. local planner의 기하 문제는 대략 다음과 같다.

- 현재 `d=0.05 m`에서 시작한다.
- 장애물 구간에서는 `d≈0.6 m`를 유지한다.
- track left boundary와 차량 폭을 침범하지 않는다.
- 인접 waypoint의 `d` 변화, Cartesian curvature, curvature rate가 한계 안이다.
- 복귀 후 controller가 볼 global tail도 충분히 남긴다.

[INFERENCE] 이것은 “장애물 중심에서 0.6 m 뺀 점 하나”를 목표로 하는 문제가 아니다. 진입·통과·복귀 전체 구간과 차량 속도를 함께 확인해야 한다.

## 8. 자주 혼동하는 개념

- `P3`는 전체 node나 lifecycle의 이름이 아니라 현재 회피 후보 생성·평가 계층이다.
- `safe-stop`은 빈 경로가 아니다. 정상적인 경우 장애물 전까지 이어지는 제동 waypoint path이며 마지막 속도가 0이다.
- `hard-valid`는 가장 좋은 후보라는 뜻이 아니다. 최소 제약을 통과했다는 뜻이고 그 뒤에 ranking이 있다.
- `TEST_ACTIVE`는 C++ unit test만을 뜻하지 않는다. 현재 launch가 쓰는 runtime mode 문자열이다.
- source의 기본값, YAML 값, launch override, 실제 실행 파라미터는 서로 다를 수 있다.

[OPEN QUESTION] 현재 정적 분석만으로 실제 대회 프로세스가 어느 override와 바이너리를 실행했는지는 확정할 수 없다. 이 교재의 “현재”는 checkout의 source/config 계약을 뜻한다.

## 반드시 설명할 수 있어야 하는 질문

1. local planner가 global waypoint 순서를 유지하는 이유는 무엇인가?
2. `runSafetyPlanningCycle()`은 어떤 여섯 질문을 조정하는가?
3. 현재 P3와 `P0_BACKUP_ONLY`는 어떤 관계인가?
4. hard-valid 후보와 selected 후보의 차이는 무엇인가?
5. continuation-first가 경로 jitter와 계산량을 동시에 줄이는 이유는 무엇인가?

