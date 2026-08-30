# Planning lifecycle

## 1. 왜 매 프레임 새 최적해를 쓰지 않는가

[GENERAL THEORY] perception envelope가 몇 cm 흔들릴 때마다 path를 다시 만들면 target `d`, curvature, speed cap, controller lookahead가 연속으로 변한다. 차량은 이전 명령을 이미 실행 중이므로 독립적인 frame-wise optimum은 closed-loop에서 오히려 불안정할 수 있다.

[CURRENT IMPLEMENTATION] 현재 P3 lifecycle은 한 maneuver의 path와 obstacle identity를 immutable record로 저장하고, 이후에는 ego 앞 suffix만 현재 snapshot에 대해 재검증한다([`p3_maneuver_lifecycle.hpp:135`](../../src/local_planning/include/local_planning/p3_maneuver_lifecycle.hpp#L135)). generic “이전 path bonus”가 아니라 특정 장애물 기동의 ownership state다.

## 2. 상태 그림

```text
                fresh hard-valid P3
       IDLE ─────────────────────────> FRESH_SELECTED
                                           │ next valid cycle
                                           v
                                      COMMITTED
                                           │ suffix valid
                                           v
                                      CONTINUING ──────┐
                                           │          │ suffix valid
                     obstacle region passed│          └─────────
                                           v
                                       COMPLETE

 FRESH_SELECTED / COMMITTED / CONTINUING
              │ lineage, stale, geometry, collision failure
              v
          INVALIDATED

 COMPLETE와 INVALIDATED는 다음 callback에 IDLE로 reset되는 transition event
```

[CURRENT IMPLEMENTATION] enum의 실제 상태는 `Idle`, `FreshSelected`, `Committed`, `Continuing`, `Invalidated`, `Complete`다([`p3_maneuver_lifecycle.hpp:35`](../../src/local_planning/include/local_planning/p3_maneuver_lifecycle.hpp#L35)).

## 3. fresh selection

[CURRENT IMPLEMENTATION] fresh result가 ownership을 얻으려면 다음이 필요하다([`p3_maneuver_lifecycle.cpp:119`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L119)).

- evaluator가 invoked되고 hard-valid recovery path를 냈다.
- source가 stale하지 않고 safe-stop authority가 없다.
- source stamp/epoch/reference generation이 snapshot과 일치한다.
- obstacle identity와 cluster end가 완전하다.
- guarded conservative geometry exact validation 통과
- same-callback raw geometry exact validation 통과

통과하면 original path, side, obstacle IDs/guards, creation ego `s`, cluster end, identity, path digest를 record에 저장한다([`p3_maneuver_lifecycle.cpp:198`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L198)).

## 4. continuation-first

[CURRENT IMPLEMENTATION] active maneuver가 있으면 `advanceP3Lifecycle()`은 evaluator보다 `continueCurrent()`를 먼저 호출한다. valid output 또는 complete가 나오면 evaluator를 호출하기 전에 return한다([`local_planner_node.cpp:3203`](../../src/local_planning/src/local_planner_node.cpp#L3203)).

이 순서의 효과는 두 가지다.

- geometry freeze: 같은 original path의 suffix만 발행하므로 envelope noise가 path shape로 보이지 않는다.
- computation saving: 새 corridor/root/candidate/validation을 매 callback 반복하지 않는다.

[INFERENCE] 이것은 “옛 경로를 무조건 신뢰”하는 hysteresis가 아니다. suffix는 current obstacle geometry와 lineage에 대해 계속 exact validation을 받는다.

## 5. suffix와 progress

[CURRENT IMPLEMENTATION] current ego `s`에서 original waypoint 각각의 forward distance를 구해 유일한 nearest progress index를 찾고, 그 index부터 진행 순서가 유효한 점만 suffix로 복사한다([`p3_maneuver_lifecycle.cpp:372`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L372)). index가 뒤로 가거나 projection tie가 생기면 invalidation한다.

completion check는 suffix minimum-point validation보다 먼저 한다. ego가 recorded expanded cluster end를 지났다면 짧은 terminal suffix 때문에 “invalid maneuver”로 오판하지 않고 `Complete`로 넘긴다([`p3_maneuver_lifecycle.cpp:327`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L327)).

## 6. frozen guard와 retention band

[CURRENT IMPLEMENTATION] selection 때 저장한 obstacle guard가 current same-ID envelope를 포함하면 frozen guard로 validation하여 progressive reveal의 작은 흔들림을 path에 노출하지 않는다. guard가 깨져 obstacle collision이 나면 raw geometry로 다시 검사한다([`p3_maneuver_lifecycle.cpp:431`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L431)).

[CURRENT IMPLEMENTATION] raw full-reserve collision만 난 경우에는 tracking reserve의 `commitment_retention_reserve_fraction`만 남긴 검사를 한 번 더 한다. physical base clearance는 줄이지 않는다([`p3_maneuver_lifecycle.cpp:514`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L514)). retention도 실패하면 `CURRENT_RAW_OBSTACLE_COLLISION`으로 invalidation한다.

[INFERENCE] retention band는 fresh candidate의 안전마진을 낮추는 기능이 아니다. 이미 선택된 geometry를 perception margin jitter 때문에 매번 바꾸지 않기 위한 commitment-specific band다.

## 7. invalidation과 same-callback replan

[CURRENT IMPLEMENTATION] epoch/reference mismatch, stale source, stamp regression, backward/wrap progress, conflicting non-empty obstacle identity, non-obstacle exact validation failure, actual raw/retention collision 등은 record를 없애고 `Invalidated`로 만든다([`p3_maneuver_lifecycle.cpp:304`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L304)).

[CURRENT IMPLEMENTATION] continuation이 invalidated된 같은 callback에 lazy evaluator가 fresh hard-valid path를 만들 수 있으면 바로 `selectFresh()`로 넘어간다. 따라서 단순히 한 cycle 동안 authority를 비운 뒤 다음 cycle을 기다리는 구조는 아니다([`local_planner_node.cpp:3222`](../../src/local_planning/src/local_planner_node.cpp#L3222)). recovery가 없으면 기존 safety/fallback cycle이 맡는다.

## 8. commitment, side lock, sequential planning

[CURRENT IMPLEMENTATION] safety-cycle 쪽 commitment도 valid하면 frozen path를 재발행하고, actual retention-margin violation 또는 non-obstacle failure에서만 replacement를 계획한다([`local_planner_node.cpp:3976`](../../src/local_planning/src/local_planner_node.cpp#L3976)). engagement 뒤에는 preferred side를 잠그며, engagement 전에는 조건에 따라 한 번 side switch를 허용한다([`local_planner_node.cpp:4043`](../../src/local_planning/src/local_planner_node.cpp#L4043)).

[CURRENT IMPLEMENTATION] 현재 maneuver의 obstacle collision horizon 뒤 장애물은 next maneuver로 분리한다. merge/cluster completion 때 next obstacle을 위한 chained maneuver를 시작하거나 global handoff loop로 넘긴다([`local_planner_node.cpp:3865`](../../src/local_planning/src/local_planner_node.cpp#L3865)).

## 9. handoff와 non-empty 계약

[CURRENT IMPLEMENTATION] `/state`가 AVOID인 동안 단순 empty `/avoid_waypoints`를 내면 FSM이 GLOBAL 복귀 판정을 못 하는 상황이 있다. 그래서 blocking obstacle이 사라졌어도 AVOID state라면 ego를 덮는 closed global handoff loop를 발행하고, STATE_GLOBAL 확인 뒤에야 commitment를 지우고 empty를 낸다([`local_planner_node.cpp:4120`](../../src/local_planning/src/local_planner_node.cpp#L4120)).

이것은 중요한 external behavior contract다.

- 정상 AVOID guidance: non-empty
- global handoff confirmation을 기다리는 동안: non-empty `d=0` loop
- safe-stop: 일반적으로 non-empty braking/hold path
- 정말 GLOBAL로 복귀가 확인된 뒤: empty 가능

## 10. safe-stop lifecycle

[CURRENT IMPLEMENTATION] avoidance candidate가 없거나 commitment replacement가 불가능하면 stop path를 latch한다. safe-stop은 한 callback 후 자동 해제되지 않는다. release는 top-level OR다([`safe_stop_lifecycle.cpp:160`](../../src/local_planning/src/safe_stop_lifecycle.cpp#L160)).

```text
A: 장애물 위험구간을 margin과 함께 지나침
OR
B: latched obstacle에 대한 hard-valid avoidance가 연속 확인되고 FSM이 선택 가능
OR
C: 차량이 정지했고 forward corridor clear가 신선한 frame에서 연속 확인
OR
D: 정지 + 위험구간 전방 + 신선한 empty frame이 장시간 지속
   → blind timeout creep handoff
```

`B`는 avoidance path를 선택할 권한만 주며 global handoff를 허용하지 않는다. `D`는 물체가 근접 사각에 남았을 가능성을 고려해 기억 위험구간에 creep speed cap을 적용한다.

## 11. 숫자 예제

[GENERAL THEORY] planning period가 `25 ms`이고 safe-stop release confirmation이 `8 cycles`라면 이상적인 최소 연속 확인 시간은

`8×0.025=0.20 s`

이다. 그러나 실제 release에는 fresh obstacle sequence, vehicle stopped, FSM selectability 등 branch별 조건이 추가된다. 따라서 “8이면 무조건 0.2초 뒤 해제”가 아니다.

[CURRENT IMPLEMENTATION] 현재 운영 YAML은 이 두 값을 `25 ms`, `8`로 적지만 실제 runtime override는 별도 확인해야 한다([`local_planning.yaml:679`](../../src/local_planning/config/local_planning.yaml#L679)).

## 12. 자주 혼동하는 개념

- fresh selected와 committed는 같은 callback 상태 이름이 아니다.
- path freeze는 validation freeze가 아니다.
- empty detector frame 하나는 obstacle이 사라졌다는 충분조건이 아니다.
- complete는 즉시 empty publication과 동일하지 않다. obstacle-ahead hold와 handoff가 남을 수 있다.
- safe-stop release 조건의 내부 AND와 top-level OR를 섞으면 안 된다.

## 반드시 설명할 수 있어야 하는 질문

1. continuation-first가 단순 previous-path preference와 다른 이유는 무엇인가?
2. fresh selection이 guarded와 raw validation을 모두 요구하는 이유는 무엇인가?
3. retention band에서 줄어드는 것과 절대 줄지 않는 것은 무엇인가?
4. completion check가 suffix validation보다 먼저인 이유는 무엇인가?
5. AVOID에서 non-empty handoff loop가 필요한 이유는 무엇인가?
6. safe-stop release A/B/C/D는 어떤 top-level 논리로 결합되는가?

