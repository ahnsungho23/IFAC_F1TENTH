# Perception에서 planning으로

## 1. 왜 topic 하나로 끝내지 않는가

[GENERAL THEORY] LiDAR 한 프레임의 점군은 벽 반사, 부분 가림, pose 오차 때문에 흔들린다. 즉 “이번 프레임에 상자가 보였다”와 “실제로 정지해 있는 장애물이다”는 다른 주장이다. 빠른 반응과 안정적인 경로 결정에 같은 증거 문턱을 쓰면, 너무 늦게 반응하거나 유령 장애물에 경로가 흔들린다.

[CURRENT IMPLEMENTATION] detector는 scan을 map frame으로 옮기고 구조 벽을 거른 뒤 cluster와 map-frame AABB를 만들고, AABB를 CLCS Frenet envelope로 투영하여 track한다. 전체 개요는 [`obstacle_detector_node.hpp:1`](../../src/obstacle_detector/include/obstacle_detector/obstacle_detector_node.hpp#L1), CLCS 구성은 [`obstacle_detector_node.cpp:550`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L550), AABB 투영은 [`obstacle_detector_node.cpp:1623`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1623)에 있다.

```text
/scan + map TF + /map + /global_waypoints
                 │
      wall filter / clustering
                 │
          map-frame AABB
                 │
       Cartesian → Frenet envelope
                 │
         tracking + motion vote
          ┌──────┼──────────┐
          v      v          v
    /static_obs  /confirmed_static_obs  /opp_obs
    빠른 정적층     Static 확정층          Dynamic 최근접
          │              │
      speed hint      path authority
```

## 2. raw, existence-confirmed, static-confirmed

[CURRENT IMPLEMENTATION] 이름 때문에 `/static_obs`를 “아무 처리 없는 raw scan”으로 이해하면 안 된다. 현재 detector는 `TrackStatus::Confirmed`이고 envelope stability 조건을 만족하며 Dynamic이 아닌 track을 `/static_obs`에 싣는다. 그중 motion status까지 `Static`인 subset을 `/confirmed_static_obs`에 싣는다([`obstacle_detector_node.cpp:1756`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1756)). planner 코드와 YAML에서는 전자를 raw slowdown 입력이라고 부르므로, 이 교재도 planner 관점에서 “raw”라고 줄여 부른다.

| planner 관점 이름 | topic | 의미 | local planner 권한 |
|---|---|---|---|
| raw/provisional static | `/static_obs` | existence와 envelope는 안정됐지만 motion Static 확정 전인 Unknown 포함 | 속도 하향 hint만 |
| confirmed static | `/confirmed_static_obs` | detector가 Static으로 분류한 object-level envelope | P3 경로 형상·collision authority |
| dynamic opponent | `/opp_obs` | Dynamic object 중 전방 최근접 | 이 static local planner의 P3 입력 아님 |

[CURRENT IMPLEMENTATION] local planner 기본 subscription은 `obstacles_topic=/confirmed_static_obs`이고, 별도 `raw_slowdown_topic=/static_obs`를 구독한다([`local_planner_node.cpp:776`](../../src/local_planning/src/local_planner_node.cpp#L776), [`local_planning.yaml:814`](../../src/local_planning/config/local_planning.yaml#L814)).

## 3. Frenet envelope는 무엇인가

변수를 먼저 정의한다.

- `s_start`, `s_end`: 장애물이 차지하는 기준선 진행 방향 구간
- `d_right`, `d_left`: 장애물의 오른쪽·왼쪽 횡경계
- `s_center`, `d_center`: envelope 중심
- `id`: 여러 프레임에서 같은 물체를 연결하는 track identity

```text
왼쪽(+d)
  d_left  ┌───────────────┐
          │   obstacle    │   s_start → s_end
 d_center │       ×       │
 d_right  └───────────────┘
                진행(+s) →
```

[CURRENT IMPLEMENTATION] detector는 map-frame axis-aligned bounding box의 중심과 네 모서리를 함께 CLCS에 투영해 독립적인 longitudinal/lateral extent를 보존한다([`obstacle_detector_node.cpp:1623`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1623)). layer merge 뒤에도 보이는 member AABB union을 다시 투영해 published Frenet bounds와 RViz Cartesian box의 계약을 맞춘다([`obstacle_detector_node.cpp:1274`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1274)).

## 4. 왜 raw는 속도만 낮추는가

[GENERAL THEORY] 경로 형상은 몇 초 동안 차량이 따라갈 기하를 약속한다. 아직 Static인지 Dynamic인지 확정되지 않은 상자에 대해 매 프레임 좌우 경로를 바꾸면 false positive가 lateral command로 증폭된다. 반면 감속은 `min(기존 속도, cap)` 형태로 넣으면 경로 기하를 보존하면서 더 많은 관측 시간을 살 수 있다.

[CURRENT IMPLEMENTATION] raw slowdown은 `/static_obs`가 전방 race line을 위협할 때 기존 waypoint 속도를 올리지 않고 낮추는 overlay다. confirmed 경로를 생성하거나 좌우 회피 side를 고르는 권한은 없다. 특히 committed path에서는 운영 YAML의 `raw_slowdown_skip_committed: true` 설정으로 중복 영향도 제한한다([`local_planning.yaml:301`](../../src/local_planning/config/local_planning.yaml#L301)).

[INFERENCE] 이 분리는 “raw는 중요하지 않다”는 뜻이 아니다. raw는 반응 시간 authority, confirmed는 lateral geometry authority를 가진다. 안전 주장은 두 채널의 지연과 false-positive/false-negative 특성을 함께 측정해야 한다.

## 5. 정적과 동적의 차이

[GENERAL THEORY] 정적 장애물 collision은 공간상 두 영역이 겹치는지를 보면 된다. 동적 장애물은 같은 공간을 서로 다른 시간에 지나갈 수 있으므로 `s(t),d(t)` 예측과 ego trajectory time parameterization이 필요하다.

[CURRENT IMPLEMENTATION] P3 validator는 confirmed static Frenet box를 공간 envelope로 검사하며 obstacle velocity를 시간 적분하지 않는다. `/opp_obs`는 detector가 별도 dynamic layer로 발행하지만 이 local planner의 `obstacles_topic`이 아니다. 따라서 “메시지에 velocity가 있으니 P3가 moving obstacle prediction을 한다”는 설명은 틀리다.

## 6. authority의 단계

```text
scan point
  │  관측 증거
  v
detection / track
  │  존재·envelope 안정성
  v
/static_obs ── 속도만 보수적으로 낮춤
  │  motion Static 증거
  v
/confirmed_static_obs ── P3 lateral geometry 생성·검증
  │  hard-valid + lifecycle ownership
  v
/avoid_waypoints ── controller가 실제 추종
```

[CURRENT IMPLEMENTATION] fresh P3 selection도 한 번 더 conservative guarded obstacle과 raw obstacle에 대해 exact hard validation을 각각 통과해야 lifecycle ownership을 얻는다([`p3_maneuver_lifecycle.cpp:149`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L149)). 즉 topic을 받았다고 곧바로 발행 authority가 생기는 것이 아니다.

## 7. 숫자 예제

[GENERAL THEORY] 자차가 `6 m/s`이고 confirmation에 추가로 `0.10 s`가 걸린다고 하자. 그동안 이동거리는 단순히

`distance = speed × delay = 6 × 0.10 = 0.60 m`

이다. 이 때문에 raw speed hint는 0.1초를 “기다리는” 대신 먼저 감속을 요청할 가치가 있다. 그러나 0.60 m는 perception latency만 본 값이며 DDS, planner, controller, actuator 지연은 별도다.

[OPEN QUESTION] current YAML 주석의 delay 가정이 실제 대회 end-to-end latency의 상위 분위수를 덮는지는 source만으로 확인되지 않는다. raw와 confirmed가 같은 물리 장애물 ID·envelope로 얼마나 안정적으로 이어지는지도 로그 기반 검증이 필요하다.

## 8. 자주 혼동하는 개념

- `/static_obs`는 진짜 raw LaserScan이 아니다. planner가 confirmed-only layer와 대비해 raw slowdown이라고 부르는 object layer다.
- 존재 confirmation과 motion Static confirmation은 다르다.
- AABB가 map frame에서 axis-aligned라는 사실과 Frenet envelope가 `s/d`축 상자라는 사실은 다르다.
- empty array는 언제나 “안전함”의 증거가 아니다. staleness, blind area, lifecycle 기억을 함께 봐야 한다.
- 경로 authority와 속도 authority는 분리될 수 있다.

## 반드시 설명할 수 있어야 하는 질문

1. `/static_obs`가 완전히 raw한 scan point 배열이 아닌 이유는 무엇인가?
2. `/confirmed_static_obs`만 P3 lateral geometry authority를 갖는 이유는 무엇인가?
3. Frenet envelope의 네 경계는 각각 무엇을 뜻하는가?
4. 정적 collision과 동적 collision의 수학적 차이는 무엇인가?
5. raw slowdown이 false positive를 lateral jitter로 증폭하지 않는 이유는 무엇인가?

