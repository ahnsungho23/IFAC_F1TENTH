# 충돌 검사와 feasibility gate

## 1. 검사 계층

현재 시스템의 안전 검사는 하나가 아니라 다음 계층으로 나뉜다.

1. detector: map wall/corridor 밖 cluster 제거, AABB 생성
2. planner context: obstacle/track interval로 side domain 계산
3. candidate hard validator: track, footprint, obstacle, geometry
4. lifecycle: conservative guard와 raw obstacle에 대한 재검증
5. velocity shaping/diagnostic: 횡가속, 가감속, steering rate
6. controller: steering angle/rate, speed ramp, odom watchdog

### 현재 판정 행렬

| 조건 | 분류 | 현재 threshold/식 | 정의 위치 | 물리적 의미 |
|---|---|---|---|---|
| forward path 존재/길이 | hard | `start_index < size`, 기본 최소 8 points | `minimum_path_points`, [`validateCandidate():2894`](../../src/local_planning/src/raceline_spline_planner.cpp#L2894) | controller에 전달할 전방 geometry가 너무 짧거나 비지 않았는지 |
| ego→entry 연속성 | hard, 두 조건 AND | lateral gap > max(tracking budget, 0.20 m) **AND** gap/max(forward,0.50 m) > 0.8 | `entry_discontinuity_min_budget_m`, `entry_continuity_baseline_m`, `maximum_lateral_slope` | 현재 차 위치에서 첫 path point로 순간 횡점프하지 않는지 |
| center track bound | hard | `-d_right+0.04 <= d <= d_left-0.04` | `wall_safety_margin_m=0.04`, [`trackBoundaryReserve():573`](../../src/local_planning/src/raceline_spline_planner.cpp#L573) | waypoint center가 track interval 안인지 |
| rotated footprint wall bound | hard | 네 corner 최소 clearance >= 0; 차체 0.56 x 0.30 m, wall reserve 0.04 m | `vehicle_length_m`, `vehicle_half_width_m`, `wall_safety_margin_m` | 차 중심이 아니라 실제 회전 직사각형 전체가 벽 안인지 |
| static obstacle interval | hard | obstacle face ± (`0.15+0.08+reserve`)와 `d(s)`가 겹치지 않음; 현재 reserve mode `none`이라 0.23 m | `vehicle_half_width_m`, `safety_margin_m`, `obstacle_reserve_mode`; [`obstacleSafetyClearance():565`](../../src/local_planning/src/raceline_spline_planner.cpp#L565) | 차체 반폭과 추가 margin을 obstacle 면에서 확보하는지 |
| forward `s` 순서 | hard | 인접 `Delta s > epsilon` | [`validateCandidate():2992`](../../src/local_planning/src/raceline_spline_planner.cpp#L2992) | global path 진행 순서를 역행하지 않는지 |
| lateral spatial slope | hard | `|Delta d|/Delta s <= 0.8` | `maximum_lateral_slope` | offset transition이 지나치게 급하지 않은지; steering angle 자체는 아님 |
| 방향별 curvature | hard | `|kappa| <= min(1.3162665, tan(delta_side)/0.33)`; `delta_L=0.410`, `delta_R=0.361` rad | curvature/control steering parameters | 좌우 actuator 조향각 geometry로 낼 수 있는 곡률인지 |
| spatial curvature rate | hard | `|Delta kappa|/Delta s <= 20.0 1/m^2` | `maximum_curvature_rate_radpm2` | path geometry의 급격한 조향 변화 억제; 시간 steering rate는 아님 |
| lateral acceleration | speed shaping | `v^2|kappa| <= a_y,max(v)`; 6.5~7.6 m/s² table | velocity-limit arrays | 같은 곡률에서 속도를 낮춰 횡가속 예산을 맞춤 |
| longitudinal accel/decel | speed shaping + diagnostic | speed별 accel 3.0~3.7, decel 2.0 m/s² | velocity-limit arrays | waypoint 간 속도 변화를 차량 종가속 budget에 맞춤 |
| modeled steering rate | diagnostic/controller hard clamp | planner diagnostic 20 rad/s, controller 최종 rate limiter | `control_max_steering_rate_radps`, controller config | path가 요구할 조향 변화와 actuator 명령 한계의 차이 관찰 |
| exit와 다음 obstacle | soft rank demotion | boolean `exit_reaches_next_obstacle` | [`p3_shadow.cpp:1153`](../../src/local_planning/src/p3_shadow.cpp#L1153) | 현재 cluster 뒤 merge가 다음 obstacle 책임영역까지 침범하는 후보를 뒤로 보냄 |
| clearance/curvature 여유 | soft rank | 네 normalized slack의 minimum을 크게 | `measureCandidate()` | hard threshold를 통과한 후보 중 상대 여유가 큰 것을 선호 |

따라서 현재 구현은 “collision-free만 보는 순수 기하 검사”보다 강하지만, 모든 state/control을 시간축에서 전개해 feasibility를 hard guarantee하는 검사는 아니다. 특히 shaping과 diagnostic 행을 hard reject로 읽으면 안 된다.

## 2. obstacle collision model

planner가 사용하는 기본 obstacle collision은 Frenet rectangle overlap이다.

waypoint의 forward station이 obstacle `[start,end]` 안이고

```text
obstacle_test_right < waypoint.d < obstacle_test_left
```

이면 collision이다. test lateral bounds는 raw face에 vehicle half width+safety/tracking reserve를 더한 값이다: [`raceline_spline_planner.cpp:2974`](../../src/local_planning/src/raceline_spline_planner.cpp#L2974).

중요한 한계: obstacle에 대해서는 ego vehicle의 회전 직사각형과 Cartesian AABB의 polygon collision을 직접 계산하지 않는다. lateral centerline `d(s)`에 inflated Frenet interval을 적용한다. 반면 벽은 vehicle 네 모서리를 직접 검사한다.

## 3. track centerline gate

각 waypoint의 nearest global reference에서 left/right width를 읽고 `trackBoundaryReserve(v,kappa)`를 뺀다.

```text
-right_width + reserve <= d <= left_width - reserve
```

를 벗어나면 reject한다: [`raceline_spline_planner.cpp:2937`](../../src/local_planning/src/raceline_spline_planner.cpp#L2937).

## 4. rectangular footprint wall gate

차체는 길이 0.56 m, 폭 0.30 m의 회전 직사각형이다.

1. waypoint pose에서 네 corner를 만든다.
2. waypoint `s` 주변 reference segment만 후보로 둔다.
3. 각 corner를 local segment에 projection한다.
4. segment의 `d_left/d_right`를 보간한다.
5. wall safety reserve를 한 번 빼고 최소 clearance를 구한다.

구현은 [`raceline_spline_planner.cpp:2192`](../../src/local_planning/src/raceline_spline_planner.cpp#L2192)다. local segment window는 snake-shaped track의 인접한 다른 branch로 corner가 붙는 것을 막는다.

최소 corner clearance가 음수면 hard reject다: [`raceline_spline_planner.cpp:2331`](../../src/local_planning/src/raceline_spline_planner.cpp#L2331).

## 5. geometry gates

### entry continuity

ego와 가장 가까운 전방 path point 사이에서 다음 두 조건이 모두 참일 때만 reject한다.

```text
|d_entry-d_ego| > max(tracking reserve, 0.20 m)
AND
|d_entry-d_ego| / max(forward_distance, 0.50 m) > 0.8
```

근거: [`raceline_spline_planner.cpp:2854`](../../src/local_planning/src/raceline_spline_planner.cpp#L2854), [`local_planning.yaml:567`](../../src/local_planning/config/local_planning.yaml#L567).

### ordered `s`

후보 내부 forward station 차가 양수여야 한다. global waypoint 순서를 유지한다: [`raceline_spline_planner.cpp:2992`](../../src/local_planning/src/raceline_spline_planner.cpp#L2992).

### lateral slope

```text
|Delta d| / Delta s <= 0.8
```

이다. 이는 steering angle 그 자체가 아니라 Frenet offset transition의 공간 기울기다.

### curvature와 curvature rate

```text
|kappa_i| <= min(kappa_legacy, tan(delta_side)/wheelbase)
|Delta kappa|/Delta s <= 20 rad/m^2
```

근거: [`raceline_spline_planner.cpp:3005`](../../src/local_planning/src/raceline_spline_planner.cpp#L3005).

## 6. collision horizon

candidate obstacle collision은 전체 path tail이 아니라 현재 maneuver가 책임지는 cluster end+post-merge 범위까지만 검사할 수 있다. track boundary와 geometry는 여전히 전체 path에서 검사한다: [`p3_shadow.cpp:1103`](../../src/local_planning/src/p3_shadow.cpp#L1103).

이 설계는 다음 장애물까지 포함된 controller tail이 현재 장애물의 모든 후보를 전멸시키는 것을 막는다. 대신 exit가 다음 obstacle에 닿는지는 별도 flag로 계산해 ranking에서 강등한다: [`p3_shadow.cpp:1153`](../../src/local_planning/src/p3_shadow.cpp#L1153).

## 7. uncertainty guard

`buildUncertaintyGuard()`는 원칙적으로

```text
longitudinal inflation = min_long + sigma_scale sqrt(s_var)
lateral face inflation = clamp(face_sigma rule, min, max)
```

를 적용한다: [`obstacle_guard.cpp:64`](../../src/local_planning/src/obstacle_guard.cpp#L64).

하지만 현재 운영 YAML은

```text
uncertainty_sigma_scale = 0
min longitudinal = 0
min lateral = 0
max lateral = 0
```

이므로 covariance 기반 팽창은 실질적으로 꺼져 있다: [`local_planning.yaml:703`](../../src/local_planning/config/local_planning.yaml#L703).

초기 stabilization에서는 같은 ID의 여러 관측 envelope를 union하여 conservative input을 만들고, selection 후에는 frozen guard와 live/raw geometry를 비교한다: [`local_planner_node.cpp:1435`](../../src/local_planning/src/local_planner_node.cpp#L1435), [`p3_maneuver_lifecycle.cpp:311`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L311).

## 8. 속도 feasibility

### 생성 단계에서 적용되는 shaping

- curvature lateral acceleration cap
- gap-based tracking reserve cap
- confirmed obstacle critical speed hold
- response delay distance
- forward acceleration pass
- backward deceleration pass

근거: [`raceline_spline_planner.cpp:2344`](../../src/local_planning/src/raceline_spline_planner.cpp#L2344).

### publish 전 diagnostic

`inspectVelocityFeasibility()`가 최종 outgoing path에서 다음을 센다.

- lateral acceleration cap 위반
- accel/decel table 위반
- modeled steering rate 위반

근거: [`raceline_spline_planner.cpp:1587`](../../src/local_planning/src/raceline_spline_planner.cpp#L1587).

이 report는 원칙적으로 diagnostic이다. 위반했다고 path를 모두 폐기하지 않는다. 이미 braking start를 놓친 상황에서는 더 낮은 profile과 collision-free geometry를 유지하는 5C fallback을 택한다: [`local_planner_node.cpp:4306`](../../src/local_planning/src/local_planner_node.cpp#L4306).

## 9. 현재 직접 검사하지 않는 것

- continuous-time swept vehicle polygon과 obstacle polygon의 충돌
- waypoint 사이 continuous obstacle collision
- obstacle motion prediction을 포함한 space-time collision
- hard steering rate `rad/s`
- tire friction circle/ellipse
- yaw dynamics와 slip angle
- localization covariance 기반 chance constraint
- map boundary 자체의 uncertainty

이 부재가 즉시 unsafe라는 뜻은 아니다. current margins, dense waypoint, controller limiter가 일부를 흡수한다. 다만 코드상 formal guarantee는 없다.

## 10. 검증 우선순위

1. waypoint 사이 swept collision 샘플링 해상도 검증
2. Frenet inflated interval과 exact Cartesian polygon collision의 false positive/negative 측정
3. wall corner projection의 branch 선택 stress test
4. high `|kappa d|`에서 AABB projection error 측정
5. steering-rate diagnostic violation이 실제 actuator saturation과 일치하는지 검증
