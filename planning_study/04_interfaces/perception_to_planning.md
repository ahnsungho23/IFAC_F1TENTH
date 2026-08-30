# Perception → Planning 인터페이스

## 1. 계약의 핵심

현재 정적 회피의 authoritative contract는

```text
obstacle_detector /confirmed_static_obs
    -> local_planner_node obstacles_topic
```

이다. 둘 다 `f110_msgs/msg/ObstacleArray`를 사용한다.

- detector publisher: [`obstacle_detector_node.cpp:94`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L94)
- planner subscriber: [`local_planner_node.cpp:819`](../../src/local_planning/src/local_planner_node.cpp#L819)
- operational topic: [`local_planning.yaml:814`](../../src/local_planning/config/local_planning.yaml#L814)

## 2. `ObstacleArray`

메시지는 header와 obstacle vector로 단순하다: [`ObstacleArray.msg`](../../f110_msgs/msg/ObstacleArray.msg).

`header.frame_id`는 planner에서 비어 있거나 `map`이어야 한다. 다른 frame이면 전체 배열을 무시하고 마지막 valid snapshot을 유지한다: [`local_planner_node.cpp:959`](../../src/local_planning/src/local_planner_node.cpp#L959).

`header.stamp`는 source lineage와 out-of-order/restart 판정에 쓰인다. non-lockstep freshness는 source stamp가 아니라 실제 receipt time으로 계산한다. queue delay를 “방금 받은 새 데이터”로 오인하지 않기 위해서다: [`local_planner_node.cpp:1029`](../../src/local_planning/src/local_planner_node.cpp#L1029).

## 3. `Obstacle` 필드 전체

정의는 [`Obstacle.msg`](../../f110_msgs/msg/Obstacle.msg)에 있다.

### Cartesian geometry

| 필드 | 의미 | planner 회피 기하 사용 |
|---|---|---:|
| `has_cartesian` | Cartesian 값의 유효성 | lifecycle audit/guard 관찰에 보존 |
| `x_center,y_center` | map-frame AABB 중심 | 직접 후보 생성 X |
| `radius` | AABB를 감싸는 원 반경 | 직접 X |
| `x_min,x_max,y_min,y_max` | map-frame AABB | 직접 Frenet collision X |
| `x_var,y_var` | Cartesian 위치 분산 근사 | 직접 X |

### Frenet geometry/kinematics

| 필드 | 의미 | planner 사용 |
|---|---|---:|
| `s_start,s_end` | closed-loop longitudinal envelope | O |
| `s_center` | obstacle center | O |
| `d_right,d_left` | 오른쪽/왼쪽 lateral face | O |
| `d_center` | center offset | validation/context 보조 |
| `size` | envelope diagonal/fallback span | O, degenerate span fallback |
| `vs,vd` | tracked Frenet velocity | 정적 local planner 후보 X |
| `s_var,d_var` | position variance | guard code O, 현재 scale 0 |
| `vs_var,vd_var` | velocity variance | 정적 local planner 후보 X |
| `s_vs_cov,d_vd_cov` | position-velocity covariance | 정적 local planner 후보 X |

### semantics

| 필드 | 의미 | planner 사용 |
|---|---|---:|
| `id` | track/physical identity | stabilization, commitment, lifecycle |
| `is_static` | motion class | confirmed topic이 이미 static layer; planner 자체 재검사 없음 |
| `is_visible` | current measurement visibility | planner 후보 직접 gate 없음 |
| `is_interfering` | dynamic opponent corridor interference | local planner X, FSM CRUISE 관련 |
| `is_actually_a_gap` | legacy gap semantic | current static P3 주요 경로에서 확인되지 않음 |

중요: 메시지에 명시적인 `confidence` scalar나 class probability는 없다. uncertainty는 covariance와 tracker state/confirmed layer로 표현된다.

## 4. detector가 geometry를 만드는 순서

### 4.1 scan → map points

LaserScan을 `SensorDataQoS`로 받고 TF `map <- scan`을 구한다: [`obstacle_detector_node.cpp:81`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L81), [`obstacle_detector_node.cpp:1564`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1564).

### 4.2 Layer 1 filtering

- adaptive-breakpoint clustering
- pre-tracking fragment merge
- map wall distance filter
- maximum object size
- viewing window
- track boundary corridor

main scan pipeline은 [`obstacle_detector_node.cpp:1613`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1613)에 있다.

### 4.3 AABB → Frenet

각 cluster의 map AABB를 만들고 `projectCartesianAabb()`를 호출한다: [`obstacle_detector_node.cpp:1621`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1621).

projector는

1. AABB center를 CLCS에 투영
2. center tangent/normal에 four corners를 투영해 longitudinal/lateral extent 생성
3. center가 속한 `s` branch 주변에서 reference-to-AABB exact distance 계산
4. line-facing lateral face를 그 거리로 교정

한다: [`aabb_frenet_projector.cpp:198`](../../src/obstacle_detector/src/aabb_frenet_projector.cpp#L198).

### 4.4 tracking과 uncertainty

Detection에는 range, sparse point count, yaw rate에 따른 measurement variance scale이 들어간다: [`obstacle_detector_node.cpp:1693`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1693).

tracker는 constant-velocity Kalman 형태의 `s,vs,d,vd` 상태와 covariance를 관리한다. layer merge 시 member velocity는 size-weighted mean, position variance는 보수적으로 max를 사용한다: [`obstacle_detector_node.cpp:1173`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1173).

visible/static AABB union이 있으면 merge 후 다시 한번 projection하여 Cartesian marker와 Frenet envelope 계약을 맞춘다: [`obstacle_detector_node.cpp:1274`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1274).

## 5. confirmed static의 의미

tracker에서 `TrackStatus::Confirmed`이고 dynamic이 아니며 envelope stability 조건을 만족한 track이 static layer에 들어간다. 그중 `MotionStatus::Static`인 것만 confirmed static layer에 들어간다: [`obstacle_detector_node.cpp:1756`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1756).

세 output은 매 scan, empty 포함 발행된다.

- `/static_obs`: publishable static, provisional 성격 포함
- `/confirmed_static_obs`: motion static으로 확정된 객체
- `/opp_obs`: nearest-ahead confirmed dynamic 한 대

근거: [`obstacle_detector_node.cpp:1742`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1742), [`obstacle_detector_node.cpp:1792`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1792).

## 6. planner 수신 검증

`acceptObstacles()`는 다음을 확인한다: [`local_planner_node.cpp:959`](../../src/local_planning/src/local_planner_node.cpp#L959).

- frame
- 각 obstacle의 finite/valid Frenet bounds
- source stamp regression
- 명시적 empty와 all-invalid non-empty의 구분

all-invalid non-empty 배열은 degraded perception이다. accepted empty로 바꾸지 않고 마지막 snapshot을 유지한다. 반대로 producer가 실제로 empty vector를 보내면 track clear 관측으로 받아들인다.

## 7. planner가 실제로 사용하는 geometry

`expandVisibleObstacles()`의 원본은 `s_start,s_end,s_center,d_right,d_left,size`다: [`raceline_spline_planner.cpp:729`](../../src/local_planning/src/raceline_spline_planner.cpp#L729).

즉 planner는 detector가 이미 해결한 다음 문제를 다시 하지 않는다.

- raw scan point clustering
- map TF
- Cartesian AABB 생성
- AABB의 Frenet projection
- static/dynamic classification

planner는 detector-owned Frenet bounds를 그대로 받아 자기 vehicle/margin을 더한다.

## 8. `/static_obs` raw slowdown 계약

`/static_obs`는 separate subscription이고 `id,s_start,s_end,d_left,d_right`만 `RawHintObstacle`로 복사한다: [`local_planner_node.cpp:1083`](../../src/local_planning/src/local_planner_node.cpp#L1083).

용도는 confirmed 승격 전 다음과 같은 보수 감속이다.

- line-blocking raw target 중 가장 가까운 것 선택
- trigger 12 m 안에서 cap 2.8 m/s
- response delay/distance margin을 포함한 speed overlay
- confirmed commitment가 같은 ID를 책임지면 중복 cap 생략 가능

raw geometry로 avoidance path를 생성하거나 commitment하지는 않는다.

## 9. `/opp_obs`와 정적 local planning의 경계

`local_planner_node`는 `/opp_obs`를 구독하지 않는다. `state_machine_node`가 non-static opponent의 position, velocity, covariance를 읽어 확률적 interference를 평가하고 CRUISE 상태를 정한다: [`state_machine_node.cpp:481`](../../src/state_machine/src/state_machine_node.cpp#L481), [`state_machine_node.cpp:626`](../../src/state_machine/src/state_machine_node.cpp#L626).

따라서 현재 정적 P3 planner에 dynamic obstacle prediction 기능이 있다고 해석하면 안 된다.

## 10. uncertainty가 전달되지만 현재 쓰이지 않는 부분

`Obstacle.msg`는 `s_var,d_var,vs_var,vd_var,s_vs_cov,d_vd_cov`를 제공한다. 주석에는 constant-velocity prediction horizon에서 variance를 전파할 수 있는 식까지 있다: [`Obstacle.msg:29`](../../f110_msgs/msg/Obstacle.msg#L29).

그러나 정적 local planner의 현재 운영은

- velocity/cross-covariance를 path collision에 사용하지 않음
- `uncertainty_sigma_scale=0`
- lateral inflation min/max=0

이다. covariance-aware code path는 존재하지만 실효 margin이 0이다: [`obstacle_guard.cpp:64`](../../src/local_planning/src/obstacle_guard.cpp#L64), [`local_planning.yaml:703`](../../src/local_planning/config/local_planning.yaml#L703).

이 간극은 uncertainty-aware planning 연구에 유리한 baseline이다. 메시지와 tracker는 이미 있으나 calibration과 decision policy가 비어 있다.

## 11. 좌표계/시간 계약의 위험 지점

- detector와 ego Frenet converter가 같은 global geometry로 CLCS를 만들어야 한다.
- planner는 CLCS를 직접 쓰지 않고 Wpnt의 `s,psi,d_left,d_right`를 쓴다.
- source timestamp와 receipt timestamp의 의미가 다르다.
- empty array는 clear evidence일 수 있으므로 publisher가 처리 실패를 empty로 위장하면 위험하다.
- same ID는 lifecycle의 핵심이다. geometry가 같아도 ID 재생성은 commitment를 흔든다.
- stale threshold 0.75 s는 장애물 속도/거리와 무관한 고정 시간이다.

### 요청된 여섯 질문에 대한 직접 답

| 질문 | 현재 코드의 답 |
|---|---|
| 1. obstacle position을 deterministic point로 쓰는가? | **점 하나는 아니다.** detector의 deterministic Frenet AABB interval (`s_start/end`, `d_right/left`)을 사용한다. 다만 probability distribution/chance constraint 없이 하나의 확정 envelope로 판정한다는 의미에서는 deterministic이다. |
| 2. perception uncertainty를 쓰는가? | message/tracker와 `ObstacleGuard` code path에는 covariance가 있다. 그러나 기본 운영 `sigma_scale=0`, lateral min/max=0이므로 covariance 기반 planning margin은 실질적으로 꺼져 있다. 다중 관측 union/commitment guard는 쓰지만 calibrated probabilistic planning은 아니다. |
| 3. obstacle velocity를 쓰는가? | confirmed static P3 geometry와 raw slowdown은 `vs,vd`로 미래 위치를 예측하지 않는다. `/opp_obs` velocity/covariance는 local planner가 아니라 FSM의 probabilistic interference/CRUISE 경로에서 사용한다. |
| 4. timestamp/data age를 쓰는가? | `header.stamp`는 lineage, regression/restart 판정에 쓰고 freshness는 기본적으로 planner receipt age로 0.75 s를 검사한다. obstacle을 source stamp만큼 forward-predict하지는 않는다. |
| 5. sensor-to-planner latency를 고려하는가? | end-to-end로는 아니다. speed shaping에 고정 0.15 s response delay가 있으나 scan→detection→planning queue 지연을 측정해 geometry/state를 실행시점으로 옮기지 않는다. |
| 6. 오래된 obstacle은 어떻게 처리하는가? | stale을 “장애물 없음”으로 바꾸지 않는다. 마지막 accepted snapshot과 lifecycle authority를 보존하고 planning-ready/fallback/safe-stop 정책으로 간다. 명시적 accepted empty만 clear evidence다. |

## 12. 인터페이스 검증 체크리스트

1. `/confirmed_static_obs.header.frame_id == map`인가?
2. `d_right <= d_left` 부호 규약이 항상 지켜지는가?
3. seam obstacle의 `s_start/s_end` 짧은 span이 맞는가?
4. Cartesian AABB marker와 Frenet envelope가 같은 물체를 나타내는가?
5. all-invalid frame과 true empty frame을 producer가 구분하는가?
6. tracker ID가 occlusion/merge 뒤에도 유지되는가?
7. covariance가 실제 error를 calibration하는가?
8. detector publish stamp, planner receipt, plan publish, drive command latency가 기록되는가?
9. global reference 변경 시 detector/planner epoch reset이 같이 일어나는가?
10. `/static_obs`와 `/confirmed_static_obs`의 승격 지연이 실제 braking budget 안인가?
