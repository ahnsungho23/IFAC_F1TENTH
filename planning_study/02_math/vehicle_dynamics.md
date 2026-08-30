# 차량 운동학·동역학 관점의 현재 플래너

## 1. 분류

현재 로컬 플래너는 다음 세 표현 중 가운데에 가깝다.

```text
순수 기하 path planner
        < 현재: dynamics-aware geometric path + velocity planner >
완전 kinodynamic trajectory optimizer
```

이유는 경로 후보의 독립변수가 `d(s)`이고 시간 상태를 적분하지 않지만, 최종 curvature와 speed profile에 차량 제약을 적용하기 때문이다.

## 2. 반영되는 차량 정보

### 차체 footprint

- length 0.56 m
- half width 0.15 m
- 네 모서리를 회전시켜 track bound와 비교

근거: [`local_planning.yaml:24`](../../src/local_planning/config/local_planning.yaml#L24), [`raceline_spline_planner.cpp:2192`](../../src/local_planning/src/raceline_spline_planner.cpp#L2192).

### 조향 geometry

```text
|kappa| <= tan(delta_max_side)/L
```

를 hard gate로 사용한다: [`raceline_spline_planner.cpp:382`](../../src/local_planning/src/raceline_spline_planner.cpp#L382).

### 횡가속

```text
v^2 |kappa| <= a_lat,max(v)
```

를 speed cap으로 사용한다. `a_lat,max`는 속도에 따라 7.6, 7.0, 6.5 m/s²로 내려간다: [`local_planning.yaml:210`](../../src/local_planning/config/local_planning.yaml#L210).

### 종가속/감속

거리축 kinematic relation

```text
v_next^2 = v_now^2 + 2 a Delta s
```

을 사용한다.

- forward pass: ego 실측속도에서 accel table로 도달 가능한 상한
- backward pass: 미래의 낮은 cap에 decel table로 도달 가능한 상한

구현은 [`raceline_spline_planner.cpp:2590`](../../src/local_planning/src/raceline_spline_planner.cpp#L2590).

### 응답지연

confirmed obstacle speed cap 전환에 고정 0.15 s 응답지연을 둔다.

```text
required_distance = v_ego * delay + (v_ego^2-v_target^2)/(2a)
```

근거: [`raceline_spline_planner.cpp:2116`](../../src/local_planning/src/raceline_spline_planner.cpp#L2116), [`local_planning.yaml:253`](../../src/local_planning/config/local_planning.yaml#L253).

## 3. controller와 공유하는 steering model

planner diagnostic은 controller와 같은 형태를 모델링한다.

```text
delta = kappa (L + K_us v^2)
```

좌/우 understeer gradient가 다르다: [`raceline_spline_planner.cpp:396`](../../src/local_planning/src/raceline_spline_planner.cpp#L396).

controller에서는 L1이

```text
a_lat,L1 = 2 v^2 sin(eta) / L1_distance
```

를 만들고, path curvature FF와 결합해 bicycle inverse model로 steering을 만든다: [`control_map_node.cpp:1566`](../../src/f1tenth_control/control_code/control_map_node.cpp#L1566), [`control_map_node.cpp:1604`](../../src/f1tenth_control/control_code/control_map_node.cpp#L1604).

## 4. hard constraint와 diagnostic의 구분

| 항목 | planner 후보 hard reject | speed shaping | publish diagnostic/controller |
|---|---:|---:|---:|
| track/footprint | O | - | - |
| obstacle collision | O | - | - |
| max curvature | O | O(간접) | controller clamp |
| spatial curvature rate | O | - | - |
| lateral acceleration | 직접 reject 아님 | O | O |
| longitudinal accel/decel | 직접 reject 아님 | O | O |
| steering rate rad/s | X | X | planner diagnostic + controller limiter |
| tire friction circle | X | X | 일부 lateral clamp만 |

`inspectVelocityFeasibility()`는 최종 outgoing path의 위반을 보고하지만 후보를 되돌려 기각하지는 않는다: [`raceline_spline_planner.cpp:1587`](../../src/local_planning/src/raceline_spline_planner.cpp#L1587), [`local_planner_node.cpp:4306`](../../src/local_planning/src/local_planner_node.cpp#L4306).

## 5. 왜 완전 kinodynamic가 아닌가

현재 후보는 `(x,y,psi,kappa,v,ax)` waypoint를 내지만 다음을 직접 최적화하거나 적분하지 않는다.

- `x_dot, y_dot, psi_dot, v_dot, delta_dot` 상태방정식 rollout
- absolute time stamp가 있는 trajectory
- steering actuator lag와 saturation을 포함한 forward simulation
- combined longitudinal/lateral tire force
- slip angle, yaw dynamics, load transfer
- tracking controller closed-loop error propagation

따라서 waypoint field에 `v,ax`가 있다는 이유만으로 kinodynamic trajectory optimization이라 부르면 과장이다.

## 6. 현재 모델의 중요한 장점

- geometry 후보가 선택되기 전에 curvature/footprint hard validation을 한다.
- speed cap이 최종 shifted path curvature를 사용한다.
- acceleration/deceleration profile이 ego 실측속도에서 시작한다.
- planner와 controller의 wheelbase, steering limits, understeer gradient를 동기화하려는 contract가 있다: [`local_planning.yaml:591`](../../src/local_planning/config/local_planning.yaml#L591).
- controller는 실제 steering rate와 좌/우 각 clamp를 마지막에 적용한다: [`control_map_node.cpp:1662`](../../src/f1tenth_control/control_code/control_map_node.cpp#L1662).

## 7. 연구/검증에서 필요한 실측

- steering command→wheel angle transfer와 dead time
- speed별 최대 steering rate
- `K_us`의 속도·횡가속 의존성
- acceleration/braking 중 available lateral acceleration
- 타이어/노면 friction 변화
- planner path 수신→drive command→vehicle response latency
- path tracking error covariance as function of `v,kappa,dot{kappa}`

이 데이터가 없으면 더 복잡한 모델을 넣어도 parameter-identification error가 성능을 지배할 수 있다.
