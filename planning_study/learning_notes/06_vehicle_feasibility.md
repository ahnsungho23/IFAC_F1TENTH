# 차량 실현 가능성

## 1. 경로가 매끄러우면 차량도 따라갈 수 있는가

[GENERAL THEORY] 아니다. 기하학적으로 충돌 없는 path라도 조향각, 조향 속도, 타이어 횡력, 가감속, actuator delay 때문에 실제 차량은 못 따라갈 수 있다. 따라서 “path feasibility”는 최소한 geometry와 speed profile을 함께 봐야 한다.

## 2. kinematic bicycle의 기본식

변수를 정의한다.

- `L`: 앞뒤 차축 사이 wheelbase
- `δ`: 등가 앞바퀴 조향각
- `R`: 회전 반지름
- `κ=1/R`: 경로 곡률
- `v`: 종방향 속도

[GENERAL THEORY] no-slip kinematic bicycle에서

`R=L/tanδ`

이므로

`κ=tanδ/L`, `δ=atan(Lκ)`

이다. 작은 조향각에서는 `tanδ≈δ`라서 `δ≈Lκ`로 선형화할 수 있다.

## 3. 횡가속도

[GENERAL THEORY] 원운동에서 centripetal acceleration은

`a_y=v²/R=v²κ`

이다. 부호까지 쓰면 좌회전 양수, 우회전 음수로 둘 수 있고, grip limit에는 보통 절댓값 `|a_y|`를 쓴다.

속도 cap은 이상적인 constant limit `a_y,max`를 가정하면

`v_max=sqrt(a_y,max/|κ|)`

이다. 곡률이 4배면 허용 속도는 절반이 된다.

## 4. 현재 controller의 모델

[CURRENT IMPLEMENTATION] control node는 단순 `atan(Lκ)`만 쓰지 않는다. 정상상태 bicycle inverse model을 선형 형태로 사용한다.

`δ = Lκ + K_us a_y = κ(L+K_us v²)`

여기서 `K_us`는 understeer gradient다. 실제 control helper는 lateral acceleration command로

`δ=a_y(L/v²+K_us)`

를 계산하며 좌/우 `K_us`를 구분한다([`control_map_node.cpp:1007`](../../src/f1tenth_control/control_code/control_map_node.cpp#L1007)). 경로 curvature feedforward에는 부호 있는 smoothed curvature를 사용한다([`control_map_node.cpp:1022`](../../src/f1tenth_control/control_code/control_map_node.cpp#L1022)).

[GENERAL THEORY] `K_us>0`이면 속도가 높을수록 같은 curvature를 만들기 위해 더 큰 steering이 필요하다. 이는 순수 kinematic no-slip 모델과 실제 tire/load behavior의 차이를 한 개의 정상상태 보정항으로 요약한 것이다.

## 5. 현재 planner가 구현한 것

[CURRENT IMPLEMENTATION] 현재 local planner는 여러 층에서 feasibility를 반영한다.

1. 방향별 최대 curvature: `min(legacy κ limit, tan(δ_max,side)/L)`로 hard reject한다([`raceline_spline_planner.cpp:383`](../../src/local_planning/src/raceline_spline_planner.cpp#L383)).
2. 횡가속 속도 cap: speed-dependent lateral acceleration table에 대해 `v²|κ|≤limit(v)`가 될 때까지 속도를 이분 탐색으로 낮춘다([`raceline_spline_planner.cpp:416`](../../src/local_planning/src/raceline_spline_planner.cpp#L416)).
3. 접근 제동 ramp: measured ego speed, response delay, available distance, accel/decel table로 장애물 span 전에 speed cap에 도달하도록 낮춘다([`raceline_spline_planner.cpp:2486`](../../src/local_planning/src/raceline_spline_planner.cpp#L2486)).
4. longitudinal forward/backward pass: 인접 waypoint의 가속·감속 요구가 표를 넘지 않도록 속도를 낮춘다([`raceline_spline_planner.cpp:2590`](../../src/local_planning/src/raceline_spline_planner.cpp#L2590)).
5. `measureCandidate()`는 현재 ego speed에서 각 낮은 waypoint speed까지 필요한 braking distance deficit을 계산해 ranking key에 넣는다([`raceline_spline_planner.cpp:2116`](../../src/local_planning/src/raceline_spline_planner.cpp#L2116)).

## 6. 구현되어 있지만 hard reject가 아닌 것

[CURRENT IMPLEMENTATION] `inspectVelocityFeasibility()`는 acceleration, deceleration, modeled steering rate violation을 계산한다([`raceline_spline_planner.cpp:1587`](../../src/local_planning/src/raceline_spline_planner.cpp#L1587)). 그러나 현재 P3 `validateCandidate()`의 hard reject 목록에는 temporal steering-rate report가 들어가지 않는다. hard validator가 직접 보는 것은 spatial `|Δκ|/Δs`, 방향별 curvature 등이다.

[CURRENT IMPLEMENTATION] planner의 `modeledControlSteeringRad()`는 controller 계약을 따라 `κ(L+K_us v²)`를 계산하고 조향각 limit으로 clamp한다([`raceline_spline_planner.cpp:396`](../../src/local_planning/src/raceline_spline_planner.cpp#L396)). 이 모델은 diagnostic의 temporal rate 추정에 쓰이며 candidate ranking의 독립 cost가 아니다.

## 7. 현재 구현하지 않은 것

[CURRENT IMPLEMENTATION] source에서 다음을 production P3 hard validation으로 확인할 수 없다.

- full nonlinear tire model 또는 friction ellipse를 시간 적분한 rollout
- steering actuator lag/rate limiter를 포함한 closed-loop path tracking simulation
- predicted localization/perception covariance와 controller error의 joint chance constraint
- dynamic opponent와 ego의 time-indexed collision checking
- sideslip state `β`를 포함한 dynamic bicycle reachability

[INFERENCE] 따라서 현재 planner는 단순 geometry-only보다 강하지만, complete kinodynamic trajectory validator라고 부르면 과장이다.

## 8. spatial rate와 temporal demand

[GENERAL THEORY] 작은 각 근사에서 `δ≈Lκ`라면

`dδ/dt ≈ L dκ/dt = L v dκ/ds`

이다. understeer 항을 포함하면 속도 변화까지 미분에 들어간다.

`δ=κ(L+K_us v²)`

이므로 constant `K_us`에서

`dδ/dt=(L+K_us v²)dκ/dt + 2K_us κv dv/dt`

이다. 같은 `dκ/ds`도 고속 또는 급가속 중에는 더 큰 steering-rate demand를 만든다.

## 9. 숫자 예제

[GENERAL THEORY] `L=0.33 m`, `κ=0.8 1/m`, `v=4 m/s`, `K_us=0.014 rad/(m/s²)`라 하자.

- kinematic exact: `δ=atan(0.33×0.8)=atan(0.264)≈0.258 rad`
- controller linear understeer model: `δ=0.8(0.33+0.014×16)=0.4432 rad`
- 횡가속도: `a_y=16×0.8=12.8 m/s²`

두 steering 값의 큰 차이는 “어느 식이 항상 참인가”의 문제가 아니다. 첫 식은 no-slip kinematic geometry, 둘째는 현재 controller가 사용하는 정상상태 understeer compensation model이다. 계산된 `0.4432 rad`는 아래의 현재 left limit `0.410 rad`보다 크므로, 이 속도와 곡률 조합을 그대로 요구하면 controller 쪽에서는 clamp가 필요하다. 다만 planner는 횡가속 speed cap으로 실제 waypoint speed를 먼저 낮출 수 있으므로, 이 손계산만으로 최종 출력이 불가능하다고 단정해서는 안 된다. 고하중에서 실제 차량 식별이 중요하다.

[CURRENT IMPLEMENTATION] 운영 YAML의 planner steering 계약은 wheelbase `0.33`, left limit `0.410`, right limit `0.361`, 좌/우 `K_us=0.014/0.019`를 적고 있다([`local_planning.yaml:591`](../../src/local_planning/config/local_planning.yaml#L591)). 그러나 control real launch의 right default는 현재 `0.410`이어서 baseline contract test의 기존 mismatch다. 어느 값이 실제 대회 runtime에 적용됐는지는 별도 확인이 필요하다.

## 10. 자주 혼동하는 개념

- `κ=tanδ/L`과 `δ=Lκ+K_us a_y`는 같은 수준의 모델이 아니다.
- 횡가속 cap은 path를 기각할 수도 있지만 현재 구현에서는 주로 waypoint speed를 낮춘다.
- spatial curvature-rate gate가 temporal actuator-rate guarantee는 아니다.
- speed profile이 존재한다고 closed-loop tracking feasibility가 증명된 것은 아니다.
- planner YAML과 controller launch의 숫자 일치 여부는 실제 runtime-effective value와도 구분해야 한다.

[OPEN QUESTION] 실제 vehicle identification으로 좌우 `K_us`, steering reach/rate, delay, lateral acceleration envelope를 같은 주행에서 공동 추정했을 때 현재 planner cap이 얼마나 보수적 또는 낙관적인지는 아직 자동 증명되지 않는다.

## 반드시 설명할 수 있어야 하는 질문

1. `κ=tanδ/L`은 어떤 가정에서 나오는가?
2. `a_y=v²κ` 때문에 고속에서 같은 path가 더 어려워지는 이유는 무엇인가?
3. current controller의 understeer 식과 kinematic exact 식은 어떻게 다른가?
4. 현재 planner가 speed feasibility를 반영하는 다섯 계층은 무엇인가?
5. spatial curvature rate가 steering actuator guarantee가 아닌 이유는 무엇인가?
6. 현재 source에서 확인되지 않는 dynamic feasibility 항목은 무엇인가?
