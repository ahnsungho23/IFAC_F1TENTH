# 연구 방향과 학습 로드맵

## 1. 현재 baseline을 연구 언어로 표현하기

현재 시스템은 다음 요소를 가진다.

- static obstacle perception with tracked Frenet AABB and covariance
- reference-line-locked `d(s)` path family
- analytic corridor/root candidate proposal
- C2 quintic-Hermite profile
- exact sampled hard validation
- curvature/acceleration-aware velocity shaping
- lexicographic candidate ranking
- immutable maneuver lifecycle and safe-stop fallback
- L1+bicycle inverse-model controller

즉 연구 출발점은 “아무 제약 없는 naive spline”이 아니다. 새 연구는 이미 있는 geometry, speed shaping, lifecycle 중 무엇을 대체하고 무엇을 보존하는지 명확히 해야 한다.

## 2. 방향 A — Dynamics-aware local planning

### 현재 gap

planner는 `v^2 kappa`, accel/decel table, steering geometry, fixed delay를 반영한다. 그러나 steering rate는 hard constraint가 아니고 combined tire force와 closed-loop vehicle state rollout이 없다.

### 가능한 연구 질문

- spatial curvature-rate limit 대신 속도를 포함한 `delta_dot` hard constraint가 접촉률을 줄이는가?
- friction ellipse

```text
(a_x/a_x,max)^2 + (a_y/a_y,max)^2 <= 1
```

를 적용하면 braking-in-corner failure를 얼마나 줄이는가?
- 후보마다 kinematic/dynamic bicycle을 짧게 rollout하고 L1 controller를 in-the-loop로 넣으면 sampled geometry validator보다 예측력이 좋아지는가?
- planner의 `K_us` 모델과 실제 controller/vehicle identification을 어떻게 동기화할 것인가?

### 최소 구현 단위

기존 24개 geometric candidate는 유지하고, 최종 rank 전에 time parameterization + low-order vehicle rollout validator를 추가하는 것이 가장 작은 연구 변경이다.

### 필요한 데이터

- steering command/actual angle
- wheel speed, yaw rate, lateral acceleration
- longitudinal acceleration
- surface/friction condition
- command-to-response latency
- path tracking error

### 평가 지표

- hard dynamics rejection precision/recall against real saturation
- minimum actual clearance
- steering-rate saturation duration
- friction budget exceedance
- lap time and stop time
- compute time p50/p99

### 종합 평가

| 요구 항목 | 평가 |
|---|---|
| 해결하는 현재 한계 | spatial curvature/rate와 분리된 speed shaping만으로는 steering actuator lag, combined braking/cornering, closed-loop tracking failure를 보장하지 못하는 문제 |
| 기존 코드 변경 지점 | `P3ShadowEvaluator`가 만든 hard-valid 후보 뒤, `betterCandidateRank()` 전 단계에 time parameterization과 dynamics validator를 삽입; `inspectVelocityFeasibility()`는 사후 diagnostic에서 oracle 비교 대상으로 사용 |
| 필요한 추가 state/data | time-indexed ego `v,a,yaw_rate`, steering command/actual, actuator delay, tire/friction envelope, controller state |
| 예상 장점 | 실제 tracking saturation과 연결된 candidate rejection, safety margin을 무작정 키우지 않고 위험 형상 제거 가능 |
| 예상 단점 | model mismatch, identification 비용, candidate당 rollout 계산량, 보수 모델에 의한 후보 전멸 |
| simulation 검증 | 가능. 동일 24 candidates에 kinematic/dynamic bicycle+현 controller를 적용하고 geometry-only baseline과 충돌·추종오차·계산시간 비교 |
| 실차 검증 | 가능하지만 단계적이어야 한다. 먼저 shadow/offline prediction precision을 검증하고, 저속→고속 순으로 intervention을 활성화해야 한다. |
| 논문 contribution 조건 | 단순 constraint 추가를 넘어 실제 saturation ground truth에 대한 예측 정확도, geometry baseline 대비 safety/performance Pareto 개선, model mismatch/compute-time 분석과 재현 가능한 ablation이 필요 |

## 3. 방향 B — Uncertainty-aware planning

### 현재 gap

perception은 covariance를 제공하고 planner에도 uncertainty guard code가 있지만 operational `sigma_scale=0`, lateral max inflation=0이다. 즉 구조는 있으나 calibrated risk decision은 꺼져 있다.

### 가능한 연구 질문

- obstacle face error를 Gaussian/empirical quantile로 calibration할 수 있는가?
- fixed safety margin 대신

```text
margin(s) = body + q_alpha * sqrt(sigma_perception^2
                                  + sigma_localization^2
                                  + sigma_tracking^2)
```

를 쓰면 false stop을 줄이면서 target collision probability를 유지하는가?
- path point별 독립 chance constraint와 maneuver-level joint risk budget의 차이는 무엇인가?
- same-ID temporal correlation과 AABB face correlation을 어떻게 처리할 것인가?

### 주의

covariance가 존재한다는 이유만으로 Gaussian calibration이 성립하지 않는다. tracker covariance와 실제 face error의 reliability diagram을 먼저 만들어야 한다.

### 평가 지표

- empirical coverage: `P(error <= q sigma)`
- expected/quantile minimum clearance
- false-positive avoidance/stop duration
- missed collision rate
- risk calibration error
- narrow-gap completion rate

### 종합 평가

| 요구 항목 | 평가 |
|---|---|
| 해결하는 현재 한계 | fixed 0.08 m margin과 꺼진 sigma inflation이 거리·시야·면별 오차 차이를 표현하지 못하는 문제 |
| 기존 코드 변경 지점 | `ObstacleGuard`의 inflation policy와 `expandVisibleObstacles()`가 소비하는 face bounds; hard validator/lifecycle 구조는 유지 가능 |
| 필요한 추가 state/data | obstacle face별 error distribution, localization covariance와 bias, covariance age/correlation, target risk `alpha`, ground-truth obstacle/map pose |
| 예상 장점 | 같은 empirical risk에서 좁은 gap false stop 감소 또는 같은 성능에서 collision risk 감소; 이미 존재하는 message/guard 구조 재사용 |
| 예상 단점 | covariance miscalibration 시 거짓 안전성, Gaussian/independence 가정 오류, correlated samples의 joint risk 관리 난점 |
| simulation 검증 | 가능. noise/dropout/pose bias를 통제해 coverage, collision, stop time을 대량 반복 비교할 수 있다. 다만 simulator noise calibration이 실차를 대표해야 한다. |
| 실차 검증 | 가능. 정밀 ground truth 또는 반복 측정으로 face/localization error를 먼저 calibration하고 shadow margin을 거쳐 활성화해야 한다. |
| 논문 contribution 조건 | `m=m0+k sigma`만 구현해서는 약하다. face-wise calibrated uncertainty, maneuver-level risk allocation, fixed-margin 대비 risk-performance Pareto와 distribution shift robustness가 필요 |

## 4. 방향 C — Latency-aware planning

### 현재 gap

현재는 confirmed/raw speed shaping에 각각 고정 0.15 s response delay가 있고 freshness timeout은 0.75 s다. 그러나 detection stamp에서 actuator response까지 전체 latency를 state/obstacle geometry에 forward-predict하지 않는다.

### 가능한 연구 질문

- `scan stamp -> detector publish -> planner receipt -> avoid publish -> local waypoint -> drive -> vehicle response` 각 지연을 분해할 수 있는가?
- ego state를 plan execution time으로 forward propagate하면 braking deficit와 collision validation이 얼마나 달라지는가?
- static obstacle에서도 ego-only delay compensation이 candidate viability를 개선하는가?
- jitter-aware percentile latency와 fixed mean latency 중 어떤 것이 안전/성능 trade-off가 좋은가?

### 모델 예

```text
s_ego(t+tau) ~= s + v_s tau + 0.5 a_s tau^2
d_ego(t+tau) ~= d + v_d tau
Sigma(t+tau) = F Sigma F^T + Q
```

static obstacle은 움직이지 않아도 ego가 `v*tau`만큼 가까워진 상태에서 계획해야 한다.

### 평가 지표

- latency distribution p50/p95/p99
- first-detection distance vs first-effective-steering/braking distance
- braking deficit
- late-plan safe-stop rate
- deadline miss and compute jitter

### 종합 평가

| 요구 항목 | 평가 |
|---|---|
| 해결하는 현재 한계 | 고정 0.15 s response distance와 0.75 s freshness만으로 sensor→actuator 지연 분포와 jitter를 geometry/ego state에 반영하지 못하는 문제 |
| 기존 코드 변경 지점 | obstacle ingress/source stamp 기록, planning snapshot의 execution-time ego propagation, braking deficit와 validation query state; controller/drive 측 timestamp 관측 추가 |
| 필요한 추가 state/data | scan source stamp, detector/planner/FSM/controller receipt·publish time, synchronized clock, ego acceleration, command→actual response time |
| 예상 장점 | 늦은 plan의 braking budget을 명시화하고 동일 static obstacle에서도 실제 실행 시점의 ego pose로 검증 가능 |
| 예상 단점 | clock/QoS 측정 오류, tail latency 선택에 따른 보수성, delay compensation과 state-estimation error의 결합 |
| simulation 검증 | 가능. 각 pipeline stage에 통제된 delay/jitter를 주입하고 first-effective-action distance와 실패율을 비교할 수 있다. |
| 실차 검증 | 매우 가능성이 높다. 우선 observer-only timestamp instrumentation으로 분포를 얻고, forward prediction은 shadow comparison 뒤 활성화한다. |
| 논문 contribution 조건 | 단순 `v*tau` 이동보다 stage-wise latency identification, uncertainty/jitter-aware prediction, dynamics validator와의 결합, 다양한 속도/부하에서의 causal ablation이 필요 |

## 5. 가장 우선할 두 연구 주제

### 1순위: latency-aware, closed-loop dynamics feasibility

추천 이유:

- 현재 시스템은 geometry와 속도 profile은 이미 강하지만, “그 명령이 언제 실행되고 실제 actuator/타이어가 따라갈 수 있는가”의 gap이 남아 있다.
- fixed 0.15 s와 passive steering-rate diagnostics가 이미 baseline/ablation point를 제공한다.
- 기존 24개 후보 위에 validator를 추가할 수 있어 architecture를 전부 버리지 않아도 된다.
- 실차 접촉과 직접 연결되는 steering/braking saturation을 설명할 가능성이 높다.

구체적 첫 논문형 질문:

> End-to-end delay compensation과 controller-in-the-loop dynamic feasibility filtering이 기존 sampled Frenet spline planner 대비 collision clearance를 개선하면서 lap-time 손실을 제한하는가?

### 2순위: calibrated uncertainty-aware obstacle/track margins

추천 이유:

- covariance message, tracker covariance, guard code가 이미 있어 연구 변경이 명확하다.
- 현재 margins와 sigma guard가 실질적으로 꺼져 있어 baseline 대비 효과가 뚜렷하다.
- narrow gap에서 fixed margin의 과보수성과 perception miss의 비보수성을 동시에 다룰 수 있다.

구체적 첫 논문형 질문:

> Calibrated obstacle-face and localization uncertainty를 사용하는 chance-constrained Frenet corridor가 fixed 0.08 m margin보다 같은 empirical collision risk에서 false-stop time을 줄이는가?

## 6. 권장 실험 설계

### 단계 1: frozen baseline

- commit/hash와 runtime params를 고정
- raw `/scan`, pose, global path를 기록
- current planner outputs와 controller commands 기록
- candidate audit를 켠 별도 연구 run 생성

### 단계 2: offline oracle

- high-resolution Cartesian swept polygon collision
- measured actuator/vehicle dynamic envelope
- true obstacle pose 또는 정밀 annotation
- end-to-end timestamp correlation

### 단계 3: ablation

```text
B0 current
B1 latency compensation only
B2 hard steering-rate/dynamics only
B3 B1+B2
B4 uncertainty margin only
B5 uncertainty+latency
```

### 단계 4: 지표

- safety: collision, minimum clearance, constraint violation duration
- performance: lap time, average/quantile speed, stop duration
- robustness: perception dropout, localization jump, latency jitter
- efficiency: planning runtime p50/p95/p99, candidate count/cap saturation
- calibration: predicted risk vs empirical risk

## 7. 문헌을 볼 때 연결할 키워드

- Frenet optimal trajectory / reference-line trajectory planning
- quintic Hermite spline, monotone piecewise quintic interpolation
- chance-constrained motion planning
- tube MPC / robust MPC
- control barrier functions for collision avoidance
- time-elastic band / kinodynamic search
- friction-circle constrained trajectory optimization
- latency compensation and delay-aware MPC
- reachable set / viability kernel
- controller-in-the-loop planning

문헌식을 바로 이 코드에 억지로 맞추지 말고, 현재 hard validator/rank/lifecycle 중 어느 계층을 대체하는지 먼저 표시해야 한다.

## 8. 연구 전 마지막 경고

- safety margin을 낮춰 candidate 수가 늘어나는 것을 알고리즘 개선으로 오해하지 않는다.
- covariance를 쓰기 전에 calibration한다.
- sophisticated optimizer를 넣기 전에 latency와 controller feasibility oracle을 만든다.
- historical bag의 `/avoid_waypoints`를 현재 code behavior의 증거로 쓰지 않는다. raw input을 현재 binary에 replay해야 한다.
- source, YAML, launch override, runtime-effective value, vehicle response를 서로 다른 evidence layer로 기록한다.

# What I should study next

| 우선순위·개념 | 왜 필요한가 | 현재 코드 연결 | 이해 목표 수준 |
|---|---|---|---|
| 1. ROS data authority와 lifecycle | 알고리즘보다 먼저 어떤 topic/node가 경로·정지·복귀 권한을 갖는지 알아야 로그와 장애를 잘못 해석하지 않는다. | [`planner_pipeline.md`](../01_architecture/planner_pipeline.md)의 detector→planner→FSM→controller, `LocalPlannerNode::onPlanningTimer()` | callback 한 번을 입력 stamp부터 `/drive_autonomous`까지 말로 추적하고 stale/empty/hold 차이를 설명 |
| 2. Frenet/CLCS 미분기하 | 모든 candidate와 obstacle bound가 `s,d`에 있으므로 좌표 singularity와 wrap을 모르면 geometry 안전성을 평가할 수 없다. | [`frenet.md`](../02_math/frenet.md), `ClcsFrenetConverter`, `p=r+dn` | `1-kappa d`, velocity transform, closed-loop unwrapping을 손으로 유도하고 급커브 오차를 예측 |
| 3. Quintic Hermite와 C2 continuity | 후보 형상의 핵심 표현이며 “부드럽다”를 derivative 조건으로 설명해야 한다. | [`quintic.md`](../02_math/quintic.md), `p3_analytic_solver.hpp`, `p3_shadow.cpp` | 여섯 경계조건에서 계수를 계산하고 knot 공유가 C2를 만드는 이유를 증명 |
| 4. Corridor/branch candidate search | 후보 cap·좌우 domain·M0/M1가 어떤 feasible set을 탐색하고 무엇을 놓치는지 알아야 개선 방향을 정할 수 있다. | [`candidate_generation.md`](../03_current_planner/candidate_generation.md), `P3ShadowEvaluator::run()` | 실제 obstacle 한 개에 `z0..z4`, connected corridor, 모든 후보 조합과 reject 지점을 그림으로 복원 |
| 5. Collision geometry와 sampling | Frenet interval과 Cartesian rotated footprint가 서로 다른 위험을 검사하며 continuous collision guarantee는 없다. | [`collision_check.md`](../03_current_planner/collision_check.md), `validateCandidate()` | hard/soft/diagnostic을 분류하고 exact swept-polygon oracle와 현 검사 간 false positive/negative를 설계 |
| 6. Lexicographic/Pareto decision | 현재 선택은 weighted cost가 아니므로 weight tuning 사고방식이 오판을 만든다. | [`cost_function.md`](../03_current_planner/cost_function.md), `candidate_rank.hpp` | 후보 두 개의 rank key를 손으로 비교하고 priority 변경이 Pareto 선택에 미치는 영향 설명 |
| 7. Bicycle model, tire limit, controller-in-loop | geometry-valid path와 실제 추종 가능한 trajectory의 차이가 첫 연구 gap이다. | [`vehicle_dynamics.md`](../02_math/vehicle_dynamics.md), `inspectVelocityFeasibility()`, `ControlMapNode` | `kappa=tan(delta)/L`, `a_y=v^2 kappa`, steering-rate, friction ellipse를 연결하고 planner/controller model mismatch 정량화 |
| 8. 실시간 latency와 clock semantics | source stamp, receipt age, compute delay, actuator response가 서로 다르며 고속에서는 거리 오차가 된다. | `acceptObstacles()`, fixed 0.15 s response delay, FSM/controller freshness | bag 하나에서 scan→confirmed→avoid→local→drive→response의 p50/p95/p99와 `v*tau`를 재현 가능하게 계산 |
| 9. Uncertainty calibration/chance constraints | covariance가 있어도 calibration되지 않으면 adaptive margin이 안전확률을 뜻하지 않는다. | [`perception_to_planning.md`](../04_interfaces/perception_to_planning.md), `ObstacleGuard`, 현재 sigma scale 0 | reliability/coverage plot, face-wise covariance, joint maneuver risk budget을 정의하고 fixed margin과 비교 |
| 10. 실험설계와 ablation | 새 알고리즘보다 먼저 현재 baseline의 실패를 재현하고 causal improvement를 입증해야 논문 기여가 된다. | candidate diagnostics, raw-input replay, 위 B0~B5 설계 | commit/runtime params/oracle/metric을 고정하고 한 축씩 ablation해 confidence interval과 failure taxonomy까지 보고 |

# Top 10 questions I should be able to answer

1. `/scan` 한 프레임이 어떤 단계와 메시지 필드를 거쳐 `/avoid_waypoints`가 되는가?
2. ego의 Cartesian pose와 obstacle AABB가 각각 어떤 방식으로 Frenet에 투영되며 왜 방식이 다른가?
3. 현재 P3 후보의 다섯 station과 `[ego_d,target,middle,target,0]`는 각각 무엇을 뜻하는가?
4. quintic Hermite 계수는 여섯 경계조건에서 어떻게 나오며 C2 continuity는 어떻게 성립하는가?
5. 후보가 hard reject되는 모든 geometry/collision 조건을 코드 순서대로 설명할 수 있는가?
6. 현재 선택이 weighted cost가 아닌 이유와 lexicographic rank의 정확한 우선순위는 무엇인가?
7. curvature, lateral acceleration, accel/decel, steering rate 중 무엇이 hard gate이고 무엇이 shaping/diagnostic인가?
8. `/confirmed_static_obs`, `/static_obs`, `/opp_obs`는 각각 planner/FSM/controller에서 어떤 권한을 갖는가?
9. 현재 planner를 dynamics-aware라고 부를 수 있는 근거와 kinodynamic optimizer라고 부를 수 없는 근거는 무엇인가?
10. 연구 개선이 좋아졌음을 증명하려면 source diff가 아니라 어떤 runtime 데이터, oracle, ablation, metric이 필요한가?
