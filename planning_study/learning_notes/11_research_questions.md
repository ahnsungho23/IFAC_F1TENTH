# 현재 코드에서 출발하는 연구 질문

## 1. 연구 질문을 고르는 기준

[GENERAL THEORY] 논문 기여는 “코드가 복잡하다”나 “기능을 추가했다”가 아니다. 재현 가능한 gap, 명확한 research question, 비교 baseline, 평가 metric, ablation, 기존 방법과의 차이가 필요하다.

[CURRENT IMPLEMENTATION] 이미 존재하는 quintic, curvature check, lateral acceleration cap, hard validator, lexicographic rank, commitment, safe-stop를 새 기여처럼 이름만 바꾸지 않는다. 아래 후보는 현재 source의 명시적 한계에서 시작하지만, novelty는 문헌조사 전에는 확정하지 않는다.

## 2. 후보 A: cap-24 analytic candidate family의 coverage certificate

### Current

[CURRENT IMPLEMENTATION] P3는 connected Frenet corridor에서 M0-first, M0 extension, M1 round-robin template/root를 제안하고 전체 constructed candidate를 24개로 제한한다([`p3_shadow.cpp:186`](../../src/local_planning/src/p3_shadow.cpp#L186), [`p3_shadow.cpp:1867`](../../src/local_planning/src/p3_shadow.cpp#L1867)). 각 후보는 exact validator를 통과해야 한다.

### Gap

[INFERENCE] “24개 모두 invalid”는 이 template family의 실패다. 동일 corridor 안에 다른 knot state 또는 continuous `d(s)` 해가 존재하지 않는다는 certificate는 아니다.

### Research question

[OPEN QUESTION] 현재 5-knot C² family와 hard constraints에서, 어떤 corridor topology는 finite analytic root set으로 feasibility를 완전 판정할 수 있는가? 완전하지 않다면 false-infeasible 영역을 계산 가능하게 bound할 수 있는가?

### 왜 nontrivial한가

constraint는 station별 interval만이 아니라 reconstructed Cartesian footprint, curvature, curvature rate, ordered global samples, speed shaping에 걸린다. root branch와 active-set boundary도 piecewise하다.

### 필요한 이론

- piecewise-polynomial feasibility
- interval analysis와 root isolation
- active-set/connected-component topology
- spline derivative bounds
- sampling과 continuous constraint 사이의 certificate

### 실험

1. 현재 representative stream과 synthetic straight/curve/hairpin corridor를 고정한다.
2. cap-24 P3와 매우 촘촘한 offline continuous/large-search oracle을 비교한다.
3. P3 false infeasible, oracle false feasible, runtime, certificate gap을 측정한다.
4. M0, M0 extension, M1 template별 ablation을 한다.

### Baseline

- 현재 cap-24 P3
- target/entry/exit/middle dense grid
- generic nonlinear constrained spline optimizer

### novelty risk

[OPEN QUESTION] spline corridor completeness와 lattice resolution 연구는 이미 넓다. novelty가 있으려면 “P3라는 이름”이 아니라 이 코드의 harmonic derivative/analytic branch/corridor 구조에서 새 theorem 또는 유용한 certificate가 나와야 한다. 현재 네 후보 중 custom algorithm과 가장 직접 연결되지만 성공 난도도 높다.

## 3. 후보 B: execution-aware robust candidate certificate

### Current

[CURRENT IMPLEMENTATION] planner는 curvature hard gate, lateral acceleration speed cap, acceleration/deceleration passes를 쓰고 controller model 기반 steering-rate report도 계산한다. 그러나 production hard validation은 closed-loop vehicle rollout이 아니다.

### Gap

[INFERENCE] geometry와 1D speed profile이 valid해도 steering rate, understeer, braking-steering coupling, delay, tracking error가 합쳐지면 vehicle footprint는 planned tube를 벗어날 수 있다.

### Research question

[OPEN QUESTION] 현재 controller와 식별된 uncertainty를 포함한 저차 closed-loop rollout으로, cap-24 candidate마다 “주어진 속도에서 tracking tube 안에 남는다”는 실시간 certificate를 만들 수 있는가?

### 왜 nontrivial한가

너무 단순한 model은 위험을 놓치고, 너무 복잡한 tire model은 Jetson의 25 ms cycle과 식별 가능성을 넘는다. controller 내부 preview, saturation, latency를 planner model과 중복 없이 맞춰야 한다.

### 필요한 이론

- kinematic/dynamic bicycle model
- L1 guidance closed-loop error dynamics
- actuator delay와 rate saturation
- robust reachable tube 또는 contraction/error bound
- real-time model reduction

### 실험

1. current geometry-only hard-valid candidates를 모두 저장한다.
2. 동일 path를 current controller-in-the-loop simulator와 real bag replay initial states에 넣는다.
3. collision, peak tracking error, steering saturation time, runtime을 비교한다.
4. no-understeer, no-delay, no-rate-limit ablation으로 어떤 model term이 필요한지 보인다.

### Baseline

- 현재 hard validator
- `inspectVelocityFeasibility()` diagnostic threshold
- kinematic bicycle rollout
- richer dynamic bicycle oracle

### novelty risk

[OPEN QUESTION] dynamics-aware trajectory validation과 reachable tube는 매우 성숙한 분야다. 단순히 bicycle rollout을 붙이는 것은 구현 개선이지 논문 novelty가 아닐 가능성이 높다. 25 ms 안의 검증 가능한 error bound와 현재 analytic candidate family의 결합이 기여가 되어야 한다.

## 4. 후보 C: perception-localization-control 불확실성의 공동 risk allocation

### Current

[CURRENT IMPLEMENTATION] detector envelope, safety margin, optional tracking LUT/localization reserve, frozen guard, retention band가 서로 다른 단계에서 여유를 다룬다. 운영 설정은 LUT reserve를 끄고 fixed margin을 쓰며, fresh와 committed path에 다른 reserve scale을 적용한다.

### Gap

[INFERENCE] 이 여유들은 probability가 아니라 engineering bound다. detector face error, Frenet projection error, localization bias, controller tracking error가 상관되어 있을 수 있고, 같은 오차를 중복 계상하거나 누락할 수 있다.

### Research question

[OPEN QUESTION] same-frame perception/localization/control residual로 횡방향 collision risk budget을 구성하고, fresh selection과 committed retention에 서로 다른 허용 risk를 주면서도 전체 maneuver risk bound를 유지할 수 있는가?

### 왜 nontrivial한가

오차 분포는 speed, curvature, obstacle side, occlusion phase에 의존하고 iid가 아니다. lifecycle의 repeated validation 때문에 per-frame risk를 단순 합산할 수도 없다.

### 필요한 이론

- conditional quantile/conformal calibration
- chance constraints와 risk allocation
- correlated error propagation
- sequential/hybrid event probability
- dataset shift detection

### 실험

1. obstacle face ground truth 또는 high-quality annotation, localization reference, planned/actual path를 timestamp-align한다.
2. speed×curvature×visibility bin별 error joint distribution을 추정한다.
3. fixed margin, LUT, calibrated bound를 같은 raw-input replay에 비교한다.
4. collision/near-miss coverage, false infeasible rate, lap time, path churn을 함께 보고한다.

### Baseline

- 현재 fixed margin + reserve mode none
- current LUT mode
- 독립 Gaussian chance constraint
- distribution-free calibrated quantile

### novelty risk

[OPEN QUESTION] chance-constrained planning 자체는 새롭지 않다. novelty는 multi-source error를 current lifecycle의 guard/retention semantics와 일관되게 결합하고 실제 racing data에서 calibration coverage를 입증하는 데 있어야 한다.

## 5. 후보 D: sequential maneuver와 safe-stop의 deadlock-free hybrid policy

### Current

[CURRENT IMPLEMENTATION] 현재 system은 continuation, invalidation, same-callback fresh selection, chained maneuver, handoff hold, safe-stop A/B/C/D release를 rule-based hybrid lifecycle로 연결한다. 여러 실제 deadlock fix가 코드에 축적되어 있다.

### Gap

[INFERENCE] 개별 branch test는 많지만, “static obstacle가 bounded perception dropout을 겪어도 collision 없이 결국 progress하거나 명시적 terminal stop에 도달한다”는 시스템 수준 liveness/safety proof는 없다.

### Research question

[OPEN QUESTION] 현재 non-empty output contract와 bounded dropout/latency 가정 아래에서 lifecycle을 finite hybrid automaton으로 추상화하고, collision safety와 no-silent-deadlock progress property를 model-check하거나 runtime monitor로 증명할 수 있는가?

### 왜 nontrivial한가

continuous `s,d,v`, wrapped track, remembered obstacle guards, counter-based release, external FSM state가 함께 있다. overly coarse abstraction은 false alarm을 만들고, fine abstraction은 state explosion을 일으킨다.

### 필요한 이론

- hybrid automata와 temporal logic
- abstraction/refinement
- runtime verification
- bounded-delay asynchronous systems
- reachability와 liveness

### 실험

1. current lifecycle tests와 observed failure timelines를 transition trace로 변환한다.
2. empty/non-empty perception, stale source, close obstacle, sequential obstacle를 bounded nondeterminism으로 만든다.
3. current policy에서 counterexample를 찾고, 이미 수정된 과거 policy가 known failure를 재현하는지 확인한다.
4. monitor overhead와 false alarm을 real replay에 측정한다.

### Baseline

- 현재 unit/scenario tests
- rule coverage only
- bounded explicit-state model checking
- runtime temporal monitor

### novelty risk

[OPEN QUESTION] model checking과 runtime verification도 성숙한 분야다. 단순 상태도 작성은 논문이 아니다. real racing lifecycle의 non-empty path/FSM/safe-stop coupling에서 재현 가능한 counterexample와 일반화 가능한 abstraction을 제시해야 한다.

## 6. 우선순위

[INFERENCE] 코드 고유성과 논문 가능성만 보면 A가 가장 직접적이다. 실제 안전 가치와 구현 가능성은 B 또는 C가 높을 수 있지만 선행 연구가 많아 novelty risk도 크다. D는 현재 축적된 failure evidence를 가장 잘 활용하지만 formal methods 역량과 정확한 environment assumption이 필요하다.

권장 순서는 다음과 같다.

1. A의 oracle gap 측정: 실제 false infeasible가 거의 없으면 theorem 연구를 시작하기 전에 중단할 수 있다.
2. B의 diagnostic-only shadow rollout: production authority 없이 현재 candidate를 평가해 정보가 있는지 본다.
3. C의 data audit: ground truth와 timestamp quality가 부족하면 probabilistic claim을 하지 않는다.
4. D의 작은 automaton: safe-stop release만 먼저 모델링하고 전체 stack으로 확대한다.

## 7. 논문 전에 반드시 고정할 baseline

[CURRENT IMPLEMENTATION] cleanup baseline은 representative stream의 candidate order, selection identity, path digest를 deterministic fingerprint로 비교할 수 있다([`behavior_contract.md:10`](../00_cleanup_baseline/behavior_contract.md#L10)). 연구 branch에서는 최소한 다음을 고정해야 한다.

- source commit과 runtime parameter dump
- raw sensor/odom/reference 입력
- current cap-24 candidate trace 전체
- selected identity와 path digest
- current controller/vehicle parameters
- failure taxonomy와 metric definition
- compute platform과 runtime percentile

[OPEN QUESTION] 외부 문헌과 정확한 novelty comparison은 이 교재의 static source audit 범위 밖이다. 연구 착수 전 최신 Frenet sampling, corridor/lattice completeness, robust MPC/reachable tube, chance-constrained planning, hybrid verification 문헌을 체계적으로 조사해야 한다.

## 반드시 설명할 수 있어야 하는 질문

1. 이미 구현된 기능과 research gap을 어떻게 구분하는가?
2. 후보 A에서 cap-24 실패와 continuous infeasibility는 왜 다른가?
3. 후보 B가 단순 bicycle rollout 추가로 끝나면 novelty가 약한 이유는 무엇인가?
4. 후보 C에서 per-frame error quantile을 maneuver risk로 바로 부를 수 없는 이유는 무엇인가?
5. 후보 D의 safety와 liveness property는 각각 무엇인가?
6. 네 후보를 시작하기 전에 어떤 baseline을 고정해야 하는가?

