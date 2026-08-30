## Observation 01 — Planner의 temporal steering-rate hard constraint 부재

### Current implementation

현재 local planner는 spatial curvature rate

$$
\left|\frac{\Delta \kappa}{\Delta s}\right|
$$

를 hard constraint로 사용한다.

현재 제한값은:

$$
\left|\frac{\Delta \kappa}{\Delta s}\right|
\le 20\ \mathrm{rad/m^2}
$$

이다.

그러나 실제 시간 기준 steering rate

$$
\dot{\delta}=\frac{d\delta}{dt}
$$

는 P3 candidate의 hard reject 조건으로 사용되지 않는다.

`inspectVelocityFeasibility()`에서 modeled steering-rate violation을 계산하지만, candidate validation 단계에서 이를 이용해 path를 reject하지는 않는다.

Controller에서는 최종 steering command에 steering-rate limit을 적용한다.

### Why this matters

작은 조향각의 kinematic approximation에서는

$$
\dot{\delta}
\approx
Lv\frac{d\kappa}{ds}.
$$

따라서 동일한 spatial curvature-rate를 가진 경로라도 속도가 높을수록 더 큰 steering-rate가 필요하다.

현재 planner의

$$
\left|d\kappa/ds\right|
$$

constraint만으로는 실제 actuator의

$$
|\dot{\delta}|\le\dot{\delta}_{\max}
$$

조건을 보장할 수 없다.

현재 controller의 understeer model까지 고려하면

$$
\dot{\delta}
=
(L+K_{us}v^2)v\frac{d\kappa}{ds}
+
2K_{us}\kappa v\dot v
$$

이므로 steering demand는 curvature-rate뿐 아니라 velocity, curvature, longitudinal acceleration에도 의존한다.

### Possible limitation

Planner가 hard-valid로 판단한 path를 controller가 steering-rate limit 때문에 그대로 추종하지 못할 수 있다.

즉,

$$
\text{planner geometric feasibility}
\neq
\text{closed-loop steering feasibility}.
$$

### Research question

Spatial curvature-rate constraint 대신 또는 함께 speed-aware predicted steering-rate constraint를 candidate feasibility에 포함하면 고속 회피 경로의 실제 추종 가능성을 더 정확하게 판정할 수 있는가?

### Status

[OBSERVATION] Current source에서 확인됨.
[RESEARCH IDEA] 아직 개선안 구현 또는 성능 검증은 하지 않음.

이건 당장 바꾸라는 뜻은 아니고 연구노트에 기록해야 해.

예를 들어 두 후보:

A
velocity loss = 0.100
safety slack  = 0.90

B
velocity loss = 0.099
safety slack  = 0.10

이라고 하자.

B가 이겨.

그런데 겨우

$$ 0.001 $$

의 velocity loss 차이 때문에 safety slack이

$$ 0.9\rightarrow0.1 $$

로 떨어져도 괜찮은가?

이건 current ranking policy의 정당성을 물을 수 있는 좋은 질문이야.

학습노트도 정확히 이걸 open question으로 남겨놨어. 상위 key의 아주 작은 차이가 하위 key의 큰 robustness 차이를 덮는 빈도와 실제 tracking/collision에 미치는 영향은 replay/closed-loop 검증이 필요하다.

다만 첫 논문의 중심은 여전히 candidate generation / geometry-to-root mapping으로 두는 게 좋아.

Ranking까지 동시에 갈아엎으면:

$$ \text{새 generator 때문에 좋아졌나?} $$ $$ \text{새 ranker 때문에 좋아졌나?} $$

를 못 나누게 되니까.

따라서 논문 1차 실험에서는

$$ \boxed{\text{현재 ranker를 고정}} $$

하는 게 맞아.

18. 아주 중요한 연구 관점 한 가지

앞으로 새 mapping A와 기존 mapping B를 비교할 때

“최종 선택 path만 비교하면 안 돼.”

왜냐하면 generator가 이런 후보를 만들었는데:

Generator A
valid candidate 10개
best oracle candidate 포함

Generator B
valid candidate 3개

ranker가 다른 걸 고를 수도 있거든.

그래서 실험에서는 반드시 두 층을 따로 봐야 해.

Generator performance
$$ \boxed{ \text{feasible candidate를 생성했는가?} } $$
Final planner performance
$$ \boxed{ \text{그 후보들 중 ranker가 무엇을 골랐는가?} } $$

이걸 분리해야 나중에 결과 해석이 가능해.
