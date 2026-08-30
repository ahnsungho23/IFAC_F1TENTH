# Frenet 좌표계

## 1. Cartesian만으로 부족한 이유

[GENERAL THEORY] Cartesian `(x,y)`는 지도에서 위치를 표현하기 좋지만, “트랙을 따라 8 m 앞”, “race line 왼쪽 0.5 m” 같은 질문에는 바로 답하지 못한다. Frenet 좌표는 기준 곡선을 따라가는 방향과 그 곡선에서 벗어난 방향을 분리한다.

## 2. 변수와 기저

먼저 기호를 정의한다.

- `r(s)=[x_r(s),y_r(s)]`: 기준선 위치
- `s`: 기준선 시작부터 잰 arc length
- `t(s)=dr/ds`: 단위 tangent
- `n(s)=[-t_y,t_x]`: tangent를 반시계로 90도 돌린 왼쪽 unit normal
- `d`: 기준선에서 normal 방향 signed offset
- `p(s,d)`: 실제 또는 계획 위치

기준선이 arc length로 매개화되면 `||dr/ds||=1`이다. 따라서 위치 변환은

`p(s,d)=r(s)+d n(s)`

이다.

좌표로 쓰면 tangent heading을 `ψ_r`라 할 때

`t=[cosψ_r,sinψ_r]`, `n=[-sinψ_r,cosψ_r]`

이므로

`x=x_r-d sinψ_r`, `y=y_r+d cosψ_r`

가 된다.

[CURRENT IMPLEMENTATION] P3는 global waypoint의 `x_m,y_m,psi_rad`를 복사한 뒤 정확히 이 normal shift로 candidate `x_m,y_m`를 만든다([`p3_shadow.cpp:1071`](../../src/local_planning/src/p3_shadow.cpp#L1071)). 이때 `d>0`은 기준 진행 방향의 왼쪽이다.

## 3. arc length `s`

[GENERAL THEORY] 임의 매개변수 `q`로 곡선 `r(q)`가 주어지면 미소 길이는

`ds = ||dr/dq|| dq`

이고 누적 길이는

`s(q)=∫ ||dr/dq|| dq`

이다. `s` 자체로 매개화하면 `||dr/ds||=1`이 되어 tangent와 curvature 식이 단순해진다.

[CURRENT IMPLEMENTATION] CLCS converter는 입력 waypoint의 저장된 `s_m`을 그대로 믿기보다 geometric path length와의 오차를 계산하고 CLCS의 geometric arc length를 conversion 결과로 사용한다([`frenet_odom_node.cpp:285`](../../src/global_planning/src/frenet_odom_node.cpp#L285), [`clcs_frenet_converter.cpp:538`](../../src/global_planning/src/clcs_frenet_converter.cpp#L538)). closed loop에서는 `s`를 track length 구간으로 wrap한다([`clcs_frenet_converter.cpp:622`](../../src/global_planning/src/clcs_frenet_converter.cpp#L622)).

## 4. 세 가지 직관

### 직선

```text
d>0 (왼쪽)
      p(s,d) ●
             ↑ n
r(s) ────────→────────  t, +s
```

[GENERAL THEORY] 기준선이 x축이면 `r(s)=[s,0]`, `t=[1,0]`, `n=[0,1]`이다. 따라서 `p=[s,d]`로 Cartesian과 거의 같다.

### 원호

[GENERAL THEORY] 반지름 `R`의 원을 반시계로 따라가면 `s=Rθ`이고 기준선 곡률은 `κ_r=1/R`이다. 왼쪽 normal은 원의 안쪽을 향하므로 `d>0`이면 실제 반지름은 대략 `R-d`가 된다. 같은 `d`라도 직선보다 좌표 변환과 곡률 변화가 강하다.

### hairpin

```text
진행 →  =================
         가까운 두 branch
진행 ←  =================
```

[GENERAL THEORY] Euclidean 거리만 보면 반대 방향 branch가 더 가까울 수 있다. 그래서 “가장 가까운 점”만으로 `(x,y)→(s,d)`를 매 프레임 계산하면 `s`가 다른 leg로 점프할 수 있다.

[CURRENT IMPLEMENTATION] detector의 ego projection은 이전 `s` 주변의 monotonic window를 쓰는 `convertTracked()`로 branch continuity를 유지한다([`obstacle_detector_node.cpp:671`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L671)). hairpin opposite-leg를 거부하는 현재 test도 있다([`test_clcs_frenet_converter.cpp:272`](../../src/global_planning/test/test_clcs_frenet_converter.cpp#L272)).

## 5. `(x,y)`에서 `(s,d)`로

[GENERAL THEORY] 개념적으로는 기준 곡선에서 점 `p`에 가장 가까운 `r(s*)`를 찾고,

`d = n(s*) · (p-r(s*))`

로 signed distance를 구한다. 그러나 self-near track에서는 nearest point가 유일하지 않을 수 있으므로 이전 `s`, 진행 방향, 허용 이동창이 필요하다.

[CURRENT IMPLEMENTATION] global planning의 CLCS converter가 정밀 Cartesian-to-Frenet projection을 담당한다. local planner의 P3는 이미 Frenet으로 주어진 ego와 obstacle을 받아 global ordered sample을 normal shift하므로, candidate마다 역 projection을 다시 풀지 않는다.

## 6. `d(s)` 경로와 실제 tangent

`p(s)=r(s)+d(s)n(s)`를 미분한다. 기준선의 Frenet 관계를

`t'(s)=κ_r n`, `n'(s)=-κ_r t`

라 두면

`p'(s)=(1-κ_r d)t+d'n`

이다. 따라서 candidate heading은 단순히 reference heading과 같지 않고, tangent 방향 성분 `1-κ_r d`와 lateral 성분 `d'`의 합으로 정해진다.

[GENERAL THEORY] `1-κ_r d≈0`이면 offset curve가 특이해진다. 예를 들어 `κ_r=1 1/m`, `d=1 m`이면 기준선 tangent 방향 성분이 0이 된다. 좁은 F1TENTH 트랙에서도 큰 곡률·큰 offset 조합은 이 원리를 의식해야 한다.

[CURRENT IMPLEMENTATION] 현재 planner는 이 symbolic Frenet curvature식을 candidate validation에 직접 넣지 않는다. normal shift로 Cartesian 점을 만든 뒤 local cubic fitting 또는 Menger fallback으로 실제 `psi`와 `kappa`를 재계산한다([`raceline_spline_planner.cpp:2663`](../../src/local_planning/src/raceline_spline_planner.cpp#L2663)).

## 7. raceline-lock와 waypoint ordering

[CURRENT IMPLEMENTATION] P3는 `nextReferenceIndex(ego.s)`부터 global waypoint를 순서대로 복사하고 path horizon까지 진행한다([`p3_shadow.cpp:1056`](../../src/local_planning/src/p3_shadow.cpp#L1056)). validator는 인접 waypoint의 ego-forward `s`가 증가하지 않으면 기각한다([`raceline_spline_planner.cpp:2992`](../../src/local_planning/src/raceline_spline_planner.cpp#L2992)). 이것이 raceline-lock의 핵심이다. 새로운 자유형 Cartesian curve를 만드는 대신 기준선 샘플 identity와 순서를 보존한다.

[INFERENCE] 장점은 obstacle, track width, state machine, controller가 같은 `s` 순서를 공유한다는 것이다. 한계는 기준선 projection이나 track width가 틀리면 candidate도 같은 오류를 상속한다는 것이다.

## 8. 숫자 예제

[GENERAL THEORY] reference point가 `(10,3)`, heading이 `ψ_r=30°`, offset이 `d=0.4 m`라고 하자.

- `sin30°=0.5`
- `cos30°≈0.866`
- `x=10-0.4×0.5=9.8`
- `y=3+0.4×0.866≈3.346`

즉 진행 방향의 왼쪽으로 0.4 m 이동한 Cartesian 점은 약 `(9.8,3.346)`이다. `d`의 부호를 반대로 하면 오른쪽 점이 된다.

## 9. 자주 혼동하는 개념

- waypoint 배열 index와 `s`는 같은 것이 아니다. 샘플 간격이 일정하지 않을 수 있다.
- `d`는 global y가 아니다. reference heading에 따라 normal 방향이 회전한다.
- reference heading과 shifted path heading은 `d'(s)` 때문에 다르다.
- closed-loop에서 작은 숫자의 `s`가 반드시 차량 뒤라는 뜻은 아니다. wrap distance를 써야 한다.
- Cartesian nearest projection과 tracked Frenet projection은 hairpin에서 다른 결과를 낼 수 있다.

## 반드시 설명할 수 있어야 하는 질문

1. `p=r+dn`에서 `n=[-sinψ,cosψ]`가 되는 이유는 무엇인가?
2. arc length로 매개화하면 tangent와 곡률 계산이 왜 쉬워지는가?
3. `p'=(1-κ_rd)t+d'n`의 두 항은 각각 무엇을 뜻하는가?
4. hairpin에서 stateless nearest projection이 위험한 이유는 무엇인가?
5. 현재 planner의 raceline-lock는 어떤 ordering 검사를 통해 보존되는가?

