# 곡률

## 1. 곡률은 무엇을 말하는가

[GENERAL THEORY] 곡률 `κ`는 경로가 이동 거리당 heading을 얼마나 빨리 바꾸는지를 나타낸다. 직선은 `κ=0`, 반지름 `R`인 원은 크기 `|κ|=1/R`이다. 이 교재의 부호 규약에서는 왼쪽 회전이 양수, 오른쪽 회전이 음수다.

## 2. heading과 tangent

임의 매개변수 `q`의 평면 곡선을

`p(q)=[x(q),y(q)]`

라 하자. tangent vector는 `p_q=[x_q,y_q]`이고 heading은

`ψ(q)=atan2(y_q,x_q)`

이다. `atan(y_q/x_q)` 대신 `atan2`를 쓰는 이유는 사분면과 `x_q=0`을 올바르게 처리하기 위해서다.

## 3. `q`와 arc length의 차이

`q`가 꼭 거리일 필요는 없다. chain rule로

`dψ/ds = (dψ/dq)/(ds/dq)`

이고

`ds/dq=sqrt(x_q²+y_q²)`

이다. 따라서 signed curvature 정의는

`κ=dψ/ds`

이다. `q=s`인 arc-length parameter라면 `ds/dq=1`이라 곧바로 `κ=dψ/ds`가 된다.

## 4. Cartesian curvature 유도

`ψ=atan2(y_q,x_q)`를 미분하면

`dψ/dq=(x_q y_qq-y_q x_qq)/(x_q²+y_q²)`

이다. 이것을 `ds/dq`로 한 번 더 나누면

`κ = (x_q y_qq-y_q x_qq)/(x_q²+y_q²)^(3/2)`

를 얻는다. 분자는 tangent와 acceleration-like vector의 2D cross product이므로 좌우 부호를 주고, 분모는 parameter speed의 영향을 제거한다.

[GENERAL THEORY] `q`의 진행 방향을 뒤집으면 tangent가 뒤집히고 signed curvature 해석도 경로 진행 규약과 함께 바뀐다. 그래서 ordered waypoint 방향이 중요하다.

## 5. 이산 세 점 곡률

연속 미분 대신 세 점 `A,B,C`만 있을 때 각 변 길이를

- `a=|AB|`
- `b=|BC|`
- `c=|AC|`

라 하고 signed cross를

`cross=(B-A)×(C-A)`

라 하면 Menger/circumcircle curvature는

`κ=2 cross/(abc)`

이다. 세 점이 거의 일직선이면 cross가 0에 가까워 `κ≈0`이고, 점이 중복되어 분모가 0에 가까우면 안전한 fallback이 필요하다.

## 6. 현재 planner의 geometry 계산

[CURRENT IMPLEMENTATION] `analytic_path_geometry_enable=true`인 현재 YAML에서는 candidate waypoint가 4개 이상이면 각 점 주변 최대 5개 Cartesian sample에 local cubic을 fitting한다. fitting parameter는 점 사이 누적 Euclidean station이며, fitting한 `x(q),y(q)`의 1·2차 미분을 위 Cartesian 식에 넣어 heading과 signed curvature를 구한다([`raceline_spline_planner.cpp:2663`](../../src/local_planning/src/raceline_spline_planner.cpp#L2663), [`local_planning.yaml:256`](../../src/local_planning/config/local_planning.yaml#L256)).

[CURRENT IMPLEMENTATION] 한 창이라도 derivative가 퇴화하거나 non-finite이면 일부 점만 섞지 않고 path 전체를 legacy 방식으로 되돌린다. legacy heading은 앞뒤 점의 `atan2`, 내부 curvature는 정확히 `2×cross/(abc)`이며 끝점은 이웃 curvature를 복사한다([`raceline_spline_planner.cpp:2750`](../../src/local_planning/src/raceline_spline_planner.cpp#L2750)).

## 7. 왜 local cubic을 쓰는가

[GENERAL THEORY] 세 점 원은 간단하고 기하적으로 명확하지만 waypoint noise와 간격 변화에 민감하다. local cubic fitting은 여러 점을 이용해 derivative를 부드럽게 추정한다. 반대로 창이 너무 넓으면 sharp local feature를 평균내고, 너무 좁으면 noise를 다시 따라간다.

[INFERENCE] 현재 5-sample local fit은 연속 P3 polynomial 자체를 symbolic하게 미분하는 것이 아니다. normal-shift된 최종 Cartesian samples를 다시 fitting하므로 reference curvature와 sampling까지 포함한 실사용 geometry를 평가한다는 장점이 있다. 하지만 fitting window와 sample spacing에 따른 estimation bias는 남는다.

## 8. `d(s)`와 Cartesian 곡률은 다르다

[GENERAL THEORY] `d''(s)`가 작다고 Cartesian `κ`가 반드시 작은 것은 아니다. candidate는 이미 굽은 reference 위에 놓이며

`p'(s)=(1-κ_r d)t+d'n`

이다. 즉 reference curvature `κ_r`, offset `d`, slope `d'`, second derivative `d''`가 모두 actual path curvature에 영향을 준다.

특수하게 `d`가 상수이고 reference가 반지름 `R`의 원이라면 왼쪽 offset curve의 반지름은 `R-d`이고

`κ_offset=1/(R-d)=κ_r/(1-κ_r d)`

이다. `d'=d''=0`이어도 곡률은 reference와 달라진다.

## 9. curvature rate

[GENERAL THEORY] 공간 곡률률은

`dκ/ds`

이며 단위는 `1/m²`이다. 차량이 속도 `v`로 달리면 시간 변화율은 chain rule로

`dκ/dt = v dκ/ds`

이다. 같은 spatial curve라도 고속에서는 steering demand가 더 빨리 변한다.

[CURRENT IMPLEMENTATION] candidate measurement와 hard validator는 인접 waypoint에서 `|Δκ|/Δs`를 계산해 `maximum_curvature_rate_radpm2`와 비교한다([`raceline_spline_planner.cpp:2131`](../../src/local_planning/src/raceline_spline_planner.cpp#L2131), [`raceline_spline_planner.cpp:3005`](../../src/local_planning/src/raceline_spline_planner.cpp#L3005)). 이는 spatial gate이며 직접적인 `rad/s` steering actuator gate는 아니다.

## 10. 숫자 예제

[GENERAL THEORY]

- 반지름 `R=2 m`인 좌회전 원: `κ=+0.5 1/m`
- 같은 원에서 왼쪽으로 `d=0.4 m` constant offset: 새 반지름 `1.6 m`, `κ=0.625 1/m`
- `0.25 m` 사이에 curvature가 `0.2→0.7 1/m`로 바뀌면 spatial rate는 `(0.5)/0.25=2 1/m²`
- 속도 `5 m/s`이면 시간 curvature 변화율은 `5×2=10 1/(m·s)`

마지막 값은 steering angle rate가 아니다. steering model을 통과해야 actuator angle rate가 된다.

## 11. 자주 혼동하는 개념

- `κ=1/R`은 원에서의 크기 관계이며 signed curvature에는 진행 방향과 좌우가 필요하다.
- waypoint의 `s_m`과 local cubic fitting의 Euclidean station은 목적이 다르다.
- `d''`와 Cartesian curvature는 같지 않다.
- spatial curvature rate와 temporal steering rate는 단위부터 다르다.
- smoothing된 curvature는 계산 안정성을 높이지만 진짜 peak를 낮춰 볼 수도 있다.

## 반드시 설명할 수 있어야 하는 질문

1. 임의 parameter `q`의 Cartesian curvature 식을 어떻게 유도하는가?
2. `atan2`가 `atan`보다 필요한 이유는 무엇인가?
3. 현재 local cubic 방식이 P3 polynomial을 직접 미분하는 방식과 어떻게 다른가?
4. constant `d`인데도 원호의 curvature가 바뀌는 이유는 무엇인가?
5. `dκ/ds`와 steering angle rate가 같은 것이 아닌 이유는 무엇인가?

