# 곡률 계산과 제약

## 1. 연속 곡선 곡률

평면 곡선 `p(q)=[x(q),y(q)]`의 signed curvature는

```text
kappa = (x' y'' - y' x'') / (x'^2+y'^2)^(3/2)
```

이다. `q`가 꼭 arc length일 필요는 없다. 현재 `analytic_path_geometry_enable=true`일 때 각 waypoint 주변 최대 5점을 local station에 대해 cubic fit하고 이 식으로 곡률을 계산한다: [`raceline_spline_planner.cpp:2663`](../../src/local_planning/src/raceline_spline_planner.cpp#L2663), [`raceline_spline_planner.cpp:2726`](../../src/local_planning/src/raceline_spline_planner.cpp#L2726).

## 2. fallback 곡률

local cubic fit 중 한 점이라도 퇴화하면 경로 전체를 legacy 방식으로 돌린다.

- heading: 이전/다음 점 차이의 `atan2`
- 내부점 curvature: 세 점 외접원/Menger 형태

```text
kappa = 2 cross(P_i-P_{i-1}, P_{i+1}-P_{i-1}) / (a b c)
```

구현은 [`raceline_spline_planner.cpp:2754`](../../src/local_planning/src/raceline_spline_planner.cpp#L2754)다. 일부만 analytic, 일부만 legacy가 되지 않도록 전체 fallback한다.

## 3. 곡률 hard limit

legacy absolute limit과 실제 control steering geometry limit 중 작은 값을 쓴다.

```text
kappa_max(side) = min(kappa_legacy, tan(delta_max_side)/wheelbase)
```

왼쪽과 오른쪽 최대 조향각이 다르므로 signed curvature의 방향을 본다: [`raceline_spline_planner.cpp:382`](../../src/local_planning/src/raceline_spline_planner.cpp#L382).

운영값:

- wheelbase: 0.33 m
- max steering left/right: 0.410/0.361 rad
- legacy max curvature: 1.3162665 rad/m

근거: [`local_planning.yaml:596`](../../src/local_planning/config/local_planning.yaml#L596).

## 4. 곡률 변화율

인접 waypoint에서 공간 변화율을

```text
|Delta kappa| / Delta s
```

로 계산하고 `maximum_curvature_rate_radpm2`보다 크면 hard reject한다: [`raceline_spline_planner.cpp:3005`](../../src/local_planning/src/raceline_spline_planner.cpp#L3005). 운영 상한은 20 rad/m²다.

이 값은 시간 steering rate `rad/s`가 아니다. 대략

```text
dot{kappa} = (d kappa/ds) * v
```

이므로 속도에 따라 actuator 요구가 달라진다.

## 5. 횡가속 속도 cap

kinematic lateral acceleration approximation은

```text
a_y = v^2 kappa
```

이다. planner는 speed-dependent table `a_lat,max(v)`에 대해

```text
v^2 |kappa| <= a_lat,max(v)
```

가 성립하는 최대 `v`를 이분탐색한다: [`raceline_spline_planner.cpp:416`](../../src/local_planning/src/raceline_spline_planner.cpp#L416).

이 cap은 최종 path curvature에 적용되므로 기준선 곡률이 아니라 회피 전이의 실제 곡률을 반영한다.

## 6. ranking에서 곡률이 들어가는 방식

독립적인 `w_kappa * integral(kappa^2)` cost는 없다. 곡률은 다음 방식으로 영향을 준다.

- 방향별 maximum curvature hard gate
- curvature-rate hard gate
- curvature/rate normalized safety slack
- curvature speed cap이 만든 velocity loss

측정은 [`raceline_spline_planner.cpp:2029`](../../src/local_planning/src/raceline_spline_planner.cpp#L2029), 선택 순서는 [`candidate_rank.hpp:75`](../../src/local_planning/include/local_planning/candidate_rank.hpp#L75)에 있다.

## 7. Frenet `d(s)`와 Cartesian 곡률

기준선이 직선이면 작은 slope에서 대략 `kappa_path ~= d''`로 생각할 수 있다. 하지만 실제 곡선 기준선에서는 `kappa_ref`, `d`, `d'`, `d''`, `1-kappa_ref d`가 함께 들어간다. 그래서 `d(s)` C2나 slope limit만으로 최종 curvature를 보장할 수 없다.

현재 구현이 최종 `x,y`를 만든 뒤 curvature를 재계산하고 hard validate하는 이유가 이것이다.

## 8. 남은 모델 차이

- planner hard gate의 `tan(delta)/L`은 kinematic bicycle geometry다.
- steering diagnostic/controller model은 `delta=kappa(L+K_us v^2)`라는 small-angle understeer model이다.
- 하나는 exact tangent, 다른 하나는 linearized steering relation이라 고곡률에서 완전히 동일하지 않다.
- tire saturation, combined braking/cornering friction circle은 planner hard constraint에 없다.

이 차이는 dynamics-aware 연구의 직접적인 출발점이다.
