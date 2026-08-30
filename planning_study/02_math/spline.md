# 현재 spline 구조

## 1. 무엇을 spline으로 만드는가

현재 플래너의 주 미지수는 Cartesian `x(t),y(t)`가 아니라 기준선 대비 lateral offset `d(s)`다. global waypoint의 `s` 순서는 보존되며, 다섯 station에서 정한 offset을 네 개의 5차 Hermite segment로 연결한다.

```text
station: z0       z1       z2       z3       z4
offset : ego_d -> target -> middle -> target -> 0
```

근거:

- station 생성: [`p3_shadow.cpp:544`](../../src/local_planning/src/p3_shadow.cpp#L544)
- offset pattern: [`p3_shadow.cpp:1064`](../../src/local_planning/src/p3_shadow.cpp#L1064)
- C2 profile: [`p3_shadow.cpp:638`](../../src/local_planning/src/p3_shadow.cpp#L638)

## 2. 다섯 station의 의미

일반적인 경우:

```text
z1 = cluster_start
z0 = z1 - z1 * pre_apex_far * entry_fraction / detection_lookahead
z2 = (z1 + cluster_end)/2
z3 = cluster_end
z4 = cluster_end + post_apex_far * exit_scale * outside_multiplier
```

`z0`: 회피 진입 시작, `z1`: 목표 offset 도달, `z2`: corridor probe가 정한 중간 offset, `z3`: 장애물 군집 뒤, `z4`: `d=0` 복귀점이다.

자차가 이미 cluster start에 너무 가까우면 `z0=0`으로 두고

```text
z1 = max(|target-ego_d| / maximum_lateral_slope, reference_spacing)
```

로 바꾼다. 이는 음수/0 길이 segment를 피하고 실제 충돌 여부를 hard validator에 맡기는 처리다: [`p3_shadow.cpp:520`](../../src/local_planning/src/p3_shadow.cpp#L520).

## 3. knot derivative와 acceleration

인접 knot secant slope를

```text
m_i = (d_{i+1}-d_i)/(z_{i+1}-z_i)
```

라 하면 내부 knot derivative는 `sourceHarmonicDerivative()` 규칙으로 정하고, acceleration은

```text
d''_i = 2(m_i-m_{i-1})/(h_{i-1}+h_i)
```

로 정한다: [`p3_shadow.cpp:642`](../../src/local_planning/src/p3_shadow.cpp#L642).

끝점 derivative와 acceleration은 zero-initialized array 때문에 0이다. 각 segment가 양 끝의 `d,d',d''`를 공유하므로 전체 `d(s)`는 knot에서 C2 연속이다.

## 4. C2가 보장하는 것과 보장하지 않는 것

C2 `d(s)`는 offset의 값, 1차, 2차 미분이 연속이라는 뜻이다. 일반적으로 이는 경로 heading과 curvature jump를 줄인다. 그러나 다음을 자동 보장하지는 않는다.

- Cartesian curvature가 제한 이하라는 보장
- curvature rate 연속/제한 보장
- steering rate 제한
- 벽/장애물 collision-free
- 시간축 jerk 제한

그래서 후보 생성 뒤 `x,y`를 만들고 geometry를 다시 계산한 후 hard validation한다: [`p3_shadow.cpp:1097`](../../src/local_planning/src/p3_shadow.cpp#L1097), [`raceline_spline_planner.cpp:2805`](../../src/local_planning/src/raceline_spline_planner.cpp#L2805).

## 5. corridor와 spline의 관계

P3 corridor는 각 sampled station에서

```text
track interval = [-d_right + body/wall projection,
                   d_left  - body/wall projection]
feasible = track interval - inflated obstacle intervals
```

를 만든다: [`p3_shadow.cpp:845`](../../src/local_planning/src/p3_shadow.cpp#L845).

왼쪽 회피는 feasible intervals의 가장 오른쪽 branch, 오른쪽 회피는 가장 왼쪽 branch를 연결한다. 연결되지 않으면 domain이 invalid다: [`p3_shadow.cpp:873`](../../src/local_planning/src/p3_shadow.cpp#L873).

Spline은 이 corridor를 모든 `s`에서 직접 inequality constraint로 최적화하지 않는다. corridor probe로 `target/middle`을 정하고, 완성된 spline을 dense한 global waypoint sample에서 exact validate한다. 따라서 이는 continuous constrained spline optimization보다 **analytic proposal + sampled exact validation**에 가깝다.

## 6. global waypoint sampling

`d(s)`를 arbitrary step으로 재표본화하지 않는다. ego 다음 global waypoint부터 path end까지 기존 ordered waypoint를 복사하고 `d,x,y`만 바꾼다: [`p3_shadow.cpp:1079`](../../src/local_planning/src/p3_shadow.cpp#L1079).

장점:

- global `s` ordering과 track metadata가 유지된다.
- downstream controller의 waypoint 밀도 계약을 유지한다.
- 기준 속도/경계 정보를 직접 재사용한다.

제약:

- 좁은 obstacle/corner feature가 waypoint 사이에 있으면 sampled validator가 놓칠 수 있다.
- station 두 개가 reference spacing보다 가까우면 발행 path에서 구분되지 않는다.
- spline 자체의 연속성과 최종 discrete `x,y,kappa`의 연속성이 완전히 같지 않다.

## 7. 별도 smoothstep helper

`raceline_spline_planner.cpp`에는

```text
q(u)=10u^3-15u^4+6u^5
```

형태의 `quinticSmoothStep()`와 `quinticBlend()`가 있다: [`raceline_spline_planner.cpp:67`](../../src/local_planning/src/raceline_spline_planner.cpp#L67). 현재 repository 검색 기준으로 정의 외 호출은 확인되지 않았다. 따라서 이것을 현재 P3 후보 생성의 핵심 spline으로 설명하면 틀리다. 실제 P3는 일반 경계상태를 받는 `quinticHermite()`를 사용한다.

## 8. 공부할 때 직접 그려볼 것

하나의 실제 후보에서 다음을 `s`축으로 함께 그리면 구조가 빨리 보인다.

- corridor lower/upper
- obstacle `d_right/d_left`
- knot `z0..z4`
- `d(s), d'(s), d''(s)`
- 최종 Cartesian `kappa(s)`
- speed `v(s)`
- hard validation slack
