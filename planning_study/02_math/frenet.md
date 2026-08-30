# Frenet 좌표계와 현재 구현

## 1. 수학적 정의

기준선을 호길이 `s`로 매개화한 곡선 `r(s)=[x_r(s), y_r(s)]`라 하자. 단위 접선과 왼쪽 법선은

```text
t(s) = dr/ds = [cos psi_r, sin psi_r]
n(s) = [-sin psi_r, cos psi_r]
```

이고 Frenet 좌표 `(s,d)`의 Cartesian 위치는

```text
p(s,d) = r(s) + d n(s)
x = x_r(s) - d sin psi_r(s)
y = y_r(s) + d cos psi_r(s)
```

이다. 이 저장소도 같은 부호를 사용한다. `d>0`은 기준선 왼쪽이다.

- P3 waypoint 이동: [`p3_shadow.cpp:1090`](../../src/local_planning/src/p3_shadow.cpp#L1090)
- 일반 helper: [`raceline_spline_planner.cpp:3670`](../../src/local_planning/src/raceline_spline_planner.cpp#L3670)

## 2. Cartesian에서 Frenet으로

현재 구현은 단순 “가장 가까운 waypoint”가 아니라 CommonRoad `CurvilinearCoordinateSystem`을 사용한다.

1. global waypoint의 invalid/duplicate point를 제거하고 closed loop이면 시작점을 끝에 붙인다: [`clcs_frenet_converter.cpp:483`](../../src/global_planning/src/clcs_frenet_converter.cpp#L483).
2. CLCS projection domain을 만든다: [`clcs_frenet_converter.cpp:112`](../../src/global_planning/src/clcs_frenet_converter.cpp#L112).
3. 일반 변환은 `convertToCurvilinearCoordsAndGetSegmentIdx()`를 사용한다: [`clcs_frenet_converter.cpp:168`](../../src/global_planning/src/clcs_frenet_converter.cpp#L168).
4. ego 변환은 이전 `s` 주변의 단조 창을 먼저 검색한다: [`clcs_frenet_converter.cpp:203`](../../src/global_planning/src/clcs_frenet_converter.cpp#L203).

segment-local projection은 접선·법선 성분과 endpoint tangent slope를 이용한다. 구현 식은 [`clcs_frenet_converter.cpp:67`](../../src/global_planning/src/clcs_frenet_converter.cpp#L67)에 있다.

### 왜 ego에는 tracked projection이 필요한가

헤어핀이나 뱀 모양 트랙에서는 Cartesian으로 가까운 두 branch가 서로 먼 `s`일 수 있다. 매 프레임 global nearest projection을 하면 branch flip이 생길 수 있다. 현재 구현은

```text
s in [s_prev - backward_tolerance, s_prev + forward_window]
```

만 먼저 탐색하고, 연속 miss가 설정 횟수에 도달할 때만 global reacquire한다. 창 안에서 실패했는데 즉시 global search로 돌아가지 않는 fail-closed 정책은 [`clcs_frenet_converter.cpp:229`](../../src/global_planning/src/clcs_frenet_converter.cpp#L229)에 명시되어 있다.

## 3. 속도 변환

map-frame 속도 `v=[v_x,v_y]`를 기준선 frame으로 회전하면

```text
v_s =  cos(psi_r) v_x + sin(psi_r) v_y
v_d = -sin(psi_r) v_x + cos(psi_r) v_y
```

다. body-frame 입력이면 먼저 ego yaw로 map-frame에 회전한다. 구현은 [`clcs_frenet_converter.cpp:338`](../../src/global_planning/src/clcs_frenet_converter.cpp#L338)에 있다.

주의: 이 `v_s`는 엄밀한 좌표 미분 `dot{s}`와 항상 같지는 않다. Frenet metric에서는 `p_s=(1-kappa_r d)t`이므로 이상적 관계는

```text
dot{s} = v_t / (1 - kappa_r d)
dot{d} = v_n
```

이다. 현재 converter는 회전 투영한 tangential velocity를 `v_s`로 발행하며 `1-kappa d` 보정을 하지 않는다. 이는 소스에서 확인되는 모델 단순화다.

## 4. closed-loop `s` wrap

track length `L`에 대해

```text
wrap(s) = s mod L, in [0,L)
forwardDistance(a,b) = wrap(b-a)
```

를 사용한다.

- global CLCS normalize: [`clcs_frenet_converter.cpp:622`](../../src/global_planning/src/clcs_frenet_converter.cpp#L622)
- local planner wrap/forward distance: [`raceline_spline_planner.cpp:638`](../../src/local_planning/src/raceline_spline_planner.cpp#L638)
- detector AABB projector wrap: [`aabb_frenet_projector.cpp:31`](../../src/obstacle_detector/src/aabb_frenet_projector.cpp#L31)

장애물 span은 `s_start -> s_end`와 역방향 중 짧은 쪽을 택한다. seam을 가로지르는 AABB도 짧은 물체로 해석하려는 규칙이다: [`raceline_spline_planner.cpp:729`](../../src/local_planning/src/raceline_spline_planner.cpp#L729).

## 5. 장애물의 Frenet 경계

detector는 AABB 중심을 CLCS로 투영한 뒤 중심 접선/법선 축에 네 모서리를 투영하여 1차 `s/d` extent를 만든다: [`aabb_frenet_projector.cpp:225`](../../src/obstacle_detector/src/aabb_frenet_projector.cpp#L225).

라인 쪽 면은 center tangent 근사만 쓰지 않고, 중심과 같은 branch의 reference segment들과 AABB 사이의 최소 거리를 다시 계산한다: [`aabb_frenet_projector.cpp:137`](../../src/obstacle_detector/src/aabb_frenet_projector.cpp#L137), [`aabb_frenet_projector.cpp:271`](../../src/obstacle_detector/src/aabb_frenet_projector.cpp#L271).

따라서 planner가 받는 `s_start/s_end/d_right/d_left`는 단순 원형 radius가 아니라 detector가 만든 Frenet AABB envelope다.

## 6. 급커브에서의 위험

Frenet mapping의 Jacobian은 `1-kappa_r d`에 비례한다. `|kappa_r d|`가 1에 가까우면 다음 문제가 생긴다.

- 서로 다른 `(s,d)`가 가까운 Cartesian 위치에 놓일 수 있다.
- 법선들이 교차하여 mapping이 비단사적이 된다.
- `d(s)`의 작은 변화가 Cartesian curvature를 크게 바꿀 수 있다.
- center tangent로 큰 AABB를 근사하면 `s/d` extent 오차가 커진다.

현재 완화책은 projection domain, ego monotonic window, branch-locked AABB face distance, footprint local reference window, final Cartesian curvature 재계산이다. 그러나 `1-kappa d`를 명시적 hard constraint로 검사하는 코드는 확인하지 못했다. 급커브·큰 offset 조합에서는 별도 검증 가치가 있다.

## 7. 반드시 기억할 구현 차이

- ego `(x,y)->(s,d)`: CommonRoad CLCS + continuity window
- obstacle `(x,y,AABB)->Frenet bounds`: stateless center CLCS + branch-locked face distance
- candidate `(s,d)->(x,y)`: global waypoint의 저장된 `psi_rad` normal 이동
- track width 조회: nearest reference waypoint 또는 local segment 보간

이 네 변환은 동일한 연속 곡선 연산 하나가 아니다. 기준선 sampling, 저장된 `psi_rad`, CLCS tangent가 불일치하면 작은 contract error가 생길 수 있다.

## 8. 추가로 확인할 데이터

- `CLCS geometric s`와 `Wpnt.s_m`의 실제 최대 차이
- `psi_rad`와 CLCS tangent yaw의 차이 분포
- `1-kappa d` 최소값
- hairpin에서 branch flip/reacquire 횟수
- obstacle AABB corner를 모두 직접 CLCS 투영했을 때 현재 envelope와의 차이
