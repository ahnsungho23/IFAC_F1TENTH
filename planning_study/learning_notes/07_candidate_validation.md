# Candidate validation

## 1. 생성과 검증은 다른 책임이다

[GENERAL THEORY] generator는 “가능해 보이는 해”를 제안한다. validator는 그 제안이 반드시 지켜야 할 조건을 하나씩 검사한다. generator의 corridor나 analytic equation이 정교해도, 별도 validator가 없으면 reconstruction·sampling·모델 차이에서 안전 구멍이 생길 수 있다.

[CURRENT IMPLEMENTATION] P3는 candidate path를 만든 뒤 geometry와 speed를 갱신하고 공통 `validateCandidate()`를 호출한다([`p3_shadow.cpp:1084`](../../src/local_planning/src/p3_shadow.cpp#L1084), [`raceline_spline_planner.cpp:2805`](../../src/local_planning/src/raceline_spline_planner.cpp#L2805)). fresh selection은 guarded geometry와 raw geometry에 대해서도 exact validation을 통과해야 lifecycle ownership을 얻는다.

```text
corridor/root/template
        │ proposal
        v
  reconstructed path
        │
  geometry + speed shaping
        │
        v
 hard validator ──fail──> rejection reason
        │pass
        v
 ranking 후보 집합
```

## 2. hard, shaping, ranking을 구분한다

| 종류 | 실패하면 | 현재 예 |
|---|---|---|
| hard gate | 해당 candidate 즉시 탈락 | wall, footprint, obstacle, ordering, slope, curvature/rate |
| speed shaping | path 속도를 낮춰 조건을 맞춤 | lateral acceleration, gap speed, accel/decel pass |
| ranking metric | candidate는 남고 선호도만 바뀜 | velocity loss, safety slack, braking deficit, deviation |

[CURRENT IMPLEMENTATION] `validateCandidate()`가 실제 hard reject하는 것과 `measureCandidate()`가 점수만 측정하는 것을 혼동하면 안 된다.

## 3. 입력과 forward path

[CURRENT IMPLEMENTATION] `validatePath()` wrapper는 reference ready, finite ego, non-empty path, valid collision horizon을 검사하고 자차 앞에 남은 nearest waypoint를 찾는다([`raceline_spline_planner.cpp:3029`](../../src/local_planning/src/raceline_spline_planner.cpp#L3029)). candidate validator는 start index 뒤에 최소 waypoint 수가 없으면 `kNoForwardPath`로 기각한다([`raceline_spline_planner.cpp:2921`](../../src/local_planning/src/raceline_spline_planner.cpp#L2921)).

왜 필요한가: 끝난 path나 빈 배열은 controller lookahead를 정의할 수 없다. geometry가 과거 구간에서만 안전해도 현재 명령으로 쓸 수 없다.

## 4. ego-to-entry continuity

변수를 정의한다.

- `Δd_entry=|d_first_forward-d_ego|`
- `s_entry`: ego에서 첫 forward waypoint까지 거리
- `B`: tracking/error budget 하한
- `S_max`: maximum lateral slope
- `s_base`: 작은 분모 방지 baseline

[CURRENT IMPLEMENTATION] 기각 조건은 단순 slope 하나가 아니라 다음 AND다([`raceline_spline_planner.cpp:2887`](../../src/local_planning/src/raceline_spline_planner.cpp#L2887)).

```text
Δd_entry > max(trackingErrorReserve, B)
AND
Δd_entry / max(s_entry, s_base) > S_max
```

왜 필요한가: path 내부가 매끄러워도 ego와 path 첫 점 사이가 멀리 점프하면 controller는 그 빈 구간을 순간적으로 메워야 한다.

실패 결과: `kGeometry`, `path entry is discontinuous from the current ego d`.

## 5. center track boundary

[CURRENT IMPLEMENTATION] 각 waypoint의 `d`가 reference의 left/right track width에서 `trackBoundaryReserve()`를 뺀 center admissible range를 벗어나면 hard reject한다([`raceline_spline_planner.cpp:2940`](../../src/local_planning/src/raceline_spline_planner.cpp#L2940)). 현재 `trackBoundaryReserve()`는 `wall_safety_margin_m`만 반환한다([`raceline_spline_planner.cpp:573`](../../src/local_planning/src/raceline_spline_planner.cpp#L573)).

왜 필요한가: corridor proposal과 실제 reconstructed waypoint가 track center domain 안에 있는지 다시 확인한다.

실패 결과: `kTrackBoundary`, `d-offset leaves the global waypoint track bounds`.

## 6. 회전 직사각형 footprint와 벽

[CURRENT IMPLEMENTATION] center가 track 안에 있어도 차량 앞·뒤 corner가 wall 밖으로 나갈 수 있다. validator는 vehicle half-length/half-width로 네 Cartesian corner를 만들고, 주변 reference segment에 각각 projection하여 좌우 wall clearance를 계산한다([`raceline_spline_planner.cpp:2192`](../../src/local_planning/src/raceline_spline_planner.cpp#L2192)). 하나라도 reserve 밖이면 hard reject한다.

왜 필요한가: 큰 heading error에서 wallward front corner는 단순 center `d±half_width`보다 더 튀어나온다.

실패 결과: `kTrackBoundary`, `footprint_track_bound`, corner 위치·heading·side diagnostic.

## 7. static obstacle collision

변수를 정의한다.

- obstacle raw lateral bounds: `[d_right,d_left]`
- `C=vehicle_half_width+safety_margin+scaled tracking reserve`
- inflated obstacle interval: `[d_right-C,d_left+C]`
- `H`: current maneuver collision horizon

[CURRENT IMPLEMENTATION] waypoint의 forward `s`가 obstacle longitudinal span에 있고 `d`가 inflated interval 내부에 있으면 hard reject한다([`raceline_spline_planner.cpp:2974`](../../src/local_planning/src/raceline_spline_planner.cpp#L2974)). obstacle collision에만 `H`가 적용된다. wall, ordering, slope, curvature 검사는 path 전체에 적용된다.

왜 필요한가: candidate center가 장애물 box와 겹치지 않더라도 차량 반폭과 margin을 포함하면 충돌할 수 있다.

실패 결과: `kObstacleCollision`, obstacle ID와 source/test bounds가 diagnostic에 남는다.

[CURRENT IMPLEMENTATION] obstacle collision은 rotated rectangle-vs-AABB polygon test가 아니라 Frenet centerline `d`와 inflated interval test다. 반면 track wall에는 rotated rectangular footprint를 쓴다. 두 검사의 geometry fidelity가 같다고 가정하면 안 된다.

## 8. waypoint order

[CURRENT IMPLEMENTATION] 인접 waypoint의 ego-forward distance 차 `Δs`가 양수가 아니면 hard reject한다([`raceline_spline_planner.cpp:2992`](../../src/local_planning/src/raceline_spline_planner.cpp#L2992)).

왜 필요한가: controller와 lifecycle은 배열이 global race-line 진행 순서를 따른다고 가정한다. wrap 처리 후에도 뒤로 가거나 중복된 station은 lookahead와 suffix progress를 모호하게 만든다.

실패 결과: `kGeometry`, `candidate no longer follows increasing global race-line order`.

## 9. lateral slope

[CURRENT IMPLEMENTATION] 인접 waypoint에 대해

`|Δd|/Δs ≤ maximum_lateral_slope`

를 요구한다([`raceline_spline_planner.cpp:2999`](../../src/local_planning/src/raceline_spline_planner.cpp#L2999)). 현재 운영 YAML 값은 `0.8`이지만 runtime override는 별도 확인 대상이다.

왜 필요한가: 매우 짧은 종거리에서 큰 횡이동을 요구하는 path를 제거한다. 다만 slope는 steering·tire dynamics의 완전한 대체물이 아니다.

실패 결과: `kGeometry`, `quintic d-offset exceeds maximum_lateral_slope`.

## 10. curvature와 curvature rate

[CURRENT IMPLEMENTATION] 각 waypoint에서 방향별

`|κ| ≤ min(maximum_curvature, tan(δ_max,side)/L)`

를 요구한다([`raceline_spline_planner.cpp:3013`](../../src/local_planning/src/raceline_spline_planner.cpp#L3013)). 인접 점에서는

`|Δκ|/Δs ≤ maximum_curvature_rate`

를 요구한다([`raceline_spline_planner.cpp:3005`](../../src/local_planning/src/raceline_spline_planner.cpp#L3005)).

왜 필요한가: path가 조향 geometry limit을 넘거나 공간적으로 너무 급하게 굽힘을 바꾸는 것을 막는다.

실패 결과: 둘 다 `kGeometry`이며 direction/curvature-rate reason이 다르다.

## 11. speed와 maneuver 범위

[CURRENT IMPLEMENTATION] `a_y=v²κ` 초과는 현재 `validateCandidate()`의 별도 hard reject가 아니다. 먼저 `applyAvoidanceVelocityLimit()`가 curvature·gap에 맞춰 `vx_mps`를 낮추고, forward/backward pass가 acceleration/deceleration profile을 낮춘다. ego braking deficit은 hard reject하지 않고 ranking 우선순위로 쓴다.

[CURRENT IMPLEMENTATION] collision horizon은 현재 obstacle cluster가 책임지는 끝과 post-merge tail에 맞춘다([`p3_shadow.cpp:1087`](../../src/local_planning/src/p3_shadow.cpp#L1087)). 다음 obstacle이 global tail에 있다는 이유로 현재 maneuver를 무조건 실패시키지 않고 chained maneuver/fallback이 맡게 한다. 이것은 “horizon 밖은 안전하다”는 뜻이 아니라 책임을 순차 분할한다는 뜻이다.

## 12. 숫자 예제

[GENERAL THEORY] obstacle의 lateral bounds가 `[-0.2,+0.2] m`, vehicle half-width가 `0.15 m`, safety margin이 `0.08 m`, 추가 reserve가 0이라 하자.

- `C=0.15+0.08=0.23 m`
- inflated interval=`[-0.43,+0.43] m`
- candidate `d=+0.50 m`: obstacle interval 밖, 이 lateral test는 통과
- candidate `d=+0.40 m`: inflated interval 안, hard collision reject

그러나 `d=0.50`도 left wall/footprint/curvature 검사를 모두 통과해야 최종 hard-valid다.

## 13. 자주 혼동하는 개념

- corridor feasible과 exact hard-valid는 다르다.
- speed cap과 hard reject는 다르다.
- center track bound와 rotated footprint track bound는 둘 다 필요하다.
- collision horizon은 모든 검사를 자르는 것이 아니라 obstacle collision 범위만 제한한다.
- margin이 0이어도 vehicle physical half-width는 사라지지 않는다.

## 반드시 설명할 수 있어야 하는 질문

1. generator와 validator를 분리해야 하는 이유는 무엇인가?
2. entry discontinuity가 두 조건의 AND인 이유는 무엇인가?
3. center track bound와 footprint track bound는 어떻게 다른가?
4. obstacle collision에만 maneuver horizon을 적용하는 이유는 무엇인가?
5. lateral acceleration이 현재 hard reject가 아닌데도 어떻게 반영되는가?
6. hard-valid가 dynamic closed-loop safety guarantee가 아닌 이유는 무엇인가?

