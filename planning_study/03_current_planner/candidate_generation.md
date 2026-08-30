# 현재 회피 후보 생성 과정

## 1. 진입점: `RacelineSplinePlanner::plan()`

현재 정상 계획 진입은 [`raceline_spline_planner.cpp:3451`](../../src/local_planning/src/raceline_spline_planner.cpp#L3451)이다.

```text
ready/finite ego 확인
-> expandVisibleObstacles
-> nearestCluster
-> outsideIsLeft
-> generateP3Candidates
-> hard-valid 후보 정렬
-> avoidance / margin slow pass / safe-stop
```

후보 생성기는 P3 하나뿐이다. 옛 P0 quintic grid는 제거되었다: [`raceline_spline_planner.cpp:3480`](../../src/local_planning/src/raceline_spline_planner.cpp#L3480).

## 2. 장애물 envelope 만들기

`expandVisibleObstacles()`는 detector가 준 절대 Frenet AABB를 ego-relative interval로 바꾼다: [`raceline_spline_planner.cpp:729`](../../src/local_planning/src/raceline_spline_planner.cpp#L729).

장애물 `j`에 대해 개념적으로

```text
center_j = forwardDistance(ego_s, obstacle.s_center)
half_span_j = 0.5 * shortestWrappedSpan(s_start,s_end)
              + obstacle_longitudinal_padding
start_j = center_j - half_span_j
end_j   = center_j + half_span_j

clearance(v,kappa) = vehicle_half_width + safety_margin
                     + optional tracking reserve
inflated d = [raw_d_right-clearance, raw_d_left+clearance]
```

를 만든다. 현재 `obstacle_reserve_mode=none`, `localization_reserve_m=0`이므로 일반 운영 장애물 clearance는 0.15+0.08=0.23 m다. 단, 원본 YAML의 일부 오래된 주석은 0.00을 말하므로 실제 scalar line을 우선해야 한다: [`local_planning.yaml:28`](../../src/local_planning/config/local_planning.yaml#L28), [`local_planning.yaml:40`](../../src/local_planning/config/local_planning.yaml#L40), [`local_planning.yaml:59`](../../src/local_planning/config/local_planning.yaml#L59).

## 3. blocking cluster

라인을 막는 조건은 inflated lateral interval이 `d=0`을 포함하는 것이다. 첫 blocking obstacle을 찾고, 다음 obstacle의 start가 현재 cluster end+0.8 m 이내면 같은 cluster로 묶는다: [`raceline_spline_planner.cpp:783`](../../src/local_planning/src/raceline_spline_planner.cpp#L783), [`raceline_spline_planner.cpp:908`](../../src/local_planning/src/raceline_spline_planner.cpp#L908).

이 설계는 가까운 복수 장애물을 하나의 lateral maneuver로 처리한다. 그러나 longitudinal gap만으로 묶기 때문에 서로 다른 lateral side의 물체도 같은 cluster에 들어갈 수 있고 corridor topology가 복잡해질 수 있다.

## 4. 좌/우 target domain

`computeSideTargetRange()`가 cluster를 왼쪽/오른쪽으로 통과하기 위한 lateral target 범위를 만든다: [`raceline_spline_planner.cpp:959`](../../src/local_planning/src/raceline_spline_planner.cpp#L959).

### 왼쪽 통과

```text
target_min = max(cluster obstacle inflated d_left)
target_max = min_s(track_left_width - wall/body reserve)
```

### 오른쪽 통과

```text
target_max = min(cluster obstacle inflated d_right)
target_min = max_s(-track_right_width + wall/body reserve)
```

target 절댓값은 `minimum_target_offset_m=0.15`와 `maximum_target_offset_m=1.50`으로 제한된다: [`local_planning.yaml:544`](../../src/local_planning/config/local_planning.yaml#L544).

strict domain이 없으면 minimum avoidance speed에서 tracking reserve가 줄어드는 구성을 위한 relaxed domain도 계산한다. 현재 reserve gate가 꺼져 있어 이 사다리의 실효는 제한적이다.

## 5. 코너 바깥쪽 판정

`outsideIsLeft()`는 obstacle 뒤 2 m 정도의 reference curvature 합을 사용한다. 합이 음수인 오른쪽 코너면 왼쪽이 바깥쪽이다: [`raceline_spline_planner.cpp:931`](../../src/local_planning/src/raceline_spline_planner.cpp#L931).

이 값은 좌/우를 직접 선택하지 않고 바깥쪽 exit transition scale에 multiplier를 적용하는 데 쓰인다.

## 6. P3 corridor

`P3ShadowEvaluator::makeCorridor()`는 다음 station들을 모은다: [`p3_shadow.cpp:810`](../../src/local_planning/src/p3_shadow.cpp#L810).

- ego 0
- cluster start/end/mid
- visible obstacle start/center/end
- horizon 안의 모든 global waypoint station

각 station에서 vehicle half width+wall margin을 뺀 track interval을 만들고, active obstacle inflated interval을 차집합한다.

```text
sample.feasible = track_interval - union(obstacle_intervals)
```

왼쪽 통과는 feasible set의 오른쪽 끝 interval, 오른쪽 통과는 왼쪽 끝 interval을 고른다. 인접 sample interval들이 겹치지 않으면 disconnected domain으로 본다: [`p3_shadow.cpp:873`](../../src/local_planning/src/p3_shadow.cpp#L873).

## 7. M0와 M1의 의미

소스 주석상 P3 runtime port는 M0-first, M1 closure, branch filtering, exact validation, ranking, cap-24를 보존한다: [`p3_shadow.cpp:42`](../../src/local_planning/src/p3_shadow.cpp#L42).

- M0: 동결된 baseline template 조합을 먼저 평가
- M0 extension: failure 방향을 판별할 수 있을 때 entry scale을 추가 탐색
- M1: analytic root mapping으로 corridor boundary/curvature continuity 문제의 해를 제안

이 문서에서 M0/M1의 모든 analytic root equation을 재유도하지는 않았다. 실제 branch solver는 [`p3_analytic_solver.hpp`](../../src/local_planning/include/local_planning/p3_analytic_solver.hpp)에 있고, runtime mapping은 [`p3_shadow.cpp`](../../src/local_planning/src/p3_shadow.cpp)에 있다.

후보 상한:

- frozen M0 cap: 16
- M0 extension cap: 12
- 전체 cap: 24

근거: [`p3_shadow.cpp:310`](../../src/local_planning/src/p3_shadow.cpp#L310).

## 8. parameter sampling

운영 baseline은 다음 배열을 사용한다.

- entry fractions: `[0.51458, 0.75, 1.0]`
- exit scales: `[0.49717, 0.69915, 3.69877]`
- outside exit multiplier: `0.40600`
- pre-apex distance family: `[11.4422, 7.6281, 3.8141]` m
- post-apex family: `[2.0595, 4.1190, 6.1785]` m

근거: [`local_planning.yaml:501`](../../src/local_planning/config/local_planning.yaml#L501).

이 배열들의 모든 단순 Cartesian product가 항상 최종 24개가 되는 것은 아니다. domain, analytic branch, cap, early invalidity에 따라 실제 생성 수가 달라진다.

## 9. 다섯 knot와 C2 profile

일반 station은 다음과 같다.

```text
z0: entry start
z1: target 도달/cluster start
z2: cluster middle
z3: cluster end
z4: d=0 merge
```

offset은 `[ego_d,target,middle,target,0]`이고 네 quintic Hermite segment로 연결한다: [`p3_shadow.cpp:638`](../../src/local_planning/src/p3_shadow.cpp#L638).

`middle`은 corridor branch와 curvature-continuity probe에서 제안된다. 따라서 단순 “장애물 옆에 고정 d를 찍고 smoothstep으로 들어갔다 나오는” 구조보다 자유도가 높다.

## 10. ordered global path로 재구성

ego 다음 global waypoint부터

```text
path_end = z4 + max(post_merge_lookahead,
                    |ego_speed| * post_merge_min_time)
```

까지 복사한다. 각 점의 `d`만 profile에서 평가하고 reference normal로 `x,y`를 옮긴다: [`p3_shadow.cpp:1075`](../../src/local_planning/src/p3_shadow.cpp#L1075).

현재 post-merge tail은 최소 5 m 또는 1 s분이다: [`local_planning.yaml:532`](../../src/local_planning/config/local_planning.yaml#L532).

## 11. finalize 순서

`finalizeP3ShadowPath()`가 후보에 다음을 적용한다.

- 최종 geometry/curvature 계산
- curvature/gap/confirmed obstacle speed caps
- approach response-delay ramp
- longitudinal acceleration/deceleration passes
- `ax` 재계산

그 뒤 exact hard validator를 한 번 실행한다: [`p3_shadow.cpp:1097`](../../src/local_planning/src/p3_shadow.cpp#L1097).

## 12. lifecycle 때문에 매 tick 새로 만들지 않는 경우

후보 생성은 active immutable suffix가 hard-valid인 동안 호출되지 않는다. continuation-first ordering은 계산량을 줄이고 path jitter를 막는다: [`local_planner_node.cpp:3240`](../../src/local_planning/src/local_planner_node.cpp#L3240), [`local_planner_node.cpp:3587`](../../src/local_planning/src/local_planner_node.cpp#L3587).

fresh selection 시 conservative guard와 raw geometry 모두 exact-valid해야 record를 소유한다: [`p3_maneuver_lifecycle.cpp:188`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L188).

## 13. candidate generation의 연구 질문

- 24개 cap에서 잘리는 후보가 어떤 topology/속도 구간에 몰리는가?
- sampled corridor connected 판정이 continuous collision-free homotopy를 얼마나 잘 근사하는가?
- target/middle/entry/exit 중 성능 민감도가 가장 큰 변수는 무엇인가?
- `z0..z4` 다섯 knot로 표현할 수 없는 안전 경로가 얼마나 자주 있는가?
- 현 lexicographic rank와 후보 제안 순서가 결합해 좌/우 bias를 만드는가?
