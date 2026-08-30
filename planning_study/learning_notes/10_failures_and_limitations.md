# 실제 실패에서 배우는 한계

## 1. 증거 수준

[CURRENT IMPLEMENTATION] 이 장의 “관측”은 현재 source 주석과 repository의 기존 deterministic artifact에 기록된 사건을 뜻한다. 이번 문서 작성에서는 bag을 다시 재생하거나 차량 실험을 하지 않았다. 따라서 사건의 수치와 원인 설명은 현재 코드에 보존된 engineering evidence이며, 독립 재현 결과와는 구분한다.

## 2. 진입점 jump와 과잉 수정

### 증상

[CURRENT IMPLEMENTATION] validator 주석은 2026-08-17 실차에서 ego `d≈0.043`인데 0.20 m 앞 path 첫 점 `d≈0.490`인 경로가 통과되어 충돌한 사례를 기록한다. lateral gap은 약 0.447 m, 단순 기울기는 약 2.3이었다([`raceline_spline_planner.cpp:2854`](../../src/local_planning/src/raceline_spline_planner.cpp#L2854)).

### 원인

path 내부 waypoint 사이 slope만 검사했고, 현재 ego에서 첫 forward waypoint로 건너뛰는 구간은 검사하지 않았다.

### 수학·알고리즘

필요한 질문은 `|d_first-d_ego|/s_entry`가 너무 큰가이다. 그러나 `s_entry`는 ego가 두 sample 사이 어디에 있느냐에 따라 0에 가까워질 수 있어 단독 slope는 발산한다.

### 현재 수정

[CURRENT IMPLEMENTATION] 현재는

`gap > tracking_budget AND gap/max(s_entry,0.50 m) > max_slope`

일 때만 기각한다. tracking budget 하한과 denominator baseline으로 normal tracking error를 작은 수로 나눈 false invalidation을 막는다([`raceline_spline_planner.cpp:2897`](../../src/local_planning/src/raceline_spline_planner.cpp#L2897)).

### 남은 문제

[OPEN QUESTION] `0.20 m`, `0.50 m`, slope limit의 조합은 source에 보존된 특정 사례로 정당화되지만, localization/controller 변경 후에도 false accept와 false reject의 최적 경계인지는 자동 보장되지 않는다.

## 3. speed step과 제동 포화

### 증상

[CURRENT IMPLEMENTATION] source는 obstacle span 경계 또는 S-transition에서 `6.63→4.58 m/s`를 `0.25 m` 안에 요구하여 약 `46 m/s²` 감속 명령이 생긴 기록과, 급제동이 service brake saturation·slip·steering loss로 이어진 실차 충돌을 기록한다([`raceline_spline_planner.cpp:2467`](../../src/local_planning/src/raceline_spline_planner.cpp#L2467), [`raceline_spline_planner.cpp:2486`](../../src/local_planning/src/raceline_spline_planner.cpp#L2486)).

### 원인

각 waypoint의 curvature/gap speed cap을 독립적으로 적용하면, 곡률이 잠깐 낮아지는 점에서 reference speed로 튀었다가 다음 점에서 다시 낮아질 수 있다.

### 수학·알고리즘

deceleration limit `a`에서 다음 점 속도 `v_{i+1}`까지 거리 `Δs` 안에 도달 가능한 이전 속도는

`v_i≤sqrt(v_{i+1}²+2aΔs)`

이다. acceleration 방향도 같은 energy relation으로 전진 전파할 수 있다.

### 현재 수정

[CURRENT IMPLEMENTATION] measured ego speed를 seed로 forward acceleration pass를 하고, 뒤에서 앞으로 speed-dependent deceleration pass를 적용한다. obstacle 접근에는 response delay distance를 포함한 ramp를 별도로 깐다([`raceline_spline_planner.cpp:2520`](../../src/local_planning/src/raceline_spline_planner.cpp#L2520), [`raceline_spline_planner.cpp:2590`](../../src/local_planning/src/raceline_spline_planner.cpp#L2590)).

### 남은 문제

[OPEN QUESTION] 이 profile은 command feasibility의 1D model이다. tire friction circle, steering과 braking의 동시 포화, actuator lag, grade를 시간 적분하지 않는다.

## 4. collision horizon 불일치

### 증상

[CURRENT IMPLEMENTATION] source는 앞 장애물용 path의 merge 뒤 global tail이 약 12 m 앞의 다음 장애물과 겹친다는 이유로 P3 candidate가 반복 전멸하고, 선택·폐기·재선택 및 safe-stop이 반복된 기록을 남긴다([`p3_shadow.cpp:1087`](../../src/local_planning/src/p3_shadow.cpp#L1087), [`p3_maneuver_lifecycle.cpp:413`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L413)).

### 원인

candidate 선택과 committed suffix 재검증이 서로 다른 obstacle collision horizon을 봤다. 현재 maneuver가 책임지지 않는 post-merge controller tail까지 다음 장애물로 심판했다.

### 수학·알고리즘

한 maneuver의 obstacle scope를 `cluster_end + post_merge_lookahead`로 정의하고, generation·validation·continuation 모두 같은 scope를 써야 한다. 그 밖의 next obstacle은 sequential planning으로 넘긴다.

### 현재 수정

[CURRENT IMPLEMENTATION] P3 certificate, `generateP3Candidates()`, lifecycle suffix가 같은 maneuver-scope 정의를 사용한다. wall과 geometry는 path 전체를 계속 검사한다.

### 남은 문제

[OPEN QUESTION] sequential split은 next obstacle에 대한 time-to-replan이 충분하다는 가정을 둔다. close multi-obstacle arrangement에서 next maneuver가 항상 제때 생성된다는 formal guarantee는 없다.

## 5. safe-stop이 만든 영구 정지

### 증상

[CURRENT IMPLEMENTATION] source는 안전하게 정지했지만 obstacle에 너무 가까워져 정지 위치에서는 모든 회피 후보가 곡률/진입거리로 전멸하고, 사람이 개입해야 했던 사례를 기록한다([`raceline_spline_planner.cpp:3277`](../../src/local_planning/src/raceline_spline_planner.cpp#L3277)). 또 empty detector frame을 무조건 clear가 아니라고 처리했을 때 근접 사각에서 영구 래치가 된 사례도 safe-stop lifecycle 주석에 남아 있다([`safe_stop_lifecycle.cpp:116`](../../src/local_planning/src/safe_stop_lifecycle.cpp#L116)).

### 원인

“충돌 전에 설 수 있는가”만 물었고 “그 정지점에서 전진 회피로 다시 출발할 수 있는가”를 묻지 않았다. release 쪽에서는 위험구간이 이미 뒤로 갔는지와 fresh empty frame을 함께 해석하지 않았다.

### 수학·알고리즘

정지점 후보마다 ego를 그 위치와 `v=0`으로 옮겨 P3 feasible candidate 존재 여부를 다시 묻는다. 불가능하면 더 뒤의 정지점을 찾는다. release는 obstacle passed, valid avoidance, stopped+clear, blind timeout의 OR로 구성한다.

### 현재 수정

[CURRENT IMPLEMENTATION] `buildSafeStop()`은 requested stop point가 escapable한지 검사하고, 필요하면 feasible/infeasible interval을 이분 탐색해 가장 늦은 escapable point를 찾는다. 어디서도 불가능하면 경고를 남긴다([`raceline_spline_planner.cpp:3282`](../../src/local_planning/src/raceline_spline_planner.cpp#L3282)). lifecycle은 fresh sequence와 remembered danger range를 포함한 release 조건을 쓴다.

### 남은 문제

[OPEN QUESTION] 모든 forward path가 불가능할 때 현재 local planner는 reverse maneuver를 생성하지 않는다. blind timeout은 creep fallback이지 물체 부재의 증명도 아니다.

## 6. 너무 짧은 stop path

### 증상

[CURRENT IMPLEMENTATION] 두 점짜리 `[현재속도,0]` stop path에서 controller lookahead가 곧바로 끝점 0을 읽어 gradual profile을 건너뛰고 즉시 0 command를 낸 사례가 source에 기록되어 있다([`raceline_spline_planner.cpp:3363`](../../src/local_planning/src/raceline_spline_planner.cpp#L3363)).

### 원인

planner는 속도 profile을 두 점에 넣었지만 downstream controller의 lookahead sampling contract에는 점이 부족했다.

### 수학·알고리즘

연속 제동곡선 `v(s)=sqrt(2a(s_stop-s))`를 controller가 여러 lookahead 위치에서 읽으려면 충분한 spatial sample이 필요하다.

### 현재 수정

[CURRENT IMPLEMENTATION] stop path를 `minimum_path_points` 이상으로 densify하고 모든 보간점에 제동 profile을 다시 적용한다.

### 남은 문제

[OPEN QUESTION] minimum point count보다 실제 sample spacing과 controller lookahead distance의 조합이 본질적이다. controller 변경 시 contract test가 필요하다.

## 7. 이미 해결된 문제를 연구 기여로 착각하지 않기

[CURRENT IMPLEMENTATION] 다음은 현재 코드에 이미 있다.

- 5-knot C² quintic-Hermite reconstruction
- direction-specific curvature hard gate
- local cubic curvature estimation과 Menger fallback
- curvature/gap lateral-acceleration speed shaping
- acceleration/deceleration forward/backward feasibility passes
- exact hard validator와 lexicographic ranking 분리
- continuation-first immutable suffix와 retention band
- safe-stop escape point 검사와 persistent release lifecycle
- raw speed authority와 confirmed lateral-path authority 분리
- single source candidate ranking과 path digest utility

[INFERENCE] “곡률을 고려한다”, “5차 경로를 쓴다”, “안전정지를 넣는다”, “경로를 유지해 jitter를 줄인다”만으로는 현재 코드 대비 새 연구 질문이 아니다.

## 8. 현재 구조적 한계

### 동적 obstacle

[CURRENT IMPLEMENTATION] P3 collision은 static spatial envelope다. dynamic opponent의 time-indexed collision을 계산하지 않는다.

### model mismatch

[CURRENT IMPLEMENTATION] hard curvature gate는 kinematic steering geometry를 사용하고, temporal steering-rate/understeer feasibility는 diagnostic에 더 가깝다. closed-loop actuator/tire rollout은 없다.

### 불확실성

[CURRENT IMPLEMENTATION] margin/LUT/guard/retention은 존재하지만 calibrated probabilistic collision bound는 아니다. 운영 YAML은 `obstacle_reserve_mode: none`이며 current comments와 숫자에도 stale 부분이 있다. 예를 들어 상단 주석은 `safety_margin=0.00`을 설명하지만 실제 key는 `0.08`이다([`local_planning.yaml:15`](../../src/local_planning/config/local_planning.yaml#L15), [`local_planning.yaml:40`](../../src/local_planning/config/local_planning.yaml#L40)).

### control contract mismatch

[CURRENT IMPLEMENTATION] cleanup baseline test는 planner real YAML right steering `0.361`과 control real launch default `0.410`의 기존 mismatch를 기록한다([`test_results.md:23`](../00_cleanup_baseline/test_results.md#L23)). 이것은 이번 교재에서 어느 쪽을 고쳐 결정할 문제가 아니다.

### finite candidate family

[CURRENT IMPLEMENTATION] P3는 cap 24의 template/root family다. hard-valid candidate가 없다는 결과는 이 family 안에서 없다는 뜻이지, 모든 연속 `d(s)` 공간에 해가 없다는 formal proof가 아니다.

### geometry approximation

[CURRENT IMPLEMENTATION] obstacle collision은 Frenet center point와 inflated interval을 비교하고, wall은 rotated rectangular footprint를 검사한다. reference/projection error와 non-rectangular obstacle shape는 완전 모델링하지 않는다.

## 9. 자주 혼동하는 개념

- 현재 수정이 들어갔다고 모든 원인이 영구 해결된 것은 아니다.
- source 주석의 bag 통계는 이번 turn의 독립 replay 결과가 아니다.
- safe-stop success는 이후 자율 탈출 success와 다르다.
- candidate 전멸은 continuous feasible path 부재의 증명이 아니다.
- deterministic replay 동등성은 physical safety validation과 다르다.

## 반드시 설명할 수 있어야 하는 질문

1. ego-entry discontinuity 수정이 왜 단순 slope gate에서 AND gate로 발전했는가?
2. waypoint별 speed cap만으로 제동 feasibility를 보장할 수 없는 이유는 무엇인가?
3. maneuver collision horizon을 generation과 continuation에서 같게 써야 하는 이유는 무엇인가?
4. 안전하게 정지하는 것과 정지 후 탈출 가능한 것은 왜 다른가?
5. 현재 구현된 항목 중 연구 novelty로 다시 주장하면 안 되는 것은 무엇인가?
6. cap-24 candidate 전멸이 continuous infeasibility proof가 아닌 이유는 무엇인가?

