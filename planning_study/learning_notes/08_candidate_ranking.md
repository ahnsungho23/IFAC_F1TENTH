# Candidate ranking

## 1. validation 뒤에 ranking이 온다

[GENERAL THEORY] hard constraint는 “허용/불허”를 가른다. objective 또는 ranking은 허용된 해들 중 어떤 것을 고를지 정한다. 이 순서를 뒤집으면 위험한 후보가 좋은 점수로 안전 제약을 상쇄할 수 있다.

[CURRENT IMPLEMENTATION] 현재 planner는 hard-valid candidate index만 모은 뒤 공통 `betterCandidateRank()`로 stable sort한다([`raceline_spline_planner.cpp:3496`](../../src/local_planning/src/raceline_spline_planner.cpp#L3496)). P3 evaluator와 `plan()`이 같은 header의 비교 함수를 사용한다([`candidate_rank.hpp:24`](../../src/local_planning/include/local_planning/candidate_rank.hpp#L24)).

## 2. weighted sum이 아니다

[GENERAL THEORY] weighted sum이라면 보통

`J=w1J1+w2J2+...`

를 최소화한다. 한 항의 손해를 다른 항의 이득으로 교환할 수 있고, 단위와 weight tuning이 중요하다.

[CURRENT IMPLEMENTATION] 현재 비교는 사전식 순위다. 첫 key가 다르면 그 자리에서 승자가 결정되고 아래 key는 보지 않는다. 따라서 “safety weight”, “curvature weight” 같은 현재 존재하지 않는 항을 설명에 만들면 안 된다.

## 3. 정확한 우선순위

[CURRENT IMPLEMENTATION] `CandidateRankKey`의 실제 비교 순서는 다음과 같다([`candidate_rank.hpp:75`](../../src/local_planning/include/local_planning/candidate_rank.hpp#L75)).

1. `exit_reaches_next_obstacle=false` 우선
2. ego braking deficit이 `ε=1e-9` 이하인 feasible 후보 우선
3. braking deficit이 더 작은 후보
4. `velocity_loss`가 더 작은 후보
5. `minimum_normalized_safety_slack`이 더 큰 후보
6. `global_path_deviation_m`이 더 작은 후보
7. `generation_index`가 더 작은 후보

이를 카드 정렬로 생각할 수 있다.

```text
[다음 장애물 침범?]
        ↓ 동률
[제때 감속 가능?]
        ↓ 동률
[부족 거리]
        ↓ 동률
[속도 손실]
        ↓ 동률
[최소 안전 여유]
        ↓ 동률
[평균 |d|]
        ↓ 동률
[생성 순서]
```

## 4. 각 key의 물리 의미

### 다음 장애물 exit 침범

[CURRENT IMPLEMENTATION] 현재 maneuver의 exit가 다음 obstacle span까지 도달하는 후보는 최우선으로 강등된다. 이것은 현재 obstacle을 피하면서 다음 sequential maneuver의 공간을 없애는 path를 피하려는 정책이다.

### braking deficit

변수는 다음과 같다.

- `v_e`: measured ego speed
- `v_i`: candidate waypoint speed
- `τ`: response delay
- `a`: allowed deceleration
- `s_i`: waypoint까지 forward distance

필요 거리는

`s_required=v_e τ+(v_e²-v_i²)/(2a)`

이고 deficit은 candidate 전체에서 `max(0,s_required-s_i)`의 최대값이다([`raceline_spline_planner.cpp:2116`](../../src/local_planning/src/raceline_spline_planner.cpp#L2116)). 0이면 current ego state에서 profile seam을 제때 따라잡을 수 있다고 보는 모델이다.

### velocity loss

[CURRENT IMPLEMENTATION] 각 waypoint에서

`max(0,v_ref-v_candidate)/v_ref`

를 구해 path 평균을 낸다([`raceline_spline_planner.cpp:2139`](../../src/local_planning/src/raceline_spline_planner.cpp#L2139)). hard gate 이상의 여유를 더 버는 것보다 속도 손실을 먼저 줄이는 정책이다.

### minimum normalized safety slack

[CURRENT IMPLEMENTATION] 각 path가 가진 가장 약한 여유를 비교한다. 현재 candidate 값은 대략 다음 네 normalized slack의 minimum이다([`raceline_spline_planner.cpp:2149`](../../src/local_planning/src/raceline_spline_planner.cpp#L2149)).

- body-referenced wall slack
- obstacle slack
- directional curvature slack
- spatial curvature-rate slack

한 항이 매우 작으면 다른 세 항이 커도 minimum이 작다. rotated footprint clearance는 별도 hard gate/diagnostic이고 이 rank minimum에 직접 들어가지 않는다.

### global path deviation

[CURRENT IMPLEMENTATION] waypoint `|d|`의 평균이다([`raceline_spline_planner.cpp:2144`](../../src/local_planning/src/raceline_spline_planner.cpp#L2144)). 위 key가 모두 동률일 때 race line에 덜 벗어난 후보를 선호한다.

### generation index

[CURRENT IMPLEMENTATION] 마지막 완전 동률을 결정적으로 가른다. 작은 floating noise가 없더라도 symmetric geometry에서 항상 같은 후보가 선택되도록 한다.

## 5. 곡률 cost에 대한 정확한 답

[CURRENT IMPLEMENTATION] 현재 rank에는 독립적인

- `∫κ² ds`
- `∫(dκ/ds)² ds`
- peak curvature 최소화

항이 없다. curvature와 curvature rate는 먼저 hard limit으로 후보를 기각하고, 통과한 후보의 normalized safety slack 중 일부로만 ranking에 간접 참여한다. 따라서 “P3가 곡률 적분 cost를 최소화한다”는 설명은 틀리다.

[INFERENCE] 곡률이 낮은 path가 종종 선택될 수는 있다. 낮은 곡률은 speed cap을 덜 받아 velocity loss가 작고 curvature slack이 클 가능성이 있기 때문이다. 하지만 이는 독립 curvature objective의 결과가 아니다.

## 6. 숫자 비교 예제

[GENERAL THEORY] 두 hard-valid 후보의 key가 다음과 같다고 하자.

| key | A | B |
|---|---:|---:|
| exit reaches next | false | false |
| braking deficit | 0.00 m | 0.00 m |
| velocity loss | 0.12 | 0.10 |
| min slack | 0.80 | 0.30 |
| mean `|d|` | 0.40 m | 0.25 m |

B가 선택된다. velocity loss가 먼저 다르므로 A의 훨씬 큰 safety slack은 비교되지 않는다. 단, 둘 다 이미 hard minimum은 통과했다는 전제가 있다.

이번에는 B의 braking deficit이 `0.04 m`라면 A가 선택된다. braking feasible 여부가 velocity보다 위에 있기 때문이다.

## 7. epsilon과 stable ordering

[CURRENT IMPLEMENTATION] floating 비교 epsilon은 `1e-9` 하나로 통합되어 있다([`candidate_rank.hpp:47`](../../src/local_planning/include/local_planning/candidate_rank.hpp#L47)). 차이가 epsilon 이하이면 다음 key로 넘어간다. `stable_sort`와 generation index가 deterministic ordering을 지킨다.

[INFERENCE] epsilon은 단순 수치 세부사항이 아니다. 두 구현이 다른 epsilon을 쓰면 같은 candidate 집합에서도 다른 path를 선택할 수 있다. 현재 single-source comparator는 그 분기를 줄인다.

## 8. 자주 혼동하는 개념

- “maximum safety slack candidate”라는 result 문구만 보고 slack이 항상 1순위라고 보면 안 된다.
- hard gate를 통과한 뒤의 extra slack과 최소 물리 안전조건은 다르다.
- velocity loss가 낮다는 것은 actual lap time이 반드시 짧다는 뜻이 아니다.
- generation order는 안전 우선순위를 대신하지 않고 마지막 tie-break다.
- weighted sum이 아니므로 weight tuning으로 우선순위를 조금씩 바꿀 수 없다.

[OPEN QUESTION] lexicographic 상위 key의 아주 작은 실측 차이가 하위 key의 큰 robustness 차이를 덮는 빈도, 그리고 그 선택이 실제 lap time·tracking error·collision risk에 미치는 영향은 deterministic replay와 closed-loop 실험이 필요하다.

## 반드시 설명할 수 있어야 하는 질문

1. hard validation과 ranking의 책임은 어떻게 다른가?
2. 실제 일곱 단계 lexicographic 우선순위는 무엇인가?
3. weighted sum과 lexicographic ranking의 trade-off 방식은 어떻게 다른가?
4. curvature가 현재 ranking에 들어가는 정확한 경로는 무엇인가?
5. braking deficit 0인 후보가 speed loss보다 먼저 우선되는 이유는 무엇인가?
6. generation index가 deterministic behavior에 필요한 이유는 무엇인가?

