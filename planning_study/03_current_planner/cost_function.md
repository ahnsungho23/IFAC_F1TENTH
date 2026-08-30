# 후보 선택 함수: weighted cost가 아니라 사전식 순위

## 1. 가장 중요한 결론

현재 플래너에는 흔히 쓰는

```text
J = w_clear J_clear + w_kappa J_kappa + w_jerk J_jerk
    + w_length J_length + w_deviation J_deviation + ...
```

형태의 weighted-sum cost가 없다. hard-valid 후보를 `CandidateRankKey`의 **lexicographic order**로 정렬한다: [`candidate_rank.hpp:49`](../../src/local_planning/include/local_planning/candidate_rank.hpp#L49).

따라서 “cost weight를 tuning한다”는 표현은 현재 코드에 맞지 않는다. tuning 대상은 후보 형상 parameter, hard constraint threshold, lexicographic policy다.

## 2. 실제 우선순위

`betterCandidateRank(first,second)`의 순서는 다음과 같다: [`candidate_rank.hpp:75`](../../src/local_planning/include/local_planning/candidate_rank.hpp#L75).

1. `exit_reaches_next_obstacle=false` 우선
2. ego braking deficit가 0인 후보 우선
3. 모두 deficit가 있으면 더 작은 deficit 우선
4. 더 작은 `velocity_loss` 우선
5. 더 큰 `minimum_normalized_safety_slack` 우선
6. 더 작은 `global_path_deviation_m` 우선
7. 완전 동률이면 더 이른 generation index

동률 epsilon은 `1e-9`다.

## 3. 각 metric의 정의와 단위

### exit reaches next obstacle

- 타입: bool
- 의미: 현재 cluster 뒤 exit ramp가 다음 obstacle의 physical envelope에 닿는가
- 역할: hard reject가 아니라 최우선 강등

### ego braking distance deficit

- 단위: m
- 정의:

```text
max_i( v_ego delay + (v_ego^2-v_i^2)/(2 a_decel) - forward_s_i, 0 )
```

- 근거: [`raceline_spline_planner.cpp:2116`](../../src/local_planning/src/raceline_spline_planner.cpp#L2116)

### velocity loss

- 무차원
- 각 waypoint에서

```text
max(0, v_ref-v_candidate)/v_ref
```

의 path 평균: [`raceline_spline_planner.cpp:2139`](../../src/local_planning/src/raceline_spline_planner.cpp#L2139).

### minimum normalized safety slack

- 무차원
- 다음 네 slack의 최솟값:

```text
body-referenced wall clearance / maximum_target_offset
obstacle clearance / maximum_target_offset
directional curvature margin / directional curvature limit
curvature-rate margin / curvature-rate limit
```

- 근거: [`raceline_spline_planner.cpp:2149`](../../src/local_planning/src/raceline_spline_planner.cpp#L2149)

### global path deviation

- 단위: m
- `mean(|d_i|)`: [`raceline_spline_planner.cpp:2144`](../../src/local_planning/src/raceline_spline_planner.cpp#L2144).

### weight·trade-off 해석

현재 metric에는 숫자 weight가 **모두 없다**. 아래 “우선순위 강화”는 존재하지 않는 weight를 올린다는 뜻이 아니라, 향후 lexicographic 순서를 앞당기거나 해당 metric 차이를 더 엄격하게 비교한다고 가정한 정성적 효과다.

| metric | 현재 weight/우선순위 | 우선순위를 강화하면 | 주 trade-off와 한계 |
|---|---|---|---|
| exit reaches next obstacle | weight 없음, 1순위 bool | 짧은 merge/다음 기동 분리가 더 강해짐 | 현재도 최상위라 더 강화할 숫자 knob가 없음; 다음 obstacle geometry를 현재 horizon에서 충분히 보지 못할 수 있음 |
| ego braking deficit | weight 없음, 2순위 | 현재 속도에서 제때 감속 가능한 후보를 더 강하게 선호 | 더 긴/완만한 진입과 조기 감속으로 lap time·candidate availability 감소; fixed latency/decel model error에 민감 |
| velocity loss | weight 없음, 3순위 | global reference speed를 더 유지 | safety slack과 deviation보다 앞이므로 이미 성능 지향; 실제 controller saturation/실차 lap time을 직접 cost로 쓰지는 않음 |
| minimum normalized safety slack | weight 없음, 4순위, 큰 값 우선 | 벽·obstacle·곡률·곡률률 중 최악 여유가 큰 후보 | velocity loss에 밀리며, 네 slack의 normalization이 실제 collision probability와 동일하지 않음 |
| mean `|d|` | weight 없음, 5순위 | global line 복귀/짧은 횡이동 성향 증가 | 넓은 clearance 또는 쉬운 dynamics보다 reference 추종을 택할 수 있음; path length/시간을 직접 나타내지 않음 |
| generation index | weight 없음, 최종 tie-break | 앞에서 생성되는 side/M0/M1 bias 증가 | 물리적 quality가 아닌 구현 순서라 연구 objective로 해석하면 안 됨 |

curvature는 독립 soft cost가 아니다. limit 위는 hard reject되고, limit 아래에서도 `minimum_normalized_safety_slack`과 `v^2|kappa|` 속도 cap에 따른 `velocity_loss`를 통해 간접 순위에 들어간다. 그래서 동일 `kappa`라도 속도가 높을수록 velocity loss/feasibility 영향이 커지지만, tire friction이나 실제 tracking error까지 cost가 모델링하는 것은 아니다.

## 4. dimensional consistency

weighted sum이 아니므로 m, 무차원, bool을 더하지 않는다. 각 항은 자기 단위 안에서만 비교되고, 우선순위가 항 사이 trade-off를 결정한다. 따라서 “단위가 다른 cost를 더했다”는 문제는 없다.

대신 다음 policy 문제가 있다.

- 상위 항의 아주 작은 차이가 하위 항의 큰 개선을 완전히 무시할 수 있다.
- 모든 항에 같은 absolute epsilon `1e-9`를 쓰지만 scale은 다르다.
- velocity loss가 safety slack보다 항상 먼저다. 단, hard gate가 최소 안전을 이미 보장한다는 설계 판단이다.
- 최종 동률은 generation order이므로 generator 순서가 숨은 bias가 될 수 있다.

## 5. 명시적으로 없는 soft term

현재 source에서 후보 선택의 soft objective로 확인되지 않은 항목:

- path length
- maneuver duration
- lateral jerk 또는 steering jerk
- integral curvature squared
- integral curvature-rate squared
- clearance 평균/적분
- terminal heading error cost
- side switch penalty의 연속 cost
- uncertainty/risk probability cost
- energy consumption

일부는 hard gate나 lifecycle policy로 간접 처리된다. 예를 들어 side switching은 preferred side/lock으로 제한하고, curvature는 hard gate와 speed loss에 반영한다.

## 6. hard constraint와 rank metric을 혼동하지 않기

| 항목 | hard gate | rank metric |
|---|---:|---:|
| wall center/footprint | O | body wall slack |
| obstacle collision | O | obstacle slack |
| max curvature | O | normalized margin + speed loss |
| curvature rate | O | normalized margin |
| lateral slope | O | X |
| braking feasibility | X | deficit |
| acceleration/deceleration | speed shaping | 간접 velocity loss |
| deviation | X | mean absolute d |

## 7. 실제 정렬 위치

`plan()`은 valid candidate index만 모아 `betterCandidateRank()`로 stable sort한다: [`raceline_spline_planner.cpp:3496`](../../src/local_planning/src/raceline_spline_planner.cpp#L3496).

P3 evaluator와 planner commit path가 같은 rank header를 공유한다. 과거 두 구현의 epsilon이 달라 선택이 갈리던 문제를 막는 구조다: [`candidate_rank.hpp:24`](../../src/local_planning/include/local_planning/candidate_rank.hpp#L24).

## 8. audit 가능성

각 candidate에는 다음 값이 남는다.

- generation index, side, target, entry/exit scale
- hard-valid와 rejection reason
- wall/footprint/obstacle clearance
- peak curvature/rate
- braking deficit, velocity loss, deviation, safety slack
- final rank와 exit demotion 전 rank

구성은 [`local_planner_node.cpp:4482`](../../src/local_planning/src/local_planner_node.cpp#L4482)에 있다. 다만 `replay_diagnostics_enable=false`가 운영 기본이므로 runtime bag에 항상 존재한다고 가정하면 안 된다: [`local_planning.yaml:832`](../../src/local_planning/config/local_planning.yaml#L832).

## 9. 연구자가 검토할 질문

- lexicographic 순서가 Pareto frontier의 어떤 부분만 선택하는가?
- `velocity_loss`를 safety slack보다 먼저 두는 것이 실제 lap time/접촉률에서 우월한가?
- normalized safety slack의 네 항이 정말 비교 가능한 risk scale인가?
- minimum 하나만 쓰지 말고 path-integrated risk를 쓰면 무엇이 바뀌는가?
- generation index tie-break가 좌/우 또는 M0/M1 bias를 만드는가?
