# Production candidate validation, ranking, lifecycle audit

## 감사 범위와 판독 규칙

- `[CURRENT IMPLEMENTATION]` 감사 대상은 `planning_cleanup`, HEAD `b4e54ebc3b915d28609353c7e1c93f3ec1cec164`의 현재 working-tree source이다. 기존 `planning_study` 문장은 결론의 근거로 사용하지 않았다.
- production source, 기본 launch가 읽는 YAML, header, unit/regression test를 서로 대조했다. 이 문서를 만들면서 build나 test를 실행하지 않았고 runtime parameter dump도 얻지 않았다. 따라서 아래의 “effective production”은 **기본 `local_planning.launch.py`를 override 없이 실행할 때의 정적 유효값**이다.
- `[GENERAL THEORY]`는 현재 코드와 구분해야 하는 일반 개념이다.
- `[INFERENCE]`는 코드가 직접 이름 붙이지 않은 해석이다.
- `[NEEDS INSTRUMENTATION]`은 현재 외부 diagnostic만으로 실험 후 판별할 수 없는 항목이다.

가장 먼저 바로잡아야 할 이름은 다음과 같다.

> `[CURRENT IMPLEMENTATION]` 현재 package의 회피 후보 generator는 P3 analytic corridor 하나뿐이다. 옛 P0 quintic grid는 제거되었다. `P0`는 현재 node에서 pre-integration safety/commitment/fallback 흐름의 owner 이름으로 남아 있을 뿐 별도 후보 family가 아니다. `P2`라는 production 단계, class, 함수, mode는 source/config/test 어디에도 없다. 따라서 실제 경로를 `P0 → P2 → P3`로 설명하면 틀린다. 근거는 [`generateP3Candidates()`](../../src/local_planning/src/raceline_spline_planner.cpp#L3104), [`plan()`](../../src/local_planning/src/raceline_spline_planner.cpp#L3451), [P3 mode 분기](../../src/local_planning/src/local_planner_node.cpp#L3507)이다.

---

## 1. Candidate Validation

### 1.1 실제 validation authority

`validateCandidate()`가 최종 공통 hard geometry authority이다. 다만 그 함수 하나만 보면 전체 판정은 알 수 없다.

- P3 candidate는 먼저 C2 profile과 Cartesian path를 만들고, geometry와 speed를 성형한 뒤 `validateP3ShadowPath()`에서 `measureCandidate()`와 `validateCandidate()`를 호출한다. [`buildCandidate()`](../../src/local_planning/src/p3_shadow.cpp#L1045), [`finalizeP3ShadowPath()`](../../src/local_planning/src/raceline_spline_planner.cpp#L1147), [`validateP3ShadowPath()`](../../src/local_planning/src/raceline_spline_planner.cpp#L1163)
- `RacelineSplinePlanner::plan()`이 P3 trace를 safety flow로 가져올 때 trace의 verdict를 그대로 믿지 않고 다시 `measureCandidate()`와 `validateCandidate()`를 호출한다. [`generateP3Candidates()`](../../src/local_planning/src/raceline_spline_planner.cpp#L3104)
- fresh P3 lifecycle은 guarded-geometry certificate를 재사용할 수 있지만 current raw obstacle geometry에 대한 exact validation은 반드시 한 번 더 한다. [`selectFresh()`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L119)
- frozen suffix, legacy commitment, safe-stop prefix도 결국 같은 validator를 쓰지만 minimum point 수, obstacle reserve scale, collision horizon, entry-continuity 면제 여부가 caller별로 다르다. [`continueCurrent()`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L272), [`validatePath()`](../../src/local_planning/src/raceline_spline_planner.cpp#L3029)

### 1.2 Hard reject 표

여기서 $s_i,d_i,v_i,\kappa_i$는 candidate waypoint, $f_i=\operatorname{forwardDistance}(s_{ego},s_i)$, $\epsilon=10^{-9}$이다. track의 `d_left`는 양의 좌측 폭, `d_right`는 양의 우측 폭이고 Frenet $d>0$가 좌측이다.

| 순서 | constraint | 실제 검사와 수학식 | 기본 production threshold | 검사 대상 | 실패 reason / 분류 | 적용 범위 |
|---:|---|---|---|---|---|---|
| 1 | entry continuity | [`validateCandidate()`](../../src/local_planning/src/raceline_spline_planner.cpp#L2887). 자차보다 엄밀히 앞선 최소 $f_e$ 점을 고르고 $\Delta d_e=|d_e-d_{ego}|$, $B=\max(f_e,0.50)$, $E=\max(\text{trackingReserve},0.20)$. **두 조건의 AND**, $\Delta d_e>E \land \Delta d_e/B>0.8$, 일 때 reject | reserve mode `none`, localization 0이므로 $E=0.20\,m$; baseline $0.50\,m$; slope 0.8 | 현재 ego와 첫 ahead waypoint 사이 seam | `path entry is discontinuous from the current ego d`, `kGeometry` | fresh candidate, P3 suffix, P0-named commitment에 공통. 단 F5 hold만 `skip_entry_continuity=true`가 가능하나 기본 `hold_min_clear_time_sec=0.0`이라 꺼져 있다 |
| 2 | forward/path-size gate | `start_index >= N`이면 reject. `minimum_points==0`이면 YAML `minimum_path_points`; $N-start\ge N_{min}$ 요구 | fresh P3 8점; `validatePath()` continuation은 명시적으로 1점; safe-stop validator는 2점 계약을 사용하고 짧은 prefix는 보통 8점으로 densify | path container/remaining suffix | `path has no waypoint ahead of ego`, `path does not meet minimum_path_points`, `kNoForwardPath` | caller별 하한이 다름. P3 reconstruction 자체도 8점 미만이면 validator 전에 `spline segment has too few global race-line samples` |
| 3 | center track bound | 각 점에 대해 $-R_i+w \le d_i \le L_i-w$. $w=\text{trackBoundaryReserve}=0.04\,m$; 폭이 0.05 이하이면 fallback 1.50 m | wall margin 0.04 m | candidate center at every waypoint | `d-offset leaves the global waypoint track bounds`, `kTrackBoundary` | 공통 hard reject |
| 4 | rotated rectangular footprint | [`measureFootprintTrackBound()`](../../src/local_planning/src/raceline_spline_planner.cpp#L2192). $0.56\times0.30\,m$ 직사각형의 네 corner $p_c=p_i+R(\psi_i)[\pm0.28,\pm0.15]^T$를 인근 reference segment에 투영. 각 corner에서 $c=\min(L-w-d_c,d_c+R-w)$. $\min c<-10^{-9}$이면 reject | length 0.56 m, half-width 0.15 m, wall margin 0.04 m | 회전한 차체 네 corner, local branch-limited projection | `footprint_track_bound`, `kTrackBoundary`; side/heading/protrusion도 failure struct에 기록 | 공통 hard reject. center bound를 통과해도 footprint로 실패 가능 |
| 5 | obstacle collision | 각 waypoint와 담당 obstacle에 대해 $d_R^{test}=d_R^{input}-c_{obs}$, $d_L^{test}=d_L^{input}+c_{obs}$. $f_i\in[s_{start},s_{end}]$이고 $d_i>d_R^{test}+10^{-6}\land d_i<d_L^{test}-10^{-6}$이면 reject | $c_{obs}=0.15+0.08+1.0\times0=0.23\,m$. longitudinal padding 0.0 m | waypoint center 대 input obstacle envelope. TEST_ACTIVE fresh는 conservative guard와 raw geometry를 둘 다 exact-check | `d-offset intersects an inflated static-obstacle box`, `kObstacleCollision`; obstacle id, tested faces, waypoint s/d 기록 | 공통. 단 obstacle 검사만 maneuver collision horizon으로 잘릴 수 있고 track/geometry는 전체 path를 검사 |
| 6 | increasing race-line order | $i>start$에서 $\Delta f_i=f_i-f_{i-1}>10^{-9}$ | epsilon $10^{-9}$ | adjacent path samples | `candidate no longer follows increasing global race-line order`, `kGeometry` | 공통 hard reject |
| 7 | Frenet lateral slope | $|d_i-d_{i-1}|/\Delta f_i \le 0.8$ | 0.8 | adjacent samples | `quintic d-offset exceeds maximum_lateral_slope`, `kGeometry` | 공통 hard reject. reason의 “quintic”은 현재 P3-only 현실보다 오래된 이름 |
| 8 | spatial curvature rate | $|\kappa_i-\kappa_{i-1}|/\Delta f_i \le20\,rad/m^2$ | 20.0 rad/m² | adjacent samples | `shifted race line exceeds maximum_curvature_rate_radpm2`, `kGeometry` | 공통 hard reject. 시간축 steering-rate와 다름 |
| 9 | signed curvature / steering angle | $|\kappa_i|\le \min(\kappa_{legacy},\tan\delta_{side}/L)$. $\kappa\ge0$은 left limit, $\kappa<0$은 right limit | left $\min(1.316266519,\tan0.410/0.33)=1.316266519\,m^{-1}$; right $\min(1.316266519,\tan0.361/0.33)=1.144075639\,m^{-1}$ | every waypoint after recomputed geometry | `shifted race line exceeds left control steering curvature` 또는 `... right ...`, `kGeometry` | 공통 hard reject. understeer $K_{us}$는 이 hard limit에 들어가지 않음 |

`[CURRENT IMPLEMENTATION]` 이 순서는 [`validateCandidate()` 본문](../../src/local_planning/src/raceline_spline_planner.cpp#L2805)의 실제 early-return 순서다. 예를 들어 같은 waypoint에서 obstacle collision이 먼저 검출되면 그 점 이후의 order/slope/curvature-rate/signed-curvature 검사는 실행되지 않는다.

### 1.3 Generator gate, speed shaping, ranking, diagnostic 구분

| 요구 항목 | 실제 분류 | source 기준 동작 |
|---|---|---|
| maneuver length / transition constraint | **GENERATOR GATE**, 일부 **NOT IMPLEMENTED** | M1은 station 5개의 모든 segment가 양수인지 `strictPositiveSegments()`로 candidate 생성 전에 검사하며 실패 시 `m1_positive_segment_rejection_count`, `BOUNDARY_HANDOFF_UNRESOLVED`로 남긴다. M0-V1/V2의 non-positive C2 segment는 예외를 evaluator 경계에서 fail-closed 처리한다. 반면 생성된 path의 총 maneuver length를 validator가 별도 상·하한으로 reject하거나 rank하는 항목은 없다. [`addM1Candidate()`](../../src/local_planning/src/p3_shadow.cpp#L1735) |
| maximum exit length | **NOT ACTIVE** | 식을 적용하는 helper는 있으나 YAML `maximum_exit_length_m: 0.0`이므로 비활성이다. path length ranking key도 아니다. [`cappedCombinedExitScale()`](../../src/local_planning/src/raceline_spline_planner.cpp#L537) |
| lateral acceleration | **SPEED SHAPING** | `limitedAvoidanceSpeed()`가 보간된 $a_{lat,max}(v)$에 대해 $v^2|\kappa|\le a_{lat,max}(v)$인 최대 속도를 60회 bisection으로 찾는다. candidate를 reject하지 않는다. [`limitedAvoidanceSpeed()`](../../src/local_planning/src/raceline_spline_planner.cpp#L416) |
| longitudinal acceleration | **SPEED SHAPING** | forward pass: $v_i\leftarrow\min(v_i,\sqrt{v_{i-1}^2+2a_{max}(v_{i-1})\Delta s})$. seed는 $\max(v_{launch-floor},v_{ego})$. [`applyLongitudinalFeasibility()`](../../src/local_planning/src/raceline_spline_planner.cpp#L2590) |
| longitudinal deceleration | **SPEED SHAPING** | backward pass: $v_{i-1}\leftarrow\min(v_{i-1},\sqrt{v_i^2+2a_{dec,max}(\max(v_i,v_{i-1}))\Delta s})$. hard reject가 아니다. 같은 값을 publish 전 diagnostic이 다시 점검한다. |
| response delay / braking distance | **SPEED SHAPING + RANKING ONLY** | approach ramp는 command-hold 시작을 $v_{ego}\times0.15\,s$만큼 앞당긴다. rank metric은 $D_{def}=\max_i[0,v_e\tau+(v_e^2-v_i^2)/(2a_{dec})-f_i]$. 양수여도 hard geometry를 버리지 않고 rank에서 뒤로 보낸다. [`measureCandidate()`](../../src/local_planning/src/raceline_spline_planner.cpp#L2116) |
| steering angle | **HARD REJECT** | 위 signed-curvature 행의 simple bicycle limit로 검사한다. |
| temporal steering rate | **DIAGNOSTIC ONLY** | publish 직전 $\delta_i=\operatorname{clamp}(\kappa_i(L+K_{us,side}v_i^2),-\delta_R,\delta_L)$, $|\Delta\delta|v_{avg}/\Delta s>20\,rad/s$를 센다. 선택/발행을 막지 않는다. [`inspectVelocityFeasibility()`](../../src/local_planning/src/raceline_spline_planner.cpp#L1587) |
| speed limiter 전체 | **SPEED SHAPING** | source 순서는 pointwise curvature cap → gap-reserve cap → confirmed-cluster critical-speed hold → approach ramp → longitudinal forward/backward pass이다. [`applyAvoidanceVelocityLimit()`](../../src/local_planning/src/raceline_spline_planner.cpp#L2344) |
| gap speed limiter | 현재 설정에서는 사실상 **identity** | reserve mode `none`과 localization 0 때문에 tracking reserve가 0이다. 따라서 gap inversion은 속도를 내리지 않는다. 장애물 pass/fail은 0.23 m clearance에 대한 이진 판정이 된다. [`trackingErrorReserve()`](../../src/local_planning/src/raceline_spline_planner.cpp#L496), [`test_obstacle_reserve_gate.cpp`](../../src/local_planning/test/test_obstacle_reserve_gate.cpp#L60) |
| safe-stop compatibility | **LIFECYCLE/FALLBACK**, candidate rank key 아님 | 회피 전멸 후 margin-only slow pass 또는 `buildSafeStop()`으로 넘어간다. stop geometry도 별도 common validation을 받지만 “이 avoidance candidate가 safe-stop compatible한가”라는 rank/hard key는 없다. 정지점에서 다시 회피 가능한지는 escape check가 별도로 묻는다. |
| tire friction ellipse, time-indexed dynamics, continuous swept-volume collision | **NOT IMPLEMENTED** | lateral/longitudinal 축은 서로 독립적으로 speed shape하며 combined friction ellipse는 없다. obstacle collision은 discrete waypoint center test이고 차량의 obstacle swept footprint 연속 교차 검사는 없다. rotated rectangle은 track wall 전용이다. |

### 1.4 공통 적용 여부의 정확한 의미

`[CURRENT IMPLEMENTATION]` “P0/P2/P3 모두 동일”이라는 문장은 현재 구조에 그대로 적용할 수 없다.

- P3/M0/M1이 만든 path, P3 frozen suffix, P0-named safety commitment, safe-stop prefix는 **같은 validator 구현**을 재사용한다.
- 하지만 caller별로 `minimum_points`, obstacle geometry(raw/guard), obstacle reserve scale(1.0 또는 retention 0.5), collision horizon, entry-continuity 면제가 다르므로 동일한 path가 모든 lifecycle 지점에서 무조건 같은 verdict를 받는 것은 아니다.
- P2 path는 존재하지 않아 “P2에도 적용”을 검증할 대상이 없다.

---

## 2. Validation order

### 2.1 fresh P3 candidate 한 개의 실제 순서

```text
analytic tuple (side, d_target, d_mid/root, entry scale, exit scale)
  → station[0..4]와 C2 piecewise profile 생성
    [non-positive segment/invalid station이면 candidate 생성 전 fail-closed]
  → ordered global reference samples 선택
  → d(s) 평가
  → x = x_ref - d sin(psi_ref), y = y_ref + d cos(psi_ref)
  → waypoint 수 < 8 ?
      YES: "spline segment has too few global race-line samples" [EARLY EXIT]
      NO:
        → updateGeometry(): 최종 Cartesian path의 psi/kappa 재계산
        → applyAvoidanceVelocityLimit(): curvature/gap/confirmed/approach/longitudinal speed shaping
        → updateAccelerationOnly()
        → measureCandidate(): rank/audit metrics 계산
        → validateCandidate():
            1 entry continuity                         [fail → return]
            2 start/point-count                        [fail → return]
            for each waypoint, path order대로:
              3 center track bound                     [fail → return]
              4 rotated rectangular footprint          [fail → return]
              5 obstacle collision within horizon      [fail → return]
              6 increasing s, if not first             [fail → return]
              7 lateral slope, if not first             [fail → return]
              8 spatial curvature-rate, if not first    [fail → return]
              9 signed curvature/steering geometry      [fail → return]
        → hard_valid=true
```

근거는 [`buildCandidate()`](../../src/local_planning/src/p3_shadow.cpp#L1045), [`finalizeP3ShadowPath()`](../../src/local_planning/src/raceline_spline_planner.cpp#L1147), [`validateCandidate()`](../../src/local_planning/src/raceline_spline_planner.cpp#L2805)이다.

### 2.2 그 뒤의 재검증

`[CURRENT IMPLEMENTATION]` candidate 내부 검증을 통과한 것만으로 publish authority가 생기지 않는다.

1. `plan()` safety route는 returned trace마다 metric과 hard verdict를 다시 계산한다.
2. TEST_ACTIVE fresh lifecycle은 conservative/guard certificate를 확인한 뒤 **current raw geometry**에 대해 다시 exact-check한다.
3. 다음 frame부터는 original path를 새로 생성하지 않고 현재 ego 앞 suffix만 잘라 current guard/raw geometry로 재검증한다.
4. publish 직전에는 velocity/steering-rate diagnostic을 계산하지만 위반해도 더 낮은 profile과 검증된 geometry를 그대로 발행한다. [`publishResult()`](../../src/local_planning/src/local_planner_node.cpp#L4182)

---

## 3. Candidate Ranking

### 3.1 comparator의 완전한 lexicographic key

`[CURRENT IMPLEMENTATION]` weighted sum이 아니다. [`betterCandidateRank()`](../../src/local_planning/include/local_planning/candidate_rank.hpp#L75)의 다음 lexicographic 순서를 `P3ShadowEvaluator`와 `plan()`이 공유한다. 비교 epsilon은 모든 실수 key에 (10^{-9})이다.

| 우선순위 | key | 좋은 방향 | 의미 |
|---:|---|---|---|
| 1 | `exit_reaches_next_obstacle` | `false` 우선 | 현재 cluster 뒤 exit ramp가 다음 non-cluster obstacle physical envelope에 닿으면 최우선 강등. hard reject는 아님 |
| 2 | braking feasible | `deficit <= 1e-9` 우선 | 현재 ego speed에서 candidate speed seam까지 반응지연+제동거리가 충분한 후보 우선 |
| 3 | `ego_braking_distance_deficit_m` | 작을수록 좋음 | 모두 늦었을 때 부족 거리의 최댓값이 작은 후보 |
| 4 | `velocity_loss` | 작을수록 좋음 | 각 점의 (max(0,v_{ref}-v_i)/v_{ref}) 평균. hard-valid 후에는 extra slack보다 속도를 먼저 본다 |
| 5 | `minimum_normalized_safety_slack` | 클수록 좋음 | 아래 네 normalized slack의 최소값 |
| 6 | `global_path_deviation_m` | 작을수록 좋음 | path 전 점의 (|d_i|) 평균 |
| 7 | `tiebreak_index` | 작을수록 좋음 | generation order. 완전 동률의 결정적 tie-break |

`std::stable_sort`를 사용하고 최종 generation index까지 비교하므로 동일 입력·동일 floating-point 실행에서는 deterministic하다. strict-weak-ordering과 epsilon 경계는 [`test_candidate_rank.cpp`](../../src/local_planning/test/test_candidate_rank.cpp#L49)가 고정한다.

### 3.2 metric 출처

모든 주요 rank metric은 [`measureCandidate()`](../../src/local_planning/src/raceline_spline_planner.cpp#L2029)에서 계산한다.

- **wall clearance**: rank용 wall room은 centerline가 아니라 `wall margin 0.04 + vehicle half-width 0.15 + avoidance tracking reserve 0`을 뺀 body-reference room이다.
- **obstacle clearance**: input obstacle face에 0.23 m를 지출한 뒤 candidate center까지의 signed gap이다. obstacle band 내부이면 음수다.
- **curvature slack**: $(\kappa_{max,side}-|\kappa_i|)/\kappa_{max,side}$의 최솟값.
- **curvature-rate slack**: $(20-\max|\Delta\kappa/\Delta s|)/20$.
- **minimum normalized safety slack**: `min(body-wall/1.5, obstacle/1.5, curvature slack, curvature-rate slack)`.
- **velocity loss**: normalized positive speed loss의 waypoint 평균.
- **deviation**: (|d|)의 waypoint 평균.
- **braking deficit**: response delay 0.15 s를 포함한 최대 부족 거리.

중요한 제외 항목은 다음과 같다.

- rotated footprint clearance는 **hard gate와 audit**에는 있으나 minimum safety slack에 들어가지 않는다.
- lateral slope는 hard gate와 P3 trace의 peak diagnostic에는 있으나 rank key와 minimum slack에 들어가지 않는다.
- path/maneuver length, temporal steering rate, total runtime은 rank key가 아니다.
- avoidance side 선호 cost는 없다. 다만 side-lock은 generator output을 filter하고, 최종 완전 동률은 생성 순서로 갈리므로 순서에 따른 간접 효과는 있다. M0에서 양쪽 모두 invalid이면 canonical record는 RIGHT-first이다. 이는 명시적 “right-side cost”가 아니라 control-flow/tie 결과다.
- `rank_without_exit_demotion`은 감사용 가상 순위이며 실제 선택에는 사용하지 않는다. [`plan()` audit order](../../src/local_planning/src/raceline_spline_planner.cpp#L3522)

---

## 4. Generator vs Validator vs Ranker 분리

### A. Candidate generator

`[CURRENT IMPLEMENTATION]` P3 generator는 obstacle/track에서 side domain과 connected corridor branch를 만들고, M0-V1 → M0-V2 → M1 순으로 `(side, d_target, d_mid, entry, exit)` tuple을 제안한다. tuple마다 C2 `d(s)`와 Cartesian path를 만들고 speed shaping까지 수행한다. M0/M1 자체는 P3 내부 template 이름이지 독립 planner stage가 아니다.

### B. Hard validator

생성된 discrete path를 현재 ego, track, guarded/raw obstacles에 대조한다. “좋은 root를 찾는” 역할도, 후보 사이를 고르는 역할도 없다. 첫 hard failure에서 즉시 종료한다.

### C. Candidate ranker

hard-valid candidate만 위 7-key comparator로 정렬한다. geometry를 수정하거나 invalid candidate를 되살리지 않는다.

### 현재 구분 가능한 실패

| 질문 | 현재 내부 source 상태 | 현재 외부 diagnostic 상태 |
|---|---|---|
| root/candidate가 전혀 생성되지 않았는가 | raw/finite/branch/bounded/accepted root count, M0/M1 candidate count, positive-segment count, `NO_ALGEBRAIC_ROOT`, `ROOT_BRANCH_MISMATCH`, `ROOT_LATERAL_BOUND`, `BOUNDARY_HANDOFF_UNRESOLVED`가 있어 상당 부분 구분 가능 | P3 cycle JSON은 root count와 positive-segment count를 내보내지 않아 불완전 |
| 생성됐지만 validator에서 죽었는가 | candidate trace별 `hard_valid`, exact reason, failure obstacle/waypoint가 있음 | recovery 실패 cycle에만 candidate 배열을 내보낸다. 성공 cycle의 탈락 후보는 P3 topic에서 생략 |
| valid 후보 중 무엇이 선택됐는가 | 공통 comparator와 trace metrics로 정확히 알 수 있음 | legacy `PLAN_CANDIDATE`는 replay diagnostics를 켜면 rank/selected를 내보내지만 기본은 off이고, P3 success cycle의 full candidate rank table은 없음 |
| generator family 밖에 실제 feasible P3 곡선이 있었는가 | 알 수 없음 | `[NEEDS INSTRUMENTATION]` 더 넓은 oracle/baseline family와 동일 validator의 paired evaluation 필요 |

`[CURRENT IMPLEMENTATION]` 특히 counter 이름에 주의해야 한다. M0-V1은 right와 left를 각각 최대 16개까지 실제 구성/검증한 뒤 canonical side record 하나만 보존한다. 보존된 record에 대해 V2/M1을 합친 공개 budget은 24이다. 따라서:

- `result.candidate_count <= 24`, `result.hard_validator_call_count <= 24`는 **보존된 canonical record 기준 counter**이다.
- 한 strict `P3ShadowEvaluator::run()`에서 실제 실행 가능한 M0-V1 검증은 양쪽 합계 최대 32이고, invalid canonical baseline 뒤 V2/M1이 남은 budget을 쓰면 총 실제 scheduled validation은 최대 (32+(24-16)=40)이 될 수 있다.
- strict 실패 후 relaxed run을 한 번 더 할 수 있으므로 한 `evaluateP3Shadow()`의 실제 내부 scheduled validation은 이론상 최대 80이다. 그 뒤 `plan()` route는 반환된 최대 24 trace를 다시 검증할 수 있다.
- candidate path가 8점 미만이면 counter는 증가했어도 `validateCandidate()` 본문은 호출되지 않는다.
- 그러므로 **callback 전체의 실제 validator cap은 24가 아니며**, safe-stop escape가 evaluator를 반복 호출할 수도 있다. 현재 callback-global validator counter/cap은 없다.

이 차이는 [`run()`의 canonical record 선택](../../src/local_planning/src/p3_shadow.cpp#L121), [상수 cap](../../src/local_planning/src/p3_shadow.cpp#L310), [M0-V1 per-side loop](../../src/local_planning/src/p3_shadow.cpp#L1941), [strict/relaxed 재시도](../../src/local_planning/src/raceline_spline_planner.cpp#L2183)에서 직접 확인된다.

---

## 5. Planner Lifecycle

### 5.1 callback/event 전체 흐름

```text
/confirmed_static_obs ingress
  → latest ingress buffer
  → 25 ms planning timer에서 drain
  → reference / Frenet odom / obstacle source readiness와 freshness 확인
  → p3_mode 분기

TEST_ACTIVE (기본)
  → active P3 record가 있으면 continuation-first
      → current suffix hard-valid: evaluator를 호출하지 않고 P3_COMMITTED_SUFFIX publish
      → complete/invalid: 같은 callback에서 fresh evaluation 가능
  → fresh P3 evaluation
      → M0-V1
          success: rank-selected fresh P3
          failure → M0-V2
              success: rank-selected fresh P3
              failure → M1 with remaining retained budget
  → guarded certificate + raw exact validation
      success: immutable lifecycle record 생성, P3 publish
      failure/no blocking cluster/not ready:
        → hold/completion handoff gates
        → runSafetyPlanningCycle (owner label P0_BACKUP_ONLY)
            → commitment/latch/handoff 처리
            → planner.plan() [별도 P0 generator가 아니라 같은 P3 generator 재사용]
            → valid avoidance / margin slow pass / safe stop / empty-or-global-handoff
  → publishResult(): raw slowdown overlay + diagnostic-only feasibility check + /avoid_waypoints
```

### 5.2 mode와 production 우선순위

| mode | 진입/우선순위 | publish authority |
|---|---|---|
| `OFF` | P3 lifecycle snapshot을 만들지 않고 곧바로 `runSafetyPlanningCycle()` | safety flow. 단 그 안의 `planner.plan()` 후보 생성기는 여전히 P3 analytic generator |
| `SHADOW` | safety flow를 먼저 실행/발행한 뒤 동일 snapshot으로 P3를 관측 평가 | P0-named safety flow만 command authority; P3는 diagnostic only |
| `TEST_ACTIVE` 기본 | active suffix continuation → fresh P3 M0/M1 → hold/handoff → safety backup | exact-current-hard-valid P3 path/suffix가 최우선. 없을 때만 `P0_BACKUP_ONLY` label의 safety flow |

따라서 `[CURRENT IMPLEMENTATION]` production 우선순위는 **P3 lifecycle first, safety layer backup**이다. P2는 없고, P0 candidate family도 없다.

### 5.3 단계별 조건과 budget

| 단계 | 진입 | 성공 | 실패/다음 단계 | 이전 state | retained candidate budget |
|---|---|---|---|---|---:|
| perception accept | timer가 latest ingress를 drain | frame/geometry가 유효하면 `static_obstacles_`, sequence, source stamp 갱신 | invalid/out-of-order는 drop; 큰 stamp regression은 epoch와 lifecycle reset | 마지막 accepted snapshot을 stale 동안 유지 | 해당 없음 |
| initial selection guard | active P3가 없고 blocking cluster 존재 | 동일 ID count 3 + 최소 0.15 s, 또는 max wait 0.35 s에 ready. 그 전에도 conservative union과 raw를 모두 hard-valid하면 fresh ownership 가능 | 관측 union을 누적 | ID별 envelope/count/time | 해당 없음 |
| P3 continuation | immutable record active | current suffix가 guard/raw/retention 계약을 통과 | epoch/reference/stamp/identity/progress/geometry failure면 invalidate; cluster end 통과면 complete | original path, guards, side, IDs, lineage, progress index | 새 후보 0 |
| M0-V1 | fresh evaluator, valid side domain | 각 side 내부 best, 양쪽 best 중 global comparator best | canonical record에 valid가 없으면 V2 | frame 간 history 없음 | side당 16; 반환 record는 한 side만 |
| M0-V2 | M0-V1 canonical record가 hard-valid 없음 | zero-interface/branch-aware/bisected template에서 best | 없으면 M1 | 없음 | 자체 최대 12, 동시에 `24 - retained M0-V1 count` 이하 |
| M1 | V1/V2가 모두 실패 | context round-robin: `ZERO_BOUNDARY_SHORT`, `ZERO_BOUNDARY_SPAN`, `NEAR_LONG`, `FAR_SPAN`, 필요 시 `NEAR_BISECTED`; best rank | 없으면 failure classification → backup | 없음 | `24 - retained M0 count` |
| safety commitment | P3 authority가 없거나 OFF/SHADOW safety owner | current committed path exact-valid이면 그대로 publish; 아니면 same P3 generator로 replacement plan | avoidance 없음 → margin-only slow pass 또는 safe stop | committed path, side lock, guards, completion/handoff state | 한 `P3ShadowResult` 반환 cap 24; callback-global cap 없음 |
| safe-stop latch | no safe avoidance 또는 explicit stop branch | stop path publish; 매 cycle escape replan | 아래 해제 조건 중 하나가 성립할 때만 avoidance/global handoff | latched IDs, danger interval, stop target, counts | escape check가 P3 generator 반복 가능 |
| publish | avoidance/preparation/safe-stop/handoff result가 존재 | raw slowdown min-overlay 후 `/avoid_waypoints` publish | clear GLOBAL이면 empty publish; AVOID이면 closed global handoff로 FSM 복귀를 돕는다 | last valid guidance, last side, owner/digest diagnostic | 해당 없음 |

### 5.4 safe-stop release

[`SafeStopLifecycle::evaluate()`](../../src/local_planning/src/safe_stop_lifecycle.cpp#L80)은 다음 우선순위로 해제한다.

1. activation 이후 ego가 기억 danger end를 `safe_stop_buffer_m=2.60 m`까지 통과.
2. latched obstacle을 담당하는 hard-valid avoidance가 8회 연속이고 현재 FSM이 AVOID를 선택 가능.
3. 차량 정지 + explicit corridor clear 또는, 기억 danger end를 이미 지난 뒤 latch 이후 fresh empty frame을 8회 관측.
4. 차량 정지 + danger가 아직 앞인데 fresh empty frame만 4.0 s 지속하면 blind-timeout으로 0.7 m/s creep handoff.

빈 obstacle array 하나는 clear 증거가 아니다. sequence freshness와 기억된 danger interval을 함께 본다. 이 계약은 [`test_safe_stop_lifecycle.cpp`](../../src/local_planning/test/test_safe_stop_lifecycle.cpp#L48)가 회귀 시험한다.

### 5.5 normal raceline/default path

`[CURRENT IMPLEMENTATION]` local planner는 clear GLOBAL 주행 때 정상 race line을 복제해 `/avoid_waypoints`로 계속 발행하지 않는다. `kNoObstacle`이고 FSM도 GLOBAL이면 empty를 발행해 downstream이 global source를 쓰게 한다. 반면 이미 AVOID이면 non-empty closed global handoff loop를 발행하여 FSM이 GLOBAL 복귀를 확인할 수 있게 한다. [`runSafetyPlanningCycle()`](../../src/local_planning/src/local_planner_node.cpp#L4120)

---

## 6. Temporal state

| 이전 frame 정보 | 실제 보존 | 다음 path selection에 영향 | 판정 |
|---|---|---|---|
| previous avoidance side | P3 record의 `go_left`; safety commitment의 `committed_result_.go_left` | P3 active 동안 immutable. safety flow는 $|d_{ego}|\ge0.10\,m$ 또는 진행 0.50 m 뒤 side lock, 그 전 한 번 switch 가능 | **selection/lifecycle state** |
| previous selected path | P3 `original_path` immutable record; safety `committed_result_`; `last_valid_guidance_result_` | current suffix revalidation, commitment hold, hold-recovery, stop geometry에 직접 사용 | **강한 temporal state** |
| previous `d_target` | safety result에는 보존. P3 record는 scalar를 별도 저장하지 않고 path geometry가 암묵적으로 포함 | fresh P3 rank의 hysteresis cost로는 사용하지 않음 | **부분 state** |
| previous `d_mid` | active record에 scalar 없음 | 다음 fresh selection에 영향 없음 | **diagnostic/current-result only** |
| previous `s_probe/d_probe` | lifecycle record에 없음 | 다음 selection에 영향 없음 | **없음** |
| hysteresis | generic cost hysteresis는 없음 | immutable suffix continuation, guard retention 0.5, side lock, observation stabilization, safe-stop latch, completion defer가 실질 hysteresis | **명시적 lifecycle hysteresis** |
| commitment | P3 lifecycle record와 safety commitment가 별도 존재 | 새 candidate를 매 frame 재선택하지 않게 함 | **핵심 state** |
| hold time | initial 0.15/0.35 s, completion defer 3.0 s, blind release 4.0 s; F5 0.0으로 off | state transition에 영향 | **timer/counter state** |
| obstacle tracking state | planner는 association을 만들지 않고 detector ID를 소비. latest accepted snapshot, ID별 conservative union/guard, face window, completed IDs, obstacle rear memory를 보존 | guard, lineage, handoff/hold, replan 대상에 영향 | **ID-dependent state** |
| candidate identity/history | selected identity, logical identity, path digest, source lineage를 P3 record에 저장 | provenance와 동일 maneuver 유지에 사용. 과거 candidate 성능/rank를 누적해 새 candidate score를 바꾸지는 않음 | **lineage state, 학습 history 아님** |

`[CURRENT IMPLEMENTATION]` P3 continuation은 “이전 candidate에 작은 bonus”를 주는 방식이 아니다. valid한 동안 **이전 original geometry 자체**를 authority로 유지하고 current suffix만 재검증한다. 이것이 frame-to-frame 안정성의 주된 구조다.

---

## 7. Effective production parameters

### 7.1 기본 launch 기준 실제 값

| 분류 | key | 실제 값 / 효과 |
|---|---|---|
| vehicle | `vehicle_length_m`, `vehicle_half_width_m` | 0.56 m, 0.15 m; physical rectangle $0.56\times0.30\,m$ |
| obstacle margin | `safety_margin_m` | 0.08 m |
| reserve | `obstacle_reserve_mode`, `localization_reserve_m` | `none`, 0.0 m; LUT와 fallback reserve는 사용 안 함 |
| effective obstacle clearance | source formula | (0.15+0.08=0.23\,m), input obstacle face에서 차량 중심까지 |
| wall | `wall_safety_margin_m` | 0.04 m. footprint hard check에서 한 번 적용 |
| curvature | `maximum_curvature_radpm` | 1.316266519079011 m⁻¹ legacy cap |
| steering | wheelbase/left/right | 0.33 m / 0.410 rad / 0.361 rad → hard signed (+1.316266519/-1.144075639\,m^{-1}) |
| understeer/rate | left/right (K_{us}), max rate | 0.014/0.019 rad/(m/s²), 20 rad/s; publish diagnostic model에 사용 |
| curvature rate | `maximum_curvature_rate_radpm2` | 20.0 rad/m² hard |
| lateral slope | `maximum_lateral_slope` | 0.8 hard |
| entry seam | min budget / baseline | 0.20 m / 0.50 m |
| lateral acceleration table | speed 0..9 m/s | `[7.6,7.6,7.6,7.6,7.0,7.0,7.0,6.5,6.5,6.5]` m/s² |
| longitudinal acceleration table | speed 0..9 m/s | `[3.7,3.7,3.7,3.7,3.7,3.47,3.33,3.0,3.0,3.0]` m/s² |
| longitudinal deceleration table | speed 0..9 m/s | 모두 2.0 m/s² |
| response delay | `confirmed_speed_response_delay_sec` | 0.15 s |
| speed envelope | enable/post-hold/min speed | true / 1.0 m / 1.0 m/s |
| station geometry | pre distances | `[11.442220427651225, 7.628146951767484, 3.814073475883742]` m |
| station geometry | post distances | `[2.059509950005119, 4.119019900010238, 6.178529850015357]` m |
| transition | entry fractions | `[0.5145810930150512, 0.75, 1.0]` |
| transition | exit scales / outside multiplier | `[0.4971684162574945,0.6991537701867223,3.698773101198193]` / 0.4060036444074003 |
| exit/tail | maximum exit / post-merge | 0.0 m(off) / max(5.0 m, ego speed × 1.0 s) |
| path | `minimum_path_points` | 8 fresh; wrapper-specific exception은 앞 절 참조 |
| P3 retained caps | source constants | V1 side당 16, V2 최대 12, returned M0+M1 24 |
| lifecycle | retention / observation | 0.5 / 3 messages, min 0.15 s, max 0.35 s |
| safe stop | buffer/decel/release | 2.60 m / 1.8 m/s² / 8 cycles; escape check true, step 0.30 m, max retreats 8 |
| mode | `p3_mode` | YAML와 두 launch default 모두 `TEST_ACTIVE` |
| diagnostic | P3 detail / replay | `EVENTS`; replay diagnostics false |

값은 [`local_planning.yaml`](../../src/local_planning/config/local_planning.yaml#L1), [velocity table](../../src/local_planning/config/local_planning.yaml#L210), [geometry/control values](../../src/local_planning/config/local_planning.yaml#L500), [launch override](../../src/local_planning/launch/local_planning.launch.py#L142)에서 확인했다. longitudinal table/CSV 동기화는 [`test_velocity_limits_match_csv.py`](../../src/local_planning/test/test_velocity_limits_match_csv.py#L106), steering/control 계약은 [`test_control_contract_match.py`](../../src/local_planning/test/test_control_contract_match.py#L111)가 검사한다.

### 7.2 주석과 actual mismatch

| 위치 | 주석 | actual source/YAML | 결론 |
|---|---|---|---|
| YAML 18–19, 29–32 | safety margin 0.00, clearance 0.15라고 서술 | `safety_margin_m: 0.08`, effective clearance 0.23 | 주석 stale. actual 0.23 사용 |
| YAML 274–275 | physical clearance 0.158이라고 서술 | actual 0.23 | stale historical comment |
| YAML 607–608 | longitudinal padding 0.4149925라고 서술 | line 13 actual 0.0 | stop nominal distance 해석에서 주석을 사용하면 안 됨 |
| YAML 745–746 | soft/hard clearance 0.2982893이라고 서술 | reserve off, localization 0, base 0.23 | stale comment |

`[NEEDS INSTRUMENTATION]` launch/YAML의 정적 합성은 위와 같지만 실제 실행 중 command-line override, parameter service 변경, 다른 params file 사용까지 증명하려면 run 시작 시 전체 effective parameter snapshot 또는 `ros2 param dump`를 artifact에 저장해야 한다.

---

## 8. Research instrumentation recommendation

알고리즘을 바꾸지 않고 research branch에 추가할 최소 단위는 **planning event 1개 + 그 event에 속한 candidate row N개**이다. 공통 join key는 `(run_id, callback_sequence, source_epoch, source_stamp_ns, obstacle_sequence, reference_generation)`로 한다.

### 8.1 이미 있는 것과 빠진 것

| 연구 필드 | 현재 존재 | 외부 artifact에서의 한계 / 새로 필요한 것 |
|---|---|---|
| scenario/timestamp/lineage | P3 cycle에 callback, ROS time, source stamp/epoch, obstacle sequence, reference generation | `scenario_id`, run/config SHA를 추가하고 모든 stream에 같은 join key 필요 |
| generator stage/template | trace에 M0-V1/V2/M1 source/template/cell/branch | success cycle P3 JSON에 candidate 배열이 없고 `PLAN_CANDIDATE`에는 stage가 없음. 항상 emit 필요 |
| candidate identity | trace와 lifecycle에 identity/logical identity/digest | success candidate 전체 identity table 필요 |
| `d_target`, `d_mid/root` | trace 내부, failure cycle candidate JSON | success cycle 포함 항상 emit; root index/active-vs-inactive filtering 단계 추가 |
| `s_probe`, `d_probe` | result에 selected fields 두 개 | **per-candidate trace에는 없음**. M1 `addM1Candidate()`는 `Probe*`를 버리고, result의 selected probe는 M0 canonical baseline에서 한 번만 채워져 M1 selected probe를 대표한다고 보장할 수 없음. per-candidate로 저장해야 함 |
| candidate/validator count | result에 stage counts | counter는 discarded M0 side를 제외하고 8점 미만 path도 “validator call”로 센다. `constructed_count`, `validateCandidate_executed_count`, `discarded_side_count`, strict/relaxed attempt를 분리 |
| rejection reason/location | trace failure reason, obstacle id/waypoint/footprint details | success cycle 탈락 후보에도 항상 emit; failure kind enum과 first-failure order index 추가 |
| track/obstacle margin | trace와 `PLAN_CANDIDATE` | guarded/raw/retention 어느 geometry/scale에서 측정했는지 authority label 추가 |
| curvature/rate/slope margin | selected result와 일부 trace | candidate row마다 `limit - peak`; signed side limit; slope peak/limit를 항상 emit |
| speed loss/braking deficit/ranking keys | trace/plan audit에 대부분 존재 | exact 7-key tuple, comparator epsilon, final rank를 success P3 candidate마다 저장 |
| selected candidate | selected identity/digest 및 plan audit flag | fresh/continued/backup/hold/safe-stop owner와 같은 event row에서 join |
| stage runtime | result에 total/corridor/root/reconstruction/hard-validation | P3 JSON은 total만 emit하고 discarded side time이 stage sum에서 빠진다. strict/relaxed 및 side별 wall-clock을 별도 측정 |

### 8.2 권장 최소 schema

```text
PLANNING_EVENT:
  run/scenario/config_sha, callback lineage, ego, obstacle snapshot digest,
  mode, owner, lifecycle state/reason,
  strict_attempted, relaxed_attempted,
  constructed_total, validator_executed_total, hard_valid_total,
  selected_candidate_identity or NONE, fallback_kind,
  total/corridor/root/reconstruction/validation/ranking runtime

CANDIDATE_EVENT:
  planning_event_key, generator_stage, template, side, generation_order,
  candidate_identity, component/branch/root-regime/root-index,
  d_target, s_probe, d_probe, d_mid,
  entry/exit/stations, point_count,
  validation_authority, validator_executed, hard_valid,
  first_failure_kind/reason/waypoint/obstacle,
  track/obstacle/curvature/curvature-rate/slope margins,
  speed_loss, braking_deficit, deviation, exit_next,
  exact_rank_tuple, final_rank, selected
```

이 schema는 logging만 추가하고 generator/validator/ranker 결정을 바꾸지 않아야 한다. timer budget에 영향을 주지 않도록 bounded ring buffer 또는 별도 low-priority writer를 사용하되, 그것은 구현 단계에서 benchmark해야 한다.

---

## 9. 연구 관점 failure taxonomy

| taxonomy | 판정 초안 | 현재 판별 가능성 |
|---|---|---|
| `NO_CANDIDATE_GENERATED` | blocking cluster와 valid side/domain은 있으나 constructed candidate 0 | **부분 가능**. 내부 count/classification은 있으나 external P3 event에 root/segment 상세가 부족 |
| `MAPPING_MISSED_EXISTING_P3` | 현재 mapping은 후보를 못 냈으나 동일 scene에서 더 완전한 P3-family oracle가 hard-valid path를 냄 | `[NEEDS INSTRUMENTATION]` paired oracle와 동일 validator 필요 |
| `TRACK_BOUND_FAILURE` | first hard failure가 center bound 또는 `footprint_track_bound` | **가능**. footprint와 center를 하위 code로 분리 권장 |
| `OBSTACLE_COLLISION` | first failure kind `kObstacleCollision` | **가능**. guard/raw/retention/horizon authority를 반드시 함께 기록 |
| `CURVATURE_FAILURE` | first reason이 left/right control steering curvature | **가능** |
| `CURVATURE_RATE_FAILURE` | first reason이 max curvature-rate | **가능** |
| `SLOPE_FAILURE` | entry discontinuity 또는 internal max lateral slope | **가능하나 두 subcode로 분리 필요** |
| `MANEUVER_CONSTRAINT_FAILURE` | non-positive segment, boundary handoff unresolved, station/probe range, too-few samples | **부분 가능**. candidate 전 생성 gate의 exact tuple/reason emit 필요 |
| `SPEED/BRAKING_DIFFICULTY` | geometry hard-valid이지만 braking deficit >0 또는 publish feasibility violation | **부분 가능**. rank deficit은 있으나 outgoing profile의 모든 diagnostic을 candidate/event에 join해야 함 |
| `VALID_CANDIDATE_EXISTS_BUT_RANKING_SELECTED_OTHER` | hard-valid candidate가 2개 이상이고 특정 연구 oracle가 선호한 candidate와 production rank 1이 다름 | production rank/selected는 계산 가능. “other가 더 좋다”는 연구 objective 없이는 `[NEEDS INSTRUMENTATION]`/정의 불가 |
| `TEMPORAL_INSTABILITY` | 동일 obstacle lineage에서 identity/side/owner가 과도하게 변경되거나 fresh↔invalid↔safe-stop oscillation | lifecycle/digest 단편은 있음. `[NEEDS INSTRUMENTATION]` event join과 windowed churn metric 필요 |
| `P3_FAMILY_NO_FEASIBLE_SOLUTION` | 충분히 완전한 P3 family oracle도 동일 hard validator에서 모두 실패 | 현재 bounded template가 실패한 것만 알 수 있다. 물리적 infeasible과 family insufficiency 분리는 `[NEEDS INSTRUMENTATION]` |

`[INFERENCE]` 논문 실험에서는 `NO_CANDIDATE_GENERATED`를 root=0, mapping/filter 탈락, pre-construction maneuver gate로 다시 세분해야 generator coverage 개선과 validator 완화를 혼동하지 않을 수 있다.

---

## 관련 unit/regression test가 실제로 고정하는 계약

| 영역 | test evidence |
|---|---|
| footprint, 다른 hard constraint 보존, signed steering | [`test_raceline_spline.cpp`](../../src/local_planning/test/test_raceline_spline.cpp#L171), 같은 파일의 directional steering test |
| entry continuity AND와 baseline | [`PathDiscontinuousFromEgoIsRejected`](../../src/local_planning/test/test_raceline_spline.cpp#L576), [`EntrySlopeIsMeasuredOverABaselineTheCarCanActOn`](../../src/local_planning/test/test_raceline_spline.cpp#L699) |
| accel/decel speed shaping | 같은 파일의 `EveryDrop...`/`EveryRise...` tests, lines 840 이후 |
| exact lexicographic rank와 determinism | [`test_candidate_rank.cpp`](../../src/local_planning/test/test_candidate_rank.cpp#L49) |
| fresh raw revalidation, immutable suffix, lineage invalidation, retention | [`test_p3_maneuver_lifecycle.cpp`](../../src/local_planning/test/test_p3_maneuver_lifecycle.cpp#L148) |
| M0 fail-closed/cap, multiple layouts, unresolved corridor limitation | [`test_p3_production_parity.cpp`](../../src/local_planning/test/test_p3_production_parity.cpp#L106), [`P3NonpositiveEntryBoundary...`](../../src/local_planning/test/test_raceline_spline.cpp#L148) |
| safe-stop empty-frame/latch/release/debounce | [`test_safe_stop_lifecycle.cpp`](../../src/local_planning/test/test_safe_stop_lifecycle.cpp#L48) |
| YAML/control/CSV consistency | [`test_params_match_yaml.py`](../../src/local_planning/test/test_params_match_yaml.py#L78), [`test_velocity_limits_match_csv.py`](../../src/local_planning/test/test_velocity_limits_match_csv.py#L106), [`test_control_contract_match.py`](../../src/local_planning/test/test_control_contract_match.py#L111) |

`[CURRENT IMPLEMENTATION]` 이 테스트들은 중요한 회귀 계약을 제공하지만 runtime 실행 증거는 아니다. 이번 read-only audit에서는 실행하지 않았다.

---

## 10. 최종 요약

1. **hard validator 순서**: entry continuity → forward/point count → waypoint마다 center track bound → rotated footprint → obstacle collision → increasing s → lateral slope → spatial curvature-rate → signed curvature/steering. 모두 first-failure early exit이다.
2. **hard reject가 아닌 항목**: lateral/longitudinal acceleration과 deceleration은 speed shaping, response-delay braking deficit과 exit-next는 ranking, temporal steering-rate와 publish feasibility는 diagnostic only이다. friction ellipse와 continuous swept obstacle collision은 없다.
3. **ranking 우선순위**: `exit-next=false` → braking feasible → 작은 braking deficit → 작은 velocity loss → 큰 minimum normalized safety slack → 작은 mean |d| deviation → 작은 generation index. weighted sum이 아니다.
4. **실제 lifecycle**: 기본 `TEST_ACTIVE`에서 P3 continuation → fresh P3의 M0-V1 → M0-V2 → M1 → guarded+raw exact validation → publish. 없으면 safety/P0-named backup → 같은 P3 generator를 재사용한 plan → margin slow pass 또는 safe-stop. P2와 별도 P0 generator는 없다.
5. **generation/validation/ranking 실패 구분**: 내부 trace로 상당 부분 가능하지만 외부 event는 success candidate table, root/filter 단계, 실제 validator 실행 수가 빠진다. mapping miss와 family infeasible은 oracle 없이는 구분 불가다.
6. **temporal stability**: generic previous-cost hysteresis 대신 immutable P3 suffix, current exact revalidation, guard retention, side lock, obstacle-ID lineage, safe-stop latch와 handoff state를 쓴다. 이전 `d_mid`나 probe를 다음 rank에 사용하지 않는다.
7. **논문 전 필수 instrumentation**: event join key, 모든 success/failure candidate의 stage/identity/root/probe/metrics/rank, discarded side와 strict/relaxed를 포함한 실제 constructed/validator counts, authority별 rejection, stage runtime, config/source SHA.
8. **구조적 limitation 5개**: (a) collision은 discrete waypoint center 검사이며 swept footprint가 아니다, (b) acceleration 축과 steering dynamics가 hard coupled constraint가 아니다, (c) bounded mapping 실패와 P3 family/물리적 infeasible을 현재 구분 못 한다, (d) advertised cap-24 counter가 discarded-side 실제 계산량을 가린다, (e) success P3 diagnostic이 비선택 후보와 per-candidate probe/root provenance를 생략한다.
