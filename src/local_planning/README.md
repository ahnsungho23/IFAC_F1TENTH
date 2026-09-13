# local_planning

정적 장애물 회피 로컬 플래너 패키지입니다. 실행 노드는 `local_planner_node` 하나이며 C++로
구현되어 있습니다.

핵심 원칙은 **레이스라인 잠금(race-line locking)** 입니다. 장애물이 나타나도 자유공간에서 새
경로를 탐색하지 않고, `/global_waypoints`의 waypoint 순서를 그대로 유지한 채 각 점의 Frenet
횡오프셋 `d(s)`만 바꿉니다. 따라서 스네이크 구간처럼 서로 다른 트랙 조각이 지도상 가까이
붙어 있어도 다른 조각으로 경로가 점프하지 않습니다. 동적 상대차 대응은 이 패키지 범위 밖입니다.

- 계획 주기: 25 ms (40 Hz, `planning_period_ms`)
- 후보 생성기: analytic corridor 생성기(코드명 P3) 하나
- 검증기: `RacelineSplinePlanner::validateCandidate` 하나
- 운영 파라미터: `config/local_planning.yaml`

## 1. 입력과 출력

### 1.1 구독 토픽

| 토픽 (파라미터) | 메시지 | QoS | 용도 |
|---|---|---|---|
| `/global_waypoints` (`global_waypoints_topic`) | `f110_msgs/WpntArray` | reliable, transient_local | 순서를 고정할 글로벌 레이스라인. `s_m` 엄격 증가, 모든 필드 유한, 4점 이상이어야 수락합니다. 트랙 길이는 마지막 `s_m` + 중앙값 간격으로 추정합니다. |
| `/confirmed_static_obs` (`obstacles_topic`) | `f110_msgs/ObstacleArray` | reliable, volatile | 회피 **기하**의 유일한 입력. obstacle_detector Layer 2가 STATIC으로 확정한 객체만 실립니다. `s_start/s_end/d_right/d_left`만 사용하고 Cartesian 필드는 쓰지 않습니다. |
| `/static_obs` (`raw_slowdown_topic`) | `f110_msgs/ObstacleArray` | reliable, volatile | 확정 전 장애물에 대한 **감속 힌트 전용**. 기하·커밋·정지 판단에는 쓰지 않습니다. |
| `/car_state/frenet/odom` (`frenet_odom_topic`) | `nav_msgs/Odometry` | reliable, volatile | ego Frenet 상태. `pose.position.x = s`, `pose.position.y = d`, `twist.linear.x = 속도`. |
| `/state` (`state_topic`) | `f110_msgs/StateMachine` | reliable, transient_local | AVOID 진입 확인과 GLOBAL 복귀(handoff) 완료 확인. |

장애물 메시지는 `header.frame_id`가 `frame_id`(기본 `map`)와 같아야 하며, 각 장애물은
`d_right <= d_left`이고 종·횡 폭 중 하나 이상이 양수여야 합니다. 조건에 맞지 않는 장애물은
제외하고, 비어 있지 않은 배열이 전부 거부되면 perception 이상으로 보고 직전 스냅샷을 유지합니다.
TF 조회는 하지 않습니다.

### 1.2 발행 토픽

| 토픽 (파라미터) | 메시지 | 발행 시점 | 내용 |
|---|---|---|---|
| `/avoid_waypoints` (`ot_waypoints_topic`) | `f110_msgs/OTWpntArray` | 매 계획 주기 1회 | ego 위치부터 글로벌 합류 뒤 tail까지의 회피 세그먼트. |
| `/local_planning/path` (`local_path_topic`) | `nav_msgs/Path` | 구독자가 있을 때만, 0.1 s 간격 | RViz 확인용 현재 경로. |
| `/local_planning/p3_shadow` (`p3_diagnostics_topic`) | `std_msgs/String` (JSON) | 후보·검증·lifecycle 상태가 바뀔 때 + 0.5 s 하트비트 | 진단 전용. 구독자가 없으면 JSON을 만들지 않습니다. |

`/avoid_waypoints`의 각 `Wpnt`는 글로벌 waypoint를 복사한 뒤 `d_m`, `x_m`, `y_m`을 새로
쓰고 `psi_rad`, `kappa_radpm`, `vx_mps`, `ax_mps2`를 재계산한 것입니다. `s_m`, `d_left`,
`d_right`는 글로벌 값을 그대로 물려받습니다.

| 필드 | 값 |
|---|---|
| `ot_side` | `left` / `right`, 정지·준비 경로는 `stop` |
| `ot_line` | `raceline_local_d_offset_spline`(정상 회피), `raceline_static_prepare`(최초 관측 중 준비 감속), `raceline_static_safe_stop`(안전 정지), `raceline_global_handoff`(글로벌 복귀) |
| `side_switch` | 직전 발행과 회피 방향이 바뀌었으면 `true` |
| 빈 발행 | `wpnts`가 비어 있고 `ot_line`에 사유 문자열이 실립니다 |

최종 `/local_waypoints` 선택과 발행은 state_machine이 단독으로 담당합니다. 이 노드는
`/local_waypoints`를 절대 발행하지 않습니다.

## 2. 회피 경로 생성 방법

### 2.1 장애물 전처리

1. 장애물 `s`를 ego 기준 전방 거리로 펼치고(폐루프 wrap 처리), `detection_lookahead_m`(15 m)
   안에 있는 것만 남깁니다.
2. 횡방향 여유 `C_obs = vehicle_half_width_m + safety_margin_m = 0.15 + 0.08 = 0.23 m`를
   장애물 좌우 면에 더해 **차량 중심 기준 금지 구간** `[d_right − C_obs, d_left + C_obs]`를
   만듭니다. 이 값이 생성과 검증에서 쓰는 유일한 장애물 clearance입니다.
3. 팽창된 구간이 레이스라인 `d = 0`을 덮으면 blocking 장애물입니다. 그 뒤로
   `obstacle_cluster_gap_m`(0.8 m) 이내에 이어지는 장애물은 같은 군집으로 묶어 한 번의
   기동으로 처리합니다.
4. 최초 군집은 같은 ID를 `initial_observation_count`(3)회, 최소
   `initial_observation_min_duration_sec`(0.15 s) 동안 모은 뒤(최대 0.35 s) 여러 프레임의
   Frenet 경계 합집합으로 계획합니다. 그동안은 `raceline_static_prepare` 준비 감속 경로를 냅니다.

### 2.2 코리도어와 목표 오프셋 `d`

- ego부터 지평까지 station(군집 시작·중간·끝, 각 장애물 시작·중심·끝, 글로벌 waypoint 위치)을
  깔고, station마다 주행 가능 구간
  `[−d_right + p, d_left − p] − (장애물 금지 구간)`, `p = vehicle_half_width_m + wall_safety_margin_m`
  을 계산합니다. station 간 연결성이 끊기면 그 방향은 버립니다.
- 좌·우 **양쪽 모두** 후보를 만들고 마지막에 순위로 고릅니다. 방향별 최소 목표는 군집 금지
  구간의 바깥 면이며 `minimum_target_offset_m`(0.15) 이상, `maximum_target_offset_m`(1.5)
  이하여야 합니다.
- 군집 구간 전체에서 공통으로 열린 연결 성분마다 비율 {0, 0.25, 0.5, 0.75, 1}에서
  목표 `d`를 뽑고 `|d|`가 작은 순으로 시도합니다.

### 2.3 station 5개와 5차 프로파일

기동 하나는 5개 station과 5개 오프셋으로 정의합니다.

```text
stations = { start, apex, (apex+end)/2, end, end + exit_length }
offsets  = { ego.d, target, d_mid, target, 0 }

apex        = 군집 시작
start       = apex − apex · (pre_apex_distances_m[0] · entry_scale / detection_lookahead_m)
exit_length = post_apex_distances_m[2] · exit_scale (· outside_line_transition_scale, 코너 바깥쪽 복귀일 때)
```

ego가 이미 군집 옆에 있어 `apex`가 `|target − ego.d| / maximum_lateral_slope`보다 짧으면
`start = 0`으로 두고 ego 위치에서 곧바로 램프를 시작합니다.

인접 station 사이는 **5차 Hermite 다항식** 4개로 잇습니다. 각 절점에서 위치·1차·2차 미분을
맞추므로 `d(s)`는 전 구간에서 C² 연속이고, 절점 기울기는 양옆 할선의 조화평균(부호가
다르면 0)으로 잡아 오버슈트가 생기지 않습니다. 양 끝 절점은 기울기와 2차 미분이 0입니다.
가운데 오프셋 `d_mid`는 열거하지 않고, 군집 안에서 곡률이 가장 큰 station(또는 가장 좁은
station)에서 원하는 `d`를 정확히 지나도록 2차 방정식을 풀어 해석적으로 구합니다.

### 2.4 후보 집합

후보는 세 가족을 차례로 시도하며, 앞 가족에서 유효 후보가 나오면 뒤 가족은 생략합니다.
전체 상한은 24개입니다.

| 가족 | 내용 | 상한 |
|---|---|---|
| M0 | 목표 `d` 1개 × 복귀 길이 비율 {0.016, 0.5, 1.0} × 진입 배율 {`entry_transition_fractions` 최소, 최대}. 짧은 진입이 곡률·기울기로, 긴 진입이 벽·footprint로 실패하면 진입 배율을 이분 탐색합니다. | 16 |
| M0 확장 | 연결 성분별 고정 템플릿(경계 안쪽, 1/4 지점 긴 복귀, 중앙 긴 복귀 등)과 해석적 `d_mid` 근. | 12 |
| M1 | 남은 예산으로 (방향 × 성분) 라운드로빈 템플릿(짧은 경계, 경계 span, 근거리 긴 복귀, 원거리 span)과 이분 탐색. | 24 − M0 |

동일 `(방향, target, d_mid, entry, exit)` 조합은 한 번만 평가합니다.

### 2.5 Cartesian 복원

ego 다음 글로벌 waypoint부터 마지막 station + tail까지 글로벌 점을 **순서대로** 복사하고
`d`만 프로파일 값으로 바꾼 뒤 아래 식으로 좌표를 옮깁니다.

```text
x_local = x_global − d(s) · sin(psi_global)
y_local = y_global + d(s) · cos(psi_global)
tail    = max(post_merge_lookahead_m, |v_ego| · post_merge_min_time_sec)   # 5 m 또는 1 s
```

이후 `psi_rad`와 `kappa_radpm`을 새 좌표에서 다시 계산하고(국소 3차 최소제곱 피팅), 4절의
속도 정책을 적용합니다.

### 2.6 후보 순위

검증을 통과한 후보끼리는 다음 순서로 사전식 비교합니다(`candidate_rank.hpp`).

1. 복귀 램프가 군집 밖의 다음 장애물 영역에 닿지 않는 후보 우선
2. ego 현재 속도에서 제동이 가능한 후보 우선, 그다음 제동 거리 부족량이 작은 순
3. 글로벌 대비 속도 손실이 작은 순
4. 정규화 안전 여유(벽·장애물·곡률·곡률변화율 중 최솟값)가 큰 순
5. 글로벌 경로 이탈량이 작은 순
6. 생성 순서 (완전 동률 시, 결정성 유지)

### 2.7 커밋과 유지

- **continuation-first**: 이미 커밋된 기동이 있으면 매 주기 먼저 그 경로의 남은 구간을
  현재 장애물로 재검증하고, 통과하면 후보를 새로 만들지 않고 그대로 발행합니다. 재검증이
  실패했을 때만 2.1~2.6을 다시 돌립니다.
- **frozen guard**: 커밋 시 장애물 ID별 Frenet 경계를 고정합니다. 이후 관측된 경계가 고정
  경계 안에 있으면 고정값으로 검증하므로 perception 흔들림이 경로 형상으로 전달되지 않습니다.
  빈 프레임이 오면 detector 탈락으로 보고 고정 경계를 그대로 씁니다.
- **방향 잠금**: 커밋된 경로는 불변이므로 전체 무효화 없이는 방향이 바뀌지 않습니다.
- **완료**: ego 진행 거리가 확장된 군집 끝을 넘으면 기동을 완료하고, 지나간 장애물 ID를
  제외 목록에 넣어 같은 장애물로 다시 계획하지 않습니다.
- **다음 장애물 연결**: 현재 커밋에 포함되지 않은 blocking 군집은 회피 중에도 동시에 관측을
  쌓습니다. 현재 군집을 지나면 현재 `ego.s/ego.d`를 시작점으로 다음 기동을 바로 이어 붙이고,
  중간에 빈 경로나 글로벌 경로를 내보내지 않습니다.
- **글로벌 복귀(handoff)**: 모든 blocking 군집이 끝나면 글로벌 전체 루프를 ego 위치 기준으로
  회전시킨 `d = 0` 폐루프를 `raceline_global_handoff`로 발행합니다. 속도 상한은
  `state_handoff_speed_cap_mps`(6 m/s), tail은 `state_handoff_tail_distance_m`(6 m)입니다.
  커밋 장애물 뒤쪽 끝이 아직 전방 `handoff_latch_commit_distance_m`(8 m) 안이면 handoff를
  막습니다. `/state`가 `STATE_GLOBAL`로 돌아온 것을 확인한 뒤에만 빈 `/avoid_waypoints`를
  발행합니다.

## 3. Validator 동작 방법

모든 후보(신규·커밋 유지·정지 경로)는 같은 함수 `validateCandidate`로 검사합니다.
아래 순서대로 검사하며 첫 실패에서 사유(종류·waypoint·`s/d`·좌표)를 기록하고 멈춥니다.

| # | 검사 | 조건 | 관련 파라미터 (운영값) |
|---|---|---|---|
| 1 | 진입 연속성 | `\|d_entry − d_ego\| > entry_discontinuity_min_budget_m` **이고** `\|Δd\| / max(entry_forward, entry_continuity_baseline_m) > maximum_lateral_slope`이면 거부 | 0.20 m, 0.50 m, 0.8 |
| 2 | 전방 길이 | ego 앞 waypoint가 `minimum_path_points` 이상 | 8 |
| 3 | 중심선 트랙 경계 | `−(d_right − w) ≤ d ≤ d_left − w`, `w = wall_safety_margin_m` | 0.04 m |
| 4 | 차체 footprint 트랙 경계 | `vehicle_length_m × 2·vehicle_half_width_m` 직사각형을 `psi_rad`로 회전시킨 네 모서리를 인접 글로벌 segment에 투영해 `d_left/d_right − w` 안에 있어야 함 | 0.56 m, 0.15 m |
| 5 | 장애물 충돌 | 기동 범위 안의 장애물에 대해 `forward_s ∈ [s_start, s_end]`이면서 `d`가 `[d_right − C_obs, d_left + C_obs]` 안이면 거부 | `C_obs` = 0.23 m |
| 6 | 순서 단조성 | 연속 waypoint 사이 `Δs > 0` | |
| 7 | 횡기울기 | `\|Δd\| / Δs ≤ maximum_lateral_slope` | 0.8 |
| 8 | 곡률 변화율 | `\|Δκ\| / Δs ≤ maximum_curvature_rate_radpm2` | 20 rad/m² |
| 9 | 곡률 | `\|κ\| ≤ min(maximum_curvature_radpm, tan(δ_max,방향) / L)`. 좌·우 조향 한계가 달라 방향별로 상한이 다름 | 1.316, L = 0.33 m, δ = 0.410 / 0.361 rad |

장애물 충돌 검사(5번)만 **기동 범위**(군집 끝 + `post_merge_lookahead_m`와 다음 장애물
시작 중 작은 값)까지 보고, 나머지는 경로 전체를 봅니다. 후보 선택·생성·커밋 재검증이 같은
범위를 써야 재계획이 수렴합니다.

다음 항목은 validator에서 거부하지 않습니다.

- 횡가속도 `v²|κ| ≤ a_lat,max(v)`: 속도를 낮춰 **구성 단계에서** 만족시킵니다(3절).
- ego 제동 가능성: 거부 대신 순위 항목(2.6의 2번)으로만 씁니다.
- 발행 직전 `inspectVelocityFeasibility`는 횡가속·가감속·조향률 위반 개수를 경고 로그로만
  남기고 경로를 바꾸지 않습니다.

### 3.1 커밋 경로 재검증: hard와 soft

커밋된 경로가 재검증에 실패하면 실패 종류로 나눕니다.

- **hard**: 트랙 경계·기하·순서 등 장애물 외 사유로 실패하거나, detector **원본** 경계에
  `C_obs`를 적용해도 충돌하면 그 주기에 즉시 무효화하고 재계획합니다.
- **soft**: 고정 guard 기준으로만 충돌하고 원본 경계로는 통과하면 고정 형상을 유지합니다.
  카운터(`commitment_soft_violation_confirm_cycles`)는 진단 로그용입니다.

### 3.2 유효 후보가 없을 때

1. 군집이 `C_obs` 여유로만 레이스라인을 막고 물리적으로는 겹치지 않으면 `d = 0`을 유지한
   채 `margin_pass_speed_cap_mps`(2 m/s)로 감속 통과합니다.
2. 그 외에는 **안전 정지**: 현재 `d`를 유지하며 `s_stop = 군집 시작 − safe_stop_buffer_m`(2.6 m)
   까지 감속합니다. 정지점을 확정하기 전에 **탈출 검사**를 합니다. 정지점에서 속도 0으로
   다시 계획했을 때 유효 후보가 나오는 가장 늦은 지점을 이분 탐색(해상도 0.30 m, 최대 8회)으로
   찾고, 어느 지점에서도 탈출이 안 되면 원래 정지점을 씁니다.
3. 정지 경로를 만들 때는 우선순위가 있습니다. 커밋된 기하 위에서 충돌 지점 앞 2.6 m까지
   제동 → 그것이 2점 미만이면 버퍼를 포기하고 충돌 직전 정지 → 마지막 발행 경로 위 제동 →
   현재 `d`의 속도 0 hold. 어느 경우든 제어기 lookahead 확보를 위해 `vx = 0` 점을
   `controller_lookahead_floor_m`(2.5 m)까지 덧붙입니다.
4. 안전 정지는 즉시 latch되고, ego가 위험 구간 끝 + 2.6 m를 지나거나, 같은 장애물에 대한
   유효 회피가 `safe_stop_release_cycles`(8)회 연속 나오고 상태가 `STATE_AVOID`이거나,
   정차 후 새 detector 프레임이 8회 연속 전방 clear를 보이거나, 정차 후 `safe_stop_blind_release_sec`
   (4 s) 동안 빈 프레임만 오면(이때 위험 구간은 `safe_stop_blind_creep_speed_mps` 0.7 m/s로
   통과) 해제됩니다. 정차 판정은 `|v| ≤ safe_stop_deceleration_mps2 × 계획주기 = 0.045 m/s`입니다.

## 4. 속도값 선정 방법

모든 waypoint의 `vx_mps`는 글로벌 레이스라인 속도에서 시작해 아래 단계를 **min으로만**
적용합니다. 어떤 단계도 속도를 올리지 않습니다.

| 순서 | 단계 | 규칙 | 파라미터 (운영값) |
|---|---|---|---|
| 1 | 횡가속 상한 | 새 기하의 곡률로 `v²\|κ\| ≤ a_lat,max(v)`를 만족하는 최대 `v`를 이분 탐색 | 4.1 표 |
| 2 | 확정 장애물 통과 속도 hold | station[0]~station[3] 사이 1단계 결과의 최솟값을 station[1]부터 station[3] + `confirmed_speed_post_hold_distance_m` 까지 유지 | 1.0 m |
| 3 | 준비(접근) 감속 램프 | 장애물 앞 목표 속도로 내려가는 역방향 램프. 제어 응답 지연 `v_ego · confirmed_speed_response_delay_sec`만큼 당겨서 시작하고, 기울기는 `[approach_feasibility_decel_mps2, 표의 감속 상한]` 안에서 필요값으로 정함 | 0.15 s, 2.0 m/s² |
| 4 | 종방향 전진 패스 | `v_ego`(하한 `longitudinal_launch_speed_floor_mps`)에서 시작해 `v ← √(v² + 2·a_acc(v)·Δs)` | 1.0 m/s, 4.1 표 |
| 5 | 종방향 후진 패스 | 뒤에서부터 `v_i ← min(v_i, √(v_{i+1}² + 2·a_dec·Δs))` | 4.1 표 |
| 6 | `ax_mps2` 재계산 | `(v_{i+1}² − v_i²) / (2·Δs)`, 마지막 점 0 | |
| 7 | 미확정 장애물 감속 오버레이 | 발행 직전 사본에 4.2 규칙 적용 | `raw_slowdown_*` |

### 4.1 속도별 차량 한계표

기준 문서는 `config/local_planning_velocity_limits.csv`이고, 런타임은 YAML 배열을 읽습니다.
횡가속 열은 `min(CSV, f1tenth_control의 max_lateral_accel 7.6)`으로 합성한 값입니다.
`test/test_velocity_limits_match_csv.py`와 `test/test_control_contract_match.py`가 CSV·YAML·
C++ 기본값·control launch가 서로 같은지 검사하므로, 한계를 바꾸면 CSV를 먼저 고치고 두
테스트를 통과시켜야 합니다.

| 속도 [m/s] | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|---|
| a_lat,max [m/s²] | 7.6 | 7.6 | 7.6 | 7.6 | 7.0 | 7.0 | 7.0 | 6.5 | 6.5 | 6.5 |
| a_acc,max [m/s²] | 3.7 | 3.7 | 3.7 | 3.7 | 3.7 | 3.47 | 3.33 | 3.0 | 3.0 | 3.0 |
| a_dec,max [m/s²] | 2.0 | 2.0 | 2.0 | 2.0 | 2.0 | 2.0 | 2.0 | 2.0 | 2.0 | 2.0 |

구간 사이는 선형 보간하고 범위 밖은 끝 값으로 clamp합니다.

### 4.2 미확정 장애물 감속 힌트 (`/static_obs`)

아직 STATIC으로 확정되지 않은 장애물이 전방 `raw_slowdown_trigger_distance_m`(12 m) 안에서
레이스라인 ±`raw_slowdown_lateral_margin_m`(0.25 m)에 걸리면, 발행하는 경로 사본에만 다음
프로파일을 씌웁니다.

```text
장애물 앞      : min(vx, √(cap² + 2·a·(front − s)))      cap = raw_slowdown_speed_cap_mps (2.8)
장애물 구간    : min(vx, cap)                            a   = approach_feasibility_decel_mps2 (2.0)
구간 뒤 1.0 m  : min(vx, cap)                            (raw_slowdown_post_hold_distance_m)
그 이후        : 표의 가속 한계로 복귀
```

시작점은 `min(front, v_ego·0.15 + 0.5)`만큼 앞당깁니다. 같은 장애물이 이미 확정되어 현재
회피 경로에 포함되어 있으면 (`raw_slowdown_skip_committed: true`) 힌트를 적용하지 않습니다.

### 4.3 글로벌 복귀 속도 성형

`handoff_speed_shaping_enable: true`일 때 handoff 경로는 `state_handoff_speed_cap_mps`(6 m/s)
상한 뒤에 실제 기하 곡률로 1단계 횡가속 상한을 다시 적용하고, 측정 `v_ego`에서 출발하는
전진 가속 램프와 후진 감속 패스를 랩 경계를 넘어 ego 순서로 수행합니다.

### 4.4 안전 정지 속도

정지 경로는 별도 상수 `safe_stop_deceleration_mps2`(1.8)를 씁니다.

```text
vx(s) = min(v_global, √(2 · 1.8 · max(0, s_stop − s)))
```

8점 미만이면 점을 보간해 채우고 보간점에도 같은 식을 다시 적용하며, 마지막 점은 0으로
고정합니다. 커밋 기하 위에서 버퍼를 포기하고 정지할 때는 1.8 m/s²를 넘을 수 있습니다.

## 5. 파라미터

운영값은 `config/local_planning.yaml` 한 곳에 있으며 launch가 기본으로 읽습니다.
`config/local_planning_sim.yaml`은 시뮬 전용 마진값 세트로, 쓰려면 `params_file:=`로 명시해야
합니다(기본 launch는 시뮬에서도 운영 YAML을 읽습니다).

| 분류 | 파라미터 (운영값) |
|---|---|
| 토픽·프레임 | `global_waypoints_topic`, `obstacles_topic`, `raw_slowdown_topic`, `frenet_odom_topic`, `state_topic`, `ot_waypoints_topic`, `local_path_topic`, `frame_id` (`map`) |
| 주기·신선도 | `planning_period_ms` 25, `local_path_publish_period_sec` 0.1, `obstacle_stale_timeout_sec` 0.75, `odometry_stale_timeout_sec` 5.0 |
| 차체·마진 | `vehicle_length_m` 0.56, `vehicle_half_width_m` 0.15, `safety_margin_m` 0.08, `wall_safety_margin_m` 0.04, `fallback_track_half_width_m` 1.5 |
| 검출·군집 | `detection_lookahead_m` 15, `obstacle_cluster_gap_m` 0.8, `initial_observation_count` 3, `initial_observation_min_duration_sec` 0.15, `initial_observation_max_wait_sec` 0.35 |
| 목표·전환 | `minimum_target_offset_m` 0.15, `maximum_target_offset_m` 1.5, `target_d_candidate_count` 5, `pre_apex_distances_m`, `post_apex_distances_m`, `entry_transition_fractions`, `transition_distance_scales`, `outside_line_transition_scale`, `post_merge_lookahead_m` 5, `post_merge_min_time_sec` 1.0 |
| 기하 한계 | `maximum_lateral_slope` 0.8, `maximum_curvature_radpm` 1.316, `maximum_curvature_rate_radpm2` 20, `entry_discontinuity_min_budget_m` 0.20, `entry_continuity_baseline_m` 0.50, `minimum_path_points` 8 |
| control 계약 미러 | `control_wheelbase_m` 0.33, `control_max_steering_left_rad` 0.410, `control_max_steering_right_rad` 0.361, `control_understeer_gradient_*`, `control_max_steering_rate_radps` 20 |
| 속도 | `avoidance_velocity_limit_speed_bins_mps`, `avoidance_velocity_limit_lateral_accel_mps2`, `avoidance_velocity_limit_accel_mps2`, `avoidance_velocity_limit_decel_mps2`, `avoidance_minimum_speed_mps` 1.0, `margin_pass_speed_cap_mps` 2.0, `approach_feasibility_decel_mps2` 2.0, `confirmed_speed_post_hold_distance_m` 1.0, `confirmed_speed_response_delay_sec` 0.15, `longitudinal_launch_speed_floor_mps` 1.0, `state_handoff_speed_cap_mps` 6.0 |
| 감속 힌트 | `raw_slowdown_enable` true, `raw_slowdown_trigger_distance_m` 12, `raw_slowdown_speed_cap_mps` 2.8, `raw_slowdown_lateral_margin_m` 0.25, `raw_slowdown_post_hold_distance_m` 1.0, `raw_slowdown_skip_committed` true |
| 커밋·복귀 | `commitment_soft_violation_confirm_cycles` 3, `commitment_lock_lateral_threshold_m` 0.10, `commitment_lock_longitudinal_m` 0.50, `chain_release_distance_m`, `handoff_latch_commit_distance_m` 8, `state_handoff_tail_distance_m` 6, `merge_lateral_tolerance_m` 0.15, `merge_confirm_cycles` 15, `maneuver_memory_clear_frames` 3, `maneuver_memory_max_ahead_m` 15 |
| 안전 정지 | `safe_stop_buffer_m` 2.6, `safe_stop_deceleration_mps2` 1.8, `safe_stop_release_cycles` 8, `safe_stop_escape_check_enable` true, `safe_stop_escape_retreat_step_m` 0.30, `safe_stop_escape_max_retreats` 8, `safe_stop_blind_release_sec` 4.0, `safe_stop_blind_creep_speed_mps` 0.7, `controller_lookahead_floor_m` 2.5, `stop_geometry_extend` true, `hold_republish_last_guidance` true |
| 진단 | `p3_diagnostics_topic`, `p3_diagnostics_detail` `EVENTS`, `p3_diagnostics_heartbeat_sec` 0.5 |

## 6. 실행 방법

`local_planning.launch.py`는 로컬 플래너와 함께 obstacle_detector, 검출기용 wall-only 참조
맵 서버(`/local_planning/reference_map`)를 띄웁니다. 플래너 자체는 점유격자 맵을 쓰지 않습니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
# 실차
ros2 launch local_planning local_planning.launch.py
# 시뮬레이터 (gym 브리지 ego odom 토픽 사용)
ros2 launch local_planning local_planning.launch.py simulator:=true
```

| 인자 | 기본값 | 설명 |
|---|---|---|
| `params_file` | `config/local_planning.yaml` | 플래너 파라미터 파일 |
| `simulator` | `false` | `true`면 obstacle_detector가 ego odom을 `/ego_racecar/odom`에서 받음. 시뮬에서는 반드시 명시 |
| `use_sim_time` | `false` | 시뮬레이션 시간 |
| `start_obstacle_detector` | `true` | `false`면 플래너만 띄움(`local_planner_only.launch.py`와 동일). 검출기를 따로 띄울 때 사용 |
| `reference_map` | `kinematic_localization/maps/map_kissmap_render.yaml` | 검출기 벽 필터 참조 맵 |

실행 확인:

```zsh
ros2 topic hz /avoid_waypoints                 # 40 Hz
ros2 topic echo /avoid_waypoints --field ot_line
ros2 topic echo /local_planning/p3_shadow      # 후보·검증 진단 JSON
```

RViz에서 `/local_planning/path`를 추가하면 현재 경로가 보입니다.

## 7. 빌드와 테스트

```zsh
cd ~/2026_IFAC
colcon build --packages-select local_planning
colcon test --packages-select local_planning && colcon test-result --verbose
```

`test/`에는 validator·순위·guard·lifecycle 단위 테스트, 기록된 장애물 스트림
(`test/data/p3_scenarios/`)으로 생성기 결과를 고정하는 parity 테스트, CSV/YAML/C++/control
계약 일치를 검사하는 pytest가 있습니다.

패키지 규칙은 [`AGENTS.md`](AGENTS.md), 변경 이력을 포함한 상세 설계 노트는
[`docs/local_planner.md`](docs/local_planner.md)를 참고하십시오.
