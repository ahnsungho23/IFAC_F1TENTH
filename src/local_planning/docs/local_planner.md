# local_planner_node

## 1. 노드 목적

`local_planner_node`는 정적 장애물이 글로벌 Race Line을 막을 때만 로컬 회피 세그먼트를 만듭니다.
동적 상대 차량 정보는 `obstacle_detector`의 `/opp_obs`로 분리되며, 이 노드는 정적 장애물용
`/static_obs`만 구독하고 `/avoid_waypoints`를 발행합니다.

가장 중요한 설계 조건은 다음과 같습니다.

> 출력 경로는 항상 `/global_waypoints`의 진행 순서와 `s_m`을 유지하며, 각 점의 로컬 `d_m`만
> 바꾼다.

따라서 Cartesian 공간에서 가까운 점을 다시 찾거나 자유공간을 가로질러 새 경로를 연결하지
않습니다. 스네이크 구간처럼 서로 다른 트랙 조각이 가까이 있어도 차량은 현재 Race Line의 다음
점들만 따라갑니다.

## 2. 참고 구현과 적용 범위

알고리즘 개념은 `vaithak/f1tenth-icra-race`의 `scripts/spliner.py`를 참고했습니다.

- 참고 저장소: `git@github.com:vaithak/f1tenth-icra-race.git`
- 분석 기준 커밋: `ac9a4c98948cc74077f0435d40017468d68d4d6c`
- 가져온 핵심 개념: Frenet `s`를 독립변수로 하는 cubic spline, pre/apex/post 제어점,
  장애물 좌우 여유에 따른 회피 방향 선택, 트랙 폭 검사

현재 프로젝트에는 다음 차이를 반영해 C++17로 새로 구현했습니다.

1. CSV 대신 `/global_waypoints`를 사용합니다.
2. `/static_obs`의 `s_start/s_end/d_right/d_left`를 장애물의 authoritative Frenet 경계로
   사용합니다. `obstacle_detector`가 map-frame Cartesian AABB 전체를 CLCS로 투영하고, 가까운
   Race Line 선분과 AABB 면 사이의 최단거리까지 반영합니다. local planner는 이 좌표변환을
   반복하지 않습니다. Cartesian AABB와 enclosing-circle `radius`는 회피 형상에 사용하지
   않으며, 유효한 현재 AABB가 있을 때 RViz 표시에만 사용합니다.
3. 한 점 apex가 아니라 장애물 군집의 앞·뒤에서 목표 `d`를 유지해 긴 정적 장애물도 처리합니다.
4. 글로벌 waypoint 자체를 출력 표본으로 사용해 Race Line의 위상 순서를 강제합니다.
5. 좌우 모두 불가능하면 장애물 앞 감속 경로를 발행합니다.
6. 이동 후 heading, curvature, velocity, acceleration을 다시 계산합니다.

## 3. 동작 원리

### 3.1 입력 준비

1. `/global_waypoints`의 모든 값이 유한하고 `s_m`이 엄격히 증가하는지 검사합니다.
2. 마지막 `s_m`과 waypoint 중앙 간격으로 폐루프 트랙 길이를 구합니다.
3. Frenet odometry의 `position.x`를 ego `s`, `position.y`를 ego `d`로 읽습니다.
4. `/static_obs`를 provisional/confirmed 정적 레이어 계약에 따라 그대로 입력받습니다.
5. detector가 채운 Frenet 값이 유한하고 `d_right <= d_left`이며 종·횡방향 중 하나 이상의
   폭이 양수인지 검사합니다. 조건을 만족하지 않으면 해당 장애물을 제외합니다. Cartesian
   AABB는 `has_cartesian=true`이고 값이 유효할 때만 marker metadata로 보존합니다.

### 3.2 가장 가까운 정적 장애물 군집

1. ego 앞 `detection_lookahead_m` 안의 장애물 상자를 폐루프 `s`로 펼칩니다.
2. 최초 commitment용 Frenet 경계 합집합에는 `uncertainty_sigma_scale * sqrt(s_var/d_var)`와
   `uncertainty_min_*_margin_m`을 더해 측정 불확실성 Guard를 만듭니다.
3. 이 Guard를 다시 종방향 `obstacle_longitudinal_padding_m`, 횡방향
   `obstacle_clearance_m`만큼 팽창합니다. 현재 횡방향 clearance 기본값은 0.35 m이며,
   spline 목표에는 `commitment_clearance_reserve_m=0.05 m`의 추가 기하 여유를 둡니다.
4. 원본 장애물의 가장 가까운 면에서 구한 곡선 기준 `|d|`가 글로벌 `d=0`의 차량 envelope
   (`vehicle_half_width_m + blocking_margin_m`) 안에 들어올 때만 blocking 장애물로 봅니다.
   경로 생성용 `obstacle_clearance_m`을 blocking 판정에 다시 더하지 않습니다.
5. `obstacle_cluster_gap_m`보다 가까운 후속 장애물은 같은 기동으로 처리합니다.

### 3.3 최초 군집 안정화

처음 blocking 장애물이 들어오면 곧바로 좌우 spline을 확정하지 않습니다. 먼저 글로벌 `d=0` 위의
검증된 감속 prefix를 `ot_line=raceline_static_prepare`로 발행합니다. 가장 가까운 군집의 각
ID가 서로 다른 `/static_obs` 메시지에서 `initial_observation_count`회 관측되고
`initial_observation_min_duration_sec`도 지날 때까지 wrap-aware Frenet 경계 합집합과 가장 큰
`s_var/d_var`를 누적합니다. planning timer가 같은 메시지를 여러 번 사용하더라도 관측 횟수는
한 번만 증가합니다. 기본 최소 0.15초를 함께 요구하므로 약 250Hz detector의 연속 3개 메시지만
약 12ms 동안 받은 상태에서 곧바로 commitment하지 않습니다.

기본 3회 관측과 최소 0.15초가 모두 끝나면 다음 순서로 고정 Guard를 만듭니다.

1. 같은 ID의 세 detector Frenet 경계를 폐루프 `s`를 고려해 합집합으로 만듭니다.
2. 좌표변환 없이 이 종·횡 경계를 최초 obstacle envelope로 사용합니다.
3. 종방향 양쪽에
   `uncertainty_min_longitudinal_margin_m + uncertainty_sigma_scale * sqrt(s_var)`를 더합니다.
4. 횡방향 양쪽에
   `uncertainty_min_lateral_margin_m + uncertainty_sigma_scale * sqrt(d_var)`를 더합니다.
5. 이 Guard 전체를 피하는 spline을 만들고 Guard와 경로를 함께 commitment에 저장합니다.

분산은 Kalman 중심 위치 불확실성이고 footprint 크기 오차를 직접 포함하지 않으므로 고정 최소 마진을
별도로 더합니다. 분산이 음수이거나 유한하지 않으면 해당 sigma 항은 0으로 두고 최소 마진은
항상 적용합니다. 관측 횟수와 최소 시간을 모두 만족하면 계획하며, 입력이 누락되어 관측 횟수를
채우지 못해도
`initial_observation_max_wait_sec`에 도달하면 그동안의 가장 보수적인 합집합과 분산으로 계획합니다.
장애물이 이미 `safe_stop_buffer_m` 안에 있어 감속 prefix조차 만들 수 없으면 3회를 기다리지 않고
즉시 zero-speed safe-stop을 latch합니다.

### 3.4 좌우 목표 d 계산

- 왼쪽 후보: 군집의 가장 큰 `d_left + clearance + commitment_clearance_reserve_m`
- 오른쪽 후보: 군집의 가장 작은 `d_right - clearance - commitment_clearance_reserve_m`

두 후보를 모두 만들며, 각 글로벌 waypoint의 허용 중심 범위
`[-d_right + 차량반폭 + boundary_margin, d_left - 차량반폭 - boundary_margin]`를 벗어나면
폐기합니다. 이때 장애물 군집의 확대된 `s_start~s_end` 구간에서 `target_d` 자체가 이 범위를
벗어나는 방향은 cubic spline을 만들기 전에 조기 폐기합니다. 이 검사는 명백히 불가능한 방향의
최대 3개 길이 후보 생성을 생략하기 위한 gate이며, 통과한 방향도 전환 구간의 좁은 벽이나 다른
장애물을 놓치지 않도록 기존 전체 waypoint 경계·충돌·곡률 검사를 그대로 수행합니다.

안정화가 끝난 최초 계획에서는 조기 검사를 통과한 양쪽을 비교합니다. commitment 뒤 기존 경로가
위험해졌더라도 ego가 `commitment_lock_lateral_threshold_m`만큼 횡이동하거나
`commitment_lock_longitudinal_m`만큼 전진하기 전이라면 반대쪽도 다시 평가할 수 있습니다.
둘 중 하나에 도달해 실제 회피에 진입한 뒤에는 진행 중 갑자기 반대편으로 꺾지 않도록 방향을
고정합니다. 기본 reserve 0.05m는 uncertainty Guard와 clearance의 검증 경계에 spline이 정확히
접촉하지 않도록 목표를 5cm 더 바깥에 둡니다. 측정 흔들림 자체는 고정 Guard가 담당합니다.

### 3.5 로컬 d-offset spline

장애물 군집 앞에는 `pre_apex_distances_m`의 세 점을 `d=0`으로, 장애물 앞·뒤에는 목표 `d`를,
장애물 뒤에는 `post_apex_distances_m`의 세 점을 `d=0`으로 둡니다. 이 제어점 사이에 natural
cubic spline `d(s)`를 맞춥니다. spline의 반대편 overshoot는 제어점의 최소·최대 `d`로 제한합니다.

그다음 ego부터 merge 뒤 global tail까지의 글로벌 waypoint를 순서대로 복사합니다. tail 길이는
다음처럼 거리 하한과 계획 당시 속도 기준 시간 하한 중 큰 값입니다.

```text
tail_distance = max(post_merge_lookahead_m, ego_speed * post_merge_min_time_sec)
```

따라서 고속에서도 상태 전환이 끝나기 전에 열린 회피 경로의 끝점에 도달하지 않습니다.
각 점은 다음 식으로만 이동합니다.

```text
x_local = x_global - d(s) * sin(psi_global)
y_local = y_global + d(s) * cos(psi_global)
```

`s_m`과 글로벌 진행 순서는 바뀌지 않습니다. 가까운 다른 트랙 조각을 검색하는 단계가 없으므로
비볼록 트랙에서도 잘못된 branch로 이동하지 않습니다.

### 3.6 안전성과 속도

완성된 후보는 다음 조건을 모두 통과해야 합니다.

1. 모든 점이 해당 글로벌 waypoint의 좌우 트랙 폭 안에 있음
2. 팽창된 정적 장애물 Frenet 상자와 겹치지 않음
3. `|dd/ds|`, Cartesian 곡률, 거리당 곡률 변화율 제한 만족
4. 이동된 geometry에서 계산한 곡률로 횡가속도 속도 상한 만족
5. 전·후방 속도 패스로 종가속도와 종감속도 제한 만족

짧은 전환이 실패하면 `transition_distance_scales` 순서대로 같은 목표 `d`를 더 긴 `s` 구간에
펼칩니다. 첫 번째 유효 후보가 나오면 더 긴 후보는 계산하지 않습니다. 이것은 자유공간 후보
탐색이 아니라 동일한 글로벌 점에 적용하는 spline 길이 조정입니다.

좌우가 모두 실패하면 글로벌 `d=0` 위에서 장애물 앞 `safe_stop_buffer_m`까지의 충돌 없는 prefix를
만들고 마지막 속도를 0으로 둡니다. 회피 spline의 `minimum_path_points`보다 짧더라도 2점 이상의
정지 prefix는 별도로 검증해 사용합니다. 장애물이 이미 buffer 안에 있어 prefix를 만들 수 없으면
현재 `d`를 유지하는 전방 waypoint들의 속도를 모두 0으로 둔 emergency hold를 발행합니다. 따라서
정지 실패가 빈 `/avoid_waypoints`를 통해 global 경로 선택으로 이어지지 않습니다.

### 3.7 commitment와 합류

안전 경로가 선택되면 방향, 경로 geometry, ID별 uncertainty Guard를 고정합니다. 매
`/static_obs`에서 같은 ID의 최신 Frenet 경계에도 동일한 uncertainty 확장을 적용합니다. 그 전체가
저장된 Guard 안에 있으면 live envelope 대신 고정 Guard로 기존 경로를 재검증하므로 중심과 크기가
조금 변해도 `target_d`와 출력 waypoint가 바뀌지 않습니다. Guard는 직전 관측을 따라 이동하지
않으므로 작은 변화가 누적된 실제 이동은 결국 Guard 밖으로 나옵니다.

최신 uncertainty envelope가 Guard를 벗어나면 기존 경로를 두 단계로 검사합니다.

1. detector 원본 Frenet 경계에 `vehicle_half_width_m + hard_collision_margin_m`을 더한
   hard 영역과 겹치면 실제 차체 충돌 가능성이므로 그 planning cycle에서 즉시 재계획합니다.
2. 원본 hard 영역은 피하지만 uncertainty Guard와 전체 `obstacle_clearance_m`을 적용한 영역만
   침범하면 soft 충돌로 분류합니다. `commitment_soft_violation_confirm_cycles`회 연속일 때만
   재계획하고, 그 전에 해소되면 카운터를 지우고 고정 경로를 유지합니다.
3. 충돌 로그에는 장애물 ID, 충돌 waypoint의 `s/d`, 장애물 `s` 범위, 입력 및 검사 `d` 범위와
   적용 clearance를 기록합니다.

기본 planning 주기 25ms와 3회 확인은 약 75ms입니다. hard 충돌과 경로 끝 소진, 트랙 경계 및
기하 오류에는 이 지연을 적용하지 않습니다. 재계획 시 회피 진입 전에는 반대편 전환을 허용하고,
진입 후에는 같은 방향만 평가합니다. 허용된 방향의 대체 경로도 불가능하면
`raceline_static_safe_stop`을 latch합니다. safe-stop은
`safe_stop_release_cycles`회 연속으로 회피 가능 또는 장애물 없음이 확인되어야 해제합니다.
기본 25ms 주기와 8회 설정에서는 0.2초입니다. safe-stop이 활성화된 동안 state machine은 그
경로의 `d=0` tail을 합류 완료로 해석하지 않고 `STATE_AVOID`를 유지합니다.

장애물을 지난 뒤 perception에서 물체가 사라져도 검증된 spline과 뒤쪽 global tail을 유지합니다.
ego가 실제 spline merge 지점에 도달하고
`|ego d| <= merge_lateral_tolerance_m`을 `merge_confirm_cycles` 동안 만족하면 기하학적 합류가
확인됩니다.

기하학적 합류만으로 commitment를 해제하지는 않습니다. 합류가 확인되면 전체 global waypoint를
원래 순서 그대로 한 번 포함하는 폐루프 handoff 경로로 교체합니다. 배열 시작점만 회전해 현재
ego가 마지막 `state_handoff_tail_ratio` 구간의 첫 부분에 위치하도록 하고
`ot_line=raceline_global_handoff`를 설정합니다. state machine은 이 표식을 받으면 고정 tail을
다시 만날 때까지 기다리지 않고 실제 ego가 global line에 0.5초 동안 유지되는지만 확인합니다.
컨트롤러에는 충분한 전방 global 경로가 계속 제공됩니다.

현재 commitment에서 `/state`의 `STATE_AVOID`를 한 번 이상 확인한 뒤 `STATE_GLOBAL` 복귀가
발행될 때까지 이 non-empty 폐루프를 계속 발행합니다. `STATE_GLOBAL` 확인 후에만 빈
`/avoid_waypoints`를 발행합니다. 기본 `planning_period_ms=25`, `merge_confirm_cycles=15`의
기하 확인 시간은 0.375초입니다. handoff 중 waypoint 속도는
`state_handoff_speed_cap_mps` 이하로 제한합니다.

### 3.8 연속 장애물 maneuver 연결

현재 commitment에 포함되지 않은 장애물의 팽창된 Guard 앞면이 현재 spline의 실제 merge 뒤에서
시작하면 그 장애물은 **다음 maneuver 군집**으로 분류합니다. 현재 commitment의 충돌 검사는
ego부터 실제 merge까지만 이 군집을 적용하고, merge 뒤에 controller 시야 확보용으로 덧붙인
global tail은 현재 maneuver의 안전 소유 구간으로 보지 않습니다. 따라서 그 tail과 다음
장애물이 겹친다는 이유만으로 첫 maneuver가 조기 safe-stop으로 바뀌지 않습니다.

반대로 장애물 Guard가 merge 전에서 시작하거나 merge와 겹치면 **현재 maneuver 장애물**입니다.
이 경우 현재 commitment를 즉시 다시 검사하고, 충돌하면 기존 방향을 유지한 재계획 또는 진입
전 반대 방향 재평가를 수행합니다.

다음 maneuver 군집은 첫 회피를 수행하는 동안에도 기존 최초 관측 조건, 즉 각 ID의 실제
`/static_obs` 3회 관측, 최소 `initial_observation_min_duration_sec=0.15초`와 최대
`initial_observation_max_wait_sec=0.35초`를 사용해 동시에 안정화합니다. 현재 장애물 Guard의
뒤쪽을 `chain_release_margin_m`만큼 완전히 지난 뒤 다음 군집이 안정화되어 있으면 다음 순서로
직접 연결합니다.

1. 완료한 군집 ID를 이번 연속 회피가 끝날 때까지 제외 목록에 넣습니다.
2. 이전 maneuver의 좌우 방향 잠금을 해제합니다.
3. 현재 측정된 `ego.s`와 `ego.d`를 새 spline의 시작점으로 고정해 좌우를 새로 평가합니다.
4. 안전한 이동 경로가 있으면 기존 commitment를 다음 경로로 원자적으로 교체하고
   `STATE_AVOID`를 유지합니다. 중간 빈 경로, global 경로, 불필요한 정지는 발행하지 않습니다.
5. 아직 안전한 다음 경로를 만들 수 없으면 현재 commitment를 merge까지 유지합니다. merge에
   도착하면 미리 누적한 관측을 그대로 승계해 다음 계획을 이어갑니다.

이미 `raceline_global_handoff`를 발행 중이어도 새 blocking 군집이 들어오면 handoff를
선점합니다. 모든 미완료 blocking 군집이 사라진 뒤에만 최종 global handoff를 완료합니다.
안전한 이동 경로가 전혀 없을 때에는 먼저 현재 committed geometry 위에서 충돌 전까지
감속합니다. 그 prefix조차 만들 수 없을 때만 현재 `ego.d`를 유지하는 zero-speed hold를
최악 상황의 마지막 수단으로 사용합니다.

### 3.9 장애물 센서 stale과 다음 랩 기억

유효한 `/static_obs`를 한 번 이상 받은 뒤
`obstacle_stale_timeout_sec` 동안 새 메시지가 없으면 planner는 **degraded perception
mode**로 전환합니다. 이때 stale을 장애물이 사라졌다는 뜻으로 해석하지 않습니다.

1. 진행 중인 검증된 회피 spline과 방향 commitment를 그대로 유지합니다.
2. Frenet odometry로 merge 도달을 계속 확인하고, 합류 뒤에는 평소와 같은
   `raceline_global_handoff`를 발행합니다.
3. `/state`가 `STATE_AVOID`를 거쳐 `STATE_GLOBAL`로 복귀하면 회피 출력은 정상적으로
   종료합니다. 센서 stale만으로 차량을 정지시키지 않습니다.
4. 마지막 유효 장애물 스냅샷은 지우지 않습니다. 센서가 계속 끊긴 채 다음 랩에서 같은
   장애물이 lookahead에 들어오면, 새 관측을 기다리는 준비 감속 없이 저장된 uncertainty
   Guard로 즉시 회피 계획을 다시 만듭니다.
5. frame이 잘못된 장애물 배열은 무시하되 기존 기억은 보존합니다. 올바른 frame의 새 배열이
   도착하면 빈 배열도 유효한 최신 관측으로 보고 저장된 기억을 교체합니다.

정지는 센서 stale 자체가 아니라 저장된 장애물에 대해 양쪽 회피와 검증된 정지 prefix가 모두
불가능하거나, 충돌 위험이 발생한 경우에만 사용합니다. Frenet odometry가
`odometry_stale_timeout_sec`를 넘겨 차량 위치를 신뢰할 수 없는 경우는 최악 상황으로 분류해
마지막으로 알려진 `s/d`에서 모든 속도가 0인 emergency hold를 발행합니다. 이때도 기존
commitment는 지우지 않으므로 odometry가 회복되면 다시 검증한 뒤 이어갈 수 있습니다.

## 4. 토픽과 메시지

| 구분 | 기본 토픽 | 메시지 | 설명 |
|---|---|---|---|
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` | 순서를 고정할 글로벌 Race Line |
| 구독 | `/static_obs` | `f110_msgs/msg/ObstacleArray` | Layer 2 authoritative Frenet 경계와 `s_var/d_var` 중심 위치 분산 |
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | `x=s`, `y=d` ego 상태 |
| 구독 | `/state` | `f110_msgs/msg/StateMachine` | AVOID 진입 및 GLOBAL handoff 완료 확인 |
| 발행 | `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` | ego부터 글로벌 합류 뒤 lookahead까지의 회피 세그먼트 |
| 발행 | `/local_planning/path` | `nav_msgs/msg/Path` | RViz용 현재 안전 경로 |
| 발행 | `/local_path` | `nav_msgs/msg/Path` | 기존 시각화 호환 토픽 |
| 발행 | `/local_planning/markers` | `visualization_msgs/msg/MarkerArray` | 경로, spline 제어점, 입력 정적 장애물 AABB |

`/avoid_waypoints.ot_line`은 최초 군집 관측용 감속 경로일 때 `raceline_static_prepare`, 정상
회피일 때 `raceline_local_d_offset_spline`, 허용된 회피 방향이 모두 막힌 감속 경로일 때
`raceline_static_safe_stop`입니다.

## 5. 주요 파라미터

모든 운영값은 `config/local_planning.yaml`에 있습니다.

- 검출: `detection_lookahead_m`, `obstacle_cluster_gap_m`
- 장애물 여유: `obstacle_longitudinal_padding_m`, `obstacle_clearance_m`, `blocking_margin_m`
- 차체/트랙: `vehicle_half_width_m`, `boundary_margin_m`(기본 0.13 m),
  `fallback_track_half_width_m`
- spline 제어점: `pre_apex_distances_m`, `post_apex_distances_m`
- spline 길이: `transition_distance_scales`, `outside_line_transition_scale`
- 합류 후 시야: `post_merge_lookahead_m`, `post_merge_min_time_sec`
- 목표 제한: `minimum_target_offset_m`, `maximum_target_offset_m`,
  `commitment_clearance_reserve_m`
- 최초 관측: `initial_observation_count`, `initial_observation_min_duration_sec`,
  `initial_observation_max_wait_sec`
- 불확실성 Guard: `uncertainty_sigma_scale`, `uncertainty_min_longitudinal_margin_m`,
  `uncertainty_min_lateral_margin_m`
- commitment 충돌 확인: `commitment_soft_violation_confirm_cycles`,
  `hard_collision_margin_m`
- 방향 잠금: `commitment_lock_lateral_threshold_m`, `commitment_lock_longitudinal_m`
- maneuver 연결: `chain_release_margin_m`
- 기하 제한: `maximum_lateral_slope`, `maximum_curvature_radpm`,
  `maximum_curvature_rate_radpm2`
- 실패 시 정지: `safe_stop_buffer_m`, `safe_stop_deceleration_mps2`,
  `safe_stop_release_cycles`

정상 회피 경로는 글로벌 waypoint의 `vx_mps`를 그대로 유지하고 변경된 경로의 heading,
curvature와 `ax_mps2`만 다시 계산한다. 좌우 경로가 모두 안전하지 않을 때 생성하는 safe-stop
경로만 `safe_stop_deceleration_mps2`에 따라 속도를 낮춘다.
- 입력 freshness: `obstacle_stale_timeout_sec`, `odometry_stale_timeout_sec`
  - obstacle stale: 마지막 유효 경로와 장애물 기억으로 주행/다음 랩 계획 지속
  - odometry stale: 마지막 위치에서 zero-speed hold
- 합류 확인: `merge_lateral_tolerance_m`, `merge_confirm_cycles`, `state_topic`,
  `state_handoff_tail_ratio`, `state_handoff_speed_cap_mps`
- 토픽과 프레임: `*_topic`, `frame_id`

## 6. 빌드와 테스트

저장소의 `~/.zshrc` 빌드 별칭은 symlink-install과 Ninja를 사용합니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
cb --packages-select local_planning
source install/setup.zsh
colcon test --packages-select local_planning --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/local_planning
```

`test/test_raceline_spline.cpp`는 다음을 검사합니다.

1. 비단조 글로벌 `s_m` 거부
2. 글로벌 waypoint의 `s`와 순서를 보존한 d-offset
3. 한쪽 트랙 폭이 부족할 때 반대쪽 선택
4. 회피 진입 전 반대편 재평가와 진입 후 commitment 방향 고정
5. reserve가 적용된 목표, 작은 Frenet 경계 흔들림과 soft/hard 충돌 경계 구분
6. 준비 감속 경로와 전체 blocking cluster ID 전달
7. safe-stop buffer 안의 장애물에 준비 지연을 적용하지 않음
8. 글로벌 라인과 원본 clearance가 충분한 옆 장애물 무시
9. 양쪽이 막혔을 때 점진 정지와 짧은 정지 prefix
10. 회피 중 현재 `ego.d`를 유지하는 safe-stop
11. 기존 committed geometry 위에서 충돌 전에 감속하는 정지 prefix
12. merge 뒤 controller tail 충돌을 현재 commitment 충돌로 오판하지 않음
13. 현재 `ego.d`에서 다음 maneuver spline으로 연속 연결
14. 0속도 emergency hold와 랩 경계 장애물 처리
15. 가까운 반대편 스네이크 branch로 점프하지 않음

`test/test_obstacle_guard.cpp`는 `s_var/d_var`의 표준편차 확장, 최소 크기 마진, 폐루프 `s` wrap,
고정 Guard 안의 작은 중심 이동 허용, 누적 이동의 Guard 이탈, 잘못된 분산의 fallback을 검사합니다.

`test/frenet_static_pipeline_test.py`는 준비 감속 뒤 같은 ID의 detector-style Frenet 경계를
±1cm 흔들고 `s_var/d_var`를 제공해도 10회 연속 동일 commitment가 발행되는지 확인합니다.
Cartesian AABB-to-Frenet 투영 단위 테스트는 좌표변환의 소유자인
`obstacle_detector/test/test_aabb_frenet_projector.cpp`에 있습니다.
`test/initial_cluster_stabilization_pipeline_test.py`는 첫 검출 0.1초 뒤 같은 군집에 ID를 하나
추가해 최소 0.15초 및 실제 토픽 3회 관측을 모두 거친 뒤, 넓어진 군집을 반영한 방향으로 최초
commitment가 만들어지는지 확인합니다.
`test/soft_violation_confirmation_pipeline_test.py`는 한두 cycle의 soft 충돌에서 고정 경로를
유지하고, 지속되는 soft 충돌만 3회 확인 뒤 같은 방향으로 재계획하는지 검사합니다.
`test/pre_engagement_side_switch_pipeline_test.py`는 ego가 회피 진입
기준 전일 때 기존
방향을 막아 반대편 경로로 직접 교체되는지 확인합니다. `test/safe_stop_latch_pipeline_test.py`는
`local_planner_node`, `state_machine_node`, `wpnt_publisher` 사이에서 safe-stop이
`STATE_AVOID`/`local_waypoints`에 유지되고 연속 안전 판정 뒤에만 회피로 복귀하는지 확인합니다.
`test/sequential_obstacle_handoff_pipeline_test.py`는 첫 장애물은 왼쪽, 두 번째 장애물은 오른쪽만
통과할 수 있게 만들어 두 maneuver 사이에 global handoff나 빈 경로가 없고, 두 번째 계획에서
첫 번째 방향 잠금이 해제되는지 확인합니다. 같은 스크립트에 `--during-handoff`를 주면 두 번째
장애물을 global handoff 발행 뒤에 투입해 handoff 선점도 확인합니다.
`test/post_merge_tail_chaining_pipeline_test.py`는 두 번째 장애물이 첫 경로의 merge 뒤
controller tail에 놓여도 첫 경로를 safe-stop으로 바꾸지 않고, 첫 장애물을 지난 뒤 현재
`ego.d`에서 두 번째 회피 경로로 직접 연결되는지 확인합니다.
`test/stale_obstacle_memory_pipeline_test.py`는 첫 회피 commitment 뒤 `/static_obs` 발행을
중단해 stale timeout을 넘겨도 경로가 비지 않고 geometry가 유지되는지, merge 뒤 GLOBAL
handoff가 완료되는지, 센서가 계속 끊긴 다음 랩에도 마지막 장애물 스냅샷으로 다시 회피하는지
검사합니다.

## 7. 실행 방법

### 7.1 perception을 함께 실행

기본 launch는 `obstacle_detector`를 함께 실행합니다. local planning 전용 reference-map 서버를
`/local_planning/reference_map`에 올리고 detector의 지도 필터 입력을 그 토픽으로 remap합니다.
이 지도는 장애물이 미리 그려지지 않은 wall-only 지도여야 합니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch particle_filter_cpp mcl_launch.py mod:=sim map_name:=ifac_track use_rviz:=false
```

다른 터미널에서 local planning을 실행합니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch local_planning local_planning.launch.py
```

### 7.2 외부 perception 사용

이미 `/static_obs` 발행기가 실행 중이면 detector 포함을 끕니다.

```zsh
ros2 launch local_planning local_planning.launch.py \
  start_obstacle_detector:=false
```

시뮬레이션 clock을 쓰는 전체 파이프라인이면 `use_sim_time:=true`를 함께 지정합니다.

```zsh
ros2 launch local_planning local_planning.launch.py use_sim_time:=true
```

## 8. 실행 확인

글로벌 플래너, Frenet odometry, perception이 먼저 준비된 상태에서 확인합니다.

```zsh
ros2 topic echo /avoid_waypoints --once
ros2 topic echo /local_planning/path --once
ros2 topic hz /local_planning/markers
```

RViz에서 `/local_planning/markers`를 추가하면 초록 선은 검증된 spline, 주황 선은 safe stop,
보라색 점은 spline 제어점, 빨간 직육면체는 detector가 현재 관측 metadata로 제공한 정적 장애물
Cartesian AABB입니다. `has_cartesian=false`인 predicted-only 객체는 marker를 만들지 않지만
Frenet 경계는 계속 계획 입력으로 사용할 수 있습니다.

local planner가 실제 사용하는 Frenet 장애물 영역을 그대로 확인하려면 detector의
`/static_obs/markers`를 추가합니다. 이 토픽은 최종 `/static_obs`의
`s_start/s_end/d_right/d_left`에서 생성되며 predicted-only 객체도 옅은 테두리로 표시합니다.

Frenet 입력 계약 확인:

```bash
python3 src/local_planning/test/frenet_static_pipeline_test.py \
  --waypoints-csv /path/to/global_waypoints.csv
```

별도 터미널에서 `local_planner_node`가 실행 중이어야 합니다. 테스트는 detector-style Frenet
경계를 넣고 `/avoid_waypoints`의 모든 `x_m/y_m`이 유한하며 횡방향 회피가 실제로 생성됐는지
확인합니다.

실제 detector 연결을 포함한 전체 경로는 두 노드를 실행한 상태에서 다음으로 확인한다.

```bash
python3 src/local_planning/test/static_obs_pipeline_test.py
```

이 테스트는 원형 글로벌 경로, free map, ego odometry, TF와 정적 장애물이 있는 LaserScan을 발행하고,
`obstacle_detector`가 유효한 Cartesian AABB와 이에 대응하는 Frenet 경계를 `/static_obs`에
만든 뒤 `local_planning`이 그 Frenet 경계로 횡방향 `/avoid_waypoints`를 만드는지 확인한다.

## 9. 전체 파이프라인 영향

perception 메시지는 유지하고, local planner와 state machine 사이의 `ot_line` 계약에 준비
감속 표식을 추가했습니다.

- `state_machine`: `raceline_static_prepare`를 합류 완료로 해석하지 않고 AVOID를 유지합니다.
- `wpnt_publisher`: `STATE_AVOID`일 때 기존처럼 `/avoid_waypoints`를 `/local_waypoints`로 중계합니다.
- `obstacle_detector`: Layer 2 `/static_obs`의 `f110_msgs/msg/ObstacleArray`에
  authoritative `s_start/s_end/d_right/d_left`와 `is_static=true`를 채워 발행합니다. 현재 visible
  객체는 같은 footprint의 `has_cartesian=true`, Cartesian 중심/AABB/radius도 함께 제공합니다.
  local planner는 Frenet 경계만 회피 형상으로 사용합니다.
- 회피 결과: `/avoid_waypoints` 각 점의 `x_m/y_m`은 map-frame Cartesian 좌표입니다.

`f110_msgs` 형식과 토픽은 변경하지 않았습니다.
