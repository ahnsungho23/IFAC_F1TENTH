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
2. `/static_obs`의 map-frame Cartesian AABB인 `x_min/x_max/y_min/y_max`를 사용합니다.
   AABB 중심을 CLCS로 투영해 현재 트랙 branch를 고정합니다. 종방향 범위와 장애물 반대편 경계는
   중심 접선에서 회전한 네 꼭짓점으로 만들고, Race Line을 향한 경계는 해당 branch의 실제
   글로벌 waypoint 선분들과 AABB 네 면 사이의 최단거리로 계산합니다. 따라서 코너에서도 중심
   접선 하나의 근사값이 아니라 곡선을 따라 가장 가까운 면의 `|d|`가 blocking 판정에 들어갑니다.
   enclosing-circle `radius`는 회피 형상에 사용하지 않습니다.
3. 한 점 apex가 아니라 장애물 군집의 앞·뒤에서 목표 `d`를 유지해 긴 정적 장애물도 처리합니다.
4. 글로벌 waypoint 자체를 출력 표본으로 사용해 Race Line의 위상 순서를 강제합니다.
5. 좌우 모두 불가능하면 장애물 앞 감속 경로를 발행합니다.
6. 이동 후 heading, curvature, velocity, acceleration을 다시 계산합니다.

## 3. 동작 원리

### 3.1 입력 준비

1. `/global_waypoints`의 모든 값이 유한하고 `s_m`이 엄격히 증가하는지 검사합니다.
2. 마지막 `s_m`과 waypoint 중앙 간격으로 폐루프 트랙 길이를 구합니다.
3. Frenet odometry의 `position.x`를 ego `s`, `position.y`를 ego `d`로 읽습니다.
4. perception 장애물 중 `is_static=true`이거나 속도가 `static_speed_threshold_mps` 이하인 것만
   남깁니다.
5. AABB 값이 유한하고 `x_min <= x_max`, `y_min <= y_max`이며 대각선 길이가 0보다 큰지
   검사합니다. 조건을 만족하지 않거나 중심의 CLCS 투영이 실패하면 해당 장애물을 제외합니다.

### 3.2 가장 가까운 정적 장애물 군집

1. ego 앞 `detection_lookahead_m` 안의 장애물 상자를 폐루프 `s`로 펼칩니다.
2. 장애물 상자를 종방향 `obstacle_longitudinal_padding_m`, 횡방향
   `obstacle_clearance_m`만큼 팽창합니다.
3. 원본 장애물의 가장 가까운 면에서 구한 곡선 기준 `|d|`가 글로벌 `d=0`의 차량 envelope
   (`vehicle_half_width_m + blocking_margin_m`) 안에 들어올 때만 blocking 장애물로 봅니다.
   경로 생성용 `obstacle_clearance_m`을 blocking 판정에 다시 더하지 않습니다.
4. `obstacle_cluster_gap_m`보다 가까운 후속 장애물은 같은 기동으로 처리합니다.

### 3.3 최초 군집 안정화

처음 blocking 장애물이 들어오면 곧바로 좌우 spline을 확정하지 않습니다. 먼저 글로벌 `d=0` 위의
검증된 감속 prefix를 `ot_line=raceline_static_prepare`로 발행하면서 가장 가까운 군집을
`initial_cluster_stabilization_sec` 동안 관측합니다. 이 시간에는 같은 ID의 AABB를 Cartesian
합집합으로 누적하므로 순간적으로 작아진 검출 형상 때문에 여유가 줄지 않습니다.

인접한 새 ID가 군집에 추가되거나 합집합 경계가
`cluster_envelope_change_threshold_m` 이상 넓어지면 안정화 시간을 다시 셉니다. 계속 형상이
변하더라도 `initial_cluster_max_wait_sec`에 도달하면 그동안 누적한 가장 보수적인 AABB로
계획을 시작합니다. 장애물이 이미 `safe_stop_buffer_m` 안에 있어 감속 prefix조차 만들 수
없으면 안정화를 기다리지 않고 즉시 zero-speed safe-stop을 latch합니다.

### 3.4 좌우 목표 d 계산

- 왼쪽 후보: 군집의 가장 큰 `d_left + clearance + commitment_clearance_reserve_m`
- 오른쪽 후보: 군집의 가장 작은 `d_right - clearance - commitment_clearance_reserve_m`

두 후보를 모두 만들며, 각 글로벌 waypoint의 허용 중심 범위
`[-d_right + 차량반폭 + boundary_margin, d_left - 차량반폭 - boundary_margin]`를 벗어나면
폐기합니다. 안정화가 끝난 최초 계획에서는 양쪽을 비교합니다. commitment 뒤 기존 경로가
위험해졌더라도 ego가 `commitment_lock_lateral_threshold_m`만큼 횡이동하거나
`commitment_lock_longitudinal_m`만큼 전진하기 전이라면 반대쪽도 다시 평가할 수 있습니다.
둘 중 하나에 도달해 실제 회피에 진입한 뒤에는 진행 중 갑자기 반대편으로 꺾지 않도록 방향을
고정합니다. 기본 reserve 0.05m는 경로 검증에 쓰는 AABB보다 목표를 5cm 더 바깥에 놓아 작은
측정 흔들림을 흡수합니다.

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

안전 경로가 선택되면 방향과 경로 geometry를 고정합니다. 매 planning tick에는 먼저 현재 ego
앞에 남은 committed 경로를 최신 AABB로 재검증합니다. 여전히 안전하면 좌우 spline 후보를 다시
만들지 않고 같은 경로를 발행합니다. 따라서 같은 ID의 AABB나 인접 cluster 구성이 조금 흔들려도
안전 여유 안에서는 `target_d`와 출력 경로가 바뀌지 않습니다.

기존 경로가 최신 AABB에 대해 위험해졌을 때만 대체 경로를 계산합니다. 위의 회피 진입 조건
전에는 반대편 전환을 허용하고, 진입 후에는 같은 방향만 평가합니다. 허용된 방향의 대체 경로도
불가능하면 즉시 `raceline_static_safe_stop`을 latch합니다. safe-stop 진입은 지연하지 않으며,
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

## 4. 토픽과 메시지

| 구분 | 기본 토픽 | 메시지 | 설명 |
|---|---|---|---|
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` | 순서를 고정할 글로벌 Race Line |
| 구독 | `/static_obs` | `f110_msgs/msg/ObstacleArray` | `obstacle_detector` Layer 2 정적 장애물 Cartesian AABB |
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | `x=s`, `y=d` ego 상태 |
| 구독 | `/state` | `f110_msgs/msg/StateMachine` | AVOID 진입 및 GLOBAL handoff 완료 확인 |
| 발행 | `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` | ego부터 글로벌 합류 뒤 lookahead까지의 회피 세그먼트 |
| 발행 | `/local_planning/path` | `nav_msgs/msg/Path` | RViz용 현재 안전 경로 |
| 발행 | `/local_path` | `nav_msgs/msg/Path` | 기존 시각화 호환 토픽 |
| 발행 | `/local_planning/markers` | `visualization_msgs/msg/MarkerArray` | 경로, spline 제어점, 입력 정적 장애물 AABB |
| 선택 발행 | `/local_waypoints` | `f110_msgs/msg/WpntArray` | 단독 실행 옵션. 기본값은 꺼짐 |

`/avoid_waypoints.ot_line`은 최초 군집 관측용 감속 경로일 때 `raceline_static_prepare`, 정상
회피일 때 `raceline_local_d_offset_spline`, 허용된 회피 방향이 모두 막힌 감속 경로일 때
`raceline_static_safe_stop`입니다.

## 5. 주요 파라미터

모든 운영값은 `config/local_planning.yaml`에 있습니다.

- 검출: `detection_lookahead_m`, `obstacle_cluster_gap_m`
- 장애물 여유: `obstacle_longitudinal_padding_m`, `obstacle_clearance_m`, `blocking_margin_m`
- 차체/트랙: `vehicle_half_width_m`, `boundary_margin_m`, `fallback_track_half_width_m`
- spline 제어점: `pre_apex_distances_m`, `post_apex_distances_m`
- spline 길이: `transition_distance_scales`, `outside_line_transition_scale`
- 합류 후 시야: `post_merge_lookahead_m`, `post_merge_min_time_sec`
- 목표 제한: `minimum_target_offset_m`, `maximum_target_offset_m`,
  `commitment_clearance_reserve_m`
- 최초 안정화: `initial_cluster_stabilization_sec`, `initial_cluster_max_wait_sec`,
  `cluster_envelope_change_threshold_m`
- 방향 잠금: `commitment_lock_lateral_threshold_m`, `commitment_lock_longitudinal_m`
- 기하 제한: `maximum_lateral_slope`, `maximum_curvature_radpm`,
  `maximum_curvature_rate_radpm2`
- 속도: `avoidance_speed_scale`, `maximum_lateral_accel_mps2`,
  `maximum_longitudinal_accel_mps2`, `maximum_longitudinal_decel_mps2`
- 실패 시 정지: `safe_stop_buffer_m`, `safe_stop_deceleration_mps2`,
  `safe_stop_release_cycles`

현재 `config/local_planning.yaml`은 제어기 단독 감속 시험을 위해
`avoidance_speed_scale=1.0`으로 설정하고, 횡·종가속도 한계를 `1000000.0`으로 높여
회피 경로의 속도 배율, 곡률 기반 속도 캡, 종방향 속도 재프로파일을 실질적으로
비활성화한다. 경로 형상의 곡률·곡률 변화율 검증과 안전정지 속도 프로파일은 그대로 유지된다.
- 입력 freshness: `obstacle_stale_timeout_sec`, `odometry_stale_timeout_sec`
- 합류 확인: `merge_lateral_tolerance_m`, `merge_confirm_cycles`, `state_topic`,
  `state_handoff_tail_ratio`, `state_handoff_speed_cap_mps`
- 토픽과 프레임: `*_topic`, `frame_id`

## 6. 빌드와 테스트

저장소의 `~/.zshrc` 빌드 별칭은 symlink-install과 Ninja를 사용합니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
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
5. reserve가 적용된 목표와 작은 AABB 흔들림에서 기존 경로 유지
6. 준비 감속 경로와 전체 blocking cluster ID 전달
7. safe-stop buffer 안의 장애물에 준비 지연을 적용하지 않음
8. 글로벌 라인과 원본 clearance가 충분한 옆 장애물 무시
9. 양쪽이 막혔을 때 점진 정지와 짧은 정지 prefix
10. 0속도 emergency hold
11. 랩 경계 장애물 처리
12. 가까운 반대편 스네이크 branch로 점프하지 않음

`test/test_aabb_frenet_projector.cpp`는 다음을 검사합니다.

1. 긴 직사각형의 종방향 길이와 좁은 횡방향 폭이 서로 독립적으로 보존됨
2. 트랙 접선이 map 축과 회전된 경우 네 꼭짓점의 Frenet envelope가 올바름
3. 곡선 구간에서 AABB 중심 접선이 아니라 실제 Race Line과 가장 가까운 면의 `|d|`가 사용됨
4. 순서가 뒤집혔거나 점 크기인 잘못된 AABB가 거부됨

`test/cartesian_static_pipeline_test.py`는 준비 감속 뒤 같은 ID의 Cartesian AABB를 ±1cm
흔들어도 10회 연속 동일 commitment가 발행되는지 확인합니다.
`test/initial_cluster_stabilization_pipeline_test.py`는 첫 검출 0.1초 뒤 같은 군집에 ID를 하나
추가해 안정화 타이머가 재시작되고, 넓어진 군집을 반영한 방향으로 최초 commitment가 만들어지는지
확인합니다. `test/pre_engagement_side_switch_pipeline_test.py`는 ego가 회피 진입 기준 전일 때 기존
방향을 막아 반대편 경로로 직접 교체되는지 확인합니다. `test/safe_stop_latch_pipeline_test.py`는
`local_planner_node`, `state_machine_node`, `wpnt_publisher` 사이에서 safe-stop이
`STATE_AVOID`/`local_waypoints`에 유지되고 연속 안전 판정 뒤에만 회피로 복귀하는지 확인합니다.

## 7. 실행 방법

### 7.1 perception을 함께 실행

기본 launch는 `obstacle_detector`를 함께 실행합니다. local planning 전용 reference-map 서버를
`/local_planning/reference_map`에 올리고 detector의 지도 필터 입력을 그 토픽으로 remap합니다.
이 지도는 장애물이 미리 그려지지 않은 wall-only 지도여야 합니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch particle_filter_cpp mcl_launch.py mod:=sim map_name:=ifac_track use_rviz:=false
```

다른 터미널에서 local planning을 실행합니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
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
보라색 점은 spline 제어점, 빨간 직육면체는 local planner가 입력으로 받은 정적 장애물
Cartesian AABB입니다.

Cartesian 입출력 통합 확인:

```bash
python3 src/local_planning/test/cartesian_static_pipeline_test.py \
  --waypoints-csv /path/to/global_waypoints.csv
```

별도 터미널에서 `local_planner_node`가 실행 중이어야 합니다. 테스트는 map-frame AABB를 넣고
`/avoid_waypoints`의 모든 `x_m/y_m`이 유한하며 횡방향 회피가 실제로 생성됐는지 확인합니다.

실제 detector 연결을 포함한 전체 경로는 두 노드를 실행한 상태에서 다음으로 확인한다.

```bash
python3 src/local_planning/test/static_obs_pipeline_test.py
```

이 테스트는 원형 글로벌 경로, free map, ego odometry, TF와 정적 장애물이 있는 LaserScan을 발행하고,
`obstacle_detector`가 유효한 Cartesian `/static_obs`를 만든 뒤 `local_planning`이 횡방향
`/avoid_waypoints`를 만드는지 확인한다.

## 9. 전체 파이프라인 영향

perception 메시지는 유지하고, local planner와 state machine 사이의 `ot_line` 계약에 준비
감속 표식을 추가했습니다.

- `state_machine`: `raceline_static_prepare`를 합류 완료로 해석하지 않고 AVOID를 유지합니다.
- `wpnt_publisher`: `STATE_AVOID`일 때 기존처럼 `/avoid_waypoints`를 `/local_waypoints`로 중계합니다.
- `obstacle_detector`: Layer 2 `/static_obs`의 `f110_msgs/msg/ObstacleArray`에
  `has_cartesian=true`, Cartesian 중심/AABB/radius와 `is_static=true`를 채워 발행합니다.
  local planner는 이 중 AABB를 실제 회피 형상으로 사용하고 radius는 사용하지 않습니다.
- 회피 결과: `/avoid_waypoints` 각 점의 `x_m/y_m`은 map-frame Cartesian 좌표입니다.

`f110_msgs` 형식과 토픽은 변경하지 않았습니다.
