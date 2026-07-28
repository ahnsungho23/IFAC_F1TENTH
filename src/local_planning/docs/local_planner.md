# local_planner_node

## 1. 노드 목적

<<<<<<< HEAD
`local_planner_node`는 정적 장애물이 글로벌 Race Line을 막을 때만 로컬 회피 세그먼트를 만듭니다.
동적 상대 차량의 추월·추종은 `opponent_detector`의 `/overtake_waypoints`가 담당하고, 이 노드는
`/avoid_waypoints`만 발행합니다.
=======
`local_planner_node`는 `/map`의 정적 장애물과 글로벌 기준 경로의 간섭을 검사하고, 차량 앞에서 글로벌 경로로 다시 합류하는 Frenet lattice 회피 구간을 생성합니다. 양쪽 방향의 여러 횡오프셋과 전환 길이를 평가해 최저 비용 경로를 선택하며, 기본 탐색이 실패하면 더 촘촘한 복구 lattice를 실행합니다. 복구도 실패할 때만 장애물 전의 충돌 없는 점진 감속 구간을 발행합니다.
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970

가장 중요한 설계 조건은 다음과 같습니다.

<<<<<<< HEAD
> 출력 경로는 항상 `/global_waypoints`의 진행 순서와 `s_m`을 유지하며, 각 점의 로컬 `d_m`만
> 바꾼다.
=======
1. `/map`을 받으면 내용 서명을 비교하고, 실제로 지도가 바뀐 경우에만 전체 점유 셀 mask와 8방향 연결요소를 다시 만듭니다.
2. 경로 차단 판단에는 컴포넌트 면적과 무관하게 전체 OccupancyGrid를 사용합니다. 연결요소의 크기 분류는 장애물 그룹 중심 시각화와 로그에만 사용하므로 벽에 붙거나 2.0 m²보다 큰 장애물도 검출됩니다.
3. `/car_state/frenet/odom`의 Frenet `s`, `d`가 트랙 범위 안에서 연속적으로 들어오는지 확인합니다. 시작 직후에는 3개 연속 샘플을 요구합니다. 주행 중 순간적인 이상 샘플은 설정 횟수만큼 무시하고, 짧은 입력 공백에는 마지막 충돌 검증 경로를 제한시간 동안 유지합니다. 안정화 후 `s`와 글로벌 웨이포인트의 `s_m`을 비교해 현재 시작점을 찾습니다. CLCS 내부 `child_frame_id`는 글로벌 인덱스로 사용하지 않습니다.
4. 계획 horizon의 각 sampled `s`에 트랙 경계, 전체 점유 셀, 차량 반폭/보수적 원형, 위치추정 여유와 추가 안전 여유를 적용한 Frenet safe corridor를 만듭니다. `blocked_intervals`, 통과 가능한 `feasible_intervals`, 좌우 통과 가능 여부를 명시적으로 저장합니다. 종방향 검색은 고정 폭이 아니라 waypoint 반간격+차체 종방향 길이+셀 반대각선으로 자동 확장하고 start/finish 인덱스를 감습니다. 지도와 글로벌 경로가 동일하면 waypoint별 계산 결과를 재사용하고, 아직 계산하지 않은 전방 단면만 새로 검사합니다. 지도 또는 경로가 바뀌면 이 캐시를 전부 폐기합니다.
5. 연속 `detection_confirm_cycles`회 검출된 경우 가장 가까운 종방향 장애물 군집을 고르고 좌우 여유 공간을 비교해 선호 방향을 정합니다. 서로 떨어진 다음 장애물은 현재 군집을 통과한 뒤 연속해서 다시 계획합니다. 기본 정적 지도 설정은 첫 검출 주기에 확정합니다.
6. 좌우 양쪽에서 `lattice_lateral_samples`개의 횡오프셋과 `lattice_transition_scales`의 전환 길이를 조합해 실제 기준선 호길이 `s`에 대한 5차 Frenet `d(s)` 후보를 만듭니다. 장애물 구간 전체에 공통인 `d`가 없지만 각 단면에는 통과 공간이 있는 곡선·대각선 통로에서는 중간 safe-corridor knot를 따라 bounded beam search를 수행합니다. 내부 knot의 기울기는 PCHIP 규칙으로 연결해 조각마다 직선화되며 곡률이 튀는 현상을 막습니다. waypoint 간격이 불균일해도 거리 기준 전환 형상은 유지됩니다.
7. 각 후보 좌표에서 heading과 곡률을 다시 계산하고, 곡률 기반 속도 상한과 횡가속도 한계를 적용합니다.
8. quintic `d(s)`를 점별 clamp하지 않고 safe corridor 밖 후보 전체를 제거합니다. 웨이포인트와 그 사이 보간 pose에서 oriented rectangle footprint와 기존 보수적 원형을 원본 OccupancyGrid로 검사합니다. 점유 셀, unknown 정책 위반, 지도 밖 footprint와 트랙 경계 위반은 hard reject입니다. 같은 보간점의 clearance를 비용에도 포함합니다.
9. 안전한 후보의 공간 횡저크 적분 `J_s=∫d'''(s)²ds`, 전환 거리 `S`, clearance, 기준선 이탈, 곡률, 거리당 곡률 변화율, 추가 길이, 속도 손실을 비용으로 평가하고 최저 비용 경로를 선택합니다. `J_s`와 `S`의 균형은 Werling 등(ICRA 2010)의 저속 Frenet 비용식을 반영하며, 너무 급한 전환과 지나치게 긴 전환을 동시에 억제합니다. 곡률 변화율 상한을 넘는 좌우 조향 반전 후보는 제거합니다.
10. 최초로 선택한 안전 경로를 저장하고, 현재 위치부터 기존 합류점까지 모든 점과 선분을 현재 지도에서 최소 한 번 충돌 검사합니다. 같은 커밋과 같은 지도에서 통과한 구간은 검증 결과를 재사용합니다. `/map`이 변경되면 커밋과 검증 캐시를 즉시 폐기하므로 변경된 장애물을 이전 결과로 통과시키지 않습니다. 장애물이 검출 범위 뒤로 사라져도 합류 전에는 경로를 지우지 않으며, 막히거나 다음 장애물이 들어온 경우에만 lattice를 다시 계산합니다.
11. 기본 lattice 후보가 모두 실패하면 진입점부터 장애물 통과와 글로벌 경로 합류점까지 전 horizon의 safe corridor knot를 bounded beam search로 연결하는 복구 lattice를 실행합니다. 장애물 구간에서는 선택한 좌/우 통과 방향을 강제하고, 진입·합류 구간도 동일한 corridor와 점·선분 충돌 검사를 통과해야 합니다. 전구간 안내 후보가 없을 때만 기존 장애물 구간 조밀 샘플을 최후 fallback으로 검사하며 OccupancyGrid 충돌 반경은 완화하지 않습니다. 연속 장애물 합류 구간에서 복구도 실패하면 직전에 검증한 전체 회피 경로를 현재 위치부터 다시 점·선분·트랙 경계 검사하고, 안전한 prefix에 비가속 제동 프로파일을 적용합니다. 이 캐시는 `lattice_replan_brake_timeout_sec`까지만 사용합니다. 해당 prefix가 없으면 글로벌 경로에서 장애물 전까지 충돌 없는 부분을 잘라 점진 감속 구간을 발행합니다. 두 구간 모두 만들 수 없고 제한된 제동 경로 유지시간도 끝났을 때만 빈 경로를 발행합니다. 5차 smoothstep은 별도 fallback이 아니라 각 lattice 후보의 횡전환 함수로만 사용됩니다.
12. 회피경로가 확정된 동안에는 global 중심선 `d=0`뿐 아니라 committed 경로의 waypoint별 `d`도 safe corridor와 비교합니다. 다음 장애물이 중심선을 비켜가더라도 현재 횡이동·합류 경로를 막으면 새로운 충돌 군집으로 판정하고 merge 전에 재계획합니다.
12. 안전한 후보의 실제 합류 인덱스를 보존한 채, 합류점 뒤의 충돌 없는 글로벌 waypoint를 `lattice_post_merge_lookahead_wpnts`개까지 추가합니다. 설계 합류점을 지나도 실제 차량 `|d|`가 허용 범위 밖이면 같은 글로벌 후속 구간을 계속 발행하고, 횡방향 복귀가 확인된 뒤에만 GLOBAL로 전환합니다.
13. 완성된 구간을 `/avoid_waypoints`와 `/local_planning/path`로 발행합니다. 일시적으로 장애물 검출이 누락되어도 해제 히스테리시스 동안 고정 경로를 유지합니다.
14. 회피가 커밋된 뒤에는 현재 방향에 유효 후보가 하나라도 있으면 같은 방향을 유지하며, 현재 방향이 모두 불가능할 때만 반대편으로 전환합니다.
15. 현재 merge까지 `lattice_preplan_before_merge_m` 이내이고 다음 장애물 군집이 보이면 현재 차량 위치에서 후속 경로를 선행 계산합니다. 군집은 글로벌 waypoint의 시작·끝으로 식별하며, 차량 진행으로 검출 시작점이 잘려도 끝점이 기존 commitment와 가까우면 같은 군집으로 간주해 50 ms마다 lattice를 다시 계산하지 않습니다. 전체 안전 검사와 시작점 연속성 검사를 통과한 경우에만 기존 커밋 경로를 교체합니다.
16. 실행기는 두 thread의 `MultiThreadedExecutor`를 사용합니다. 지도·장애물·계획 timer는 같은 mutually-exclusive callback group에서 grid 일관성을 유지하고, Frenet odometry는 별도 callback group에서 계속 수신합니다. odometry callback은 mutex로 보호된 최신 입력 버퍼만 갱신하며, 계획 주기는 무거운 계산 전에 pose를 한 번 복사해 고정된 snapshot으로 후보를 평가합니다.
16. RViz에는 컴포넌트 중심 외에도 안전 회랑 경계, 막힌 횡구간, 좌우 통과 가능 공간, 팽창 장애물 셀과 이유별 후보 거절 횟수를 표시합니다.
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970

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
2. `/perception/static_obstacles/cartesian`의 map-frame Cartesian 중심 `(x,y)`와 양의 `radius`를
   사용합니다. 중심은 CLCS로 `(s,d)`에 투영하고, 장애물을 원으로 간주해 s축과 d축 양쪽에 같은
   `radius`를 적용한 내부 Frenet 경계를 만듭니다.
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

### 3.2 가장 가까운 정적 장애물 군집

1. ego 앞 `detection_lookahead_m` 안의 장애물 상자를 폐루프 `s`로 펼칩니다.
2. 장애물 상자를 종방향 `obstacle_longitudinal_padding_m`, 횡방향
   `obstacle_clearance_m`만큼 팽창합니다.
3. 원본 장애물 상자가 글로벌 `d=0`의 차량 envelope와 겹칠 때만 blocking 장애물로 봅니다.
   경로 생성용 `obstacle_clearance_m`을 blocking 판정에 다시 더하지 않습니다.
4. `obstacle_cluster_gap_m`보다 가까운 후속 장애물은 같은 기동으로 처리합니다.

### 3.3 좌우 목표 d 계산

- 왼쪽 후보: 군집의 가장 큰 `d_left + clearance`
- 오른쪽 후보: 군집의 가장 작은 `d_right - clearance`

두 후보를 모두 만들며, 각 글로벌 waypoint의 허용 중심 범위
`[-d_right + 차량반폭 + boundary_margin, d_left - 차량반폭 - boundary_margin]`를 벗어나면
폐기합니다. 이미 회피 방향이 확정된 동안에는 그 방향을 먼저 사용하고, 안전 후보가 없어야만
반대 방향을 사용합니다.

### 3.4 로컬 d-offset spline

장애물 군집 앞에는 `pre_apex_distances_m`의 세 점을 `d=0`으로, 장애물 앞·뒤에는 목표 `d`를,
장애물 뒤에는 `post_apex_distances_m`의 세 점을 `d=0`으로 둡니다. 이 제어점 사이에 natural
cubic spline `d(s)`를 맞춥니다. spline의 반대편 overshoot는 제어점의 최소·최대 `d`로 제한합니다.

그다음 ego부터 merge 뒤 `post_merge_lookahead_m`까지의 글로벌 waypoint를 순서대로 복사합니다.
각 점은 다음 식으로만 이동합니다.

```text
x_local = x_global - d(s) * sin(psi_global)
y_local = y_global + d(s) * cos(psi_global)
```

`s_m`과 글로벌 진행 순서는 바뀌지 않습니다. 가까운 다른 트랙 조각을 검색하는 단계가 없으므로
비볼록 트랙에서도 잘못된 branch로 이동하지 않습니다.

### 3.5 안전성과 속도

완성된 후보는 다음 조건을 모두 통과해야 합니다.

1. 모든 점이 해당 글로벌 waypoint의 좌우 트랙 폭 안에 있음
2. 팽창된 정적 장애물 Frenet 상자와 겹치지 않음
3. `|dd/ds|`, Cartesian 곡률, 거리당 곡률 변화율 제한 만족
4. 이동된 geometry에서 계산한 곡률로 횡가속도 속도 상한 만족
5. 전·후방 속도 패스로 종가속도와 종감속도 제한 만족

짧은 전환이 실패하면 `transition_distance_scales` 순서대로 같은 목표 `d`를 더 긴 `s` 구간에
펼칩니다. 이것은 자유공간 후보 탐색이 아니라 동일한 글로벌 점에 적용하는 spline 길이 조정입니다.

좌우가 모두 실패하면 글로벌 `d=0` 위에서 장애물 앞 `safe_stop_buffer_m`까지의 충돌 없는 prefix를
만들고 마지막 속도를 0으로 둡니다. 안전한 prefix조차 없으면 빈 `/avoid_waypoints`를 발행합니다.

### 3.6 commitment와 합류

안전 경로가 선택되면 방향을 고정합니다. 장애물을 지난 뒤 perception에서 물체가 사라져도 spline
tail까지 경로를 유지합니다. ego가 tail에 도달하고 `|ego d| <= merge_lateral_tolerance_m`을
`merge_confirm_cycles` 동안 만족하면 commitment를 해제합니다. 기본 20 Hz에서 15회(0.75초)로
설정하여 `state_machine`의 기본 `enter_global_sec=0.5`보다 오랫동안 non-empty tail을 유지합니다.

## 4. 토픽과 메시지

| 구분 | 기본 토픽 | 메시지 | 설명 |
|---|---|---|---|
<<<<<<< HEAD
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` | 순서를 고정할 글로벌 Race Line |
| 구독 | `/perception/static_obstacles/cartesian` | `f110_msgs/msg/ObstacleArray` | 정적 장애물 x/y/s/d/radius |
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | `x=s`, `y=d` ego 상태 |
| 발행 | `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` | ego부터 글로벌 합류 뒤 lookahead까지의 회피 세그먼트 |
| 발행 | `/local_planning/path` | `nav_msgs/msg/Path` | RViz용 현재 안전 경로 |
| 발행 | `/local_path` | `nav_msgs/msg/Path` | 기존 시각화 호환 토픽 |
| 발행 | `/local_planning/markers` | `visualization_msgs/msg/MarkerArray` | 경로, spline 제어점, 정적 장애물 중심 |
| 선택 발행 | `/local_waypoints` | `f110_msgs/msg/WpntArray` | 단독 실행 옵션. 기본값은 꺼짐 |
=======
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` | 글로벌 기준 경로 |
| 구독 | `/map` | `nav_msgs/msg/OccupancyGrid` | 트랙 벽과 정적 장애물이 포함된 지도 |
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | 차량 Frenet `s`, `d` |
| 발행 | `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` | 차량→합류점 회피 구간과 충돌 없는 글로벌 후속 구간. 장애물이 없으면 빈 배열 |
| 발행 | `/local_waypoints` | `f110_msgs/msg/WpntArray` | 단독 실행 옵션이 켜졌을 때의 전방 구간 |
| 발행 | `/local_planning/path` | `nav_msgs/msg/Path` | RViz용 충돌 검증 완료 전방/회피 구간. 회피 실패 시 빈 경로 |
| 발행 | `/local_path` | `nav_msgs/msg/Path` | 호환용 전체 폐루프 경로 |
| 발행 | `/local_planning/markers` | `visualization_msgs/msg/MarkerArray` | 실제 장애물 중심과 현재 경로 |
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970

`/avoid_waypoints.ot_line`은 정상 회피일 때 `raceline_local_d_offset_spline`, 좌우가 모두 막힌
감속 경로일 때 `raceline_static_safe_stop`입니다.

## 5. 주요 파라미터

모든 운영값은 `config/local_planning.yaml`에 있습니다.

<<<<<<< HEAD
- 검출: `detection_lookahead_m`, `obstacle_cluster_gap_m`
- 장애물 여유: `obstacle_longitudinal_padding_m`, `obstacle_clearance_m`, `blocking_margin_m`
- 차체/트랙: `vehicle_half_width_m`, `boundary_margin_m`, `fallback_track_half_width_m`
- spline 제어점: `pre_apex_distances_m`, `post_apex_distances_m`
- spline 길이: `transition_distance_scales`, `outside_line_transition_scale`
- 합류 후 시야: `post_merge_lookahead_m`
- 목표 제한: `minimum_target_offset_m`, `maximum_target_offset_m`
- 기하 제한: `maximum_lateral_slope`, `maximum_curvature_radpm`,
  `maximum_curvature_rate_radpm2`
- 속도: `avoidance_speed_scale`, `maximum_lateral_accel_mps2`,
  `maximum_longitudinal_accel_mps2`, `maximum_longitudinal_decel_mps2`
- 실패 시 정지: `safe_stop_buffer_m`, `safe_stop_deceleration_mps2`
=======
- `lookahead_wpnt_num`: 경로 생성에 사용하는 전체 전방 웨이포인트 수
- `detection_lookahead_wpnt_num`: 실제 장애물 검출 범위
- `spline_window_margin_wpnts`: 장애물 전후 기준 전환 및 합류 길이
- `corner_spline_window_margin_wpnts`: 급코너 전용 전환 길이. 긴 Frenet 횡이동이 코너 안쪽 벽을 가로지르는 현상을 방지
- `safety_margin`, `avoid_offset`: 장애물 회피 오프셋
- `corner_avoid_offset`: 급코너의 최소 회피 오프셋. 측정된 장애물 경계가 더 큰 값을 요구하면 자동 증가
- `wall_margin`: 트랙 벽과 경로 중심 사이 여유. 내부적으로 차량 최종 안전반경보다 작아지지 않음
- `vehicle_radius`, `path_clearance_margin`: 기존 보수적 원형 점유 셀 충돌 검사 반경
- `vehicle_front_extent_m`, `vehicle_rear_extent_m`, `vehicle_width_m`: base_link 기준 oriented rectangle footprint
- `localization_margin_m`, `corridor_safety_margin_m`: footprint와 안전 회랑에 추가하는 위치추정/안전 여유
- `preserve_circular_collision_check`: 직사각형 검사와 함께 기존 원형 검사를 유지할지 여부
- `legacy_clamp_candidate_d`: 기존 점별 clamp 호환 옵션. 기본값 `false`는 회랑 밖 후보 전체를 폐기
- `unknown_cell_policy`: `treat_as_free`, `treat_as_occupied`, `reject_candidate` 중 unknown 셀 처리 정책
- `corner_tracking_margin`, `corner_curvature_threshold`: 급코너 장애물 검출에 추가하는 추종 오차와 적용 곡률
- `speed_reduction_ratio`: 일반 회피 구간에서 글로벌 기준 속도에 곱하는 배율
- `corner_speed_reduction_ratio`: 급코너 회피 구간의 속도 배율
- `post_obstacle_speed_recovery_ratio`: 장애물 마지막 지점을 지난 후 글로벌 속도로 복구할 최대 배율. 횡오프셋 감소량에 따라 5차 smoothstep으로 적용
- `lattice_max_longitudinal_accel_mps2`, `lattice_max_longitudinal_decel_mps2`: 전방/후방 속도 패스의 최대 종가속도와 감속도
- `lattice_lateral_samples`: 공통 safe corridor 목표 또는 공통 `d`가 없을 때 각 corridor knot 목표의 방향별 샘플 수
- `lattice_corridor_target_inset_m`: 목표 `d`를 공통 통로 경계에서 안쪽으로 넣는 수치 여유
- `lattice_corridor_validation_tolerance_m`: grid 양자화로 생기는 mm 단위 corridor 경계 오차 허용값. 최종 footprint 충돌 검사는 완화하지 않음
- `lattice_corridor_knot_stride_wpnts`, `lattice_corridor_beam_width`: 가변 통로의 knot 간격과 각 단계에서 유지할 최상위 부분 경로 수. 거친 간격으로 연결하지 못하면 waypoint 간격까지 자동 축소
- `lattice_narrow_corridor_width_threshold_m`, `lattice_narrow_corridor_transition_scales`: 좁은 공통 통로에서 차체 yaw를 줄이기 위해 추가하는 긴 전환 후보
- `lattice_obstacle_cluster_gap_wpnts`: 서로 다른 장애물을 나누는 충돌 샘플 간격. 기본 16개(약 1.6 m) 이내의 가까운 장애물은 한 기동으로 계획
- `lattice_lateral_step_m`: 이전 파라미터 파일 호환용 값. safe corridor 기반 목표 샘플링에는 사용하지 않음
- `lattice_transition_scales`, `lattice_min_transition_wpnts`: 종방향 전환 길이 후보
- `lattice_recovery_profile_limit`: 공통 횡방향 통로가 없을 때 recovery beam에서 정밀 평가할 서로 다른 전구간 profile 수
- `lattice_primary_search_budget_ms`, `lattice_recovery_search_budget_ms`: 차량이 이동하는 동안 오래된 후보를 과도하게 계산하지 않도록 제한하는 탐색 시간
- `lattice_max_result_pose_drift_m`: 계산 시작 이후 차량 진행량이 이 값을 넘거나 localization reset이 발생한 결과를 폐기하는 기준
- Frenet 위치가 불연속적으로 바뀌면 기존 commitment와 held path를 제거하고 새 위치에서 연속 샘플을 다시 확인합니다. 따라서 2D Pose Estimate 이후 이전 위치용 경로가 재사용되지 않습니다.
- `lattice_recovery_*`: 기본 후보 실패 시 사용하는 횡오프셋 수·간격, 전환 길이, 최소 전환 길이, 곡률·곡률 변화율 상한 배율
- `lattice_max_curvature_radpm`: 후보가 가질 수 있는 최대 곡률
- `lattice_max_curvature_rate_radpm2`: 거리당 곡률 변화율 상한. 급격한 좌우 조향 반전 후보를 제거
- `lattice_max_lateral_accel_mps2`, `lattice_speed_safety_factor`: 곡률 기반 속도와 횡가속도 한계
- `lattice_collision_sample_step_m`: 웨이포인트 사이 연속 충돌 검사 간격
- `lattice_preferred_clearance_m`: 후보 전체에서 선호하는 차량 중심-점유 셀 최소 거리
- `lattice_weight_spatial_lateral_jerk`, `lattice_weight_maneuver_length`: Werling 저속 Frenet 식의 공간 횡저크 적분과 전환 거리 가중치
- `lattice_weight_*`: 평균 clearance, 최소 clearance 부족분, 이탈, 곡률, 곡률 변화, 길이, 속도 손실 비용 가중치
- `lattice_weight_opposite_side`: 고정된 회피 방향 반대편 후보에 추가하는 비용
- `lattice_commit_path_until_clear`: 최초 안전 경로를 회피 상태 해제까지 고정하고 재검증할지 여부
- `lattice_preplan_before_merge_m`: 현재 merge 전부터 다음 장애물 군집을 선행 계획하는 거리
- `lattice_preplan_max_start_gap_m`: 선행 경로 시작점과 현재 차량 사이에 허용할 최대 거리
- `lattice_post_merge_lookahead_wpnts`: 실제 합류점 뒤에 추가할 충돌 검증 완료 글로벌 waypoint 수
- `lattice_merge_lateral_tolerance_m`: 실제 차량이 글로벌 라인에 복귀했다고 판단하는 Frenet `d` 허용값
- `lattice_merge_settle_max_wpnts`: 설계 합류점 이후 동일한 경로 소스를 유지할 수 있는 최대 waypoint 수
- `lattice_safe_stop_buffer_wpnts`, `lattice_safe_stop_deceleration_mps2`: lattice 전체 실패 시 감속 구간의 정지 여유와 제동 감속도
- `lattice_replan_brake_timeout_sec`: 연속 장애물 재계획 공백에서 직전 전체 회피 경로를 재검사해 사용할 수 있는 최대 시간
- `frenet_odom_confirm_cycles`: 시작 또는 좌표 점프 후 필요한 연속 정상 입력 수
- `frenet_odom_invalid_grace_cycles`: 주행 중 무시할 연속 이상 입력 수
- `frenet_odom_stale_timeout_sec`: Frenet 입력 만료 시간
- `frenet_odom_path_hold_timeout_sec`: 입력 이상 시 마지막 충돌 검증 경로의 최대 유지 시간
- `frenet_odom_max_s_jump_m`: 한 입력 주기에서 허용하는 최대 원형 `s` 변화량
- `frenet_odom_track_margin_m`: 트랙 폭 검사에 추가하는 위치 추정 여유
- `occupied_threshold`: 점유 셀 판정 임계값
- `obstacle_component_min_area_m2`, `obstacle_component_max_area_m2`: 시각화/로그용 컴포넌트 분류 범위. 검출 hard mask에는 영향 없음
- `detection_confirm_cycles`, `detection_clear_cycles`: 검출/해제 히스테리시스
- `timer_period_ms`: planning과 안전 검사의 계산 주기
- `debug_publish_period_ms`: RViz 진단 MarkerArray 발행 주기. planning 주기에는 영향을 주지 않음
- `frenet_odom_max_s_jump_m`, `frenet_odom_speed_jump_scale`, `frenet_odom_jump_slack_m`: 고정 한계와 실제 속도·수신 간격을 함께 사용하는 Frenet 연속성 판정
- `obstacle_marker_scale`, `path_marker_width`, `corridor_debug_stride`: RViz 마커 크기와 회랑 표시 간격
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970

현재 `config/local_planning.yaml`은 제어기 단독 감속 시험을 위해
`avoidance_speed_scale=1.0`으로 설정하고, 횡·종가속도 한계를 `1000000.0`으로 높여
회피 경로의 속도 배율, 곡률 기반 속도 캡, 종방향 속도 재프로파일을 실질적으로
비활성화한다. 경로 형상의 곡률·곡률 변화율 검증과 안전정지 속도 프로파일은 그대로 유지된다.
- 입력 freshness: `obstacle_stale_timeout_sec`, `odometry_stale_timeout_sec`
- 합류 확인: `merge_lateral_tolerance_m`, `merge_confirm_cycles`
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
4. commitment 방향 히스테리시스
5. 글로벌 라인과 원본 clearance가 충분한 옆 장애물 무시
6. 양쪽이 막혔을 때 점진 정지
7. 랩 경계 장애물 처리
8. 가까운 반대편 스네이크 branch로 점프하지 않음

## 7. 실행 방법

### 7.1 perception을 함께 실행

기본 launch는 `opponent_detector`를 함께 실행합니다. 지도 서버는 중복 실행하지 않으며,
먼저 실행한 `particle_filter_cpp` MCL map server의 `/map`을 detector가 그대로 구독합니다.
MCL의 기본 지도는 `monte_carlo_localization/maps/ifac_track.yaml`입니다.

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

<<<<<<< HEAD
이미 `/perception/static_obstacles/cartesian` 발행기가 실행 중이면 detector 포함을 끕니다.

```zsh
ros2 launch local_planning local_planning.launch.py \
  start_opponent_detector:=false
```
=======
1. `MarkerArray` 디스플레이를 추가합니다.
2. Topic을 `/local_planning/markers`로 설정합니다.
3. 빨간 구체는 그룹화된 장애물 중심, 초록 선은 안전 검사를 통과한 전방 경로입니다. 연두 경계는 안전 회랑, 빨간 횡선은 막힌 구간, 청록/파란 횡선은 좌우 통과 공간, 반투명 주황 구체는 팽창 장애물입니다.
4. `/local_planning/path`에는 안전 검사를 통과한 경로만 표시됩니다.
5. 흰색 텍스트 마커에는 safe corridor·트랙·점유·unknown·곡률·횡가속도별 후보 거절 횟수와 직전 계획 latency가 표시됩니다. 기본 후보가 실패하면 복구 lattice를 탐색하고, 복구도 실패하면 점진 감속 구간을 발행합니다.

`/avoid_waypoints.ot_line`은 상태를 알려줍니다. 정상 lattice 경로는 `frenet_lattice_segment`, 복구 탐색 경로는 `frenet_lattice_recovery`, 연속 장애물 재계획 공백의 재검증 제동 경로는 `frenet_lattice_replan_brake`, 짧은 입력 이상 중 유지되는 경로는 기존 이름 뒤에 `_held`, lattice 전체 실패 후 글로벌 기준 감속 구간은 `frenet_lattice_safe_stop`, 감속 경로가 소진된 뒤 현재 위치에서 정지하며 재계획하는 경로는 `frenet_lattice_stationary_hold`, 정지 위치조차 충돌 검사를 통과하지 못하면 `no_safe_path`, 유지 제한시간이 끝난 Frenet 입력 이상은 `invalid_frenet_odom`입니다.
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970

시뮬레이션 clock을 쓰는 전체 파이프라인이면 `use_sim_time:=true`를 함께 지정합니다.

```zsh
ros2 launch local_planning local_planning.launch.py use_sim_time:=true
```

## 8. 실행 확인

글로벌 플래너, Frenet odometry, perception이 먼저 준비된 상태에서 확인합니다.

```zsh
ros2 topic echo /avoid_waypoints --once
<<<<<<< HEAD
ros2 topic echo /local_planning/path --once
=======
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970
ros2 topic hz /local_planning/markers
```

RViz에서 `/local_planning/markers`를 추가하면 초록 선은 검증된 spline, 주황 선은 safe stop,
보라색 점은 spline 제어점, 빨간 구체는 정적 장애물 중심입니다.

Cartesian 입출력 통합 확인:

```bash
python3 src/local_planning/test/cartesian_static_pipeline_test.py \
  --waypoints-csv /path/to/global_waypoints.csv
```

별도 터미널에서 `local_planner_node`가 실행 중이어야 한다. 테스트는 map-frame 중심과 radius를 넣고
`/avoid_waypoints`의 모든 `x_m/y_m`이 유한하며 횡방향 회피가 실제로 생성됐는지 확인한다.

## 9. 전체 파이프라인 영향

상태머신과 perception의 메시지 계약은 바꾸지 않았습니다.

- `state_machine`: 기존 `/avoid_waypoints` ego→merge 규약을 그대로 사용합니다.
- `wpnt_publisher`: `STATE_AVOID`일 때 기존처럼 `/avoid_waypoints`를 `/local_waypoints`로 중계합니다.
- `opponent_detector`: `f110_msgs/msg/ObstacleArray`에 Cartesian x/y, Frenet s/d, radius를 채워 발행합니다.
- 회피 결과: `/avoid_waypoints` 각 점의 `x_m/y_m`은 map-frame Cartesian 좌표입니다.

따라서 이번 변경에는 `src/local_planning` 밖의 소스 수정이 필요하지 않습니다.
