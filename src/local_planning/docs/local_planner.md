# local_planner_node

## 1. 노드 목적

`local_planner_node`는 정적 장애물이 글로벌 Race Line을 막을 때만 로컬 회피 세그먼트를 만듭니다.
동적 상대 차량의 추월·추종은 `opponent_detector`의 `/overtake_waypoints`가 담당하고, 이 노드는
`/avoid_waypoints`만 발행합니다.

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
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` | 순서를 고정할 글로벌 Race Line |
| 구독 | `/perception/static_obstacles/cartesian` | `f110_msgs/msg/ObstacleArray` | 정적 장애물 x/y/s/d/radius |
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | `x=s`, `y=d` ego 상태 |
| 발행 | `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` | ego부터 글로벌 합류 뒤 lookahead까지의 회피 세그먼트 |
| 발행 | `/local_planning/path` | `nav_msgs/msg/Path` | RViz용 현재 안전 경로 |
| 발행 | `/local_path` | `nav_msgs/msg/Path` | 기존 시각화 호환 토픽 |
| 발행 | `/local_planning/markers` | `visualization_msgs/msg/MarkerArray` | 경로, spline 제어점, 정적 장애물 중심 |
| 선택 발행 | `/local_waypoints` | `f110_msgs/msg/WpntArray` | 단독 실행 옵션. 기본값은 꺼짐 |

`/avoid_waypoints.ot_line`은 정상 회피일 때 `raceline_local_d_offset_spline`, 좌우가 모두 막힌
감속 경로일 때 `raceline_static_safe_stop`입니다.

## 5. 주요 파라미터

모든 운영값은 `config/local_planning.yaml`에 있습니다.

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

이미 `/perception/static_obstacles/cartesian` 발행기가 실행 중이면 detector 포함을 끕니다.

```zsh
ros2 launch local_planning local_planning.launch.py \
  start_opponent_detector:=false
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
