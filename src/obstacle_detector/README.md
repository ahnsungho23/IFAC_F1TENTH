# obstacle_detector

> 한국어 · [English](README_en.md)

2D LiDAR만으로 트랙 위의 비지도 장애물을 검출하고, Frenet 프레임에서 추적하여 정적 장애물과 동적
상대차로 분리하는 ROS 2 Jazzy C++ 패키지다.

이 패키지는 **검출만 담당**한다. 경로 계획, 회피·추월 waypoint 생성, `/state` 발행은 포함하지 않는다.

## 데이터 흐름

```text
/scan + /global_waypoints + /map + ego odom + TF
  → adaptive-breakpoint clustering
  → tracking 전 LiDAR 파편 병합
  → Cartesian AABB 전체의 Frenet 경계 투영
  → 크기·관측거리·트랙경계·점유지도 필터
  → 거리·희소도·회전량 기반 adaptive 측정 공분산
  → Frenet 등속 Kalman association + map-frame 등속 Kalman motion 추정
  → RAW / TENTATIVE / CONFIRMED 존재 상태
  → UNKNOWN / STATIC / DYNAMIC motion 상태
  → 레이어 내부 객체 병합
  → visible track의 Cartesian AABB 합집합과 일치하는 Frenet 경계
  → 1초 누적 perception 진단 로그
  ├─ /static_obs
  ├─ /confirmed_static_obs
  ├─ /opp_obs
  ├─ /static_obs/markers
  └─ /opp_obs/markers
```

### 레이어

- Layer 1: `/map`에 등록된 벽과 알려진 구조물을 제거하는 필터. 발행하지 않는다.
- Layer 2: 최근 5 scan 중 3회 관측된 `CONFIRMED` 장애물 가운데 motion이 `UNKNOWN` 또는
  `STATIC`인 객체를 같은 ID로 `/static_obs`에 발행한다.
- Confirmed Layer 2: 위치 지속성과 속도 통계 voting으로 확정된 `STATIC`만
  `/confirmed_static_obs`에 별도로 발행한다. 장기 저장
  노드는 이 토픽을 사용하며 기존 `/static_obs` 계약은 바뀌지 않는다.
- Layer 3: 확정 동적 물체 중 에고 전방에서 가장 가까운 하나를 `/opp_obs`로 발행하고,
  ego corridor와 현재/예측 간격을 비교한 `is_interfering` 값을 함께 제공한다. 기본 5.0 m에서
  진입하고 같은 ID를 추종하는 동안 +10% 여유를 둬 5.5 m까지 간섭 상태를 유지한다.

세 장애물 레이어 view는 `f110_msgs/msg/ObstacleArray`이며 매 scan마다 발행된다. 해당 view가
비어 있으면 빈 배열을 발행한다.

분류는 기존 Frenet KF와 별도의 map-frame KF `[x,vx,y,vy]`를 사용한다. 속도 통계
`Tv=vᵀPv⁻¹v`와 map 위치 RMS를 최근 measurement history에서 voting하며, prediction-only scan은
vote로 세지 않는다. 기본값에서 최근 5개 중 dynamic evidence 3개면 같은 ID로 `/opp_obs`로
이동한다. STATIC은 최근 15개 중 static evidence 10개와 위치 RMS 0.10 m 이하를 함께 요구한다.

외부로 발행하는 `Obstacle.id`는 내부 Kalman track 번호와 분리된 물리 객체 ID다. 1차
Frenet/Kalman association이 끊겨도 AABB가 같은 공간 cluster이면 motion 상태와 무관하게 2차로
기존 track에 연결한다. 한 scan에서 같은 객체가 여러 track으로 갈라져도 같은 공개 ID를 쓰며,
확정 객체가 처음 `STATIC`으로 안정화된 실측 Frenet footprint와 map-frame AABB를 identity
anchor로 고정한다. 이후 관측면 변화나 잘못된 `DYNAMIC` evidence가 생겨도 anchor는 움직이지
않는다. `STATIC`에 도달하지 않은 객체는 마지막 실측 footprint를 사용한다. track 폐기 뒤에도
기본 30초 동안 이 anchor와 ID를 기억하며, Frenet envelope 또는 map AABB가 같은 공간 cluster인
재검출에 ID를 재사용한다. 미관측 구간의 Kalman 예측 위치는 ID 기억에 사용하지 않는다. 따라서
`UNKNOWN`, `STATIC`, `DYNAMIC` 전환으로 출력 토픽이
`/static_obs`, `/confirmed_static_obs`, `/opp_obs` 사이에서 바뀌어도 ID는 유지된다.
Global planner가 같은 `/global_waypoints`를 주기적으로 재발행해도 CLCS와 tracker를 유지하며,
`x/y/s/d_left/d_right` 기준 형상이 실제로 바뀔 때만 tracker와 ID 기억을 초기화한다.

각 visible 객체는 map-frame AABB 전체를 CLCS에 투영한
`s_start/s_end/d_right/d_left`를 authoritative geometry로 제공한다. 같은 footprint의
`has_cartesian=true`, AABB 중심, AABB를 감싸는 원의 반지름도 함께 제공한다. Detection이 끊겨
Kalman 예측만 남은 객체는 마지막 측정 Frenet 크기를 예측 중심에 유지하지만 stale raw AABB를
현재 위치로 오해하지 않도록 `has_cartesian=false`로 발행한다.

RViz용 `/static_obs/markers`와 `/opp_obs/markers`는 각각 최종 ObstacleArray의
`s_start/s_end/d_right/d_left`를 map 좌표로 변환한 테두리다. `/static_obs/markers`는
provisional 객체까지 포함하므로, `/confirmed_static_obs`만 사용하는 local planner 입력의
상위집합이다. Predicted-only 객체도 표시하며 현재
관측 객체보다 옅게 그린다.

## 주요 입출력

| 방향 | 기본 토픽 | 타입 |
|---|---|---|
| 구독 | `/scan` | `sensor_msgs/msg/LaserScan` |
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` |
| 구독 | `/map` | `nav_msgs/msg/OccupancyGrid` |
| 구독 | `/pf/pose/odom` | `nav_msgs/msg/Odometry` |
| 발행 | `/static_obs` | `f110_msgs/msg/ObstacleArray` |
| 발행 | `/confirmed_static_obs` | `f110_msgs/msg/ObstacleArray` |
| 발행 | `/opp_obs` | `f110_msgs/msg/ObstacleArray` |
| 발행 | `/static_obs/markers` | `visualization_msgs/msg/MarkerArray` |
| 발행 | `/opp_obs/markers` | `visualization_msgs/msg/MarkerArray` |

`simulator:=true`에서는 ego odom 입력이 `/ego_racecar/odom`으로 바뀐다.

## 빌드

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-up-to obstacle_detector
source install/setup.zsh
```

## 실행

실차:

```bash
ros2 launch obstacle_detector obstacle_detector.launch.py
```

시뮬레이터:

```bash
ros2 launch obstacle_detector obstacle_detector.launch.py simulator:=true
```

RViz:

```bash
ros2 launch obstacle_detector obstacle_detector.launch.py rviz:=true
```

장애물이 포함된 live map을 사용하는 경우, Layer 1 필터에는 장애물이 없는 별도 지도를 지정한다.

```bash
ros2 launch obstacle_detector obstacle_detector.launch.py \
  simulator:=true \
  detector_map_yaml:=/absolute/path/to/clean_map.yaml
```

## 설정

모든 detector 파라미터는
[`config/obstacle_detector.yaml`](config/obstacle_detector.yaml)에 있다.

주요 그룹:

- 클러스터링: `max_range`, `lambda_deg`, `cluster_sigma`, `min_cluster_points`, `max_obs_size`
- 파편 병합: `cluster_merge_enable`, `cluster_merge_distance`,
  `cluster_merge_min_fragment_points`
- Layer 1 필터: `max_viewing_distance`, `boundaries_inflation`, `use_map_filter`,
  `wall_assoc_distance_m`, `wall_linear_ratio`, `wall_min_length_m`
- 측정 공분산: `meas_range_var_scale`, `meas_sparse_var_scale`,
  `meas_yaw_rate_var_scale`, `meas_reference_points`
- 추적: `meas_var_s/d`, `process_var_vs/vd`, `assoc_gate`,
  `assoc_use_mahalanobis`, `assoc_mahalanobis_gate`, `ttl_dynamic/static`
- 물리 객체 ID 연속성: `physical_id_reassociation_enable`,
  `physical_id_reassociation_gap_s/d/map`, `physical_id_memory_sec`
- 존재 확인: `min_hits_confirm`, `confirmation_window`
- 분류: `motion_classification.dynamic_chi2_threshold`, `static_chi2_threshold`,
  `dynamic_vote_*`, `static_vote_*`, `position_history_size`, `static_max_position_rms`,
  `dynamic_to_static_*`
- 레이어 출력 병합: `layer_merge_enable`, `layer_merge_gap_s/d`
- 진단: `diagnostics_enable`, `diagnostics_period_sec`

진단 로그는 beam 범위 탈락, 병합 전후 cluster, Layer 1 gate, association, track 생성·폐기 및
static/dynamic 분류 상태를 구분한다. Detector가 수행하지 않는 noise filtering과 deskew 통계는
포함하지 않는다.

상세 설명은 [`docs/obstacle_detector_node.md`](docs/obstacle_detector_node.md), 테스트 절차는
[`docs/sim_test_commands.md`](docs/sim_test_commands.md)를 참고한다.
