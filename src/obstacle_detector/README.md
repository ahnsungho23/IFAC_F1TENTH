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
  → 크기·관측거리·트랙경계·점유지도 필터
  → 거리·희소도·회전량 기반 adaptive 측정 공분산
  → Frenet 등속 Kalman tracking
  → static/dynamic 분류
  → 레이어 내부 객체 병합
  → visible track의 Cartesian AABB 합집합·중심·반지름
  → 1초 누적 perception 진단 로그
  ├─ /static_obs
  ├─ /opp_obs
  └─ /perception/obstacles/markers
```

### 레이어

- Layer 1: `/map`에 등록된 벽과 알려진 구조물을 제거하는 필터. 발행하지 않는다.
- Layer 2: 지도에는 없지만 정지해 있는 확정 장애물 전체를 `/static_obs`로 발행한다.
- Layer 3: 확정 동적 물체 중 에고 전방에서 가장 가까운 하나를 `/opp_obs`로 발행한다.

두 출력은 `f110_msgs/msg/ObstacleArray`이며 매 scan마다 발행된다. 해당 레이어가 비어 있으면 빈 배열을
발행한다.

각 visible 객체는 `has_cartesian=true`와 map-frame AABB, AABB 중심, AABB를 감싸는 원의 반지름을
함께 제공한다. Detection이 끊겨 Kalman 예측만 남은 객체는 Frenet 상태는 유지하지만 stale raw AABB를
현재 위치로 오해하지 않도록 `has_cartesian=false`로 발행한다.

## 주요 입출력

| 방향 | 기본 토픽 | 타입 |
|---|---|---|
| 구독 | `/scan` | `sensor_msgs/msg/LaserScan` |
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` |
| 구독 | `/map` | `nav_msgs/msg/OccupancyGrid` |
| 구독 | `/pf/pose/odom` | `nav_msgs/msg/Odometry` |
| 발행 | `/static_obs` | `f110_msgs/msg/ObstacleArray` |
| 발행 | `/opp_obs` | `f110_msgs/msg/ObstacleArray` |
| 발행 | `/perception/obstacles/markers` | `visualization_msgs/msg/MarkerArray` |

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
  `map_point_reject_ratio`
- 측정 공분산: `meas_range_var_scale`, `meas_sparse_var_scale`,
  `meas_yaw_rate_var_scale`, `meas_reference_points`
- 추적: `meas_var_s/d`, `process_var_vs/vd`, `assoc_gate`,
  `assoc_use_mahalanobis`, `assoc_mahalanobis_gate`, `ttl_dynamic/static`
- 분류: `classifier_mode`, `dyn_vel_enter/exit`, `static_ref_gate`
- 레이어 출력 병합: `layer_merge_enable`, `layer_merge_gap_s/d`
- 진단: `diagnostics_enable`, `diagnostics_period_sec`

진단 로그는 beam 범위 탈락, 병합 전후 cluster, Layer 1 gate, association, track 생성·폐기 및
static/dynamic 분류 상태를 구분한다. Detector가 수행하지 않는 noise filtering과 deskew 통계는
포함하지 않는다.

상세 설명은 [`docs/obstacle_detector_node.md`](docs/obstacle_detector_node.md), 테스트 절차는
[`docs/sim_test_commands.md`](docs/sim_test_commands.md)를 참고한다.
