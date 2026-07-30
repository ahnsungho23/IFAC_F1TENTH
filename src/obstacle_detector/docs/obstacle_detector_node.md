# obstacle_detector_node

## 1. 노드 목적

`obstacle_detector_node`는 2D LiDAR scan에서 트랙 위의 비지도 장애물을 검출하고 Frenet 프레임에서
추적한다. 확정된 장애물을 정적 레이어와 동적 레이어로 분리해 다음 토픽으로 발행한다.

- `/static_obs`: 지도에는 없는 provisional/confirmed 정적 장애물 전체
- `/opp_obs`: 에고 전방의 가장 가까운 동적 상대차 최대 1개

이 패키지는 검출만 담당한다. 경로 계획, 회피·추월 waypoint, 주행 상태 결정은 다른 패키지의 책임이다.

## 2. 동작 원리

노드는 `/scan`이 들어올 때마다 다음 순서로 실행된다.

### 2.1 기준 경로 준비

1. `/global_waypoints`를 받는다.
2. `FrenetProjector`에 트랙 길이와 `d_left/d_right` 경계를 저장한다.
3. `global_planning::ClcsFrenetConverter`를 생성한다.

CLCS가 준비되기 전에는 scan을 처리하지 않는다.

### 2.2 Scan 좌표 변환과 군집화

1. scan header의 `frame_id`에서 `map`으로 가는 TF를 조회한다.
2. NaN, Inf, `range_min` 미만, `max_range` 이상 beam을 제거한다.
3. 각 beam을 map 좌표의 점으로 변환한다.
4. 인접 beam 사이의 거리 적응형 임계값으로 adaptive-breakpoint clustering을 수행한다.
5. `cluster_merge_enable=true`이면 `cluster_merge_min_fragment_points` 이상의 작은 파편을
   임시로 보존한다.
6. 두 파편의 Cartesian AABB 간격과 실제 점 사이 최소거리가 모두
   `cluster_merge_distance` 이내이고, 병합 AABB 대각선이 `max_obs_size` 이하일 때만
   tracking 전에 병합한다.
7. 병합 후에도 `min_cluster_points`보다 작은 클러스터를 버린다.

현재 detector 내부에는 scan deskew, median filter, temporal scan filter가 없다. 전처리 노드를 추가할
경우 `scan_topic`을 전처리 결과 토픽으로 변경한다.

이 pre-tracking `cluster_merge`는 scan range를 보정하는 필터가 아니다. 끊어진 LiDAR 표면을 하나의
detection으로 복원해 하나의 Kalman track이 생성되도록 하는 segmentation 후처리다.

### 2.3 Layer 1 필터

각 클러스터에 다음 필터를 순서대로 적용한다.

1. Cartesian centroid와 AABB 대각선 `size`를 계산한다.
2. `size > max_obs_size`인 큰 구조물을 제거한다.
3. centroid를 CLCS로 Frenet `(s,d)`에 투영한다.
4. 에고 기준 `view_behind_distance`부터 `max_viewing_distance`까지의 관측 창만 남긴다.
5. waypoint의 `d_left/d_right` 안에 있는 클러스터만 남긴다.
6. 클러스터 점 중 `/map` occupied cell 위의 비율이 `map_point_reject_ratio` 이상이면 제거한다.

Layer 1은 벽과 알려진 지도 구조물을 제거하는 필터이며 별도 토픽으로 발행하지 않는다.

### 2.4 Kalman 추적

필터를 통과한 detection을 등속 Kalman tracker에 전달한다.

```text
state = [s, vs, d, vd]
measurement = [s, d]
```

각 detection은 다음 측정 공분산 스케일을 가진다.

```text
range_ratio  = clamp(mean_range / max_range, 0, 1)
sparse_ratio = clamp((meas_reference_points - point_count) / meas_reference_points, 0, 1)

variance_scale =
    1
  + meas_range_var_scale × range_ratio²
  + meas_sparse_var_scale × sparse_ratio
  + meas_yaw_rate_var_scale × |fresh odometry yaw rate|
```

`variance_scale`은 `[1, meas_variance_scale_max]`로 제한한다. Scan timestamp와 odometry
timestamp 차이가 `meas_motion_timeout`을 넘거나 yaw rate가 유한하지 않으면 회전 항은 0으로
처리한다. 앞단에서 deskew한 scan을 받는 경우 `meas_yaw_rate_var_scale`을 낮추거나 0으로 설정한다.

```text
R(s,s) = meas_var_s × variance_scale
R(d,d) = meas_var_d × variance_scale
```

Association은 다음 순서로 수행한다.

1. 기존 static/unknown 0.5 m, dynamic 1.0 m Frenet Euclidean hard gate를 적용한다.
2. `S = HPHᵀ + R`로 detection별 Mahalanobis distance를 계산해 `assoc_mahalanobis_gate`를
   넘는 후보를 제거한다.
3. 남은 후보를 기존 Frenet 거리순으로 정렬해 greedy 1:1 연결한다.

Adaptive `R`이 큰 원거리 detection이 작은 정규화 거리를 얻어 우선되는 것을 막기 위해
Mahalanobis distance는 gate로만 사용하고 정렬 기준은 Frenet 거리를 유지한다. 연결된 track은
adaptive `R`로 Kalman 측정 갱신하고, 연결되지 않은 track은 TTL을 감소시키며, 남은 detection은
새 track으로 생성한다.

### 2.5 정적/동적 분류

느린 track들의 평균 Frenet 속도를 static-field reference로 계산한다. 각 track의 속도가 이 기준에서
얼마나 벗어나는지로 정적/동적을 분류한다.

```text
v_rel = [vs - static_ref_vs, vd - static_ref_vd]
relative_speed = ||v_rel||
velocity_m2 = v_relᵀ P_velocity⁻¹ v_rel
```

기본 상태 전이는 다음과 같다.

1. hits 1~2: `classified=false`, `PENDING`. `/static_obs`와 `/opp_obs` 모두 발행하지 않는다.
2. hits 3: `classified=true`, `PROVISIONAL_STATIC`. 즉시 `/static_obs`에 발행한다.
3. `relative_speed < dyn_vel_exit(0.25 m/s)`가 추가로 3회 연속 관측되면
   `CONFIRMED_STATIC`이 된다.
4. 아래 조건을 모두 만족하는 관측이 25회 연속 쌓이면 `DYNAMIC`이 된다.
   - `relative_speed > dyn_vel_enter(0.5 m/s)`
   - `velocity_m2 >= dyn_velocity_mahalanobis_gate(9.21)`
   - ego odometry가 `meas_motion_timeout` 안의 최신 값
   - `|yaw_rate| <= dyn_max_abs_yaw_rate(1.5 rad/s)`
5. `DYNAMIC` track에서 저속 조건이 3회 연속 확인되면 `CONFIRMED_STATIC`으로 복귀한다.

`0.25~0.5 m/s` hysteresis 구간, 속도 신뢰도 부족, 빠르거나 오래된 ego 회전 정보, detection miss는
연속 이동 증거를 끊는다. 빠른 회전 중 발생한 Frenet 속도 오차가 dynamic으로 누적되지 않게 하기
위함이다. 반면 저속 정적 증거는 회전율 gate 때문에 지연하지 않아 정적 장애물을 빠르게 확정한다.

`PROVISIONAL_STATIC → CONFIRMED_STATIC`은 같은 Track 객체와 ID를 유지하며 두 상태 모두
`/static_obs`에 실리므로 전환 공백이 없다. `DYNAMIC`으로 바뀐 동일 scan부터 `/static_obs`에서
제거하고 `/opp_obs` 후보로 옮긴다.

기본 `classifier_mode=velocity`에서는 사용하지 않는 positional-std 이력을 저장하거나 분산을 계산하지
않는다. `classifier_mode=std` 또는 `both`일 때만 `std_window` 길이의 `(s,d)` 이력을 유지한다.

### 2.6 레이어 병합과 발행

발행 가능한 track을 static과 dynamic으로 분리한다. 같은 레이어 안에서 Frenet 박스의 모서리 간격이
`layer_merge_gap_s/d` 이내인 track들을 하나의 객체로 병합한다.

이 `layer_merge`는 tracking 전 `cluster_merge`와 역할이 다르다.

- `cluster_merge`: LiDAR 파편을 합쳐 하나의 detection과 하나의 track을 만든다.
- `layer_merge`: 이미 별도로 추적된 같은 레이어 track들을 최종 출력에서 하나의 객체로 표현한다.

- 모든 병합 static 객체를 `/static_obs`로 발행한다.
- 병합 dynamic 객체 중 에고 전방에서 가장 가까운 하나를 `/opp_obs`로 발행한다.
- RViz 마커는 실제 Cartesian AABB 크기의 사각형으로 표시하며 static은 파란색, dynamic
  opponent는 빨간색이다.

두 ObstacleArray 토픽은 장애물이 없는 scan에서도 빈 배열로 발행된다.

### 2.7 Cartesian AABB

각 cluster의 map-frame `x_min/x_max/y_min/y_max`를 Detection과 Track에 보존한다. 같은 레이어에서
여러 track이 한 객체로 병합되면 현재 scan에서 실제로 측정된 `is_visible=true` 멤버들의 AABB 합집합을
출력한다.

```text
x_center = (x_min + x_max) / 2
y_center = (y_min + y_max) / 2
radius   = 0.5 × hypot(x_max - x_min, y_max - y_min)
```

유효한 합집합이 있으면 `has_cartesian=true`와 함께 중심, 경계, 반지름을 채운다. `x_var/y_var`는
CLCS 접선으로 공분산을 완전히 회전하기 전까지 보수적으로 `max(s_var, d_var)`를 양축에 사용한다.

Track이 이번 scan에서 detection과 연결되지 않으면 Kalman의 Frenet `(s,d)`는 예측되지만 마지막 raw
Cartesian AABB는 움직이지 않는다. 따라서 component 전체가 predicted-only이면 Frenet 출력은 유지하되
`has_cartesian=false`로 발행하고 RViz Cartesian marker도 만들지 않는다. 이는 dynamic TTL 동안 낡은
AABB가 현재 충돌 위치로 해석되는 것을 방지한다.

Cartesian AABB는 map 축에 정렬된 관측 footprint다. 기존 `size × size` Frenet merge 판정과
`s_start/s_end/d_left/d_right` envelope는 바꾸지 않는다.

### 2.8 Perception 진단 통계

`diagnostics_enable=true`이면 `diagnostics_period_sec` 동안 처리 결과를 누적해 INFO 로그 한 줄로
출력한다. 기본 주기는 1초이며 wall clock을 사용하므로 `/clock` 유무와 관계없이 진단 주기가 유지된다.

```text
DIAG perception [1.00s scans=40/40 drop(clcs=0 tf=0)]
beam(valid=.../... nonfinite=... below_min=... at_or_above_max=...)
cluster=...->... fragment_drop=... detections=...
reject(size=... clcs_projection=... view=... boundary=... map=...)
track(total=... visible=... hit_pending=... class_pending=... provisional=...
      static=... dynamic=... motion_gated=...)
assoc(pairs=... match=... spawn=... retire=... euclid_reject=... maha_reject=...)
motion(yaw_used=... fresh=... ref_vs=... ref_vd=...)
```

- `scans=processed/received`는 수신 scan 중 전체 파이프라인을 끝까지 처리한 개수다.
- `drop(clcs/tf)`는 기준 경로 또는 scan→map TF가 없어 scan 전체를 처리하지 못한 횟수다.
- beam 카운터는 유효 beam과 비유한값, `range_min` 미만, `max_range` 이상 탈락 원인을 구분한다.
- `cluster=A->B`는 임시 파편 수에서 병합·최소 포인트 필터 후 클러스터 수로의 변화를 뜻한다.
- `fragment_drop`은 병합 후에도 `min_cluster_points`를 채우지 못한 파편 수다.
- `reject(...)`는 detection 생성 전 각 Layer 1 gate에서 탈락한 클러스터 수다.
- `assoc`는 1초 동안 누적한 tracker event다. `pairs`, `euclid_reject`, `maha_reject`는 장애물 수가
  아니라 `track × detection` 후보 쌍 수다.
- `track`은 누적합이 아니라 로그 시점의 최신 snapshot이다. `hit_pending`은
  `min_hits_confirm` 미달이다. 정상 상태기계에서는 hit 조건을 채우는 즉시 provisional이 되므로
  `class_pending=0`이다. `motion_gated`는 속도가 `dyn_vel_enter`를 넘었지만 속도 Mahalanobis
  신뢰도 또는 ego yaw 조건을 통과하지 못한 visible track 수다.
- `yaw_used`는 adaptive covariance 계산에 실제 사용한 yaw rate다. odometry timestamp가
  `meas_motion_timeout`을 넘으면 `yaw_used=0`, `fresh=false`가 된다.

이 노드는 scan noise filter와 deskew를 수행하지 않으므로 `noise_rejected`와 `deskew_source`를
추정해서 출력하지 않는다. 해당 값은 향후 LiDAR 전처리 노드가 실제 처리 결과를 기준으로 진단해야 한다.
이 진단 기능은 카운터와 로그만 추가하며 scan, detection, Kalman 상태를 변경하지 않는다.

## 3. 구독 토픽

| 토픽 파라미터 | 기본 토픽 | 메시지 타입 | 용도 |
|---|---|---|---|
| `scan_topic` | `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR 입력 |
| `global_waypoints_topic` | `/global_waypoints` | `f110_msgs/msg/WpntArray` | CLCS 기준 경로와 트랙 경계 |
| `map_topic` | `/map` | `nav_msgs/msg/OccupancyGrid` | Layer 1 지도 필터 |
| `ego_odom_topic` | `/pf/pose/odom` | `nav_msgs/msg/Odometry` | 에고 Frenet 위치 |

시뮬레이터 launch에서는 `ego_odom_topic`이 `/ego_racecar/odom`으로 바뀐다.

## 4. 발행 토픽

| 토픽 파라미터 | 기본 토픽 | 메시지 타입 | 내용 |
|---|---|---|---|
| `static_obs_topic` | `/static_obs` | `f110_msgs/msg/ObstacleArray` | provisional/confirmed 정적 객체 전체와 visible Cartesian AABB |
| `opp_obs_topic` | `/opp_obs` | `f110_msgs/msg/ObstacleArray` | 최근접 동적 상대차 최대 1개와 visible Cartesian AABB |
| `markers_topic` | `/perception/obstacles/markers` | `visualization_msgs/msg/MarkerArray` | RViz 표시 |

## 5. 주요 파라미터

운영값은 [`../config/obstacle_detector.yaml`](../config/obstacle_detector.yaml)에 있다.

| 그룹 | 파라미터 | 설명 |
|---|---|---|
| 범위 | `max_range`, `max_viewing_distance`, `view_behind_distance` | LiDAR 및 전후 관측 범위 |
| 군집화 | `lambda_deg`, `cluster_sigma`, `min_2_points_dist` | adaptive-breakpoint 임계값 |
| 파편 병합 | `cluster_merge_enable`, `cluster_merge_distance`, `cluster_merge_min_fragment_points` | tracking 전 작은 LiDAR 파편 병합 |
| 크기 | `min_cluster_points`, `max_obs_size` | 병합 후 최소 beam 수와 최대 객체 크기 |
| 경계 | `boundaries_inflation`, `fallback_track_halfwidth` | 트랙 내부 통과 조건 |
| 지도 | `use_map_filter`, `map_occupied_thresh`, `map_inflation_cells`, `map_point_reject_ratio` | 점유지도 필터 |
| 측정 불확실성 | `meas_range_var_scale`, `meas_sparse_var_scale`, `meas_yaw_rate_var_scale`, `meas_reference_points`, `meas_variance_scale_max`, `meas_motion_timeout` | Detection별 adaptive Kalman `R` |
| 추적 | `meas_var_s/d`, `process_var_vs/vd`, `assoc_gate`, `aggro_multi`, `assoc_use_mahalanobis`, `assoc_mahalanobis_gate` | Kalman 및 2단계 association |
| 수명 | `ttl_dynamic`, `ttl_static`, `min_hits_confirm` | track 유지와 발행 확정. `ttl_static=25`는 약 250 Hz 입력에서 약 0.1초의 정적 track 검출 공백을 허용 |
| 분류 | `classifier_mode`, `dyn_vel_enter/exit`, `static_confirm_frames`, `dynamic_confirm_frames`, `dyn_velocity_mahalanobis_gate`, `dyn_max_abs_yaw_rate`, `static_ref_gate` | provisional/static/dynamic 판정 |
| 레이어 병합 | `layer_merge_enable`, `layer_merge_gap_s/d` | tracking 후 같은 레이어 객체 병합 |
| 진단 | `diagnostics_enable`, `diagnostics_period_sec` | 누적 perception INFO 로그 활성화와 주기 |

## 6. 빌드

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-up-to obstacle_detector
source install/setup.zsh
```

## 7. 실행

실차:

```bash
ros2 launch obstacle_detector obstacle_detector.launch.py
```

시뮬레이터:

```bash
ros2 launch obstacle_detector obstacle_detector.launch.py simulator:=true
```

RViz 포함:

```bash
ros2 launch obstacle_detector obstacle_detector.launch.py rviz:=true
```

장애물이 포함된 live map과 분리된 clean map을 Layer 1에 사용:

```bash
ros2 launch obstacle_detector obstacle_detector.launch.py \
  simulator:=true \
  detector_map_yaml:=/absolute/path/to/clean_map.yaml
```

`use_sim_time:=true`는 `/clock` publisher가 실제로 존재할 때만 사용한다.

## 8. 검증

격리된 ROS domain에서 새 detector를 실행한 뒤 synthetic harness를 실행한다.

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=87

ros2 launch obstacle_detector obstacle_detector_node.launch.py
```

다른 터미널:

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=87
source /opt/ros/jazzy/setup.zsh
source ~/2026_IFAC/install/setup.zsh
python3 ~/2026_IFAC/src/obstacle_detector/test/synthetic_opponent_test.py
```

PASS 조건은 동적 상대차가 먼저 `/static_obs`에 provisional로 나타난 뒤 같은 ID로 `/opp_obs`에
이동하고, 정적 장애물이 `/static_obs`에만 나타나며,
5포인트 미만 LiDAR 파편들이 tracking 전에 하나의 detection으로 복원되고, 별도 track으로 남은
조각난 정적 물체도 layer merge에서 하나의 출력 객체로 병합되는 것이다.
