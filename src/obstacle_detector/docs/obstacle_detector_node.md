# obstacle_detector_node

## 1. 노드 목적

`obstacle_detector_node`는 2D LiDAR scan에서 트랙 위의 비지도 장애물을 검출하고 Frenet 프레임에서
추적한다. 확정된 장애물을 정적 레이어와 동적 레이어로 분리해 다음 토픽으로 발행한다.

- `/static_obs`: 존재가 확정된 motion `UNKNOWN` 또는 `STATIC` 장애물
- `/confirmed_static_obs`: 존재와 정지 상태가 모두 확정된 `STATIC` 장애물
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

1. map-frame Cartesian AABB와 대각선 `size`를 계산한다.
2. `size > max_obs_size`인 큰 구조물을 제거한다.
3. `aabb_frenet_projector`가 AABB 중심을 CLCS로 투영해 branch를 고정한다.
4. 중심의 local tangent/normal에서 네 모서리 envelope를 계산해 종방향 반폭과 반대편 횡경계를
   구한다. Race Line을 향한 횡경계는 그 branch 주변의 실제 waypoint 선분과 AABB 네 면 사이
   최단거리로 보정한다.
5. 투영된 AABB 중심을 detection `(s,d)`로 사용하고, 종방향 반폭 및 중심 기준 좌·우 offset을
   함께 tracker에 전달한다.
6. 에고 기준 `view_behind_distance`부터 `max_viewing_distance`까지의 관측 창만 남긴다.
7. waypoint의 `d_left/d_right` 안에 있는 클러스터만 남긴다. 이때 중심이 아니라 팽창된
   envelope의 좌·우 가장자리(`d_left`, `d_right`)를 복도와 비교한다. 벽에 붙은 부채꼴 산란은
   중심은 복도 안이지만 AABB 가장자리는 이미 벽을 파고들기 때문이다.
8. 클러스터 점 중 `/map` occupied cell 위의 비율이 `map_point_reject_ratio` 이상이면 제거한다.

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

### 2.5 존재 상태와 motion 상태 분리

기존 Frenet KF와 별도로 각 track에 분류 전용 map-frame 등속 KF를 둔다.

```text
association/output KF: [s, vs, d, vd]
classification KF:     [x, vx, y, vy]
```

map KF의 measurement는 현재 detection AABB 중심이다. Prediction 위치는 위치 이력이나 vote에
추가하지 않는다. 이 구조는 기존 closed-track Frenet association과 출력 geometry를 유지하면서,
헤어핀에서 `s` projection이 변하는 현상이 motion 판정을 직접 흔들지 않게 한다.

존재 상태는 최근 scan association만 사용한다.

```text
RAW → TENTATIVE → CONFIRMED
기본값: 최근 confirmation_window=5 scan에서 measurement 3회
```

`RAW/TENTATIVE`는 항상 motion `UNKNOWN`이며 발행하지 않는다. `CONFIRMED`가 된 뒤에만 아래
motion evidence를 누적한다.

map KF 속도와 속도 공분산 블록으로 다음 통계량을 계산한다.

```text
v  = [vx, vy]
Pv = map KF covariance의 [vx, vy] 2×2 block
Tv = vᵀ Pv⁻¹ v
```

행렬 inverse는 만들지 않고 Eigen `LDLT.solve()`를 사용한다. 공분산은 대칭화한 뒤
`minimum_velocity_covariance`와 `covariance_regularization_epsilon`으로 작은 고윳값을
regularize한다. NaN/Inf 또는 명백한 음의 고윳값은 해당 measurement를 `UNCERTAIN`으로 처리한다.

- `Tv > dynamic_chi2_threshold(9.21)`: dynamic evidence
- `Tv < static_chi2_threshold(5.99)`: static evidence
- 그 사이: uncertain evidence

한 measurement의 evidence만으로 상태를 바꾸지 않는다.

- 최근 5개 중 dynamic evidence 3개: `UNKNOWN/STATIC → DYNAMIC`
- 최근 15개 중 static evidence 10개, map 위치 이력 10개 이상,
  `position_rms <= 0.10 m`: `UNKNOWN → STATIC`
- `DYNAMIC → STATIC`: DYNAMIC 진입 후 measurement 20개 이상, 최근 static vote 15개,
  `position_rms <= 0.08 m`를 모두 요구한다.

따라서 잠시 멈춘 상대차는 곧바로 STATIC이 되지 않는다. Measurement miss는 vote와 위치 이력을
추가하지 않고 `static_confidence`만 forgetting factor로 감소시킨다. Confidence는 static
evidence, 작은 위치 RMS, 반복 association에서 증가하고 dynamic evidence와 큰 RMS에서 감소한다.
현재 detector에는 ray-traced free-space contradiction 입력이 없어서 해당 감점은 TODO로 남겨뒀다.

호환성과 안전을 위해 `CONFIRMED+UNKNOWN`은 provisional 객체로 `/static_obs`에 포함한다.
`DYNAMIC`으로 바뀐 동일 scan부터 `/static_obs`에서 빠지고 `/opp_obs` 후보로 이동한다.
`/confirmed_static_obs`에는 `CONFIRMED+STATIC`이면서 envelope 안정성 게이트를 통과한 객체만 실린다.

envelope 안정성 게이트: 매칭될 때마다 측정 중심과 extent가 track의 예측/보존 값에서
`envelope_stability_tolerance_m`(기본 0.10 m) 이내로 안정됐는지 검사하고, 연속 횟수가
`envelope_stability_frames`(기본 2)에 못 미치는 정적 track은 발행 레이어에서 제외한다.
정사각형 AABB와 실제 형태의 괴리로 형태가 계속 변하는 부채꼴 산란 클러스터는 이 streak를
채우지 못해 `/static_obs`에 나타나지 않는다. envelope가 안정적인 실제 장애물은 hits 3시점에
streak 2를 함께 만족하므로 발행 지연이 추가되지 않는다. prediction-only 프레임에는 streak가
유지되므로, 한번 안정화된 track의 ghost 발행은 계속 허용된다.

### 2.6 레이어 병합과 발행

발행 가능한 track을 static과 dynamic으로 분리한다. tracker가 보존한 독립적인 종·횡 Frenet
extent로 박스 모서리 간격을 계산하고, 같은 레이어 안에서 그 간격이 `layer_merge_gap_s/d`
이내인 track들을 하나의 객체로 병합한다.

이 `layer_merge`는 tracking 전 `cluster_merge`와 역할이 다르다.

- `cluster_merge`: LiDAR 파편을 합쳐 하나의 detection과 하나의 track을 만든다.
- `layer_merge`: 이미 별도로 추적된 같은 레이어 track들을 최종 출력에서 하나의 객체로 표현한다.

- 모든 병합 static 객체를 `/static_obs`로 발행한다.
- 그중 confirmed static만 다시 병합해 `/confirmed_static_obs`로 발행한다.
- 병합 dynamic 객체 중 에고 전방에서 가장 가까운 하나를 `/opp_obs`로 발행한다.
- `/static_obs/markers`는 최종 `/static_obs`의 Frenet 경계를 파란 테두리로 표시한다.
- `/opp_obs/markers`는 최종 `/opp_obs`의 Frenet 경계를 빨간 테두리로 표시한다.
- 마커는 `s_start/s_end/d_right/d_left`에서 직접 만들어지므로 local planner 입력과 같은
  영역을 나타낸다.

세 ObstacleArray 토픽은 장애물이 없는 scan에서도 빈 배열로 발행된다. 단 `/opp_obs`(와 그 마커)는
ego odometry timestamp가 `meas_motion_timeout`보다 오래되면 전방 순위를 신뢰할 수 없으므로 해당
scan에서 발행을 억제하고 throttled WARN을 남긴다.

### 2.7 Cartesian AABB와 authoritative Frenet 경계

각 cluster의 map-frame `x_min/x_max/y_min/y_max`와 투영된 독립 Frenet extent를 Detection과
Track에 보존한다. 매칭될 때마다 Frenet extent의 크기는 fast-grow/slow-shrink로 완화한다
(`extent_shrink_alpha`): 더 큰 측정에는 즉시 확장하고, 더 작은 측정에는 약 `1/alpha`
프레임에 걸쳐 서서히 축소한다. 정사각형 AABB와 실제 형태의 괴리로 스캔마다 출렁이던
발행 envelope가 raceline을 오가며 플래너를 흔드는 것을 막는다. 같은 레이어에서 여러
track이 한 객체로 병합되면 현재 scan에서 실제로 측정된
`is_visible=true` 멤버들의 AABB 합집합을 만든 뒤 그 완전한 합집합을 한 번 다시 투영한다.
또한 visible 멤버가 하나도 없는 predicted-only component는 병합 envelope 대각선이
`max_obs_size`를 넘으면 발행하지 않는다. 부채꼴 산란이 ghost blob으로 합쳐지며 복도 전체를
덮는 가짜 blocking을 방지한다.
따라서 visible 출력의 Cartesian AABB와 `s_start/s_end/d_right/d_left`는 같은 footprint를
표현한다. 이 투영은 detector에서만 수행하며 downstream planner는 결과를 그대로 사용한다.

```text
x_center = (x_min + x_max) / 2
y_center = (y_min + y_max) / 2
radius   = 0.5 × hypot(x_max - x_min, y_max - y_min)
```

유효한 합집합이 있으면 `has_cartesian=true`와 함께 중심, 경계, 반지름 및 재투영된 Frenet 경계를
채운다. `x_var/y_var`는 CLCS 접선으로 공분산을 완전히 회전하기 전까지 보수적으로
`max(s_var, d_var)`를 양축에 사용한다.

Track이 이번 scan에서 detection과 연결되지 않으면 Kalman의 Frenet `(s,d)`는 예측되지만 마지막 raw
Cartesian AABB는 움직이지 않는다. 따라서 component 전체가 predicted-only이면 마지막 측정
Frenet 종·횡 크기를 예측 중심 주위에 유지하되 `has_cartesian=false`로 발행한다. RViz는 stale
Cartesian AABB 대신 이 예측 Frenet 경계를 낮은 alpha의 테두리로 표시한다.

### 2.8 Perception 진단 통계

`diagnostics_enable=true`이면 `diagnostics_period_sec` 동안 처리 결과를 누적해 INFO 로그 한 줄로
출력한다. 기본 주기는 1초이며 wall clock을 사용하므로 `/clock` 유무와 관계없이 진단 주기가 유지된다.

```text
DIAG perception [1.00s scans=40/40 drop(clcs=0 tf=0)]
beam(valid=.../... nonfinite=... below_min=... at_or_above_max=...)
cluster=...->... fragment_drop=... detections=...
reject(size=... clcs_projection=... view=... boundary=... map=...)
track(total=... visible=... raw=... tentative=...
      unknown=... static=... dynamic=... predicted=... invalid_Pv=...)
assoc(pairs=... match=... spawn=... retire=... euclid_reject=... maha_reject=...)
motion(yaw_used=... fresh=...)
```

- `scans=processed/received`는 수신 scan 중 전체 파이프라인을 끝까지 처리한 개수다.
- `drop(clcs/tf)`는 기준 경로 또는 scan→map TF가 없어 scan 전체를 처리하지 못한 횟수다.
- beam 카운터는 유효 beam과 비유한값, `range_min` 미만, `max_range` 이상 탈락 원인을 구분한다.
- `cluster=A->B`는 임시 파편 수에서 병합·최소 포인트 필터 후 클러스터 수로의 변화를 뜻한다.
- `fragment_drop`은 병합 후에도 `min_cluster_points`를 채우지 못한 파편 수다.
- `reject(...)`는 detection 생성 전 각 Layer 1 gate에서 탈락한 클러스터 수다.
- `assoc`는 1초 동안 누적한 tracker event다. `pairs`, `euclid_reject`, `maha_reject`는 장애물 수가
  아니라 `track × detection` 후보 쌍 수다.
- `track`은 누적합이 아니라 로그 시점의 최신 snapshot이다. `raw/tentative`는 존재 확인 상태,
  `unknown/static/dynamic`은 CONFIRMED track의 motion 상태다. `predicted`는 이번 scan에
  measurement가 없었던 track, `invalid_Pv`는 map 속도 공분산이 유효하지 않았던 visible
  CONFIRMED track 수다.
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
| `static_obs_topic` | `/static_obs` | `f110_msgs/msg/ObstacleArray` | CONFIRMED+UNKNOWN 또는 STATIC 객체의 Frenet 경계와 visible Cartesian AABB |
| `confirmed_static_obs_topic` | `/confirmed_static_obs` | `f110_msgs/msg/ObstacleArray` | CONFIRMED+STATIC만 포함한 장기 지도 입력 |
| `opp_obs_topic` | `/opp_obs` | `f110_msgs/msg/ObstacleArray` | 최근접 동적 상대차 최대 1개의 Frenet 경계와 visible Cartesian AABB |
| `static_markers_topic` | `/static_obs/markers` | `visualization_msgs/msg/MarkerArray` | 최종 `/static_obs` Frenet 경계의 RViz mirror |
| `opp_markers_topic` | `/opp_obs/markers` | `visualization_msgs/msg/MarkerArray` | 최종 `/opp_obs` Frenet 경계의 RViz mirror |

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
| 수명 | `ttl_dynamic`, `ttl_static`, `min_hits_confirm`, `confirmation_window`, `extent_shrink_alpha`, `envelope_stability_tolerance_m`, `envelope_stability_frames` | 3-of-5 존재 확인, track 유지, extent 완화와 정적 레이어 envelope 안정성 gate |
| 분류 | `motion_classification.dynamic_chi2_threshold`, `static_chi2_threshold`, `dynamic_vote_*`, `static_vote_*` | map 속도의 통계적 evidence와 최근 voting |
| 위치 지속성 | `motion_classification.position_history_size`, `static_min_observations`, `static_max_position_rms`, `dynamic_to_static_*` | STATIC 진입과 보수적인 DYNAMIC→STATIC 복귀 |
| 분류 수치 안정성 | `motion_classification.covariance_regularization_epsilon`, `minimum_velocity_covariance`, `static_score_forgetting_factor` | 속도 공분산 regularization과 confidence decay |
| 레이어 병합 | `layer_merge_enable`, `layer_merge_gap_s/d` | tracking 후 같은 레이어 객체 병합 |
| 진단 | `diagnostics_enable`, `diagnostics_period_sec`, `motion_classification.debug_enable`, `debug_period_sec` | 누적 perception 로그와 track별 motion debug 로그 |

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

Track별 motion 통계를 확인하려면
`config/obstacle_detector.yaml`의 `motion_classification.debug_enable`을 `true`로 바꾸거나
직접 실행 시 parameter override를 사용한다.

```bash
ros2 run obstacle_detector obstacle_detector_node --ros-args \
  --params-file "$(ros2 pkg prefix obstacle_detector)/share/obstacle_detector/config/obstacle_detector.yaml" \
  -p motion_classification.debug_enable:=true
```

출력 한 줄에는 ID, `TrackStatus/MotionStatus`, map `vx/vy`, `Tv`, static/dynamic vote,
위치 RMS와 이력 수, static confidence, 마지막 measurement 경과시간이 포함된다.

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

단위 테스트:

```bash
colcon test --packages-select obstacle_detector
colcon test-result --verbose
```

PASS 조건은 동적 상대차가 먼저 `/static_obs`에 provisional로 나타난 뒤 같은 ID로 `/opp_obs`에
이동하고, confirmed 정적 장애물이 `/confirmed_static_obs`에도 나타나며,
5포인트 미만 LiDAR 파편들이 tracking 전에 하나의 detection으로 복원되고, 별도 track으로 남은
조각난 정적 물체도 layer merge에서 하나의 출력 객체로 병합되며, 모든 출력 Frenet 경계가
유한하고 `d_right <= d_left`인 것이다. 폐루프 경계를 넘는 객체의 `s_start > s_end`는 정상이다.
## 통합 시각화 프로파일

launch는 `f1tenth_control/config/runtime_visualization.yaml`을 패키지 YAML 뒤에 읽는다.
`publish_markers=false`이면 `/static_obs/markers`, `/opp_obs/markers` publisher와 MarkerArray
생성을 생략한다. `/static_obs`, `/confirmed_static_obs`, `/opp_obs`에는 영향이 없다.
