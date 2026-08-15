# obstacle_detector_node

## 1. 노드 목적

`obstacle_detector_node`는 2D LiDAR scan에서 트랙 위의 비지도 장애물을 검출하고 Frenet 프레임에서
추적한다. 확정된 장애물을 정적 레이어와 동적 레이어로 분리해 다음 토픽으로 발행한다.

- `/static_obs`: 지도에는 없는 provisional/confirmed 정적 장애물 전체
- `/confirmed_static_obs`: 장기 지도 저장용 confirmed 정적 장애물만 포함
- `/opp_obs`: 에고 전방의 가장 가까운 동적 상대차 최대 1개와 `is_interfering` 간섭 판정

이 패키지는 검출만 담당한다. 경로 계획, 회피·추월 waypoint, 주행 상태 결정은 다른 패키지의 책임이다.

## 2. 동작 원리

노드는 `/scan`이 들어올 때마다 다음 순서로 실행된다.

### 2.1 기준 경로 준비

1. `/global_waypoints`를 받는다.
2. `FrenetProjector`에 트랙 길이와 `d_left/d_right` 경계를 저장한다.
3. `global_planning::ClcsFrenetConverter`를 생성한다.

기준 경로(컨버터)가 준비되기 전에는 scan을 처리하지 않는다.

### 2.2 Scan 좌표 변환과 군집화

1. scan header의 `frame_id`에서 `map`으로 가는 TF를 조회한다.
2. NaN, Inf, `range_min` 미만, `max_range` 이상 beam을 제거한다.
3. 각 beam을 map 좌표의 점으로 변환한다.
4. 구조적 벽 필터: SLAM 맵에서 추출한 선형 벽 구조로부터 `wall_assoc_distance_m`(기본 0.2 m)
   이내의 점은 Layer 1 구조물로 보고 군집화 전에 beam 단위로 제거한다(2.3절 참조). 제거된
   점은 무효 beam처럼 클러스터 연속성을 끊는다.
5. 인접 beam 사이의 거리 적응형 임계값으로 adaptive-breakpoint clustering을 수행한다.
6. `cluster_merge_enable=true`이면 `cluster_merge_min_fragment_points` 이상의 작은 파편을
   임시로 보존한다.
7. 두 파편의 Cartesian AABB 간격과 실제 점 사이 최소거리가 모두
   `cluster_merge_distance` 이내이고, 병합 AABB 대각선이 `max_obs_size` 이하일 때만
   tracking 전에 병합한다.
8. 병합 후에도 `min_cluster_points`보다 작은 클러스터를 버린다.

현재 detector 내부에는 scan deskew, median filter, temporal scan filter가 없다. 전처리 노드를 추가할
경우 `scan_topic`을 전처리 결과 토픽으로 변경한다.

이 pre-tracking `cluster_merge`는 scan range를 보정하는 필터가 아니다. 끊어진 LiDAR 표면을 하나의
detection으로 복원해 하나의 Kalman track이 생성되도록 하는 segmentation 후처리다.

### 2.3 Layer 1 필터

Layer 1은 벽 구조물을 제거하는 구조적 필터다. SLAM 맵과 현재 scan은 어긋날 수 있으므로(충돌
후 환경 변화, 위치추정 오프셋) 셀 단위 점유 비율 투표 대신 맵에서 벽 "구조"를 직접 추출한다.

맵 수신 시 한 번만 수행하는 전처리:

1. `/map`에서 occupancy가 `map_occupied_thresh` 이상인 셀을 추출한다.
2. 8-연결 flood fill로 connected component를 만든다. 이는 grid 상의 DBSCAN과 동등하다
   (격자 자체가 eps-이웃을 정의).
3. 각 컴포넌트의 셀 좌표에 2x2 PCA를 적용해 선형성을 검사한다. 고유값 비율
   (λmax/λmin)이 `wall_linear_ratio`(기본 4.0) 이상이고 주축 방향 길이가
   `wall_min_length_m`(기본 1.0 m) 이상인 컴포넌트만 벽으로 인정한다. 맵에 구워진 장애물이나
   debris 같은 비선형 blob은 벽이 아니므로, 그 근처의 LiDAR 포인트는 장애물 후보로 유지된다.
4. 모든 벽 셀을 source로 하는 multi-source distance transform(8-이웃 Dijkstra)을 계산해
   셀별 "가장 가까운 벽까지의 거리"를 미리 만들어 둔다.

이후 매 scan에서는 beam을 map 좌표로 변환할 때 쿼리 한 번(O(1))으로 판정한다. 벽까지의
거리가 `wall_assoc_distance_m`(기본 0.2 m) 미만인 점은 Layer 1 구조물로 보고 군집화 전에
제거한다. 격자 밖의 점은 벽으로 판정하지 않는다.

군집화를 통과한 클러스터에는 다음 필터를 순서대로 적용한다.

1. map-frame Cartesian AABB와 대각선 `size`를 계산한다.
2. `size > max_obs_size`인 큰 구조물을 제거하고, `size < min_obs_size`(기본 0.1 m)인
   클러스터는 LiDAR 노이즈로 폐기한다.
3. `aabb_frenet_projector`가 AABB 중심을 Frenet으로 투영해 branch를 고정한다.
4. 중심의 local tangent/normal에서 네 모서리 envelope를 계산해 종방향 반폭과 반대편 횡경계를
   구한다. Race Line을 향한 횡경계는 그 branch 주변의 실제 waypoint 선분과 AABB 네 면 사이
   최단거리로 보정한다.
5. 투영된 AABB 중심을 detection `(s,d)`로 사용하고, 종방향 반폭 및 중심 기준 좌·우 offset을
   함께 tracker에 전달한다.
6. 에고 기준 `view_behind_distance`부터 `max_viewing_distance`까지의 관측 창만 남긴다.
   이때 에고 s는 `convertTracked()`(윈도우 + 히스테리시스, 텔레포트 폴리시 포함)로 얻은
   연속 좌표다. 무상태 전역 탐색으로 에고를 찍으면 헤어핀에서 에고 s가 반대편 다리로
   튀면서 관측 창 전체가 잘못된 branch에 고정될 수 있기 때문이다.
7. waypoint의 `d_left/d_right` 안에 있는 클러스터만 남긴다. 이때 중심이 아니라 팽창된
   envelope의 좌·우 가장자리(`d_left`, `d_right`)를 복도와 비교한다. 벽에 붙은 부채꼴 산란은
   중심은 복도 안이지만 AABB 가장자리는 이미 벽을 파고들기 때문이다.

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

1. 연속 매칭 횟수가 required 미만: `classified=false`, `PENDING`. 장애물 토픽에 발행하지 않는다.
   required는 마지막 측정 거리에 따라 `confirm_frames_near`(0 m, 기본 3)에서
   `confirm_frames_far`(`max_range` 거리, 기본 8)까지 선형 보간된다. 누적이 아닌 연속
   횟수이므로 한 프레임이라도 매칭이 끊기면 카운터가 0으로 리셋되고, 간헐적으로 깜빡이는
   노이즈 blob은 TTL 안에 누적 hit가 많아져도 절대 확정되지 않는다.
2. required번째 연속 매칭: `classified=true`, `PROVISIONAL_STATIC`. 즉시 `/static_obs`에
   발행한다. 단, envelope 안정성 게이트를 함께 통과한 track만 발행 대상이다(아래 참조).
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
`/confirmed_static_obs`에는 `CONFIRMED_STATIC`이면서 같은 envelope 안정성 게이트를 통과한
객체만 별도로 실린다.

envelope 안정성 게이트: 매칭될 때마다 측정 중심과 extent가 track의 예측/보존 값에서
`envelope_stability_tolerance_m`(기본 0.10 m) 이내로 안정됐는지 검사하고, 연속 횟수가
`envelope_stability_frames`(기본 2)에 못 미치는 정적 track은 발행 레이어에서 제외한다.
정사각형 AABB와 실제 형태의 괴리로 형태가 계속 변하는 부채꼴 산란 클러스터는 이 streak를
채우지 못해 `/static_obs`에 나타나지 않는다. envelope가 안정적인 실제 장애물은 확정 시점에
streak를 함께 만족하므로 발행 지연이 추가되지 않는다. prediction-only(미매칭) 프레임에는
streak가 0으로 리셋된다. 또한 `static_publish_requires_visible`(기본 true)가 켜져 있으면
이번 scan에 실제로 매칭된(is_visible) 정적 track만 발행하므로, detection이 끊긴 track은
TTL 동안 내부에 유지되지만 `/static_obs`와 `/confirmed_static_obs`에 ghost로 발행되지
않는다.

기본 `classifier_mode=velocity`에서는 사용하지 않는 positional-std 이력을 저장하거나 분산을 계산하지
않는다. `classifier_mode=std` 또는 `both`일 때만 `std_window` 길이의 `(s,d)` 이력을 유지한다.

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
- 선택된 상대차의 Frenet 횡영역이 ego 차체 폭과 안전 여유로 만든 corridor에 겹치고,
  현재 또는 등속 상대속도 예측 후면 간격이 `interference_distance_m` 이내이면
  `is_interfering=true`로 설정한다.
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
채운다. `x_var/y_var`는 기준 경로 접선으로 공분산을 완전히 회전하기 전까지 보수적으로
`max(s_var, d_var)`를 양축에 사용한다.

Track이 이번 scan에서 detection과 연결되지 않으면 Kalman의 Frenet `(s,d)`는 예측되지만 마지막 raw
Cartesian AABB는 움직이지 않는다. 따라서 component 전체가 predicted-only이면 마지막 측정
Frenet 종·횡 크기를 예측 중심 주위에 유지하되 `has_cartesian=false`로 발행한다. 단, 정적
레이어는 기본 설정(`static_publish_requires_visible=true`)에서 predicted-only 객체를 발행하지
않으므로 이 출력은 동적 레이어에만 나타난다. RViz는 stale
Cartesian AABB 대신 이 예측 Frenet 경계를 낮은 alpha의 테두리로 표시한다.

### 2.8 Perception 진단 통계

`diagnostics_enable=true`이면 `diagnostics_period_sec` 동안 처리 결과를 누적해 INFO 로그 한 줄로
출력한다. 기본 주기는 1초이며 wall clock을 사용하므로 `/clock` 유무와 관계없이 진단 주기가 유지된다.

```text
DIAG perception [1.00s scans=40/40 drop(conv=0 tf=0)]
beam(valid=.../... nonfinite=... below_min=... at_or_above_max=...)
cluster=...->... fragment_drop=... detections=...
reject(size=... projection=... view=... boundary=... map=...)
track(total=... visible=... hit_pending=... provisional=...
      static=... dynamic=... motion_gated=...)
assoc(pairs=... match=... spawn=... retire=... euclid_reject=... maha_reject=...)
motion(yaw_used=... fresh=... ref_vs=... ref_vd=...)
```

- `scans=processed/received`는 수신 scan 중 전체 파이프라인을 끝까지 처리한 개수다.
- `drop(conv/tf)`는 기준 경로 또는 scan→map TF가 없어 scan 전체를 처리하지 못한 횟수다.
- beam 카운터는 유효 beam과 비유한값, `range_min` 미만, `max_range` 이상 탈락 원인을 구분한다.
- `cluster=A->B`는 임시 파편 수에서 병합·최소 포인트 필터 후 클러스터 수로의 변화를 뜻한다.
- `fragment_drop`은 병합 후에도 `min_cluster_points`를 채우지 못한 파편 수다.
- `reject(...)`는 detection 생성 전 각 Layer 1 gate에서 탈락한 수다. `map`만 군집화 전에
  beam 단위로 제거되므로 beam 수이고, 나머지는 클러스터 수다.
- `assoc`는 1초 동안 누적한 tracker event다. `pairs`, `euclid_reject`, `maha_reject`는 장애물 수가
  아니라 `track × detection` 후보 쌍 수다.
- `track`은 누적합이 아니라 로그 시점의 최신 snapshot이다. `hit_pending`은 연속 매칭 확정
  (range-scaled required frames)에 아직 도달하지 못한 track 수다. `motion_gated`는 속도가 `dyn_vel_enter`를 넘었지만 속도 Mahalanobis
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
| `global_waypoints_topic` | `/global_waypoints` | `f110_msgs/msg/WpntArray` | Frenet 기준 경로와 트랙 경계 |
| `map_topic` | `/map` | `nav_msgs/msg/OccupancyGrid` | Layer 1 지도 필터 |
| `ego_odom_topic` | `/pf/pose/odom` | `nav_msgs/msg/Odometry` | 에고 Frenet 위치 |

시뮬레이터 launch에서는 `ego_odom_topic`이 `/ego_racecar/odom`으로 바뀐다.

## 4. 발행 토픽

| 토픽 파라미터 | 기본 토픽 | 메시지 타입 | 내용 |
|---|---|---|---|
| `static_obs_topic` | `/static_obs` | `f110_msgs/msg/ObstacleArray` | provisional/confirmed 정적 객체 전체의 Frenet 경계와 visible Cartesian AABB |
| `confirmed_static_obs_topic` | `/confirmed_static_obs` | `f110_msgs/msg/ObstacleArray` | confirmed 정적 객체만 포함한 장기 지도 입력 |
| `opp_obs_topic` | `/opp_obs` | `f110_msgs/msg/ObstacleArray` | 최근접 동적 상대차, 간섭 여부, Frenet 경계와 visible Cartesian AABB |
| `static_markers_topic` | `/static_obs/markers` | `visualization_msgs/msg/MarkerArray` | 최종 `/static_obs` Frenet 경계의 RViz mirror |
| `opp_markers_topic` | `/opp_obs/markers` | `visualization_msgs/msg/MarkerArray` | 최종 `/opp_obs` Frenet 경계의 RViz mirror |

## 5. 주요 파라미터

운영값은 [`../config/obstacle_detector.yaml`](../config/obstacle_detector.yaml)에 있다.

| 그룹 | 파라미터 | 설명 |
|---|---|---|
| 범위 | `max_range`, `max_viewing_distance`, `view_behind_distance` | LiDAR 및 전후 관측 범위 |
| 군집화 | `lambda_deg`, `cluster_sigma`, `min_2_points_dist` | adaptive-breakpoint 임계값 |
| 파편 병합 | `cluster_merge_enable`, `cluster_merge_distance`, `cluster_merge_min_fragment_points` | tracking 전 작은 LiDAR 파편 병합 |
| 크기 | `min_cluster_points`, `min_obs_size`, `max_obs_size` | 병합 후 최소 beam 수와 최소/최대 객체 크기(AABB 대각선 기준, `min_obs_size` 미만은 노이즈로 폐기) |
| 경계 | `boundaries_inflation`, `fallback_track_halfwidth` | 트랙 내부 통과 조건 |
| 지도 | `use_map_filter`, `map_occupied_thresh`, `wall_assoc_distance_m`, `wall_linear_ratio`, `wall_min_length_m` | 구조적 벽 필터(선형 벽 컴포넌트 추출 + 거리 변환, wall_assoc_distance_m 이내 포인트를 군집화 전 제거) |
| 측정 불확실성 | `meas_range_var_scale`, `meas_sparse_var_scale`, `meas_yaw_rate_var_scale`, `meas_reference_points`, `meas_variance_scale_max`, `meas_motion_timeout` | Detection별 adaptive Kalman `R` |
| 추적 | `meas_var_s/d`, `process_var_vs/vd`, `assoc_gate`, `aggro_multi`, `assoc_use_mahalanobis`, `assoc_mahalanobis_gate` | Kalman 및 2단계 association |
| 수명 | `ttl_dynamic`, `ttl_static`, `confirm_frames_near`, `confirm_frames_far`, `extent_shrink_alpha`, `envelope_stability_tolerance_m`, `envelope_stability_frames`, `static_publish_requires_visible` | track 유지와 발행 확정(연속 매칭 프레임 수, 측정 거리에 따라 near→far 선형 보간, 미매칭 프레임에서 리셋), envelope extent 완화(1.0=기존 덮어쓰기), 정적 레이어 발행 안정성 게이트(측정 중심+envelope가 tolerance 이내로 frames회 연속 안정이고 미매칭 프레임에서 리셋될 때만 /static_obs 발행), prediction-only ghost 발행 억제(기본 true). `ttl_static=25`는 약 250 Hz 입력에서 약 0.1초의 정적 track 검출 공백을 허용 |
| 분류 | `classifier_mode`, `dyn_vel_enter/exit`, `static_confirm_frames`, `dynamic_confirm_frames`, `dyn_velocity_mahalanobis_gate`, `dyn_max_abs_yaw_rate`, `static_ref_gate` | provisional/static/dynamic 판정 |
| 레이어 병합 | `layer_merge_enable`, `layer_merge_gap_s/d` | tracking 후 같은 레이어 객체 병합 |
| 상대차 간섭 | `interference_check_enable`, `interference_distance_m`, `interference_time_horizon_sec`, `interference_min_closing_speed_mps`, `interference_lateral_margin_m`, `interference_ego_half_width_m`, `interference_ego_front_offset_m` | ego corridor 횡겹침과 현재/예측 후면 간격으로 `is_interfering` 판정 |
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

장애물이 포함된 live map과 분리된 clean map을 Layer 1에 사용(선택 — 비선형 blob은 벽으로
인정되지 않으므로 필수는 아님):

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
이동하고, confirmed 정적 장애물이 `/confirmed_static_obs`에도 나타나며,
5포인트 미만 LiDAR 파편들이 tracking 전에 하나의 detection으로 복원되고, 별도 track으로 남은
조각난 정적 물체도 layer merge에서 하나의 출력 객체로 병합되며, 모든 출력 Frenet 경계가
유한하고 `d_right <= d_left`인 것이다. 폐루프 경계를 넘는 객체의 `s_start > s_end`는 정상이다.
