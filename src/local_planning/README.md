# local_planning

`local_planning`은 벽만 포함한 `fuck_f1`의 `/map`과 opponent detector가 발행하는
`/perception/obstacles`를 함께 사용합니다. 이 중 `is_static=true`인 정적 장애물만 planning
grid에 합성하고, 글로벌 기준 경로에서 장애물의 왼쪽 또는 오른쪽으로 우회한 뒤 다시 합류하는
회피 구간을 생성하는 ROS 2 Kilted 패키지입니다. 동적 장애물은 이 패키지에서 경로 생성에
사용하지 않으며 opponent detector의 추월·추종 로직이 담당합니다.

이 패키지의 핵심 역할은 다음과 같습니다.

- `/map`에서 트랙 벽과 주행 경계를 가져옵니다.
- `/perception/obstacles`에서 `is_static=true`, `is_visible=true`인 정적 장애물을 선택합니다.
- 현재 차량 위치부터 일정 거리 앞의 글로벌 경로를 검사합니다.
- 장애물 좌우의 사용 가능한 공간을 비교합니다.
- 좌우 횡방향 오프셋과 전환 길이를 조합해 Frenet lattice 후보군을 만듭니다.
- 충돌 여유, 기준선 이탈, 곡률, 곡률 변화, 경로 길이, 속도 손실을 함께 평가합니다.
- 차량 곡률·횡가속도 한계와 연속 구간 충돌 검사를 통과한 최저 비용 경로만 발행합니다.
- 최초 선택 경로를 회피 종료까지 고정하고 매 주기 안전성을 재검사하여 재계획 흔들림을 방지합니다.
- 시작 직후 Frenet 좌표가 연속적으로 안정화되기 전에는 로컬 경로 발행을 차단합니다.
- 기본 lattice가 실패하면 진입-장애물-합류 전 구간의 safe corridor를 따라 복구 lattice를 탐색하고, 그것도 실패할 때만 충돌 전 점진 감속 경로를 발행합니다.

## 1. 데이터 흐름

```text
                         ┌──────────────────────────────┐
/map (fuck_f1 벽) ──────>│                              │
                         │                              │
/perception/obstacles ──>│     local_planner_node       │
  (is_static=true만 사용)│                              │
                         │                              ├──> /avoid_waypoints
/global_waypoints ──────>│                              ├──> /local_planning/path
                         │                              ├──> /local_path
/car_state/frenet/odom ─>│                              ├──> /local_planning/markers
                         └──────────────────────────────┘
                                        │
                                        │ 회피 경로 후보
                                        v
                  /global_waypoints + /avoid_waypoints + /state
                                        │
                                        v
                                wpnt_publisher
                                        │
                                        v
                                /local_waypoints
                                        │
                                        v
                              f1tenth_control
```

### 입력 데이터

| 토픽 | 메시지 타입 | 사용하는 정보 |
|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | 장애물이 없는 `fuck_f1`의 트랙 벽, map 경계 및 unknown 영역 |
| `/perception/obstacles` | `f110_msgs/msg/ObstacleArray` | opponent detector가 추적한 물체 중 `is_static=true`, `is_visible=true`인 정적 장애물의 Frenet 경계 |
| `/global_waypoints` | `f110_msgs/msg/WpntArray` | `x_m`, `y_m`, `s_m`, `psi_rad`, `kappa_radpm`, 좌우 트랙 폭, 기준 속도 |
| `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | `pose.pose.position.x/y`에 저장된 현재 Frenet `s`, `d` |

운영 시 MCL map server는 반드시 장애물이 구워진 `*_obs` 맵이 아니라 원본 `fuck_f1`을
발행해야 합니다. `/scan`에서 검출한 벽은 opponent detector가 이 `/map`과 비교해 제거하고,
지도에 없는 물체를 추적·분류한 결과를 `/perception/obstacles`로 보냅니다. local planner는
그 배열에서 정적 물체만 사용합니다.

### 출력 데이터

| 토픽 | 메시지 타입 | 의미 |
|---|---|---|
| `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` | 안전 검사를 통과한 차량 전방 회피 구간 |
| `/local_planning/path` | `nav_msgs/msg/Path` | RViz에서 확인하는 로컬 플래너의 전방 경로 |
| `/local_path` | `nav_msgs/msg/Path` | 회피가 반영된 전체 폐루프 경로 |
| `/local_planning/markers` | `visualization_msgs/msg/MarkerArray` | 장애물 중심과 성공·실패 후보 디버그 표시 |
| `/local_waypoints` | `f110_msgs/msg/WpntArray` | 단독 발행 옵션이 켜진 경우에만 이 노드가 직접 발행 |

기본값은 `publish_standalone_local: false`입니다. 따라서 일반적인 시스템에서는 `local_planning`이 `/avoid_waypoints`를 만들고, `wpnt_publisher`가 `/state`에 따라 글로벌·회피·추월 경로 중 하나를 선택하여 최종 `/local_waypoints`를 발행합니다.

## 2. 전체 작동 순서

노드는 기본적으로 `timer_period_ms=50`인 20 Hz 주기로 다음 작업을 반복합니다.

```text
입력 준비 확인
    ↓
Frenet s·d 유효성, 연속 샘플 수, 최신성 확인
    ├─ 짧은 입력 이상/공백 ─────────────────> 마지막 충돌 검증 경로 유지
    ├─ 유지 제한시간 초과 ──────────────────> 빈 회피 경로 발행
    └─ 안정된 입력
          ↓
현재 Frenet s와 가장 가까운 글로벌 웨이포인트 탐색
    ↓
전방 detection_lookahead_wpnt_num 구간의 장애물 간섭 검사
    ↓
장애물 검출 히스테리시스 적용
    ↓
고정된 회피 경로 존재 여부 확인
    ├─ 존재하고 남은 구간이 안전함 ─────────> 같은 경로의 남은 구간 발행
    └─ 없거나 기존 경로가 더 이상 안전하지 않음
          ↓
좌우 여유 공간 비교로 선호 방향 결정
          ↓
양쪽 방향의 횡오프셋 × 전환 길이 기본 lattice 생성
          ↓
트랙 경계·OccupancyGrid·곡률·횡가속도 검사
          ↓
안전 후보의 다목적 비용 계산
    ├─ 안전 후보 존재 ──────────────────────> 최저 비용 후보 고정 후 발행
    └─ 안전 후보 없음 ──────────────────────> 복구 lattice 최대 84개 추가 탐색
          ├─ 안전 후보 존재 ─────────────────> 최저 비용 복구 후보 고정 후 발행
          ├─ 충돌 전 정지 구간 존재 ─────────> 점진 감속 경로 발행
          └─ 정지 구간도 없음 ───────────────> 빈 회피 경로 발행
```

## 3. 벽 지도와 perception 장애물의 합성

### 3.1 지도 변경 감지

`/map`은 같은 `fuck_f1`을 반복 발행할 수 있습니다. 매번 전체 지도를 다시 분석하지 않도록
지도 크기, 해상도, origin 위치·회전, 모든 점유 셀 값으로 내용 서명을 만들고, 실제 내용이
바뀐 경우에만 기반 grid를 다시 계산합니다.

### 3.2 점유 셀 분류

OccupancyGrid 셀 값이 `occupied_threshold`보다 크면 점유 셀로 간주합니다. 기본 임계값은 50입니다.

기반 `/map`의 전체 점유 셀은 트랙 벽과 map 경계 안전검사에 사용합니다. 그 위에 perception
장애물을 점유값 100인 셀로 합성한 **planning grid**를 경로 검출과 최종 충돌 판정에 사용합니다.
따라서 합성된 정적 장애물이 벽에 붙거나 큰 컴포넌트가 되어도 계획 입력에서 빠지지 않습니다.

### 3.3 `/perception/obstacles` 정적 장애물 합성

기본 설정은 다음과 같습니다.

```yaml
use_perception_obstacles: true
obstacles_topic: "/perception/obstacles"
perception_static_only: true
perception_obstacle_padding_m: 0.06
freeze_committed_static_obstacles: true
committed_obstacle_match_distance_m: 0.35
```

각 `Obstacle`은 다음 조건을 모두 만족할 때만 planning grid에 들어갑니다.

- `is_static == true`
- `is_visible == true`
- `is_actually_a_gap == false`

`s_center`, `d_center`를 글로벌 기준선에서 map 좌표로 변환하고, `s_start/s_end`와
`d_left/d_right`로 표현된 Frenet 박스를 트랙 접선 방향의 회전 사각형으로 rasterize합니다.
검출 불확실성과 셀 이산화 오차를 위해 `perception_obstacle_padding_m`만큼 외곽 여유를
추가합니다. 동적 물체(`is_static=false`)는 local planning grid에 넣지 않습니다.

회피 경로를 확정할 때 사용한 정적 장애물 박스는 merge 완료까지 snapshot으로 고정합니다.
이후 검출이 `committed_obstacle_match_distance_m` 안에서 흔들리면 같은 장애물로 판단해 snapshot
좌표를 유지합니다. snapshot과 매칭되지 않는 새로운 장애물은 현재 검출값으로 grid에 추가하므로
새 장애물에 대한 재검증은 계속 수행합니다.

점유 셀의 BFS 기반 8방향 연결 요소 탐색은 그룹 이름, 중심점 시각화와 로그에만 사용합니다.
대각선으로 붙어 있는 셀도 같은 컴포넌트로 처리합니다.

컴포넌트의 실제 면적은 다음과 같이 계산합니다.

```text
component_area = occupied_cell_count × map_resolution²
```

면적에 따라 시각화용 컴포넌트를 다음과 같이 분류합니다.

- `obstacle_component_min_area_m2`보다 작음: 중심점 마커에서 제외
- `obstacle_component_max_area_m2`보다 큼: 벽 그룹으로 기록
- 두 값 사이: 독립 컴포넌트 중심점을 빨간 구체로 표시

`obstacle_mask_`에는 컴포넌트 면적과 무관하게 전체 점유 셀이 저장됩니다. RViz의 빨간 구체는
충돌 지점이 아니라 시각화 대상으로 그룹화된 컴포넌트 중심점입니다.

지도 원점에 회전이 있는 경우에도 올바르게 처리하도록 world/map 좌표 변환에 OccupancyGrid origin의 yaw를 적용합니다.

## 4. 차량 위치와 전방 검사 구간 결정

현재 차량 위치는 다음 값을 사용합니다.

```text
current_s = /car_state/frenet/odom.pose.pose.position.x
```

이 값과 글로벌 웨이포인트의 `s_m`을 비교하여 현재 위치와 가장 가까운 웨이포인트 인덱스를 찾습니다. 트랙은 폐루프이므로 `s`가 트랙 길이를 넘어가면 나머지 연산으로 처음 구간에 연결합니다.

현재 인덱스부터 두 개의 범위를 사용합니다.

- `lookahead_wpnt_num`: 후보 경로를 만들 수 있는 전체 계획 범위
- `detection_lookahead_wpnt_num`: 실제로 장애물 간섭을 검사하는 범위

현재 설정은 웨이포인트 간격이 약 0.1 m인 `fuck_f1`을 기준으로 160개, 120개입니다. 따라서 약 16 m를 계획하고 약 12 m 전방부터 장애물을 검사합니다.

## 5. 글로벌 경로와 장애물의 간섭 검사

### 5.1 명시적 Frenet 안전 회랑

계획 범위의 각 sampled `s`마다 차량 중심이 들어갈 수 있는 횡구간을 만듭니다. 트랙 경계를
차량 반폭, 기존 보수적 원형 반경, 위치추정 여유, 추가 안전 여유 중 더 보수적인 값으로 줄입니다.

```text
lateral_support = max(
    vehicle_radius + path_clearance_margin,
    vehicle_width / 2 + localization_margin + corridor_safety_margin
)

track_interval = [
    -d_right + lateral_support,
     d_left  - lateral_support
]
```

각 점유 셀의 횡구간도 같은 차량 여유와 셀 반대각선만큼 팽창한 뒤 `track_interval`에서 뺍니다.
그 결과를 `blocked_intervals`, `feasible_intervals`, `left_feasible`, `right_feasible`로 명시적으로
보관합니다. 후보의 `d(s)`가 어느 sampled `s`에서든 feasible interval 밖이면 후보 전체를 hard
reject합니다. 기본 동작은 quintic 점을 경계에 clamp하지 않으므로 원래 5차 형상이 훼손되지 않습니다.

### 5.2 장애물 셀을 웨이포인트 좌표계로 투영

웨이포인트 위치를 `(x_w, y_w)`, 진행 방향을 `ψ`, 장애물 셀을 `(x_o, y_o)`라고 하면 상대 위치는 다음과 같이 종방향과 횡방향으로 변환됩니다.

```text
dx = x_o - x_w
dy = y_o - y_w

longitudinal =  dx cos(ψ) + dy sin(ψ)
lateral      = -dx sin(ψ) + dy cos(ψ)
```

종방향 검색 반폭은 고정 `±0.15 m`가 아닙니다. `longitudinal_search_half_width_m`을 최솟값으로
하고, 주변 waypoint 반간격, 차량의 앞/뒤 footprint, 위치추정·안전 여유, map 셀 반대각선을
더해 자동 확장합니다. 따라서 waypoint 정중앙 사이의 얇은 장애물이나 대각선 셀도 인접한 `s`
sample의 회랑에 포함됩니다. 폐루프에서는 waypoint 인덱스를 나머지 연산으로 감고 별도의
`unwrapped_s`를 누적합니다.

### 5.3 full-grid 기반 차단 판단

글로벌 기준선의 `d=0`이 팽창된 blocked interval에 포함되면 planning trigger로 기록합니다.
이 판단은 connected-component ID나 면적을 사용하지 않습니다. 회피 방향은 각 차단 sample에서
실제로 남은 왼쪽/오른쪽 feasible interval 폭의 합으로 정합니다.

## 6. 회피 방향 선택 로직

충돌 예상 지점마다 장애물 경계와 트랙 경계 사이의 남은 공간을 계산합니다.

```text
left_room  = left_limit - (obstacle_max_d + safety_margin)
right_room = (obstacle_min_d - safety_margin) - right_limit
```

모든 충돌 지점에서 양수인 여유 공간을 방향별로 합산합니다.

```text
sum(left_room) >= sum(right_room)  → 왼쪽 우선
sum(left_room) <  sum(right_room)  → 오른쪽 우선
```

이 선택은 우선순위일 뿐입니다. 선호 방향의 후보가 최종 충돌 검사에 실패하면 반대 방향도 자동으로 검사합니다.

## 7. 회피 구간 결정

첫 충돌 인덱스를 `collision_begin`으로 잡고, 이후 충돌 샘플 사이의 간격이
`lattice_obstacle_cluster_gap_wpnts` 이내인 **가장 가까운 장애물 군집**까지만
`collision_end`로 묶습니다. 멀리 떨어진 장애물을 하나의 고정 횡오프셋으로 동시에 풀지 않고,
현재 군집을 통과한 뒤 다음 주기에 다음 장애물을 다시 계획합니다. 경로가 갑자기 꺾이지 않도록
이 충돌 구간 전후에 전환 구간을 추가합니다.

```text
window_begin = collision_begin - window_margin
window_end   = collision_end   + window_margin
```

일반 구간은 `spline_window_margin_wpnts`, 급커브는 `corner_spline_window_margin_wpnts`를 사용합니다. 코너에서는 Frenet 법선 방향이 빠르게 회전하므로 너무 긴 횡이동을 만들면 경로가 코너 안쪽 벽을 가로지를 수 있습니다. 따라서 급커브에서는 더 짧은 전환 구간을 사용합니다.

## 8. Frenet lattice 후보 생성

### 8.1 횡방향 목표 샘플

장애물 경계와 `safety_margin`으로 각 방향에서 반드시 필요한 최소 오프셋을 먼저 계산합니다.

```text
required_left  = max(minimum_avoid_offset, obstacle_max_d + safety_margin)
required_right = min(-minimum_avoid_offset, obstacle_min_d - safety_margin)
```

그다음 `collision_begin`부터 `collision_end`까지 모든 safe corridor를 교차해, 구간 전체에서
계속 유지되는 좌측·우측 통과 구간을 각각 구합니다. 목표 `d`는 고정 간격으로 통로 밖까지
늘리지 않고 이 공통 통로의 선호점, 중앙, 양쪽 내부 경계에서
`lattice_lateral_samples`개까지 샘플링합니다. `lattice_corridor_target_inset_m`만큼 경계에서
안쪽으로 들어오므로 수치 오차로 corridor hard reject가 발생하는 것도 막습니다.

좁고 휘어진 구간이나 대각선 장애물에서는 각 `s` 단면에 통과 공간이 있어도 전 구간에 공통인
하나의 고정 `d`가 없을 수 있습니다. 이때는 `lattice_corridor_knot_stride_wpnts` 간격으로 중간
단면의 feasible interval을 샘플링하고 bounded beam search로 이어지는 횡오프셋 조합을 찾습니다.
거친 knot 간격으로 연결할 수 없으면 간격을 절반씩 줄여 waypoint 단위까지 재시도합니다.
각 단계에서는 `lattice_corridor_beam_width`개의 연속성이 좋은 부분 경로만 남겨 조합 폭발을
막습니다. 생성된 전체 곡선은 모든 중간 safe-corridor 단면, 차량 footprint, 트랙 경계에 대해
다시 hard reject 검사를 받습니다. 따라서 좁은 길을 통과시키기 위해 경계를 완화하거나 점별
clamp를 사용하지 않습니다. 어느 한 단면에도 해당 방향의 feasible interval이 없을 때만 그
방향을 실제로 막힌 것으로 처리합니다.

### 8.2 종방향 전환 길이 샘플

일반 구간은 `spline_window_margin_wpnts`, 급커브는 `corner_spline_window_margin_wpnts`를 기준 전환 길이로 사용합니다. 여기에 `lattice_transition_scales`의 각 값을 곱해 짧고 긴 전환 후보를 동시에 만듭니다.

기본 설정은 긴 전환 길이부터 평가해 장애물 직전 급격한 횡이동을 줄입니다. 좌우 양쪽의
후보를 모두 안전 검사하고 비용이 가장 낮은 유효 경로를 선택합니다.

이미 회피경로가 확정된 상태에서는 global 중심선만 검사하지 않습니다. 현재 committed
경로의 각 waypoint `d_m`도 새 safe corridor 안에 남아 있는지 검사합니다. 새 장애물이
`d=0`을 막지 않더라도 이전 회피의 횡오프셋이나 합류 경로를 막으면 다음 장애물로 인식해
즉시 재계획합니다. 이 경우 로그에 `Active-path obstacle trigger`가 출력됩니다.

`ifac_track`에서는 약 0.1 m 간격의 waypoint 120개를 검사하므로 정적 장애물을 최대 약
12 m 전방에서 계획 대상으로 봅니다. 실제 최초 인식 거리는 LiDAR 가시성과
`opponent_detector`의 track 확정 시점에 의해 짧아질 수 있습니다. local path는 약 18 m를
발행하고, 장애물 상자 흔들림과 차량 폭·위치추정·제어 추종 오차를 포함한 보수적 footprint로
검증합니다. 따라서 global 중심선 바로 위에 없더라도 차체 swept area에 걸치는 장애물은
회피 대상으로 처리합니다.
공통 통로 폭이 `lattice_narrow_corridor_width_threshold_m`보다 작으면
`lattice_narrow_corridor_transition_scales`를 추가합니다. 긴 전환은 좁은 통로 입구에서 차체 yaw가
크게 틀어져 모서리가 벽을 침범하는 현상을 줄입니다. 안전 회랑이나 footprint 기준은 완화하지
않습니다.

32개가 모두 탈락하면 `lattice_recovery_*` 설정으로 충돌 여유를 바꾸지 않은 2차 탐색을
수행합니다. 장애물 구간 전체에 공통 `d` 통로가 있으면 계산이 저렴한 조밀 quintic 후보를
바로 검사하고, 공통 통로가 없을 때만 전구간 beam profile을 전환 길이와 방향마다
`lattice_recovery_profile_limit`개까지 정밀 평가합니다. Primary와 recovery는 각각 설정된
시간 예산을 넘으면 탐색을 종료합니다.
더 짧고 긴 합류 거리와 더 촘촘한 횡오프셋을 확인합니다. 복구 모드의 추가 곡률 허용분은
경로를 그대로 빠르게 주행하라는 의미가 아니며, 실제 속도는 동일한 횡가속도 한계로 다시
제한됩니다.

### 8.3 knot의 의미

`knot`(매듭점)는 최종 waypoint 전체가 아니라, 복잡한 회피 경로의 큰 형상을 정하는 소수의
Frenet 기준점 `(s, d)`입니다.

```text
진입점          knot 1          knot 2          합류점
(s0, d0) ──── (s1, d1) ──── (s2, d2) ──── (s3, 0)
```

- `s`: 글로벌 기준선을 따라 전진한 호길이
- `d`: 글로벌 기준선으로부터의 좌우 오프셋
- knot: 특정 `s`에서 지나갈 목표 `d`

장애물 구간 전체에 공통인 하나의 안전한 `d`가 있으면 중간 knot 없이 일정한 회피 오프셋을
사용할 수 있습니다. 반대로 커브나 대각선 장애물처럼 safe corridor가 위치에 따라 옆으로
이동하면 여러 knot를 corridor 안에서 선택합니다. bounded beam search는 가능한 knot 조합 중
연속성 비용이 좋은 일부만 유지하며 탐색하고, 선택된 knot 사이를 아래의 5차 Hermite 구간으로
연결합니다. 최종 local waypoint는 이 연속 곡선을 글로벌 waypoint 간격으로 샘플링한 결과입니다.

따라서 knot는 차량이 하나씩 추종해야 하는 별도의 명령점이 아니라, 경로 생성기가 곡선의
형상을 결정하기 위해 사용하는 제어 기준점입니다.

### 8.4 5차 Frenet 횡이동

각 후보는 Werling 등(2010)의 저속 Frenet 궤적 생성식과 같은 형태로, 실제 글로벌 기준선의
호길이 `s`를 독립변수로 하는 다음 5차 다항식을 사용합니다.

```text
h(u) = 6u⁵ - 15u⁴ + 10u³,  u = (s - s_start) / S,  0 ≤ u ≤ 1
```

충돌 구간 전에는 현재 `d`에서 첫 목표 오프셋까지 이동합니다. 공통 통로가 있으면 장애물을
지나는 동안 같은 목표값을 유지하고, 공통 통로가 없으면 여러 corridor knot를 조각별 5차
Hermite 곡선으로 연결합니다. 내부 knot의 `d'(s)`는 이웃 secant의 부호와 간격을 사용하는
PCHIP 가중 조화평균으로 정합니다. 따라서 단조 구간은 불필요하게 멈추거나 overshoot하지 않고,
실제 횡방향 극값에서만 기울기가 0이 됩니다. 모든 조각의 `d''(s)`는 knot에서 0이고 양쪽
`d'(s)`가 같아 위치·접선·2차 미분이 연속입니다. 첫 진입과 마지막 `d=0` 합류의 기울기는 0으로
유지합니다. waypoint 번호가 아니라 실제 누적 호길이를 사용하므로 점 간격이 불균일해도 같은
거리 기준의 횡전환을 유지합니다.

이 구현은 경로와 속도가 분리된 현재 ROS 인터페이스에 맞춰 논문의 저속 모드 `d(s)`를
적용한 것입니다. 논문의 고속 `d(t)` 및 종방향 quartic/quintic 속도 궤적은 동적 장애물의
시간 예측이 필요한 별도 확장 범위이며, 현재 perception 기반 정적 장애물 회피에는 적용하지 않습니다.

```text
x_local = x_global - d · sin(ψ_global)
y_local = y_global + d · cos(ψ_global)
```

### 8.5 spline, 5차 다항식, 최소자승 다항식의 차이

현재 구현을 단순히 “일반 장애물은 5차, 곡선 장애물은 spline”이라고 구분하면 정확하지
않습니다. 선택 기준은 장애물 모양 자체가 아니라 **장애물 구간 전체에 공통인 고정 `d`가
존재하는가**입니다.

| 방법 | 현재 구현에서의 의미 | 장점 | 주의점 |
|---|---|---|---|
| 단일 5차 다항식 | 시작 `d`에서 목표 `d`로 한 번에 전환 | 위치·기울기·2차 미분 경계조건을 직접 만족하고 계산이 단순함 | 이동하는 좁은 corridor를 한 구간으로 통과하기 어려울 수 있음 |
| 조각별 5차 Hermite | 여러 knot 사이마다 5차 구간을 만들고 하나의 연속 곡선으로 연결 | 복잡하게 이동하는 corridor를 따라갈 수 있음 | knot와 내부 기울기를 잘못 정하면 흔들림이나 overshoot가 생길 수 있음 |
| 일반적인 spline | 여러 점을 조각별 다항식으로 연결하는 방법의 총칭 | 많은 기준점을 부드럽게 연결하기 좋음 | spline의 차수·경계조건·보간법에 따라 성질이 달라짐 |
| 최소자승 다항식 | 모든 점을 정확히 지나지 않고 전체 오차가 작도록 하나의 식을 fitting | 잡음이 있는 측정값을 근사하는 데 유용 | 안전 경계를 반드시 통과해야 하는 회피에서는 corridor 위반과 국부 overshoot를 직접 막기 어려움 |

즉, 현재 복잡 구간의 경로는 넓은 의미에서는 piecewise polynomial spline이라고 부를 수 있지만,
각 조각은 **5차 Hermite 다항식**입니다. 내부 knot의 기울기는 PCHIP의 shape-preserving 규칙을
사용하며, 과거의 최소자승 cubic spline fallback은 현재 안전 검증 구조와 일치하지 않아
제거되어 있습니다.

### 8.6 후보 형상과 속도 재계산

오프셋을 적용한 좌표에서 중심 차분으로 `psi_rad`를 다시 계산하고, 연속한 세 점의 외접원 곡률로 `kappa_radpm`을 갱신합니다. 따라서 제어기는 글로벌 경로 곡률이 아니라 실제 회피 경로 형상을 전달받습니다.

장애물 전과 장애물 옆에서는 회피 감속 비율을 적용합니다. 장애물의 마지막 충돌 예상점을
지난 뒤에는 현재 횡오프셋이 목표 오프셋에서 `d=0`으로 줄어드는 비율을 계산하고, 5차
smoothstep으로 글로벌 속도까지 점진적으로 복구합니다. 따라서 로컬 경로가 끝날 때까지
고정된 저속을 유지하지 않으면서도 합류 시작점에서 속도가 갑자기 뛰지 않습니다.

```text
v_curve = lattice_speed_safety_factor
          × sqrt(lattice_max_lateral_accel_mps2 / |kappa|)

recovery = smoothstep(1 - |d_current| / |d_avoid|)
speed_ratio = avoidance_ratio
              + (post_obstacle_ratio - avoidance_ratio) × recovery

v_candidate = min(v_global × speed_ratio, v_curve)
```

후보 곡률이 `lattice_max_curvature_radpm`보다 크거나 속도와 곡률로 계산한 횡가속도가 한계를
넘으면 해당 후보를 제거합니다. 마지막으로 전방 가속 패스와 후방 감속 패스를 적용해 모든
waypoint 사이의 종가속도를 `lattice_max_longitudinal_accel_mps2`와
`lattice_max_longitudinal_decel_mps2` 안으로 제한합니다. 후방 패스는 장애물이나 큰 곡률에
도달하기 전에 감속을 시작하고, 전방 패스는 장애물 통과 후 속도 회복을 안정적으로 만듭니다.

## 9. Frenet lattice 비용함수

안전 검사를 통과한 후보만 다음 비용으로 비교합니다.

```text
J = w_clearance        × mean(1 / free_clearance)
  + w_min_clearance    × normalized_min_clearance_deficit²
  + w_deviation        × mean(d²)
  + w_curvature        × mean(kappa²)
  + w_curvature_change × mean((Δkappa / Δs)²)
  + w_spatial_jerk     × integral(d'''(s)² ds)
  + w_maneuver_length  × S
  + w_path_length      × additional_path_length
  + w_speed_loss       × mean(relative_speed_loss²)
  + opposite_side_penalty
```

- `clearance`: 트랙 벽과 장애물에서 멀리 떨어진 후보를 선호합니다.
- `min_clearance_deficit`: 경로에서 가장 좁은 지점이 `lattice_preferred_clearance_m`보다 작으면 별도 벌점을 줍니다. 긴 전환 구간의 평균에 위험 지점이 희석되지 않도록 하는 항입니다.
- `deviation`: 필요 이상으로 글로벌 레이싱 라인에서 벗어나는 것을 억제합니다.
- `curvature`: 큰 조향각이 필요한 경로를 억제합니다.
- `curvature_change`: 거리당 곡률 변화율 `|Δkappa|/Δs`를 평가하여 조향이 짧은 거리에서 좌우로 빠르게 반전되는 경로를 억제합니다. `lattice_max_curvature_rate_radpm2`를 넘는 후보는 비용 계산 전에 제거합니다.
- `spatial_jerk`: 논문의 저속 Frenet 비용 `J_s=∫d'''(s)²ds`입니다. 현재 5차 전환에서는 각
  전환의 값을 `720·Δd²/S⁵`로 정확히 계산하며, 짧고 급격한 횡이동을 강하게 억제합니다.
- `maneuver_length`: 논문의 전환 거리 항 `k_t S`입니다. jerk 비용만 사용할 때 필요 이상으로
  긴 전환을 고르는 현상을 막아 두 항 사이에서 부드러움과 신속한 합류를 절충합니다.
- `path_length`: 불필요하게 긴 우회를 억제합니다.
- `speed_loss`: 속도를 지나치게 낮춰야 하는 후보를 불리하게 만듭니다.
- `opposite_side_penalty`: 회피 중 고정된 방향에 우선권을 줍니다. 현재 방향 후보가 모두 충돌하면 반대편의 안전 후보로 전환할 수 있습니다.

고정된 순서에서 첫 경로를 선택하던 이전 방식과 달리, 현재 방식은 모든 안전 후보를 평가한 후 비용이 가장 작은 경로를 발행합니다.

사용한 논문 개념과 프로젝트 고유 확장은 [16. 참고 문헌과 구현 대응 관계](#16-참고-문헌과-구현-대응-관계)에
구분하여 정리했습니다.

## 10. 연속 충돌 검사와 실패 처리

후보 생성과 최종 검사는 모두 `/map`의 벽과 perception 정적 장애물을 합성한 planning grid
전체를 사용하므로 두 종류의 점유 구조를 모두 확인합니다.

```text
collision_radius = vehicle_radius + path_clearance_margin
```

각 웨이포인트뿐 아니라 웨이포인트 사이도 `lattice_collision_sample_step_m` 이하 간격으로
위치와 yaw를 보간합니다. 각 pose에서 base_link 기준 oriented rectangle footprint와 기존 보수적
원형을 함께 검사합니다. 점유 셀, unknown 정책 위반, 지도 밖 footprint, 트랙 회랑 위반은 hard
reject입니다. 동일한 보간점의 clearance도 soft clearance 비용에 포함되므로 waypoint 사이의
가장 좁은 지점이 비용에서 빠지지 않습니다.

기본 lattice가 실패하면 더 촘촘한 복구 lattice를 먼저 계산합니다. 복구 모드도
`vehicle_radius + path_clearance_margin` 충돌 반경과 planning grid 검사를 그대로 적용하므로,
토픽을 채우기 위해 벽이나 장애물을 통과하는 후보를 강제로 선택하지 않습니다.

연속 장애물 때문에 기존 경로의 합류 지점에서 재계획하는 도중 복구 후보까지 모두 실패하면, 먼저 직전에 검증한 전체 회피 경로를 현재 위치부터 다시 검사합니다. 트랙 경계, 각 pose의 footprint, 선분 내부의 swept collision 검사를 통과한 prefix만 잘라 `frenet_lattice_replan_brake`로 발행합니다. 이 경로는 `lattice_replan_brake_timeout_sec` 안에서만 사용할 수 있고, 속도를 다시 올리지 않는 제동 프로파일을 적용해 마지막 웨이포인트에서 0이 됩니다. 따라서 일시적인 후보 공백 때문에 `/avoid_waypoints`가 비거나 GLOBAL 경로로 순간 전환되는 현상을 막으면서도, 이미 막힌 과거 경로를 강제로 재사용하지 않습니다.

이전 전체 경로에서 안전한 prefix를 만들 수 없으면 글로벌 경로 중 장애물 전까지 충돌 검사를 통과한 구간을 자르고, `lattice_safe_stop_deceleration_mps2`로 계산한 제동 속도 프로파일을 넣습니다. 이전처럼 전 구간 속도를 즉시 0으로 만들지 않으므로 제어 명령이 갑자기 정지로 바뀌지 않으며, 마지막 웨이포인트에서만 속도가 0이 됩니다. 두 방법 모두 최소 2개의 안전한 웨이포인트를 만들지 못하고 제한된 제동 경로 유지시간도 끝났을 때만 빈 `/avoid_waypoints`와 빈 `/local_planning/path`를 발행합니다.

이전 최소자승 다항식과 독립 smoothstep fallback은 실제 lattice 실패 시 사용되고 있었지만, 점·선분 충돌만 확인하고 수정 경로의 곡률·횡가속도를 lattice와 동일하게 검증하지 않아 제거했습니다. 5차 smoothstep 함수 자체는 각 lattice 후보의 부드러운 횡전환에만 사용되며, 그 결과는 lattice의 전체 동역학·연속 충돌 검사를 거칩니다.

시작 직후에는 `frenet_odom_confirm_cycles`개의 연속된 Frenet 입력이 확인될 때까지 경로 생성을 막습니다. 주행 중 `s`가 크게 튀거나 `d`가 트랙 폭을 벗어난 입력은 `frenet_odom_invalid_grace_cycles`개까지 직전 유효 상태로 넘깁니다. 입력 공백이 `frenet_odom_stale_timeout_sec`를 넘더라도 이미 검증된 회피·제동 경로가 있으면 `frenet_odom_path_hold_timeout_sec`까지 그 경로를 유지합니다. 제한시간이 끝나거나 지도·글로벌 경로가 바뀌면 캐시를 폐기합니다. 회피 전환이 현재 계획 시작점부터 시작되는 경우에는 첫 후보의 `d`를 현재 차량 `d`에 맞춰 시작점 급변도 방지합니다.

글로벌 웨이포인트에 NaN/Inf가 있거나 `s_m`이 증가 순서가 아니면 입력을 거부합니다. 주행 중 글로벌 경로의 형상이나 속도가 바뀌면 이전 경로에서 만든 회피 commitment를 폐기하고 Frenet 입력을 다시 확인합니다. `/map` 캐시도 셀 내용뿐 아니라 해상도와 origin 위치·회전을 포함하므로, 같은 픽셀 데이터가 다른 좌표계로 다시 들어오는 경우 이전 변환을 재사용하지 않습니다.

## 11. 경로 고정과 검출 히스테리시스

매 20 Hz 주기마다 최저 비용 후보를 새로 발행하면, 비슷한 비용을 가진 좌우 후보나 인접한 횡오프셋 후보가 번갈아 선택될 수 있습니다. `lattice_commit_path_until_clear=true`이면 최초로 검증된 전체 회피 경로와 합류 인덱스를 저장합니다.

다음 주기부터는 새로운 후보를 만들기 전에 저장된 경로의 현재 차량 위치부터 합류점까지를 다시 검사합니다. 모든 점과 선분이 계속 안전하면 동일한 기하 경로의 남은 구간만 잘라서 발행합니다. 장애물이 검출 범위 뒤로 사라진 것은 정상적인 통과 과정이므로 이때는 경로를 해제하지 않습니다. 지도 변경으로 기존 경로가 막혔거나 다음 장애물이 계획 범위에 들어온 경우에만 lattice를 다시 계산합니다.

합류점까지만 잘라서 발행하면 차량이 합류점에 가까워질수록 제어기가 받는 전방 경로가 지나치게 짧아집니다. 이를 방지하기 위해 실제 합류 인덱스 뒤에 `lattice_post_merge_lookahead_wpnts`개의 글로벌 waypoint를 덧붙입니다. 기본값 40은 현재 약 0.1 m 간격에서 약 4 m의 추가 전방 경로입니다.

추가 구간도 planning grid에 대해 점과 선분 충돌 검사를 통과한 만큼만 발행합니다. 경로의 합류
인덱스를 통과했더라도 실제 차량의 Frenet `|d|`가 `lattice_merge_lateral_tolerance_m`보다 크면
아직 횡복귀가 끝나지 않은 것으로 봅니다. 이때는 최대 `lattice_merge_settle_max_wpnts` 동안
같은 경로에 포함된 글로벌 후속 구간을 계속 발행합니다. 실제 차량까지 글로벌 라인에 안정화된
뒤에만 빈 종료 메시지를 보내므로 제어기가 횡복귀 도중 `avoid → global → avoid`로 전환되지 않습니다.

회피 방향도 저장되며 `lattice_weight_opposite_side`가 재계획 시 불필요한 방향 전환을 억제합니다. 현재 방향에서 안전 후보가 하나도 없으면 반대 방향 후보를 사용할 수 있으므로 경로 고정이 충돌 검사를 우회하지는 않습니다.

한 프레임의 일시적인 검출 변화로 회피 상태가 반복 전환되는 것을 방지하기 위해 히스테리시스를 사용합니다.

- `detection_confirm_cycles`: 연속으로 장애물을 확인해야 회피를 활성화하는 횟수
- `detection_clear_cycles`: 연속으로 장애물이 없어야 회피를 해제하는 횟수

정적 perception 장애물은 현재 기본값으로 첫 검출 주기에 바로 활성화합니다. 경로가 아직
고정되지 않은 상태에서는 장애물이 사라진 상태를 3주기 연속 확인한 뒤 해제합니다. 이미 회피
경로를 고정했다면 검출 해제보다 경로 합류와 실제 차량의 횡방향 안정화를 우선하므로,
장애물을 통과하는 중간에 글로벌 경로로 조기 전환되지 않습니다.

## 12. 발행 결과의 의미

`/avoid_waypoints`의 `ot_line`에는 실제로 선택된 생성 방법이 기록됩니다.

| `ot_line` 값 | 의미 |
|---|---|
| `frenet_lattice_segment` | 다중 후보 중 최저 비용 Frenet lattice 경로가 선택됨 |
| `frenet_lattice_recovery` | 기본 후보 실패 후 더 촘촘한 복구 lattice 경로가 선택됨 |
| `frenet_lattice_replan_brake` | 연속 장애물 재계획 공백에서 직전 전체 경로를 다시 충돌 검사해 만든 제동 prefix |
| `*_held` | 짧은 Frenet 입력 이상 동안 마지막 충돌 검증 경로를 제한시간 내 재발행함 |
| `frenet_lattice_safe_stop` | 기본·복구 후보가 실패해 장애물 전의 점진 감속 구간을 발행함 |
| `frenet_lattice_stationary_hold` | 감속 경로 소진 후 현재 위치의 충돌 검증된 0속도 경로를 유지하며 재계획함 |
| `no_safe_path` | 현재 정지 위치까지 충돌 검사를 통과하지 못해 발행 가능한 경로가 없음 |
| `invalid_frenet_odom` | Frenet 입력이 아직 안정화되지 않았거나 오래됨 |

`side_switch`는 유효한 회피 구간이 있을 때 `true`가 되며, `ot_side`에는 `left` 또는 `right`가 기록됩니다.

RViz 디버그 마커의 의미는 다음과 같습니다.

- 빨간 구체: 간섭 검사를 통과한 장애물 컴포넌트의 중심
- 초록 선: 최종 충돌 검사를 통과한 안전 경로
- 연두색 경계: 차량 footprint를 반영한 트랙 안전 회랑 경계
- 빨간 횡선: 점유/unknown 셀로 막힌 횡구간
- 청록/파란 횡선: 왼쪽/오른쪽의 통과 가능한 횡구간
- 반투명 주황 구체: 회랑 계산에 포함된 팽창 장애물 셀
- 흰색 텍스트: 회랑·트랙·점유·unknown·곡률·횡가속도별 후보 거절 횟수와 직전 latency

## 13. 핵심 로직 의사 코드

```text
on_map(map):
    if map_content_changed:
        base_grid = fuck_f1_wall_cells(map, unknown_policy)

on_perception_obstacles(obstacles):
    static_obstacles = filter(
        obstacles, is_static=true, is_visible=true, is_actually_a_gap=false)
    planning_grid = overlay_frenet_boxes_as_occupied_cells(
        base_grid, static_obstacles, global_waypoints)
    obstacle_mask = all_occupied_cells(planning_grid, unknown_policy)
    clearance_field = rebuild_clearance(planning_grid)

every_timer_cycle():
    if global_waypoints, map, frenet_odom are not ready:
        return

    if frenet_odom is transiently unstable or stale:
        if last_validated_path age <= path_hold_timeout:
            publish(last_validated_path)
        else:
            clear_committed_path()
            publish_empty_avoid_waypoints()
        return

    start = closest_global_waypoint(current_frenet_s)
    corridor = build_safe_corridor(
        wrapped_s_horizon,
        track_boundaries,
        planning_grid,  # /map 벽 + perception 정적 장애물
        vehicle_footprint + localization_margin + safety_margin)
    collisions = samples_where_d_zero_is_blocked(corridor)
    update_detection_hysteresis(collisions)

    if avoidance_is_not_active:
        publish_empty_avoid_waypoints()
        return

    nearest_cluster = first_collision_cluster(
        collisions, lattice_obstacle_cluster_gap_wpnts)
    preferred_side = side_with_more_clearance(nearest_cluster)
    candidates = []

    for side in [preferred_side, opposite_side]:
        common_passage = intersect_side_corridors(corridor, nearest_cluster, side)
        target_offsets = sample_inside(common_passage)
        target_profiles = [constant_profile(target_d) for target_d in target_offsets]
        if target_profiles is empty:
            target_profiles = beam_search_over_corridor_knots(
                knot_stride, beam_width, side)
        transition_lengths = sampled_transition_lengths()
        if passage_width(common_passage, target_profiles) < narrow_corridor_threshold:
            transition_lengths += longer_narrow_corridor_transitions()
        for target_profile in target_profiles:
            for transition_length in transition_lengths:
                candidate = piecewise_quintic_frenet_path(
                    target_profile, pchip_internal_slopes, transition_length)
                if any candidate.d is outside corridor:
                    reject(candidate)  # 점별 clamp 없음
                recompute_heading_curvature_and_velocity(candidate)
                if dynamically_feasible(candidate) and full_footprint_swept_collision_free(candidate):
                    candidate.cost = evaluate_multi_objective_cost(candidate)
                    candidates.append(candidate)

    if candidates is empty:
        candidates = generate_recovery_lattice_without_relaxing_collision_clearance()

    if candidates is not empty:
        publish(minimum_cost(candidates))
    else:
        safe_stop = collision_free_prefix_of_last_validated_full_path()
        if safe_stop has at least two waypoints and cache is not expired:
            apply_nonaccelerating_braking_profile(safe_stop)
            publish(safe_stop)
        else:
            safe_stop = collision_free_global_segment_before_obstacle()
            if safe_stop has at least two waypoints:
                apply_gradual_braking_profile(safe_stop)
                publish(safe_stop)
            else:
                publish_empty_avoid_waypoints()
```

## 14. 주요 튜닝 값

모든 운영 값은 [config/local_planning.yaml](config/local_planning.yaml)에서 조정합니다.

| 목적 | 주요 파라미터 |
|---|---|
| 더 일찍 장애물 탐지 | `detection_lookahead_wpnt_num` 증가 |
| 더 긴 계획 구간 사용 | `lookahead_wpnt_num` 증가 |
| safe corridor 내부 횡방향 후보 변경 | `lattice_lateral_samples`, `lattice_corridor_target_inset_m` 조정 |
| 전환 길이 후보 변경 | `lattice_transition_scales`, `lattice_min_transition_wpnts` 조정 |
| 좁은 통로의 긴 전환 후보 변경 | `lattice_narrow_corridor_width_threshold_m`, `lattice_narrow_corridor_transition_scales` 조정 |
| 기본 탐색 실패 후 복구 범위 변경 | `lattice_recovery_lateral_samples`, `lattice_recovery_transition_scales`, `lattice_recovery_profile_limit`, `lattice_recovery_max_curvature_scale`, `lattice_recovery_max_curvature_rate_scale` 조정 |
| 탐색 지연 및 오래된 결과 폐기 기준 변경 | `lattice_primary_search_budget_ms`, `lattice_recovery_search_budget_ms`, `lattice_max_result_pose_drift_m` 조정 |
| 일반 구간 기준 전환 길이 변경 | `spline_window_margin_wpnts` 조정 |
| 코너 기준 전환 길이 변경 | `corner_spline_window_margin_wpnts` 조정 |
| 장애물과 더 멀리 떨어진 후보 선호 | `lattice_weight_clearance` 증가 |
| 벽-장애물 사이 최소 이격을 더 보수적으로 선택 | `lattice_preferred_clearance_m`, `lattice_weight_min_clearance` 증가 |
| 글로벌 라인에 가까운 후보 선호 | `lattice_weight_deviation` 증가 |
| 더 부드러운 후보 선호 | `lattice_weight_curvature`, `lattice_weight_curvature_change`, `lattice_weight_spatial_lateral_jerk` 증가 |
| 회피 중 반대 방향 선택을 더 강하게 억제 | `lattice_weight_opposite_side` 증가 |
| 회피 경로 고정 사용 여부 | `lattice_commit_path_until_clear` 변경 |
| 합류 직전 제어기 전방 경로 길이 변경 | `lattice_post_merge_lookahead_wpnts` 조정 |
| 실제 차량이 글로벌 라인에 복귀한 뒤 전환 | `lattice_merge_lateral_tolerance_m`, `lattice_merge_settle_max_wpnts` 조정 |
| lattice 전체 실패 시 정지 여유·감속도 변경 | `lattice_safe_stop_buffer_wpnts`, `lattice_safe_stop_deceleration_mps2` 조정 |
| 연속 장애물 재계획 시 직전 전체 경로 사용 제한시간 | `lattice_replan_brake_timeout_sec` 조정 |
| 초기 위치 안정화 조건 변경 | `frenet_odom_confirm_cycles`, `frenet_odom_max_s_jump_m` 조정 |
| Frenet 입력 이상 허용 및 경로 유지 시간 변경 | `frenet_odom_invalid_grace_cycles`, `frenet_odom_stale_timeout_sec`, `frenet_odom_path_hold_timeout_sec` 조정 |
| 짧고 빠른 후보 선호 | `lattice_weight_maneuver_length`, `lattice_weight_path_length`, `lattice_weight_speed_loss` 증가 |
| 장애물 최소 이격 자체를 증가 | `safety_margin`, `avoid_offset` 증가 |
| 차량 충돌 반경 증가 | `vehicle_radius`, `path_clearance_margin` 증가 |
| 코너 추종 오차를 더 보수적으로 반영 | `corner_tracking_margin` 증가 |
| 허용 곡률·곡률 변화율·횡가속도 변경 | `lattice_max_curvature_radpm`, `lattice_max_curvature_rate_radpm2`, `lattice_max_lateral_accel_mps2` 조정 |
| 회피 구간 속도 변경 | `speed_reduction_ratio`, `corner_speed_reduction_ratio` 조정 |
| 장애물 통과 후 속도 회복 변경 | `post_obstacle_speed_recovery_ratio` 조정 |
| 속도 프로파일의 급가감속 제한 | `lattice_max_longitudinal_accel_mps2`, `lattice_max_longitudinal_decel_mps2` 조정 |

튜닝 시에는 경로 생성 여부만 보지 말고 `/local_planning/path`와 실제 차량 추종 궤적을 함께 확인해야 합니다. 경로 자체가 충돌하지 않아도 제어기의 추종 오차가 크면 실제 차량은 벽이나 장애물과 충돌할 수 있습니다.

더 세부적인 토픽, 파라미터, 실행 방법은 [docs/local_planner.md](docs/local_planner.md)를 참고하세요.

## 15. 자주 묻는 로직 질문

### 15.1 safe corridor가 회피 방향과 최종 안전성을 모두 결정하는가?

일부는 맞지만 safe corridor 하나만으로 최종 경로가 결정되지는 않습니다.

1. 각 `s` 단면에서 차량 중심이 들어갈 수 있는 좌우 횡구간을 계산합니다.
2. 장애물 군집 전체의 좌우 통과 폭을 비교해 선호 방향을 정합니다.
3. 양쪽 corridor 안에서 목표 `d` 또는 knot 조합을 샘플링합니다.
4. 생성된 모든 후보가 각 safe-corridor 단면 안에 있는지 hard reject 검사합니다.
5. footprint, waypoint 사이 swept collision, 트랙 경계, 곡률, 곡률 변화율, 횡가속도를 추가로 검사합니다.
6. 모든 검사를 통과한 후보만 비용으로 비교합니다.

따라서 safe corridor는 **가능한 통로와 선호 방향을 제공하는 Frenet 기반 1차 안전 제약**입니다.

### 15.2 corridor 검사를 통과했는데 footprint 검사를 또 하는 이유는 무엇인가?

safe corridor는 각 Frenet 단면에서 차량 중심이 들어갈 수 있는 범위를 보수적으로 빠르게
계산합니다. 하지만 실제 차량은 회피 중 yaw가 변하는 회전 직사각형이며, Frenet 단면 사이에서
모서리가 장애물에 닿거나 waypoint 사이 선분이 점유 셀을 가로지를 수 있습니다. 또한 지도
origin 회전, 곡률이 큰 기준선, 이산 샘플 오차도 존재합니다.

그래서 최종 검사는 map 좌표계에서 다음을 확인합니다.

- 각 pose의 oriented-rectangle footprint
- 보수적인 원형 충돌 반경
- waypoint 사이를 보간한 swept segment
- 점유 셀, unknown 정책, 지도 및 트랙 경계

corridor는 후보를 빠르게 제한하고, footprint/swept 검사는 실제 차체 형상으로 마지막 안전성을
보증하므로 둘은 중복이 아니라 서로 다른 수준의 검사입니다.

### 15.3 폭이 넓은 쪽만 선택하는가?

아닙니다. 좌우 폭 비교는 `preferred_side`를 정할 뿐 반대편 후보를 즉시 폐기하지 않습니다.
양쪽에서 안전 후보를 만들고, 반대 방향에는 `lattice_weight_opposite_side` 비용을 추가합니다.
따라서 폭이 조금 좁더라도 더 짧고, 부드럽고, 충분히 안전한 반대편 경로의 총비용이 더 낮으면
처음 선택될 수 있습니다.

다만 이미 한 방향의 회피 경로를 commit한 뒤에는 좌우 전환으로 인한 흔들림을 막기 위해 강한
hysteresis를 적용합니다. 현재 방향에 안전 후보가 하나라도 있으면 그 방향을 유지하고, 모두
실패할 때만 반대 방향으로 바꿉니다.

### 15.4 primary 32개와 recovery 84개의 관계는 무엇인가?

기본 설정의 최대 후보 수는 다음 조합에서 나옵니다.

```text
primary:  좌우 2 × 횡방향 목표 4 × 전환 길이 4 = 최대 32
recovery: 좌우 2 × 횡방향 목표 7 × 전환 길이 6 = 최대 84
```

먼저 primary 후보를 모두 생성·검증합니다. 안전 후보가 하나라도 있으면 recovery를 실행하지
않고 그중 최소 비용 경로를 선택합니다. primary가 전부 탈락할 때만 더 촘촘한 횡목표와 더
다양한 전환 길이로 recovery를 수행합니다. 실제 생성 수는 corridor 폭이나 중복 제거에 따라
최댓값보다 작을 수 있습니다.

recovery는 안전 여유를 낮추는 우회 수단이 아닙니다. corridor와 footprint 충돌 기준은 그대로
유지하며, 곡률 및 곡률 변화율 한계만 설정된 배율 안에서 넓혀 탐색합니다. 속도는 동일한
횡가속도 한계로 다시 제한됩니다.

### 15.5 primary와 recovery가 모두 실패하면 어떻게 되는가?

차량이 검증되지 않은 직선 경로를 계속 주행하도록 두지 않습니다.

1. 최근 commit 경로의 남은 부분을 현재 지도에서 다시 검사합니다.
2. 안전한 prefix가 있으면 가속하지 않는 점진 제동 경로로 발행합니다.
3. 그것도 불가능하면 글로벌 경로에서 장애물 전까지 안전한 구간을 찾아 safe-stop을 만듭니다.
4. 짧은 입력 공백에는 이미 검증된 제동 경로만 제한시간 동안 유지합니다.
5. 감속 경로가 소진되면 현재 Frenet 위치에 충돌 검증된 2점 0속도 경로를 발행하면서
   매 planning 주기마다 이동 가능한 회피 경로를 다시 탐색합니다.
6. 현재 정지 위치의 footprint조차 안전하지 않을 때만 빈 avoid path를 발행합니다.

최종 `/local_waypoints` 선택과 정지 명령은 `wpnt_publisher`와 상태 머신의 연결 상태에도 영향을
받으므로, 실패 로그를 볼 때는 `/avoid_waypoints`, `/state`, `/local_waypoints`를 함께 확인해야
합니다.

### 15.6 local path와 global path는 언제 전환되는가?

local planner가 만든 경로에는 장애물 회피뿐 아니라 글로벌 기준선으로 돌아오는 **merge 구간**과
그 뒤의 글로벌 lookahead가 포함됩니다. 기하학적 merge 인덱스를 통과하고 실제 차량의
`|d|`가 `lattice_merge_lateral_tolerance_m` 안으로 안정화될 때까지 commit 경로를 유지합니다.
그 후 `wpnt_publisher`가 상태에 따라 글로벌 waypoint를 선택합니다.

연속 장애물에서는 기존 merge에 거의 도착한 뒤부터 계산을 시작하면 늦습니다. 현재 구현은
다음 장애물 군집이 기존 merge 부근의 horizon에 들어오면 merge 약 2~3 m 전부터 선행 재계획을
시도합니다. 새 후보가 완전한 corridor·footprint·동역학 검증과 시작점 gap 제한을 통과했을 때만
commit을 교체하고, 실패하면 기존 검증 경로를 그대로 유지합니다.

## 16. 참고 문헌과 구현 대응 관계

현재 플래너는 하나의 논문을 그대로 재현한 구현이 아니라, 아래 문헌의 수학적 개념을 가져와
이 프로젝트의 OccupancyGrid 기반 안전 로직과 결합한 것입니다.

### 16.1 Frenet 궤적 생성과 비용

M. Werling, J. Ziegler, S. Kammel, S. Thrun, “Optimal Trajectory Generation for Dynamic
Street Scenarios in a Frenét Frame,” *IEEE International Conference on Robotics and
Automation (ICRA)*, 2010, DOI: [`10.1109/ROBOT.2010.5509799`](https://doi.org/10.1109/ROBOT.2010.5509799).

이 문헌에서 사용한 개념은 다음과 같습니다.

- 기준 경로를 따라가는 Frenet 좌표계에서 후보 궤적을 생성하는 구조
- 저속 상황에서 횡이동을 시간 `t` 대신 기준선 호길이 `s`의 함수 `d(s)`로 표현하는 방식
- 시작·끝 위치와 미분 경계조건을 만족하는 quintic polynomial
- 횡방향 jerk 적분과 전환 길이를 포함해 여러 후보를 비교하는 발상

현재 구현은 perception 정적 장애물 회피에 맞춰 저속 `d(s)` 부분을 사용합니다. 논문의 동적 객체
시간 예측, 고속 `d(t)`, 종방향 quartic/quintic 속도 궤적 전체를 구현한 것은 아닙니다. clearance,
최소 clearance deficit, footprint, 곡률 변화, 속도 손실, 반대 방향 벌점도 프로젝트에서 추가한
항목입니다.

### 16.2 PCHIP 내부 knot 기울기

F. N. Fritsch and J. Butland, “A Method for Constructing Local Monotone Piecewise Cubic
Interpolants,” *SIAM Journal on Scientific and Statistical Computing*, vol. 5, no. 2,
1984, DOI: [`10.1137/0905021`](https://doi.org/10.1137/0905021).

F. N. Fritsch and R. E. Carlson, “Monotone Piecewise Cubic Interpolation,” *SIAM Journal
on Numerical Analysis*, vol. 17, no. 2, 1980, DOI:
[`10.1137/0717021`](https://doi.org/10.1137/0717021).

이 문헌들에서 가져온 핵심은 데이터의 단조 형상을 보존하고 overshoot를 억제하는 내부 절점
기울기 선택 원리입니다. 현재 코드는 이웃 secant의 부호가 같을 때 Fritsch–Butland 계열의
가중 조화평균으로 knot의 `d'(s)`를 정하고, 부호가 바뀌는 실제 극값에서는 기울기를 0으로
둡니다.

중요한 차이는 문헌의 보간 조각 자체는 cubic인 반면, 현재 플래너는 그 **기울기 선택 규칙**을
사용해 knot 사이를 quintic Hermite 조각으로 연결한다는 점입니다. 즉 PCHIP 논문의 cubic 식을
그대로 복사한 구현은 아닙니다.

### 16.3 프로젝트에서 추가한 안전·운영 설계

다음 항목은 위 문헌에 그대로 제시된 단일 알고리즘이 아니라, 이 트랙과 ROS 2 시스템에서
안전한 연속 주행을 위해 결합하거나 새로 설계한 부분입니다.

- OccupancyGrid와 트랙 폭으로 만드는 wrapped-s safe corridor
- 좌우 corridor 폭을 이용한 선호 방향과 반대 방향 비용 벌점
- 공통 `d`가 없을 때 corridor knot를 찾는 bounded beam search
- oriented footprint와 waypoint 사이 swept collision hard reject
- primary 32개 실패 뒤 안전 기준을 유지하는 recovery 84개 탐색
- 경로 commitment, 방향 hysteresis, merge 이후 글로벌 lookahead
- 다음 장애물 군집의 merge 2~3 m 전 선행 재계획과 검증 후 교체
- 전체 후보 실패 시 검증된 prefix 제동 및 global safe-stop
- 지도·corridor·commit 충돌 검사 캐시와 저주기 RViz 디버그 발행

따라서 논문이나 보고서에는 “Werling의 Frenet quintic 후보 생성과 jerk 비용을 기반으로 하고,
Fritsch–Butland/Fritsch–Carlson의 shape-preserving 기울기 원리를 복수 knot 연결에 적용했으며,
safe corridor와 실제 footprint 기반 안전 검증 및 복구·commit 로직을 추가했다”고 기술하는 것이
현재 코드와 가장 정확하게 일치합니다.
