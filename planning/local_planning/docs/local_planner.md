# Local Planner (local_planning) 패키지 문서

`local_planning`은 ROS 2 Humble 환경에서 정적 장애물 간섭을 감지하고, 최소자승법(Least Squares Method)을 이용한 3차 다항식 스플라인 회피 경로를 실시간으로 생성하는 C++ 기반 로컬 플래너 패키지입니다.

---

## 1. 개요 및 동작 원리

1. **글로벌 경로 및 맵 수신**: `/global_waypoints`(`f110_msgs/msg/WpntArray`)와 `/map`(`nav_msgs/msg/OccupancyGrid`, 장애물 맵 메이커로 생성된 `fuck_f1_obs` 맵 등)을 구독합니다.
2. **장애물 간섭 검사**: `/car_state/frenet/odom`을 기반으로 차량 전방 지정된 개수(`lookahead_wpnt_num`)의 웨이포인트 주변(안전 마진 반경)을 격자 지도에서 탐색하여 장애물 셀(Occupancy > 50)과의 간섭 여부를 검사합니다.
3. **회피 방향 판단**: 감지된 장애물의 상대 횡방향 위치 및 트랙 여유폭(`d_left`, `d_right`)을 비교하여 좌측 또는 우측 회피 방향을 결정합니다.
4. **최소자승법 스플라인 생성**: Eigen 라이브러리의 QR 분해(`colPivHouseholderQr`)를 사용하여 전방 거리 $s$에 대한 회피 오프셋 $d(s) = c_0 + c_1 s + c_2 s^2 + c_3 s^3$의 최적 계수를 도출합니다.
5. **토픽 발행**: 계산된 스플라인 경로를 기존 `wpnt_publisher`와 호환되는 `/planner/avoidance/otwpnts` 및 스탠드얼론 `/local_waypoints`로 발행합니다.

---

## 2. 입출력 토픽 (Topic Interfaces)

| 구분 | 토픽명 | 메시지 타입 | 설명 |
|---|---|---|---|
| **Subscribe** | `/global_waypoints` | `f110_msgs/msg/WpntArray` | 글로벌 레퍼런스 경로 |
| **Subscribe** | `/map` | `nav_msgs/msg/OccupancyGrid` | 장애물이 포함된 격자 지도 |
| **Subscribe** | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | 프레네 좌표계 기준 차량 오도메트리 |
| **Publish** | `/local_path` | `nav_msgs/msg/Path` | 글로벌 경로와 엮인 하나의 닫힌 곡선(Closed Loop) 로컬 경로 |
| **Publish** | `/planner/avoidance/otwpnts` | `f110_msgs/msg/OTWpntArray` | `wpnt_publisher` 연동용 회피 스플라인 웨이포인트 |
| **Publish** | `/local_waypoints` | `f110_msgs/msg/WpntArray` | 직접 발행되는 로컬 웨이포인트 (옵션) |
| **Publish** | `/local_planning/path` | `nav_msgs/msg/Path` | RViz 시각화용 로컬 경로 (Path) |
| **Publish** | `/local_planning/markers` | `visualization_msgs/msg/MarkerArray` | 충돌 감지 지점(적색) 및 스플라인(녹색) 디버깅 마커 |

---

## 3. 핵심 파라미터 (`config/local_planning.yaml`)

### 3.1 핵심 알고리즘 파라미터
- `lookahead_wpnt_num` (int, 기본값: 40): 로컬 플래너가 한번에 연산할 전방 웨이포인트 개수
- `safety_margin` (double, 기본값: 0.65): 웨이포인트 중심에서 장애물을 감지할 마진 반경 (m)
- `wall_margin` (double, 기본값: 0.35): 벽 안전 마진 (m)
- `avoid_offset` (double, 기본값: 1.0): 장애물 회피 시 목표로 하는 횡방향 거리 (m)
- `poly_degree` (int, 기본값: 3): 최소자승법 다항식 차수 (기본 3차 다항식 스플라인)
- `speed_reduction_ratio` (double, 기본값: 0.6): 회피 경로 주행 시 적용할 속도 감속 비율
- `publish_standalone_local` (bool, 기본값: true): `/local_waypoints` 토픽 직접 발행 여부
- `timer_period_ms` (int, 기본값: 500): 연산 타이머 주기 (ms, 기본 500ms / 0.5초 주기 갱신)

### 3.2 토픽 및 좌표계 프레임 파라미터
- `global_waypoints_topic` (string, 기본값: `"/global_waypoints"`): 글로벌 웨이포인트 구독 토픽명
- `map_topic` (string, 기본값: `"/map"`): 점유 격자 지도(Occupancy Grid Map) 구독 토픽명
- `frenet_odom_topic` (string, 기본값: `"/car_state/frenet/odom"`): 프레네 좌표계 오도메트리 구독 토픽명
- `ot_waypoints_topic` (string, 기본값: `"/planner/avoidance/otwpnts"`): 회피 웨이포인트(OTWpntArray) 발행 토픽명
- `local_waypoints_topic` (string, 기본값: `"/local_waypoints"`): 스탠드얼론 로컬 웨이포인트 발행 토픽명
- `local_path_topic` (string, 기본값: `"/local_planning/path"`): 로컬 플래닝 Path 메시지 발행 토픽명
- `exact_local_path_topic` (string, 기본값: `"/local_path"`): 닫힌 곡선 Local Path 메시지 발행 토픽명
- `marker_topic` (string, 기본값: `"/local_planning/markers"`): RViz 디버깅 마커 배열 발행 토픽명
- `frame_id` (string, 기본값: `"map"`): 기본 좌표계 프레임 ID

---

## 4. 실행 방법

### 4.1 기본 런치 명령어
```bash
ros2 launch local_planning local_planning.launch.py
```

### 4.2 파라미터 파일 직접 지정 실행
```bash
ros2 launch local_planning local_planning.launch.py params_file:=/home/myungsub/2026_IFAC/planning/local_planning/config/local_planning.yaml
```
