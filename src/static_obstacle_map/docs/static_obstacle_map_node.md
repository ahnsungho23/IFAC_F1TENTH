# static_obstacle_map 노드

## 1. 목적

`static_obstacle_map_node`는 `obstacle_detector`가 확정한 정적 장애물을 map 좌표로 저장하고,
저장된 장애물 전체를 `/adaptive_obstacle_map`으로 발행한다.

`/adaptive_obstacle_map`의 타입은 `nav_msgs/msg/OccupancyGrid`가 아니라
`f110_msgs/msg/ObstacleArray`다. 배열의 각 원소가 `f110_msgs/msg/Obstacle`이며, 기본 지도나
벽 점유 정보는 포함하지 않는다. 같은 저장 내용을 RViz에서 확인할 수 있도록
`/adaptive_obstacle_map/markers`도 함께 발행한다.

이 노드는 `/map`을 구독하지 않고 OccupancyGrid를 생성하지 않는다. 경로 생성과 정적·동적
분류는 각각 planner와 `obstacle_detector`가 담당한다.

## 2. 동작 원리

1. `/confirmed_static_obs`에서 `f110_msgs/msg/ObstacleArray`를 받는다.
2. `is_static=true`, `is_visible=true`, `has_cartesian=true`이고 AABB가 유효한 객체만 저장한다.
3. 장애물은 Frenet 좌표가 아니라 map-frame `x_min/x_max/y_min/y_max`로 저장한다.
4. AABB 간격과 detector source ID를 이용해 반복 관측을 기존 저장 항목에 연결한다.
5. 최초 confirmed AABB를 보존하고 이후 관측의 외곽 부분만 bounded union으로 추가한다.
6. 새 외곽 경계는 `edge_match_tolerance_m` 안에서 `edge_confirm_frames`회 연속 관측된 경우에만
   저장 경계에 포함한다.
7. 최초 AABB는 `max_obstacle_diagonal_m`으로 제한한다. 누적 union이 상한을 넘으면 기존
   형상은 유지하고 해당 확장만 거부한다.
8. confirmed 입력이 비거나 관측이 끊겨도 저장 항목을 지우지 않는다.
9. 삽입, 형상 확장, reset 또는 활성화된 동적 재분류 삭제가 발생하면 저장된 전체 장애물을
   `f110_msgs/msg/ObstacleArray`로 다시 발행한다.
10. 시작 직후와 reset 뒤에는 빈 `ObstacleArray`와 `DELETEALL` MarkerArray를 발행한다.
11. 기본 설정에서는 `/opp_obs`의 일시적인 동적 재분류를 구독하지 않는다.

저장소는 메모리에 있으므로 노드가 종료되면 사라진다. 다음 주행 전에는 reset 서비스를
호출할 수 있다.

## 3. 출력 Obstacle 필드

`/adaptive_obstacle_map`은 여러 저장 장애물을 하나의 `ObstacleArray`에 담는다. 단일
`Obstacle`을 반복 발행하지 않으므로 Transient Local QoS에서 마지막 객체 하나만 남는 문제가
없다.

각 `Obstacle.msg` 원소는 다음처럼 구성된다.

| 필드 | 값 |
|---|---|
| `id` | 노드가 부여한 안정적인 memory ID |
| `has_cartesian` | `true` |
| `x_min/x_max/y_min/y_max` | 저장된 map-frame AABB |
| `x_center/y_center` | 저장 AABB 중심 |
| `radius` | AABB 대각선의 절반 |
| `size` | AABB 대각선 |
| `is_static` | `true` |
| `is_visible` | `true` — 저장된 Cartesian 형상이 출력에서 유효함 |

이 노드는 Frenet 기준선을 소유하지 않으므로 `s_start/s_end/d_right/d_left` 등 Frenet 필드는
재계산하지 않는다. adaptive global planner는 출력의 Cartesian 필드를 사용해야 한다.

## 4. 토픽과 서비스

| 방향 | 기본 이름 | 타입 | 설명 |
|---|---|---|---|
| 구독 | `/confirmed_static_obs` | `f110_msgs/msg/ObstacleArray` | confirmed 정적 장애물 |
| 구독 | `/opp_obs` | `f110_msgs/msg/ObstacleArray` | 옵션 활성화 시 동적 재분류 삭제 |
| 발행 | `/adaptive_obstacle_map` | `f110_msgs/msg/ObstacleArray` | 저장된 정적 `Obstacle.msg` 전체 |
| 발행 | `/adaptive_obstacle_map/markers` | `visualization_msgs/msg/MarkerArray` | 저장 AABB의 RViz box |
| 서비스 | `/static_obstacle_map/reset` | `std_srvs/srv/Empty` | 저장 장애물 전체 삭제 |

두 출력은 Reliable + Transient Local QoS를 사용한다. 늦게 시작한 planner와 RViz도 마지막 전체
저장 상태를 받을 수 있다. 기본값 `remove_reclassified_dynamic=false`에서는 `/opp_obs`를 실제로
구독하지 않는다.

## 5. 주요 파라미터

파라미터 파일은 `config/static_obstacle_map.yaml`이다.

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `confirmed_static_obs_topic` | `/confirmed_static_obs` | confirmed 정적 입력 |
| `dynamic_obs_topic` | `/opp_obs` | 선택적 동적 재분류 입력 |
| `output_obstacles_topic` | `/adaptive_obstacle_map` | persistent `ObstacleArray` 출력 |
| `frame_id` | `map` | 출력 ObstacleArray와 MarkerArray 좌표계 |
| `publish_visualization` | `true` | RViz MarkerArray 발행 여부 |
| `visualization_topic` | `/adaptive_obstacle_map/markers` | RViz 출력 |
| `visualization_marker_namespace` | `adaptive_static_obstacles` | marker namespace |
| `visualization_height_m` | `0.15` | box 높이 |
| `visualization_minimum_footprint_m` | `0.025` | box의 최소 가로·세로 크기 |
| `visualization_color_r/g/b/a` | `1.0/0.1/0.1/0.85` | box RGBA 색상 |
| `association_distance_m` | `0.30` | 같은 객체로 연결할 AABB 간 최대 간격 |
| `edge_confirm_frames` | `3` | 새 외곽 경계 확인 횟수 |
| `edge_match_tolerance_m` | `0.05` | 같은 외곽 경계로 인정할 최대 차이 |
| `max_obstacle_diagonal_m` | `0.80` | 저장 AABB 대각선 상한 |
| `remove_reclassified_dynamic` | `false` | `true`일 때만 같은 track의 동적 승격 시 삭제 |
| `reset_service` | `/static_obstacle_map/reset` | 저장 초기화 서비스 |

## 6. 빌드

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-up-to obstacle_detector static_obstacle_map \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
source install/setup.zsh
```

## 7. 실행 및 확인

먼저 detector를 실행한다.

```bash
ros2 launch obstacle_detector obstacle_detector.launch.py
```

다른 터미널에서 저장 노드를 실행한다.

```bash
ros2 launch static_obstacle_map static_obstacle_map.launch.py
```

토픽 타입과 저장 내용을 확인한다.

```bash
ros2 topic type /adaptive_obstacle_map
ros2 topic echo /adaptive_obstacle_map --once
ros2 topic echo /adaptive_obstacle_map/markers --once
```

첫 명령은 `f110_msgs/msg/ObstacleArray`를 출력해야 한다. ROS 그래프에서 `/map` 구독과
`nav_msgs/msg/OccupancyGrid` 발행은 없어야 한다.

RViz2에서는 다음 순서로 표시한다.

1. `Global Options > Fixed Frame`을 `map`으로 설정한다.
2. `Add > MarkerArray`를 선택한다.
3. Topic을 `/adaptive_obstacle_map/markers`로 설정한다.
4. 저장된 confirmed 정적 장애물이 반투명 빨간 box로 유지되는지 확인한다.

저장 내용을 초기화한다.

```bash
ros2 service call /static_obstacle_map/reset std_srvs/srv/Empty "{}"
```

## 통합 시각화 프로파일

launch는 `f1tenth_control/config/runtime_visualization.yaml`을 패키지 YAML 뒤에 읽는다.
`publish_visualization=false`이면 `/adaptive_obstacle_map/markers`만 끄며 핵심
`/adaptive_obstacle_map` ObstacleArray 발행은 유지한다.
