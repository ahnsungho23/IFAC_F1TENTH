# static_obstacle_map 노드

## 1. 목적

`static_obstacle_map_node`는 장애물이 없는 Layer 1 기본 지도에 perception이 확정한 정적
장애물을 추가해 `/adaptive_obstacle_map`으로 발행한다. 장애물은 일시적으로 LiDAR에서 보이지
않아도 주행이 끝날 때까지 메모리에 유지되며, adaptive global path는 벽과 확정 정적 장애물이
모두 포함된 하나의 `OccupancyGrid`를 사용할 수 있다.

RViz에서는 `/adaptive_obstacle_map`을 `Map` 디스플레이로 표시하고,
`/adaptive_obstacle_map/markers`를 `MarkerArray` 디스플레이로 겹쳐 표시할 수 있다. MarkerArray는
저장된 confirmed 정적 장애물의 map-frame AABB를 반투명 box로 강조한다.

이 노드는 경로를 생성하거나 장애물을 분류하지 않는다. 정적/동적 분류는
`obstacle_detector`가 담당한다.

## 2. 동작 원리

1. `base_map_topic`에서 장애물이 없는 `nav_msgs/msg/OccupancyGrid`를 받는다.
2. `/confirmed_static_obs`에서 `ConfirmedStatic` 객체만 받는다.
3. `is_static=true`, `is_visible=true`, `has_cartesian=true`이며 AABB가 유효한 객체만 저장한다.
4. 장애물은 Frenet 좌표가 아니라 map-frame `x_min/x_max/y_min/y_max`로 저장한다.
5. 같은 객체는 AABB 간격과 source ID를 이용해 기존 저장 항목에 연결한다.
6. 처음 confirmed된 AABB를 보존하고, 이후 관측에서 기존 경계 밖으로 나온 부분을 bounded
   union으로 추가한다. 따라서 차량이 장애물을 통과한 뒤의 부분 scan이 앞에서 확보한 형상을
   덮어쓰거나 축소하지 않는다.
7. 새 외곽 경계가 `edge_match_tolerance_m` 안에서 `edge_confirm_frames`회 연속 관측된 경우에만
   저장 경계를 확장한다. 한 프레임의 튀는 scan은 영구 형상에 들어가지 않는다.
8. 최초 AABB는 `max_obstacle_diagonal_m`으로 제한한다. 기존 장애물의 누적 union이 이 값을
   넘으면 기존 형상을 이동·축소하지 않고 새 확장만 거부한다.
9. 출력할 때마다 최신 기본 지도를 새로 복사하고 저장 장애물 전체를 다시 rasterize한다.
10. 저장 정보는 confirmed callback마다 갱신하고, 큰 OccupancyGrid의 재합성·발행은
   `publish_period_ms`로 제한한다. 새 장애물, 기본 지도, reset과 활성화된 동적 재분류 삭제는
   즉시 반영한다.
11. confirmed 토픽이 비거나 객체 관측이 끊겨도 저장 항목은 지우지 않는다.
12. 기본 설정에서는 같은 detector track이 `/opp_obs`에서 잠시 동적으로 재분류되어도 저장
    항목을 유지한다. 따라서 분류기가 static/dynamic 사이에서 흔들려도 confirmed marker와
    합성 지도 셀이 사라지지 않는다.
13. 장시간 검증된 동적 재분류 신호를 사용하는 경우에만
    `remove_reclassified_dynamic=true`로 설정해 같은 track의 저장 항목을 제거한다.

저장소는 메모리에 있으므로 노드가 종료되면 사라진다. 같은 프로세스로 다음 주행을 시작할 때는
reset 서비스를 호출한다.

## 3. 토픽과 서비스

| 방향 | 기본 이름 | 타입 | 설명 |
|---|---|---|---|
| 구독 | `/map` | `nav_msgs/msg/OccupancyGrid` | 벽만 포함하는 Layer 1 기본 지도 |
| 구독 | `/confirmed_static_obs` | `f110_msgs/msg/ObstacleArray` | confirmed 정적 장애물 |
| 구독 | `/opp_obs` | `f110_msgs/msg/ObstacleArray` | 옵션 활성화 시 동적 재분류 정정 |
| 발행 | `/adaptive_obstacle_map` | `nav_msgs/msg/OccupancyGrid` | 벽과 저장 정적 장애물의 합성 지도 |
| 발행 | `/adaptive_obstacle_map/markers` | `visualization_msgs/msg/MarkerArray` | 저장 정적 장애물 AABB의 RViz 오버레이 |
| 서비스 | `/static_obstacle_map/reset` | `std_srvs/srv/Empty` | 저장 장애물 전체 삭제 |

기본 지도 구독, 합성 지도 발행, RViz MarkerArray 발행은 Reliable + Transient Local QoS를
사용한다. 늦게 실행된 adaptive global path와 RViz도 최신 상태를 한 번에 받을 수 있다.
MarkerArray는 매 발행 앞에 `DELETEALL`을 포함하므로 reset, 위치 갱신, 동적 재분류 뒤에 이전
box가 RViz에 남지 않는다. 지도 벽의 모든 occupied cell은 Marker로 중복하지 않아 토픽 크기를
억제한다.

## 4. 주요 파라미터

파라미터 파일은 `config/static_obstacle_map.yaml`이다.

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `base_map_topic` | `/map` | Layer 1 기본 지도 |
| `confirmed_static_obs_topic` | `/confirmed_static_obs` | confirmed 정적 입력 |
| `dynamic_obs_topic` | `/opp_obs` | 동적 재분류 입력 |
| `output_map_topic` | `/adaptive_obstacle_map` | 합성 지도 출력 |
| `publish_visualization` | `true` | RViz MarkerArray 발행 여부 |
| `visualization_topic` | `/adaptive_obstacle_map/markers` | RViz MarkerArray 출력 |
| `visualization_marker_namespace` | `adaptive_static_obstacles` | RViz marker namespace |
| `visualization_height_m` | `0.15` | box 높이 |
| `visualization_color_r/g/b/a` | `1.0/0.1/0.1/0.85` | box RGBA 색상 |
| `publish_period_ms` | `100` | 저장 갱신 중 OccupancyGrid 재합성·발행 최소 주기 |
| `association_distance_m` | `0.30` | 같은 객체로 연결할 AABB 간 최대 간격 |
| `edge_confirm_frames` | `3` | 새 외곽 경계를 누적하기 위한 연속 관측 횟수 |
| `edge_match_tolerance_m` | `0.05` | 같은 외곽 경계로 인정할 관측 간 최대 차이 |
| `max_obstacle_diagonal_m` | `0.80` | 저장 AABB 대각선의 절대 상한 |
| `obstacle_inflation_m` | `0.0` | 출력 grid에만 적용하는 추가 여유 |
| `occupied_value` | `100` | 장애물 cell 값 |
| `remove_reclassified_dynamic` | `false` | `true`일 때만 같은 track의 동적 승격 시 저장 삭제 |
| `clear_on_base_map_geometry_change` | `true` | 지도 형상이 바뀌면 저장 초기화 |

`max_obstacle_diagonal_m`은 저장되는 원본 장애물 크기에 적용된다. 누적 확장이 상한을 넘으면
기존 앞쪽 형상을 유지하고 해당 확장을 거부한다. `obstacle_inflation_m`은 planner 안전 여유를
위한 별도 출력 확장이며 저장 크기로 다시 들어가지 않아 누적되지 않는다.

`remove_reclassified_dynamic=false`이면 노드는 `/opp_obs`를 구독하지 않는다. 최근 detector
분류처럼 동일 track이 짧은 간격으로 static/dynamic을 오갈 때도 confirmed 저장 결과는 reset,
노드 종료 또는 설정된 base-map geometry 변경 전까지 유지된다.

## 5. 빌드

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-up-to obstacle_detector static_obstacle_map \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
source install/setup.zsh
```

## 6. 실행

먼저 detector를 실행한다.

```bash
ros2 launch obstacle_detector obstacle_detector.launch.py
```

다른 터미널에서 합성 지도 노드를 실행한다.

```bash
ros2 launch static_obstacle_map static_obstacle_map.launch.py
```

출력을 확인한다.

```bash
ros2 topic echo /adaptive_obstacle_map --once
ros2 topic echo /adaptive_obstacle_map/markers --once
```

RViz2에서는 다음 순서로 추가한다.

1. `Global Options > Fixed Frame`을 `map`으로 설정한다.
2. `Add > Map`에서 Topic을 `/adaptive_obstacle_map`으로 선택한다.
3. `Add > MarkerArray`에서 Topic을 `/adaptive_obstacle_map/markers`로 선택한다.
4. 지도 위의 반투명 빨간 box가 저장된 confirmed 정적 장애물인지 확인한다.

다음 주행 전에 저장소를 초기화한다.

```bash
ros2 service call /static_obstacle_map/reset std_srvs/srv/Empty "{}"
```

## 7. clean map 연결

`obstacle_detector`를 `detector_map_yaml`과 함께 실행하면 detector의 Layer 1 입력은
`/obstacle_detector/map`이다. 이 경우 본 노드의 `base_map_topic`도 같은 clean map으로 설정한다.

```bash
ros2 launch static_obstacle_map static_obstacle_map.launch.py \
  params_file:=/absolute/path/to/static_obstacle_map.yaml
```

```yaml
static_obstacle_map:
  ros__parameters:
    base_map_topic: /obstacle_detector/map
```

합성 출력 `/adaptive_obstacle_map`을 `/map`이나 `/obstacle_detector/map`으로 remap하면 안 된다.
합성 장애물이 detector의 Layer 1 필터로 되먹임되어 검출에서 사라질 수 있다.
