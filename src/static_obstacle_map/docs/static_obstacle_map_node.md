# static_obstacle_map 노드

## 1. 목적

`static_obstacle_map_node`는 `obstacle_detector`가 확정한 정적 장애물을 주행 중 map 좌표로
기억하고 RViz MarkerArray로 표시한다. 정적 장애물이 일시적으로 LiDAR에서 보이지 않아도 저장
정보는 reset 전까지 유지된다.

이 노드는 OccupancyGrid 지도를 구독·합성·발행하지 않으며 경로 생성이나 장애물 분류도 하지
않는다.

## 2. 동작 원리

1. `/confirmed_static_obs`에서 confirmed 정적 장애물을 받는다.
2. `is_static=true`, `is_visible=true`, `has_cartesian=true`이고 Cartesian AABB가 유효한 객체만
   저장한다.
3. 같은 객체는 AABB 간격과 detector source ID를 이용해 기존 저장 항목에 연결한다.
4. 최초 AABB를 보존하고 이후 관측에서 기존 경계 밖으로 나온 부분만 bounded union으로
   추가한다.
5. 새 경계가 `edge_match_tolerance_m` 이내에서 `edge_confirm_frames`회 연속 관측된 경우에만
   저장 범위를 확장한다.
6. 최초 AABB와 누적 union은 `max_obstacle_diagonal_m` 상한을 지킨다.
7. `/opp_obs`에서 같은 track이 동적으로 재분류되면 설정에 따라 저장 항목을 제거한다.
8. 저장 내용이 바뀌면 CUBE marker를 발행하며, 배열 맨 앞에 `DELETEALL`을 넣어 이전 marker가
   RViz에 남지 않게 한다.

저장소는 메모리에 있으므로 노드가 종료되면 사라진다. 같은 프로세스로 다음 주행을 시작할 때는
reset 서비스를 호출한다.

## 3. 토픽과 서비스

| 방향 | 기본 이름 | 타입 | 설명 |
|---|---|---|---|
| 구독 | `/confirmed_static_obs` | `f110_msgs/msg/ObstacleArray` | confirmed 정적 장애물 |
| 구독 | `/opp_obs` | `f110_msgs/msg/ObstacleArray` | 동적 재분류 정정 |
| 발행 | `/static_obstacle_map/markers` | `visualization_msgs/msg/MarkerArray` | 저장 정적 장애물 AABB |
| 서비스 | `/static_obstacle_map/reset` | `std_srvs/srv/Empty` | 저장 장애물 전체 삭제 |

MarkerArray 발행은 Reliable + Transient Local QoS를 사용하므로 늦게 실행된 RViz도 최신 상태를
받는다.

## 4. 주요 파라미터

파라미터 파일은 `config/static_obstacle_map.yaml`이다.

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `confirmed_static_obs_topic` | `/confirmed_static_obs` | confirmed 정적 입력 |
| `dynamic_obs_topic` | `/opp_obs` | 동적 재분류 입력 |
| `publish_visualization` | `true` | MarkerArray 발행 여부 |
| `visualization_topic` | `/static_obstacle_map/markers` | MarkerArray 출력 |
| `frame_id` | `map` | marker 좌표 프레임 |
| `visualization_marker_namespace` | `persistent_static_obstacles` | marker namespace |
| `visualization_height_m` | `0.15` | box 높이 |
| `visualization_color_r/g/b/a` | `1.0/0.1/0.1/0.85` | box RGBA 색상 |
| `association_distance_m` | `0.30` | 같은 객체로 연결할 AABB 간 최대 간격 |
| `edge_confirm_frames` | `3` | 새 외곽 경계를 누적할 연속 관측 횟수 |
| `edge_match_tolerance_m` | `0.05` | 같은 외곽 경계로 인정할 최대 차이 |
| `max_obstacle_diagonal_m` | `0.80` | 저장 AABB 대각선의 절대 상한 |
| `remove_reclassified_dynamic` | `true` | 같은 track의 동적 승격 시 저장 삭제 |
| `reset_service` | `/static_obstacle_map/reset` | 저장 초기화 서비스 이름 |

## 5. 빌드

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-up-to obstacle_detector static_obstacle_map
source install/setup.zsh
```

## 6. 실행

먼저 detector를 실행한다.

```bash
ros2 launch obstacle_detector obstacle_detector.launch.py
```

다른 터미널에서 메모리 노드를 실행한다.

```bash
ros2 launch static_obstacle_map static_obstacle_map.launch.py
```

출력을 확인한다.

```bash
ros2 topic echo /static_obstacle_map/markers --once
```

RViz2에서는 다음 순서로 확인한다.

1. `Global Options > Fixed Frame`을 `map`으로 설정한다.
2. `Add > MarkerArray`에서 Topic을 `/static_obstacle_map/markers`로 선택한다.
3. 지도 위의 반투명 빨간 box가 저장된 confirmed 정적 장애물인지 확인한다.

다음 주행 전에 저장소를 초기화한다.

```bash
ros2 service call /static_obstacle_map/reset std_srvs/srv/Empty "{}"
```

다른 설정 파일을 쓰려면 launch 인자로 절대경로를 전달한다.

```bash
ros2 launch static_obstacle_map static_obstacle_map.launch.py \
  params_file:=/absolute/path/to/static_obstacle_map.yaml
```
