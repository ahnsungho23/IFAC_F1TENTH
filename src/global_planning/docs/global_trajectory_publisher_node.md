# global_trajectory_publisher_node

## 1. 노드 목적

오프라인 생성 결과인 `global_waypoints.json`을 시작할 때 한 번 읽고, 글로벌 경로와
RViz 마커를 ROS 2 토픽으로 반복 발행한다. 주행 중 참조 경로를 변경하지 않는다.

## 2. 동작 원리

1. 시작 시 JSON 디렉터리를 다음 우선순위로 결정한다.
   - `map_path`가 비어 있지 않으면 `<map_path>/global_waypoints.json`
   - 아니면 `<output_base_dir>/<map_name>/global_waypoints.json`
2. 파일을 열고 JSON 번들로 역직렬화한다. 파일이 없거나 비었거나 파싱에 실패하면 번들을
   발행하지 않는다.
3. 읽은 번들을 메모리에 유지하고 `publish_period_sec`마다 다시 발행한다.
4. `/global_waypoints`는 reliable + transient-local QoS이므로 늦게 시작한 노드도 최신
   글로벌 경로를 받는다.
5. 이 노드는 경로나 마커를 계산하지 않는다. 마커는
   `offline_trajectory_generator/bin/generate_global_trajectory`가 JSON에 저장한 값을
   그대로 발행한다.
6. 마커 배열이 비어 있으면 `DELETEALL` 마커 하나를 발행해 RViz에 이전 마커가 남지 않게
   한다.

다른 글로벌 라인을 사용하려면 노드를 재시작하면서 `map_name`, `F1_MAP`, 또는
`map_path`를 바꾼다. reload 서비스와 랩 기반 자동 전환 기능은 제공하지 않는다.

## 3. 구독 토픽과 서비스

구독 토픽과 서비스는 없다. JSON 파일만 시작 시 읽는다.

## 4. 발행 토픽

| 토픽 | 타입 | 조건 |
| --- | --- | --- |
| `/global_waypoints` | `f110_msgs/msg/WpntArray` | 항상, transient-local |
| `/map_infos` | `std_msgs/msg/String` | 항상 |
| `/estimated_lap_time` | `std_msgs/msg/Float32` | 항상 |
| `/global_waypoints/markers` | `visualization_msgs/msg/MarkerArray` | `publish_markers=true` |
| `/trackbounds/markers` | `visualization_msgs/msg/MarkerArray` | `publish_markers=true` |
| `/global_waypoints/shortest_path` | `f110_msgs/msg/WpntArray` | `publish_shortest_path=true` |
| `/centerline_waypoints` | `f110_msgs/msg/WpntArray` | `publish_centerline=true` |
| `/centerline_waypoints/markers` | `visualization_msgs/msg/MarkerArray` | centerline과 marker 모두 활성 |
| `/lattice_viz` | `visualization_msgs/msg/MarkerArray` | `publish_lattice=true` |

## 5. 주요 파라미터

파라미터 파일은 `src/global_planning/config/global_planning.yaml`이다.

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `output_base_dir` | `offline_trajectory_generator/output` | JSON 번들 상위 디렉터리 |
| `map_name` | `map` | `<output_base_dir>/<map_name>` 번들 선택 |
| `map_path` | `""` | 설정하면 위 두 값보다 우선하는 디렉터리 |
| `publish_markers` | `true` | RViz 마커 발행 |
| `publish_shortest_path` | `true` | shortest-path 웨이포인트 발행 |
| `publish_centerline` | `true` | 센터라인 발행 |
| `publish_lattice` | `false` | lattice 시각화 발행 |
| `publish_period_sec` | `2.0` | 번들 반복 발행 주기 |

상대경로는 노드의 작업 디렉터리를 기준으로 해석하므로 워크스페이스 루트에서 실행한다.
`global_planning.launch.py`는 YAML의 `map_name`을 launch 인자로 덮어쓰며, 인자를
생략하면 `F1_MAP` 환경변수 또는 `map`을 사용한다.

## 6. 실행 방법

1. ROS 2 Jazzy와 워크스페이스를 source한다.
2. `global_planning`을 빌드한다.
3. 사용할 JSON 번들이 존재하는지 확인한다.
4. 워크스페이스 루트에서 launch한다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select global_planning
source install/setup.zsh
ros2 launch global_planning global_planning.launch.py map_name:=map
```

Forza처럼 별도로 생성한 번들을 처음부터 사용하려면 다음처럼 시작한다.

```bash
F1_MAP=forza_map ros2 launch global_planning global_planning.launch.py
```

이 경우 `offline_trajectory_generator/output/forza_map/global_waypoints.json`이 시작 전에
존재해야 하며 첫 랩부터 해당 라인을 사용한다.

## 7. 확인 절차

1. 로그의 `loaded global waypoints from <경로>`를 확인한다.
2. 글로벌 경로가 한 번 이상 발행되는지 확인한다.
3. RViz 마커를 확인한다.
4. 노드 실행 중 JSON 파일이나 `map_name`을 바꿔도 활성 번들이 자동 전환되지 않는지
   확인한다. 새 파일을 적용하려면 노드를 재시작한다.

```bash
ros2 topic echo /global_waypoints --once
ros2 topic echo /global_waypoints/markers --once
ros2 param get /global_trajectory_publisher_node map_name
```
