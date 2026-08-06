# global_trajectory_publisher_node

## 1. 노드 목적

오프라인 최적화 결과인 `global_waypoints.json`을 읽어 전역 경로 웨이포인트와
RViz 시각화 마커를 ROS 2 토픽으로 재발행(republish)한다.

## 2. 동작 원리

1. 파라미터로 지정된 디렉토리에서 `global_waypoints.json`을 읽는다.
   - 경로 우선순위: `map_path`(명시적 override) → `<output_base_dir>/<map_name>`.
2. 읽은 웨이포인트 번들을 latched(`transient_local`) QoS로 발행한다.
3. **마커 생성**: 오프라인 생성기는 마커 배열을 빈 값(`{"markers": []}`)으로 저장한다.
   따라서 이 노드가 웨이포인트로부터 직접 마커를 만든다 (`generateMarkers()`).
   - `/global_waypoints/markers`: 전역 궤적을 속도 색상 `LINE_STRIP`으로 표시
     (초록=저속, 빨강=고속). `vx_mps`의 min/max로 정규화해 per-point 색상을 넣는다.
   - `/trackbounds/markers`: 각 웨이포인트에서 경로 법선(`psi_rad` ± 90°) 방향으로
     `d_left`/`d_right`만큼 떨어진 좌/우 경계점을 잇는 `LINE_STRIP` 2개.
   - JSON에 마커가 이미 채워져 있으면 생성하지 않고 그대로 사용한다.
4. `/global_planning/reload_waypoints` 요청이 오면
   `<output_base_dir>/<reload_map_name>/global_waypoints.json`을 별도로 읽고 검증한다.
   검증에 성공한 경우에만 메모리 번들과 활성 참조 경로를 원자적으로 교체한다.
   기존 `<output_base_dir>/<map_name>` 파일과 YAML은 수정하지 않는다.
5. 타이머(`publish_period_sec`)마다 현재 활성 번들 전체를 반복 발행한다.

## 3. 구독 토픽

없음. 이 노드는 파일에서 데이터를 읽어 발행만 한다.

## 4. 발행 토픽

| 토픽 | 타입 | 조건 |
| --- | --- | --- |
| `/global_waypoints` | `f110_msgs/msg/WpntArray` | 항상 (latched) |
| `/map_infos` | `std_msgs/msg/String` | 항상 |
| `/estimated_lap_time` | `std_msgs/msg/Float32` | 항상 |
| `/global_waypoints/markers` | `visualization_msgs/msg/MarkerArray` | `publish_markers=true` |
| `/trackbounds/markers` | `visualization_msgs/msg/MarkerArray` | `publish_markers=true` |
| `/global_waypoints/shortest_path` | `f110_msgs/msg/WpntArray` | `publish_shortest_path=true` |
| `/centerline_waypoints` | `f110_msgs/msg/WpntArray` | `publish_centerline=true` |
| `/centerline_waypoints/markers` | `visualization_msgs/msg/MarkerArray` | `publish_centerline=true` & `publish_markers=true` |
| `/lattice_viz` | `visualization_msgs/msg/MarkerArray` | `publish_lattice=true` |

### 서비스

| 서비스 | 타입 | 설명 |
| --- | --- | --- |
| `/global_planning/reload_waypoints` | `std_srvs/srv/Trigger` | 검증된 `reload_map_name` 결과로 런타임 참조 전환 |

## 5. 주요 파라미터

파라미터 파일: `src/global_planning/config/global_planning.yaml`

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `output_base_dir` | `offline_trajectory_generator/output` | JSON 상위 디렉토리 (상대경로는 실행 작업 디렉토리 기준) |
| `map_name` | `map` | 시작 시 `<output_base_dir>/<map_name>/global_waypoints.json`을 읽음 |
| `map_path` | `""` | 시작 경로의 명시적 디렉토리 override |
| `reload_map_name` | `obstacle_map` | reload 성공 시 전환할 별도 JSON 디렉토리 이름 |
| `publish_markers` | `true` | RViz 마커 발행 여부 |
| `publish_shortest_path` | `true` | 최단경로 웨이포인트 발행 여부 |
| `publish_centerline` | `true` | 센터라인 발행 여부 |
| `publish_lattice` | `false` | lattice 시각화 발행 여부 |
| `publish_period_sec` | `2.0` | 발행 주기(초) |
| `marker_frame_id` | `map` | 생성 마커의 `header.frame_id` |
| `traj_marker_width` | `0.10` | 전역 궤적 라인 두께(m) |
| `trackbound_marker_width` | `0.05` | 트랙 경계 라인 두께(m) |

## 6. 실행 방법

1. ROS 2 Jazzy 환경을 source한다.
2. 패키지를 빌드한다.
3. 워크스페이스 루트(`~/2026_IFAC`)에서 launch를 실행한다 (상대경로 JSON 해석 기준).

```bash
source /opt/ros/jazzy/setup.zsh
colcon build --packages-up-to map_creator
source install/setup.zsh
ros2 launch global_planning global_planning.launch.py
```

이 launch는 `map_creator_node`도 함께 실행한다. map creator 파라미터 파일을 바꿔야 하면
`map_creator_params_file:=<경로>`를 추가한다.

> 참고: launch 파일이 노드명을 `global_trajectory_publisher_node`로 지정하므로
> 파라미터 파일의 노드 키와 일치한다. `ros2 run`으로 직접 띄울 때는
> `-r __node:=global_trajectory_publisher_node`로 노드명을 맞춰야 파라미터가 실린다.

## 7. 확인 절차

1. 노드 로그에 `loaded global waypoints from <경로>`가 출력되는지 확인한다.
2. `/global_waypoints`에 웨이포인트가 실려 있는지 확인한다.
3. RViz에서 `/global_waypoints/markers`를 Add하면 속도 색상 궤적선이 보인다.
4. RViz에서 `/trackbounds/markers`를 Add하면 좌/우 트랙 경계선이 보인다.
   - Fixed Frame은 `marker_frame_id`(기본 `map`)와 일치시켜야 한다.
5. map creator가 `swapped`를 보고한 뒤 `ros2 param get
   /global_trajectory_publisher_node map_name`이 `obstacle_map`인지 확인한다.
