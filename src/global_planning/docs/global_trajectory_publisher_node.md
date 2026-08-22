# global_trajectory_publisher_node

## 1. 노드 목적

오프라인 최적화 결과인 `global_waypoints.json`을 읽어 전역 경로 웨이포인트와
RViz 시각화 마커를 ROS 2 토픽으로 재발행(republish)한다.

## 2. 동작 원리

1. 파라미터로 지정된 디렉토리에서 `global_waypoints.json`을 읽는다.
   - 경로 우선순위: `map_path`(명시적 override) → `<output_base_dir>/<map_name>`.
2. 읽은 웨이포인트 번들을 latched(`transient_local`) QoS로 발행한다.
3. **마커는 만들지 않고 참조만 한다.** 이 노드는 기하 연산을 전혀 하지 않는다.
   마커는 오프라인 생성기(`generate_global_trajectory`)가 `global_waypoints.json`에
   미리 구워 넣은 것을 그대로 발행한다.
   - `/global_waypoints/markers`: 전역 궤적을 속도 색상 `LINE_STRIP`으로 표시
     (초록=저속, 빨강=고속). `vx_mps`의 min/max로 정규화한 per-point 색상.
   - `/trackbounds/markers`: 각 웨이포인트에서 경로 법선(`psi_rad` ± 90°) 방향으로
     `d_left`/`d_right`만큼 떨어진 좌/우 경계점을 잇는 `LINE_STRIP` 2개.
   - 마커 스타일(프레임 `map`, 궤적 두께 0.10 m, 경계 두께 0.05 m, 색상)은
     생성기 쪽 상수로 고정되어 있다. 바꾸려면 생성기를 수정하고 재생성해야 한다.
4. **마커가 비어 있으면 `DELETEALL`을 발행한다.**
   in-race 재생성 경로(`regenerate_obstacle_map`, map_creator)는 시각화를 만들지 않고
   빈 배열을 쓴다. RViz의 `MarkerArray`는 ns+id 단위로 persistent라서, 빈 배열을
   그냥 발행하면 **직전 맵의 라인이 화면에 그대로 남는다.** 이를 막기 위해 마커 배열이
   비어 있으면 `action=DELETEALL` 마커 1개를 대신 발행한다.
   즉 랩2 reload 이후에는 궤적선·경계선이 **화면에서 사라지는 것이 정상 동작**이다.
5. `/global_planning/reload_waypoints` 요청이 오면
   `<output_base_dir>/<reload_map_name>/global_waypoints.json`을 별도로 읽고 검증한다.
   검증에 성공한 경우에만 메모리 번들과 활성 참조 경로를 원자적으로 교체한다.
   기존 `<output_base_dir>/<map_name>` 파일과 YAML은 수정하지 않는다.
6. **`/lap_count`가 `lap_switch_count`에 도달하면 사전 생성 번들로 1회 전환한다.**
   `lap_switch_map_name`이 비어 있지 않을 때만 동작하며, 기본값은 비활성이다.
   reload와 동일한 읽기·검증 절차(`swapTo()`)를 거치므로 잘못된 번들은 거부되고
   기존 라인이 유지된다.
7. 타이머(`publish_period_sec`)마다 현재 활성 번들 전체를 반복 발행한다.

### 2.1 주행 중 참조 경로 전환 순서

```text
기동          output/map                 (map_name)
  ↓ 랩 3      output/obstacle_map        map_creator가 reload 서비스 호출
  ↓ 랩 11     output/forza_map           /lap_count 구독 (이 노드)
```

- **랩 11 전환은 단방향·1회성이다.** `/lap_count`는 latched(`transient_local`)라
  같은 값이 반복 전달되는데, 내부 플래그로 파일을 다시 읽지 않는다.
- 비교는 `>=`다. 메시지를 한 번 놓쳐도 전환이 건너뛰어지지 않는다.
- 전환에 실패하면(디렉토리 없음·검증 실패) **플래그를 세우지 않아 다음 랩에 재시도**한다.
  그동안 활성 라인은 그대로다.
- **map_creator가 이를 되돌리지 않는다.** 스왑 후 map_creator의 단계는 종착 상태이며
  재스왑 경로가 제거되어 있다(`src/map_creator/AGENTS.md`).
- ⚠️ **`forza_map` 번들은 주행 전에 만들어져 있어야 한다.** 없으면 랩 11에 WARN만 남고
  `obstacle_map` 라인으로 계속 주행한다.
- ⚠️ 라인이 바뀌면 `frenet_odom_node`가 CLCS를 재구성하고, `lap_counter_node`가 세는
  기준 `s`도 함께 바뀐다. 새 라인의 `s_max`가 `finish_s_min`(기본 10.0)보다 작으면
  **이후 랩이 세어지지 않는다.** 다른 맵/라인을 쓸 때 반드시 확인할 것.

## 3. 구독 토픽

| 토픽 | 타입 | 조건 |
| --- | --- | --- |
| `lap_count_topic` (기본 `/lap_count`) | `std_msgs/msg/Int32` | `lap_switch_map_name`이 비어 있지 않을 때만 구독 |

> QoS는 `KeepLast(1)` + `reliable` + **`transient_local`** 로,
> `lap_counter_node`의 발행 QoS와 일치시킨다. volatile로 구독하면 이미 발행된
> latched 값을 못 받아 다음 랩까지 전환이 밀린다.

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
| `lap_switch_map_name` | `""` (코드 기본) / `forza_map` (YAML) | 랩 전환 대상 디렉토리 이름. **비우면 랩 전환 기능 자체가 비활성** |
| `lap_switch_count` | `11` | 이 값 **이상**의 `/lap_count`에서 전환 |
| `lap_count_topic` | `/lap_count` | 랩 카운트 구독 토픽 |
| `publish_markers` | `true` | RViz 마커 발행 여부 |
| `publish_shortest_path` | `true` | 최단경로 웨이포인트 발행 여부 |
| `publish_centerline` | `true` | 센터라인 발행 여부 |
| `publish_lattice` | `false` | lattice 시각화 발행 여부 |
| `publish_period_sec` | `2.0` | 발행 주기(초) |

> 마커 스타일 파라미터(`marker_frame_id` / `traj_marker_width` /
> `trackbound_marker_width`)는 제거되었다. 노드가 더 이상 마커를 만들지 않으므로
> 스타일은 오프라인 생성기 쪽 상수([`trajectory_core.cpp`](../../../offline_trajectory_generator/src/trajectory_core.cpp)의
> `kTrajMarkerWidth` / `kTrackboundMarkerWidth`)가 유일한 출처다.

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

이 launch는 `map_creator_node`와 그 입력을 제공하는 `static_obstacle_map`도 함께 실행한다.
map creator 파라미터 파일을 바꿔야 하면
`map_creator_params_file:=<경로>`를 추가한다.

> 참고: launch 파일이 노드명을 `global_trajectory_publisher_node`로 지정하므로
> 파라미터 파일의 노드 키와 일치한다. `ros2 run`으로 직접 띄울 때는
> `-r __node:=global_trajectory_publisher_node`로 노드명을 맞춰야 파라미터가 실린다.

## 7. 확인 절차

1. 노드 로그에 `loaded global waypoints from <경로>`가 출력되는지 확인한다.
2. `/global_waypoints`에 웨이포인트가 실려 있는지 확인한다.
3. RViz에서 `/global_waypoints/markers`를 Add하면 속도 색상 궤적선이 보인다.
4. RViz에서 `/trackbounds/markers`를 Add하면 좌/우 트랙 경계선이 보인다.
   - Fixed Frame은 마커의 `header.frame_id`(생성기 고정값 `map`)와 일치시켜야 한다.
5. map creator가 `swapped`를 보고한 뒤 `ros2 param get
   /global_trajectory_publisher_node map_name`이 `obstacle_map`인지 확인한다.
6. 랩 11 전환은 노드 로그로 확인한다.

```bash
# 기동 시 (기능이 켜져 있으면)
#   lap switch armed: at lap 11 the source becomes offline_trajectory_generator/output/forza_map
# 전환 시
#   lap 11 switch: loaded 349 waypoints from offline_trajectory_generator/output/forza_map
# 실패 시
#   lap 11 switch to ... rejected: <사유>
```

   수동 시험은 발행 QoS를 맞춰야 한다(§3 주석).

```bash
ros2 topic pub -1 -w 0 --qos-durability transient_local --qos-reliability reliable \
  /lap_count std_msgs/msg/Int32 "{data: 11}"
```

   전환 후 마커가 다시 보인다 — Forza 번들은 마커를 갖고 있어
   `obstacle_map`에서 `DELETEALL`로 비었던 화면이 복구된다.
7. reload 이후 RViz에서 궤적선·경계선이 **사라지는지** 확인한다.
   이는 의도된 동작이다(§2-4). 옛 라인이 남아 있다면 `DELETEALL`이 발행되지 않은
   것이므로 버그다. 다음으로 확인할 수 있다:

```bash
ros2 topic echo /global_waypoints/markers --once --field markers[0].action
# reload 전: 0 (ADD) / reload 후: 3 (DELETEALL)
```
