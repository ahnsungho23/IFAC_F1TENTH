# Planning/Control Pipeline Architecture

이 문서는 `/home/haejun/2026_IFAC/planning` 코드를 기준으로 현재 실제 동작을 정리한 아키텍처 문서입니다.
문서 내용은 `planning/global_planner` 소스, launch, config 기준으로 작성했습니다.

## 0) Scope

1. 이 문서는 `planning` 패키지 내부 런타임 파이프라인을 중심으로 설명합니다.
2. `wpnt_publisher`는 planning 바깥 패키지이지만, 인터페이스 연동에 필요한 최소 내용만 포함합니다.
3. 과거 문서에 있던 `offline_trajectory_generator` GUI 파라미터 설명은 별도 도구 영역이므로 이 문서에서 분리했습니다.

## 1) Planning 디렉터리 구성

| 경로 | 역할 |
|---|---|
| `planning/global_planner/src/global_planner_node.cpp` | `/map`, `/pf/pose/odom` 수신 후 외부 optimizer 커맨드 실행(브리지) |
| `planning/global_planner/src/global_trajectory_publisher_node.cpp` | `global_waypoints.json` 로드 후 `/global_waypoints` 등 토픽 재발행 |
| `planning/global_planner/src/frenet_odom_node.cpp` | 최근접 global waypoint 인덱스를 계산해 `/car_state/frenet/odom` 발행 |
| `planning/global_planner/src/readwrite_global_waypoints.cpp` | JSON <-> ROS 메시지 직렬화/역직렬화 유틸 |
| `planning/global_planner/launch/global_planning.launch.py` | 위 3개 노드를 함께 기동 |
| `planning/global_planner/config/global_planning.yaml` | 노드 파라미터 파일 |
| `planning/global_planner/data/global_waypoints.json` | 재발행 대상 샘플/결과 파일 |

## 2) End-to-End 런타임 플로우

```text
[Localization]
monte_carlo_localization
  ↓
/pf/pose/odom (nav_msgs/msg/Odometry)

[Planning Bridge]
global_planner_node (C++)
  입력:
    /map
    /pf/pose/odom
  처리:
    두 토픽 수신 확인
    optimizer_command 실행 (외부 Python)
    map_dir/global_waypoints.json 생성 확인
  출력:
    (토픽 publish 없음)
    global_waypoints.json 파일 생성/갱신

[Global Waypoint Republish]
global_trajectory_publisher_node (C++)
  입력:
    global_waypoints.json (map_path)
  출력:
    /global_waypoints
    /map_infos
    /estimated_lap_time
    (+ 옵션: markers, shortest_path, centerline 토픽)

[Frenet Index]
frenet_odom_node (C++)
  입력:
    /pf/pose/odom
    /global_waypoints
  처리:
    현재 위치와 가장 가까운 waypoint index 탐색
  출력:
    /car_state/frenet/odom

[Downstream]
wpnt_publisher
  입력:
    /global_waypoints
    /car_state/frenet/odom
  출력:
    /local_waypoints
```

## 3) Node별 실제 역할 (코드 기준)

### 3.1 `global_planner_node` (C++)

1. `/map`, `/pf/pose/odom`를 구독합니다.
2. 두 입력이 준비되면 `optimizer_command --map-dir <map_dir>`를 `system()`으로 실행합니다.
3. 실행 후 `<map_dir>/global_waypoints.json`을 읽어 파싱 성공 여부를 확인합니다.
4. 이 노드는 `/global_waypoints`를 직접 publish하지 않습니다.

핵심 파라미터:
- `map_dir`
- `optimizer_command`
- `pythonpath_extra`
- `run_once`
- `trigger_on_start`

### 3.2 `global_trajectory_publisher_node` (C++)

1. 시작 시 `map_path`에서 `global_waypoints.json`을 1회 읽습니다.
2. 로드 성공 시 timer 주기로 아래 토픽을 재발행합니다.
3. 기본 글로벌 경로는 `global_traj_wpnts_iqp`를 `/global_waypoints`로 발행합니다.

기본 발행 토픽:
- `/global_waypoints` (`f110_msgs/msg/WpntArray`, transient_local+reliable)
- `/map_infos` (`std_msgs/msg/String`)
- `/estimated_lap_time` (`std_msgs/msg/Float32`)

옵션 발행 토픽:
- `publish_markers=true`: `/global_waypoints/markers`, `/trackbounds/markers`
- `publish_shortest_path=true`: `/global_waypoints/shortest_path`, `/global_waypoints/shortest_path/markers`
- `publish_centerline=true`: `/centerline_waypoints`, `/centerline_waypoints/markers`
- `publish_lattice=true`: `/lattice_viz` 퍼블리셔 생성 (현재 소스에서 실데이터 publish 로직은 없음)

### 3.3 `frenet_odom_node` (C++)

1. `/global_waypoints`를 latched QoS로 구독하고 waypoint 배열을 저장합니다.
2. `/pf/pose/odom` 수신 시 차량 XY를 모든 인접 waypoint segment에 직교투영합니다.
3. 투영점과 차량 위치 사이의 거리 제곱이 가장 작은 segment를 closest segment로 선택합니다.
4. `s = waypoint[i].s_m + t * segment_length`로 계산하고, `closed_loop=true`이면 마지막 waypoint와 첫 번째 waypoint segment도 포함합니다.
5. `d`는 segment 방향과 투영점 기준 차량 offset의 cross product 부호로 계산합니다.
6. 출력 Odometry를 다음처럼 구성합니다.
7. `header.frame_id = "frenet"`
8. `child_frame_id = "<closest_segment_index>"`
9. `pose.pose.position.x = calculated_s`, `pose.pose.position.y = calculated_d`
10. `/car_state/frenet/odom`으로 publish합니다.

## 4) `global_planner_node`가 호출하는 vendor optimizer 내부 단계

`optimizer_command` 기본값은 `vendor/gb_optimizer/src/global_planner_node_ros2.py`입니다.

실행 흐름:
1. 임시 rclpy 노드로 `/map`, `/pf/pose/odom`를 다시 수집합니다.
2. occupancy grid를 이진 free-space로 변환하고 morphology + skeletonize를 수행합니다.
3. centerline contour를 추출/스무딩합니다.
4. `trajectory_optimizer`를 2회 실행합니다.
5. 1차: `mincurv_iqp`
6. 2차: `shortest_path`
7. 결과를 `global_waypoints.json`으로 저장합니다.
8. 저장 키는 `map_info_str`, `est_lap_time`, `centerline_waypoints`, `global_traj_wpnts_iqp`, `global_traj_wpnts_sp` 등입니다.

## 5) Topic 인터페이스 요약

| 노드 | Subscribe | Publish |
|---|---|---|
| `global_planner_node` | `/map`, `/pf/pose/odom` | 없음 (파일 생성만) |
| `global_trajectory_publisher_node` | 없음 | `/global_waypoints`, `/map_infos`, `/estimated_lap_time`, 옵션 토픽 |
| `frenet_odom_node` | `/pf/pose/odom`, `/global_waypoints` | `/car_state/frenet/odom` |

Downstream(참고):

| 노드 | Subscribe | Publish |
|---|---|---|
| `wpnt_publisher` | `/global_waypoints`, `/car_state/frenet/odom`, `/planner/avoidance/otwpnts` | `/local_waypoints`, `/local_waypoints/path` |

## 6) Launch/Parameter 사용법

### 6.1 기본 launch

```bash
ros2 launch global_planner global_planning.launch.py \
  params_file:=/home/haejun/2026_IFAC/planning/global_planner/config/global_planning.yaml
```

### 6.2 주의할 점

1. `launch`의 기본 `params_file` 경로가 `/home/haejum-park/...`로 하드코딩되어 있습니다.
2. 현재 환경(`/home/haejun/...`)에서는 `params_file:=...`를 명시해서 실행하는 것이 안전합니다.
3. `global_planning.yaml` 내부 `map_dir`, `map_path`, `optimizer_command`, `pythonpath_extra`도 절대경로가 `/home/haejum-park/...`이므로 환경에 맞게 수정해야 합니다.

## 7) 운용 시 주의사항 (중요)

1. `global_trajectory_publisher_node`는 JSON을 시작 시 1회만 읽습니다.
2. 시작 시 파일이 없거나 파싱 실패하면 `has_bundle_`가 false로 남고 publish가 진행되지 않습니다.
3. 따라서 안정 운용은 다음 순서를 권장합니다.
4. 먼저 `global_planner_node`로 JSON 생성 확인
5. 그다음 `global_trajectory_publisher_node` 시작(또는 재시작)
6. `global_planner_node`와 `global_trajectory_publisher_node`를 동시에 띄우는 경우, stale JSON 또는 미로딩 상태가 발생할 수 있습니다.

## 8) 현재 문서와 코드 일치 체크 포인트

1. `/global_waypoints` 직접 발행 주체는 `global_trajectory_publisher_node`입니다.
2. `global_planner_node`는 브리지/파일 생성 검증 역할입니다.
3. `frenet_odom_node`는 C++ 노드이며 `child_frame_id`에 최근접 인덱스를 문자열로 넣습니다.
4. 구현 경로는 `planning/src/...`가 아니라 `planning/global_planner/src/...`입니다.
