# frenet_odom_node

## 1. 노드 목적

`frenet_odom_node`는 map frame 기준 차량 위치 `(x, y)`를 global path 기준 Frenet 좌표 `(s, d)`로 변환한다.
입력 odometry는 `/pf/pose/odom`에서 받고, 변환 결과는 `/car_state/frenet/odom`으로 publish한다.

## 2. 동작 원리

1. `/global_waypoints`에서 `f110_msgs/msg/WpntArray`를 latched QoS로 구독한다.
2. waypoint의 `x_m`, `y_m`, `s_m`만 Frenet 계산에 사용한다.
3. waypoint를 CommonRoad-CLCS C++ `CurvilinearCoordinateSystem` 기준 경로로 빌드한다.
4. 차량 odometry가 들어오면 CLCS projection으로 Frenet `s`, `d`와 segment index를 계산한다.
5. `closed_loop=true`이면 마지막 waypoint와 첫 번째 waypoint를 연결한 기준 경로를 만들고, 출력 `s`는 track length 기준으로 정규화한다.
6. yaw 변환에는 `tf2`를 사용하지 않고, 로컬 수학 함수로 heading error를 계산한다.

`d_m`은 waypoint 자체의 lateral offset일 수 있으므로 차량의 `d` 계산에 사용하지 않는다.

## 3. 구독 토픽

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/pf/pose/odom` | `nav_msgs/msg/Odometry` | 차량의 map frame 기준 위치. `pose.pose.position.x`, `pose.pose.position.y`를 사용한다. |
| `/global_waypoints` | `f110_msgs/msg/WpntArray` | global path. 각 waypoint의 `x_m`, `y_m`, `s_m`을 사용한다. |

## 4. 발행 토픽

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | Frenet 좌표 결과. `pose.pose.position.x=s`, `pose.pose.position.y=d`, `pose.pose.position.z=0.0`이다. |

출력 odometry의 `header.frame_id` 기본값은 `frenet`이고, `child_frame_id`에는 closest segment index 문자열을 넣는다.
`pose.pose.position.x=s`, `pose.pose.position.y=d`이고, 설정에 따라 twist의 `linear.x`, `linear.y`에는 Frenet velocity `v_s`, `v_d`가 들어간다.

## 5. 주요 파라미터

파라미터 파일: `src/global_planning/config/global_planning.yaml`

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `odom_topic` | `/pf/pose/odom` | 입력 odometry 토픽 |
| `waypoint_topic` | `/global_waypoints` | global waypoint 토픽 |
| `frenet_odom_topic` | `/car_state/frenet/odom` | 출력 Frenet odometry 토픽 |
| `closed_loop` | `true` | 마지막 waypoint와 첫 번째 waypoint를 연결할지 여부 |
| `frenet_frame_id` | `frenet` | 출력 odometry frame id |
| `projection_failure_policy` | `drop_message` | projection 실패 시 처리 방식. `drop_message`, `publish_last_valid`, `publish_nan` |
| `velocity_frame` | `body` | 입력 odometry twist를 body frame 또는 map frame으로 해석 |

## 6. 실행 방법

1. ROS 2 Humble 환경을 source한다.
2. 패키지를 빌드한다.
3. launch 파일을 실행한다.

```bash
source /opt/ros/humble/setup.zsh
colcon build --packages-select global_planning
source install/setup.zsh
ros2 launch global_planning global_planning.launch.py
```

개별 실행이 필요하면 설치 후 다음처럼 실행한다.

```bash
ros2 run global_planning frenet_odom_node --ros-args --params-file src/global_planning/config/global_planning.yaml
```

## 7. 확인 절차

1. `/global_waypoints`가 2개 이상의 waypoint를 publish하는지 확인한다.
2. `/pf/pose/odom`이 차량 위치를 publish하는지 확인한다.
3. `/car_state/frenet/odom`의 `pose.pose.position.x`, `pose.pose.position.y`가 각각 계산된 `s`, `d`인지 확인한다.
4. `/car_state/frenet/odom.child_frame_id`가 closest segment index 문자열인지 확인한다.

## 8. 단조 s-윈도우 추적

`continuity_enabled=true`이면 에고 차량의 연속적인 이동을 이용해 헤어핀처럼 공간상 가까운 다른 경로 구간으로
Frenet 투영이 갑자기 넘어가는 현상을 방지한다. 이 기능은 에고 odometry에만 적용되며, 장애물처럼 서로 연속되지
않은 점을 투영할 때 사용하는 무상태 `convert()`에는 적용되지 않는다.

### 8.1 동작 순서

1. 첫 odometry는 `initial_seed_window`가 `0` 이하이면 전체 경로에서 투영 위치를 찾는다.
2. 다음 odometry부터는 이전 `s`를 기준으로
   `[s_prev - backward_tolerance, s_prev + forward_window]`와 겹치는 세그먼트만 탐색한다.
3. 폐루프에서는 탐색 구간이 `s=0`과 트랙 끝을 넘어갈 때 자동으로 wrap된다.
4. 후보가 없거나 투영 결과의 `|d|`가 `tracked_max_projection_distance`보다 크면 해당 입력을 실패로 처리하고
   `projection_failure_policy`를 적용한다. 이때 즉시 전체 경로 탐색으로 전환하지 않는다.
5. 연속 실패가 `reacquire_after_misses`에 도달하면 전체 경로를 한 번 탐색하여 위치를 재획득하고 WARN 로그를 남긴다.
6. `/global_waypoints` 변경으로 CLCS 기준 경로를 다시 만들면 이전 경로의 연속성 상태를 초기화한다.

### 8.2 YAML 운용값

아래 값은 `src/global_planning/config/global_planning.yaml`에서 조정한다.

| 파라미터 | 설정값 | 설명 |
| --- | --- | --- |
| `continuity_enabled` | `true` | 단조 s-윈도우 추적 활성화 |
| `forward_window` | `1.0` | 이전 `s` 기준 전방 탐색 거리 [m] |
| `backward_tolerance` | `0.5` | 역방향 진행과 위치 노이즈를 허용하는 후방 거리 [m] |
| `initial_seed_window` | `0.0` | 첫 투영 탐색 범위. `0` 이하는 전체 경로 탐색 |
| `tracked_max_projection_distance` | `1.5` | 추적 중 허용하는 최대 `|d|` [m] |
| `reacquire_after_misses` | `5` | 전체 경로 재탐색 전 허용하는 연속 실패 횟수. `0`이면 재획득 비활성화 |

`forward_window`는 한 odometry 주기 동안 차량이 이동할 수 있는 거리보다 충분히 커야 한다.
`tracked_max_projection_distance`는 정상적인 장애물 회피 오프셋보다 크게 설정하되, 트랙 형상과 센서 오차를
확인하여 조정한다.

참조 경로 smoothing/reduction 기능은 제거되었으므로 `reference_resample_step`,
`enable_path_smoothing`, `enable_curvature_reduction`은 현재 노드의 지원 파라미터가 아니다.
