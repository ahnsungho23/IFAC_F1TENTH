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

공용 `ClcsFrenetConverter`는 단발 점 투영용 `convert()`와 연속 ego stream용
`convertTracked()`를 함께 제공한다. `convertTracked()`는 직전 `s` 주변의 wrap-aware monotonic
window만 검색하고 연속 miss 뒤에만 bounded global re-acquisition을 수행한다. 따라서 hairpin에서
유클리드 거리가 가까운 반대편 branch로 ego 기준점이 순간 이동하는 것을 막는다. 서로 무관한
장애물 점 투영에는 state를 공유하지 말고 기존 `convert()`를 사용한다.

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

1. ROS 2 Jazzy 환경을 source한다.
2. 패키지를 빌드한다.
3. launch 파일을 실행한다.

```bash
source /opt/ros/jazzy/setup.zsh
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
