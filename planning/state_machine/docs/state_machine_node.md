# state_machine_node

## 1. 노드 목적

`state_machine_node`는 주행 모드를 `f110_msgs/msg/StateMachine` 형식으로 발행하는 skeleton 노드이다.

이 노드는 `/state`만 발행한다. `/local_waypoints`를 발행하지 않고, waypoint source를 직접 선택하지 않는다. 실제 global/avoid/overtake waypoint 선택은 downstream waypoint selector가 담당해야 한다.

## 2. 동작 원리

1. 노드는 시작 시 YAML 파라미터를 읽는다.
2. `/car_state/frenet/odom`, `/global_waypoints`, `/planner/avoidance/otwpnts`를 구독한다.
3. 선택적으로 `/scan`을 구독할 수 있다.
4. 현재 skeleton에서는 상세 obstacle 판단과 overtake 판단을 구현하지 않는다.
5. `default_state` 파라미터를 요청 상태로 사용한다.
6. 요청 상태가 `avoid` 또는 `overtake`인데 fresh한 avoidance/overtake waypoint가 없으면 `STATE_GLOBAL`로 conservative fallback한다.
7. 요청 상태 문자열이 잘못되면 `STATE_GLOBAL`로 fallback한다.
8. publish timer 주기마다 `/state`를 발행한다.

## 3. 구독 토픽

| Topic | Message | 사용 정보 |
|---|---|---|
| `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | 차량의 Frenet 위치와 closest waypoint index freshness 확인 |
| `/global_waypoints` | `f110_msgs/msg/WpntArray` | global waypoint 존재 여부와 freshness 확인 |
| `/planner/avoidance/otwpnts` | `f110_msgs/msg/OTWpntArray` | avoid/overtake path 존재 여부와 freshness 확인 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 향후 obstacle 판단 확장용. 기본 비활성 |

## 4. 발행 토픽

| Topic | Message | 설명 |
|---|---|---|
| `/state` | `f110_msgs/msg/StateMachine` | 현재 주행 모드 |

`StateMachine.msg` 값은 다음과 같다.

```text
STATE_GLOBAL = 0
STATE_AVOID = 1
STATE_OVERTAKE = 2
```

## 5. 주요 파라미터

YAML 위치:

```text
planning/state_machine/config/state_machine.yaml
```

| Parameter | Default | 설명 |
|---|---:|---|
| `state_topic` | `/state` | 상태 발행 토픽 |
| `frenet_odom_topic` | `/car_state/frenet/odom` | Frenet odom 구독 토픽 |
| `global_waypoints_topic` | `/global_waypoints` | global waypoint 구독 토픽 |
| `avoidance_wpnts_topic` | `/planner/avoidance/otwpnts` | avoid/overtake waypoint 구독 토픽 |
| `scan_topic` | `/scan` | 선택적 LaserScan 구독 토픽 |
| `frame_id` | `map` | `/state` header frame |
| `publish_rate_hz` | `10.0` | 상태 발행 주기 |
| `default_state` | `global` | skeleton에서 사용할 요청 상태 |
| `avoidance_stale_timeout_sec` | `0.5` | avoidance/overtake path stale timeout |
| `global_stale_timeout_sec` | `2.0` | global waypoint stale timeout |
| `frenet_stale_timeout_sec` | `0.5` | Frenet odom stale timeout |
| `use_scan_subscription` | `false` | `/scan` 구독 활성화 여부 |

## 6. 실행 방법

패키지 빌드 후 다음 명령으로 실행한다.

```bash
ros2 launch state_machine state_machine.launch.py
```

다른 YAML을 쓰려면 다음처럼 실행한다.

```bash
ros2 launch state_machine state_machine.launch.py \
  params_file:=/home/haejun/2026_IFAC/planning/state_machine/config/state_machine.yaml
```

## 7. Pipeline 위치

```text
global_trajectory_publisher_node
  -> /global_waypoints : f110_msgs/msg/WpntArray

frenet_odom_node
  subscribes /global_waypoints, /pf/pose/odom
  -> /car_state/frenet/odom : nav_msgs/msg/Odometry

avoidance/overtake planner
  -> /planner/avoidance/otwpnts : f110_msgs/msg/OTWpntArray

state_machine_node
  subscribes /car_state/frenet/odom, /global_waypoints, /planner/avoidance/otwpnts, optional /scan
  -> /state : f110_msgs/msg/StateMachine

downstream waypoint selector
  subscribes /state, /global_waypoints, /planner/avoidance/otwpnts, /car_state/frenet/odom
  -> /local_waypoints : f110_msgs/msg/WpntArray
```

## 8. 상태별 의도

`STATE_GLOBAL`:

1. 기본 주행 상태이다.
2. downstream waypoint selector는 global waypoint에서 local segment를 만들어야 한다.

`STATE_AVOID`:

1. 장애물 회피 경로가 필요한 상태이다.
2. downstream waypoint selector는 fresh한 `/planner/avoidance/otwpnts`를 사용해야 한다.
3. 경로가 비었거나 stale이면 global fallback이 필요하다.

`STATE_OVERTAKE`:

1. 추월 전략 상태이다.
2. 초기 skeleton에서는 avoidance/overtake waypoint topic을 공유할 수 있다.
3. 향후 별도 overtake waypoint topic과 fallback policy를 분리할 수 있다.

## 9. TODO

1. obstacle evidence를 `/scan` 또는 obstacle topic에서 계산한다.
2. Frenet `s`, `d`, closest index 기반으로 state transition 조건을 정의한다.
3. opponent trajectory 또는 projected opponent trajectory 메시지를 사용할지 결정한다.
4. `STATE_AVOID` 진입 조건과 탈출 조건에 hysteresis를 추가한다.
5. `STATE_OVERTAKE` 진입 조건, side selection, abort 조건을 추가한다.
6. downstream waypoint selector에서 `/state` 기반 source selection을 연결한다.
7. 상태 전이 로그와 debug topic을 추가한다.
8. reactive/safety state 확장을 검토한다.
