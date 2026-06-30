# state_machine_node

## 1. 노드 목적

`state_machine_node`는 주행 모드를 `f110_msgs/msg/StateMachine` 형식으로 발행하는 skeleton 노드이다.

이 노드는 `/state`만 발행한다. `/local_waypoints`를 발행하지 않고, waypoint source를 직접 선택하지 않는다. 실제 global/avoid/overtake waypoint 선택은 `wpnt_publisher`가 담당한다.

## 2. 동작 원리

1. 노드는 시작 시 YAML 파라미터를 읽는다.
2. `/car_state/frenet/odom`, `/global_waypoints`, `/avoid_waypoints`, `/overtake_waypoints`, `/perception/obstacles`를 구독한다.
3. `/scan`은 직접 구독하지 않는다. 장애물 검출은 perception 노드가 담당한다.
4. perception 노드는 `/perception/obstacles`를 `f110_msgs/msg/ObstacleArray`로 발행한다.
5. State Machine은 obstacle array를 `ObstacleEvidence`로 요약한다.
6. static obstacle이 global path를 막고 fresh한 `/avoid_waypoints`가 있으면 `STATE_AVOID`를 발행한다.
7. dynamic obstacle이 global path를 막고 fresh한 `/overtake_waypoints`가 있으면 `STATE_OVERTAKE`를 발행한다.
8. static obstacle과 dynamic obstacle이 동시에 감지되고 global path가 막히면 `STATE_AVOID`를 우선한다.
9. obstacle 기반 강제 전이가 없으면 `default_state` 파라미터를 요청 상태로 사용한다.
10. 요청 상태가 `avoid`인데 fresh한 `/avoid_waypoints`가 없으면 `STATE_GLOBAL`로 fallback한다.
11. 요청 상태가 `overtake`인데 fresh한 `/overtake_waypoints`가 없으면 `STATE_GLOBAL`로 fallback한다.
12. 요청 상태 문자열이 잘못되면 `STATE_GLOBAL`로 fallback한다.
13. publish timer 주기마다 `/state`를 발행한다.

## 3. 구독 토픽

| Topic | Message | 사용 정보 |
|---|---|---|
| `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | 차량의 Frenet `s`, `d`와 freshness 확인 |
| `/global_waypoints` | `f110_msgs/msg/WpntArray` | global waypoint 존재 여부, track length 추정, freshness 확인 |
| `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` | 정적 장애물 회피 path 존재 여부와 freshness 확인 |
| `/overtake_waypoints` | `f110_msgs/msg/OTWpntArray` | 동적 장애물/상대 차량 추월 path 존재 여부와 freshness 확인 |
| `/perception/obstacles` | `f110_msgs/msg/ObstacleArray` | perception/tracking 결과 obstacle evidence |

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

## 5. Obstacle Evidence

`state_machine_node`는 `/scan`을 처리하지 않는다. perception 파트가 scan 또는 tracking 정보를 처리한 뒤 `/perception/obstacles`를 발행해야 한다.

Obstacle evidence skeleton은 다음 원칙을 따른다.

1. `is_actually_a_gap == true`인 항목은 장애물이 아니라 gap 후보로 보고 obstacle 판단에서 제외한다.
2. `is_visible == false`인 obstacle은 global path blocking 판단에서 제외한다.
3. 현재 Frenet `s`가 아직 없으면 obstacle 존재 여부만 기록하고 blocking 판단은 하지 않는다.
4. 현재 Frenet `s`가 있으면 obstacle의 앞쪽 `s` gap을 계산한다.
5. `obstacle_lookahead_m` 안에 있고 `abs(d_center)`가 `global_blocking_d_threshold_m` 이하이면 global path를 막는 obstacle로 본다.
6. static obstacle이 global path를 막으면 `static_blocks_global = true`로 요약한다.
7. dynamic obstacle이 global path를 막으면 `dynamic_blocks_global = true`로 요약한다.
8. `static_blocks_global || dynamic_blocks_global`이면 `global_blocked = true`이다.
9. static과 dynamic obstacle이 같이 있고 global path가 막히면 `simultaneous_static_dynamic_on_global = true`이다.
10. 가장 가까운 obstacle, static obstacle, dynamic obstacle의 id와 `s` gap, `d_center`, dynamic velocity 정보를 저장한다.

이 evidence는 `/state` 전이에 직접 사용된다. static path blocking은 `/avoid_waypoints` freshness와 함께 `STATE_AVOID` 조건이 되고, dynamic path blocking은 `/overtake_waypoints` freshness와 함께 `STATE_OVERTAKE` 조건이 된다.

## 6. 주요 파라미터

YAML 위치:

```text
planning/state_machine/config/state_machine.yaml
```

| Parameter | Default | 설명 |
|---|---:|---|
| `state_topic` | `/state` | 상태 발행 토픽 |
| `frenet_odom_topic` | `/car_state/frenet/odom` | Frenet odom 구독 토픽 |
| `global_waypoints_topic` | `/global_waypoints` | global waypoint 구독 토픽 |
| `avoid_waypoints_topic` | `/avoid_waypoints` | 정적 장애물 회피 waypoint 구독 토픽 |
| `overtake_waypoints_topic` | `/overtake_waypoints` | 동적 장애물/상대 차량 추월 waypoint 구독 토픽 |
| `obstacles_topic` | `/perception/obstacles` | perception obstacle array 구독 토픽 |
| `frame_id` | `map` | `/state` header frame |
| `publish_rate_hz` | `10.0` | 상태 발행 주기 |
| `default_state` | `global` | skeleton에서 사용할 요청 상태. 지원 값: `global`, `avoid`, `overtake` |
| `avoid_stale_timeout_sec` | `0.5` | `/avoid_waypoints` stale timeout. 양수 값 사용 |
| `overtake_stale_timeout_sec` | `0.5` | `/overtake_waypoints` stale timeout. 양수 값 사용 |
| `global_stale_timeout_sec` | `2.0` | global waypoint stale timeout. 양수 값 사용 |
| `frenet_stale_timeout_sec` | `0.5` | Frenet odom stale timeout. 양수 값 사용 |
| `obstacles_stale_timeout_sec` | `0.5` | obstacle array stale timeout. 양수 값 사용 |
| `obstacle_lookahead_m` | `3.0` | global lane blocking 판단 전방 거리 |
| `global_blocking_d_threshold_m` | `0.4` | global lane blocking 판단 lateral threshold |

## 7. 실행 방법

패키지 빌드 후 다음 명령으로 실행한다.

```bash
ros2 launch state_machine state_machine.launch.py
```

다른 YAML을 쓰려면 다음처럼 실행한다.

```bash
ros2 launch state_machine state_machine.launch.py \
  params_file:=/home/haejun/2026_IFAC/planning/state_machine/config/state_machine.yaml
```

## 8. Pipeline 위치

```text
perception_node
  subscribes /scan
  -> /perception/obstacles : f110_msgs/msg/ObstacleArray

global_trajectory_publisher_node
  -> /global_waypoints : f110_msgs/msg/WpntArray

frenet_odom_node
  subscribes /global_waypoints, /pf/pose/odom
  -> /car_state/frenet/odom : nav_msgs/msg/Odometry

static obstacle avoidance planner
  -> /avoid_waypoints : f110_msgs/msg/OTWpntArray

dynamic obstacle overtake planner
  -> /overtake_waypoints : f110_msgs/msg/OTWpntArray

state_machine_node
  subscribes /car_state/frenet/odom, /global_waypoints, /avoid_waypoints, /overtake_waypoints, /perception/obstacles
  -> /state : f110_msgs/msg/StateMachine

wpnt_publisher
  subscribes /state, /global_waypoints, /avoid_waypoints, /overtake_waypoints, /car_state/frenet/odom
  -> /local_waypoints : f110_msgs/msg/WpntArray
```

## 9. 상태별 의도

`STATE_GLOBAL`:

1. 기본 주행 상태이다.
2. `wpnt_publisher`는 global waypoint에서 local segment를 만들어야 한다.

`STATE_AVOID`:

1. 정적 장애물 회피 경로가 필요한 상태이다.
2. static obstacle이 global path를 막고 fresh한 `/avoid_waypoints`가 있으면 발행된다.
3. static obstacle과 dynamic obstacle이 동시에 global path를 막는 상황에서는 `STATE_OVERTAKE`보다 우선한다.
4. `wpnt_publisher`는 `/state`를 보고 fresh한 `/avoid_waypoints`를 선택해야 한다.
5. 경로가 비었거나 stale이면 `state_machine_node`는 `STATE_GLOBAL`로 fallback한다.

`STATE_OVERTAKE`:

1. 추월 전략 상태이다.
2. dynamic obstacle이 global path를 막고 fresh한 `/overtake_waypoints`가 있으면 발행된다.
3. static obstacle이 같이 global path를 막는 경우에는 안전 우선 정책으로 `STATE_AVOID`가 발행된다.
4. `wpnt_publisher`는 `/state`를 보고 fresh한 `/overtake_waypoints`를 선택해야 한다.
5. 경로가 비었거나 stale이면 `state_machine_node`는 `STATE_GLOBAL`로 fallback한다.

## 10. TODO

1. Frenet `s`, `d`, closest index 기반으로 state transition 조건을 더 정교하게 정리한다.
2. `global_blocked`에 hysteresis와 debounce를 추가한다.
3. minimum state duration을 추가해 상태 떨림을 줄인다.
4. `STATE_OVERTAKE` side selection과 abort 조건을 추가한다.
5. `wpnt_publisher`에서 `/state` 기반 source selection을 연결한다.
6. 상태 전이 로그와 debug topic을 추가한다.
7. reactive/safety state 확장을 검토한다.
