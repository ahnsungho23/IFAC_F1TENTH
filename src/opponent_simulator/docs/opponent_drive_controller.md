# opponent_drive_controller 사용법

## 1. 목적

`opponent_drive_controller`는 `f1sim_C`의 2-agent 모드에서 상대차가 센터라인을 따라
주행하도록 만드는 시뮬레이션 전용 노드다. 에고 차량의 상태 머신 및 크루즈 제어와 분리되어
있으며, 상대차용 `/opp_drive`만 발행한다.

## 2. 동작 원리

1. `/centerline_waypoints`에서 폐루프 센터라인과 기준 속도를 받는다.
2. `/opp_racecar/odom`에서 상대차 위치와 자세를 받는다.
3. 현재 위치에서 가장 가까운 waypoint를 찾는다.
4. 경로를 따라 `lookahead_distance_m`만큼 앞선 목표점을 선택한다.
5. Pure Pursuit로 조향각을 계산하고 waypoint 속도에 `speed_scale`을 곱한다.
6. 결과를 `/opp_drive`로 발행한다.

이 노드는 에고 차량의 `/drive`, `/drive_autonomous`, `/cruise_speed_limit`을 사용하거나
발행하지 않으므로 에고 컨트롤러와 토픽이 충돌하지 않는다.

## 3. 토픽

| 구분 | 기본 토픽 | 메시지 형식 | 설명 |
|---|---|---|---|
| 구독 | `/centerline_waypoints` | `f110_msgs/msg/WpntArray` | 상대차 추종 경로와 속도 |
| 구독 | `/opp_racecar/odom` | `nav_msgs/msg/Odometry` | 시뮬레이터 상대차 상태 |
| 발행 | `/opp_drive` | `ackermann_msgs/msg/AckermannDriveStamped` | 상대차 구동 명령 |

## 4. 파라미터

운용값은 `config/opponent_simulator.yaml`에 있다.

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `waypoints_topic` | `/centerline_waypoints` | 상대차가 추종할 waypoint 토픽 |
| `opponent_odom_topic` | `/opp_racecar/odom` | 상대차 odometry 토픽 |
| `opponent_drive_topic` | `/opp_drive` | 상대차 구동 명령 토픽 |
| `speed_scale` | `0.8` | waypoint 속도 배율 |
| `lookahead_distance_m` | `2.0` | Pure Pursuit 전방 주시 거리 `[m]` |
| `wheelbase_m` | `0.33` | 차량 축거 `[m]` |
| `steering_limit_rad` | `0.4` | 조향각 절댓값 상한 `[rad]` |
| `control_rate_hz` | `40.0` | 명령 계산 및 발행 주기 `[Hz]` |
| `odom_timeout_sec` | `0.5` | odometry가 이 시간 이상 갱신되지 않을 때 명령 발행 중단 |
| `enabled` | `true` | 상대차 제어 활성화 |

## 5. 실행 방법

먼저 `f1sim_C`를 `num_agent:=2`로 실행하고, 글로벌 플래너가
`/centerline_waypoints`를 발행하도록 실행한다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch opponent_simulator opponent_simulator.launch.py
```

상대차 속도 배율을 변경하려면 다음처럼 실행한다.

```bash
ros2 launch opponent_simulator opponent_simulator.launch.py speed_scale:=0.7
```

전체 2대 크루즈 시뮬레이션은 다음 명령으로 실행한다.

```bash
~/Desktop/launch_cruise.zsh
```

실행 후 `/opp_drive` 발행자가 하나인지 확인한다.

```bash
ros2 topic info /opp_drive --verbose
```
