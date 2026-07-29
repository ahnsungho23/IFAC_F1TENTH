# state_machine_node

## 목적

글로벌 주행, 정적 장애물 회피, 동적 장애물 추월 상태를 관리하고
`f110_msgs/msg/StateMachine`을 발행합니다. 상태는 `global`, `avoid`,
`overtake` 세 가지이며, `breake`/정지 상태와 관련된 로직은 포함하지 않습니다.

## 동작 원리

`global` 상태에서 최근 5회 수신한 회피 또는 추월 경로 중 non-empty 경로가 3회 이상이면
해당 상태로 전환합니다. 회피·추월 경로의 합류 조건이
충족되면 `global`로 복귀합니다. 두 경로의 진입 조건이 동시에 만족되면 `avoid`를 우선합니다.

일반 local path는 마지막 tail 구간 도달 여부와 ego의 `d`를 함께 검사합니다. 정적 회피
경로가 `ot_line=raceline_global_handoff`를 발행하면 local planner가 merge geometry를 이미
확인한 것으로 보고, ego가 global line에 설정 시간 동안 유지되는지만 확인해 복귀합니다.
반대로 `ot_line=raceline_static_prepare`는 최초 장애물 군집을 안정화하는 준비 감속이고,
`ot_line=raceline_static_safe_stop`은 장애물 앞 정지 상태입니다. 둘 다 `d=0`이더라도 합류로
판정하지 않습니다. 이 표식들이 유지되는 동안 합류 타이머를 리셋하고 `STATE_AVOID`를 유지하여
waypoint selector가 장애물을 통과하는 global 경로로 되돌아가지 않게 합니다.

## 토픽과 메시지

| 구분 | 토픽 | 메시지 |
|---|---|---|
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` |
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` |
| 구독 | `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` |
| 구독 | `/overtake_waypoints` | `f110_msgs/msg/OTWpntArray` |
| 발행 | `/state` | `f110_msgs/msg/StateMachine` |

상태 값은 `GLOBAL=0`, `AVOID=1`, `OVERTAKE=2`입니다.

## 파라미터와 실행

파라미터 파일은 `config/state_machine.yaml`이며, 토픽 이름·주기·입력 stale 시간·
M-of-N 진입 조건(`local_path_confirmation_window_size`,
`local_path_confirmation_min_hits`)·경로 검증 및 합류 조건을 설정할 수 있습니다.

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch state_machine state_machine.launch.py
```
