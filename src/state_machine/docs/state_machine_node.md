# state_machine_node

## 1. 노드 목적

`state_machine_node`는 주행 상태 결정과 최종 waypoint 선택을 한 프로세스에서 수행합니다.
글로벌 주행(`GLOBAL`), 정적 장애물 회피(`AVOID`), 동적 상대차 추월(`OVERTAKE`) 상태를
결정해 `/state`로 발행하고, 결정된 상태에 맞는 경로를 `/local_waypoints`와
`/local_waypoints/path`로 발행합니다.

별도 `wpnt_publisher` 노드는 사용하지 않습니다.

## 2. 동작 원리

1. 글로벌·회피·추월 waypoint와 Frenet odometry를 각각 최신 캐시에 저장합니다.
2. `publish_rate_hz` 타이머에서 committed-state FSM을 한 단계 평가하고 `/state` heartbeat를
   발행합니다.
3. Frenet odometry가 도착할 때마다 현재 committed state에 맞는 경로를 선택합니다.
4. 선택한 `WpntArray`와 RViz용 `Path`를 같은 timestamp로 발행합니다.

`/state`는 기본 10 Hz 타이머 구동이고, local waypoint는 Frenet odometry 이벤트 구동입니다.
Frenet 입력이 멈추면 마지막 index로 경로를 재발행하지 않습니다.

### 2.1 상태 전환

- `GLOBAL`: 최근 N개 메시지 중 M개 이상이 non-empty이면 `AVOID` 또는 `OVERTAKE`로
  진입합니다. 둘 다 만족하면 `AVOID`가 우선입니다.
- `AVOID`/`OVERTAKE`: ego가 local 경로 tail에 도달하고 global line에 설정 시간 동안
  합류하면 `GLOBAL`로 복귀합니다.
- `allow_avoid_transition`과 `allow_overtake_transition`은 진입만 차단합니다. YAML 기본값은
  둘 다 `false`이므로 기본 운용은 `GLOBAL` 고정입니다.

### 2.2 경로 유효성 및 선택

- GLOBAL: waypoint가 2개 이상이고 `s_m`이 엄격히 증가하며 `s_m/x_m/y_m`이 유한한 경로만
  저장합니다. 정상 경로는 정적 데이터로 계속 사용합니다.
- AVOID: 마지막 non-empty 경로를 유지하고 빈 메시지가 오면 즉시 무효화합니다.
- OVERTAKE: 첫 경로를 `overtake_hold_duration_sec` 동안 고정합니다. hold 경과 후 도착한
  non-empty 메시지로 갱신하고, hold 경과 후 도착한 빈 메시지로만 무효화합니다.
- AVOID/OVERTAKE 상태에서 전용 경로가 무효하면 `global_fallback` 정책으로 글로벌 전방
  구간을 발행합니다.

GLOBAL 출력은 Frenet odometry의 `child_frame_id`를 최근접 글로벌 segment index로 해석해
그 다음 waypoint부터 `waypoint_num`개를 원형으로 추출합니다.

## 3. 토픽과 메시지

| 구분 | 기본 토픽 | 메시지 | QoS/역할 |
|---|---|---|---|
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` | Reliable + Transient Local, 글로벌 경로 |
| 구독 | `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` | Reliable + Volatile, 정적 회피 경로 |
| 구독 | `/overtake_waypoints` | `f110_msgs/msg/OTWpntArray` | Reliable + Volatile, 추월 경로 |
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | Reliable + Volatile, 위치·발행 트리거 |
| 발행 | `/state` | `f110_msgs/msg/StateMachine` | Reliable + Transient Local, FSM 상태 |
| 발행 | `/local_waypoints` | `f110_msgs/msg/WpntArray` | Reliable + Volatile, 제어 입력 경로 |
| 발행 | `/local_waypoints/path` | `nav_msgs/msg/Path` | Reliable + Volatile, RViz 시각화 |

상태 값은 `GLOBAL=0`, `AVOID=1`, `OVERTAKE=2`입니다.

## 4. 주요 파라미터

운영 파라미터 파일은 `config/state_machine.yaml`입니다.

| 파라미터 | YAML 값 | 설명 |
|---|---:|---|
| `publish_rate_hz` | `10.0` | FSM 평가와 `/state` heartbeat 주기 |
| `waypoint_num` | `50` | GLOBAL에서 추출할 전방 waypoint 수 |
| `allow_avoid_transition` | `false` | GLOBAL→AVOID 진입 허용 |
| `allow_overtake_transition` | `false` | GLOBAL→OVERTAKE 진입 허용 |
| `local_path_confirmation_window_size` | `5` | 진입 확인 메시지 창 크기 N |
| `local_path_confirmation_min_hits` | `3` | 필요한 non-empty 수 M |
| `overtake_hold_duration_sec` | `2.0` | 추월 경로 갱신 억제 시간 |
| `global_publisher_warn_timeout_sec` | `5.0` | 정적 GLOBAL 발행자 침묵 경고 시간 |
| `frenet_stale_timeout_sec` | `0.5` | 모든 local 출력의 Frenet freshness 제한 |
| `invalid_local_path_policy` | `global_fallback` | local 전용 경로 무효 시 정책 |
| `enter_global_*` | YAML 참고 | local 경로 tail에서 GLOBAL 복귀 조건 |

현재 `invalid_local_path_policy`는 `global_fallback`만 지원합니다. 파라미터는 기동 시 한 번
읽으므로 값을 바꾼 뒤 노드를 재시작해야 합니다.

## 5. 빌드와 실행

Ubuntu 24.04와 ROS 2 Jazzy 환경에서 다음 순서로 실행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select state_machine
source install/setup.zsh
ros2 launch state_machine state_machine.launch.py
```

다른 파라미터 파일을 사용하려면 다음과 같이 지정합니다.

```bash
ros2 launch state_machine state_machine.launch.py \
  params_file:=/absolute/path/to/state_machine.yaml
```

## 6. 단계별 확인

1. `/global_waypoints`와 `/car_state/frenet/odom`이 수신되는지 확인합니다.
2. `/state`가 `publish_rate_hz`와 같은 주기로 발행되는지 확인합니다.
3. `/local_waypoints`가 Frenet odometry와 같은 주기로 발행되는지 확인합니다.
4. `/local_waypoints` publisher가 `state_machine_node` 하나인지 확인합니다.

```bash
ros2 topic echo /state --once
ros2 topic echo /local_waypoints --once
ros2 topic hz /state
ros2 topic hz /local_waypoints
ros2 topic info -v /local_waypoints
```

GLOBAL 모드에서 `child_frame_id`가 빈 문자열, 음수, 숫자가 아닌 값 또는 글로벌 경로 범위
밖의 index이면 local waypoint를 발행하지 않고 경고를 출력합니다.
## RViz 경로 출력 제어

`publish_path_visualization`이 `false`이면 `/local_waypoints/path` publisher와 Path 메시지
생성을 생략한다. 제어 입력인 `/local_waypoints`와 상태 출력 `/state`는 그대로 발행한다.
전체 주행 스택에서는 `f1tenth_control/config/runtime_visualization.yaml`이 이 값을 덮어쓴다.
