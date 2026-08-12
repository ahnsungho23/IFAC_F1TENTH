# CMA 회피 명령 timing 진단

## 1. 목적

`control_map_node`와 `drive_source_selector`가 회피 경로를 실제 simulator 명령으로 전달하는
구간을 동일한 monotonic clock으로 계측합니다. 이 기능은 CMA tuning·simulation 전용이며
기본값은 꺼져 있습니다. 진단 메시지는 제어 입력으로 사용하지 않고 기존 조향·속도 계산도
바꾸지 않습니다.

## 2. 동작 원리

1. `control_map_node`는 진단 모드에서 `/state`를 함께 구독합니다.
2. `STATE_AVOID` 전환 이후 발행된 `/local_waypoints`를 처음 사용하는 20 ms control cycle을
   T5로 기록합니다.
3. 같은 cycle의 `/drive_autonomous` 실제 발행을 T6로 기록합니다.
4. `drive_source_selector`가 그 명령을 `/drive`로 실제 전달한 시점을 T7로 기록합니다.
5. 각 event는 `std_msgs/msg/String` JSON companion message로 `/cma_timing/events`에
   발행됩니다. Ackermann command 값이나 header에는 진단용 값을 삽입하지 않습니다.

T6와 T7은 Ackermann header timestamp로 연결합니다. Simulator의 T8/T9와 결합하면
controller 출력부터 callback 수신, 다음 `env.step()` 적용까지 인과적으로 추적할 수 있습니다.

## 3. 토픽과 메시지

| 노드 | 구독 | 메시지 | 역할 |
|---|---|---|---|
| `control_map_node` | `/local_waypoints` | `f110_msgs/msg/WpntArray` | 제어 경로 |
| `control_map_node` | `/state` | `f110_msgs/msg/StateMachine` | 진단 모드에서 회피 경로 식별 |
| `control_map_node` | localization odometry | `nav_msgs/msg/Odometry` | ego 상태 |
| `control_map_node` | `/cma_timing/events` | 해당 없음 | 구독하지 않음 |
| `control_map_node` | `/drive_autonomous` | `ackermann_msgs/msg/AckermannDriveStamped` | 자율 제어 명령 발행 |
| `drive_source_selector` | `/drive_autonomous` | `ackermann_msgs/msg/AckermannDriveStamped` | 자율 명령 입력 |
| `drive_source_selector` | `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | simulator 명령 전달 |
| 두 노드 | `/cma_timing/events` | `std_msgs/msg/String` | default-off T5/T6/T7 진단 발행 |

## 4. 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `timing_diagnostics_enable` | `false` | T5/T6/T7 companion event 활성화 |
| `timing_diagnostics_topic` | `/cma_timing/events` | 진단 event 토픽 |
| `lockstep_mode` | `false` | CMA 전용 동일 timestamp path/odom control cycle |
| `lockstep_period_sec` | `0.01` | lockstep controller 적분 간격 |

공통 launch argument는 `launch/_control_common.py`에 정의되고 simulator selector에는
`launch/control_sim.launch.py`가 같은 값을 전달합니다.

## 5. 빌드와 실행

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select f1tenth_control
source install/setup.zsh
ros2 launch f1tenth_control control_sim.launch.py \
  timing_diagnostics_enable:=true \
  timing_diagnostics_topic:=/cma_timing/events
```

진단을 사용하지 않는 일반 실행에서는 인자를 생략합니다. 그러면 `/state` 진단 구독과 timing
publisher도 생성되지 않습니다.

## 6. 확인 절차

1. `/drive_autonomous`와 `/drive`의 publisher/subscriber가 각각 하나인지 확인합니다.
2. `/cma_timing/events`에서 T5 다음 T6, T7 순서가 유지되는지 확인합니다.
3. T6의 `drive_stamp_ns`와 T7의 `input_drive_stamp_ns`가 같은지 확인합니다.
4. 진단을 끈 실행에서 `/cma_timing/events` publisher가 생성되지 않는지 확인합니다.

```bash
ros2 topic info -v /drive_autonomous
ros2 topic info -v /drive
ros2 topic echo /cma_timing/events
```

## 7. Deterministic lockstep 제어

CMA runner는 `lockstep_mode=true`로 20 ms wall timer를 끄고 같은 timestamp의 GT odom과
`/local_waypoints`가 모인 경우에만 기존 control algorithm을 한 번 실행합니다. 적분 `dt`는
`lockstep_period_sec`로 고정하며 yaw rate도 같은 odom snapshot에서 가져옵니다. 계산된
`/drive_autonomous(k)`는 coordinator가 직접 다음 `env.step()`에 적용합니다. 일반 simulator에서는
두 파라미터를 지정하지 않으므로 기존 timer와 IMU callback 경로가 유지됩니다.
