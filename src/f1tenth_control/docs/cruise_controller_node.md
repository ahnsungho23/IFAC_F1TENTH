# cruise_controller_node

## 1. 목적

`cruise_controller_node`는 전방 상대차와의 Frenet 종방향 간격을 계산하고,
안전한 추종을 위한 속도 상한을 `/cruise_speed_limit`으로 발행합니다. 경로와 조향각은
변경하지 않으며, `control_map_node`가 waypoint 속도·곡률 제한·크루즈 상한 중 가장 작은
값을 최종 목표 속도로 사용합니다.

## 2. 동작 원리

1. `/state`가 `STATE_CRUISE`인지 확인합니다. 다른 상태에서는 `maximum_speed`를 발행합니다.
2. `/opp_obs`, `/car_state/frenet/odom`, `/global_waypoints`에서 상대차·에고 상태와 트랙 길이를
   가져옵니다.
3. 순환 트랙을 고려해 에고 앞범퍼부터 상대차 후면까지의 간격을 계산합니다.
4. 간격 PID는 실제 `raw_gap`을 목표 간격으로 추종합니다. 공분산을 전파해
   `uncertainty_sigma * sigma_gap`만큼 줄인 `effective_gap`은 제동·비상 제약에만 사용합니다.
5. raw-gap PID 상한과 보수적 제동거리 상한 중 작은 값을 발행합니다. 유효 간격이
   `emergency_stop_distance` 이하면 속도 상한은 0입니다.
6. CRUISE 중 입력이 stale이거나 상대차를 아직 획득하지 못했으면
   `blind_trailing_speed`로 제한합니다. `control_map_node`도 첫 상한 수신 전부터
   `cruise_stale_speed`를 적용합니다.

고정 거리 모드의 목표 간격은 `max(minimum_gap, trailing_gap)`이고, 시간 간격 모드는
`minimum_gap + trailing_gap * ego_speed`입니다. 목표 간격은 `state_machine.yaml`의
`interference_distance_m`보다 커서는 안 됩니다. 목표는 실제 범퍼 간 간격이며,
불확실성 마진은 정상상태 물리 간격을 늘리지 않고 안전 제약만 조입니다.

## 3. 토픽과 메시지

| 구분 | 기본 토픽 | 메시지 | 용도 |
|---|---|---|---|
| 구독 | `/opp_obs` | `f110_msgs/msg/ObstacleArray` | 전방 상대차 상태와 공분산 |
| 구독 | `/state` | `f110_msgs/msg/StateMachine` | CRUISE 활성 여부 |
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | 에고 Frenet 위치와 속도 |
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` | 트랙 길이 계산 |
| 발행 | `/cruise_speed_limit` | `std_msgs/msg/Float64` | 종방향 속도 상한 |
| 발행 | `/cruise/gap_data` | `f110_msgs/msg/GapData` | 간격·불확실성·제약 진단 |

## 4. 파라미터와 launch 인자

운영 기준값은 `config/cruise_controller.yaml`에 있습니다. `control_sim.launch.py`와
`control_real.launch.py`는 공통으로 `_control_common.py`를 사용하며, 아래 튜닝 파라미터를
같은 이름의 launch 인자로 모두 덮어쓸 수 있습니다.

- `trailing_mode_distance`, `trailing_gap`, `minimum_gap`, `max_desired_gap`: 목표 간격
- `trailing_p_gain`, `trailing_i_gain`, `trailing_d_gain`, `integral_limit`: 간격 제어
- `maximum_speed`, `allow_accel_trailing`: 속도 범위
- `emergency_stop_distance`, `relative_deceleration`: 긴급 정지와 기본 제동 상한
- `ego_deceleration`, `opponent_deceleration`, `actuation_latency`: 제동거리 모델
- `uncertainty_sigma`, `gap_uncertainty_horizon_max`, `opp_speed_confidence_z`: 불확실성 조임
- `opponent_timeout`, `ego_timeout`, `state_timeout`, `clear_confirm_sec`,
  `blind_trailing_speed`: 입력 신선도와 fail-safe
- `cruise_speed_limit_timeout`, `cruise_stale_speed`: `control_map_node`의 크루즈 상한
  수신 fail-safe

`maximum_speed`는 별도 크루즈 인자 대신 기존 `max_speed`를 공유합니다. 토픽 이름과
`publish_rate_hz`는 튜닝 인자가 아니므로 YAML의 인터페이스 설정을 그대로 사용합니다.

세부 튜닝 절차는 `CRUISE_TUNING_GUIDE.md`를 참고합니다.

## 5. 실행

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_control control_sim.launch.py cruise_enable:=true
```

예를 들어 YAML을 수정하지 않고 고정 2 m 간격의 P/D 조합을 시험하려면 다음과 같이
실행합니다.

```bash
ros2 launch f1tenth_control control_sim.launch.py \
  trailing_mode_distance:=true trailing_gap:=2.0 \
  trailing_p_gain:=1.0 trailing_i_gain:=0.0 trailing_d_gain:=0.35
```

시간 간격 모드는 `max_desired_gap`을 상태 머신의 `interference_distance_m`
이하로 함께 설정해야 합니다.

```bash
ros2 launch f1tenth_control control_sim.launch.py \
  trailing_mode_distance:=false minimum_gap:=0.8 trailing_gap:=0.2 \
  max_desired_gap:=5.0
```

실차에서는 `control_real.launch.py cruise_enable:=true`를 사용합니다. 실행 후 `/state`,
`/opp_obs`, `/cruise_speed_limit`, `/cruise/gap_data`를 확인합니다.
