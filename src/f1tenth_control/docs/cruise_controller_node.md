# cruise_controller_node

전방 동적 상대차와의 **간격을 속도 상한으로 변환**해 발행하는 종방향 보조 노드입니다.
경로(조향)에는 일절 개입하지 않고 `/cruise_speed_limit` 하나만 내며,
`control_map_node`가 그 값을 목표 속도에 `min()`으로 얹습니다.

> 튜닝 절차·수식 유도·실차 계측 방법은 `CRUISE_TUNING_GUIDE.md`를 보십시오.
> 이 문서는 "무엇을 구독해 무엇을 내고 어떻게 띄우는가"를 다룹니다.

## 1. 목적

- 추월이 불가능한 구간에서 앞차를 들이받지 않고 **일정 간격으로 따라가기**.
- 상대차가 급감속해도 제동거리가 남도록 **속도 상한을 미리 낮추기**.
- 상대차·에고·상태 입력이 끊기면 **느려지는 방향으로 실패**하기.

## 2. 동작 원리 (단계별)

1. `/state`가 `STATE_CRUISE`가 아니면 **`maximum_speed`를 그대로 발행**합니다.
   즉 GLOBAL·AVOID 주행에는 아무 영향이 없습니다(상한이 항상 무해한 값).
2. `STATE_CRUISE`이면 `/opp_obs`의 첫 장애물을 추종 대상으로 잡습니다.
   `is_static`이거나 필드에 NaN이 있으면 버립니다.
3. 에고 Frenet `s`와 상대차 `s_center`·`s_start`·`s_end`로 **원시 간격**을 냅니다.
   `raw_gap = (앞으로의 s 차이) − 상대차 길이/2 − ego_front_offset`.
4. 상대차 위치 분산 `s_var`(옵션으로 `vs_var`·`s_vs_cov`까지 시간 전파)로
   `sigma_gap`을 만들고, `effective_gap = raw_gap − uncertainty_sigma × sigma_gap`으로
   **보수적인 간격**을 씁니다.
5. 목표 간격 `desired_gap`을 계산합니다.
   - 거리 모드(`trailing_mode_distance: true`): `max(minimum_gap, trailing_gap)`
   - 시간 모드: `minimum_gap + trailing_gap × v_ego` (ACC/IDM 표준형)
   - `max_desired_gap > 0`이면 그 값으로 자릅니다.
6. 속도 상한을 두 항의 `min()`으로 정합니다.
   - **피드백 항**: `v_opp + P·간격오차 + I·∫ + D·상대속도`
   - **제동 항**: 상대차가 지금 급제동해도 비상거리 안에서 멈출 수 있는 속도
     `v = −b_ego·τ + √((b_ego·τ)² + 2·b_ego·(여유간격 + v_opp²/(2·b_opp)))`
7. `effective_gap`(비상 판정은 시간 전파 없이 순간값)이 `emergency_stop_distance`
   이하이면 **상한 0**을 냅니다.
8. 결과를 `[0, maximum_speed]`로 자르고, `allow_accel_trailing: false`이면
   현재 에고 속도를 넘지 않게 추가로 누릅니다.

### 페일세이프 (전부 "느려지는 방향")

2026-08-21 후속 sync(`1f6e500b`)에서 **fail-open 경로 두 개가 fail-closed 로 바뀌었습니다.**
아래 표의 ★ 두 줄이 그것이고, 안전 쪽으로는 맞지만 **캡 값이 데드존 아래라 위험합니다**(아래 경고).

| 상황 | 발행값 |
|---|---|
| `/state` 미수신 또는 CRUISE 아님 | `maximum_speed` (무해) |
| CRUISE인데 `/state` heartbeat stale | `blind_trailing_speed` (기본 1.5) |
| CRUISE인데 `/opp_obs`·에고 odom stale, 또는 랩길이 미확보 | `blind_trailing_speed` |
| ★ CRUISE인데 **아직 타깃을 못 잡음**(기동 순서·첫 샘플 유실·검출기 고장) | `blind_trailing_speed` — 구판은 `maximum_speed` 였음 |
| `/opp_obs`가 `clear_confirm_sec` 동안 비어 있음 | 타깃 해제 → `maximum_speed` |
| ★ **`control_map_node`가 `/cruise_speed_limit`을 한 번도 못 받음** | `cruise_stale_speed` (기본 1.5) — 구판은 무제한 |
| **`control_map_node` 쪽에서 `/cruise_speed_limit`이 stale** | `cruise_stale_speed` (기본 1.5) |

> 🔴 **`blind_trailing_speed`·`cruise_stale_speed`의 기본 1.5 m/s는 VESC 구동계
> 데드존(실측 ~2.5 m/s)보다 낮습니다.** 이 값으로 캡이 걸리면 차가 데드존에 걸터앉아
> 못 나갑니다.
>
> 🔴 특히 ★ 두 번째 줄이 위험합니다 — `cruise_limit_enable:=true` 인데
> **크루즈 노드가 안 떠 있으면 프로세스 시작 시점부터 영구히 1.5 m/s 캡**이라
> 차가 아예 출발하지 못합니다. 그래서 `cruise_enable` 런치 기본값을 **false**로 두었습니다.
> 켤 때는 두 페일세이프를 데드존 위(예: 2.8)로 함께 올리십시오.

## 3. 토픽

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| 구독 | `/opp_obs` | `f110_msgs/ObstacleArray` | 추종 대상 (obstacle_detector Layer 3) |
| 구독 | `/car_state/frenet/odom` | `nav_msgs/Odometry` | 에고 Frenet `s`(`position.x`)·`v_s` |
| 구독 | `/global_waypoints` | `f110_msgs/WpntArray` | 랩 길이(전방 간격의 wrap 계산용), transient_local |
| 구독 | `/state` | `f110_msgs/StateMachine` | CRUISE 진입 판정, transient_local |
| 발행 | `/cruise_speed_limit` | `std_msgs/Float64` | 종방향 속도 상한 [m/s], 50 Hz |
| 발행 | `/cruise/gap_data` | `f110_msgs/GapData` | 진단(간격 사슬·속도 사슬·구속 항) |

`/cruise/gap_data`의 `active_constraint`로 **무엇이 상한을 정했는지**를 바로 읽을 수
있습니다: `0=피드백 / 1=제동 / 2=비상정지 / 3=maximum_speed / 4=가속금지`.

## 4. 파라미터

파일: `config/cruise_controller.yaml` (단독 실행용 기준값)
런치에서 같은 이름의 인자가 이 값을 **덮어씁니다** — 실제 적용값은 런치 인자입니다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `trailing_mode_distance` | `true` | true=고정 거리, false=시간 간격 추종 |
| `trailing_gap` | 5.0 | 목표 간격 [m] 또는 headway [s] |
| `minimum_gap` | 0.8 | 정지 시에도 유지할 최소 간격 [m] |
| `max_desired_gap` | 0.0 | 목표 간격 상한 [m], 0=비활성 |
| `trailing_p_gain` / `_i_gain` / `_d_gain` | 1.0 / 0.0 / 0.5 | 간격 PID |
| `integral_limit` | 2.0 | 적분 절댓값 상한 |
| `allow_accel_trailing` | `true` | 상한이 현재 속도보다 높아지는 것을 허용 |
| `emergency_stop_distance` | 0.45 | 이하이면 상한 0 [m] |
| `relative_deceleration` | 1.8 | 제동 캡의 기본 감속도 [m/s²] |
| `ego_deceleration` / `opponent_deceleration` / `actuation_latency` | 0.0 / 0.0 / 0.0 | 제동거리 분해. 전부 0 = 구 식과 비트 동일 |
| `ego_front_offset` | 0.25 | Frenet 기준점→앞범퍼 [m] |
| `uncertainty_sigma` | 2.0 | `sigma_gap`에 곱해 raw gap에서 뺄 배수 |
| `gap_uncertainty_horizon_max` / `opp_speed_confidence_z` | 1.0 / 1.0 | 공분산 시간 전파. 둘 다 0 = 전파 없음 |
| `opponent_timeout` / `ego_timeout` / `state_timeout` | 0.15 / 0.20 / 0.30 | 입력 신선도 [s] |
| `clear_confirm_sec` | 1.00 | 타깃 해제 확인 시간 [s] |
| `blind_trailing_speed` | 1.5 | 입력 stale 시 상한 [m/s] |
| `maximum_speed` | 런치의 `max_speed` | 상한의 상한 [m/s] |

`control_map_node` 쪽 (같은 런치가 함께 넘김):

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `cruise_limit_enable` | 런치 `cruise_enable`(**false**) | `/cruise_speed_limit` 적용 on/off |
| `cruise_speed_limit_topic` | `/cruise_speed_limit` | 구독 토픽 |
| `cruise_speed_limit_timeout` | 0.15 | 신선도 [s] |
| `cruise_stale_speed` | 1.5 | stale 시 폴백 상한 [m/s] |

⚠️ **`max_desired_gap`은 `state_machine.yaml`의 `interference_distance_m`(5.0) 이하로
두십시오.** 크루즈가 유지하려는 간격에서 상태머신이 CRUISE를 빠져나가면
진입/이탈 리밋사이클이 됩니다.

## 5. 실행

🔴 **현재 런치 기본값은 `cruise_enable:=false`(꺼짐)입니다** (2026-08-21 결정).
위 §2 경고대로 페일세이프 값 1.5 m/s가 실측 데드존 아래이기 때문입니다.
`state_machine.yaml`의 `allow_cruise_transition`도 `false`라 **이중 차단** 상태입니다.

되살리려면 **두 곳을 같이** 켜야 합니다 — 한쪽만 켜면 조용히 아무 일도 안 일어납니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

# 실차 — 현재 기본 (크루즈 꺼짐)
ros2 launch f1tenth_control control_real.launch.py

# 크루즈 켜기 (state_machine.yaml 의 allow_cruise_transition:true 도 함께 필요)
ros2 launch f1tenth_control control_real.launch.py cruise_enable:=true

# 데드존 위로 페일세이프를 올려서 켜기 (권장 A/B 형태)
ros2 launch f1tenth_control control_real.launch.py \
    cruise_enable:=true blind_trailing_speed:=2.8 cruise_stale_speed:=2.8

# 추종 간격을 2 m로 좁혀 A/B
ros2 launch f1tenth_control control_real.launch.py cruise_enable:=true trailing_gap:=2.0

# 시뮬
ros2 launch f1tenth_control control_sim.launch.py

# 단독 실행 (디버깅용 — yaml 기준값 그대로)
ros2 run f1tenth_control cruise_controller_node --ros-args \
    --params-file install/f1tenth_control/share/f1tenth_control/config/cruise_controller.yaml
```

## 6. 확인 방법

```bash
# 상한이 실제로 나오는지
ros2 topic echo /cruise_speed_limit

# 무엇이 상한을 정했는지 (active_constraint)
ros2 topic echo /cruise/gap_data --field active_constraint

# 단위 테스트
colcon test --packages-select f1tenth_control && colcon test-result --verbose
```

CRUISE가 아닐 때 `/cruise_speed_limit`이 `max_speed`와 같은 값으로 계속 나오면
정상입니다(무해한 상한). 그보다 낮은 값이 GLOBAL 주행 중에 나오면
`/state`가 잘못 CRUISE로 가 있는지 먼저 보십시오.
