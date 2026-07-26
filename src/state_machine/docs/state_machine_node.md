# state_machine_node

## 목적

정적 장애물, 정적 회피 경로, 동적 추월 경로와 차량 Frenet 상태를 종합하여 주행 상태를
결정합니다. 정적 장애물 처리 실패나 입력 만료 시 글로벌 경로로 무조건 복귀하지 않고
`STATE_SAFE_STOP`을 발행합니다.

## 정적 장애물 상태 전이

```text
GLOBAL
  └─ 전방 글로벌 corridor 차단 확인 → BLOCKED

BLOCKED
  ├─ 장애물 해제 + ego가 global 위 → GLOBAL
  ├─ fresh·유효·충돌 없는 회피 경로 5회 연속 수신 → AVOID
  └─ 경로 없음/장애물 근접/입력 stale → SAFE_STOP

AVOID
  ├─ 회피 경로와 장애물 corridor 재검증 → AVOID 유지
  ├─ TTL 안의 마지막 유효 경로 → AVOID 유지
  ├─ merge 완료 + ego d≈0 + global corridor clear → GLOBAL
  └─ 경로 TTL 만료/새 충돌/입력 stale → SAFE_STOP
```

동적 상대차의 `STATE_OVERTAKE` 전이와 `/overtake_waypoints` 계약은 기존과 동일합니다.

## 동작 원리

1. `/perception/obstacles`에서 정적 또는 설정 속도 이하인 장애물만 선택합니다.
2. 장애물의 Frenet 좌우 경계에 추적 분산과 차량 폭·마진을 적용합니다.
3. 장애물이 설정 전방 거리 안에서 `d=0` 글로벌 corridor를 연속 차단하면 `BLOCKED`입니다.
4. `/avoid_waypoints`는 길이, 시작점과 ego 간격, finite 값, 최대 곡률, tail 합류와
   장애물 corridor 충돌을 검사합니다.
5. 검사를 통과한 회피 경로를 `avoid_path_confirm_count`회 연속 수신하면 `AVOID`로
   전환합니다. 빈 경로, 유효하지 않은 경로 또는 stale 간격이 발생하면 횟수를 초기화합니다.
6. AVOID 복귀는 차량이 회피 tail과 글로벌 라인에 도달하고 전방 글로벌 corridor가
   연속으로 clear일 때만 허용합니다.

## 토픽과 메시지

| 구분 | 토픽 | 메시지 |
|---|---|---|
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` |
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` |
| 구독 | `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` |
| 구독 | `/overtake_waypoints` | `f110_msgs/msg/OTWpntArray` |
| 구독 | `/perception/obstacles` | `f110_msgs/msg/ObstacleArray` |
| 발행 | `/state` | `f110_msgs/msg/StateMachine` |

상태 값은 `GLOBAL=0`, `AVOID=1`, `OVERTAKE=2`, `BLOCKED=3`, `SAFE_STOP=4`입니다.

## 파라미터

YAML은 `config/state_machine.yaml`입니다. 주요 그룹은 다음과 같습니다.

- 입력 만료: `*_stale_timeout_sec`, `avoid_path_ttl_sec`
- 확인 조건: `obstacle_confirm_sec`, `obstacle_clear_confirm_sec`, `avoid_path_confirm_count`
- corridor: `global_corridor_horizon_m`, 차량 반폭과 글로벌·회피 마진
- 불확실성: `obstacle_uncertainty_sigma_scale`
- 경로 검사: 최소 길이, 시작 간격, 최대 곡률
- 합류: `enter_global_*`

## 실행

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch state_machine state_machine.launch.py
```

```bash
ros2 topic echo /perception/obstacles
ros2 topic echo /avoid_waypoints
ros2 topic echo /state
```
