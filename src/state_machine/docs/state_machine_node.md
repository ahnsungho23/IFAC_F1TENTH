# state_machine_node

## 목적

글로벌 주행, 정적 장애물 회피, 동적 장애물 추월 상태를 관리하고
`f110_msgs/msg/StateMachine`을 발행합니다. 상태는 `global`, `avoid`,
`overtake` 세 가지이며, `breake`/정지 상태와 관련된 로직은 포함하지 않습니다.

## 동작 원리

`global` 상태에서 최근 5회 수신한 회피 또는 추월 경로 중 non-empty 경로가 3회 이상이면
해당 상태로 전환합니다. 회피·추월 경로의 합류 조건이
충족되면 `global`로 복귀합니다. 두 경로의 진입 조건이 동시에 만족되면 `avoid`를 우선합니다.

### 상태 진입 토글 (`allow_avoid_transition` / `allow_overtake_transition`)

두 파라미터를 `false`로 두면 해당 상태로의 **진입만** 완전히 차단합니다. 판정 순서는 다음과 같습니다.

1. `can_enter_avoid()` / `can_enter_overtake()`의 맨 앞에서 early-return 하므로,
   위의 M-of-N 수신 확인과 경로 검증(`path_eval_*`)은 **아예 평가되지 않습니다**.
   `/avoid_waypoints`가 정상적으로 들어와도 상태는 바뀌지 않습니다.
2. `AVOID`/`OVERTAKE` → `GLOBAL` 복귀는 토글을 보지 않으므로 **항상 허용**됩니다.
   따라서 `default_state`를 `avoid`로 두고 토글을 `false`로 하면, 한 번 `global`로 빠진 뒤
   다시 돌아오지 못합니다.
3. 진입이 꺼진 입력은 "입력 없음" 경고(`State publisher inputs: ...`) 대상에서 제외됩니다.
   단, 이미 그 상태에 진입해 있으면(예: `default_state`) 입력이 실제로 필요하므로 경고를 유지합니다.
4. 기동 로그에 두 값이 함께 출력되고, 둘 다 `false`면
   `Both local-path transitions are disabled...` 경고가 한 번 나옵니다.

파라미터는 기동 시 한 번만 읽습니다(동적 재설정 콜백 없음). 값을 바꾸면 노드를 재시작하세요.
현재 YAML 기본값은 둘 다 `false`(GLOBAL 고정 운용)이며, 코드 기본값은 `true`입니다.

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
상태 진입 토글(`allow_avoid_transition`, `allow_overtake_transition`)·
M-of-N 진입 조건(`local_path_confirmation_window_size`,
`local_path_confirmation_min_hits`)·경로 검증 및 합류 조건을 설정할 수 있습니다.

| 파라미터 | 코드 기본값 | YAML 값 | 설명 |
|---|---|---|---|
| `allow_avoid_transition` | `true` | `false` | `false`면 GLOBAL→AVOID 진입 완전 차단 |
| `allow_overtake_transition` | `true` | `false` | `false`면 GLOBAL→OVERTAKE 진입 완전 차단 |

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch state_machine state_machine.launch.py
```
