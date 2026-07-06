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
10. 요청 상태가 `avoid`인데 fresh하고 유효한 avoid path가 없으면 `STATE_GLOBAL`로 fallback한다.
11. 요청 상태가 `overtake`인데 fresh하고 유효한 overtake path가 없으면 `STATE_GLOBAL`로 fallback한다.
12. 요청 상태 문자열이 잘못되면 `STATE_GLOBAL`로 fallback한다.
13. publish tick마다 발행된 local path(`/avoid_waypoints`, `/overtake_waypoints`)를 재평가하고, 평가를 통과해야 전이를 허용한다 (5.1절).
14. raw 요청 상태에 anti-oscillation 필터(dwell time + N-tick debounce)를 적용한 뒤 확정 상태를 발행한다 (5.2절).
15. publish timer 주기마다 `/state`를 발행한다.

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

## 5.1 Local Path 평가 (LocalPathAssessment)

freshness만으로는 planner가 발행한 local path가 실제로 따라갈 수 있는 경로인지 보장하지 못한다. `path_eval_enabled: true`이면 publish tick마다 마지막으로 수신한 `/avoid_waypoints`, `/overtake_waypoints`를 최신 ego pose와 obstacle set 기준으로 재평가하고, 아래 check를 전부 통과해야(`valid == true`) 해당 상태로의 전이를 허용한다.

평가 절차는 다음과 같다.

1. **has_path**: waypoint array가 비어 있지 않아야 한다.
2. **long_enough**: path의 `s` span이 `path_min_length_m` 이상이어야 한다 (track wrap 고려).
3. **starts_near_ego**: path 시작점이 ego의 현재 `s`에서 `path_start_max_gap_m` 안에 있어야 한다. 오래됐거나 엉뚱한 구간의 잔여 경로를 거부한다. ego pose가 없으면 보수적으로 거부한다.
4. **within_track_bounds**: 모든 waypoint가 트랙 경계까지 `path_min_bound_margin_m` 이상의 마진을 유지해야 한다. planner가 `d_right`/`d_left`를 채우지 않으면(전부 0) 경계 정보 없음으로 보고 이 check는 통과시킨다.
5. **collision_free**: 각 waypoint가 최신 obstacle의 `s` 구간과 겹칠 때 obstacle 폭 `[d_right, d_left]`까지의 lateral 간격이 `path_min_obstacle_gap_m` 이상이어야 한다. (TODO: dynamic obstacle 위치 예측, ego 차폭 inflation)
6. **curvature_ok**: `max |kappa_radpm|`이 `path_max_kappa_radpm` 이하여야 한다. (TODO: 속도 프로파일과 마찰 한계 `kappa * v^2` 검사)

평가 실패 시 어떤 check가 실패했는지 throttled warning으로 남긴다. `score` 필드는 향후 score 기반 히스테리시스와 AVOID/OVERTAKE tie-breaking 용도로 예약된 skeleton이다.

## 5.2 전이 안정화 (Anti-Oscillation)

obstacle evidence나 path freshness가 경계값 근처에서 흔들리면 `/state`가 tick마다 뒤바뀌는 oscillation이 발생할 수 있다. 이를 막기 위해 `resolve_requested_state()`의 raw 요청 상태에 `apply_transition_stability()` 필터를 적용한 결과를 발행한다.

동작 규칙은 다음과 같다.

1. **안전 fallback은 즉시**: 확정 상태(`AVOID`/`OVERTAKE`)를 지탱하는 path가 stale/empty/invalid가 되면 debounce와 dwell을 무시하고 즉시 `STATE_GLOBAL`로 복귀한다. 유효하지 않은 path로 회피/추월 상태를 유지하지 않는다.
2. **debounce**: 새 요청 상태는 연속 `transition_confirm_ticks` tick(publish 주기 기준) 동안 유지되어야 전이가 확정된다. 요청이 중간에 바뀌면 카운트는 리셋된다.
3. **dwell time**: 마지막 전이 후 `min_state_dwell_sec`이 지나야 다음 전이를 허용한다. 안전 fallback으로 `GLOBAL`에 떨어진 직후에도 dwell을 다시 채워야 재진입할 수 있어 stale-flicker에 의한 진동을 막는다.

기본값(10 Hz, `transition_confirm_ticks: 3`, `min_state_dwell_sec: 1.0`) 기준으로 상태 진입은 최소 0.3 s의 일관된 evidence를 요구하고, 상태 간 전환은 1 s에 한 번으로 제한된다. 안전 방향(GLOBAL 복귀)만 지연이 없다.

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
| `path_eval_enabled` | `true` | local path 품질 평가를 전이 게이트로 사용할지 여부 |
| `path_min_length_m` | `1.5` | local path 최소 `s` span |
| `path_start_max_gap_m` | `1.0` | path 시작점과 ego `s`의 최대 허용 거리 |
| `path_min_bound_margin_m` | `0.05` | waypoint가 트랙 경계까지 유지할 최소 마진 |
| `path_min_obstacle_gap_m` | `0.3` | path가 obstacle과 유지할 최소 lateral 간격 |
| `path_max_kappa_radpm` | `3.0` | local path 최대 곡률 한계 |
| `min_state_dwell_sec` | `1.0` | 전이 후 다음 전이까지 최소 체류 시간 (안전 fallback 제외) |
| `transition_confirm_ticks` | `3` | 전이 확정에 필요한 연속 tick 수 (debounce) |

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
2. `global_blocked` evidence 자체에 enter/release 이원화 threshold(히스테리시스)를 추가한다. (상태 수준 debounce/dwell은 5.2절에 구현됨)
3. local path 평가에서 dynamic obstacle의 `vs`/`vd` 기반 위치 예측과 ego 차폭 inflation을 반영한다.
4. `LocalPathAssessment.score`를 정규화 품질 점수로 구현하고 score 기반 히스테리시스로 확장한다.
5. `STATE_OVERTAKE` side selection과 abort 조건을 추가한다.
6. `wpnt_publisher`에서 `/state` 기반 source selection을 연결한다.
7. 상태 전이 로그와 debug topic을 추가한다.
8. reactive/safety state 확장을 검토한다.
