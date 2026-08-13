# frenet_odom_node

## 1. 노드 목적

`frenet_odom_node`는 map frame 기준 차량 위치 `(x, y)`를 global path 기준 Frenet 좌표 `(s, d)`로 변환한다.
입력 odometry는 `/pf/pose/odom`에서 받고, 변환 결과는 `/car_state/frenet/odom`으로 publish한다.

## 2. 동작 원리

1. `/global_waypoints`에서 `f110_msgs/msg/WpntArray`를 latched QoS로 구독한다.
2. waypoint의 `x_m`, `y_m`, `s_m`만 Frenet 계산에 사용한다.
3. waypoint를 CommonRoad-CLCS C++ `CurvilinearCoordinateSystem` 기준 경로로 빌드한다.
4. 차량 odometry가 들어오면 CLCS projection으로 Frenet `s`, `d`와 segment index를 계산한다.
5. `closed_loop=true`이면 마지막 waypoint와 첫 번째 waypoint를 연결한 기준 경로를 만들고, 출력 `s`는 track length 기준으로 정규화한다.
6. yaw 변환에는 `tf2`를 사용하지 않고, 로컬 수학 함수로 heading error를 계산한다.

`d_m`은 waypoint 자체의 lateral offset일 수 있으므로 차량의 `d` 계산에 사용하지 않는다.

## 3. 구독 토픽

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/pf/pose/odom` | `nav_msgs/msg/Odometry` | 차량의 map frame 기준 위치. `pose.pose.position.x`, `pose.pose.position.y`를 사용한다. |
| `/global_waypoints` | `f110_msgs/msg/WpntArray` | global path. 각 waypoint의 `x_m`, `y_m`, `s_m`을 사용한다. |

## 4. 발행 토픽

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | Frenet 좌표 결과. `pose.pose.position.x=s`, `pose.pose.position.y=d`, `pose.pose.position.z=0.0`이다. |

출력 odometry의 `header.frame_id` 기본값은 `frenet`이고, `child_frame_id`에는 closest segment index 문자열을 넣는다.
`pose.pose.position.x=s`, `pose.pose.position.y=d`이고, 설정에 따라 twist의 `linear.x`, `linear.y`에는 Frenet velocity `v_s`, `v_d`가 들어간다.

## 5. 주요 파라미터

파라미터 파일: `src/global_planning/config/global_planning.yaml`

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `odom_topic` | `/pf/pose/odom` | 입력 odometry 토픽 |
| `waypoint_topic` | `/global_waypoints` | global waypoint 토픽 |
| `frenet_odom_topic` | `/car_state/frenet/odom` | 출력 Frenet odometry 토픽 |
| `closed_loop` | `true` | 마지막 waypoint와 첫 번째 waypoint를 연결할지 여부 |
| `frenet_frame_id` | `frenet` | 출력 odometry frame id |
| `projection_failure_policy` | `drop_message` | projection 실패 시 처리 방식. `drop_message`, `publish_last_valid`, `publish_nan` |
| `velocity_frame` | `body` | 입력 odometry twist를 body frame 또는 map frame으로 해석 |

## 6. 실행 방법

1. ROS 2 Jazzy 환경을 source한다.
2. 패키지를 빌드한다.
3. launch 파일을 실행한다.

```bash
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select global_planning
source install/setup.zsh
ros2 launch global_planning global_planning.launch.py
```

개별 실행이 필요하면 설치 후 다음처럼 실행한다.

```bash
ros2 run global_planning frenet_odom_node --ros-args --params-file src/global_planning/config/global_planning.yaml
```

## 7. 확인 절차

1. `/global_waypoints`가 2개 이상의 waypoint를 publish하는지 확인한다.
2. `/pf/pose/odom`이 차량 위치를 publish하는지 확인한다.
3. `/car_state/frenet/odom`의 `pose.pose.position.x`, `pose.pose.position.y`가 각각 계산된 `s`, `d`인지 확인한다.
4. `/car_state/frenet/odom.child_frame_id`가 closest segment index 문자열인지 확인한다.

## 8. 단조 s-윈도우 추적 (Monotonic s-window tracking, 기본 활성)

호길이 s의 단조 진행을 이용한 윈도우 탐색으로, `convertTracked()`가 담당한다.
**브랜치 근접형 비유일성**
(헤어핀 반대 레그: 공간상 1.41 m 옆이지만 호길이로는 반 바퀴 거리)에 의한
투영 플립을 방어한다.

> ℹ️ 곡률 특이형(rho ≥ 1) 비유일성을 다루던 **참조 경로 적응
> (ReferencePathAdapter, IV'24 Alg.1 포팅)은 2026-08-09에 제거**됐다 —
> 검증 스냅샷에서 no-op이었고, 이 노드 단독 적용은 obstacle_detector와의
> Frenet 프레임 불일치를 만들기 때문이다. 재도입하려면 publisher 단 적용이
> 선행돼야 한다. 상세 경위는 `AGENTS.md`의 제거 기록과
> `docs/proposal_remove_reference_path_adapter.md` 참고.

### 8.1 동작 원리 (단계별)

1. **첫 fix**: `initial_seed_window <= 0`(기본)이면 전체 탐색으로 초기 s를
   획득한다 (폐루프 트랙은 2D Pose Estimate로 임의 위치에서 시작 가능).
   `> 0`이면 `[0, initial_seed_window]` 구간만 탐색한다 (출발 위치가 사전에
   정해져 있는 경우에만 사용).
2. **추적**: 이후에는 호길이 구간 `[s_prev − backward_tolerance,
   s_prev + forward_window]`와 교차하는 세그먼트만 후보로 투영한다.
   폐루프에서는 윈도우가 봉합점(s=0/L)을 넘어 wrap된다. 반대 레그는 이
   구간에 아예 들어오지 않으므로 플립이 원천 차단된다.
3. **유클리드 게이트**: 윈도우 안에서 찾은 투영이라도 |d| >
   `tracked_max_projection_distance`이면 미스로 처리한다 (텔레포트로 인접한
   무관 구간에 눌러앉는 것 방지).
4. **fail-closed**: 윈도우 미스는 실패로 발행된다
   (`projection_failure_policy` 적용). **조용한 전역 폴백은 없다** — 그것이
   바로 이 로직이 막는 플립이기 때문.
5. **재획득 탈출구**: `reacquire_after_misses`회 연속 미스가 쌓이면 단 한 번
   전역 재탐색으로 s를 재획득하고 WARN 로그를 남긴다 (충돌·텔레포트·RViz
   2D Pose Estimate 복구용). `0`이면 재획득 없이 엄격 fail-closed로 동작한다.
6. 성공 시에만 `s_prev`가 갱신되고 미스 카운터가 리셋된다. 참조 경로가
   재빌드되면 상태는 초기화되어 재획득한다.

### 8.2 파라미터 (`global_planning.yaml`)

| 파라미터 | 기본값 | 의미 | 검증 (위반 시) |
|---|---|---|---|
| `continuity_enabled` | true | false면 무상태 `convert()` 사용 (전역 min\|d\|) | — |
| `forward_window` | 1.0 | s_prev 전방 윈도우 W [m] | ≤ 0이면 1.0으로 리셋 |
| `backward_tolerance` | 1.0 | s_prev 후방 허용 ε [m] (역주행·노이즈) | < 0이면 1.0으로 리셋 |
| `initial_seed_window` | 0.0 | 첫 fix 탐색 구간. **0 = 전체 탐색** | — |
| `tracked_max_projection_distance` | 1.5 | 추적 fix 전용 유클리드 게이트 [m]. 정상 회피 \|d\|(~1.1)보다 크고 레그 간격(1.41)보다 작게 | — |
| `reacquire_after_misses` | 15 | 연속 미스 후 전역 재획득 (30 Hz 기준 ~0.5초). **0 = 재획득 없음** | < 0이면 15로 리셋 |

### 8.3 주의 사항

- `forward_window`는 프레임당 이동거리(`v·Δt`, 실차 20 Hz × 7.5 m/s ≈ 0.38 m)
  보다 충분히 커야 한다. 기본 1.0 m는 약 2.7배 여유.
- 윈도우 미스와 재획득은 로그로 확인한다: 미스는
  `CLCS projection failed: no projection within monotonic window (miss N)`,
  재획득은 `Monotonic s-window re-acquired via global search after N ...`.
- 무상태 `convert()`(장애물 투영용)는 변경 없다 — 추적은 에고 pose 스트림
  전용이다.
- 검증: gtest 8종 — 무상태 일치, 헤어핀 반대 레그 거부(148°·레그 1.41 m
  재현), 봉합점 wrap, fail-closed 텔레포트, 재획득, 후방 허용, 시드 윈도우,
  유클리드 게이트.
