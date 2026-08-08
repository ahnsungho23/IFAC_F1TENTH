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

## 8. 참조 경로 적응 (Reference Path Adaptation, 기본 비활성)

Würsching & Althoff, *"Robust and Efficient Curvilinear Coordinate
Transformation with Guaranteed Map Coverage for Motion Planning"* (IEEE IV
2024)의 Alg. 1을 폐루프 트랙용으로 C++ 포팅한 전처리 단계다
(`reference_path_adapter.{hpp,cpp}`). CLCS를 빌드하기 **전에** 참조 경로를
수정해, 트랙 코리도 안의 모든 점이 **유일한 곡선좌표 투영**(곡률 특이형)을
갖도록 만든다.

### 8.1 동작 원리 (단계별)

1. **앵커 선정**: `|kappa| < anchor_curvature_threshold`인 구간(직선)의
   중앙점들을 고정 앵커로 잡는다. 논문의 파티션 경계에 해당하며, 앵커 없이
   폐루프 전체를 세분하면 볼록한 코너가 평탄해지는 대신 수축한다(끝점 고정이
   Lemma 3의 전제).
2. **세분(Subdivision)**: 앵커 사이 각 굽음 구간을 양 끝 고정 상태로
   Lane-Riesenfeld 3차 B-스플라인 세분(`subdivision_refinements`회, 논문 k=5)
   하여 C² 근사 곡선을 얻는다 (Lemma 1).
3. **판정**: 모든 점에서 `rho = |kappa| * (안쪽 경계까지 법선 거리 +
   boundary_margin) < 1`이면 종료 (`criterion_met`, 식 (4)/(6), Lemma 2).
   안쪽 경계는 입력 waypoint의 `d_left`/`d_right`로 만든 트랙 경계
   폴리라인이고, 거리는 레이캐스팅으로 매 반복 재측정한다.
4. **리샘플**: 미충족이면 각 구간을 `reference_resample_step` 간격으로 균일
   리샘플(끝 고정)한다. 볼록포 성질로 곡률이 줄며(Lemma 3), 이 스텝 크기가
   수렴 속도를 결정한다 (논문 Sec. IV).
5. **경계 가드**: 리샘플 결과의 3칸 코드(p_i→p_{i+3})가 트랙 경계와 교차하면
   수용하지 않고 중단한다 (`boundary_hit`, 논문 Fig. 6). 반복 상한은
   `adaptation_max_iterations` (`max_iterations`로 보고).

### 8.2 파라미터 (`global_planning.yaml`)

#### 새로 추가된 파라미터 (5개)

| 파라미터 | 기본값 | 의미 | 검증 (위반 시) |
|---|---|---|---|
| `subdivision_refinements` | 5 | 반복당 B-스플라인 세분 횟수 k (논문 k=5) | [1, 8] 밖이면 5로 리셋 |
| `adaptation_max_iterations` | 10 | 세분→판정→리샘플 루프 반복 상한 (논문 n̄_iter) | < 1이면 10으로 리셋 |
| `boundary_margin` | 0.05 | 경계 거리에 더하는 안전 여유 ε [m] (논문의 거리 과대근사) | < 0 또는 비유한이면 0.05로 리셋 |
| `max_absolute_curvature` | 0.0 | 전역 \|κ\| 상한 κ̄ [1/m]. **0 = 비활성**. 논문의 파티션 캡 κ̄_Gm을 대신하는 선택적 전역 캡 | — |
| `anchor_curvature_threshold` | 0.1 | 이 값 미만의 \|κ\|는 "직선"으로 판정 [1/m]. 직선 런의 중앙이 고정 앵커가 됨 | ≤ 0이면 0.1로 리셋 |

#### 기존 선언만 있다가 실동작이 연결된 파라미터 (3개)

| 파라미터 | 기본값 | 이전 | 현재 |
|---|---|---|---|
| `enable_path_smoothing` | false | 경고만 출력 | 세분 1회 적용 (Lemma 1만) |
| `enable_curvature_reduction` | false | 경고만 출력 | **전체 Alg. 1 루프** 실행 |
| `reference_resample_step` | 0.0 | 경고만 출력 | 리샘플 간격 Δs [m]. **0이면 입력 중앙값 간격 사용**. 수렴 속도를 결정하는 핵심 노브 (혼자 >0이면 리샘플 전용 모드) |

#### 조합별 동작

```
셋 다 기본값             → 어댑터 완전 미작동 (현행과 동일)
smoothing만 true         → 세분 1회 (+step>0이면 리샘플)
curvature_reduction=true → Alg.1 루프 (smoothing 플래그 무관하게 루프에 세분 포함)
step만 >0                → 균일 리샘플만
```

### 8.3 주의 사항

- **기본 비활성이며, 켜기 전에 반드시 읽을 것**: `obstacle_detector`는 원본
  `/global_waypoints`로 자체 CLCS를 빌드한다. 이 노드에서만 적응을 켜면
  에고 프레임과 장애물 프레임의 (s, d)가 서로 달라진다. 수정이 실제로
  일어나면 노드가 WARN을 1회 출력한다.
- **현재 IQP raceline에서는 no-op**: 전 구간 `rho ≈ 0.81 < 1`이라
  `already_satisfied`로 즉시 종료하고 경로를 건드리지 않는다 (실측 검증됨).
- **좁은 헤어핀에서는 기준이 불충족일 수 있다**: ifac_track centerline의
  헤어핀은 필요한 접촉원 반경이 코리도 폭을 넘어 기하적으로 달성 불가능하고,
  루프는 벽을 넘는 대신 `boundary_hit`/`max_iterations`로 정직하게 보고한다.
  또한 이 적응은 곡률 특이형 비유일성만 다루며, 헤어핀 레그 간 근접(브랜치
  근접형)은 원리적으로 해결하지 못한다 — 그 문제는 시간 연속성
  (`convertTracked`, edge_test 브랜치)이 담당한다.
- 로그 형식: `Reference path adaptation: <stop_reason> (iterations=..
  points=..-><.. max|kappa|=..->.. max_rho=..->..)`.

## 9. 단조 s-윈도우 추적 (Monotonic s-window tracking, 기본 활성)

호길이 s의 단조 진행을 이용한 윈도우 탐색으로, `convertTracked()`가 담당한다.
**브랜치 근접형 비유일성**
(헤어핀 반대 레그: 공간상 1.41 m 옆이지만 호길이로는 반 바퀴 거리)에 의한
투영 플립을 방어한다 — §8의 경로 적응(곡률 특이형)과는 다른 종류의 문제를
다루며 서로 독립적으로 동작한다.

### 9.1 동작 원리 (단계별)

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

### 9.2 파라미터 (`global_planning.yaml`)

| 파라미터 | 기본값 | 의미 | 검증 (위반 시) |
|---|---|---|---|
| `continuity_enabled` | true | false면 무상태 `convert()` 사용 (전역 min\|d\|) | — |
| `forward_window` | 1.0 | s_prev 전방 윈도우 W [m] | ≤ 0이면 1.0으로 리셋 |
| `backward_tolerance` | 1.0 | s_prev 후방 허용 ε [m] (역주행·노이즈) | < 0이면 1.0으로 리셋 |
| `initial_seed_window` | 0.0 | 첫 fix 탐색 구간. **0 = 전체 탐색** | — |
| `tracked_max_projection_distance` | 1.5 | 추적 fix 전용 유클리드 게이트 [m]. 정상 회피 \|d\|(~1.1)보다 크고 레그 간격(1.41)보다 작게 | — |
| `reacquire_after_misses` | 15 | 연속 미스 후 전역 재획득 (30 Hz 기준 ~0.5초). **0 = 재획득 없음** | < 0이면 15로 리셋 |

### 9.3 주의 사항

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
