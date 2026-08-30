# Perception에서 제어까지: 현재 플래너 파이프라인

## 분석 기준과 결론

- 기준 커밋: `f1e1bba9550923ca1ac48a52c44c82b35b707aad` (`parkasd/safety`와 동일)
- 분석 방식: 소스, 메시지 정의, YAML, launch의 정적 추적
- 실행·rosbag 재생·차량 시험: 수행하지 않음
- 핵심 결론: 현재 로컬 플래너는 **Frenet 기준선의 순서를 보존하면서 `d(s)`만 바꾸는 정적 장애물 회피용 geometry-first path/velocity planner**다. 단순 기하 플래너보다는 강하다. 곡률·횡가속·가감속·조향각 한계를 반영하지만, 시간축 차량 상태를 직접 적분하는 완전한 kinodynamic trajectory optimizer는 아니다.

현재 운영 모드는 `p3_mode: TEST_ACTIVE`이고, 회피 후보 생성기는 P3 analytic corridor 하나다. 소스의 “P0 backup”은 별도의 옛 quintic 후보 격자를 뜻하지 않는다. 안전정지, 준비정지, 커밋, handoff 같은 기존 운영 사다리를 뜻한다.

근거:

- [`local_planning.yaml:842`](../../src/local_planning/config/local_planning.yaml#L842): `TEST_ACTIVE`와 P3 단일 후보 생성기 계약
- [`raceline_spline_planner.cpp:3451`](../../src/local_planning/src/raceline_spline_planner.cpp#L3451): `plan()`
- [`raceline_spline_planner.cpp:3480`](../../src/local_planning/src/raceline_spline_planner.cpp#L3480): 옛 P0 quintic 격자가 제거되었고 `generateP3Candidates()`만 호출
- [`local_planner_node.cpp:3693`](../../src/local_planning/src/local_planner_node.cpp#L3693): P3 출력 부재 시 `P0_BACKUP_ONLY` 운영 사다리

## Phase 1 파일 범위 지도

아래 표는 테스트·과거 설계 문서를 제외하고, 현재 production data path를 구성하거나 그 계약을 정의하는 핵심 파일을 추린 것이다. 헤더와 구현을 한 행에 묶은 경우 입력·출력은 함수 인자만이 아니라 해당 컴포넌트의 ROS/data-flow 입력·출력을 뜻한다.

| 파일과 주요 class/function | 입력 | 출력·역할·연결 관계 |
|---|---|---|
| [`local_planner_main.cpp:21`](../../src/local_planning/src/local_planner_main.cpp#L21) `main()` | ROS arguments | `LocalPlannerNode`를 3-thread executor로 실행한다. obstacle ingress, planning, 기타 callback group의 동시 실행 지점이다. |
| [`local_planner_node.hpp:86`](../../src/local_planning/include/local_planning/local_planner_node.hpp#L86), [`local_planner_node.cpp:798`](../../src/local_planning/src/local_planner_node.cpp#L798) `LocalPlannerNode`, `initializeInterfaces()` | global/obstacle/odom/state ROS messages, timer | 입력 freshness와 snapshot, commitment/safe-stop lifecycle을 조정하고 `RacelineSplinePlanner::plan()` 결과를 `/avoid_waypoints`로 발행하는 orchestration 계층이다. |
| [`raceline_spline_planner.hpp:506`](../../src/local_planning/include/local_planning/raceline_spline_planner.hpp#L506), [`raceline_spline_planner.cpp:3451`](../../src/local_planning/src/raceline_spline_planner.cpp#L3451) `RacelineSplinePlanner::plan()` | global `Wpnt` 배열, ego `s,d,v`, confirmed obstacle snapshot | visible obstacle/context를 만들고 P3 후보 생성, 속도 성형, hard validation, 순위 계산을 연결해 `RacelineSplineResult`를 반환한다. |
| [`raceline_spline_planner.cpp:730`](../../src/local_planning/src/raceline_spline_planner.cpp#L730) `expandVisibleObstacles()` 및 [`raceline_spline_planner.cpp:908`](../../src/local_planning/src/raceline_spline_planner.cpp#L908) `nearestCluster()` | obstacle `s_start,s_end,d_left,d_right`, ego `s`, track length | 차량 폭·margin으로 팽창한 forward obstacle과 한 번에 회피할 blocking cluster를 만든다. perception과 candidate generator 사이의 필터/선택 계층이다. |
| [`p3_shadow.cpp:51`](../../src/local_planning/src/p3_shadow.cpp#L51), [`p3_shadow.cpp:75`](../../src/local_planning/src/p3_shadow.cpp#L75) `P3ShadowEvaluator::run()` | planning context, 좌/우 side, P3 parameter | corridor branch와 M0/M1 knot 조합을 탐색하고 최대 24개 path candidate를 생성·평가한다. `RacelineSplinePlanner::generateP3Candidates()`에서 호출된다. |
| [`p3_analytic_solver.hpp:27`](../../src/local_planning/include/local_planning/p3_analytic_solver.hpp#L27) analytic helpers | station/offset boundary condition, slope·curvature bound | quintic-Hermite 계수, 극값과 제약 검사용 해석 함수를 제공한다. 별도 ROS 입출력은 없다. |
| [`raceline_spline_planner.cpp:2663`](../../src/local_planning/src/raceline_spline_planner.cpp#L2663) geometry recomputation | candidate Cartesian points | `psi_rad`, `kappa_radpm`을 재계산해 속도 성형과 validator/controller가 사용할 waypoint geometry를 만든다. |
| [`raceline_spline_planner.cpp:2344`](../../src/local_planning/src/raceline_spline_planner.cpp#L2344) `applyAvoidanceVelocityLimit()` 및 [`raceline_spline_planner.cpp:2486`](../../src/local_planning/src/raceline_spline_planner.cpp#L2486) approach ramp | path curvature, obstacle gap, ego speed, reference speed | 횡가속 cap, obstacle 옆 속도, 반응거리·제동거리, 전후방 가감속 pass를 결합해 각 waypoint의 `vx_mps`, `ax_mps2`를 만든다. |
| [`raceline_spline_planner.cpp:2805`](../../src/local_planning/src/raceline_spline_planner.cpp#L2805) `validateCandidate()` | candidate path, ego, inflated obstacles, track bound | footprint/벽/obstacle/순서/slope/curvature hard constraint를 검사해 valid 여부와 reject reason을 반환한다. collision과 feasibility의 최종 공통 gate다. |
| [`raceline_spline_planner.cpp:2029`](../../src/local_planning/src/raceline_spline_planner.cpp#L2029) `measureCandidate()` 및 [`candidate_rank.hpp:49`](../../src/local_planning/include/local_planning/candidate_rank.hpp#L49) ranking helper | hard-valid candidate와 obstacle/context | velocity loss, slack, 평균 `|d|` 등의 지표를 산출하고 lexicographic 순위로 best path를 고른다. 하나의 weighted sum cost를 만들지는 않는다. |
| [`p3_maneuver_lifecycle.hpp:135`](../../src/local_planning/include/local_planning/p3_maneuver_lifecycle.hpp#L135), [`p3_maneuver_lifecycle.cpp:158`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L158) `P3ManeuverLifecycle` | 선택 경로, conservative/raw obstacle snapshot, ego progress | 선택 결과를 immutable commitment로 저장하고 suffix 재검증·완료·chaining 상태를 관리한다. |
| [`safe_stop_lifecycle.hpp:78`](../../src/local_planning/include/local_planning/safe_stop_lifecycle.hpp#L78), [`local_planner_node.cpp:2434`](../../src/local_planning/src/local_planner_node.cpp#L2434) safe-stop orchestration | 후보 부재/stale/위험 사유, ego와 obstacle 상태 | stop path latch, hold, release 조건을 관리하고 일반 회피 경로 대신 안전정지 결과를 발행하게 한다. |
| [`obstacle_detector_node.hpp:68`](../../src/obstacle_detector/include/obstacle_detector/obstacle_detector_node.hpp#L68), [`obstacle_detector_node.cpp:1564`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1564) `ObstacleDetectorNode::scanCallback()` | `/scan`, TF, `/map`, `/global_waypoints`, ego odom | cluster, wall filter, track/motion classification을 거쳐 `/static_obs`, `/confirmed_static_obs`, `/opp_obs`를 만든다. local planner의 perception producer다. |
| [`aabb_frenet_projector.cpp:198`](../../src/obstacle_detector/src/aabb_frenet_projector.cpp#L198) `projectCartesianAabb()` | map-frame cluster AABB와 CLCS | 네 꼭짓점/곡선상 최근접 관계를 이용해 obstacle `s_start,s_end,d_left,d_right`를 만든다. detector의 Cartesian→Frenet 경계 변환기다. |
| [`clcs_frenet_converter.hpp:125`](../../src/global_planning/include/global_planning/clcs_frenet_converter.hpp#L125), [`clcs_frenet_converter.cpp:168`](../../src/global_planning/src/clcs_frenet_converter.cpp#L168) `ClcsFrenetConverter::convert()` | ordered global reference와 map-frame point | CommonRoad CLCS의 `s,d`와 projection 정보를 반환한다. detector와 ego Frenet odometry가 같은 좌표 계약을 공유하게 한다. |
| [`frenet_odom_node.cpp:76`](../../src/global_planning/src/frenet_odom_node.cpp#L76) `FrenetOdomNode` | `/pf/pose/odom`, `/global_waypoints` | `/car_state/frenet/odom`에 ego `s,d,v_s,v_d`를 실어 local planner/FSM에 전달한다. |
| [`Obstacle.msg:1`](../../f110_msgs/msg/Obstacle.msg#L1), [`ObstacleArray.msg:1`](../../f110_msgs/msg/ObstacleArray.msg#L1) | detector가 채우는 개별 obstacle/배열 필드 | perception→planning/FSM 메시지 계약이다. position, Frenet box, velocity, covariance, confidence, static flag가 있으나 소비 여부는 필드별로 다르다. |
| [`Wpnt.msg:1`](../../f110_msgs/msg/Wpnt.msg#L1), [`WpntArray.msg:1`](../../f110_msgs/msg/WpntArray.msg#L1), [`OTWpntArray.msg:1`](../../f110_msgs/msg/OTWpntArray.msg#L1) | global 또는 avoidance waypoint fields | global reference, planner output, FSM-selected controller input의 메시지 계약을 정의한다. |
| [`state_machine_node.cpp:730`](../../src/state_machine/src/state_machine_node.cpp#L730) `publish_state_cycle()`/`select_waypoints()` | global/avoid waypoint, ego Frenet odom, opponent | GLOBAL/AVOID/CRUISE를 판정하고 선택 경로를 `/local_waypoints`로 발행한다. planner와 controller 사이의 실제 mux다. |
| [`control_map_node.cpp:147`](../../src/f1tenth_control/control_code/control_map_node.cpp#L147), [`control_map_node.cpp:1523`](../../src/f1tenth_control/control_code/control_map_node.cpp#L1523) `ControlMapNode` | `/local_waypoints`, global fallback, Cartesian odom, state | L1 feedback+curvature feedforward와 속도/조향 rate limit을 적용해 `/drive_autonomous`를 발행한다. |
| [`local_planning.yaml:1`](../../src/local_planning/config/local_planning.yaml#L1), [`local_planning.launch.py:1`](../../src/local_planning/launch/local_planning.launch.py#L1) | launch arguments와 YAML 값 | vehicle/track/collision/speed/P3/freshness/topic 운영값을 `local_planner_node`에 주입하고 detector/map-server 포함 여부를 연결한다. runtime override의 실제 유효값은 정적 분석만으로 확인되지 않는다. |

수학 helper, validator, lifecycle은 `local_planning` 내부 함수 호출로 연결되고 ROS topic을 직접 소유하지 않는다. 반대로 detector, Frenet odometry, FSM, controller는 패키지 경계를 넘는 ROS interface를 소유한다. 세부 메시지 필드의 “존재”와 planner의 “실제 사용”은 [perception-to-planning 문서](../04_interfaces/perception_to_planning.md)에서 구분한다.

## 한눈에 보는 데이터 흐름

```text
/scan + TF(map<-scan) + /map + /global_waypoints + ego odom
                    |
                    v
          obstacle_detector
  scan clustering -> wall/corridor filtering -> CLCS AABB projection
  -> tracking/motion classification -> layer merge
                    |
        +-----------+-------------------+
        |                               |
 /confirmed_static_obs              /static_obs
 confirmed static geometry          provisional speed hint
        |                               |
        +-----------+-------------------+
                    v
             local_planner_node
  snapshot/freshness -> guard/stabilization -> P3 corridor
  -> C2 quintic-Hermite candidates -> speed shaping
  -> hard validation -> lexicographic ranking
  -> commitment / safe-stop / global handoff
                    |
                    v
        /avoid_waypoints (OTWpntArray)
                    |
                    v
             state_machine_node
  GLOBAL/AVOID/CRUISE 판정 + 경로 선택
                    |
                    v
          /local_waypoints (WpntArray)
                    |
                    v
             control_map_node
  L1 guidance + curvature FF + bicycle inverse model
  + steering/speed rate limiting
                    |
                    v
             /drive_autonomous
```

## 단계별 입출력·조건·실패 표

| 단계 | 입력 → 출력 | 핵심 함수 | 주요 운영 parameter | 적용 조건 | 대표 failure/대응 |
|---|---|---|---|---|---|
| scan perception | `/scan`, TF, map, global path → tracked obstacle layers | `ObstacleDetectorNode::scanCallback()` | detector clustering/wall/tracker YAML | scan·TF·map·CLCS가 유효 | TF/geometry 불능이면 해당 scan에서 정상 obstacle 갱신 불가; 세부 정책은 detector가 소유 |
| obstacle 수신/검증 | `/confirmed_static_obs` → accepted latest snapshot | `acceptObstacles()` | frame=`map`, stale 0.75 s | finite Frenet bounds이며 stamp가 수용 가능 | malformed non-empty를 clear로 바꾸지 않고 마지막 valid snapshot 유지 |
| ego Frenet | Cartesian odom + global path → `s,d,v_s,v_d` | `ClcsFrenetConverter::convertTracked()` | CLCS projection window/closed loop | global reference와 pose 존재 | projection 실패·odom age 초과 시 planning not-ready/hold |
| obstacle filter/cluster | snapshot + ego `s` → inflated visible cluster | `expandVisibleObstacles()`, `nearestCluster()` | lookahead 15.0 m, gap 0.8 m, half-width 0.15 m, margin 0.08 m | global `d=0`을 막는 forward obstacle 존재 | blocking cluster가 없으면 global/clear lifecycle; side domain이 없으면 후보 부재 |
| candidate generation | context + 좌/우 corridor → 최대 24개 `d(s)` path | `P3ShadowEvaluator::run()` | target 5 samples, entry/exit arrays, P3 cap 24 | `TEST_ACTIVE`, side corridor 연결 가능 | M0/M1 root/domain/corridor 실패는 해당 후보 reject; 양쪽 전멸 시 fallback |
| geometry/speed | Cartesian candidate → `psi,kappa,vx,ax` waypoint | geometry recomputation, `applyAvoidanceVelocityLimit()` | steering/wheelbase, accel/decel/lat-accel table, response delay 0.15 s | path sample과 reference speed가 유효 | analytic derivative 실패는 3-point fallback; 늦은 제동은 diagnostic/fallback 사다리 |
| hard feasibility | shaped candidate → valid/reject reason | `validateCandidate()` | slope 0.8, curvature 1.316... 1/m와 조향 유도 cap, curvature-rate 20 1/m², wall 0.04 m | 최소 path point와 forward suffix 존재 | wall/obstacle/geometry 위반 즉시 reject |
| selection/lifecycle | hard-valid candidates → selected/committed result | `measureCandidate()`, `betterCandidateRank()`, `P3ManeuverLifecycle` | lexicographic key, stabilization/commitment thresholds | valid candidate가 하나 이상 | 전멸하면 margin slow pass 또는 latched safe-stop; 기존 suffix가 valid면 새 후보보다 유지 |
| publish/mux/control | `/avoid_waypoints` → `/local_waypoints` → `/drive_autonomous` | `publishResult()`, `select_waypoints()`, controller cycle | FSM freshness/transition, local path timeout 0.3 s, controller 20 ms | AVOID에서 최신 non-empty avoid path, fresh odom | FSM은 stale odom에서 local publish 중단; controller는 stale local에 global fallback하므로 결합 동작은 runtime 확인 필요 |

표의 parameter는 기본 YAML/기본 launch 정적 값이다. 실제 대회 실행에서 override되었는지는 확인되지 않았으며, 전체 목록과 source 위치는 [parameters.md](../03_current_planner/parameters.md)에 분리했다.

## 단계 1: 입력 데이터의 생성

### 글로벌 기준선과 자차 Frenet 상태

`/global_waypoints`는 단순 시각화 선이 아니다. 각 `Wpnt`가 `(s, d, x, y, psi, kappa, vx, ax, d_left, d_right)`를 제공한다. 메시지 정의는 [`Wpnt.msg:1`](../../f110_msgs/msg/Wpnt.msg#L1)에 있다.

자차의 map pose는 `frenet_odom_node`가 CommonRoad CLCS에 투영한다.

1. `/pf/pose/odom`과 `/global_waypoints`를 구독한다: [`frenet_odom_node.cpp:85`](../../src/global_planning/src/frenet_odom_node.cpp#L85).
2. 위치를 CLCS에 투영하고, 연속 추적 모드에서는 이전 `s` 주변 창만 탐색한다: [`clcs_frenet_converter.cpp:203`](../../src/global_planning/src/clcs_frenet_converter.cpp#L203).
3. `pose.position.x=s`, `pose.position.y=d`, `twist.linear.x=v_s`, `twist.linear.y=v_d`로 `/car_state/frenet/odom`을 발행한다: [`frenet_odom_node.cpp:335`](../../src/global_planning/src/frenet_odom_node.cpp#L335).

이 표현은 `nav_msgs/Odometry` 타입을 재사용한 프로젝트 내부 계약이다. 필드 이름만 보고 Cartesian odometry로 해석하면 안 된다.

### 장애물 perception

`obstacle_detector`는 `/scan`을 기준으로 구동된다.

1. LaserScan, 글로벌 경로, 참조 맵, ego odom을 구독한다: [`obstacle_detector_node.cpp:78`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L78).
2. map-frame scan point를 clustering하고 맵 벽을 제거한다: [`obstacle_detector_node.cpp:882`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L882), [`obstacle_detector_node.cpp:937`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L937).
3. 각 cluster의 map-frame AABB를 CLCS에 투영한다: [`obstacle_detector_node.cpp:1621`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1621), [`aabb_frenet_projector.cpp:198`](../../src/obstacle_detector/src/aabb_frenet_projector.cpp#L198).
4. tracker가 `s,d,vs,vd`와 공분산을 관리하고 정적/동적을 분류한다: [`obstacle_detector_node.cpp:1711`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1711).
5. confirmed static만 `/confirmed_static_obs`, 모든 publishable static은 `/static_obs`, 동적 최근접 한 대는 `/opp_obs`로 매 scan 발행한다: [`obstacle_detector_node.cpp:1742`](../../src/obstacle_detector/src/obstacle_detector_node.cpp#L1742).

## 단계 2: local_planner_node의 입력 계약

`local_planner_node`의 실제 인터페이스는 [`local_planner_node.cpp:798`](../../src/local_planning/src/local_planner_node.cpp#L798)에 있다.

| 입력 | 타입 | 현재 역할 |
|---|---|---|
| `/global_waypoints` | `f110_msgs/WpntArray` | 기준선, 경계, 기준 속도/곡률 |
| `/confirmed_static_obs` | `f110_msgs/ObstacleArray` | 회피 기하의 authoritative 입력 |
| `/static_obs` | `f110_msgs/ObstacleArray` | confirmed 전 조기 감속 힌트만 제공 |
| `/car_state/frenet/odom` | `nav_msgs/Odometry` | ego `s,d,v_s` |
| `/state` | `f110_msgs/StateMachine` | handoff/commitment 운영 상태 |

운영 토픽 값은 [`local_planning.yaml:814`](../../src/local_planning/config/local_planning.yaml#L814)에 있다.

중요한 구분:

- `/confirmed_static_obs`가 후보 생성과 충돌 검사를 지배한다.
- `/static_obs` 콜백은 `id,s_start,s_end,d_left,d_right`만 복사하며 “속도 힌트에만 쓴다”: [`local_planner_node.cpp:1083`](../../src/local_planning/src/local_planner_node.cpp#L1083).
- `/opp_obs`는 이 로컬 플래너가 구독하지 않는다. 동적 상대차는 `state_machine`의 CRUISE 전환과 `cruise_controller` 쪽 책임이다.

장애물 수신은 계획 계산과 별도 callback group에서 최신 1건만 저장한다. 세 그룹을 동시에 처리하기 위해 executor thread도 3개다: [`local_planner_main.cpp:21`](../../src/local_planning/src/local_planner_main.cpp#L21). 이는 오래 걸린 계획이 장애물 수신을 막아 가짜 stale을 만드는 것을 줄이는 구조다.

## 단계 3: 스냅샷과 freshness

계획 timer는 기본 25 ms 주기다: [`local_planning.yaml:686`](../../src/local_planning/config/local_planning.yaml#L686).

계획 시작 시 최신 ingress를 drain하고, 다음 항목을 한 스냅샷으로 잡는다.

- ego `s,d,speed`
- confirmed obstacle 배열
- obstacle source stamp/sequence/epoch
- global reference generation
- safe-stop authority

구현은 [`local_planner_node.cpp:3017`](../../src/local_planning/src/local_planner_node.cpp#L3017)이다. obstacle receipt age가 0.75 s, odometry receipt age가 5.0 s를 넘으면 ready가 아니다: [`local_planner_node.cpp:3056`](../../src/local_planning/src/local_planner_node.cpp#L3056), [`local_planning.yaml:664`](../../src/local_planning/config/local_planning.yaml#L664).

잘못된 frame이나 모든 항목이 invalid인 비어 있지 않은 배열은 “track clear”로 보지 않고 마지막 valid snapshot을 유지한다. 명시적인 empty 배열만 유효한 clear 관측이다: [`local_planner_node.cpp:959`](../../src/local_planning/src/local_planner_node.cpp#L959).

## 단계 4: 장애물 전처리와 planning context

`RacelineSplinePlanner::expandVisibleObstacles()`가 detector 경계를 ego-relative closed-loop 거리로 바꾼다: [`raceline_spline_planner.cpp:729`](../../src/local_planning/src/raceline_spline_planner.cpp#L729).

- `s_start/s_end`에서 짧은 방향 span을 고른다.
- longitudinal padding을 더한다.
- raw lateral AABB에 차체 반폭, safety margin, 선택적 tracking reserve를 더한다.
- lookahead 뒤/자차 뒤 장애물을 제거한다.
- forward start 순으로 정렬한다.

그 다음 `nearestCluster()`가 global line의 `d=0`을 막는 첫 장애물과, `obstacle_cluster_gap_m` 이내의 뒤 장애물을 한 기동으로 묶는다: [`raceline_spline_planner.cpp:908`](../../src/local_planning/src/raceline_spline_planner.cpp#L908).

현재 주요 값은 lookahead 15.0 m, cluster gap 0.8 m, vehicle 0.56 x 0.30 m, safety margin 0.08 m다: [`local_planning.yaml:6`](../../src/local_planning/config/local_planning.yaml#L6), [`local_planning.yaml:24`](../../src/local_planning/config/local_planning.yaml#L24), [`local_planning.yaml:40`](../../src/local_planning/config/local_planning.yaml#L40).

## 단계 5: P3 후보 생성

`P3ShadowEvaluator::run()`이 좌/우를 모두 평가한다: [`p3_shadow.cpp:75`](../../src/local_planning/src/p3_shadow.cpp#L75).

1. `buildP3ShadowPlanningContext()`가 visible obstacle, nearest cluster, 좌/우 target domain을 만든다: [`raceline_spline_planner.cpp:1109`](../../src/local_planning/src/raceline_spline_planner.cpp#L1109).
2. track interval에서 장애물 inflated interval을 빼 corridor를 만든다: [`p3_shadow.cpp:810`](../../src/local_planning/src/p3_shadow.cpp#L810).
3. corridor의 왼쪽/오른쪽 연결 branch를 선택한다.
4. M0 후보를 먼저 만들고, 필요하면 M0 extension과 M1 analytic closure를 탐색한다.
5. 전체 후보 수는 최대 24개다: [`p3_shadow.cpp:310`](../../src/local_planning/src/p3_shadow.cpp#L310).
6. 각 후보는 다섯 station과 `[ego_d, target, middle, target, 0]`의 offset을 사용한다: [`p3_shadow.cpp:544`](../../src/local_planning/src/p3_shadow.cpp#L544), [`p3_shadow.cpp:1064`](../../src/local_planning/src/p3_shadow.cpp#L1064).
7. 인접 knot 사이를 quintic Hermite segment로 잇고, knot의 `d,d',d''`를 공유해 C2 profile을 만든다: [`p3_shadow.cpp:613`](../../src/local_planning/src/p3_shadow.cpp#L613).
8. ordered global waypoint마다 `d(s)`를 평가하고 `x=x_ref-d sin(psi_ref)`, `y=y_ref+d cos(psi_ref)`로 옮긴다: [`p3_shadow.cpp:1083`](../../src/local_planning/src/p3_shadow.cpp#L1083).

즉 “새 Cartesian 곡선을 자유롭게 최적화”하지 않는다. 글로벌 waypoint 순서는 보존하고 그 normal 방향으로만 이동한다.

## 단계 6: 기하 재계산과 속도 성형

후보의 `x,y`가 결정된 뒤 다음 순서가 적용된다.

1. local cubic analytic derivative로 `psi,kappa` 재계산; 실패하면 3점 곡률 방식으로 전체 fallback: [`raceline_spline_planner.cpp:2663`](../../src/local_planning/src/raceline_spline_planner.cpp#L2663).
2. `v^2 |kappa| <= a_lat,max(v)`를 만족하도록 곡률 속도 cap: [`raceline_spline_planner.cpp:416`](../../src/local_planning/src/raceline_spline_planner.cpp#L416).
3. 장애물 옆 gap reserve와 confirmed obstacle critical speed envelope 적용: [`raceline_spline_planner.cpp:2344`](../../src/local_planning/src/raceline_spline_planner.cpp#L2344).
4. 응답지연 거리와 제동식을 이용한 접근 ramp: [`raceline_spline_planner.cpp:2486`](../../src/local_planning/src/raceline_spline_planner.cpp#L2486).
5. ego 실측속도에서 시작하는 forward acceleration pass와 backward deceleration pass: [`raceline_spline_planner.cpp:2590`](../../src/local_planning/src/raceline_spline_planner.cpp#L2590).
6. `ax=(v_next^2-v_i^2)/(2 ds)` 계산: [`raceline_spline_planner.cpp:2781`](../../src/local_planning/src/raceline_spline_planner.cpp#L2781).

## 단계 7: hard validation과 선택

각 후보는 생성 직후 exact validator를 통과해야 한다: [`p3_shadow.cpp:1103`](../../src/local_planning/src/p3_shadow.cpp#L1103).

hard reject 항목은 다음과 같다.

- ego에서 path entry로의 과도한 불연속
- 최소 waypoint 수와 전방 path 부재
- waypoint center의 track bound 이탈
- 회전 직사각형 footprint의 벽 침범
- inflated obstacle box와 `d(s)` 교차
- ordered `s` 역행
- `|Delta d|/Delta s` 초과
- `|Delta kappa|/Delta s` 초과
- 좌/우 조향각에서 유도한 방향별 curvature 한계 초과

구현은 [`raceline_spline_planner.cpp:2805`](../../src/local_planning/src/raceline_spline_planner.cpp#L2805)다.

hard-valid 후보만 lexicographic ranking한다. weighted-sum cost가 아니다. 우선순위는 다음과 같다: 다음 장애물과 exit 충돌 없음, ego braking feasibility, 낮은 velocity loss, 큰 normalized safety slack, 작은 평균 `|d|`, 생성 순서. 근거는 [`candidate_rank.hpp:49`](../../src/local_planning/include/local_planning/candidate_rank.hpp#L49)다.

## 단계 8: commitment, continuation, safe-stop

P3가 새 경로를 선택하면 conservative guard와 raw obstacle에 대해 각각 exact validation하고 immutable record로 저장한다: [`p3_maneuver_lifecycle.cpp:158`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L158).

다음 callback은 새 후보를 다시 만들기 전에 기존 suffix를 먼저 검증한다. hard-valid이면 그대로 유지해 perception jitter가 매 프레임 경로 형상을 바꾸지 않게 한다: [`local_planner_node.cpp:3216`](../../src/local_planning/src/local_planner_node.cpp#L3216).

P3 경로가 없으면 기존 운영 사다리로 간다.

- margin-only면 저속 global-line pass
- 아니면 obstacle 전 safe-stop
- safe-stop은 latch/release lifecycle로 유지
- 회피 완료 후 `raceline_global_handoff` 닫힌 global loop를 발행하여 FSM이 GLOBAL 복귀를 확인

`runP0PlanningCycle()`이라는 함수명은 역사적 이름이다. 그 안의 `planner_.plan()` 역시 현재는 P3만 생성한다: [`local_planner_node.cpp:3812`](../../src/local_planning/src/local_planner_node.cpp#L3812), [`local_planner_node.cpp:4080`](../../src/local_planning/src/local_planner_node.cpp#L4080).

## 단계 9: state machine과 controller

플래너는 `/local_waypoints`를 직접 발행하지 않는다. `/avoid_waypoints`를 `OTWpntArray`로 발행한다: [`local_planner_node.cpp:4371`](../../src/local_planning/src/local_planner_node.cpp#L4371).

`state_machine_node`가 다음을 수행한다.

- GLOBAL/CRUISE: global path를 `/local_waypoints`로 그대로 발행
- AVOID: 최신 non-empty avoid path를 `WpntArray`로 변환해 발행
- AVOID에서 empty가 오면 마지막 non-empty path를 유지
- `raceline_global_handoff`와 ego tail 정합이 일정 시간 성립할 때 GLOBAL 복귀

근거: [`state_machine_node.cpp:730`](../../src/state_machine/src/state_machine_node.cpp#L730), [`state_machine_node.cpp:802`](../../src/state_machine/src/state_machine_node.cpp#L802).

현재 소스에서는 별도 `wpnt_publisher` 노드가 아니라 통합 `state_machine_node`가 sole normal `/local_waypoints` publisher다. 실행 지침의 별도 터미널 설명과 현재 구현이 어긋날 수 있으므로 실제 graph는 `ros2 topic info -v /local_waypoints`로 확인해야 한다.

`control_map_node`는 fresh local path를 우선 사용하고 0.3 s가 지나면 global path로 fallback한다: [`control_map_node.cpp:500`](../../src/f1tenth_control/control_code/control_map_node.cpp#L500), [`control_map_node.cpp:1379`](../../src/f1tenth_control/control_code/control_map_node.cpp#L1379).

제어는 20 ms 주기로 다음을 수행한다.

- 경로 위 최근접점과 L1 target을 찾음
- `a_lat = 2 v^2 sin(eta) / L1` 계산
- path curvature FF와 L1 feedback 결합
- bicycle inverse model과 understeer gradient로 steering 계산
- steering rate/좌우 각 clamp
- waypoint speed, curvature preview, cruise cap, speed ramp를 조합
- `/drive_autonomous` 발행

근거: [`control_map_node.cpp:716`](../../src/f1tenth_control/control_code/control_map_node.cpp#L716), [`control_map_node.cpp:1523`](../../src/f1tenth_control/control_code/control_map_node.cpp#L1523), [`control_map_node.cpp:1566`](../../src/f1tenth_control/control_code/control_map_node.cpp#L1566).

## 정상 흐름과 실패 흐름

### 정상 회피

```text
confirmed static obstacle
-> blocking cluster
-> 좌/우 corridor 중 hard-valid candidate
-> lexicographic best
-> immutable commitment
-> /avoid_waypoints
-> FSM AVOID
-> /local_waypoints
-> controller
-> obstacle rear 통과/merge
-> global handoff
-> FSM GLOBAL
```

### 후보가 전멸할 때

```text
NO_VALID_SIDE_DOMAIN 또는 hard rejection
-> margin-only인지 판정
-> 아니면 safe-stop geometry/profile
-> safe-stop latch
-> 연속 selectable avoidance 또는 명시적 clear 조건으로 release
```

### perception/odometry가 stale일 때

- confirmed obstacle stale은 obstacle 소멸로 해석하지 않는다.
- odometry stale은 현재 위치에서 안전 hold 경로를 만들 수 있다.
- state machine은 Frenet odom stale이면 `/local_waypoints` 발행을 중단한다: [`state_machine_node.cpp:773`](../../src/state_machine/src/state_machine_node.cpp#L773).
- controller는 자체 odom watchdog과 local freshness fallback을 별도로 가진다.

## 이 문서에서 확인하지 못한 것

- 실제 실행 중 어떤 launch override와 ROS parameter가 유효했는지
- 실차에서 detector→planner→controller end-to-end latency 분포
- friction coefficient, tire saturation, actuator lag의 현재 실측값
- 현재 branch가 실제 대회 rosbag과 동일한 binary였는지
- 모든 safety margin의 통계적 coverage

이 항목들은 소스만으로 확정할 수 없으며, `ros2 param dump`, topic QoS/graph, 새 rosbag, actuator/vehicle measurement가 필요하다.
