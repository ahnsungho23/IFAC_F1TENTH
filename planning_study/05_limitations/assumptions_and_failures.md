# 가정, 한계, 실패 가능성

## 1. 증거 수준

이 문서는 현재 커밋의 정적 코드 분석이다. “코드가 검사한다”와 “실차에서 항상 안전했다”는 같은 명제가 아니다. 다음 평가는 source-level capability와 gap을 정리한 것이며 runtime 보증이 아니다.

### 가정별 근거와 깨졌을 때의 현상

아래 가능성은 실측 확률이 아니라 F1TENTH 환경에서 그 가정이 깨질 수 있는 경로가 존재하는지에 대한 정성 평가다.

| 암묵적 가정 | 코드 근거 | 깨질 가능성 | 깨졌을 때 예상 현상 |
|---|---|---|---|
| detector의 obstacle face가 충분히 정확하다 | planner는 `s_start,s_end,d_right,d_left`를 직접 팽창하며 Cartesian point cloud를 재검증하지 않는다: [`raceline_spline_planner.cpp:730`](../../src/local_planning/src/raceline_spline_planner.cpp#L730) | 중~높음: range, sparse beam, roll/TF, AABB face drift 영향 | 실제 clearance 부족 또는 반대로 false no-corridor/safe-stop |
| confirmed obstacle은 계획 중 정적이다 | P3 collision은 `vs,vd` 없이 spatial envelope만 검사한다: [`raceline_spline_planner.cpp:2974`](../../src/local_planning/src/raceline_spline_planner.cpp#L2974) | 정적 코스 장애물은 낮음, 오분류 상대차는 높음 | obstacle의 미래 위치와 경로가 교차하는데 valid로 선택 |
| covariance 0-inflation에서도 fixed margin 0.08 m가 오차를 덮는다 | guard는 구현돼 있으나 sigma/min/max inflation 운영값이 0이다: [`obstacle_guard.cpp:64`](../../src/local_planning/src/obstacle_guard.cpp#L64), [`local_planning.yaml:703`](../../src/local_planning/config/local_planning.yaml#L703) | 중~높음: calibration evidence는 source에서 확인되지 않음 | 좁은 gap의 risk가 상황에 따라 달라지고 predicted slack이 실제 risk와 불일치 |
| 차량은 계획 경로를 margin 안에서 추종한다 | collision validator는 planned waypoint/footprint를 검사하며 closed-loop tracking rollout은 없다: [`raceline_spline_planner.cpp:2805`](../../src/local_planning/src/raceline_spline_planner.cpp#L2805) | 고속·급곡률·localization jump에서 높아짐 | planned path는 clear하지만 실제 차체가 벽/장애물 쪽으로 이탈 |
| curvature와 speed cap이면 제어 가능성이 충분하다 | `v^2|kappa|` cap과 curvature/rate gate는 있으나 steering-rate 위반은 diagnostic이다: [`raceline_spline_planner.cpp:1587`](../../src/local_planning/src/raceline_spline_planner.cpp#L1587) | 고속 transient/방향 전환에서 중간 이상 | steering saturation, phase lag, tracking error 증가 |
| 종·횡 타이어 여력은 서로 독립적으로 다뤄도 된다 | accel/decel shaping과 lateral cap은 분리되고 friction ellipse는 없다 | braking-in-corner에서 중간 이상 | 개별 제한은 만족해도 combined tire force 포화, understeer/slide |
| response delay 0.15 s가 충분한 대표값이다 | confirmed/raw speed shaping에 고정 delay가 있고 end-to-end stamp propagation은 없다: [`raceline_spline_planner.cpp:2486`](../../src/local_planning/src/raceline_spline_planner.cpp#L2486) | compute/DDS/actuator jitter가 있으면 높음 | braking/steering 시작이 예상보다 늦어 deficit와 minimum clearance 악화 |
| confirmed detection은 회피 가능한 거리에서 온다 | geometry는 confirmed만 사용하고 provisional `/static_obs`는 속도 cap뿐이다: [`local_planner_node.cpp:1085`](../../src/local_planning/src/local_planner_node.cpp#L1085) | 원거리 sparse scan/confirmation delay에서 중간 이상 | 후보가 생길 때 이미 entry/curvature/braking gate가 전멸하여 safe-stop |
| global path와 Frenet frame이 정확하고 일관된다 | candidate는 global waypoint의 `s,psi,d_left,d_right`를 보존·신뢰한다: [`p3_shadow.cpp:1083`](../../src/local_planning/src/p3_shadow.cpp#L1083) | map/waypoint 교체, self-near branch, pose bias에서 중간 | 잘못된 normal 방향, wall width, obstacle branch로 계획 |
| `p=r+dn`이 regular하다 | `1-kappa_ref d` singularity를 hard gate하는 코드는 확인되지 않는다 | 급곡률·큰 offset에서 낮음~중간 | Frenet mapping fold/heading 급변, Cartesian curvature spike |
| waypoint 표본 사이도 안전하다 | obstacle gate는 discrete path point의 Frenet interval을 검사한다 | coarse sampling/작은 장애물에서 중간 | 두 waypoint 사이의 swept footprint가 obstacle과 교차 |
| obstacle ID와 clear/stale semantics가 신뢰 가능하다 | lifecycle은 same ID와 accepted empty/last-valid snapshot을 구분한다: [`local_planner_node.cpp:959`](../../src/local_planning/src/local_planner_node.cpp#L959) | merge/occlusion/upstream failure에서 중간 | commitment 흔들림, false release 또는 false obstacle 장기 유지 |
| planner/controller model·freshness 정책이 동기화돼 있다 | 공유 조향 parameter 계약은 있으나 FSM publish 중단과 controller global fallback은 별도다 | 배포/override drift에서 중간 | planner가 의도한 hold 대신 global path가 선택되거나 모델 한계 불일치 |

## 2. 구조적 가정

### 기준선 순서가 진실이다

global waypoint `s_m`은 strictly increasing이어야 하고 closed loop track length를 대표해야 한다: [`raceline_spline_planner.cpp:586`](../../src/local_planning/src/raceline_spline_planner.cpp#L586). 후보는 이 순서를 바꾸지 않는다.

실패 가능성:

- duplicate/reversed `s`
- CLCS geometric s와 stored `s_m` 불일치
- stale `psi_rad`, `d_left/d_right`, `kappa` metadata

### 안전 경로는 `d(s)`로 표현 가능하다

planner는 기준선 normal 방향 offset만 만든다. 후진, 정지 후 backing, reference branch를 벗어나는 자유 경로, 시간에 따른 weaving은 표현하지 못한다.

### 장애물은 정적 Frenet envelope다

P3는 confirmed static AABB를 사용한다. `vs,vd`가 메시지에 있어도 정적 collision prediction에는 쓰지 않는다.

### waypoint sampling이 충분히 조밀하다

hard validator는 discrete waypoint에서 검사한다. waypoint 사이 continuous swept collision은 직접 증명하지 않는다.

## 3. perception 관련 한계

- map/scan/pose alignment가 틀리면 detector AABB와 track wall filter가 함께 틀릴 수 있다.
- confirmed static 승격에는 지연이 있다. raw slowdown이 이를 일부 완화하지만 path geometry는 기다린다.
- empty confirmed array는 유효한 clear 관측이다. upstream processing failure가 empty로 발행되면 semantics가 깨진다.
- tracker ID가 바뀌면 stabilization/commitment/lifecycle memory가 다른 물체로 볼 수 있다.
- covariance는 전달되지만 current sigma inflation이 0이라 decision margin에 반영되지 않는다.
- 명시적인 detection confidence나 false-positive probability는 없다.

## 4. Frenet/geometry 관련 한계

- `1-kappa_ref d` singularity를 직접 gate하지 않는다.
- ego CLCS, obstacle CLCS, planner waypoint-normal transform이 완전히 같은 continuous geometry 연산은 아니다.
- AABB center tangent projection은 branch-locked face distance로 보완되지만 큰 box/고곡률의 exact Frenet image는 아니다.
- obstacle collision은 vehicle center `d`와 inflated Frenet interval 검사다. obstacle에 대한 exact rotated rectangle-vs-AABB polygon test가 아니다.
- track wall은 회전 footprint를 보지만 map occupancy와 직접 충돌 검사하지 않고 waypoint의 `d_left/d_right` 경계를 신뢰한다.

## 5. 후보 표현 한계

- 다섯 knot, 네 quintic segment family 안에서만 탐색한다.
- 전체 후보 cap은 24다.
- connected corridor branch를 sampled stations에서 판정한다.
- M0/M1 proposal이 존재하지 않는 feasible curve를 만들어내지는 못한다.
- maximum exit length가 0으로 비활성이라 긴 exit가 가능하며, 다음 장애물 접촉은 hard reject보다 rank demotion이다.
- generator order가 최종 tie-break이므로 완전 대칭 상황에서 숨은 bias가 있다.

## 6. collision/feasibility 한계

- path point 사이 continuous collision 없음
- space-time obstacle collision 없음
- hard steering-rate 없음
- hard longitudinal acceleration/deceleration reject 없음; speed shaping과 final diagnostics로 처리
- lateral acceleration도 geometry candidate hard reject보다 speed cap으로 처리
- braking infeasible 상태에서는 safest available low profile을 유지하지만 물리적 정지를 보장하지 못함
- safe-stop escape verification도 forward planning family 안에서만 escape를 찾음; reverse가 필요하면 풀지 못함

## 7. dynamics 모델 한계

현재 반영:

- kinematic curvature/steering geometry
- speed-dependent lateral acceleration
- speed-dependent longitudinal accel/decel
- understeer gradient 기반 steering model
- 고정 response delay

현재 빠짐:

- nonlinear tire force, friction ellipse
- steering actuator dynamics/lag의 planner rollout
- yaw rate/slip angle state
- load transfer와 braking-cornering coupling
- time-indexed trajectory and controller closed-loop simulation
- road friction uncertainty

따라서 “동역학을 전혀 고려하지 않는다”도 틀리고 “동역학적으로 feasible한 trajectory를 보장한다”도 틀리다.

## 8. cost/policy 한계

weighted sum이 아닌 lexicographic rank이므로 상위 항의 미세한 차이가 하위 안전여유의 큰 차이를 덮을 수 있다. 다만 최소 안전은 hard gate가 맡는다는 설계다.

현재 soft ranking에 jerk, path length, time, probability of collision, tracking-error prediction은 없다.

## 9. lifecycle/state failure 가능성

- active suffix를 유지하는 것은 jitter를 줄이지만 guard가 부정확하면 오래 유지할 수 있다.
- stale confirmed perception은 last snapshot을 유지하므로 false positive가 정지/회피를 지속시킬 수 있다.
- safe-stop release는 empty array 하나가 아니라 lifecycle 조건을 요구하지만, 장시간 sensor outage 정책은 fixed timeout/creep에 의존한다.
- FSM AVOID는 empty path에서 last non-empty를 유지한다. planner death/freshness 정책과 결합하면 stale geometry가 남을 수 있다.
- state machine Frenet stale 시 `/local_waypoints`를 중단하고 controller는 0.3 s 뒤 global fallback이 가능하다. 두 노드의 정책 결합을 runtime에서 검증해야 한다.
- 현재 source에서는 state_machine이 `/local_waypoints`를 직접 발행한다. 별도 `wpnt_publisher` 실행 문서는 현재 topology와 불일치할 수 있다.

## 10. parameter/document drift

YAML에 오래된 숫자 주석이 남아 있다.

- `safety_margin_m` 실제값 0.08인데 인접 과거 주석은 0.00 계산을 포함한다.
- `obstacle_longitudinal_padding_m` 실제값 0.0인데 safe-stop 주석 일부는 0.4149925를 말한다.

정적 분석/실험 자동화는 설명문 숫자를 파싱하지 말고 actual key/value와 runtime dump를 사용해야 한다.

## 11. 대표 실패 시나리오

### A. 늦은 confirmed detection

```text
raw obstacle appears
-> speed hint only
-> confirmed promotion delay
-> geometry plan starts too close
-> all candidates braking-infeasible or curvature-invalid
-> safe-stop / residual collision risk
```

필요 evidence: raw/confirmed timestamps, ego speed/distance, first avoidance path, drive response.

### B. 급커브의 branch/footprint mismatch

```text
CLCS center projection valid
-> large AABB tangent approximation
-> planner Frenet interval appears clear
-> actual Cartesian corner closer
-> waypoint-sampled collision miss
```

필요 evidence: exact polygon replay와 high-resolution swept collision.

### C. dynamics saturation

```text
geometry hard-valid
-> speed shaping predicts feasible
-> combined braking+cornering exceeds tire/steering actuator
-> tracking error grows beyond fixed margin
-> wall/obstacle contact
```

필요 evidence: steering target/actual, yaw rate, lateral accel, wheel speed, path tracking error.

### D. false confirmed obstacle/stale hold

```text
false static track confirmed
-> commitment or safe-stop
-> perception stale retains snapshot
-> empty clear evidence unavailable
-> prolonged stop/avoidance
```

필요 evidence: scan/raw/confirmed timeline and lifecycle reasons.

## 12. 불확실하거나 확인 불가능한 부분

- 현재 commit의 binary가 대회 실차에 그대로 배포되었는지
- 실차 runtime parameter overrides
- detector covariance calibration quality
- map boundary error distribution
- actual tire friction and actuator bandwidth
- end-to-end latency distribution
- 현재 margins의 target confidence level
- candidate cap이 실전에서 얼마나 자주 포화되는지

이것들은 source만으로 답할 수 없다.

## 13. 안전하게 개선을 평가하는 최소 기준

- 기존 hard-valid invariant 유지
- 동일 raw input replay에서 deterministic comparison
- exact Cartesian swept-collision oracle와 비교
- no-obstacle lap 성능과 obstacle scenario를 분리
- planner output뿐 아니라 controller command/vehicle response까지 측정
- false positive stop time, missed obstacle, collision clearance, lap time을 함께 보고
- runtime-effective parameters와 commit hash를 bag metadata에 남김
