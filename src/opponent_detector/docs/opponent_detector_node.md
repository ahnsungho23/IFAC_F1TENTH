# opponent_detector_node 동작 문서

`opponent_detector` 패키지의 유일한 런타임 노드(`opponent_detector_node`, C++)의 상세 동작·실행·튜닝 문서.

## 1. 무엇을 하나

에고 차량의 **2D LiDAR를 주 계측으로** 트랙 위 장애물과 움직이는 상대차를 찾아 추적하고, 그
위치·속도를 Frenet 좌표로 발행한다. IMU/odom은 물체를 직접 검출하지 않고 회전 중 LiDAR 스캔의
시간 왜곡을 보정하는 데만 선택적으로 쓴다. 논문 `2603.27207v1.pdf`는 depth 카메라(YOLO)로 "어느 LiDAR 클러스터가
상대차인지"를 골라내지만, 이 저장소엔 카메라가 없다. 대신 **raceline(Frenet) 프레임에서의 상대속도**로
상대차를 골라낸다: 벽·정적 물체는 트랙 프레임에서 제자리, 상대차만 `s`(와 `d`)를 따라 움직인다.

## 2. 파이프라인 (스캔 콜백 1회)

```
/scan + IMU/odom ──빔별 deskew + 선택적 노이즈 필터──▶ 보정 점군
      ──ABD 군집화 + 근접 파편 병합──▶ 클러스터들
      ──TF(map←laser)──▶ map 프레임 점군
      ──박스 피팅──▶ (중심 x,y, 크기)
      ──지도필터 + 코너 경계──▶ 동적 후보만 남김
      ──Frenet 투영──▶ (s, d)
      ──Mahalanobis 연관 + 적응형 등속 칼만──▶ 트랙 [s, vs, d, vd]
      ──속도 기반 정적/동적 분류──▶ is_static
      ──▶ /perception/obstacles (ObstacleArray)
      ──▶ /perception/static_obstacles/cartesian (정적 전용 x, y, s, d, radius)
      ──▶ /proj_opponent_trajectory (ProjOppTraj, 동적 상대차)
      ──추월 상태머신(Idle→Committed→Cooldown)──▶ /overtake_waypoints (OTWpntArray)
      ──▶ /perception/obstacles/markers (RViz: 상대차 + 추월 라인 초록)

추월 라인은 다운스트림에서:
  /overtake_waypoints ─▶ wpnt_publisher ─▶ /local_waypoints ─▶ new_map_con(추종)
  (Committed 동안만 발행; 완료/중단 시 빈 OT 1회 → global 폴백 → 발행 중단)
```

1. **스캔 deskew**: `LaserScan.header.stamp`를 첫 빔 시각으로 보고 각 빔의
   `time_increment`만큼 누적된 회전을 IMU `angular_velocity.z`로 되돌린다. `auto`는 신선한 IMU를
   우선하고 없으면 odom yaw rate를 쓴다. 둘 다 오래됐으면 보정 없이 처리한다. 현재 실차 VESC가
   deg/s로 관측되어 `imu_angular_scale=π/180`이며, rad/s 드라이버나 시뮬 IMU를 연결하면 `1.0`으로
   바꿔야 한다. 병진 deskew는 `deskew_translation_enable=false`가 기본이다.
2. **선택적 원시 노이즈 필터**: 중앙값 창과 정렬된 스캔 이웃 밀도 필터를 제공하지만 기본은 꺼져
   있다. 작은 원거리 장애물을 지우지 않는지 f1sim_C와 rosbag에서 확인한 뒤 켠다.
3. **Adaptive Breakpoint Detector + 파편 병합**: 인접 빔 점 간 거리가
   `r·sin(Δφ)/sin(λ−Δφ)+3σ`를 넘으면 클러스터를 끊는다. 가까운 파편의 AABB 간격이
   `cluster_merge_distance` 이내이고 병합 후에도 `max_obs_size` 이하일 때만 다시 합친다.
4. **TF 변환**: `lookupTransform(map_frame, scan.frame_id, stamp)`. 스캔의 `frame_id`를 그대로
   쓰므로 실차 `laser`·시뮬 `ego_racecar/laser` 구분이 자동. 실패 시 최신(Time0)로 폴백.
5. **박스 피팅**: 클러스터 중심(평균)과 AABB 대각 크기. `max_obs_size` 초과는 폐기. 정지 장애물도
   차 크기(≤0.5×0.5 m, 대각 0.707 m)라 크기로 상대차와 못 가르므로 `max_obs_size`는 0.8(=0.707 초과)로
   둬서 **정지 장애물이 검출되게** 하고, 정적/동적 구분은 속도로만 한다.
6. **필터(지도필터 + 속도 하이브리드의 "지도" 부분)**:
   - 전방 관측: 에고 `s` 기준 `-view_behind ≤ Δs ≤ max_viewing_distance`.
   - 코너 경계: `d`가 `[-(d_right-infl), +(d_left-infl)]` 밖이면 폐기. 경계 미설정 시
     `fallback_track_halfwidth` 사용.
   - 점유격자: 클러스터 점의 `map_point_reject_ratio` 이상이 `/map`의 점유셀에 있으면 폐기
     (알려진 정적 구조물). 상대차는 지도에 없어 살아남는다.
7. **Frenet 투영**: `global_planning`의 **CLCS 변환기**(`ClcsFrenetConverter`, vendored CommonRoad
   CLCS)로 클러스터 중심·에고를 `(s,d)`로 투영. `convert().valid`가 false(투영 도메인 밖=트랙에서 먼
   점)면 폐기. 트랙 경계(`d_left/d_right`)와 s-wrap은 CLCS가 안 주므로 경량 `FrenetProjector`를 그
   용도로만 유지. (파이썬 전용 `frenet_converter`는 C++ 링크 불가라 CLCS 라이브러리를 export해 링크.)
8. **연관 + 추적**: `assoc_gate`(동적은 ×`aggro_multi`)를 절대 안전거리로 유지하고, 그 안에서는
   예측 공분산과 측정 공분산을 합친 Mahalanobis 거리로 연관한다. 원거리·희소 클러스터·큰 yaw
   rate에는 측정 공분산을 키워 칼만 업데이트를 덜 신뢰한다. 미연관 트랙은 TTL 감소 후 삭제.
9. **분류**(`classifier_mode`) — 정적배경(=벽·정지 장애물이 공유하는 겉보기 속도) 기준:
   - 매 프레임 `static_ref_gate` 미만인 느린 트랙들의 평균속도를 **정적배경 기준속도**로 잡는다
     (map 프레임이라 ≈0, 지역화 드리프트만큼 어긋나면 그 값; 빠른 상대차는 기준에서 제외).
   - `velocity`(기본): **기준 대비 상대속도** `|v_track − v_ref|`가 `dyn_vel_enter` 초과가
     `dyn_min_frames` 지속되면 동적, `dyn_vel_exit` 미만이면 정적(히스테리시스). 상대`vs<vs_reset`면
     정적 강제(백스톱). → "벽과 같은 상대속도면 정적, 다르면 동적."
   - `std`: 최근 창의 `(s,d)` 표준편차로 판정(ForzaETH식, `min_std/max_std`).
   - `both`: 두 방식이 모두 동적일 때만 동적(보수적).
10. **발행·계측**: 확정 트랙(`hits≥min_hits_confirm`)을 `ObstacleArray`로 발행하고, 정적 트랙은
   `has_cartesian=true`인 `(x_center,y_center,s_center,d_center,radius)`로
   `/perception/static_obstacles/cartesian`에도 분리 발행한다. `radius`는 검출 AABB 전체를 감싸는
   원의 반지름으로, `0.5×0.5 m` 정사각형이면 `0.25√2 m`다.
   동적 상대차 1대를 골라
   `ProjOppPoint`로 누적해 `ProjOppTraj` 발행. 마커는 정적=파랑, 동적=빨강이며 raw/tracked 토픽을
   함께 보면 필터 효과를 비교할 수 있다. 초당 한 번 `DIAG perception` 로그에 빔 수, 노이즈 제거,
   클러스터 수, 단계별 기각, deskew 소스/yaw rate, 연관·생성·삭제 통계를 출력한다.

   Perception은 정적 장애물을 매 스캔 재검출해 `is_static=true`, Cartesian `(x,y)`, Frenet
   `(s,d)`, 최대 반지름 `radius`를 반복 발행한다.
   `ttl_static`은 순간적인 센서 누락만 연결하며, 정적 장애물의 장기 기억과 최종 회피 판단은 로컬
   플래너가 담당한다.
11. **추월 플래너(상태머신 + PCHIP spline)** — `OvertakePlanner`가 `Idle → Committed → Cooldown`
   상태머신으로 추월 기동의 시작/유지/종료를 결정한다. 상대차도 움직이므로 매 사이클 경로의 당위성을
   재판정한다.
   - **Commit 게이트(Idle→Committed, 전부 만족해야 함)**:
     1. 동적 상대차가 에고 전방 `[avoid_trigger_min_ds, avoid_trigger_max_ds]`에 있고
        **에고 경로를 실제로 막음**(`|opp_d| < ego_half+opp_half+avoid_block_margin`).
     2. **상대속도 충분**: `v_rel = max(에고 실측속도, 그 지점 raceline vx) − opp_vs ≥ ot_min_rel_vel`
        이고 예상 캐치 시간 `ds/v_rel ≤ ot_max_catch_time`. 못 따라잡으면 추종 유지.
     3. **급코너 없음**: 기동 전 구간(에고→병합점)에서 raceline `|κ| ≤ ot_max_kappa`.
     4. **여유 공간**: apex를 놓을 **예측 추월 지점**(`s_pass = ego_s + v_att·t_catch`; 상대차가
        움직이므로 현재 위치가 아님)의 좌/우 트랙 여유가 `2·ego_half + lateral_clearance` 이상.
   - **knot**: `(ego_s, ego_d)`(현 위치, 라인 밖이어도 OK) → `(s_pass−pass_clearance, d_apex)` →
     `(s_pass+pass_clearance, d_apex)`(추월 구간 apex 유지) → `(+post, 0)` → `(+1 m, 0)`을
     **형상보존 PCHIP spline**으로 잇는다 — 자연 3차와 달리 **overshoot 없음**. 경로는 추월 구간만
     덮는 **국소 경로**(전체 트랙 아님). `avoid_spline_resolution` 간격으로 샘플.
   - **벽 안전마진(apex fitting)**: 샘플을 사후에 잘라내지(clamp) 않는다 — 잘라내면 spline이 벽을
     타고 가는 평평한 구간이 생겨 min-curvature 성질이 깨진다. 대신 spline 전체가 모든 지점에서
     `d_left/right − avoid_boundary_margin − ego_half` 안에 들어올 때까지 apex를 반복 축소하고,
     축소된 apex가 상대차 최소 간격(`ego_half+opp_half`)을 못 지키면 그 사이드를 거부한다.
     발행되는 경로는 항상 **손대지 않은 매끈한 spline**이다.
   - **조향 제한(steering + curvature limit)**: spline의 횡기울기 `|dd/ds|`(라인 대비 헤딩
     오프셋의 tan)를 `ot_max_d_slope`로, **횡곡률 `|d²d/ds²|`(조향 각을 결정)**를
     `ot_max_path_kappa`로 함께 제한한다. 곡률 스파이크가 조향을 튀게 해 짧은 램프(추월 지점이
     가까운) 엣지케이스에서 차를 스핀시키므로, 진입/복귀 램프 길이를 두 한계
     (`1.5·A/slope`, `√(6·A/κ)`)의 최댓값으로 잡고 도달 가능한 apex를 그 길이로 캡한 뒤, 피팅된
     spline의 실제 최대 기울기·곡률을 다시 검사해 초과분만큼 apex를 축소한다. 이렇게 **spline
     구간의 과도한 조향**과 램프의 **급조향/스핀**을 막는다.
   - **런웨이 확장(runway extension)**: 추종 중 상대차 바로 뒤(간격 ≈ `ot_trail_gap`)에서 속도까지
     맞으면 예측 캐치 지점 `s_pass`가 너무 가까워, 기울기/곡률 제한 아래에서 클리어링 apex에 도달할
     램프가 **절대 안 나온다** — 예전엔 매 사이클 commit이 거부돼 **추종에 영원히 고착**됐다
     ("추월 기미 없음" 버그). 이제 apex 시작점이 최소 램프 길이(`ramp_len(amp_max)`)보다 가까우면
     그 지점까지 **밀어내고**, `ds_pass`/`t_apex1`을 밀어낸 지점 기준으로 다시 유도한다 — 에고가
     **먼저 옆으로 빠진 뒤** 상대차를 지나가는 계획이 된다(아래 간격 유지가 이를 안전하게 만든다).
   - **속도(곡률제한 + 전후 가속도 스무딩)**: 각 샘플의 안전 속도는
     `v_phys = √(a_cap/|κ_tot|)`(`κ_tot = raceline κ + spline d''(s)`, 곡률은
     3점 평균으로 노이즈 제거)로 하드 캡한다. 그립 용량 `a_cap`은
     `max(avoid_max_lat_accel, raceline vx²·|κ_r|)`(κ_r도 동일한 3점 평균 — spline이 곡률을 더하지
     않은 샘플에서 `v_phys == vr`이 정확히 성립) — **raceline 속도는 이미 검증된 값**이므로
     raceline이 실제 사용하는 횡가속을 그 지점의 그립 용량으로 신뢰한다. 덕분에 spline이 곡률을
     더하지 않은 구간에서는 절대 raceline보다 느려지지 않고, 곡률이 더해진 구간만
     `v ≈ vr·√(κ_r/κ_tot)`로 비례 감속한다(`avoid_max_lat_accel`은 직선 apex 구간의 하한 그립).
     하지만 **샘플별 캡만으로는 코너에서야 감속**해
     늦으므로(→ 벽 충돌), 이 목표 속도를 **전진·후진 종가속도 제한**(`ot_max_long_accel`)으로
     한 번 더 통과시켜 **타이트 구간 이전에 미리 제동**한다(오프라인 속도 생성기와 동일). 시작은
     현재 에고 실측속도로 앵커하고 병합 끝은 raceline 속도로 이어 붙여(`ot_speed_blend_s`)
     핸드오프에서 속도 지령이 튀지 않는다. **apex 전 간격 유지(pre-apex gap hold)**: 전진 패스에서
     아직 상대차를 횡으로 벗어나지 못한 샘플(`|d−opp_d| < 반폭합+0.5·lateral_clearance`)이고 예측
     종간격이 `ot_trail_gap` 안이면 속도를 **상대차 속도로 캡** — 램프 도중 아직 정면에 있는
     상대차에 다가붙지 않고, 횡으로 벗어난 뒤에야 가속한다(런웨이 확장의 안전장치).
   - **라이브 /map 충돌 검사**: CSV의 `d_left/d_right` 레이캐스트는 실제 로드된 맵에만 있는 구조물
     (장애물 베이크 맵의 벽 등)을 모를 수 있다 — Frenet 경계만 믿으면 경로가 그런 벽을 관통한다.
     그래서 모든 후보 spline의 샘플마다 라이브 `/map`에서 `ot_map_clearance` 반경의 여유 공간을
     추가 검증한다. 점유 셀에 걸리면 apex를 축소해 재시도하고, apex 축소로 해결되지 않는 위치면
     그 사이드를 거부한다(양쪽 다 거부 → commit 안 함 / 접근 중이면 중단 → global 복귀).
   - **Committed 동안 매 사이클 판정**:
     - **완료**: 상대차가 에고 뒤로 `ot_completion_margin` 이상 **실제로 관측된 적이 있고**(추월
       증거), 에고가 병합점을 지남 → 빈 OT 1회 → 발행 중단, `ot_cooldown_s` 쿨다운. 트랙 소실만으로
       완료 판정하지 않는다 — 벽에 박아 상대차가 시야에서 사라져도 "overtake complete"가 뜨지 않음.
     - **상대차 소실**: 추월 관측 전에 트랙이 사라지면 **중단**(`opponent lost before the pass`).
     - **접근 구간 지속 판정**(상대차와 나란해지기 전, `ds > max(1, ot_pass_clearance_s)`): 매 사이클
       commit 게이트 전체(코너·벽마진 내 apex fit·상대속도·창)를 현재 상태에서 재실행한다. 확정
       사이드가 불가능해지면 반대쪽으로 side switch, 양쪽 다 불가능하면 **중단하고 global 복귀**
       (`no longer feasible: <사유>`) — 추월이 어려운 환경으로 진입하면 무리하지 않는다. 상대차가
       에고 통로에서 벗어나면(`|opp_d| > 반폭합+block_margin+0.1`) 추월 자체가 불필요 → 중단.
     - **나란해진 후**: global로 내려가면 상대차 쪽으로 조향하게 되므로 지속 판정으로는 중단하지
       않는다 — 간격 상실(side switch/중단)과 상대속도 붕괴만 적용.
     - **침범**: 상대차가 확정 apex 쪽 통과 간격을 침범 → 반대쪽으로 재계획(side switch), 불가하면
       중단(빈 OT → global 추종).
     - **상대속도 붕괴**: `v_rel < ot_abort_rel_vel`(히스테리시스) → 중단.
     - **드리프트 재계획**: 상대차가 계획 시점 대비 `replan_ds/dd_threshold` 이상 이동 → 같은 쪽으로
       현재 에고 `(s,d)` 기준 재계획(경로 시작점이 항상 실제 차 위치).
     - **에고 이탈 가드** (`ot_ego_dev_replan`, 기본 0.4 m): 위 판정들은 전부 **상대차** 이동만 보는데,
       **에고** 쪽이 경로에서 이탈할 수도 있다(추종 실패·범프·벽 접촉). 확정 경로에서 s가 가장 가까운
       점의 `d`와 현재 에고 `d`를 비교해, 임계 초과면 현재 포즈 기준 같은 쪽 재계획
       (`replan (ego off path)`), **2배 초과면 나란한 상태여도 중단**(`ego far off committed path`) —
       그 정도로 벗어났으면 확정 라인 자체가 무의미하고(충돌/미끄러짐), 유지하는 쪽이 더 위험하다.
       중단 후엔 쿨다운의 추종 모드가 현재 포즈에서 경로를 재구축한다. **차가 없는 곳의 라인을
       컨트롤러가 쫓는 "이상한 로컬패스" 증상의 방지 장치.**
     - **타임아웃**: `ot_max_duration_s` 초과 → 중단(스테일 방지).
   - **에고 투영 신선도 게이트** (`ego_pose_grace_s`, 기본 0.3 s, 노드 쪽): 에고 (x,y)→(s,d) CLCS
     투영이 실패하면(벽 충돌로 트랙 밖으로 밀리는 등 투영 도메인 이탈) 마지막 성공값이 남는데, 이
     동결된 (s,d)로 계속 계획하면 충돌 전 위치 기준의 경로가 계속 발행된다. 마지막 **성공** 투영이
     유예 시간보다 오래되면 플래너에 `ego.s = -1`을 넘겨 활성 경로를 **빈 OT 1회로 정리하고 침묵**
     — 하류는 global로 폴백하고, 차가 도메인 안으로 돌아오면 자동 재개된다. **충돌 후 주행이 계속
     이상해지는 증상의 방지 장치.**
   - **발행 정책**: Committed 동안만 `OTWpntArray`(`/overtake_waypoints`) +
     `nav_msgs/Path`(`/planner/avoidance/path`, RViz 초록) 발행. 완료/중단 시 **빈 OT 1회** 발행 후
     **침묵**(Idle/Cooldown에서는 아무것도 발행 안 함 — 단, 아래 추종 모드 예외). `wpnt_publisher`는
     빈 OT를 받으면 즉시 global로 폴백한다(OT 속도배율 `ot_vx_scale`, 기본 1.0).
   - **추종(트레일) 모드** (`ot_trail_enabled`, 기본 true): 상대차가 통로를 막고 있는데 **추월이
     불가능**하면(commit 게이트 거부 — 코너·공간 없음·상대속도 부족 등 — 또는 중단 후 쿨다운 중)
     침묵하지 않는다 — 침묵하면 에고가 풀스피드 global 라인으로 복귀해 앞차를 들이받는다. 대신
     **감속-추종 로컬 경로**를 발행한다(`ot_line="trail"`):
     - **경로**: 현재 에고 `d`에서 raceline(`d=0`)으로 기울기/곡률 제한을 지키며 복귀한 뒤 raceline을
       따라 상대차 위치까지(로컬 구간만). 복귀 램프가 상대차까지의 간격보다 길어야 하면 램프를
       자르는 대신 **경로를 상대차 s 너머로 연장**한다 — 램프를 자르면 (추월 중단 직후처럼 에고가
       옆으로 벌어진 상태에서) 곡률 한계의 몇 배짜리 "갈고리" 경로가 나와 급조향/스핀을 유발했다.
       상대차 s를 지나는 기하는 안전하다: 속도가 `t_follow`부터 상대차 속도로 캡되어 간격 안으로
       들어가지 않는다.
     - **속도**: 추월과 동일한 곡률 상한 + 전후 종가속 패스를 쓰되, **상대차 뒤 `ot_trail_gap`(기본
       1.5 m) 지점부터는 상한이 상대차 속도** — 후진 패스가 그 앞에서 미리 부드럽게 감속시킨다. 이미
       간격 안이면 상대 속도 이하로 유지해 간격이 다시 벌어진다. `v_floor`는 적용하지 않는다(느린
       앞차를 따라가려면 느려질 수 있어야 함).
     - **종료**: 매 사이클 재생성(commit 아님). 상대차가 통로를 벗어남/추월됨/소실 → **빈 OT 1회**
       (`trail end`) 후 침묵. 추월이 가능해지는 순간 그대로 commit 경로로 승격(연속 발행, 빈 OT 없음).
   모든 spline·게이트·속도 파라미터는 `config/opponent_detector.yaml` 한 파일에 있다.

## 3. 실행

전제 노드가 떠 있어야 한다: `/global_waypoints`(→ `new_map_con`), `/map`+TF(→
`monte_carlo_localization`) 또는 시뮬레이터.

```bash
cd ~/2026_IFAC && source install/setup.zsh

# 실차: 에고 pose = /pf/pose/odom
ros2 launch opponent_detector opponent_detector.launch.py

# 시뮬: 에고 pose = /ego_racecar/odom, use_sim_time=true
ros2 launch opponent_detector opponent_detector.launch.py simulator:=true
```

### 3-1. 2-agent 폐루프 — gap_follow로 상대차 주행

실제 움직이는 상대차를 `~/gap_follow`(패키지 `f1tenth_control`)로 돌려 검출을 보는 전체 예다.
`f110_msgs` 정의가 두 워크스페이스에서 동일해 상호 통신된다.

**사전 설정(이미 반영됨):** `~/sim_ws/.../config/sim.yaml`에서 `num_agent: 2`,
에고 스폰 = raceline wpnt0 `(9.65, 5.69, -2.78)`, 상대차 스폰 = wpnt10 `(5.88, 4.86, 3.04)`.
> 2-agent 브리지는 **에고와 상대차가 둘 다 drive를 발행해야** 물리 스텝을 돈다. 그래서 에고도
> (new_map_con으로) 반드시 주행시켜야 하고, `new_map_con`이 `/global_waypoints`도 발행한다.

토픽 매핑(에고↔상대):
`/scan`·`/ego_racecar/odom`·`/drive`(에고) / `/opp_scan`·`/opp_racecar/odom`·`/opp_drive`(상대).
gap_follow의 최종 drive는 기본 `/drive`(=에고!)라 **상대차는 `drive_topic:=/opp_drive`로 반드시 remap**한다.

```bash
# 터미널 1 — 2-agent 시뮬(자체 RViz도 함께 뜸)
source ~/sim_ws/install/setup.zsh
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# 터미널 2 — 에고 컨트롤러 + /global_waypoints 발행
source ~/2026_IFAC/install/setup.zsh
ros2 launch new_map_con new_map_con.launch.py simulator:=true

# 터미널 3 — 상대차 = gap_follow (opp 토픽으로 remap, 조이스틱 없이 자율주행, ★AEB 끔)
source ~/gap_follow/install/setup.zsh
ros2 launch f1tenth_control gap_follow.launch.py \
    scan_topic:=/opp_scan odom_topic:=/opp_racecar/odom drive_topic:=/opp_drive \
    is_simulation:=true force_autonomous:=true enable_aeb:=false

# 터미널 4 — 검출기(+RViz). 시뮬이 이미 RViz를 띄우므로 보통 rviz:=false로 두고
#            시뮬 RViz에 /perception/obstacles/markers(MarkerArray)만 Add 하는 게 창 하나로 깔끔
source ~/2026_IFAC/install/setup.zsh
ros2 launch opponent_detector opponent_detector.launch.py simulator:=true
#   별도 창으로 보고 싶으면: ... simulator:=true rviz:=true
```

확인: 시뮬 RViz(또는 터미널 4 RViz)에서 상대차가 에고 전방 `/scan`에 들어오면 **빨강 실린더**로
검출되고 `/proj_opponent_trajectory`에 포인트가 쌓인다. 정지 장애물을 두면 **파랑**으로 유지된다.
스폰이 트랙을 벗어났으면 RViz의 `2D Pose Estimate`(에고) / 시뮬 리셋으로 raceline에 올린다.
(헤드리스 e2e 검증: 두 차량 지속 랩 주행, `/proj_opponent_trajectory` 200pts·`opp_is_on_trajectory=true`, 동적 상대차 검출 확인.)

#### 트러블슈팅 (2-agent에서 자주 걸리는 것)

- **상대차가 안 움직이고 AEB 브레이크만** (`Lidar AEB State: TRIGGERED`, `Target Speed 0.0`): gap_follow의 LiDAR
  AEB가 raceline 고속 코너 접근에서 TTC<0.35s로 트리거된 뒤 `latch_aeb:true`라 영구 잠긴다. **`enable_aeb:=false`**
  로 상대차 AEB를 끄면 계속 랩을 돈다(위 명령에 이미 포함). 정 필요하면 `/aeb/reset`(std_msgs/Empty) 발행으로도
  일시 해제되지만 곧 재잠금된다.
- **에고가 안 움직이거나 곧 멈춤**: 대개 원인은 위의 상대차 AEB다 — 상대차가 트랙 한복판에 서면 raceline을 따라
  풀스피드로 달리는 에고가 그걸 **추돌 → f110_gym 충돌 done → 시뮬 프리즈**. 상대차를 계속 움직이게 하면
  (`enable_aeb:=false`) 해결된다. 에고가 빨라 상대차를 따라잡아 추월/접촉하면 시뮬을 리셋한다.
- **검출기가 아무 것도 안 냄(멈춘 듯)**: 이 `f1tenth_gym_ros` 브리지는 **`/clock`을 발행하지 않는다**(월클럭 구동).
  그래서 `opponent_detector`는 `use_sim_time=false`로 돌아야 한다 — launch 기본값이 false이니 `simulator:=true`만
  줘도 된다(토픽 프로파일만 시뮬로 전환, 시간은 월클럭). `/clock`을 내는 시뮬을 쓸 때만 `use_sim_time:=true`.

> **단발 확인(시뮬/상대차 없이)**: 아래 4장 합성 하네스가 파이프라인만 빠르게 검증한다.

### 3-2. 회피까지 켜기 (에고가 상대차를 피하게)

검출 노드는 `/overtake_waypoints`를 발행할 뿐이다. 에고가 실제로 피하려면 이 라인을
`/local_waypoints`로 바꿔 컨트롤러에 먹이는 **2개 노드를 추가**한다(위 §3-1 4-터미널에 이어서).

```bash
# 터미널 5 — Frenet 오도메트리 (wpnt_publisher의 트리거 /car_state/frenet/odom 발행)
source ~/2026_IFAC/install/setup.zsh
ros2 launch global_planning frenet_odom.launch.py simulator:=true    # use_sim_time 기본 false

# 터미널 6 — 로컬 웨이포인트 발행기 (global + OT 회피라인 병합 → /local_waypoints)
ros2 run wpnt_publisher wpnt_publisher
```

동작: 상대차가 에고 전방 `[avoid_trigger_min_ds, max]`에 들어오면 검출 노드가 회피 라인을 발행하고
(`wpnt_publisher` 로그 "OT 경로 업데이트됨"), `new_map_con`이 `/local_waypoints`(회피 라인)를 따라
상대차를 비껴간다. 상대가 없으면 빈 OT → global 복귀. RViz에 회피 라인이 **초록**으로 뜬다.
끄려면 `avoidance_enabled:=false` 또는 두 노드를 안 띄우면 된다(= 기존 raceline 주행).

> **주의(현재 튜닝)**: 에고(new_map_con)가 raceline 풀스피드(~6 m/s)로 상대차(~2 m/s)보다 훨씬 빨라
> 추종이 거칠 수 있다. 깨끗한 회피 시연에는 에고 속도를 상대차에 맞춰 낮추는 것을 권장(후속 작업).
> 검출→회피라인→`/local_waypoints` 병합 자체는 헤드리스 e2e로 검증됨(우측 0.35 m 이탈, 충돌 없음).

## 4. 확인·디버깅

**RViz로 화면 보며 테스트** — launch에 `rviz:=true`를 주면 노드와 함께 RViz2가 프리셋
(`rviz/opponent_detector.rviz`: top-down, fixed frame=map, `/map`+`/scan`+마커)으로 뜬다.
정지 장애물=파랑, 동적 상대차=빨강 실린더.

```bash
# 방법 A) 합성 하네스로 즉시 확인(시뮬 불필요)
ros2 launch opponent_detector opponent_detector.launch.py rviz:=true        # 터미널 1
python3 src/opponent_detector/test/synthetic_opponent_test.py               # 터미널 2
#  → 빨강 실린더가 +s로 이동(상대차), 파랑 실린더 고정(정지 장애물). 하네스는 6초 후 종료.

# 방법 B) f110_gym 폐루프
ros2 launch opponent_detector opponent_detector.launch.py simulator:=true rviz:=true
```

토픽 직접 확인:

```bash
ros2 topic echo /perception/obstacles          # is_static/vs/vd 확인
ros2 topic echo /perception/static_obstacles/cartesian  # x/y/s/d/radius 확인
ros2 topic echo /proj_opponent_trajectory       # 동적 상대차 Frenet 포인트
# RViz를 따로 띄우려면:
rviz2 -d install/opponent_detector/share/opponent_detector/rviz/opponent_detector.rviz
```

합성 통합 테스트(가짜 입력으로 파이프라인만 검증, 시뮬 불필요, PASS/FAIL만):

```bash
# A: 노드
ros2 run opponent_detector opponent_detector_node --ros-args \
  --params-file src/opponent_detector/config/opponent_detector.yaml
# B: 하네스 (이동 상대차 + 정적 물체 주입 → PASS/FAIL)
python3 src/opponent_detector/test/synthetic_opponent_test.py
```

기대: 이동 물체는 `is_static=False`·`|vs|≈진값`, 정적 물체는 `is_static=True`,
`/proj_opponent_trajectory`에 포인트 누적.

## 5. 튜닝 가이드

| 증상 | 조정 |
|------|------|
| 상대차를 못 잡음(전방) | `max_viewing_distance`↑, `min_cluster_points`↓ |
| 벽/구조물이 상대차로 오검출 | `boundaries_inflation`↑, `use_map_filter:true`, `map_inflation_cells`↑ |
| 정지 장애물이 아예 검출 안 됨 | `max_obs_size`↑(0.5×0.5의 대각 0.707 초과여야) |
| 정지 장애물이 동적(상대차)로 오분류 | `dyn_vel_enter`↑, `static_ref_gate` 조정, `dyn_min_frames`↑ |
| 정지에 가까운 상대차를 정적으로 봄 | `dyn_vel_enter`↓, `vs_reset`↓ |
| 라벨이 자주 깜빡임 | `dyn_min_frames`↑, `dyn_vel_enter`와 `dyn_vel_exit` 간격↑ |
| 큰 물체가 잘림/합쳐짐 | `max_obs_size`, `lambda_deg`, `cluster_sigma` 조정 |
| 속도 추정이 느림/떨림 | `process_var_vs`/`process_var_vd`, `meas_var_*` 조정 |
| 컨트롤러가 라인을 못 따라가 재계획이 잦음 | `ot_ego_dev_replan`↑ (기본 0.4 m; 정상 추종 오차보다 커야) |
| 충돌/이탈 후에도 로컬패스가 계속 발행됨 | `ego_pose_grace_s` 확인(에고 투영 신선도 게이트), 하류 `wpnt_publisher`의 `ot_timeout_sec` 확인 |

## 6. 알려진 제약

- 폐루프 raceline 가정(Frenet `s` wrap). 오픈 경로도 동작하지만 시작/끝 이음부 근처는 부정확.
- 단일 상대차 중심 설계(다중 트랙은 추적하되 `ProjOppTraj`는 1대 선택).
- GP로 다듬은 `OpponentTrajectory`(상대차 예측 라인)는 범위 밖(후속 별도 노드).
- 시뮬 `f1tenth_gym_ros` 브리지는 LiDAR 근접 오검이 알려져 있음(메모 참고) — 필요 시 in-process
  `f110_gym`로 검증.
