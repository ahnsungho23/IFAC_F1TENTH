# opponent_detector

> 🌐 **한국어** · [English](README_en.md)

**LiDAR 단독**으로 동적 상대차를 **검출·추적하고, 확정형(committed) 추월 경로까지 만드는** C++ 노드
패키지. 2D LiDAR 스캔만으로 상대차의 Frenet 위치·속도를 추정하고, global path 위의 상대차를 추월하는
**국소(local) spline 추월 라인**을 `OTWpntArray`로 발행한다. 검출과 추월 플래너가 한 노드에 합쳐져
있고, spline 파라미터를 포함한 모든 튜닝값이 **단일 YAML**(`config/opponent_detector.yaml`)에 있다.
추월 라인은 `wpnt_publisher` → `/local_waypoints` → `new_map_con` 순으로 흘러 컨트롤러가 자동으로
따라간다.

논문 [`2603.27207v1.pdf`](2603.27207v1.pdf)(Cihlar 외, *Autonomous overtaking trajectory
optimization…*)의 상대차 검출 아이디어를 이 저장소(카메라 없음)에 맞게 각색한다. 논문은
**depth 카메라+YOLO로 "어느 LiDAR 클러스터가 상대차인가"를 선별**하지만, 우리는 카메라가 없으므로
그 역할을 **raceline(Frenet) 프레임에서의 상대속도**로 대체한다 — 정적 구조물은 트랙 프레임에서
가만히 있고, 상대차는 움직인다.

상세 동작 설명: [`docs/opponent_detector_node.md`](docs/opponent_detector_node.md).
시뮬/하네스 테스트 명령어 치트시트: [`docs/sim_test_commands.md`](docs/sim_test_commands.md).

## 1. 작동 원리

에고 pose로 에고 운동을 상쇄하는 Frenet 프레임에서 "정적/동적"을 가른다.

1. `/scan`을 TF로 map 프레임 점군으로 변환한다(스캔의 `frame_id`를 그대로 사용).
2. **Adaptive Breakpoint** 군집화 — 거리 의존 임계로 점을 클러스터로 나눈다.
3. 각 클러스터를 박스(AABB)로 피팅해 중심·크기를 얻고, 너무 큰 것은 버린다.
4. **지도필터 + 코너 경계 필터** — `/global_waypoints`의 `d_left/d_right`로 주행 통로 밖을 버리고,
   `/map`(SLAM) 점유격자에서 알려진 정적 구조물(**벽**)에 놓인 클러스터를 버린다.
5. 중심을 **Frenet (s,d)** 로 투영한다 — **`global_planning`의 CLCS 변환기**(`ClcsFrenetConverter`,
   vendored CommonRoad CLCS) 사용. 투영 도메인 밖(트랙에서 먼 점)이면 폐기. (경량 `FrenetProjector`는
   트랙 경계 `d_left/d_right` 조회와 s-wrap 용도로만 남겨둠 — CLCS가 이 둘은 제공 안 함.)
6. 최근접 연관 + **등속 칼만 필터**(상태 `[s, vs, d, vd]`)로 프레임 간 추적한다.
7. **정적배경 기준속도 대비 상대속도로 동적/정적 분류.** 벽(지도로 제거됨)과 정지 장애물은 하나의
   *정적배경 속도*(map 프레임에서 ≈0)를 공유한다. 각 트랙을 그 기준속도 대비 상대속도로 판정해
   **"벽과 같은 속도면 정적, 뚜렷이 다르면 동적(상대차)"**. 기준은 느린 트랙(`static_ref_gate` 미만)의
   평균이라 지역화 드리프트가 상쇄되고, 상대차는 기준에서 제외된다. (히스테리시스 + 상대`vs<reset`
   백스톱, `std` 위치표준편차 방식도 토글.)
8. 추적 결과를 `ObstacleArray`로(정적 장애물은 `is_static=true`), 동적 상대차를 Frenet `ProjOppTraj`로 발행한다.
9. **추월 플래너(상태머신 + PCHIP spline)** — `Idle → Committed → Cooldown` 상태머신이 추월
   기동의 존재 여부 자체를 결정한다.
   - **Commit 게이트(전부 만족해야 경로 생성):** 동적 상대차가 전방 `[trigger_min_ds, trigger_max_ds]`
     구간에서 에고 통로를 막고 있고(`avoid_block_margin`), **상대속도가 추월에 충분**하며
     (`ot_min_rel_vel`, 예상 캐치 시간 ≤ `ot_max_catch_time`), 기동 전 구간에 **급코너가 없고**
     (raceline `|κ| ≤ ot_max_kappa`), **예측 추월 지점**에 좌/우 여유 공간이 있어야 한다. 상대차도
     움직이므로 apex는 상대차의 현재 위치가 아니라 **추월이 실제로 일어날 예측 지점**에 놓인다.
   - **경로 형태:** 현재 에고 `(s,d)`에서 출발(에고가 라인 밖이어도 OK) → apex 오프셋으로 벌림 →
     추월 구간 동안 유지 → global raceline으로 복귀하는 **형상보존 PCHIP spline** `d(s)`. 경로는
     **추월 구간만 덮는 국소 경로**이며(전체 트랙 아님), PCHIP이라 overshoot 없음. spline의 횡기울기
     `|dd/ds|`(`ot_max_d_slope`)와 **횡곡률 `|d²d/ds²|`(조향 각→스핀 방지, `ot_max_path_kappa`)**를
     함께 제한해 램프 길이·apex를 맞추고, 속도는 곡률제한 `v=√(a_lat/κ)`에 **전후 종가속 스무딩**
     (`ot_max_long_accel`)을 걸어 타이트 구간 **이전에 미리 제동**하며, 양 끝을 **핸드오프 블렌딩**
     (`ot_speed_blend_s`)해 급가·감속 없이 이어진다.
   - **Committed 동안 지속 판정(상대차가 계속 움직이므로):** 완료는 **상대차가 뒤로 관측된 적이
     있을 때만**(트랙 소실은 완료가 아님 — 소실 시 중단). 나란해지기 전 접근 구간에서는 매 사이클
     commit 게이트 전체를 재실행해 **추월이 어려운 환경이 되면 중단하고 global로 복귀**(side switch
     우선, 양쪽 불가 시 abort). 상대차가 통로를 벗어나도 중단. 상대차가 확정 경로에 침범
     (`path_intrusion_margin`)하면 반대쪽 재계획, 불가하면 중단. 상대속도가 무너지면
     (`ot_abort_rel_vel` 히스테리시스) 중단(추종으로 전환). 상대차가 `replan_ds/dd_threshold` 이상
     이동하면 현재 에고 위치 기준으로 재계획. 나란해진 후에는 지속 판정으로 중단하지 않는다
     (global 복귀가 상대차 쪽 조향이 되므로).
   - **발행 정책:** Committed 동안만 OT 발행. 완료/중단 시 **빈 OT를 1회** 발행해 `wpnt_publisher`가
     global로 폴백하게 한 뒤 **발행을 완전히 중단**한다(쿨다운 `ot_cooldown_s` 후 재commit 가능).
     추월선은 `nav_msgs/Path`(`/planner/avoidance/path`)로도 미러 발행하고 RViz엔 **초록**으로 표시.
   - **추종(트레일) 모드:** 상대차가 막고 있는데 **추월이 불가능**하면(게이트 거부/중단 쿨다운)
     침묵 대신 **감속-추종 로컬 경로**(`ot_line="trail"`)를 발행한다 — raceline을 따라가되 상대차 뒤
     `ot_trail_gap` 지점에서 상대 속도에 맞도록 전후 가속도 패스로 미리 감속. 상대차가 통로를
     벗어나거나 추월되면 빈 OT 1회 후 침묵, 추월이 가능해지면 그대로 commit으로 승격.
   - **런웨이 확장(추종 고착 방지):** 상대차 바로 뒤에서 속도가 맞으면 예측 캐치 지점이 램프가 안
     나올 만큼 가까워 추종에 고착된다 — 이때 apex 지점을 최소 램프 길이까지 밀어내 **먼저 옆으로
     빠진 뒤 추월**하는 계획을 만들고, 아직 횡으로 못 벗어난 램프 구간에서는 속도를 상대차 속도로
     캡해(`ot_trail_gap` 안 간격 유지) 정면의 상대차에 다가붙지 않는다.
   - **이상 경로/충돌 복구 가드:** 확정 경로는 **에고가 이탈해도** 감시한다 — 횡 이탈이
     `ot_ego_dev_replan` 초과면 현재 포즈 기준 재계획, 2배 초과면 중단(차가 없는 곳의 라인을
     쫓지 않음). 에고 (s,d) CLCS 투영이 실패한 채 `ego_pose_grace_s`가 지나면(충돌로 트랙 밖 등)
     활성 경로를 빈 OT 1회로 정리하고 침묵한다(동결된 충돌 전 위치로 계획 금지). 추종 복귀 램프는
     간격에 맞춰 잘라내지 않고 경로를 연장해 곡률 한계를 항상 지킨다. 하류 `wpnt_publisher`도
     끊긴 OT 피드를 `ot_timeout_sec` 후 만료시켜 global로 폴백한다.

카메라의 "상대차 클러스터 선별"을 상대속도로 대체하는 것이 이 노드의 핵심이다. **정지 장애물(벽이
아닌 콘 등)도 차 크기(≤0.5×0.5 m)라 크기로는 상대차와 구분할 수 없으므로**, 속도 기준으로만 정적/동적을
가른다(`max_obs_size`는 0.5×0.5의 대각 0.707 m가 통과하도록 0.8).

### 노드 간 데이터 흐름

```
                        ┌─────── /scan (LaserScan)
                        │
                        ├─────── /global_waypoints (WpntArray, latched)
                        │           from: new_map_con
      External          │
      Inputs ───────────┼─────── /map (OccupancyGrid, latched)
                        │           from: MCL
                        │
                        ├─────── /pf/pose/odom (Odometry)
                        │           from: MCL / sim
                        │
                        └─────── TF: map → laser frame
                                          │
                               ┌──────────▼──────────────────────┐
                               │   opponent_detector_node         │
                               │                                  │
                               │  detection → tracking → planner  │
                               └──────────┬──────────────────────┘
                                          │
      External                            ├──▶ /overtake_waypoints (OTWpntArray)
      Outputs ────────────────────────────│       to: wpnt_publisher
                                          ├──▶ /perception/obstacles (ObstacleArray)
                                          ├──▶ /proj_opponent_trajectory (ProjOppTraj)
                                          └──▶ /planner/avoidance/path (Path, viz)
```

**입력:** MCL(`monte_carlo_localization`)이 제공하는 `/map`(SLAM 점유격자)·`/pf/pose/odom`(에고
위치)·TF(`map→laser`), `new_map_con`이 발행하는 `/global_waypoints`(전역 경로+트랙 경계),
그리고 LiDAR 하드웨어의 `/scan`을 구독한다.

**출력:** 확정형 추월 경로 `/overtake_waypoints`는 `wpnt_publisher`가 수신해
`/local_waypoints`로 합산한 뒤 `new_map_con`(pure-pursuit)이 추종한다.
`/perception/obstacles`와 `/proj_opponent_trajectory`는 다른 노드가 상대차 정보를
활용할 수 있도록 발행하며, `/planner/avoidance/path`는 RViz 시각화 전용이다.

## 2. 구독 / 발행 토픽

| 방향 | 토픽(파라미터) | 타입 | 기본값 |
|------|------|------|--------|
| 구독 | `scan_topic` | `sensor_msgs/LaserScan` | `/scan` |
| 구독 | `global_waypoints_topic` | `f110_msgs/WpntArray` (latched) | `/global_waypoints` (from `new_map_con`) |
| 구독 | `map_topic` | `nav_msgs/OccupancyGrid` (latched) | `/map` (from `monte_carlo_localization`) |
| 구독 | `ego_odom_topic` | `nav_msgs/Odometry` | `/pf/pose/odom` (sim: `/ego_racecar/odom`) |
| 구독 | TF | `map → <scan frame>` | MCL/시뮬레이터가 제공 |
| 발행 | `obstacles_topic` | `f110_msgs/ObstacleArray` | `/perception/obstacles` |
| 발행 | `raw_obstacles_topic` | `f110_msgs/ObstacleArray` | `/perception/detection/raw_obstacles` |
| 발행 | `proj_opp_traj_topic` | `f110_msgs/ProjOppTraj` | `/proj_opponent_trajectory` |
| 발행 | `avoidance_ot_topic` | `f110_msgs/OTWpntArray` | `/overtake_waypoints` (→ `wpnt_publisher`; Committed 동안만, 종료 시 빈 OT 1회 후 침묵) |
| 발행 | `avoidance_path_topic` | `nav_msgs/Path` | `/planner/avoidance/path` (추월선, RViz 초록) |
| 발행 | `opponent_path_topic` | `nav_msgs/Path` | `/perception/opponent/path` (상대차 궤적, RViz 주황) |
| 발행 | `markers_topic` | `visualization_msgs/MarkerArray` | `/perception/obstacles/markers` (상대=빨강/정적=파랑/회피=초록) |

## 3. 주요 파라미터

전체: [`config/opponent_detector.yaml`](config/opponent_detector.yaml).

| 파라미터 | 의미 | 기본값 |
|----------|------|--------|
| `simulator` | 시뮬 프로파일(에고 odom을 `/ego_racecar/odom`으로) | `false` |
| `lambda_deg` / `cluster_sigma` | Adaptive breakpoint 각/노이즈 | `10.0` / `0.03` |
| `min_cluster_points` / `max_obs_size` | 클러스터 최소 점수 / 최대 크기[m] (0.5×0.5 통과 위해 >0.707) | `5` / `0.8` |
| `max_viewing_distance` | 전방 관측 거리[m] | `9.0` |
| `boundaries_inflation` / `fallback_track_halfwidth` | 통로 축소[m] / 경계 미설정 시 반폭[m] | `0.1` / `1.5` |
| `use_map_filter` / `map_point_reject_ratio` | 지도(SLAM)필터로 벽 제거 / 클러스터 기각 비율 | `true` / `0.6` |
| `classifier_mode` | `velocity` \| `std` \| `both` | `velocity` |
| `dyn_vel_enter` / `dyn_vel_exit` / `dyn_min_frames` | 정적배경 대비 동적 진입/이탈 속도[m/s] / 유지 프레임 | `0.5` / `0.25` / `3` |
| `static_ref_gate` | 정적배경 속도 기준을 정의하는 트랙 속도 상한[m/s] | `0.3` |
| `vs_reset` | 저(상대)속 백스톱(정적 강제)[m/s] | `0.1` |
| `assoc_gate` / `ttl_dynamic` / `ttl_static` | 연관 게이트[m] / 동적·정적 수명 | `0.5` / `40` / `3` |
| `process_var_vs` / `process_var_vd` | 칼만 과정노이즈(s/d축) | `2.0` / `8.0` |
| **`avoidance_enabled`** | 추월 플래너 on/off | `true` |
| `avoid_trigger_min_ds` / `avoid_trigger_max_ds` | 추월 고려 전방 s-구간[m] | `0.5` / `8.0` |
| `avoid_block_margin` | 상대차가 이 여유 안(경로 위)일 때만 추월 고려[m] | `0.15` |
| **`ot_min_rel_vel`** | commit 최소 상대속도(도달가능 에고속도 − 상대 vs)[m/s] | `0.5` |
| **`ot_abort_rel_vel`** | committed 중 이 미만이면 중단(히스테리시스)[m/s] | `0.2` |
| **`ot_max_catch_time`** | 예상 캐치 시간이 이 이하일 때만 commit[s] | `6.0` |
| **`ot_max_kappa`** | 기동 전 구간 raceline \|κ\| 상한(급코너 게이트)[1/m] | `1.0` |
| `avoid_pre_distance` / `avoid_post_distance` | apex 진입 램프 최소 길이 / 복귀 길이[m] | `2.0` / `2.0` |
| **`ot_pass_clearance_s`** | 예측 추월 지점 전후 apex 유지 구간[m] | `1.0` |
| `avoid_spline_resolution` | 추월 라인 샘플 간격[m] | `0.15` |
| `avoid_lateral_clearance` / `avoid_boundary_margin` | 여유 간격 / 경계 안쪽 여유[m] | `0.30` / `0.35` |
| `ot_max_d_slope` | 조향 제한: 추월 spline 최대 `\|dd/ds\|`(램프·apex를 이 한계에 맞춤)[-] | `0.40` |
| `ot_max_path_kappa` | 곡률 제한: spline이 추가하는 최대 `\|d²d/ds²\|`(조향 각→스핀 방지)[1/m] | `0.5` |
| `ot_map_clearance` | 경로 샘플 주변 라이브 `/map` 여유 반경(CSV 경계에 없는 벽/장애물 차단)[m] | `0.20` |
| `ego_half_width` / `opponent_half_width` | 에고/상대차 반폭[m] | `0.15` / `0.25` |
| `avoid_side_mode` | 추월 방향(`auto`\|`left`\|`right`) | `auto` |
| `avoid_max_lat_accel` | 곡률제한 속도의 **하한** 횡가속 예산[m/s²] (샘플별 용량은 `max(이 값, raceline vx²·\|κ_r\|)` — raceline 자체 그립을 신뢰) | `4.0` |
| `ot_max_long_accel` | 종가속 예산[m/s²]: 전후 스무딩으로 타이트 구간 **이전에 미리 제동** | `4.0` |
| `avoid_speed_scale` / `avoid_v_floor` / `avoid_v_ceiling` | 속도 배율 / 하한·상한[m/s] | `1.5` / `6.0` / `15.0` |
| `ot_speed_blend_s` | 시작=현재 에고속도·끝=raceline 속도로 블렌딩(핸드오프 속도 스텝 방지)[m] | `1.5` |
| `path_intrusion_margin` | 상대차가 확정 경로에 이 거리 안으로 오면 재계획/중단[m] | `0.5` |
| `replan_ds_threshold` / `replan_dd_threshold` | 재계획을 강제하는 상대차 s/d 이동량[m] | `0.5` / `0.2` |
| **`ot_ego_dev_replan`** | 확정 경로 대비 **에고** 횡 이탈이 이 이상이면 현재 포즈 재계획, 2배면 중단[m] | `0.4` |
| **`ego_pose_grace_s`** | 에고 CLCS 투영 실패 허용 시간 — 초과 시 경로 정리 후 침묵(global 폴백)[s] | `0.3` |
| **`ot_completion_margin`** | 완료 판정: 상대차를 이만큼 지나쳐야 함[m] | `1.0` |
| **`ot_max_duration_s`** | 단일 기동 최대 지속시간[s] | `10.0` |
| **`ot_cooldown_s`** | 완료/중단 후 재commit까지 침묵 시간[s] | `2.0` |
| **`ot_trail_enabled`** | 추월 불가 시 감속-추종 경로 발행 | `true` |
| **`ot_trail_gap`** | 추종 시 상대차 뒤에 유지할 간격[m] | `1.5` |

## 4. 빌드 및 실행

```bash
cd ~/2026_IFAC
colcon build --packages-up-to opponent_detector    # global_planning(CLCS)까지 함께; 전체는 cb
source install/setup.zsh                            # alias: sc

# 실차
ros2 launch opponent_detector opponent_detector.launch.py

# 시뮬레이터(f110_gym): 에고 pose를 /ego_racecar/odom에서 받음
ros2 launch opponent_detector opponent_detector.launch.py simulator:=true
```

전제: `/global_waypoints`(→ `new_map_con`)와 `/map`+TF(→ `monte_carlo_localization` 또는 시뮬)가
발행 중이어야 한다.

## 5. RViz로 보면서 테스트

launch에 `rviz:=true`를 주면 노드와 함께 **RViz2**가 뜬다(map/scan/마커 뷰 프리셋:
[`rviz/opponent_detector.rviz`](rviz/opponent_detector.rviz), 위→아래 top-down, fixed frame=map).
정지 장애물=**파랑** 실린더, 동적 상대차=**빨강** 실린더로 보인다.

**방법 A — 합성 하네스(시뮬 불필요, 가장 빠름):**

```bash
# 터미널 1: 노드 + RViz
ros2 launch opponent_detector opponent_detector.launch.py rviz:=true
# 터미널 2: 가짜 raceline·map·이동 상대차·정지 장애물 주입 (PASS/FAIL 출력)
python3 src/opponent_detector/test/synthetic_opponent_test.py
```

RViz에서 **빨강 실린더가 +s로 미끄러지듯 이동(상대차)**, **파랑 실린더가 옆에 고정(정지 장애물)**
되면 정상이다. 하네스는 6초 후 종료된다.

**방법 B — f110_gym 2-agent 폐루프(실제 시뮬 + 움직이는 상대차):** 시뮬을 `num_agent:2`로 두고
에고=`new_map_con`, 상대차=`~/gap_follow`(`f1tenth_control`)로 돌린다. 상대차 drive는 반드시
`/opp_drive`로 remap. 전체 4-터미널 절차는
[`docs/opponent_detector_node.md` §3-1](docs/opponent_detector_node.md)에 정리했다. 요지:

```bash
# (1) 시뮬  (2) 에고               (3) 상대차 = gap_follow                 (4) 검출기
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 launch new_map_con new_map_con.launch.py simulator:=true
ros2 launch f1tenth_control gap_follow.launch.py \
    scan_topic:=/opp_scan odom_topic:=/opp_racecar/odom drive_topic:=/opp_drive \
    is_simulation:=true force_autonomous:=true enable_aeb:=false
ros2 launch opponent_detector opponent_detector.launch.py simulator:=true rviz:=true
```

> **필수 2가지** — 상대차는 `enable_aeb:=false`(안 그러면 LiDAR AEB가 영구 잠겨 트랙에 멈추고, 풀스피드
> 에고가 추돌해 시뮬이 프리즈됨). 검출기는 `use_sim_time`을 기본 false로 둔다(이 브리지는 `/clock` 미발행 =
> 월클럭). 자세한 트러블슈팅은 [`docs/opponent_detector_node.md` §3-1](docs/opponent_detector_node.md).

> RViz를 별도 창으로 띄우고 싶으면 `rviz2 -d install/opponent_detector/share/opponent_detector/rviz/opponent_detector.rviz`.

## 6. 검증(자동 체크)

RViz 없이 PASS/FAIL만 확인:

```bash
# 터미널 A
ros2 run opponent_detector opponent_detector_node --ros-args \
  --params-file src/opponent_detector/config/opponent_detector.yaml
# 터미널 B
python3 src/opponent_detector/test/synthetic_opponent_test.py   # PASS/FAIL 출력
```

## 7. 범위 밖(후속)

- GP로 다듬은 상대차 라인(`f110_msgs/OpponentTrajectory`)은 ForzaETH에서도 별도 학습 노드다.
  필요 시 `/proj_opponent_trajectory`를 소비하는 새 노드로 추가한다.

## 8. 참고

- node-level 규칙: [`AGENTS.md`](AGENTS.md)
- 상세 문서: [`docs/opponent_detector_node.md`](docs/opponent_detector_node.md)
- 원 논문: [`2603.27207v1.pdf`](2603.27207v1.pdf)
