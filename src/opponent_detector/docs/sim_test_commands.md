# opponent_detector 시뮬 테스트 명령어 모음

추월 플래너(검출 + 추종 + 추월)를 시뮬레이터/합성 하네스로 검증할 때 쓰는 명령어를 한 곳에
모은 치트시트다. 각 단계의 상세 원리는 [`opponent_detector_node.md`](opponent_detector_node.md)
3~4장을 참고.

> `~/.zshrc` 별칭: `cb` = colcon build(symlink+Ninja), `sc` = `source install/setup.zsh`,
> `f1sim` = `ros2 launch f1tenth_gym_ros gym_bridge_launch.py`

## 0. 빌드 & 소싱 (모든 터미널 공통 전제)

```bash
cd ~/2026_IFAC
cb --packages-select opponent_detector    # = colcon build --symlink-install ...
sc                                        # = source install/setup.zsh
```

## 1. 합성 하네스 3종 — 시뮬 없이 즉시 PASS/FAIL (가장 빠른 검증)

가짜 `/scan`·odom·맵을 주입해 파이프라인만 검증한다. **반드시 지킬 것 2가지**:

1. **깨끗한 `ROS_DOMAIN_ID`** 사용 — 다른 노드(시뮬 등)가 떠 있으면 토픽이 섞여 오판정.
2. 노드를 **`simulator:=true` 없이(차량 모드)** 띄울 것 — 하네스가 에고 odom을
   `/pf/pose/odom`(차량 토픽)으로 발행하므로, 시뮬 모드로 띄우면 에고 위치가 안 잡혀
   commit이 하나도 안 나오면서 에러 로그도 없다.

```bash
# 터미널 1 — 노드 (차량 모드! simulator:=true 금지)
export ROS_DOMAIN_ID=56
ros2 launch opponent_detector opponent_detector.launch.py

# 터미널 2 — 하네스 (같은 도메인)
export ROS_DOMAIN_ID=56
python3 src/opponent_detector/test/synthetic_opponent_test.py   # 검출: 동적/정적 분류·ProjOppTraj
python3 src/opponent_detector/test/commit_lock_test.py          # 추월: commit 유지→완료 empty 1회→침묵
python3 src/opponent_detector/test/trail_to_overtake_test.py    # 추종→추월 승격(런웨이 확장) 검증
python3 src/opponent_detector/test/offpath_recovery_test.py     # 경로 이탈/투영 실패 복구 검증
```

> **하네스 사이에는 노드를 재시작**할 것 — 이전 시나리오의 트래커 잔상(고스트 트랙)과 스테일
> 에고 상태가 다음 시나리오를 오염시킨다(아래 pkill 후 터미널 1 재실행).

| 하네스 | 시나리오 | PASS 조건 요약 |
|---|---|---|
| `synthetic_opponent_test.py` | 이동 상대차 + 정적 장애물 주입 | 동적/정적 분류 정확, `/proj_opponent_trajectory` 누적 |
| `commit_lock_test.py` | 느린 상대차를 에고가 추격(에고는 발행 경로를 횡추종) | commit 지속(중간에 trail로 안 떨어짐) → 완료 시 빈 OT 1회 → 침묵 |
| `trail_to_overtake_test.py` | 동속(0.8 m/s) 상대차 2 m 뒤 밀착 추종 | 밀착 추종에서도 commit 발생(apex ≥ 0.6), 램프 중 간격 유지, 이탈 후 가속 |
| `offpath_recovery_test.py` | commit 중 에고를 옆으로 0.9 m 순간이동 → 이어서 트랙 밖 30 m로 | 이동 후 모든 경로가 새 에고 기준 재계획, 투영 도메인 이탈 후 빈 OT 1회 + 침묵 |

테스트 후 노드 정리 (그냥 `pkill -f opponent_detector_node`는 자기 셸까지 죽일 수 있음):

```bash
pkill -f 'opponent_detector_nod[e]'
```

## 2. f1sim 2-agent 폐루프 — 실제 추월 시연 (터미널 6개)

에고(new_map_con) + 상대차(gap_follow)를 f1sim에서 돌리고 회피/추월까지 확인하는 풀 파이프라인.
사전 설정(이미 반영됨): `~/sim_ws/.../config/sim.yaml`에 `num_agent: 2` + raceline 위 스폰.

```bash
# 터미널 1 — 2-agent 시뮬 (자체 RViz 포함)
source ~/sim_ws/install/setup.zsh
f1sim                                     # = ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# 터미널 2 — 에고 컨트롤러 + /global_waypoints
source ~/2026_IFAC/install/setup.zsh
ros2 launch new_map_con new_map_con.launch.py simulator:=true

# 터미널 3 — 상대차 = gap_follow (★opp 토픽 remap + AEB 끄기 필수)
source ~/gap_follow/install/setup.zsh
ros2 launch f1tenth_control gap_follow.launch.py \
    scan_topic:=/opp_scan odom_topic:=/opp_racecar/odom drive_topic:=/opp_drive \
    is_simulation:=true force_autonomous:=true enable_aeb:=false

# 터미널 4 — 검출/추월 플래너 (시뮬 RViz에 마커만 Add하는 게 창 하나로 깔끔)
source ~/2026_IFAC/install/setup.zsh
ros2 launch opponent_detector opponent_detector.launch.py simulator:=true
#   별도 RViz 창: ... simulator:=true rviz:=true

# 터미널 5 — Frenet 오도메트리 (/car_state/frenet/odom)
source ~/2026_IFAC/install/setup.zsh
ros2 launch global_planning frenet_odom.launch.py simulator:=true

# 터미널 6 — 로컬 웨이포인트 병합 (global + OT → /local_waypoints)
source ~/2026_IFAC/install/setup.zsh
ros2 launch wpnt_publisher wpnt_publisher.launch.py   # YAML 로드(OT 스테일 타임아웃 포함)
```

기대 동작(상태 순서):

1. 상대차가 전방 트리거 창에 들어오고 통로를 막음 → **추종**(`ot_line="trail"`, 상대 속도로 감속)
2. 추월 가능해지는 순간(밀착 상태여도 **런웨이 확장**으로 apex를 밀어내 계획) → **commit 승격**
   (연속 발행, 빈 OT 없음). RViz에 추월선 **초록**.
3. 램프 중 아직 상대차 정면이면 상대 속도 유지(간격 유지) → 옆으로 빠진 뒤 가속 → 추월
4. 완료(상대차가 뒤로 관측) → **빈 OT 1회** → 침묵, global 복귀

## 3. 정지 장애물 맵으로 테스트 (선택)

정지 장애물(파랑 분류) 확인용 — 장애물을 맵에 굽고 그 맵으로 f1sim을 띄운다:

```bash
python3 ~/f1tenth_gym/tools/obstacle_map_maker.py    # GUI에서 장애물 배치 → obstacle_sim_launch.py로 시뮬 실행
```

## 4. 확인·디버깅 명령

```bash
# 검출 결과
ros2 topic echo /perception/obstacles              # is_static / vs / vd
ros2 topic echo /proj_opponent_trajectory          # 동적 상대차 Frenet 궤적

# 추월 플래너 출력
ros2 topic echo /overtake_waypoints --field ot_line   # "trail" / "overtake" / (빈 OT)
ros2 topic echo /planner/avoidance/path --no-arr             # RViz 초록 경로 미러

# 상태머신 전이 로그만 보기 (commit / replan / trail / abort / complete 사유)
ros2 launch ... 2>&1 | grep --line-buffered "overtake:"
```

자주 걸리는 문제:

| 증상 | 원인/해결 |
|---|---|
| 하네스에서 commit 0건, 에러도 없음 | 노드를 `simulator:=true`로 띄움 → 차량 모드로 재실행 (§1) |
| 하네스 결과가 실행 순서에 따라 달라짐 | 이전 시나리오 잔상 → 하네스마다 노드 재시작 (§1) |
| 상대차가 안 움직이고 정지 | gap_follow AEB 래치 → `enable_aeb:=false` (§2 터미널 3) |
| 검출기가 아무 것도 안 냄 | 이 브리지는 `/clock` 미발행 → `use_sim_time`은 기본 false 유지 |
| 추종만 하고 추월 안 함 | 게이트 사유 확인(`grep "overtake:"`). 밀착-동속 고착은 런웨이 확장으로 해결됨 — 재발 시 `ot_max_d_slope`/`ot_max_path_kappa`(램프 길이), `ot_trail_gap`(간격 유지·스윙아웃 시점) 튜닝 |
| 갑자기 급조향하는 "갈고리" 로컬패스 | 추월 중단 직후 trail 복귀 램프가 잘리던 버그 — 수정됨(경로를 상대차 s 너머로 연장). 재발 시 `ot_max_path_kappa` 확인 |
| 벽 충돌 후 주행이 계속 이상함 | 노드 로그에서 `ego Frenet projection stale` / `replan (ego off path)` / `ego far off committed path` 확인. 에고 이탈 가드(`ot_ego_dev_replan`)와 투영 신선도(`ego_pose_grace_s`)가 정리·재계획해야 정상. 하류 안전망은 `wpnt_publisher`의 `ot_timeout_sec`(0.5 s) |
| `pkill`이 내 셸을 죽임 | `pkill -f 'opponent_detector_nod[e]'` 패턴 사용 |
