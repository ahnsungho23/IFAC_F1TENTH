# map_creator_node

## 1. 노드 목적

랩 1·2에서 확정된 정적 장애물을 글로벌 경로에 **영구 반영**하는 파이프라인 노드입니다.
랩 2→3 전이(`lap_count` 1→2) 시점에 장애물 원장을 freeze하고, local_planning의 좌/우 판정 로직
(`RacelineSplinePlanner::plan`, C++ 공유 — Python 복제 없음)으로 **막을 쪽**을 정한 뒤
(레이스라인을 아예 안 막는 장애물은 plan()이 평가하지 않으므로 장애물 d 부호로 통과 쪽을 정함),
그쪽을 맵에서 벽까지 검게 칠한 `obstacle_map`을 만들고, 오프라인 생성기로 글로벌
레이스라인을 재생성해, 다음 랩 경계에서 `/global_planning/reload_waypoints`로 원자
교체합니다. 설계 근거는 `learning_adaptive_globalpath/MAP_CREATOR_PROPOSAL.md` 참고.

## 2. 동작 원리 (FSM)

```text
IDLE ──(lap_count ≥ trigger, 원장 freeze)──▶ 판정+페인팅+저장 ──▶ GENERATING
  드라이버(bin/regenerate_obstacle_map, C++) 완료·물리 게이트 통과 ──▶ ARMED
  lap_count 다음 갱신 ──▶ reload 호출 ──▶ MONITORING
  (authoritative 스냅샷에서 사라진 뒤 2랩 → 원장 제거 → 재생성 /
  전부 소실 → baseline 재시딩 + reload)
어느 단계든 실패 → ABORTED (기존 라인 유지, 로컬 회피가 계속 커버)
```

1. **P0 투영 + 원장(ledger)**: `/adaptive_obstacle_map`은 Cartesian AABB만 싣습니다
   (static_obstacle_map은 Frenet 기준선을 소유하지 않음 — 해당 패키지 문서 계약).
   map_creator가 최초 `/global_waypoints`(불변 P0)로 만든 CLCS 변환기
   (`global_planning::ClcsFrenetConverter`)로 각 AABB를 직접 투영해
   `s_center/d_center/s_start/s_end/d_left/d_right`를 채웁니다. 투영은 중심 1점만 전역
   투영하고 꼭짓점 4개는 중심 tangent 좌표계로 회전시키므로(obstacle_detector의
   `aabb_frenet_projector`와 같은 구성) 꼭짓점이 인접 branch로 튀지 않습니다.
   **한 장애물이라도 투영에 실패하면 스냅샷 전체를 거부**하고 원장을 갱신하지 않습니다
   (개별 skip은 "소실"로 오역되어 `removal_miss_laps` 오삭제를 유발). 빈 배열은 정상
   스냅샷입니다. 원장은 투영된 Frenet 중심의 wrap-aware 거리(`|Δs|<match_max_ds_m`,
   `|Δd|<match_max_dd_m`)로 매칭하며, 매칭 기하가 배열에서 사라진 시점부터만
   `removal_miss_laps`를 계산합니다(토픽 침묵·투영 실패는 소실이 아님).
   별도 재확인은 하지 않고 persistent 항목을 그대로 freeze합니다.
   모든 s/d가 항상 P0 기준이므로 스왑으로 `/global_waypoints`가 바뀌어도(검출기는 새
   라인으로 CLCS를 재구축) 원장 매칭·소실 판정은 일관됩니다.
2. **판정**: map_creator 전용 튜닝 파라미터(`decision.*`)로 만든 플래너 인스턴스에서
   ego = (장애물 s − 12 m, d=0, v=해당 waypoint 속도)로 평가. 좌 통과 → 오른쪽을 막음,
   우 통과 → 왼쪽을 막음, safe_stop → 해당 장애물은 **베이크하지 않음**(양쪽을 막으면
   트랙이 폐쇄되어 centerline 추출이 불가하기 때문).
3. **페인팅**: 원본 맵의 새 사본에서(누적 편집 금지) 장애물 사각형 + 막는 쪽 면→벽
   polygon을 0으로 채움. "비주행" 판정은 픽셀 < 250 (생성기 free 판정의 보수).
   벽을 못 찾으면(`wall_not_found`) 전체 중단 — 장애물 단독 페인팅은 cleanup의
   speckle 보호(25 px)에 지워질 수 있어 금지.
4. **저장 = 생성기 입력**: `output/obstacle_map/obstacle_map.{png,yaml}` 저장 직후 두
   경로가 모두 일반 파일인지 확인한다. 하나라도 없으면 pipeline을 중단해 optimizer를
   실행하지 않는다. 검증된 YAML 위치가 그대로 드라이버의 `--map-yaml` 입력이 된다
   (복사·수동 단계 없음).
5. **재생성**: 드라이버가 GUI와 동일한 `load_gui_params(gui_params.yaml)` 값으로
   `generate_trajectory` 실행 → 물리 게이트(폐곡선, off_map=0, s 순증가, |κ|≤3.2,
   장애물 이격 ≥0.42 m) 통과 시 `global_waypoints.json` + `gate_report.json` 기록.
   1차는 `initial_smooth_sigma=4.1`과 gui_params의 `safety_width=0.4`를 사용합니다. 게이트 실패 시
   `retry_safety_width=0.4`, `retry_smooth_sigma=2.5`로 한 번만 재시도합니다.
6. **스왑**: republisher가 기존 `output/map`은 그대로 둔 채 별도
   `output/obstacle_map/global_waypoints.json`을 읽고 검증한 후, 메모리 bundle과
   활성 참조 경로만 교체해 즉시 발행.
7. **AVOID 게이트 갱신**: 스왑 **성공 직후**(새 라인이 실제로 살아난 뒤에만)
   state_machine의 `allow_avoid_transition` 파라미터를 `false`로 내려
   GLOBAL→AVOID 진입을 차단한다. 새 라인이 장애물을 이미 우회하므로 로컬 회피가
   불필요해지기 때문이다. 반대로 장애물 전부 소멸로 baseline rollback 스왑이
   성공하면 `true`로 복원한다. `kArmed`(생성 검증 통과) 시점에는 절대 내리지
   않는다 — 스왑 전까지는 장애물을 통과하는 옛 라인 위라 회피가 계속 필요하다.
   파라미터 서비스 미준비 시 tick마다 재시도하며, 거부되면 로그만 남긴다.
8. **제어 속도 상한 전송**: 같은 트리거(스왑 성공 직후)로 control 노드
   (`control_node_name`)의 `max_speed` 파라미터를 `swap_max_speed_mps`(기본 7.0)로
   바꾼다. baseline rollback 스왑에서는 `rollback_max_speed_mps`(>0일 때만)를 보내
   복원한다.
   - **방향은 control 쪽 기동 기본값과의 비교로 정해진다.** 현재
     `control_real.launch.py`의 `max_speed`가 **5.0**이므로 7.0은 **올리는** 값이다 —
     랩1~2는 장애물 위치를 모르는 baseline 라인으로 도니 낮게 출발하고, 스왑 뒤엔
     라인 자체가 장애물을 비켜 가므로 올린다. (기동 기본값이 8.0이던 시절에는 같은
     7.0이 "내리는" 값이었다. 그 시절 서술이 문서에 남아 있었다.)
   - control_map_node는 **max_speed에 한해** 런타임 파라미터 변경을 수용한다
     (그 외 파라미터는 기존대로 생성자 1회 읽기 — 런타임 set 시 거부하지 않고 경고만
     찍으므로 `ros2 param get`은 새 값을 보여주지만 제어는 안 바뀐다).
     수신 측 상세는 `f1tenth_control/CLAUDE.md` ②-t.
   - 양의 유한 double이 아니면 control이 `reason`을 담아 거부하고, 이 노드가 그
     `reason`을 ERROR 로그(`max_speed=%.2f rejected by control: ...`)로 찍는다.
   - 전송 실패·미준비 시 AVOID 게이트와 같은 tick 재시도 경로를 쓴다.

## 3. 구독·발행·서비스

| 방향 | 이름 | 타입 | 용도 |
| --- | --- | --- | --- |
| 구독 | `/adaptive_obstacle_map` | `f110_msgs/ObstacleArray` | persistent confirmed 장애물 (Cartesian AABB 계약, Frenet은 노드가 P0로 투영) |
| 구독 | `/global_waypoints` | `f110_msgs/WpntArray` | 판정·투영 기준선 (최초 1회 = 불변 P0, adapter+CLCS 원자 캡처) |
| 구독 | `/lap_count` | `std_msgs/Int32` | 트리거·스왑 랩 경계 |
| 발행 | `/map_creator/status` | `std_msgs/String` | 단계·결과 |
| 클라이언트 | `/global_planning/reload_waypoints` | `std_srvs/Trigger` | 원자 스왑/롤백 |
| 클라이언트 | `state_machine_node/set_parameters` | `rcl_interfaces/SetParameters` | 스왑 성공 후 AVOID 게이트 갱신 |
| 클라이언트 | `control_map_node/set_parameters` | `rcl_interfaces/SetParameters` | 스왑 성공 후 max_speed 상한 전송 (`swap_max_speed_mps`) |

## 4. 주요 파라미터 (`config/map_creator.yaml`)

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `trigger_lap_count` | 2 | freeze 트리거 랩 (랩2 완주 = 1→2) |
| `match_max_ds_m` | 1.0 | P0 투영 Frenet s의 wrap-aware 매칭 상한 |
| `match_max_dd_m` | 0.3 | P0 투영 Frenet d의 매칭 상한 |
| `reference_alignment_tolerance_m` | 0.05 | P0 캡처 게이트: CLCS(기하 호길이) vs painter(s_m 보간)의 트랙길이·s 정합 허용치 |
| `max_obstacle_projection_d_m` | 2.0 | 장애물 중심 투영 \|d\| 상한 (branch/이상치 거부, CLCS max_projection_distance에도 적용) |
| `ego_lookback_m` | 12.0 | 판정 ego 위치 (최대 entry 11.43 m 절단 방지) |
| `decision.*` | (스냅샷) | 좌/우 판정 파라미터 전체 — 미제시 항목은 배포 local_planning 값 |
| `base_map_yaml` | `src/monte_carlo_localization/maps/map.yaml` | 페인팅할 원본 ROS map YAML |
| `output_map_name` | obstacle_map | 저장·생성·리로드가 공유하는 디렉터리 이름 |
| `initial_morph_kernel` | 1 | 1차 패스 morph_kernel 오버라이드 (≤0 = gui_params 값). 1이면 open/close 무효화로 페인팅 형상 보존 — 코너 케이스 이음새 κ 폭주 방지 |
| `retry_morph_kernel` | 1 | 재시도 패스 morph_kernel 오버라이드 (≤0 = gui_params 값) |
| `reseed_on_startup` | true | 시작 시 obstacle_map을 baseline 사본으로 재시딩 |
| `control_node_name` | control_map_node | 스왑 후 max_speed를 보낼 control 노드 이름 |
| `swap_max_speed_mps` | 7.0 | 장애물 라인 스왑 성공 직후 control max_speed 상한 [m/s] (≤0 = 미전송) |
| `rollback_max_speed_mps` | 0.0 | baseline rollback 스왑 시 복원 값 [m/s] (≤0 = 미전송, 스왑 값 유지) |
| `min_obstacle_clearance_after_m` | 0.42 | 새 라인↔장애물 최소 이격 (로컬 침묵 조건) |
| `initial_smooth_sigma` | 4.1 | 1차 생성 smooth_sigma |
| `retry_safety_width` | 0.4 | 게이트 실패 시에도 유지하는 safety_width |
| `retry_smooth_sigma` | 2.5 | 게이트 실패 시 1회 재시도 smooth_sigma |
| `max_swap_deferral_laps` | 3 | 리로드 서비스 미준비 시 이월 상한 |
| `disable_avoid_after_swap` | true | 스왑 성공 후 state_machine AVOID 게이트 자동 갱신 |
| `state_machine_node_name` | state_machine_node | 게이트 갱신 대상 노드 이름 |

## 5. 실행 방법

```bash
cd ~/2026_IFAC                    # 상대경로 규약: 워크스페이스 루트에서 launch
source /opt/ros/jazzy/setup.zsh
colcon build --packages-up-to map_creator
source install/setup.zsh
ros2 launch global_planning global_planning.launch.py
```

`global_planning.launch.py`가 `map_creator.launch.py`를 포함하며, 이 launch가
`static_obstacle_map`도 함께 실행합니다. 두 노드를 별도 터미널에서 중복 실행하지 않습니다.
단독 디버깅이 필요할 때만
`ros2 launch map_creator map_creator.launch.py`를 사용합니다.

전제: global_planning(리로드 서비스와 lap_counter 포함)·obstacle_detector가 함께 떠 있어야 하며,
`offline_trajectory_generator/output/obstacle_map/`
은 baseline 사본으로 시딩되어 있어야 합니다(`reseed_on_startup: true`가 자동 수행).
드라이버 의존성: `python3-numpy`, `python3-opencv`, `python3-yaml`, `python3-tk`
(trajectory_gui import에 필요), `scipy`.

## 6. 산출물 (`offline_trajectory_generator/output/obstacle_map/`)

- `obstacle_map.png` / `obstacle_map.yaml` — 페인팅된 생성기 입력 맵
- `global_waypoints.json` / `metadata.json` — 재생성된 글로벌 라인 (republisher가 읽음)
- `obstacles.json` — 베이크된 장애물 AABB (드라이버의 이격 게이트 입력)
- `gate_report.json` — 게이트별 통과 여부·수치, `regen_log.txt` — 드라이버 로그
- `manifest.json` — 트리거 랩, 장애물별 판정·사유, 상태
- `obstacle_debug_overlay.png` — 칠해진 맵 위에 centerline·재생성 레이스라인을 그린 확인용
  이미지. 노드가 드라이버에 `--preview-png`를 넘겨 생성합니다. 베이스라인 생성 경로의
  `output/map/debug_overlay.png`와 짝이 되는 파일입니다.

## 7. 단계별 확인

1. 랩 1·2 주행 중 `/map_creator/status`가 `idle`인지,
   `/adaptive_obstacle_map` 수신을 확인합니다.
2. 랩 2 완주(랩 2→3 전이) 시 status가 `generating`으로 바뀌고 `regen_log.txt`가 자라는지 확인합니다.
3. `gate_report.json`의 모든 게이트 PASS 후 status `armed`를 확인합니다.
4. 다음 랩 경계에서 `swapped: reloaded N waypoints ...`와
   RViz의 글로벌 라인이 장애물 밖으로 이동하는 것을 확인합니다.
5. `output/map/global_waypoints.json`이 변경되지 않았고, republisher의 `map_name` 런타임
   파라미터가 `obstacle_map`으로 바뀌었는지 확인합니다.
6. 랩 2+에서 로컬 플래너가 해당 장애물에 재개입하지 않는지 확인합니다.
7. 스왑 직후 status `avoid gate disabled (obstacle line active)`와
   `ros2 param get /state_machine_node allow_avoid_transition`이 `false`인지
   확인합니다 (baseline rollback 스왑 후에는 다시 `true`).
