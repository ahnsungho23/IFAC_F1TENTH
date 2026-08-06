# map_creator_node

## 1. 노드 목적

랩 1·2에서 확정된 정적 장애물을 글로벌 경로에 **영구 반영**하는 파이프라인 노드입니다.
랩 2→3 전이(`lap_count` 1→2) 시점에 장애물 원장을 freeze하고, local_planning의 좌/우 판정 로직
(`evaluateObstacleScenario`, C++ 공유 — Python 복제 없음)으로 **막을 쪽**을 정한 뒤,
그쪽을 맵에서 벽까지 검게 칠한 `obstacle_map`을 만들고, 오프라인 생성기로 글로벌
레이스라인을 재생성해, 다음 랩 경계에서 `/global_planning/reload_waypoints`로 원자
교체합니다. 설계 근거는 `learning_adaptive_globalpath/MAP_CREATOR_PROPOSAL.md` 참고.

## 2. 동작 원리 (FSM)

```text
IDLE ──(lap_count ≥ trigger, 원장 freeze)──▶ 판정+페인팅+저장 ──▶ GENERATING
  드라이버(regenerate_obstacle_map.py) 완료·물리 게이트 통과 ──▶ ARMED
  lap_count 다음 갱신 ──▶ reload 호출 ──▶ MONITORING
  (2랩 연속 미관측 장애물 → 원장 제거 → 재생성 / 전부 소실 → baseline 재시딩 + reload)
어느 단계든 실패 → ABORTED (기존 라인 유지, 로컬 회피가 계속 커버)
```

1. **원장(ledger)**: `/static_obs`의 `is_static` 장애물을 wrap-aware 매칭
   (|Δs|<1.0 m, |Δd|<0.3 m)으로 누적. `min_observations`(3) 이상 관측된 것만 freeze.
2. **판정**: map_creator 전용 튜닝 파라미터(`decision.*`)로 만든 플래너 인스턴스에서
   ego = (장애물 s − 12 m, d=0, v=해당 waypoint 속도)로 평가. 좌 통과 → 오른쪽을 막음,
   우 통과 → 왼쪽을 막음, safe_stop → 해당 장애물은 **베이크하지 않음**(양쪽을 막으면
   트랙이 폐쇄되어 centerline 추출이 불가하기 때문).
3. **페인팅**: 원본 맵의 새 사본에서(누적 편집 금지) 장애물 사각형 + 막는 쪽 면→벽
   polygon을 0으로 채움. "비주행" 판정은 픽셀 < 250 (생성기 free 판정의 보수).
   벽을 못 찾으면(`wall_not_found`) 전체 중단 — 장애물 단독 페인팅은 cleanup의
   speckle 보호(25 px)에 지워질 수 있어 금지.
4. **저장 = 생성기 입력**: `output/obstacle_map/obstacle_map.{png,yaml}` 저장 위치가
   그대로 드라이버의 `--map-yaml` 입력이 됨 (복사·수동 단계 없음).
5. **재생성**: 드라이버가 GUI와 동일한 `load_gui_params(gui_params.yaml)` 값으로
   `generate_trajectory` 실행 → 물리 게이트(폐곡선, off_map=0, s 순증가, |κ|≤3.2,
   장애물 이격 ≥0.42 m) 통과 시 `global_waypoints.json` + `gate_report.json` 기록.
   게이트 실패 시 `retry_safety_width`(현재 0.7)로 1회 재시도.
6. **스왑**: republisher가 기존 `output/map`은 그대로 둔 채 별도
   `output/obstacle_map/global_waypoints.json`을 읽고 검증한 후, 메모리 bundle과
   활성 참조 경로만 교체해 즉시 발행.

## 3. 구독·발행·서비스

| 방향 | 이름 | 타입 | 용도 |
| --- | --- | --- | --- |
| 구독 | `/static_obs` | `f110_msgs/ObstacleArray` | 랩1·2 장애물 원장 |
| 구독 | `/global_waypoints` | `f110_msgs/WpntArray` | 판정 기준선 (최초 1회 = 불변 P0) |
| 구독 | `/lap_count` | `std_msgs/Int32` | 트리거·스왑 랩 경계 |
| 발행 | `/map_creator/status` | `std_msgs/String` | 단계·결과 |
| 클라이언트 | `/global_planning/reload_waypoints` | `std_srvs/Trigger` | 원자 스왑/롤백 |

## 4. 주요 파라미터 (`config/map_creator.yaml`)

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `trigger_lap_count` | 2 | freeze 트리거 랩 (랩2 완주 = 1→2) |
| `ego_lookback_m` | 12.0 | 판정 ego 위치 (최대 entry 11.43 m 절단 방지) |
| `decision.*` | (스냅샷) | 좌/우 판정 파라미터 전체 — 미제시 항목은 배포 local_planning 값 |
| `base_map_yaml` | "" | 비우면 gui_params의 map_yaml 사용 |
| `output_map_name` | obstacle_map | 저장·생성·리로드가 공유하는 디렉터리 이름 |
| `reseed_on_startup` | true | 시작 시 obstacle_map을 baseline 사본으로 재시딩 |
| `min_obstacle_clearance_after_m` | 0.42 | 새 라인↔장애물 최소 이격 (로컬 침묵 조건) |
| `retry_safety_width` | 0.5 | 게이트 실패 시 1회 재시도 safety_width |
| `max_swap_deferral_laps` | 3 | 리로드 서비스 미준비 시 이월 상한 |

## 5. 실행 방법

```bash
cd ~/2026_IFAC                    # 상대경로 규약: 워크스페이스 루트에서 launch
source /opt/ros/jazzy/setup.zsh
colcon build --packages-up-to map_creator
source install/setup.zsh
ros2 launch global_planning global_planning.launch.py
```

`global_planning.launch.py`가 `map_creator.launch.py`를 포함하므로 별도 터미널에서
map creator를 중복 실행하지 않습니다. 단독 디버깅이 필요할 때만
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

## 7. 단계별 확인

1. 랩 1·2 주행 중 `/map_creator/status`가 `idle`인지, `/static_obs` 수신을 확인합니다.
2. 랩 2 완주(랩 2→3 전이) 시 status가 `generating`으로 바뀌고 `regen_log.txt`가 자라는지 확인합니다.
3. `gate_report.json`의 모든 게이트 PASS 후 status `armed`를 확인합니다.
4. 다음 랩 경계에서 `swapped: reloaded N waypoints ...`와
   RViz의 글로벌 라인이 장애물 밖으로 이동하는 것을 확인합니다.
5. `output/map/global_waypoints.json`이 변경되지 않았고, republisher의 `map_name` 런타임
   파라미터가 `obstacle_map`으로 바뀌었는지 확인합니다.
6. 랩 2+에서 로컬 플래너가 해당 장애물에 재개입하지 않는지 확인합니다.
