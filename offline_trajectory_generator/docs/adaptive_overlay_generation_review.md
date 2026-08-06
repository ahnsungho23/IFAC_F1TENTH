# adaptive_overlay_generation_proposal.md 엄밀성 검토 보고서

검토일: 2026-08-06
방법: 6-에이전트 검증 — 코드 사실 확인 3(플래너 API / 생성기 API / 데이터셋 기하 정량 분석)
+ 평가 렌즈 3(의미론 / 파이프라인 / 데이터셋 설계). 모든 판정은 소스 코드·실측 실행·기존
산출물(`adaptive_overlays_13:45`, 1,562장)로 교차 검증했다.

주: 검토 진행 중 워킹트리에 `evaluateObstacleScenario`(+`force_visible_cluster` 게이트 우회,
`raceline_spline_planner.cpp:1317`)와 `adaptive_side_evaluator.cpp`, `generate_adaptive_overlays.py`가
이미 구현되기 시작했다. 본 보고서의 결함 지적은 **문서 스펙** 기준이며, 구현이 진행 중인
항목은 그대로 스펙에 반영하면 된다.

---

## 0. 총평

**제안의 핵심 원칙은 옳고, 실현 비용 평가도 정확하다.** 특히:

- **원칙 1·2 (Python 복제 금지, C++ 직접 재사용)** — 검증 결과 기존 13:45 run은 Python
  복제기 산출물이 거의 확실하고(시나리오당 ~0.41 s로 mincurv 실행 불가능한 속도, 당시 C++
  평가기 부재, 생성 스크립트가 디스크 어디에도 없음), d<0 열 710건에서 right가 **0건**인
  극단 패턴 등 복제기 편향 의심 지점이 실재한다. 플래너는 이미 rclcpp 무의존 순수 클래스라
  (gtest가 ROS 없이 생성 중) 재사용 비용도 낮다: 리팩터 ~150–250줄 + CLI ~250–350줄.
- **§13 off-by-one** — 실행 재현으로 **확정된 진짜 버그**(off-map 발생 시에만 IndexError —
  검출해야 할 바로 그 순간 검사기가 죽음). 수정안 `s_dense[:-1][off]`도 정확하다.
  (표현만 정정: 길이 차이는 "달라질 수 있"는 게 아니라 항상 정확히 1.)
- **§8 run 위생 설계** — manifest가 하나도 없는 기존 run의 실패 경험을 정확히 겨냥한다.
- **§5 벽-연결 polygon** — 부수 효과로도 옳다: 장애물 사각형만 칠하면 16 px(4×4 @0.05 m/px)로
  `cleanup_free_mask`의 speckle 보호 문턱 25 px 미만이라 median blur가 되살릴 수 있는데,
  벽과 연결하면 대형 벽 컴포넌트로 보호된다.

**그러나 구현 착수 전 반드시 해소해야 할 치명 결함 4건이 있다** (§1 참조):
① 현재 파라미터로는 **무수정 맵 베이스라인부터 §9 성공 조건이 실패**한다,
② safe_stop 521건(33%)에서 mincurv가 **반드시 예외로 죽는데** 처리 경로가 미정의다,
③ guard 정책·비교 도메인 미정으로 §14 시험 1이 성립하지 않는다,
④ 시나리오마다 완전한 경로+lap_time을 계산해 놓고 **PNG만 남기고 버린다**.

---

## 1. 치명 결함 (구현 전 결정 필수)

### 1.1 파라미터 기준선 모순 — 베이스라인부터 실패 [§6+§9+§15]

문서의 9개 파라미터 값은 현재 `gui_params.yaml`과 전부 일치한다(§6 자체는 정확). 그러나
저장된 142-waypoint 기준선(`ruleset_adaptive_globalpath/map/metadata.json`)은
**smooth_sigma 4.5**로 생성된 것이고 현재 YAML은 **2.5**다. 실측: 현재 YAML로 무수정 맵에
돌리면 **144 waypoints + kappa_violations=5** (max|κ|=0.685 > 0.65).

- §1의 idx 000~141 격자(142개)와 재생성 waypoint 수(144)가 어긋나고,
- §9 성공 조건 4(kappa_violations==0)가 **장애물 없는 베이스라인에서조차 실패**해
  사실상 전 시나리오가 outline 폭포수로 쏟아진다.

**결정 필요**: 기준선 재현(smooth_sigma 4.5)으로 스냅샷 고정 vs 현재 YAML로 격자 재정의.
어느 쪽이든 §14에 **시험 0 — 무수정 맵 스모크 테스트**(waypoint 수 일치 + off_map=0 +
κ=0 통과 후에만 시나리오 루프 진입)를 추가할 것.

### 1.2 safe_stop 521건의 생성 동작 미정의 [§5+§7.2]

양쪽을 벽까지 칠하면 free space가 두 조각으로 분리되고, `cleanup_free_mask`는 **최대 연결
컴포넌트 하나만 유지**(generate_global_trajectory.py:381-386) → 폐곡선 centerline 추출이
`RuntimeError`로 **반드시** 실패한다. 문서는 safe_stop에서 `generate_trajectory()`를
돌리는지, PNG에 무엇을 그리는지 한 번도 명시하지 않았다.

**권고**: mincurv 미실행 + PNG = 양쪽 칠한 맵 + 기준 raceline 오버레이 +
`base_generation_status=not_attempted` 명시. 부수 효과로 521 × 3.4 s ≈ 30% 시간 절약.

### 1.3 guard 정책·비교 도메인 미정 — §14 시험 1이 성립 안 함 [§3.1+§4]

- 런타임 노드는 plan() **이전에** `buildUncertaintyGuard`로 장애물을 선팽창한다
  (분산 0이어도 횡 +0.03 / 종 +0.05 바닥값, 횡 cap 0.15). §3.1은 raw 박스만 정의하므로
  오프라인이 런타임보다 체계적으로 관대해져 LEFT/RIGHT vs SAFE_STOP이 정당하게 갈릴 수 있다.
- 0.2 m 장애물 기준 blocking은 |d| ≤ 0.321 → 격자 d=±0.4, ±0.5(**568/1,562건**)에서
  런타임 plan()은 kNoObstacle을 반환하므로 동등성 비교 자체가 불가능하다.
- 배포 yaml이 struct 기본값과 2곳에서 다르다(obstacle_longitudinal_padding_m 0.3 vs 0.35,
  post_merge_lookahead_m 5.0 vs 2.0) — CLI가 struct 기본값으로 파라미터를 만들면 조용히
  어긋난다.

**권고**: ① guard 정책 명문화(양쪽 모두 raw, 또는 양쪽 모두 zero-variance guard — 런타임
정합이 목적이면 후자), ② 시험 1의 도메인을 blocking 부분집합(d∈{−0.3..+0.3}, 994건)으로
제한하고 "994건 100% 일치"를 §15 완료 기준으로 승격, ③ CLI는 반드시
`local_planning_snapshot.yaml`에서 `RacelineSplineParameters`를 초기화, ④ 공통 API 범위를
"evaluate_side + tie-break(side_tie_epsilon 0.02→headroom) + tight-clearance 재시도
(0.18, 이 경로의 게이트 우회 포함)"로 못박기 — 워킹트리의 `force_visible_cluster`가 재시도
경로(cpp:1243)까지 우회하는 것을 확인했으며 이 구현이 옳다.

### 1.4 PNG-only 출력 — 계산한 경로를 버림 [§1+§8]

프로그램은 시나리오마다 완전한 mincurv 경로(`GenerationResult.global_traj`: xy, s, ψ, κ, vx,
d_left/right)와 lap_time을 생성해 놓고 **렌더된 PNG만** 남긴다. 이 데이터셋의 개연성 높은
용도(런타임 adaptive global path 룩업, 형제 설계의 rule-based 베이스라인)에는 waypoint가
필요한데, 그 시점엔 전량 재생성(수십 분 + 파라미터 드리프트 위험)해야 한다. 시나리오당
CSV/JSON ≈ 30 KB, 1,562개 ≈ 50 MB로 비용이 사실상 0이다.

**권고**: ① §1에 목적 절 신설(1차 소비자 명시), ② 시나리오별 waypoint + lap_time을
images/와 병렬 저장, ③ (강력 권장) **양측 페인팅으로 mincurv 2회** 실행해
`lap_time_left/right`를 기록 — "로컬 기하 판정 vs 랩타임 우세 사이드"의 **불일치 집합**이
이 데이터셋의 과학적으로 가장 가치 있는 산출물이며, 형제 문서(랩타임 기준 사이드 선택)와의
권위 충돌을 데이터로 판정하는 유일한 방법이다(추가 비용 8-worker ~25분).

---

## 2. 정량 사실 (실측)

### 2.1 격자 기하 vs 트랙 폭

트랙 35.30 m 폐루프, 142 wpts @0.25 m. d_left 0.30/1.00/1.925(min/med/max),
d_right 0.32/0.85/1.95. 폭 최소 0.620 m(idx 68, s=16.97).

| 분류 | 건수 | 예 |
|---|---:|---|
| 장애물 중심이 트랙 경계 밖 | 33 | idx 14–16 @d=−0.5 |
| 장애물(±0.1 m)이 벽과 겹침 | 75 | idx 13–16 @−0.5 |
| 양쪽 모두 corridor<0.44 m (mincurv 불능) | 14 | idx 67–69 전부 |
| corridor≥0.44: 양쪽/좌만/우만 가능 | 906 / 387 / 255 | — |

→ **§3.3 유효성 사전검사 신설** 필요: `invalid_obstacle_position` / `obstacle_wall_overlap` /
`geometric_no_pass`를 1급 라벨로 분리(플래너 판정·페인팅·mincurv 스킵). 대칭 d 격자는
비중심 레이스라인 때문에 좌우를 다른 비율로 커버함(+0.5=좌측 여유의 50%, −0.5=우측의 59%).

### 2.2 기존 run(13:45)과 기준 분포 주장

- 실측 파일 수: **left=748 / right=285 / safe_stop=521 / outline=8** (계 1,562).
  문서의 756은 outline 8건이 전부 left였다는 **미검증 가정**(748+8) 하에서만 정합 —
  manifest가 없어 확인 불가. §15는 이 파생 관계를 명시해야 한다.
- safe_stop 521건 중 폭 기하상 불능은 **14건뿐**. 297건(57%)은 폭이 정상(중앙값 1.725 m)인
  idx 59–85 연속 밴드에서 나옴 — 플래너 제약 체인 또는 복제기 보수성 산물. **이 밴드의
  원인 규명이 C++ 재실행의 첫 번째 관전 포인트**다.
- Python 복제기 기준선과 C++ 결과의 차이는 "회귀"가 아니라 **기대되는 교정**이다 —
  §15의 비교를 회귀 게이트가 아닌 diff 보고서(사유 분류 포함)로 강등하고, 첫 C++ run을
  신규 기준선으로 선언할 것.

### 2.3 맵 정체 함정 [§12]

- CLI에 맵 인자가 없다: 맵은 `gui_params.yaml`의 `map_yaml`
  (**/home/haejun/slam_toolbox/map.yaml**, 346×160 px)에서 암묵적으로 온다 — 문서에 명시 필요.
- **함정**: `ruleset_adaptive_globalpath/map.png`(139×343 px, origin [−1.373,−1.119])는 별개
  크롭본으로, CSV 좌표(x −14.24..0.04)가 그 extent에 들어가지 않는다. 이 파일을 칠하면
  조용히 틀린 좌표에 페인팅된다. `--map` 인자 명시 + 시작 시 "CSV bbox ⊂ 맵 extent" 검증 권고.
- **stale 기준선**: `--reference` CSV(142 wpts, 8/5 스냅샷)는 스택이 현재 발행하는
  `output/map/global_waypoints.json`(8/6 13:28 재생성, **144 wpts**, lap 7.156 s)과 이미
  다른 생성 run이다. "데이터셋은 스냅샷 시점 고정본에 바인딩되며 라이브 라인과 다를 수
  있음"과 CSV 해시 기록, 스택 라인 갱신 시 무효화 규칙이 필요하다.
- `generate_trajectory`는 디스크 경로만 받는다(load_map이 cv2.imread; in-memory 주입 API
  없음) → **시나리오별 임시 map.png + map.yaml 쌍** 프로토콜(원본 read-only, per-worker
  격리, painted-map SHA-256 기록)을 §5–6 사이에 명문화할 것.

### 2.4 페인팅 수치 스펙 [§5]

- "비주행 픽셀" 임계값 미정의: 생성기의 free 판정은 pixel ≥ 250, occupied(벽)는 ≤ 89,
  사이는 unknown. **ray march 정지 = pixel < 250**(free의 보수)으로 고정 권고. 칠하는 값
  0은 두 판정 모두 만족.
- `wall_not_found` 시: 장애물 사각형 단독 페인팅 금지(16 px < speckle 보호 25 px → cleanup이
  되살려 경로가 장애물을 관통해도 §9를 통과하는 **유령 성공** 가능). 생성 스킵 + 전용 실패
  클래스로.
- 단일 waypoint 법선 근사는 타당(max|κ|≈0.72에서 모서리 오차 ≈4 mm, 서브픽셀).

### 2.5 ego 프로토콜 [§3.2]

- 7.0 m lookback이면 cluster_start ≈ 6.6 m: 안쪽 entry 스케일 1.25/1.5가 절단되고
  **바깥쪽(×1.35)은 세 후보 전부 6.6 m로 절단**되어 기하적으로 동일한 시도 3회가 된다.
  런타임 최초 인지(12 m)에서는 성립했을 사이드가 7 m 데이터셋에서 무효가 되는 보수적
  편향 — `ego_lookback_m=12.0`(detection_lookahead와 일치) 채택 또는 {7,12} 스윕 비교 검토.
- `ego.speed`는 post-merge tail 길이에만 쓰여 **판정 불변** — 이 단순화를 명시하면 speed
  프로토콜 선택이 결과에 무영향임이 재현성 스펙으로 남는다.

### 2.6 outline 폴백 [§7.3]

- `smooth_sigma 2.5` override는 현재값과 동일한 **no-op** — 실질 override는
  boundary_margin 하나뿐(기준 파라미터가 4.5로 바뀌면 no-op이 아니게 되므로 의도 명시 필요).
- boundary_margin 0.2→0.40은 **벽-스침 off-map에만** 유효(코드의 WARN 안내와 일치).
  칠한 장애물 옆 좁은 통로에서는 **역효과**: bounds가 더 조여지고, 폭이 전 구간 소진되면
  `optimize_min_curvature`가 최적화 없이 **centerline을 그대로 반환**(845-846행) → 경로가
  칠한 장애물을 관통 → side_failure로 악화. **"centerline fallback 감지 시 무조건 실패 처리"
  가드 필수.** centerline 추출 실패·스무딩 후 bounds 이탈은 어차피 못 고침 → 실패 원인별
  조건부 재시도로 재설계.
- 0.40은 GUI 슬라이더 최대 0.25 초과지만 normalize가 상한 클램프를 안 해 프로그램 호출로는
  통과(실측) — 검증 범위 밖 값임을 주석으로 기록.

### 2.7 §9 성공 조건 세부

- 조건 5의 tolerance 0.05는 **side 라벨링 전용**(LEFT 문턱 = 장애물 왼쪽 edge − 0.05 →
  5 cm 겹침 허용). 실제 안전은 칠한 장애물=occupied → 폭 측정 → mincurv bounds(0.4 m) +
  조건 6에 위임 — 이 분해를 명시할 것. 단 스무딩(raceline_smooth_sigma)이 bounds **이후**에
  적용되므로 "densified path→칠한 장애물 픽셀 최소 거리 ≥ vehicle_half_width 0.121 m"
  한 줄 검증 추가 권장.
- 조건 3·6은 §13 수정에 의존함을 명시(현재는 off-map 발생 시 검사기가 IndexError로 사망).

### 2.8 실행 예산·결정론 [§12+§14]

- 실측 mincurv 1회 3.37 s → 1,562건 직렬 ~88분, 8-worker 이상적 ~11분, 오버헤드 포함
  현실적 15–30분. safe_stop 스킵 시 ~30% 단축, 양측 실행 채택 시 2배. 기존 run이 10.6분에
  끝난 것은 mincurv를 안 돌렸기 때문 — 기대치 설정 필요.
- §14 시험 5(직렬==병렬 해시)의 핀: per-worker `OMP_NUM_THREADS=OPENBLAS_NUM_THREADS=1`,
  multiprocessing spawn, per-worker temp dir, cv2.imwrite PNG 압축 파라미터 고정, C++ 평가기
  JSON은 페인팅 전에 1회 직렬 생성.
- §8.3 유령 상태: rename과 _SUCCESS 생성 사이 크래시 시 "_SUCCESS 없는 완성 run"이 남고
  resume 경로가 없다 → **_incomplete 안에서 _SUCCESS를 먼저 쓰고 rename을 커밋으로** 삼을 것.
  `--resume-run`은 현재 yaml vs 스냅샷 SHA-256 동일성 검증(불일치 시 중단) 필수 — 없으면
  한 run 안에 다른 파라미터 산출물이 조용히 섞인다.
- §10 manifest 보강: 원본 map.png/yaml SHA-256, reference CSV SHA-256, 시나리오별
  painted-map SHA-256, **플래너·생성기 git commit**(현재 ruleset_adaptive_globalpath와 문서
  디렉터리가 git 미추적 — 먼저 추적 전환), 평가기 바이너리 식별자, numpy/scipy/cv2 버전,
  **좌·우 각각의 target_d**(현재는 결정측 하나뿐).

---

## 3. 권고 우선순위

1. **[착수 전]** 파라미터 기준선 확정 + 시험 0(무수정 맵 스모크) — §1.1
2. **[착수 전]** guard 정책·비교 도메인·파라미터 소스(local_planning_snapshot 로드) 확정 — §1.3
3. **[스펙]** safe_stop 파이프라인 정의(mincurv 미실행) — §1.2
4. **[스펙]** §3.3 유효성 사전검사 + 퇴화 라벨 3종(33/75/14건) — §2.1
5. **[스펙]** temp-map 프로토콜 + `--map` 명시 + ruleset 크롭본 사용 금지 경고 — §2.3
6. **[스펙]** 페인팅 임계값(<250)·wall_not_found 스킵·centerline-fallback 실패 가드 — §2.4, §2.6
7. **[확장]** waypoint+lap_time 저장, 양측 mincurv로 불일치 집합 산출 — §1.4
8. **[문서]** §15 기준 분포를 "748+outline 8(사이드 미검증)"로 정정, Python 복제기 대비
   diff 보고서로 강등, 첫 C++ run을 신규 기준선으로 — §2.2
9. **[인프라]** _SUCCESS-then-rename, resume 스냅샷 검증, 결정론 핀, manifest 해시 보강 — §2.8

---

## 부록 — 근거 (주요)

- 플래너: `src/local_planning/src/raceline_spline_planner.cpp` (evaluate_side 1137-1171,
  blocking gate 274-302/1118-1124, tight retry 1229-1243; 워킹트리 신규:
  evaluateObstacleScenario 1317+, force_visible_cluster 1243), `config/local_planning.yaml`
  (yaml-vs-struct 차이 2건), `obstacle_guard.cpp:54-88` (guard 바닥값)
- 생성기: `generate_global_trajectory.py` (load_map 283-329 임계값, cleanup 364-386,
  centerline 480-497, mincurv bounds 829-846, off-by-one 934-957 — IndexError 실측 재현,
  스무딩 1499-1503, WARN 1516-1536), `trajectory_gui.py` (API 4종 실재·스니펫 실행 성공)
- 데이터: `ruleset_adaptive_globalpath/map/global_waypoints.csv` (142 wpts, 폭 통계),
  `adaptive_overlays_13:45/` (1,562장 파일명 카운트·mtime 타임라인),
  `gui_params.yaml` vs `map/metadata.json` (smooth_sigma 2.5 vs 4.5),
  무수정 맵 실측: 144 wpts + kappa_violations=5, mincurv 3.37 s/회
