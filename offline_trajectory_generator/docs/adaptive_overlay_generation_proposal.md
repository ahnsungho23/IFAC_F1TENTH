# Adaptive Global Path 이미지 생성 프로그램 제안서

## 1. 목적

이 문서는 트랙의 모든 글로벌 waypoint와 장애물 횡방향 위치 조합에 대해 로컬 플래너의
좌·우 통과 가능성을 판정하고, 판정 결과를 반영한 수정 맵에서 min-curvature 글로벌 경로를
생성하는 오프라인 프로그램을 제안한다.

생성 대상은 다음과 같이 고정한다.

```text
글로벌 waypoint index: 000 ~ 141              142개
장애물 중심 d:          -0.5 ~ +0.5, 0.1 간격  11개
전체 시나리오:          142 × 11             1,562개
```

각 시나리오는 최종 PNG를 정확히 한 장만 가진다. 결과 PNG는 실행 시작 시간을 나타내는
run 디렉터리 아래에 보관하며, 재실행 시 기존 결과를 덮어쓰지 않는다.

## 2. 핵심 원칙

1. Python으로 좌·우 판단 공식을 다시 구현하지 않는다.
2. `local_planning`의 `RacelineSplinePlanner` C++ 로직을 직접 재사용한다.
3. 글로벌 경로 생성은 `trajectory_gui.py`가 사용하는 동일한 Python 백엔드를 호출한다.
4. GUI 창을 1,562번 실행하지 않고 `generate_trajectory()`를 직접 호출한다.
5. 실행 시작 시 파라미터를 한 번 고정하고 1,562개 시나리오에 동일하게 적용한다.
6. 한 시나리오당 PNG는 한 장만 저장하며, 샘플이나 보고서용 PNG를 복제하지 않는다.
7. 성공뿐 아니라 실패도 PNG와 manifest에 남긴다.

## 3. 입력 시나리오

### 3.1 장애물 형상

장애물 기본 크기는 0.20 m × 0.20 m로 설정한다.

```text
s_start = obstacle_s - 0.10 m
s_end   = obstacle_s + 0.10 m
d_right = obstacle_d - 0.10 m
d_left  = obstacle_d + 0.10 m
```

장애물 중심의 Cartesian 위치는 해당 글로벌 waypoint의 법선으로 이동하여 계산한다.

```text
obstacle_xy = global_xy + obstacle_d × global_normal
```

### 3.2 평가용 ego 상태

모든 장애물을 동일한 조건으로 비교하기 위해 기본 ego 상태를 다음과 같이 고정한다.

```text
ego.s     = wrap(obstacle.s - 7.0 m)
ego.d     = 0.0 m
ego.speed = ego 위치의 글로벌 waypoint 속도
preferred_left = 없음
allow_side_switch = true
```

`ego_lookback_m`은 실행 설정으로 노출하지만, 기준 데이터셋 생성에서는 7.0 m로 고정하고
파라미터 스냅샷에 기록한다.

## 4. 로컬 플래너 좌·우 판정

`RacelineSplinePlanner`의 다음 절차를 그대로 사용한다.

```text
장애물 입력
  ├─ 왼쪽 target d 계산
  │   ├─ 장애물 구간 track-bound 사전검사
  │   └─ 전체 spline 경계·충돌·기울기·곡률 검사
  ├─ 오른쪽 target d 계산
  │   ├─ 장애물 구간 track-bound 사전검사
  │   └─ 전체 spline 경계·충돌·기울기·곡률 검사
  └─ score와 reference headroom 비교
      ├─ LEFT
      ├─ RIGHT
      └─ SAFE_STOP
```

런타임에서는 글로벌 라인을 실제로 막는 장애물만 `nearestCluster()`가 선택한다. 그러나 이
데이터셋은 `d=-0.5~+0.5`의 모든 위치에서 좌·우 통과 가능성을 비교해야 한다. 따라서 오프라인
평가기에서는 blocking gate만 우회하고, target 계산 이후의 좌·우 판정·검증·점수 계산은 런타임과
동일한 공통 함수를 사용한다.

이를 위해 좌·우 선택 부분을 다음과 같은 공통 API로 분리한다.

```cpp
SideEvaluationResult evaluateObstacleScenario(...);
```

런타임 `plan()`과 오프라인 평가기가 같은 구현을 호출하게 하여 Python 복제 로직과의 불일치를
방지한다. 결과에는 좌·우 유효 여부, target d, 실패 종류와 상세 사유를 구조화하여 담는다.

## 5. 수정 맵 생성 규칙

로컬 플래너가 선택하지 않은 쪽을 장애물에서 실제 맵 벽까지 검은색으로 채운다.

| 로컬 판정 | 검은색으로 채우는 쪽 | 열어 두는 쪽 |
|---|---|---|
| `left` | 장애물 오른쪽 | 왼쪽 |
| `right` | 장애물 왼쪽 | 오른쪽 |
| `safe_stop` | 장애물 양쪽 | 없음 |

벽 연결 절차는 다음과 같다.

1. 글로벌 waypoint의 `psi_rad`로 tangent와 normal을 계산한다.
2. 장애물 측면을 종방향으로 최소 9개 점으로 샘플링한다.
3. 각 점에서 선택된 normal 방향으로 맵 해상도의 1/4 간격으로 ray march한다.
4. 첫 번째 비주행 픽셀을 벽으로 판정한다.
5. 장애물 측면과 벽 샘플을 polygon으로 연결하여 검은색으로 채운다.
6. 마지막에 장애물 사각형을 검은색으로 그린다.

벽을 찾기 전에 이미지 경계를 벗어나면 자동 성공으로 처리하지 않고 `wall_not_found`를 기록한다.
`safe_stop`은 양쪽을 모두 채워 트랙을 완전히 차단한 상태를 시각화한다.

## 6. Min-curvature 글로벌 경로 생성

프로그램은 `offline_trajectory_generator/gui_params.yaml`을 읽고 `optimizer=mincurv`를 강제한다.

```python
values = load_gui_params(params_path)
values["optimizer"] = "mincurv"
args = make_namespace(values)
result = generate_trajectory(args)
rgb = render_preview_rgb(result)
```

현재 기준 주요 파라미터는 다음과 같다.

```yaml
optimizer: mincurv
width_mode: hybrid
waypoint_step: 0.25
optimizer_step: 0.46
raceline_smooth_sigma: 1.0
safety_width: 0.4
boundary_margin: 0.2
smooth_sigma: 2.5
max_curvature: 0.65
```

실행 시작 시 YAML 전체를 메모리에 고정하고 `parameters/gui_params_snapshot.yaml`에 복사한다.
실행 도중 원본 YAML이 변경되더라도 진행 중인 run에는 영향을 주지 않는다.

## 7. 결과 분류

### 7.1 일반 성공

- `left`: 로컬 플래너가 왼쪽을 선택하고 기본 mincurv 경로가 왼쪽 통과에 성공
- `right`: 로컬 플래너가 오른쪽을 선택하고 기본 mincurv 경로가 오른쪽 통과에 성공

```text
idx_000_d_m05_left.png
idx_001_d_p03_right.png
```

### 7.2 Safe-stop

양쪽 로컬 spline이 모두 실패한 경우이다. 장애물 양쪽을 벽까지 막고 safe-stop 결과를 표시한다.

```text
idx_009_d_p04_safe_stop.png
```

### 7.3 Outline

로컬 플래너는 한쪽을 통과할 수 있다고 판정했지만 기본 글로벌 경로 생성이 실패하면 outline
재계산을 수행한다. outline은 현재 GUI 스냅샷에서 다음 값만 명시적으로 override한다.

```yaml
boundary_margin: 0.40
smooth_sigma: 2.5
```

이 값은 `parameters/outline_overrides.yaml`에 저장한다. Outline 파일명은 기존 요청 형식을
유지한다.

```text
idx_002_d_m01_outline.png
idx_010_d_p00_outline.png
```

### 7.4 Outline 성공 2종류

1. `outline_success_left`
   - 로컬 선택이 왼쪽이다.
   - outline 경로가 장애물 왼쪽을 통과한다.
   - `off_map_waypoints=0`, `kappa_violations=0`이다.

2. `outline_success_right`
   - 로컬 선택이 오른쪽이다.
   - outline 경로가 장애물 오른쪽을 통과한다.
   - `off_map_waypoints=0`, `kappa_violations=0`이다.

### 7.5 Outline 실패 2종류

1. `outline_geometry_failure`
   - 글로벌 경로 추출 예외
   - 폐곡선 생성 실패
   - off-map waypoint 발생
   - 곡률 제한 위반
   - 벽 연결 실패

2. `outline_side_failure`
   - 글로벌 경로는 생성됐지만 선택 방향과 반대로 통과
   - 장애물 또는 검은 벽과 교차
   - 장애물 위치의 `path_d`가 선택 방향 조건을 만족하지 못함

실패 세부 원인은 `failure_detail`로 별도 기록한다. Outline 성공·실패 모두 basename은
`idx_NNN_d_pNN_outline.png` 형식을 유지하고, 성공 종류와 실패 종류는 하위 디렉터리와 manifest로
구분한다.

## 8. 시간 라벨 기반 출력 폴더 구조

### 8.1 Run ID

프로그램 시작 시 Asia/Seoul 시간대를 사용해 run ID를 한 번 생성한다.

```text
run_YYYYMMDD_HHMMSS_mmm_KST
```

예시는 다음과 같다.

```text
run_20260806_143215_482_KST
```

`mmm`은 밀리초이다. 전체 1,562장은 동일한 run ID를 사용한다. PNG basename에는 시간을
추가하지 않아 기존 `idx_NNN_d_pNN_<label>.png` 규칙을 보존한다.

Manifest에는 사람이 읽을 수 있는 ISO 8601 시간도 함께 저장한다.

```json
{
  "run_id": "run_20260806_143215_482_KST",
  "started_at": "2026-08-06T14:32:15.482+09:00",
  "timezone": "Asia/Seoul"
}
```

### 8.2 디렉터리 구조

결과는 다음 구조로 저장한다.

```text
ruleset_adaptive_globalpath/
└── adaptive_overlays/
    ├── run_20260806_143215_482_KST/
    │   ├── images/
    │   │   ├── left/
    │   │   │   └── idx_000_d_m05_left.png
    │   │   ├── right/
    │   │   │   └── idx_001_d_p03_right.png
    │   │   ├── safe_stop/
    │   │   │   └── idx_009_d_p04_safe_stop.png
    │   │   └── outline/
    │   │       ├── success_left/
    │   │       │   └── idx_002_d_m01_outline.png
    │   │       ├── success_right/
    │   │       │   └── idx_087_d_p02_outline.png
    │   │       ├── failure_geometry/
    │   │       │   └── idx_120_d_p03_outline.png
    │   │       └── failure_side/
    │   │           └── idx_131_d_p01_outline.png
    │   ├── reports/
    │   │   ├── manifest.json
    │   │   ├── manifest.csv
    │   │   └── summary.json
    │   ├── parameters/
    │   │   ├── gui_params_snapshot.yaml
    │   │   ├── local_planning_snapshot.yaml
    │   │   ├── outline_overrides.yaml
    │   │   └── run_config.yaml
    │   └── _SUCCESS
    ├── run_20260806_162011_093_KST/
    │   └── ...
    └── latest_run.txt
```

모든 PNG는 `images/` 아래에만 존재한다. 따라서 다음 재귀 검색 결과가 반드시 1,562여야 한다.

```bash
find <run_dir>/images -type f -name '*.png' | wc -l
```

성공·실패 샘플은 별도 복사본을 만들지 않고 manifest가 해당 원본 PNG 경로를 가리키게 한다.
이렇게 해야 총 PNG 수가 1,562장을 넘지 않는다.

### 8.3 미완료 run 처리

생성 중인 결과는 다음 임시 위치에 저장한다.

```text
adaptive_overlays/_incomplete/run_20260806_143215_482_KST/
```

1,562개 PNG와 보고서 검증이 모두 끝난 뒤에만 완성된 run 디렉터리로 원자적으로 이동하고
`_SUCCESS` 파일을 생성한다. 중간 실패 결과는 `_incomplete`에 남겨 원인을 조사한다. 재실행할
때는 새 시간 라벨 run을 생성하므로 기존 부분 결과를 덮어쓰지 않는다.

기존 완료 run은 덮어쓰지 않는다. 완료 후 `latest_run.txt`에는 가장 최근 성공 run의 상대 경로만
기록한다.

## 9. 글로벌 경로 성공 조건

다음 조건을 모두 만족해야 성공이다.

1. `generate_trajectory()`가 예외 없이 완료된다.
2. 폐곡선 글로벌 경로가 생성된다.
3. `off_map_waypoints == 0`이다.
4. `kappa_violations == 0`이다.
5. 경로가 선택된 장애물 방향을 통과한다.
6. densified path 전체가 수정된 `free_mask` 안에 있다.
7. PNG가 정상적으로 저장되고 다시 읽을 수 있다.

선택 방향은 장애물 waypoint의 tangent 평면에서 경로가 교차하는 `path_d`로 확인한다.

```text
LEFT:  path_d > obstacle_d + obstacle_half_width - tolerance
RIGHT: path_d < obstacle_d - obstacle_half_width + tolerance
```

기본 `tolerance`는 0.05 m로 두고 run 설정에 기록한다.

## 10. Manifest와 요약 보고서

`manifest.json`과 `manifest.csv`에는 시나리오별로 다음 정보를 기록한다.

```text
run_id, started_at
index, obstacle_s, obstacle_d
ego_s, ego_d, ego_speed
left_valid, right_valid
left_reason, right_reason
planner_decision, target_d
painted_side
base_generation_status
outline_status, failure_detail
path_crossing_d
off_map_waypoints, kappa_violations
relative_png_path
generation_seconds
```

`summary.json`에는 다음 집계를 기록한다.

```text
총 시나리오 수
left/right/safe_stop planner 판정 수
left/right/safe_stop/outline 최종 PNG 수
outline_success_left/right 수
outline_geometry/side_failure 수
전체 실행 시간과 실제 PNG 수
```

GUI/local planner/outline 파라미터 원문은 `parameters/`에 snapshot으로 함께 저장한다.

## 11. 프로그램 구성

```text
src/local_planning/
├── src/adaptive_side_evaluator.cpp
├── test/test_raceline_spline.cpp
└── docs/adaptive_side_evaluator.md

offline_trajectory_generator/
├── generate_adaptive_overlays.py
├── test_adaptive_overlay_generator.py
└── docs/adaptive_overlay_generation_proposal.md
```

`adaptive_side_evaluator.cpp`는 실제 `RacelineSplinePlanner`를 사용해 1,562개 좌·우 판정과 실패
원인을 CSV로 출력한다. `generate_adaptive_overlays.py`는 이 결과를 읽어 수정 맵 생성,
mincurv 실행, 검증, 시간 라벨 디렉터리 배치와 보고서 생성을 담당한다.

## 12. 실행 인터페이스

```bash
python3 offline_trajectory_generator/generate_adaptive_overlays.py \
  --reference ruleset_adaptive_globalpath/map/global_waypoints.csv \
  --gui-params offline_trajectory_generator/gui_params.yaml \
  --local-params src/local_planning/config/local_planning.yaml \
  --output-root ruleset_adaptive_globalpath/adaptive_overlays \
  --workers 8 \
  --strict-count 1562
```

렌더링 전에 C++ 판정만 확인하거나 반복 제약 최적화의 목적함수를 계산할 때는 다음처럼 실행한다.

```bash
python3 offline_trajectory_generator/generate_adaptive_overlays.py \
  --evaluate-only \
  --planner-override transition_distance_scales=1.0,1.25,1.5 \
  --planner-override outside_line_transition_scale=1.35 \
  --planner-override commitment_clearance_reserve_m=0.05 \
  --planner-override minimum_avoidance_clearance_m=0.18
```

## 13. 선행 보완 사항

현재 `count_off_map_waypoints()`는 off-map 발생 시 densified point 배열과 `cumulative_s()` 결과의
길이가 하나 달라질 수 있다. 구현 전에 다음 방식으로 보완하고 회귀 테스트를 추가한다.

```python
hit_s = s_dense[:-1][off]
```

이 보완이 없으면 검출해야 할 off-map 사례에서 검사 함수 자체가 예외를 발생시킬 수 있다.

## 14. 시험 계획

1. C++ 평가기와 런타임 planner가 동일 입력에서 같은 방향과 target d를 반환하는지 확인한다.
2. 검은 polygon이 선택 반대편 장애물 면에서 실제 벽까지 연결되는지 픽셀 단위로 확인한다.
3. `safe_stop`에서 양쪽이 모두 닫히는지 확인한다.
4. 정상·off-map·반대 방향·추출 예외 fixture로 outline 4개 상태를 확인한다.
5. 직렬 실행과 병렬 실행의 manifest 판정과 PNG 해시가 같은지 확인한다.
6. 중단 run이 `_incomplete`에 남고 다음 실행이 새 run ID를 쓰는지 확인한다.
7. 동일 초에 두 번 시작해도 밀리초 run ID로 서로 다른 디렉터리가 생성되는지 확인한다.
8. 완료 run에 `_SUCCESS`가 있고 미완료 run에는 없는지 확인한다.

## 15. 완료 기준

- 완료 run의 `images/` 아래 PNG가 정확히 1,562장이다.
- `(index, d)` 조합의 누락과 중복이 모두 0건이다.
- 모든 PNG가 원본 맵과 같은 해상도이며 다시 읽을 수 있다.
- 모든 basename이 `idx_NNN_d_mNN|pNN_<label>.png` 규칙을 따른다.
- Outline basename은 `idx_NNN_d_mNN|pNN_outline.png` 형식을 따른다.
- PNG는 실행 시간으로 라벨링된 단일 run 디렉터리 아래에만 저장된다.
- 기존 완료 run을 덮어쓰지 않는다.
- 성공 run에 파라미터 스냅샷, manifest, summary와 `_SUCCESS`가 존재한다.
- 로컬 플래너 판정은 Python 복제 없이 C++ 공통 로직을 사용한다.
- 성공 경로는 `off_map_waypoints=0`, `kappa_violations=0`이다.
- Outline 성공 좌·우 2종류와 실패 2종류를 폴더와 manifest에서 구분할 수 있다.
- 현재 기준 planner 분포 `left=756`, `right=285`, `safe_stop=521`과 비교한 회귀 결과를 제공한다.
- 최종 `left/right/safe_stop/outline` 분포와 기존 outline 8개 변화 여부를 보고한다.

이 구조는 실행 시각별 데이터셋을 서로 독립적으로 보존하면서도 파일명 규칙과 총 1,562장 제한을
유지한다. 이후 파라미터를 변경해 다시 생성하더라도 run 디렉터리와 스냅샷을 통해 결과 차이를
정확하게 비교할 수 있다.
