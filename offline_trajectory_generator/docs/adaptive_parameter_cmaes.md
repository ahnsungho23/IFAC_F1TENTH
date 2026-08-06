# Adaptive 로컬 플래너 CMA-ES 제약 최적화 구조

## 1. 목적

`optimize_adaptive_parameters.py`는 142개 글로벌 waypoint와 11개 장애물 횡방향 위치로 구성된
1,562개 조건에서 다음 목표를 순서대로 만족하는 로컬 플래너 파라미터를 찾는다.

1. `safe_stop` 개수를 먼저 최소화하고 가능하면 0으로 유지한다.
2. `safe_stop` 개수가 같은 후보 중 `minimum_avoidance_clearance_m`가 가장 큰 후보를 선택한다.
3. 같은 clearance에서는 축소 clearance 재시도 사용을 줄이고 선택 경로의 최소 headroom을 늘린다.

이 과정은 강화학습이 아니다. 상태별 행동 정책을 학습하지 않고 전체 조건에 공통으로 적용할 고정
파라미터를 찾는 **보상 기반 반복 제약 최적화**다. 탐색기는 CMA-ES를 사용하지만 좌우 통과와
`safe_stop` 판정은 Python에서 다시 구현하지 않고 실제 C++ `RacelineSplinePlanner`가 수행한다.

## 2. 안전 제약

현재 차량 반폭과 hard collision margin은 다음과 같다.

```yaml
vehicle_half_width_m: 0.121
hard_collision_margin_m: 0.03
obstacle_clearance_m: 0.25
```

따라서 탐색 가능한 축소 clearance 범위는 실행 시 다음 식으로 다시 제한한다.

```text
minimum = max(search_minimum, vehicle_half_width_m + hard_collision_margin_m)
maximum = min(search_maximum, obstacle_clearance_m)
```

현재 유효 범위는 `0.151~0.25 m`다. 탐색 YAML에 더 작은 값을 적어도 C++ 평가기에 전달되지
않는다. 운영 중인 `src/local_planning/config/local_planning.yaml`은 자동으로 덮어쓰지 않으며,
최적 결과는 별도 `best_parameters.yaml`로 저장한다.

## 3. 처리 구조

```text
config/adaptive_cmaes.yaml
          │ 초기값·탐색 범위
          ▼
optimize_adaptive_parameters.py
  CMA-ES ask ── 후보 N개 생성
          │
          ├── adaptive_side_evaluator ── 1,562개 C++ 판정
          ├── adaptive_side_evaluator ── 1,562개 C++ 판정
          └── adaptive_side_evaluator ── 1,562개 C++ 판정
          │
          ▼
제약 우선 순위 계산
  safe_stop → clearance → reduced retry → headroom
          │
          ▼
  CMA-ES tell ── 평균·공분산·step 갱신
          │
          └──────── 다음 세대 반복
```

Python은 후보별 C++ 프로세스를 병렬 실행하고 CSV를 집계한다. C++ 평가기는 런타임 로컬 플래너와
공통인 `evaluateObstacleScenario()`를 호출하므로 목표 `d`, spline, 트랙 경계, 장애물 충돌,
횡방향 slope, 곡률과 곡률 변화율 검사가 실제 로컬 planning 로직과 동일하다.

## 4. 탐색 파라미터

기본 탐색 설정은 `offline_trajectory_generator/config/adaptive_cmaes.yaml`에 있다.

| 파라미터 | 초기값 | 탐색 범위 |
|---|---:|---:|
| 첫 transition scale | 0.1 | 0.1~0.8 |
| 첫 scale 간격 | 0.5 | 0.05~1.2 |
| 둘째 scale 간격 | 1.4 | 0.05~1.8 |
| `outside_line_transition_scale` | 0.1 | 0.1~2.0 |
| `commitment_clearance_reserve_m` | 0.0 | 0.0~0.1 m |
| `minimum_avoidance_clearance_m` | 0.151 | 0.151~0.25 m |
| `boundary_margin_m` | 0.028 | 0.028~0.15 m |
| `minimum_target_offset_m` | 0.15 | 0.15~0.35 m |
| `maximum_lateral_slope` | 0.8 | 0.4~0.8 |

`transition_distance_scales`는 세 값을 직접 정렬하지 않는다. 첫 값과 양수 간격 두 개를 탐색해
다음과 같이 복원하므로 모든 후보에서 `t1 < t2 < t3`가 보장된다.

```text
t1 = first
t2 = first + first_gap
t3 = first + first_gap + second_gap
```

## 5. 후보 순위와 목적함수

CMA-ES에 큰 가중치를 임의로 더한 보상값을 전달하지 않는다. 각 세대 후보를 다음 튜플로 정렬하고
그 순위 `0, 1, 2, ...`를 CMA-ES fitness로 전달한다.

```python
(
    invalid_candidate,
    safe_stop_count,
    -minimum_avoidance_clearance_m,
    selected_reduced_clearance_count,
    -minimum_selected_headroom,
)
```

이 순서에서는 `clearance=0.25 m`지만 `safe_stop=1`인 후보보다 `clearance=0.151 m`이고
`safe_stop=0`인 후보가 항상 우선한다. `safe_stop=0` 후보끼리만 clearance를 비교하므로 안전
정지를 줄이기 위해 clearance를 희생하는 것과 안전 정지 0을 유지한 채 clearance를 늘리는 것을
명확히 분리한다.

각 세대 첫 후보에는 현재까지의 최적 후보를 다시 넣는다. CMA-ES의 무작위 표본이 모두 나빠도 이미
찾은 `safe_stop=0` 해를 잃지 않는다.

## 6. 종료 조건

다음 중 하나면 탐색을 종료한다.

1. `safe_stop=0`을 유지하면서 clearance가 탐색 상한에 도달한다.
2. `safe_stop=0` 최적 후보의 clearance가 `stall_generations` 동안 의미 있게 개선되지 않는다.
3. CMA-ES 자체 수렴 조건이 발생한다.
4. `max_generations`에 도달한다.

기본 의미 있는 clearance 개선량은 `0.0001 m`다. 종료 후 최적 후보를 C++ 평가기로 다시 실행해
1,562개 결과가 재현되는지 확인한 뒤에만 `_SUCCESS`를 생성한다.

## 7. 설치와 빌드

저장소 루트에서 실행한다. Python 오케스트레이션에는 `cma` 패키지가 추가로 필요하다.

```bash
python3 -m pip install -r offline_trajectory_generator/requirements.txt

source /opt/ros/jazzy/setup.zsh
colcon build --packages-select local_planning
source install/setup.zsh
```

ROS 2 노드를 실행할 필요는 없다. 빌드는 C++ `adaptive_side_evaluator` 실행 파일을 만들기 위해서만
필요하다.

## 8. 실행 방법

현재 `smooth_sigma=4.1`, 정확히 142 waypoint인 글로벌 경로를 사용한 기본 실행은 다음과 같다.

```bash
python3 offline_trajectory_generator/optimize_adaptive_parameters.py \
  --reference ruleset_adaptive_globalpath/map_smooth_4p1/global_waypoints.csv \
  --local-params src/local_planning/config/local_planning.yaml \
  --search-config offline_trajectory_generator/config/adaptive_cmaes.yaml \
  --output-root learning_adaptive_globalpath/cmaes \
  --population-size 12 \
  --max-generations 80 \
  --stall-generations 15 \
  --workers 8 \
  --strict-count 1562
```

한 세대만 연결 검사를 할 때는 다음처럼 실행한다.

```bash
python3 offline_trajectory_generator/optimize_adaptive_parameters.py \
  --population-size 4 \
  --max-generations 1 \
  --stall-generations 1 \
  --workers 4
```

## 9. 출력 구조

실행 중 결과는 `_incomplete`에 있고 최종 재평가 성공 후 시간 라벨 디렉터리로 이동한다.

```text
learning_adaptive_globalpath/cmaes/
├── latest_run.txt
├── _incomplete/
└── cmaes_YYYYMMDD_HHMMSS_mmm_KST/
    ├── _SUCCESS
    ├── parameters/
    │   ├── run_config.yaml
    │   ├── effective_search_space.yaml
    │   ├── initial_parameters.yaml
    │   └── best_parameters.yaml
    └── reports/
        ├── history.csv
        ├── best_side_evaluations.csv
        └── summary.json
```

- `history.csv`: 모든 세대·후보의 파라미터, 순위, 좌·우·safe-stop 수를 기록한다.
- `best_side_evaluations.csv`: 최종 후보의 1,562개 시나리오별 C++ 판정 원문이다.
- `summary.json`: 종료 원인, 세대 수, 최종 clearance와 판정 분포를 기록한다.
- `best_parameters.yaml`: 평가기에 필요한 전체 로컬 planning 파라미터 snapshot이다.

`best_parameters.yaml`은 adaptive overlay 생성기의 `--local-params` 입력으로 바로 사용할 수 있다.
다만 런타임 YAML의 topic, 안정화, stale 처리 등 다른 ROS 파라미터는 포함하지 않으므로 운영
`local_planning.yaml` 전체를 이 파일로 교체하면 안 된다. 검증 후 최적화된 키만 수동 반영해야 한다.

## 10. 현재 검증 결과

먼저 `smooth_sigma=4.1`, 142 waypoint 기준으로 후보 4개·한 세대 smoke test를 실행했다.

```text
초기값: safe_stop=0, clearance=0.151000 m
1세대 최적: safe_stop=0, clearance=0.196754 m
최종 재평가: left=763, right=799, safe_stop=0
```

이후 후보 12개, 최대 80세대, seed `20260806`으로 본 실행했다. 8세대에서 clearance 상한 도달
조건으로 종료됐고 최종 후보를 독립적으로 다시 평가했다.

```text
run: cmaes_20260806_162648_258_KST
종료 원인: safe_at_clearance_upper_bound
실행 세대: 8
평가 history: 12 candidates x 8 generations = 96 rows
최종 재평가: left=784, right=778, safe_stop=0
최종 clearance: 0.249991 m
탐색 clearance 상한: 0.250000 m
```

최종 탐색 파라미터는 다음과 같다.

```yaml
transition_distance_scales: [0.180794, 0.963633, 1.904460]
outside_line_transition_scale: 0.356840
commitment_clearance_reserve_m: 0.026811
minimum_avoidance_clearance_m: 0.249991
boundary_margin_m: 0.087942
minimum_target_offset_m: 0.168582
maximum_lateral_slope: 0.640085
```

`_SUCCESS`, 평가 CSV 1,562행, `safe_stop` 원문 0행과 summary 값이 일치하는 것을 확인했다.
다만 이는 오프라인 C++ 기하 평가의 최적 후보이므로, 운영 반영 전 전체 adaptive overlay 생성과
ROS 2 Jazzy 시뮬레이션에서 경계 이탈·곡률·실차 추종성을 추가 검증해야 한다.

## 11. 범위와 한계

- 현재 최적화 대상은 로컬 플래너의 좌우 통과 파라미터다.
- `smooth_sigma`, outline 생성 성공률과 PNG 렌더링 시간은 이 CMA-ES 목적함수에 포함하지 않는다.
- 평가 데이터는 고정된 장애물 크기와 `s/d` grid이므로 위치추정 오차나 장애물 크기 오차에 대한
  강건성은 별도 검증 세트를 추가해야 한다.
- `safe_stop=0`은 오프라인 기하 평가 통과를 뜻하며 실차 안전을 단독으로 보증하지 않는다.
- 최종 파라미터는 전체 PNG 검증과 ROS 2 Jazzy 시뮬레이션을 통과한 뒤 운영 YAML에 반영한다.
