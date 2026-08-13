# 제안서 — ReferencePathAdapter(경로 변형) 제거, 윈도우 투영(continuity)은 그대로 유지

작성일: 2026-08-09 (rev.2 — 리뷰 반영)
대상: `src/global_planning` (frenet_odom_node + clcs_frenet_converter 주변)
관련 커밋: `c060ad8` ("modify clcs") — 이 커밋이 도입한 두 기능 중 **하나만** 제거한다
베이스라인: HEAD `deff7ea` + 작업트리 미커밋 1줄 (§4-③ 참고)

## 0. 요약

> c060ad8이 함께 들여온 두 독립 기능 중, **ReferencePathAdapter(경로 자체를 변형하는
> 스무딩/곡률감소)는 전량 삭제**하고, **monotonic s-window tracking(윈도우 제한 투영,
> `convertTracked`/`projectInWindow`)은 현행 구현·운영값 그대로 유지**한다.

거동 영향의 정확한 서술: 검증 스냅샷(§1-1)에서 adapter는 `modified=false`이므로
**Frenet 좌표 출력은 제거 전후 동일할 것으로 예상**된다. 다만 HEAD 설정
(`enable_curvature_reduction: true`) 기준으로 adapter는 rho 검사·진단 로그를 실제로
수행하고 있으므로, 제거는 "죽은 코드 정리"가 아니라 **향후 경로와 파라미터
오버라이드에 대한 의도적 기능 삭제 + rho 관측성 제거**다 (§1-4 후속 과제 참고).

두 기능은 데이터 흐름에서 위치가 다르다:

```
/global_waypoints ──[제거: ReferencePathAdapter — 경로 변형]──> CLCS 빌드
                                                                  │
/pf/pose/odom ──────[유지: projectInWindow — 경로 위로 투영]────> /car_state/frenet/odom
```

## 1. 제거 근거 (ReferencePathAdapter)

### 1-1. 현재 라인에서 경로를 변형하지 않는다 (스냅샷 실측)

플래그 상태는 세 곳이 서로 다르므로 구분해 적는다:

| 위치 | `enable_path_smoothing` | `enable_curvature_reduction` |
|---|---|---|
| **HEAD** (`deff7ea`) yaml | false | **true** |
| **작업트리** (미커밋 1줄) | false | false |
| **C++ declare 기본값** | false | false |

즉 HEAD 기준으로 adapter는 기동 시 실행된다. 그러나 현재 waypoint 스냅샷에 실제
adapter 코드를 돌린 리뷰 실측(2026-08-09) 결과는:

```
입력: offline_trajectory_generator/output/map/global_waypoints.csv
      (140점, md5 10f73d607b7a05c1dfd9d3f486ce2ad3)
결과: stop_reason=already_satisfied, modified=false, initial_max_rho=0.921
```

`rho < 1`이므로 **좌표 변형 없음이 확인**됐다. 참고로 rho 수치는 스냅샷마다 다르다
— yaml 주석 "~0.77", `docs/frenet_odom_node.md` "≈0.81"은 이전 스냅샷 기준이고 위
0.921이 현재 파일 실측이다. 결론(rho<1 → no-op)은 세 스냅샷 모두에서 유지되지만,
0.921은 여유가 8%뿐이므로 **새 맵/새 라인에서는 성립을 보장할 수 없다** — 이것이
§1-4의 후속 과제가 필요한 이유다.

### 1-2. 켜서 경로가 바뀌면 안 되는 구조다 (프레임 불일치)

- `frenet_odom_node`가 변형된 경로를 쓰면 그 프레임은 이 노드 내부에서만 유효하다.
- `obstacle_detector`는 RAW `/global_waypoints`로 **자체 CLCS를 빌드**한다
  (`obstacle_detector_node.cpp`; `package.xml`이 `<depend>global_planning</depend>`로
  converter 라이브러리를 가져다 쓴다).
- `local_planning`은 RAW waypoint의 `s_m`과 ego/장애물 Frenet 값을 함께 비교한다
  (`raceline_spline_planner.cpp`).

한 소비자 안에서만 경로를 변형하면 ego·장애물·플래너의 (s, d) 좌표계가 갈라진다.
재도입하려면 **publisher 단(global_trajectory_publisher)에서 한 번 적용하고 모든
소비자가 동일한 완성 waypoint를 받아야** 한다 — 후속 문서 커밋 `09073ff`의 결론과
일치한다. 즉 frenet_odom_node 안의 adapter는 어느 방향으로도 출구가 없다.

### 1-3. 코드 규모

cpp 672 + hpp 90 + test 271 + 노드 배선 ~150줄 ≈ **1,200줄**. 루트 CLAUDE.md의 KISS
원칙에 따라, 이 위치에서 쓸 수 없는 기능은 코드베이스에서 걷어낸다.

### 1-4. 제거로 잃는 것과 후속 과제

HEAD 설정에서 adapter는 좌표를 바꾸지 않아도 **rho 검사와 진단 로그**(기동 시
`initial_max_rho` 등)를 수행해 왔다. 제거하면 "새 맵의 곡률 특이성(rho ≥ 1)"을
런타임에 알려 줄 장치가 사라진다.

**후속 과제(이번 커밋 범위 아님)**: 런타임 변형 대신, 오프라인 경로 생성 파이프라인
(offline_trajectory_generator) 또는 CI에 **rho < 1 유효성 검사**를 작은 스크립트로
추가하는 것을 권장한다. 곡률 특이성은 경로를 만들 때 잡는 것이 맞다.

## 2. 유지 범위 — 윈도우 투영(continuity), 손대지 않는다

브랜치 반전 방어의 실체이므로 **코드·운영값 일절 변경 없음**:

| 유지 항목 | 내용 |
|---|---|
| `convertTracked()` / `projectInWindow()` / `segmentCurvilinearCoords()` | s_prev 주변 윈도우 제한 최근접 투영, 윈도우 미스 fail-closed |
| `ClcsContinuityState` + path 재빌드 시 상태 리셋 | 진행도 추적 상태 |
| `newResult()` / `finishConversion()` | convert()/convertTracked() 공유 전·후처리 |
| `test_clcs_frenet_converter.cpp` | 심 wrap·미스·게이트·재획득 시나리오 포함 |

파라미터 6개 — C++ 선언 기본값과 운영 yaml 값을 구분해 기록한다 (변경 없음):

| 파라미터 | C++ 기본값 | 운영 yaml | 역할 |
|---|---|---|---|
| `continuity_enabled` | true | true | convertTracked 사용 여부 |
| `forward_window` | 1.0 | 1.0 | s_prev 전방 윈도우 [m] |
| `backward_tolerance` | 1.0 | 0.5 | s_prev 후방 허용 [m] |
| `initial_seed_window` | 0.0 | 0.0 | 첫 픽스 검색 범위 (0=전체) |
| `tracked_max_projection_distance` | 1.5 | 1.5 | tracked 전용 \|d\| 게이트 [m] |
| `reacquire_after_misses` | 15 | 5 | 연속 미스 후 1회 전역 재검색 |

`max_projection_distance: 2.5` / `projection_domain_limit: 2.7`도 그대로 둔다.

**유지 근거**: `run_0807_174227`에서 무상태 전역 투영이 s 12.16→25.21 m 반전을 일으켜
경로 전체가 뒤집힌 채 발행됐고(벽 충돌), `run_0808_203740`에서도 충돌 후 s 7.89→23.83
점프가 진단을 오염시켰다. 윈도우 추적은 이 계열 실패의 근본 방어다.

### 2-1. yaml 주석 정오 (이번 커밋에서 주석만 정정, 값 변경 없음)

- `reacquire_after_misses: 5  # ~0.5 s at 30 Hz odom` — **주석이 낡았다.** 0.5 s는
  구값 15 기준이고 현행 5는 **≈0.17 s**다. 주석만 정정한다.
- `tracked_max_projection_distance: 1.5`의 주석은 "branch gap(1.41 m)보다 작아야
  한다"고 말하는데 **1.5 > 1.41로 자기모순**이다. 값이 얽힌 문제이므로 이번 커밋에서
  건드리지 않고 **별도 안전성 검토 항목**으로 남긴다 (윈도우가 1차 방어라 이 게이트는
  이중 방어임 — 즉시 위험은 아님).

## 3. 삭제 범위

### 3.1 파일 삭제 (3개)

| 파일 | 규모 |
|---|---|
| `src/reference_path_adapter.cpp` | 672줄 |
| `include/global_planning/reference_path_adapter.hpp` | 90줄 |
| `test/test_reference_path_adapter.cpp` | 271줄 |

### 3.2 `CMakeLists.txt` — 공개 API 변경임을 명시

- `${PROJECT_NAME}_frenet_conversion` 라이브러리 소스에서 `src/reference_path_adapter.cpp` 제거
- `ament_add_gtest(test_reference_path_adapter ...)` 타겟 제거

이 라이브러리는 `install(DIRECTORY include/)` + `export_global_planning`으로
**다운스트림에 export**되며, `obstacle_detector`가 `<depend>global_planning</depend>`으로
실제 소비한다. 따라서 이 변경은 내부 정리가 아니라 **공개 헤더·심벌 제거**다.
확인 결과 `obstacle_detector`는 `clcs_frenet_converter.hpp`만 include하고 adapter
심벌(`adaptReferencePath`, `AdapterWaypoint`) 참조는 **0건**이므로 삭제해도 안전하다.
단 검증(§5)에서 `obstacle_detector` 동반 빌드·테스트를 필수로 한다.
저장소 외부에 이 라이브러리를 쓰는 배포 코드는 없는 것으로 파악됨 — 있다면 한 릴리스
deprecated 유지 여부를 별도 결정해야 한다.

### 3.3 `frenet_odom_node.cpp` 배선 (~150줄)

- `#include "global_planning/reference_path_adapter.hpp"`
- `toAdapterWaypoints()` 헬퍼
- 멤버: `adapter_config_`, `adaptation_requested_`, `enable_path_smoothing_`,
  `enable_curvature_reduction_`, `reference_resample_step_`
- 파라미터 선언·로딩·검증 블록: `subdivision_refinements`, `adaptation_max_iterations`,
  `boundary_margin`, `max_absolute_curvature`, `anchor_curvature_threshold`
  (범위 검증 WARN 4개 포함)
- waypointsCallback의 adaptation 실행 블록 (adaptReferencePath 호출, stop_reason 분기,
  약 :331–362)
- 기동 시 "Reference path adaptation enabled ..." INFO 로그
- continuity 관련 배선(파라미터 로딩, convertTracked 분기, reacquired 경고,
  상태 리셋)은 **모두 그대로 둔다**

### 3.4 파라미터 삭제 (코드 + yaml, 총 8개)

| 파라미터 | 분류 |
|---|---|
| `subdivision_refinements` | adapter 전용 (c060ad8 신설) |
| `adaptation_max_iterations` | adapter 전용 (c060ad8 신설) |
| `boundary_margin` | adapter 전용 (c060ad8 신설) |
| `max_absolute_curvature` | adapter 전용 (c060ad8 신설) |
| `anchor_curvature_threshold` | adapter 전용 (c060ad8 신설) |
| `enable_path_smoothing` | adapter 스위치 (c060ad8 이전부터 존재하나 no-op 선언이었음) |
| `enable_curvature_reduction` | adapter 스위치 (동상) |
| `reference_resample_step` | 소비처가 adapter(`resample_step`)뿐 — 확인 완료, 함께 삭제 |

yaml의 adaptation 주석 블록(IV'24 설명 + NOTE)도 함께 제거하고, §2-1의 reacquire
주석 정정을 같은 커밋에 포함한다.
`use_path_preprocessing`은 adapter와 무관(무효/중복 점 안전 필터 경고용)하므로 **유지**.

### 3.5 문서

- `docs/frenet_odom_node.md`: adaptation 절 제거 (rho ≈0.81 서술 포함), continuity 절 유지
- `AGENTS.md`: adapter 서술 제거. 대신 **제거 기록 한 단락**을 남긴다 —
  "ReferencePathAdapter(IV'24 Alg.1 포팅)는 2026-08-09 제거됨. 검증 스냅샷에서
  modified=false(rho 0.92 < 1)였고, obstacle_detector가 RAW `/global_waypoints`로 자체
  CLCS를 빌드하므로 이 위치의 경로 변형은 프레임 불일치를 만든다. 재도입 시 publisher
  단 적용 선행(`09073ff` 참고). 포팅은 논문과 전제(폐루프 앵커, waypoint 경계)가 달라
  논문의 형식적 보장이 그대로 이전되지 않음에 유의. 코드는 git `c060ad8`에서 복구 가능."

## 4. 결정 포인트

① **`enable_path_smoothing`/`enable_curvature_reduction`을 완전 삭제할지, no-op WARN
   선언으로 남길지.** 이 키를 넘기는 launch가 없고 yaml 키도 함께 지우므로
   **완전 삭제 권장** (§3.4 표는 완전 삭제 기준).

② **AGENTS.md 제거 기록 분량.** 한 단락(§3.5 초안)으로 충분.

③ **작업트리 미커밋 1줄 처리.** 현재 작업트리에는
   `enable_curvature_reduction: true → false` 미커밋 변경이 있다 (HEAD는 true).
   제거 커밋에서 이 키 자체가 삭제되므로 **별도 커밋 없이 제거 커밋에 흡수**한다.
   diff 베이스라인은 HEAD `deff7ea`로 명시한다.

## 5. 검증 계획

1. **베이스라인 기록**: 커밋 해시(`deff7ea`)와 waypoint 스냅샷 해시
   (`global_waypoints.csv` md5 `10f73d60...`)를 검증 로그에 남긴다.
2. **빌드**: `build/`·`install/`의 `global_planning`, `obstacle_detector` 잔재를 지운
   **fresh build**로 `colcon build --symlink-install --packages-select global_planning
   obstacle_detector` — 삭제된 헤더가 install에 남지 않는지 확인.
   ⚠️ 이 노트북은 ROS 2 **Humble**만 설치돼 있으므로 여기서의 빌드는 Humble 검증이다.
   타깃 배포판 **Jazzy 빌드는 젯슨에서** 별도 확인한다.
3. **테스트**: `colcon test --packages-select global_planning obstacle_detector` 후
   `colcon test-result --verbose`로 실제 결과 확인 — `test_clcs_frenet_converter`
   전체(Tracked* 포함) 통과, `test_reference_path_adapter`는 목록에서 사라져야 함.
4. **참조 0건 스윕**: `rg "reference_path_adapter|adaptReferencePath|AdapterWaypoint"`
   및 8개 파라미터명으로 저장소 전체 검색 — build/install 산출물 제외 0건.
5. **continuity 불변 확인**: diff에서 `convertTracked` 호출부, 상태 리셋, continuity
   파라미터 6개가 변경되지 않았는지 확인.
6. **bag A/B 재생 (필수)**: 가용 사고 bag `run_0808_203740`의 `/pf/pose/odom`을 동일
   입력으로 제거 전/후 노드에 재생해 s, d, segment_index, 미스/재획득 시점이 동일한지
   비교한다 (adapter가 modified=false였으므로 완전 일치가 기대값).
   `run_0807_174227` bag은 이 머신에 없음 — 확보되면 동일 절차 추가.
7. **시뮬 폐루프**: 현재 존재하는 맵 산출물은 `output/map/`뿐이므로 (`ifac_track`
   json 없음) **map 기준으로** 터미널 1~7 절차 실행 — `/car_state/frenet/odom`
   발행·연속성, 기동 로그에 "Monotonic s-window tracking enabled ..."만 남고
   adaptation 로그가 없는지 확인.

## 6. Git 진행

- `git revert c060ad8`은 쓰지 않는다 — 그 커밋에는 유지 대상인 윈도우 추적이 함께 있고,
  이후 커밋이 같은 파일 위에 쌓여 있다.
- **단일 forward-fix 커밋**으로 진행한다:

```
refactor(global_planning): ReferencePathAdapter 제거 — continuity 윈도우 추적은 유지

- 검증 스냅샷(140점, rho 0.92<1)에서 modified=false — 좌표 출력 불변
- obstacle_detector가 RAW /global_waypoints로 자체 CLCS를 빌드하므로 이 위치의
  경로 변형은 프레임 불일치를 만듦 — 재도입 시 publisher 단 선행 (09073ff)
- 공개 API 변경: export lib에서 adapter 헤더/심벌 제거 (다운스트림 참조 0건 확인)
- convertTracked/projectInWindow 및 continuity 운영값 6개는 변경 없음
- yaml reacquire 주석 정정 (5회 ≈ 0.17 s; 구 0.5 s는 15회 시절 잔재)
```

- 문서(`AGENTS.md`, `docs/frenet_odom_node.md`) 갱신을 같은 커밋에 포함한다.
- 작업 전 `git pull --rebase`로 원격과 동기화한다 (루트 CLAUDE.md 규칙).

## 7. 이번 커밋에서 하지 않는 것 (명시적 제외)

- `tracked_max_projection_distance` 1.5 vs branch gap 1.41 부등식 모순의 **값** 재조정
  → 별도 안전성 검토
- continuity 운영값(backward 0.5, reacquire 5) 재튜닝 → 별도 검토
- 오프라인/CI rho 유효성 검사 추가 (§1-4 후속 과제)
- publisher 단 경로 adaptation 재설계 (`09073ff` 문서 참고)
