# Forza 궤적 생성기 → `adaptive_global` **Python 이식** 제안서

> **이력 문서:** `map_creator`와 런타임 글로벌 경로 전환 기능은 2026-08-24에
> `main`에서 제거됐다. 아래 reload 관련 내용은 당시 설계 기록이며 현재 동작이 아니다.

작성일: 2026-08-23 (**전면 재작성**)
원본: `edge_test@95843f7a` → 대상: `adaptive_global@6ba865b3`

> 이 문서의 모든 수치·줄번호·유무 판정은 위 두 커밋에서 **직접 실측**한 값입니다.

---

## 0. 요약

`edge_test`의 ForzaETH 궤적 생성 GUI를 `adaptive_global`의 `offline_trajectory_generator/`로
**Python 코드 그대로** 옮깁니다. C++ 포팅하지 않습니다.

| 항목 | 규모 |
|---|---|
| 복사 | Python 약 270 KB (`forza_trajectory_gui.py` 53.5 KB + `forza_common.py` 79.0 KB + vendor 140 KB) |
| **신규 작성 코드** | **마커 생성부 함수 4개, 약 100~130줄** (6절) |
| 수정 | import 2줄, 경로 상수 3개, vendor `__init__.py` 1개, 파라미터 YAML 2줄 |
| 신규 런타임 의존성 | **없음** (전부 설치 확인, 3.5절) |
| 기존 C++ 파이프라인 수정 | **없음** |

이식 자체의 기술 위험은 낮습니다. **실질 작업은 마커 생성부 신규 작성 하나**이며,
나머지는 복사와 경로 정리입니다.

---

## 1. 방침

1. `edge_test`의 Forza 궤적 생성 기능을 `adaptive_global`로 가져온다.
2. 산출물은 전부 `offline_trajectory_generator/` 하위에 둔다.
3. **Python 코드를 그대로 이식한다.** C++ 포팅하지 않는다.
4. `forza_trajectory_gui.py`와 `trajectory_gui.py`는 **어떤 파일도 공유하지 않는다.**
5. 기존 C++ 파이프라인(`src/trajectory_core.cpp`, `bin/generate_global_trajectory`,
   `trajectory_gui.py`, `gui_params.yaml`, `CMakeLists.txt`)은 **한 줄도 건드리지 않는다.**

---

## 2. 대상 브랜치 실측

### 2.1 `adaptive_global`의 `offline_trajectory_generator/`

```
AGENTS.md  CMakeLists.txt  README.md  README_en.md
config/velocity_limits.csv
docs/proposal_d_ratio_raceline.md
gui_params.yaml
src/{generate_main.cpp, regenerate_main.cpp, trajectory_core.cpp, trajectory_core.hpp}
third_party/LBFGSpp/          (LBFGS 헤더 12개 + LICENSE)
trajectory_gui.py             ← C++ 바이너리를 부르는 subprocess 래퍼
```

Python 궤적 스택이 C++로 대체되면서 `generate_global_trajectory.py`,
`optimize_laptime.py`, `requirements.txt`가 **삭제**됐습니다.

### 2.2 🔴 `gb_optimizer`가 브랜치 전체에 없습니다

```bash
$ git ls-tree -r --name-only adaptive_global -- src/global_planning/vendor/gb_optimizer | wc -l
0
```

`src/global_planning/vendor/`에는 **`commonroad_clcs`(32파일)만** 있습니다.
즉 `trajectory_planning_helpers`는 "재사용할지 복사할지"의 문제가 아니라
**반드시 새로 가져와야** 합니다.

### 2.3 `forza_trajectory_gui.py`가 요구하는 것 vs 대상 브랜치

| # | 요구 (원본 줄번호) | `adaptive_global` | 조치 |
|---|---|---|---|
| 1 | `generate_global_trajectory`에서 심볼 **10개** (`:37~48`) | ❌ 파일 없음 | 5.2절 — `forza_common.py`로 개명 복사 |
| 2 | `trajectory_gui`에서 `render_preview_rgb`, `rgb_to_photoimage` (`:49`) | 🟡 **함수 존재** | 5.1절 — 방침 4 위반이 **조용히 성립함** |
| 3 | `VENDOR_SRC` = `src/global_planning/vendor/gb_optimizer/src` (`:58`) | ❌ 경로 없음 | 5.3절 |
| 4 | `DEFAULT_OPTIMIZER_CONFIG_DIR` = 같은 vendor의 `config/global_planner` (`:55~57`) | ❌ 경로 없음 | 5.3절 |
| 5 | `import trajectory_planning_helpers` (`:74~76`) | ❌ 없음 | 3절 — vendor |

> ⚠️ **2번이 이번 이식에서 가장 놓치기 쉬운 지점입니다.** 대상 브랜치
> `trajectory_gui.py`에도 두 함수가 그대로 있어(`:323`, `:360`) **아무것도 안 하면
> import가 성공해 버립니다.** 에러가 나지 않으므로 방침 4 위반이 조용히 통과합니다.

---

## 3. 이식 대상과 vendor 범위

### 3.1 파일 목록

| # | 원본 | 대상 | 비고 |
|---|---|---|---|
| 1 | `offline_trajectory_generator/forza_trajectory_gui.py` (53.5 KB) | 동명 | import 2줄 + 경로 상수 3개 수정 |
| 2 | `offline_trajectory_generator/generate_global_trajectory.py` (79.0 KB) | **`forza_common.py`** | 개명 복사 + 마커 생성 추가 |
| 3 | `trajectory_gui.py`의 미리보기 함수 3개 | **`forza_preview.py`** | 신규 파일 (5.1절) |
| 4 | `offline_trajectory_generator/forza_gui_params.yaml` (558 B) | 동명 | 경로 2줄 수정 |
| 5 | `offline_trajectory_generator/requirements.txt` (345 B) | 동명 | 그대로 |
| 6 | `gb_optimizer/config/global_planner/racecar_f110.ini` (16.8 KB) | `config/forza/` | 그대로 |
| 7 | `.../veh_dyn_info/ggv.csv` (299 B) | `config/forza/veh_dyn_info/` | ⚠️ 부록 B |
| 8 | `.../veh_dyn_info/ax_max_machines.csv` (81 B) | 동상 | 그대로 |
| 9 | `gb_optimizer/src/` Python 모듈 **21개** | `vendor/` | 3.2절 |

### 3.2 vendor — 전이 폐포 21개 / 약 140 KB

`forza_trajectory_gui.py`가 실제로 부르는 것에서 출발해 전이 폐포를 계산했습니다.

```
tph 직접 호출 8개 : calc_ax_profile, calc_head_curv_an, calc_head_curv_num,
                    calc_t_profile, calc_vel_profile, create_raceline,
                    import_veh_dyn_info, iqp_handler
helper 직접 호출  : prep_track, interp_track          (:75~76)
```

**필요한 `trajectory_planning_helpers` 19개 — 합계 132.6 KB**

| 모듈 | 크기 | 모듈 | 크기 |
|---|---:|---|---:|
| `calc_vel_profile` | 31.5 KB | `interp_track_widths` | 3.0 KB |
| `opt_min_curv` | 17.7 KB | `calc_t_profile` | 2.7 KB |
| `iqp_handler` | 10.8 KB | `interp_track` | 2.6 KB |
| `calc_splines` | 9.8 KB | `calc_ax_profile` | 2.0 KB |
| `interp_splines` | 9.8 KB | `normalize_psi` | 1.2 KB |
| `calc_head_curv_num` | 9.6 KB | `side_of_line` | 1.2 KB |
| `spline_approximation` | 8.1 KB | `conv_filt` | 2.8 KB |
| `calc_head_curv_an` | 5.0 KB | `check_normals_crossing` | 3.3 KB |
| `create_raceline` | 4.4 KB | `calc_spline_lengths` | 3.7 KB |
| `import_veh_dyn_info` | 3.4 KB | | |

**`helper_funcs_glob` 2개 — 7.3 KB**: `prep_track.py`, `interp_track.py`

### 3.3 제외 대상과 근거

| 제외 | 크기 | 근거 |
|---|---:|---|
| `inputs/frictionmaps/` (베를린·모데나) | **11.1 MB** | mintime 최적화 전용. `mincurv_iqp` 경로가 참조하지 않음 |
| `inputs/tracks/` 예제 트랙, 문서 | — | 무관 |
| 미사용 tph 12개 | — | 3.4절 |
| `helper_funcs_glob/src/__init__.py` 및 나머지 6개 | — | **3.4절 — `result_plots`가 matplotlib을 끌어옴** |
| `optimize_laptime.py` | — | `generate_global_trajectory.py:1298`에서 **함수 안에서 지연 import**. 최상위 의존성이 아니므로 제외해도 import 실패 없음. `--optimizer laptime`만 사용 불가 |

미사용 tph 12개: `angle3pt`, `calc_normal_vectors`, `calc_normal_vectors_ahead`,
`calc_tangent_vectors`, `calc_vel_profile_brake`, `get_rel_path_part`,
`import_veh_dyn_info_2`, `nonreg_sampling`, `opt_shortest_path`, `path_matching_global`,
`path_matching_local`, `progressbar`

### 3.4 🔴 vendor `__init__.py` 처리 — 두 파일의 성격이 다릅니다

**(a) `trajectory_planning_helpers/__init__.py` — 트림 필요**

원본은 **31개 모듈을 전부 import**합니다. 19개만 복사하면 그대로 깨집니다.
그런데 `create_raceline`·`iqp_handler`·`spline_approximation`이 `import trajectory_planning_helpers as tph`
후 `tph.<서브모듈>.<함수>` 형태로 접근하므로 **`__init__.py`를 지울 수도 없습니다.**
→ **19개만 노출하도록 트림**합니다. 이것이 vendor 코드에 가하는 유일한 수정입니다.

**(b) `helper_funcs_glob/__init__.py` — 복사하지 않음**

원본 `__init__.py` 2개는 `from global_racetrajectory_optimization.helper_funcs_glob.src import ...`
형태로 **전체 패키지 경로**를 참조하고, 그중 `result_plots`가 `matplotlib` + `mpl_toolkits`를
끌어옵니다. 하지만 `forza_trajectory_gui.py:70~76`은 `.../helper_funcs_glob/src`를 `sys.path`에
직접 넣고 `interp_track`·`prep_track`을 **최상위 모듈로** import하므로 이 `__init__.py`들은
애초에 실행되지 않습니다.
→ **`.py` 2개만 평면 복사**하고 `__init__.py`는 만들지 않습니다.

### 3.5 🟢 런타임 의존성은 이미 전부 설치돼 있습니다

`requirements.txt`: `numpy`, `opencv-python`, `PyYAML`, `scipy`, `scikit-image`, `quadprog`

현재 머신 실측:

```
numpy 2.2.6   scipy 1.15.3   scikit-image 0.25.2   opencv-python 5.0.0
PyYAML 5.4.1  quadprog OK    tkinter OK
```

> C++ 포팅이었다면 최대 난관이었을 `quadprog`(QP 솔버)·`skimage`(Lee thinning, watershed)
> 재구현이 **불필요**합니다. 이것이 방침 3의 핵심 이득입니다.

---

## 4. 권장 파일구조

```
offline_trajectory_generator/
│
├─ ■ 이식 (Forza Python) ───────────────────────────────────────────
├── forza_trajectory_gui.py        53.5 KB   이식 · import 2줄 + 경로 상수 3개 수정
├── forza_common.py                79.0 KB   generate_global_trajectory.py 개명 복사
│                                            + 마커 생성 4함수 추가 (6절)
├── forza_preview.py               신규      draw_polyline / render_preview_rgb
│                                            / rgb_to_photoimage (5.1절)
├── forza_gui_params.yaml          0.6 KB    이식 · map_yaml + optimizer_config_dir 수정
├── requirements.txt               0.3 KB    이식
│
├── config/
│   ├── velocity_limits.csv                  ← 기존, 무수정
│   └── forza/                               ← 신규 (기존 config와 분리)
│       ├── racecar_f110.ini      16.8 KB
│       └── veh_dyn_info/
│           ├── ggv.csv            0.3 KB    ⚠️ 전 구간 12.0 스텁 (부록 B)
│           └── ax_max_machines.csv 0.1 KB
│
├── vendor/                       신규 · 약 140 KB
│   ├── LICENSE                   🔴 필수 (LGPL-3.0 원문, 8절)
│   ├── README.md                 출처 · upstream 커밋 2개 · 트림 내역
│   ├── trajectory_planning_helpers/          19개 / 132.6 KB
│   │   ├── __init__.py           🔴 19개만 노출하도록 트림 (3.4-a)
│   │   ├── calc_ax_profile.py         calc_head_curv_an.py
│   │   ├── calc_head_curv_num.py      calc_spline_lengths.py
│   │   ├── calc_splines.py            calc_t_profile.py
│   │   ├── calc_vel_profile.py        check_normals_crossing.py
│   │   ├── conv_filt.py               create_raceline.py
│   │   ├── import_veh_dyn_info.py     interp_splines.py
│   │   ├── interp_track.py            interp_track_widths.py
│   │   ├── iqp_handler.py             normalize_psi.py
│   │   ├── opt_min_curv.py            side_of_line.py
│   │   └── spline_approximation.py
│   └── helper_funcs_glob/                    2개 / 7.3 KB
│       ├── prep_track.py                     (__init__.py 없음 — 3.4-b)
│       └── interp_track.py
│
├── docs/
│   ├── proposal_d_ratio_raceline.md          ← 기존, 무수정
│   └── forza_generator.md        신규        한국어 운영 문서 (CLAUDE.md 문서 정책)
│
├── output/                       .gitignore 대상
│   ├── <맵>/                                 C++ 생성기 산출물
│   └── <맵>/forza/                           Forza 산출물 (7.1절)
│
├── AGENTS.md                     갱신        Forza 섹션 추가
├── FORZA_PORT_PROPOSAL.md                    이 문서
│
└─ ■ 무수정 (방침 5) ──────────────────────────────────────────────
    trajectory_gui.py   gui_params.yaml   CMakeLists.txt
    src/{generate_main.cpp, regenerate_main.cpp, trajectory_core.cpp, trajectory_core.hpp}
    third_party/LBFGSpp/   config/velocity_limits.csv   README.md   README_en.md
```

### 구조 결정 근거

| 결정 | 근거 |
|---|---|
| `forza_` 접두사 3파일 | 방침 4를 **파일명 수준에서** 강제. import grep 0줄로 기계 검증 가능 |
| `vendor/`를 이 패키지 안에 | 방침 2. `sys.path` 조작이 `SCRIPT_DIR / "vendor"` 한 줄로 끝남 (현재는 4단계 밖 참조) |
| `config/forza/` 분리 | C++용 `config/velocity_limits.csv`와 섞이지 않음 |
| `output/<맵>/forza/` | C++ 생성기와 덮어쓰기 충돌 회피 (7.1절) |
| `vendor/LICENSE` 별도 | LGPL 고지를 vendor 경계에 명시 (8절) |

---

## 5. 필수 수정 4건

### 5.1 미리보기 분리 — `forza_preview.py` (방침 4)

`forza_trajectory_gui.py:49`가 `trajectory_gui`에서 2개를 가져오지만,
**실제로 옮겨야 하는 함수는 3개**입니다. `render_preview_rgb`가 같은 파일의
`draw_polyline`을 부르기 때문입니다.

| 함수 | 원본 위치 (`edge_test`) | 비고 |
|---|---|---|
| `draw_polyline` | `trajectory_gui.py:277` | `render_preview_rgb`의 내부 의존 |
| `render_preview_rgb` | `trajectory_gui.py:291` | |
| `rgb_to_photoimage` | `trajectory_gui.py:328` | PPM(P6) 인코딩 |

또한 `draw_polyline`·`render_preview_rgb`는 모듈 전역 `world_to_pixel`을 씁니다.
→ `forza_preview.py`는 `from forza_common import world_to_pixel`로 받습니다.

> 🟢 **두 브랜치의 이 3개 함수는 바이트 단위로 동일합니다** (46줄 블록 `diff` 결과 무차이).
> 즉 복사는 **동작 변경이 아니라 순수한 중복 제거 해제**이며, 회귀 위험이 없습니다.

`forza_trajectory_gui.py:49`를 다음으로 교체합니다.

```python
from forza_preview import render_preview_rgb, rgb_to_photoimage
```

### 5.2 `forza_common.py` — 개명 복사

`generate_global_trajectory.py`(1908줄)를 `forza_common.py`로 복사하고
`forza_trajectory_gui.py:37`의 import 대상을 바꿉니다. 요구되는 심볼 10개
(`GenerationResult`, `MapInfo`, `Trajectory`, `count_off_map_waypoints`,
`default_output_dir`, `load_map`, `pixel_to_world`, `world_to_pixel`,
`write_debug_image`, `write_outputs`)가 **전부 이 파일 안에 정의돼 있음을 확인**했습니다.

| 안 | 평가 |
|---|---|
| **A. 통째 복사 → `forza_common.py`** | ✅ **채택.** 파일이 자립적(최상위 import가 argparse/cv2/numpy/yaml/scipy뿐)이라 누락 위험 0 |
| B. 심볼 10개만 추출 | 내부 헬퍼 추적 필요, 누락 위험 |
| C. 원래 이름으로 복원 | ❌ C++로 대체한 파일명을 되살려 혼란. 방침 5와 충돌 |

> **대가**: 1908줄 중 상당수가 C++로 대체된 죽은 경로입니다(`--optimizer laptime`,
> shortest-path 등). Forza 전용 파일이고 C++ 쪽이 참조하지 않아 무해하지만,
> **파일 머리에 "Forza 전용 · C++ 파이프라인과 무관" 주석을 반드시 답니다.**

### 5.3 경로 상수 — 4곳

`forza_trajectory_gui.py:52~72`를 vendor 자립 구조에 맞게 고칩니다.

```python
# 변경 전
DEFAULT_OPTIMIZER_CONFIG_DIR = REPO_ROOT / "src/global_planning/vendor/gb_optimizer/config/global_planner"
VENDOR_SRC       = REPO_ROOT / "src/global_planning/vendor/gb_optimizer/src"
HELPER_FUNCS_SRC = VENDOR_SRC / "global_racetrajectory_optimization/helper_funcs_glob/src"

# 변경 후
DEFAULT_OPTIMIZER_CONFIG_DIR = SCRIPT_DIR / "config" / "forza"
VENDOR_SRC       = SCRIPT_DIR / "vendor"
HELPER_FUNCS_SRC = VENDOR_SRC / "helper_funcs_glob"
```

네 번째 지점은 `forza_trajectory_gui.py:235`입니다. YAML의 상대 `optimizer_config_dir`을
해석하는 곳으로, `REPO_ROOT / config_dir` → `SCRIPT_DIR / config_dir`로 바꿉니다.

`REPO_ROOT` 참조가 사라져 **워크스페이스 밖에서도 GUI가 단독 실행**됩니다.

### 5.4 마커 생성 — 6절

이식과 동시에 수정하는 **유일한 기능 결함**입니다. 별도 절로 분리합니다.

---

## 6. 🔴 마커 생성 이식 명세

### 6.1 현상과 원인

현재 Forza 산출물 `global_waypoints.json`은 마커 배열이 **전부 비어 있습니다.**

| 키 | Forza Python | C++ `mincurv` |
|---|---:|---:|
| `centerline_markers` | **0** | 1 |
| `global_traj_markers_iqp` | **0** | 1 |
| `trackbounds_markers` | **0** | 2 |

원인은 Forza가 아니라 **이식 대상 파일 자체**입니다.
`generate_global_trajectory.py:1737~1743`이 하드코딩으로 비웁니다.

```python
"centerline_markers":      {"markers": []},     # 1737
"global_traj_markers_iqp": {"markers": []},     # 1739
"global_traj_markers_sp":  {"markers": []},     # 1741
"trackbounds_markers":     {"markers": []},     # 1743
```

`write_forza_outputs`(`forza_trajectory_gui.py:724`)는 이 `write_outputs`를 호출한 뒤
`global_traj_wpnts_sp`와 `map_info_str`만 후처리하고 마커에는 손대지 않습니다.
**마커 생성은 C++ 포팅 과정에서 새로 추가된 기능**입니다(`trajectory_core.cpp:1125~1230`).

### 6.2 왜 반드시 고쳐야 하는가 — 런타임 증상까지 확인했습니다

`global_trajectory_publisher_node.cpp:116~131`:

```cpp
// Publish the array as-is, or a single DELETEALL when the source JSON carried no markers.
if (!markers.markers.empty()) { pub->publish(markers); return; }
deleter.action = visualization_msgs::msg::Marker::DELETEALL;
```

→ 빈 배열이면 **DELETEALL을 발행**합니다. 즉 생성은 성공하는데 RViz에서
글로벌 라인·트랙바운드가 **그려지지 않을 뿐 아니라 이전 맵의 마커까지 지워집니다.**

### 6.3 데이터는 이미 전부 있습니다

| 필드 | Python `Trajectory` | C++ `Trajectory` |
|---|---|---|
| `points_xy` `d_right` `d_left` `s_m` | ✅ | ✅ |
| `psi_rad` `kappa_radpm` `vx_mps` `ax_mps2` | ✅ | ✅ |

`build_global_trajectory`(`forza_trajectory_gui.py:642~659`)가 8개 필드를 모두 채웁니다 —
`psi_rad`는 `normalize_psi()`(`:618`) 정규화, `d_right`/`d_left`는 `distances_to_bounds()`(`:452`) 계산.

> 🟢 **순수한 1:1 직역**입니다. 추가 계산도, 파이프라인 변경도 없습니다.

### 6.4 이식할 함수 4개 (참조: `trajectory_core.cpp:1143~1224`)

#### 고정 상수

```python
TRAJ_MARKER_WIDTH       = 0.10
TRACKBOUND_MARKER_WIDTH = 0.05
MARKER_LINE_STRIP       = 4    # visualization_msgs/Marker::LINE_STRIP
MARKER_ADD              = 0    # visualization_msgs/Marker::ADD
```

> C++ 주석이 명시하듯 **스타일은 의도적으로 고정 상수**입니다. YAML로 빼면 노드 설정과
> 진실 공급원이 둘로 갈라집니다. **파라미터화하지 않습니다** —
> CLAUDE.md 파라미터 정책에 대한 의도적 예외이며, 사유를 `docs/forza_generator.md`에 남깁니다.

| 함수 | ns / id | width | 색 | 마커 수 |
|---|---|---:|---|---:|
| `line_strip(ns, id, width, color)` | — | — | 공통 골격 | — |
| `centerline_markers(traj)` | `centerline` / 0 | 0.05 | 회색 (0.5, 0.5, 0.5) | 1 |
| `trajectory_markers(traj)` | `global_traj_iqp` / 0 | 0.10 | 속도 색상 | 1 |
| `trackbound_markers(traj)` | `trackbound_left` / 0, `trackbound_right` / 1 | 0.05 | 파랑 (0.2, 0.6, 1.0) | 2 |

`line_strip` 골격: `header`(`frame_id: "map"`, stamp 0), `pose`(원점 + 단위 쿼터니언),
`scale`(x=width, y=0, z=0), `lifetime` 0, `frame_locked` false,
빈 `points`/`colors`, `text` "", `mesh_resource` "", `mesh_use_embedded_materials` false.

### 6.5 놓치기 쉬운 규칙 3가지

**(a) 속도 색상** — 초록(느림) → 빨강(빠름)

```python
span = (vmax - vmin) if (vmax - vmin) > 1e-6 else 1.0
t = clamp((vx[i] - vmin) / span, 0.0, 1.0)
colors[i] = (r=t, g=1.0-t, b=0.0, a=1.0)
```

마커 자체의 `color`는 **흰색 (1,1,1,1)**이고, 점별 색은 `colors` 배열에 들어갑니다.

**(b) 트랙바운드는 법선 오프셋** — `psi ± 90°`

```python
s, c = sin(psi[i]), cos(psi[i])
left  = (x[i] - d_left[i]  * s,  y[i] + d_left[i]  * c)
right = (x[i] + d_right[i] * s,  y[i] - d_right[i] * c)
```

**(c) 폐루프 닫기** — `n > 2`일 때 **첫 점을 끝에 한 번 더** 추가합니다.
`trajectory_markers`는 `colors`도 함께 닫습니다. 빠뜨리면 RViz에서 시작·끝이 끊어집니다.

### 6.6 적용 위치

`forza_common.py`의 `write_outputs()` — 하드코딩 4줄을 대체합니다.

```python
"centerline_markers":      centerline_markers(center_traj),
"global_traj_markers_iqp": trajectory_markers(global_traj),
"global_traj_markers_sp":  {"markers": []},   # ← 유지 (7.2절)
"trackbounds_markers":     trackbound_markers(global_traj),
```

---

## 7. 런타임 연동

### 7.1 출력 경로 — C++ 생성기와 충돌합니다

`global_planning.yaml`:

```yaml
output_base_dir: "offline_trajectory_generator/output"
map_name: "map"
reload_map_name: "obstacle_map"
```

`global_trajectory_publisher_node.cpp:78`은 **단순 문자열 결합**입니다.

```cpp
map_dir_ = output_base_dir.empty() ? name : output_base_dir + "/" + name;
```

두 생성기가 같은 `output/<맵>/`을 쓰면 나중에 실행한 쪽이 이깁니다.

| 안 | 평가 |
|---|---|
| **A. Forza 기본 출력을 C++ 산출물과 다른 디렉터리로 분리** | ✅ **채택** |
| B. 같은 경로 공유 | 어느 쪽 산출물인지 `metadata.json`을 열어야 판별 가능 |

구체적 경로는 **`output/<맵>/forza/`** 입니다. `forza_trajectory_gui.py:1204`가 이미
`default_output_dir(map_yaml) / "forza"`로 유도하고 있어 **코드 수정이 0**이고,
`output/<맵>/`·`output/obstacle_map/`처럼 맵 단위로 묶는 기존 배치와도 일관됩니다.

> 🟢 위 문자열 결합 덕분에 **`map_name: "<맵>/forza"`로 그대로 지정**할 수 있습니다.
> 노드 수정이 필요 없습니다. 실주행에 쓸 때는 이 값을 바꾸거나 `map_path` 오버라이드를 씁니다.

### 7.2 🟢 `global_traj_wpnts_sp`를 비우는 것은 무해합니다

`write_forza_outputs`가 `global_traj_wpnts_sp`를 빈 배열로 덮고
`map_info_str`에 `"; shortest_path=not_generated"`를 붙입니다.

실측 결과 `readwrite_global_waypoints.cpp:320~328`의 리더는 **7개 키만 읽습니다.**

```
map_info_str, est_lap_time, centerline_markers, centerline_waypoints,
global_traj_markers_iqp, global_traj_wpnts_iqp, trackbounds_markers
```

`global_traj_wpnts_sp`와 `global_traj_markers_sp`는 **아예 읽지 않습니다.**
`publish_shortest_path: true`여도 항상 기본 생성된 빈 메시지를 발행하므로,
Forza가 비우든 채우든 런타임 동작이 동일합니다.
→ 6.6절에서 `global_traj_markers_sp`를 빈 채로 두는 결정과 일관됩니다.

### 7.3 데이터 유효성 요건

`validBundle`(`global_trajectory_publisher_node.cpp:138~159`)이 요구하는 것:

- 웨이포인트 **4개 이상**
- `s_m`, `x_m`, `y_m`, `psi_rad`, `kappa_radpm`, `vx_mps` 전부 **유한**
- `s_m` **강한 증가**

> 이 검사는 `/global_planning/reload_waypoints` 경로에서만 돕니다(초기 로드는 미검사).
> 하지만 Forza 산출물도 이 조건을 만족해야 `map_creator` 리로드 파이프라인에 투입 가능합니다.
> 10절 검증에 포함합니다.

### 7.4 ⚠️ `expected_centerline_length`는 맵마다 재튜닝해야 합니다

`forza_trajectory_gui.py:338~350`:

```python
length = (findContours 직후 원시 폐곡선 둘레) * map_resolution
if expected_length > 0.0 and abs(expected_length / length - 1.0) >= 0.15:
    length = math.inf          # 후보 제외 → 전부 제외되면 RuntimeError
```

🔴 **비교 대상은 최종 레이스라인이 아니라 세선화 직후의 원시 폐곡선 둘레입니다.**
`/home/haejun/Downloads/map.yaml` + 저장된 파라미터 실측:

| 값 | 측정 |
|---|---:|
| 원시 skeleton 폐곡선 길이 | **42.3853 m** |
| 저장된 힌트 | 41.5929 m (오차 1.87 %) → 통과 |
| 허용 힌트 범위 | **36.03 ~ 48.74 m** |
| 최종 Forza 레이스라인 `s_max` | 36.0978 m |

#### 올바른 튜닝 절차

1. `expected_centerline_length = 0`(비활성)으로 전처리만 실행
2. 모든 원시 폐곡선의 점 개수와 **원시 길이**를 출력
3. 올바른 트랙 후보를 눈으로 확인
4. **그 후보의 원시 길이**를 힌트로 저장
5. 최종 `s_max`는 별도 지표로만 기록 (**힌트 근거로 쓰지 않습니다**)

### 7.5 `forza_gui_params.yaml` 수정 2줄

| 키 | 현재 값 | 문제 | 조치 |
|---|---|---|---|
| `map_yaml` | `/home/haejun/Downloads/map.yaml` | 저장소 밖 개인 경로 | 대상 맵 경로로 교체 |
| `optimizer_config_dir` | `src/global_planning/vendor/gb_optimizer/config/global_planner` | **대상 브랜치에 없음** (2.2절) | `config/forza` |
| `output_dir` | `.../offline_trajectory_generator/output` | 7.1절 충돌 | `.../output/forza` |

---

## 8. 라이선스

| 대상 | 라이선스 | 조치 |
|---|---|---|
| ForzaETH `race_stack` | MIT (추정) | 커밋 `202450f5…`의 LICENSE로 확인 |
| TUM `global_racetrajectory_optimization` | **LGPL-3.0** (추정) | 커밋 `b1b38ecf…`의 LICENSE로 확인 |
| TUM `trajectory_planning_helpers` | **LGPL-3.0** (추정) | 동일 |
| `quadprog` (PyPI) | GPL-2.0 | 개발 환경 설치분. 배포물에 동봉하지 않으면 무관 |

> 🟢 **Python 이식이 C++ 직역보다 안전합니다.** LGPL 코드를 다른 언어로 직역하면
> 파생저작물이 되어 전파 범위를 따져야 하지만, **원본을 거의 수정 없이 vendoring하면
> LICENSE + 출처 고지로 의무를 충족**하기가 훨씬 단순합니다.
>
> ⚠️ 단 3.4-a의 `__init__.py` 트림은 **원본 수정**에 해당합니다.
> `vendor/README.md`에 "미사용 모듈 12개 제거를 위해 `__init__.py`를 수정함"을 명시합니다.

두 커밋 SHA는 `forza_trajectory_gui.py:63~64`가 이미 상수로 들고 있고
`metadata.json`의 `forza_source`에 기록되므로 그대로 옮깁니다.

---

## 9. 작업 절차

| 단계 | 내용 | 완료 판정 |
|---|---|---|
| 1 | worktree 준비 (SHA 고정) | `git worktree add --detach ../ifac_edge_ref 95843f7a` |
| 2 | 작업 브랜치 생성 | `git worktree add -b forza-python ../ifac_forza_port 6ba865b3` |
| 3 | 라이선스 확인 (8절) | upstream 2개 커밋의 LICENSE 확보 |
| 4 | `vendor/` 21개 복사 + `__init__.py` 트림 | `python3 -c "import trajectory_planning_helpers"` 성공 |
| 5 | `config/forza/` 3개 복사 | — |
| 6 | `forza_common.py` 복사 (5.2절) | — |
| 7 | `forza_preview.py` 분리 (5.1절, **3함수**) | — |
| 8 | `forza_trajectory_gui.py` 복사 + import 2줄·경로 상수 4곳 | import grep 0줄 |
| 9 | **마커 생성 추가** (6절) | 유일한 신규 코드 |
| 10 | `forza_gui_params.yaml` 경로 3줄 수정 (7.5절) | — |
| 11 | `expected_centerline_length` 튜닝 (7.4절) | 대상 맵 힌트 확정 |
| 12 | 문서 (`AGENTS.md`, `docs/forza_generator.md`) | CLAUDE.md 문서 정책 |

> **worktree는 브랜치명이 아니라 SHA로 고정합니다.** `edge_test`가 이동하면
> 이식 원본이 달라집니다.

---

## 10. 검증

### 10.1 이식 성공 기준

| # | 항목 | 확인 방법 | 기대 |
|---|---|---|---|
| 1 | import 성공 | `python3 forza_trajectory_gui.py --help` | 정상 종료 |
| 2 | vendor 자립 | `python3 -c "import sys; sys.path.insert(0,'vendor'); import trajectory_planning_helpers"` | 성공 |
| 3 | **GUI 분리** (방침 4) | `grep -nE "^\s*(from\|import)\s+trajectory_gui\b" forza_*.py` | **0줄** |
| 4 | 워크스페이스 밖 참조 없음 | `grep -n "REPO_ROOT" forza_trajectory_gui.py` | 0줄 |
| 5 | `edge_test`와 동일 산출물 | 같은 맵·파라미터로 양쪽 실행 후 `global_waypoints.csv` 비교 | 무차이 |
| 6 | **마커 3종 생성** (6절) | 아래 스크립트 | 1 / 1 / 2 |
| 7 | 마커 스키마 일치 | C++ 산출물과 필드 구조 비교 | 동일 |
| 8 | 노드 유효성 통과 (7.3절) | `s_m` 강한 증가, 유한, ≥4개 | 통과 |
| 9 | 기존 C++ 무영향 | `git status` | 해당 파일 미변경 |

> ⚠️ 5번에서 **`metadata.json`은 비교 대상에서 제외**합니다 — 절대 출력 경로가 들어가
> 디렉터리가 다르면 반드시 달라집니다.

```bash
python3 - <<'EOF'
import json
d = json.load(open("output/forza/<맵>/global_waypoints.json"))
for k in ("centerline_markers", "global_traj_markers_iqp", "trackbounds_markers"):
    print(k, len(d[k]["markers"]))            # 기대: 1, 1, 2
m = d["global_traj_markers_iqp"]["markers"][0]
print("points", len(m["points"]), "colors", len(m["colors"]))   # 같아야 함 = 웨이포인트 수 + 1
s = [w["s_m"] for w in d["global_traj_wpnts_iqp"]["wpnts"]]
print("s strictly increasing:", all(b > a for a, b in zip(s, s[1:])), "n =", len(s))
EOF
```

### 10.2 실주행 검증

```bash
cd ~/2026_IFAC
F1_MAP=<맵이름> ros2 launch global_planning global_planning.launch.py
```

RViz에서 `/global_waypoints/markers`(속도 색상 레이스라인)와
`/trackbounds/markers`(파랑 좌우 경계)가 보이는지 확인합니다 — 6절 수정의 최종 확인입니다.

---

## 11. 위험 및 미결

### 11.1 위험

| # | 위험 | 등급 | 완화 |
|---|---|---|---|
| 1 | `__init__.py` 트림 누락 → import 실패 | 🟡 | 절차 4단계에서 import 성공 확인 |
| 2 | **방침 4 위반이 조용히 성립** — 대상 브랜치에도 함수가 있어 import가 성공함 | 🟡 | 절차 8단계 import grep 0줄 확인. 산문 언급까지 잡지 않도록 `^\s*(from\|import)` 로 앵커 |
| 3 | `draw_polyline` 누락 → `NameError` (5.1절) | 🟡 | 3함수 세트로 옮김 |
| 4 | 마커 수정 누락 → 생성은 성공, RViz는 빈 화면 + DELETEALL | 🟡 | 검증 6·9번 |
| 5 | `expected_centerline_length` 미튜닝 → 생성 실패 | 🟡 | 7.4절 절차 |
| 6 | 알고리즘 재현 실패 | 🟢 | 같은 코드를 돌리므로 위험 없음 |
| 7 | 속도 모델 신뢰성 | ⚠️ | **이식과 무관** — 부록 B |

### 11.2 미결 사항

| # | 항목 | 권장 | 상태 |
|---|---|---|---|
| 1 | vendor 범위 | 전이 폐포 21개 / 140 KB (마찰맵 11.1 MB 제외) | ✅ 결정 |
| 2 | `generate_global_trajectory.py` 처리 | `forza_common.py` 개명 복사 | ✅ 결정 |
| 3 | 미리보기 분리 | `forza_preview.py` (3함수) | ✅ 결정 |
| 4 | Forza 출력 경로 | `output/forza/<맵>/` | ✅ 결정 |
| 5 | `optimize_laptime.py` | 미포함 (지연 import라 무해) | ✅ 결정 |
| 6 | **`__init__.py` 트림 대신 31개 전체 vendoring(+383 KB)** | vendor 무수정이 되어 위험 #1과 8절 단서가 동시에 소멸. **KISS 관점에선 이쪽이 단순** | 🔵 **선택 여지 있음** |
| 7 | 대상 맵 확정 (`map_yaml`) | — | 🔴 **사용자 결정 필요** |

---

## 부록 A: 이식 후에도 손대지 않는 파일 (방침 5)

```
trajectory_gui.py            gui_params.yaml
CMakeLists.txt               config/velocity_limits.csv
src/generate_main.cpp        src/regenerate_main.cpp
src/trajectory_core.cpp      src/trajectory_core.hpp
third_party/LBFGSpp/**       README.md   README_en.md
docs/proposal_d_ratio_raceline.md
```

`CMakeLists.txt`는 이식분이 전부 Python이므로 **수정 대상이 아닙니다.**

---

## 부록 B: 이식과 무관한 별도 과제

### B.1 ⚠️ 속도 모델을 그대로 믿으면 안 됩니다

`ggv.csv`는 데이터 18행 전부 `12.0, 12.0`인 **튜닝되지 않은 스텁**입니다.
그리고 실제 산출물은 여기에 `longitudinal_accel_scale: 3.7`을 곱해 씁니다
→ **종가속 한계 44.4 m/s²**. 물리적으로 무의미합니다.

속도 프로파일 입력은 ggv 하나가 아닙니다.

```python
calc_vel_profile(ggv, ax_max_machines, v_max, kappa, el_lengths,
                 filt_window, dyn_model_exp, drag_coeff, m_veh)   # + 배율 3종 선적용
```

`drag_coeff`·`m_veh`는 `racecar_f110.ini`의 `veh_params.dragcoeff`·`mass`에서 옵니다.
반면 `veh_params.width`, `veh_params.curvlim`, `optim_opts_mincurv.width_opt`는
호출부에서 쓰이지 않습니다(GUI의 `safety_width`·`max_curvature`가 대신 전달됩니다).

> **이식 자체에는 영향이 없습니다** — 같은 코드를 돌리므로 동작이 동일합니다.
> 다만 **실주행 투입 전에 속도 모델 검증이 별도로 필요**합니다.

### B.2 ⚠️ 기존 랩타임 비교는 무효입니다

`output/map/metadata.json` 기준:

| | Forza 산출물 | 비교했던 C++ 실행 |
|---|---:|---:|
| `max_speed` | **9.0** | 4.0 (C++ 기본값) |
| `safety_width` | **1.0** | 0.35 (C++ 기본값) |
| 랩타임 | 5.777 s | 10.333 s |

차이의 지배 원인은 `max_speed` 9.0 vs 4.0입니다.
**이 두 숫자는 어떤 성능 근거로도 쓸 수 없습니다.**

---

## 12. 이행 결과 (2026-08-23)

브랜치 `forza-python` (`adaptive_global@6ba865b3` 기준)에 **이행 완료**했습니다.

### 12.1 절차 수행 결과

| 단계 | 결과 |
|---|---|
| 1~2 worktree | `../ifac_forza_port`, 브랜치 `forza-python` |
| 3 라이선스 | upstream에서 실제 확보 — TUM = **LGPL-3.0**(7,652 B, 두 저장소 동일), ForzaETH = **MIT (c) 2024 ForzaETH**. `vendor/LICENSE`에 원문 배치 |
| 4 vendor | tph 19개 + `helper_funcs_glob` 2개 = 21개. `__init__.py` 트림 후 import 성공 |
| 5 config | `config/forza/` 3개 |
| 6 `forza_common.py` | 개명 복사 + 모듈 docstring에 "Forza 전용" 명시 |
| 7 `forza_preview.py` | **3함수** 분리 (`draw_polyline` 포함) |
| 8 GUI | import 2줄 + 경로 상수 **4곳** 수정 |
| 9 마커 | 함수 7개 / 약 110줄 신규 |
| 10 파라미터 YAML | 경로 3줄 + 힌트 |
| 11 힌트 튜닝 | 7.4절 절차 실행 → **42.7723 m** |
| 12 문서 | `docs/forza_generator.md`(312줄, 한국어), `AGENTS.md` Forza 절, `vendor/README.md` |

### 12.2 검증 결과 — 10.1절 기준 전항 통과

대상 맵: `src/kinematic_localization/maps/map.yaml`
(제안서가 쓰던 `~/Downloads/map.yaml`은 그 사이 교체돼 사용 불가. 대신 `95843f7a`의 원본
코드·vendor·파라미터를 통째로 추출해 **동일 맵·동일 파라미터**로 원본과 이식본을 각각 실행)

| # | 항목 | 결과 |
|---|---|---|
| 1 | import 성공 | ✅ `--help` 정상 |
| 2 | vendor 자립 | ✅ 19/19 노출 |
| 3 | GUI 분리 (방침 4) | ✅ `forza_*.py` 어디에도 `trajectory_gui` import **0줄** |
| 4 | 워크스페이스 밖 참조 | ✅ `grep "REPO_ROOT"` **0줄** |
| 5 | **원본과 동일 산출물** | ✅ `global_waypoints.csv` **완전 동일**, `centerline.csv` **완전 동일**, 미리보기 PNG **md5 동일**. 원본·이식본 모두 센터라인 407 / mincurv_iqp 367 / 랩타임 5.914 s |
| 6 | 마커 3종 생성 | ✅ **1 / 1 / 2** |
| 7 | C++와 마커 스키마 일치 | ✅ 개수·필드 구조·ns·id·type·action·scale·color **전부 일치** |
| 8 | 노드 유효성 | ✅ n=367(≥4), 전 필드 유한, `s_m` 강한 증가 |
| 9 | 기존 C++ 무영향 | ✅ 기존 파일 미변경, `cmake --build` 성공, 바이너리 정상 실행 |

`global_waypoints.json`은 **마커 3종만** 원본과 다릅니다 — 의도한 유일한 변경입니다.

### 12.3 제안서 대비 편차 3건

| # | 제안서 | 실제 | 사유 |
|---|---|---|---|
| 1 | 출력 `output/forza/<맵>/` | **`output/<맵>/forza/`** | 코드가 이미 이 경로를 유도하고 있어 **코드 수정 0**. 충돌 회피 목적은 동일 (7.1절 정정 반영) |
| 2 | 경로 상수 3곳 | **4곳** | `:235`의 상대경로 해석부가 추가 (5.3절 정정 반영) |
| 3 | vendor 21개 | 동일 | — |

### 12.4 새로 확인된 사항

- **C++ 생성기는 `global_traj_markers_sp`·`global_traj_wpnts_sp` 키를 아예 쓰지 않습니다.**
  Python은 원본 동작대로 두 키를 남기지만, 리더가 읽지 않으므로 무해합니다 (7.2절 근거 보강).
- 노드 리더가 요구하는 7개 키는 양쪽 산출물 모두 존재합니다.

### 12.5 남은 미결

| # | 항목 | 상태 |
|---|---|---|
| 7 | 대상 맵 확정 | `src/kinematic_localization/maps/map.yaml`로 설정하고 힌트를 실측 튜닝. **다른 맵을 쓸 경우 7.4절 절차로 힌트 재측정 필요** |
| — | 속도 모델 검증 (부록 B) | **미해결.** 이식과 무관한 별도 과제로 남음 |
