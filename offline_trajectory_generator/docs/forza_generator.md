# ForzaETH 오프라인 궤적 생성기 (`forza_trajectory_gui.py`)

`edge_test@95843f7a`의 ForzaETH 궤적 생성 GUI를 Python 그대로 이식한 도구입니다.
이 패키지의 **C++ 생성기(`bin/generate_global_trajectory`)와는 완전히 독립**이며,
파일을 하나도 공유하지 않습니다.

---

## 1. 목적

SLAM 맵(ROS map_server 호환 YAML + 이미지) 하나로부터
**센터라인**과 **반복 최소곡률 레이스라인(mincurv_iqp)** 을 오프라인 생성해,
`global_planning` 패키지의 `global_trajectory_publisher_node`가 그대로 읽을 수 있는
`global_waypoints.json` 번들을 만듭니다. ROS 2 런타임도, colcon 워크스페이스도 필요 없습니다.

이 패키지에는 생성기가 **두 개**이며 목적이 다릅니다.

| | C++ 생성기 | **Forza 생성기 (이 문서)** |
|---|---|---|
| 실행 파일 | `bin/generate_global_trajectory` | `forza_trajectory_gui.py` |
| GUI | `trajectory_gui.py` | `forza_trajectory_gui.py` |
| 파라미터 YAML | `gui_params.yaml` | `forza_gui_params.yaml` |
| 최적화 | L-BFGS-B (`mincurv`, `d_ratio`, `centerline`) | **QP 반복 최소곡률 (`mincurv_iqp`)** |
| 세선화 | OpenCV ximgproc / Zhang-Suen | **scikit-image Lee thinning** |
| 트랙폭 | distance/raycast/hybrid | **watershed + distance transform** |
| 기본 출력 | `output/<맵>/` | `output/<맵>/forza/` |

> ⚠️ 둘은 같은 파일명(`global_waypoints.json` 등)을 씁니다. 출력 디렉터리를 분리해 두었으니
> 같은 경로로 겹쳐 쓰지 마세요. 마지막에 실행한 쪽이 이깁니다.

---

## 2. 동작 원리

`generate_forza_trajectory()`(`forza_trajectory_gui.py`)가 아래 순서로 진행합니다.

| # | 단계 | 함수 | 핵심 내용 |
|---|---|---|---|
| 1 | 맵 로드 | `load_map` | YAML의 `resolution`/`origin`을 읽고 이미지를 점유격자로 변환 |
| 2 | 이진화·잡음 제거 | `forza_filter_map` | `occupancy_grid_threshold`로 이진화 후 `filter_kernel_size` 커널로 정리 |
| 3 | 세선화 | `skimage.morphology.skeletonize(method="lee")` | 주행 가능 영역을 1픽셀 폭 골격으로 |
| 4 | 센터라인 선택 | `extract_centerline` | 폐곡선 후보 중 **가장 짧은 것**을 채택. `expected_centerline_length` 힌트로 ±15 % 밖 후보를 제외 (**4.1절**) |
| 5 | 이음매 평활화 | `smooth_centerline` | Savitzky-Golay 2회 통과 (길이에 따라 필터 폭 자동) |
| 6 | 등간격 재샘플 | `interp_closed_track` | 0.1 m 간격 폐곡선 |
| 7 | 트랙 경계 추출 | `extract_track_bounds` | watershed로 좌/우 벽을 가르고, 실패 시 distance transform으로 대체 |
| 8 | 폭 계산 | `centerline_with_widths` | `[x, y, w_right, w_left]` 4열 트랙 구성 |
| 9 | **최소곡률 최적화** | `optimize_min_curvature_iqp` | `tph.iqp_handler` → `tph.opt_min_curv`(quadprog QP)를 수렴할 때까지 반복 |
| 10 | 속도 프로파일 | `tph.calc_vel_profile` | ggv·모터 한계·항력·질량 기반. 배율 3종을 선적용 (**5절**) |
| 11 | 궤적 구성 | `build_global_trajectory` | `psi`는 `normalize_psi`, `d_left/d_right`는 `distances_to_bounds` |
| 12 | 출력 | `write_forza_outputs` → `forza_common.write_outputs` | CSV 2종 + JSON 번들 + 메타데이터 |

### 2.1 마커 생성 (이식 시 추가된 유일한 기능)

원본 `generate_global_trajectory.py`는 JSON의 마커 배열 4종을 **빈 배열로 하드코딩**했습니다.
`global_trajectory_publisher_node`는 마커 배열이 비면 **`DELETEALL`을 발행**하므로,
그대로 두면 RViz에 레이스라인이 안 보이는 정도가 아니라 이전 맵의 마커까지 지워집니다.

그래서 `forza_common.py`의 `write_outputs()`가 `trajectory_core.cpp:1125~1230`을 1:1 이식한
마커 생성기를 호출하도록 바꿨습니다.

| 마커 | ns / id | 선폭 | 색 | 개수 |
|---|---|---:|---|---:|
| 센터라인 | `centerline` / 0 | 0.05 | 회색 (0.5, 0.5, 0.5) | 1 |
| 레이스라인 | `global_traj_iqp` / 0 | 0.10 | **점별 속도 색상** (초록=느림 → 빨강=빠름) | 1 |
| 트랙바운드 | `trackbound_left` / 0, `trackbound_right` / 1 | 0.05 | 파랑 (0.2, 0.6, 1.0) | 2 |

- 트랙바운드는 `psi ± 90°` 법선 방향으로 `d_left`/`d_right`만큼 오프셋합니다.
- 점이 3개 이상이면 **첫 점을 끝에 한 번 더** 붙여 폐루프를 닫습니다
  (레이스라인은 `colors`도 함께 닫습니다). 빠뜨리면 RViz에서 시작·끝이 끊어집니다.
- **스타일은 고정 상수입니다.** YAML로 빼면 노드 설정과 진실 공급원이 둘로 갈라지므로
  파라미터화하지 않았습니다 (루트 `CLAUDE.md` 파라미터 정책에 대한 의도적 예외).

`global_traj_markers_sp`는 빈 채로 둡니다. Forza는 최단경로를 만들지 않고,
노드의 JSON 리더(`readwrite_global_waypoints.cpp`)는 `*_sp` 키를 **아예 읽지 않습니다.**

---

## 3. 입출력

### 3.1 입력

| 항목 | 내용 |
|---|---|
| 맵 | ROS map_server 호환 `*.yaml` + 이미지(`png`/`pgm`) |
| 차량 모델 | `config/forza/racecar_f110.ini` |
| ggv 다이어그램 | `config/forza/veh_dyn_info/ggv.csv` |
| 모터 가속 한계 | `config/forza/veh_dyn_info/ax_max_machines.csv` |

### 3.2 출력 — `output/<맵>/forza/`

| 파일 | 내용 |
|---|---|
| `global_waypoints.json` | 노드가 읽는 번들 (웨이포인트 + 마커) |
| `global_waypoints.csv` | 레이스라인 웨이포인트 |
| `centerline.csv` | 센터라인 |
| `metadata.json` | 실행 파라미터 + `forza_source`(upstream 커밋 2개) |

### 3.3 이 산출물을 쓰는 노드와 토픽

**오프라인 도구이므로 자체 토픽은 없습니다.** `global_trajectory_publisher_node`가
`global_waypoints.json`을 읽어 아래를 발행합니다.

| 토픽 | 타입 |
|---|---|
| `/global_waypoints` | `f110_msgs/WpntArray` |
| `/centerline_waypoints` | `f110_msgs/WpntArray` |
| `/global_waypoints/shortest_path` | `f110_msgs/WpntArray` (Forza는 빈 배열) |
| `/global_waypoints/markers` | `visualization_msgs/MarkerArray` |
| `/trackbounds/markers` | `visualization_msgs/MarkerArray` |
| `/centerline_waypoints/markers` | `visualization_msgs/MarkerArray` |
| `/map_infos` | `std_msgs/String` |
| `/estimated_lap_time` | `std_msgs/Float32` |

---

## 4. 파라미터 — `forza_gui_params.yaml`

파일 위치: `offline_trajectory_generator/forza_gui_params.yaml`
(C++ GUI의 `gui_params.yaml`과 **별개 파일**입니다.)

| 키 | 현재 값 | 설명 |
|---|---|---|
| `map_yaml` | 절대경로 | 대상 맵. GUI에서 맵을 고르면 자동으로 갱신됩니다 |
| `output_dir` | `""` | 비우면 `output/<맵>/forza`로 자동 유도 |
| `optimizer_config_dir` | `config/forza` | 상대경로는 **이 패키지 기준**으로 해석 |
| `occupancy_grid_threshold` | 17.10619469 | 점유격자 이진화 임계 |
| `filter_kernel_size` | 7 | 잡음 제거 커널 |
| `expected_centerline_length` | 42.7723 | **원시 폐곡선 둘레** 힌트 [m]. 0이면 비활성 (4.1절) |
| `safety_width` | 1.0 | 차량 폭 + 안전 여유 [m] |
| `max_curvature` | 0.8 | 조향 한계 [rad/m] |
| `max_speed` | 9.0 | 최대 속도 [m/s]. ggv/모터 데이터 범위를 넘으면 실행 거부 |
| `longitudinal_accel_scale` | 3.7 | ggv 종가속 배율 (**5절 경고**) |
| `lateral_accel_scale` | 1.0 | ggv 횡가속 배율 |
| `machine_accel_scale` | 1.0 | 모터 한계 배율 |
| `dynamic_model_exponent` | 1.0 | 마찰원 결합 지수 |
| `velocity_filter_window` | 3 | 속도 프로파일 이동평균 창 |
| `reverse` | false | 진행 방향 반전 |
| `show_centerline` / `show_raceline` / `debug_image` | true | 미리보기·디버그 이미지 |

모든 키는 동명의 CLI 인자로 덮어쓸 수 있습니다 (`--expected-centerline-length` 등).

### 4.1 ⚠️ `expected_centerline_length`는 맵마다 다시 튜닝해야 합니다

```python
length = (findContours 직후 원시 폐곡선 둘레) * map_resolution
if expected_length > 0.0 and abs(expected_length / length - 1.0) >= 0.15:
    length = math.inf          # 후보 제외 → 전부 제외되면 RuntimeError
```

🔴 **비교 대상은 최종 레이스라인 `s_max`가 아니라 세선화 직후의 원시 폐곡선 둘레입니다.**
이 둘은 크게 다릅니다 — 현재 맵 기준 **원시 폐곡선 42.77 m** vs 레이스라인 `s_max` 36.53 m (센터라인 40.60 m).

#### 튜닝 절차

1. 힌트를 **0으로 두고** 아래 스크립트로 후보들의 **원시 길이**를 출력합니다.

   ```bash
   cd offline_trajectory_generator
   python3 - <<'EOF'
   import sys; sys.path.insert(0, ".")
   import cv2, numpy as np
   from pathlib import Path
   from skimage.morphology import skeletonize
   from forza_common import load_map
   from forza_trajectory_gui import forza_filter_map

   MAP = Path("src/kinematic_localization/maps/map.yaml")   # 대상 맵으로 교체
   info, image, _ = load_map(MAP, unknown_as_free=False)
   filtered = forza_filter_map(image, info, 17.10619469, 7)  # 쓰려는 임계·커널과 동일하게
   skel = np.asarray(skeletonize(filtered, method="lee"), dtype=np.uint8)
   if int(skel.max(initial=0)) <= 1:
       skel *= 255
   contours, hierarchy = cv2.findContours(skel, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
   for i, c in enumerate(contours):
       p = c[:, 0, :].astype(np.float64)
       L = float(np.linalg.norm(p - np.roll(p, 1, axis=0), axis=1).sum()) * info.resolution
       print(f"{i:>3} points={len(p):>6} raw_len={L:.4f} m")
   EOF
   ```

2. 올바른 트랙 후보를 눈으로 확인합니다.
3. **그 후보의 원시 길이**를 `expected_centerline_length`에 저장합니다.
4. 최종 `s_max`는 별도 지표로만 기록하고, **힌트 근거로 쓰지 않습니다.**

> `occupancy_grid_threshold`나 `filter_kernel_size`를 바꾸면 골격이 달라지므로
> 힌트도 다시 측정해야 합니다.

---

## 5. ⚠️ 속도 모델은 그대로 믿으면 안 됩니다

- `config/forza/veh_dyn_info/ggv.csv`는 데이터 18행이 **전부 `12.0, 12.0`인 튜닝되지 않은 스텁**입니다.
- 여기에 `longitudinal_accel_scale: 3.7`이 곱해집니다 → **종가속 한계 44.4 m/s²**.
  물리적으로 무의미한 값입니다.
- `racecar_f110.ini`의 `veh_params.width`·`curvlim`, `optim_opts_mincurv.width_opt`는
  호출부에서 쓰이지 않습니다 (GUI의 `safety_width`·`max_curvature`가 대신 전달됩니다).
  실제로 쓰이는 값은 `veh_params.dragcoeff`와 `veh_params.mass`뿐입니다.

**이식 자체와는 무관한 문제**(원본과 동일 동작)이지만, **실차 투입 전 속도 모델 검증이 별도로 필요합니다.**
같은 이유로 C++ 생성기와의 랩타임 비교도 `max_speed`·`safety_width` 기본값이 다르면 의미가 없습니다.

---

## 6. 실행 방법

### 6.1 의존성

```bash
cd offline_trajectory_generator
pip install -r requirements.txt      # numpy, opencv-python, PyYAML, scipy, scikit-image, quadprog
```

`vendor/`에 TUM 최적화 모듈이 들어 있어 `trajectory_planning_helpers`를 따로 설치할 필요가 없습니다.

### 6.2 GUI 실행

```bash
cd offline_trajectory_generator
python3 forza_trajectory_gui.py
```

좌측 패널에서 맵과 파라미터를 고르고 Generate → Save outputs 순으로 진행합니다.

### 6.3 헤드리스 실행 (검증·자동화용)

```bash
cd offline_trajectory_generator
python3 forza_trajectory_gui.py \
  --map-yaml ../src/kinematic_localization/maps/map.yaml \
  --expected-centerline-length 42.7723 \
  --output-dir output/map/forza \
  --render-test /tmp/forza_preview.png
```

생성 결과를 눈으로 확인할 미리보기 PNG와 출력 파일 4종이 함께 나옵니다.

### 6.4 산출물 확인

```bash
python3 - <<'EOF'
import json
d = json.load(open("output/map/forza/global_waypoints.json"))
for k in ("centerline_markers", "global_traj_markers_iqp", "trackbounds_markers"):
    print(k, len(d[k]["markers"]))            # 기대: 1, 1, 2
m = d["global_traj_markers_iqp"]["markers"][0]
n = len(d["global_traj_wpnts_iqp"]["wpnts"])
print("points", len(m["points"]), "colors", len(m["colors"]), "wpnts", n)  # points=colors=n+1
s = [w["s_m"] for w in d["global_traj_wpnts_iqp"]["wpnts"]]
print("s_m 강한 증가:", all(b > a for a, b in zip(s, s[1:])), " n =", len(s))
EOF
```

마커가 하나라도 0이면 RViz에 아무것도 안 그려집니다 (2.1절).

### 6.5 ROS 2에서 사용하기

`global_trajectory_publisher_node`는 `<output_base_dir>/<map_name>/global_waypoints.json`을
읽습니다. 두 값을 `/` 로 단순 결합하므로 **`map_name`에 하위 경로를 그대로 넣으면 됩니다.**

```bash
cd ~/2026_IFAC
ros2 launch global_planning global_planning.launch.py
```

`src/global_planning/config/global_planning.yaml`에서:

```yaml
global_trajectory_publisher_node:
  ros__parameters:
    output_base_dir: "offline_trajectory_generator/output"
    map_name: "map/forza"        # ← Forza 번들을 가리킴 (기본값은 "map" = C++ 산출물)
```

RViz에서 `/global_waypoints/markers`(속도 색상 레이스라인)와
`/trackbounds/markers`(파랑 좌우 경계)를 Add 하면 결과가 보입니다.

---

## 7. 파일 구성

```
offline_trajectory_generator/
├── forza_trajectory_gui.py    Forza GUI + 파이프라인
├── forza_common.py            맵 로드 / 출력 / 마커 생성 (원본 generate_global_trajectory.py)
├── forza_preview.py           미리보기 렌더링 3함수
├── forza_gui_params.yaml      파라미터
├── requirements.txt
├── config/forza/              racecar_f110.ini, veh_dyn_info/{ggv,ax_max_machines}.csv
└── vendor/                    TUM 최적화 모듈 (LGPL-3.0, vendor/README.md 참고)
    ├── trajectory_planning_helpers/   19개 모듈
    └── helper_funcs_glob/             prep_track.py, interp_track.py
```

`forza_*`는 **C++ 생성기 쪽 파일(`trajectory_gui.py`, `gui_params.yaml`, `src/`, `CMakeLists.txt`)을
하나도 import 하지 않습니다.** 다음 두 명령이 모두 결과 0줄이어야 합니다.

```bash
grep -nE "^\s*(from|import)\s+trajectory_gui\b" forza_*.py   # 결과 0줄
grep -n "REPO_ROOT" forza_trajectory_gui.py                    # 결과 0줄
```

---

## 8. 출처와 라이선스

| 대상 | 라이선스 | 커밋 |
|---|---|---|
| ForzaETH `race_stack` (알고리즘 원본) | MIT (c) 2024 ForzaETH | `202450f51081b618fcca71924ad6d33f44f90e44` |
| TUM `global_racetrajectory_optimization` / `trajectory_planning_helpers` | **LGPL-3.0** | `b1b38ecfefec7200d5bcb243f115af1d795b3766` |

LGPL 원문은 `vendor/LICENSE`에 있고, vendoring 범위와 upstream 대비 수정 내역은
`vendor/README.md`에 기록돼 있습니다. 두 커밋 SHA는 매 실행의 `metadata.json`
`forza_source` 항목에도 기록됩니다.
