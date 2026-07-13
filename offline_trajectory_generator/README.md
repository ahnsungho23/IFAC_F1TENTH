# Offline Trajectory Generator

> 🌐 **한국어** · [English](README_en.md)

이 도구는 SLAM이 끝난 뒤 생성된 ROS map YAML/이미지를 입력으로 받아, ROS 2를 실행하지 않고 global waypoint 파일을 한 번 생성한다.

## 1. 목적

- 입력: `map.yaml` + `map.pgm` 또는 `map.png`
- 처리: free-space 추출, centerline 생성, waypoint 재샘플링, 곡률 기반 속도 프로파일 계산
- 출력:
  - `global_waypoints.json`
  - `global_waypoints.csv`
  - `centerline.csv`
  - `metadata.json`
  - 선택: `debug_overlay.png`

`global_waypoints.json`은 기존 `f110_msgs/Wpnt` 필드명과 같은 구조를 사용한다.

## 2. 실행 방법

저장소 루트에서 실행한다.

GUI로 파라미터를 조정하면서 확인하려면 다음 명령을 사용한다.

```bash
python3 offline_trajectory_generator/trajectory_gui.py
```

처음 실행하면 map YAML 선택 창이 열린다. 예를 들어 `monte_carlo_localization/maps/slam_map.yaml`을 선택하면 된다.

f1sim에서 쓸 trajectory를 만들 때는 f1sim의 `config/sim.yaml`에 있는 `map_path`와 같은 map YAML을 선택해야 한다. 현재 f1sim 기본 map은 `src/monte_carlo_localization/maps/fuck_f1.yaml`이고, `new_map_con` 기본 CSV도 이 map 좌표계의 `src/new_map_con/maps/fuck_f1.csv`를 사용한다.

처음부터 특정 map을 열고 싶으면 다음처럼 넘긴다.

```bash
python3 offline_trajectory_generator/trajectory_gui.py \
  --map-yaml monte_carlo_localization/maps/slam_map.yaml
```

화면 왼쪽에는 map 경로, 저장 위치, trajectory 파라미터가 표시된다. 숫자 파라미터는 `Sampling`, `Track & safety`, `Speed profile`, `Map cleanup`, `Centerline`, `Min-curvature`, `Straightening` 그룹별 소제목으로 묶여 있어 원하는 항목을 빠르게 찾을 수 있다. 오른쪽에는 map 이미지 위에 centerline과 RT lane이 함께 표시된다. 슬라이더나 체크박스를 바꾸면 잠시 후 자동으로 다시 계산되어 overlay가 갱신된다.

각 숫자 파라미터는 이름 옆 입력칸에 현재 값이 표시되고, 그 칸에 값을 직접 입력할 수도 있다. 입력칸에서 `Enter`를 누르거나 다른 칸으로 이동하면 값이 적용된다. 직접 입력값은 파라미터 단위에 맞게 반올림되고 최소값보다 작으면 최소값으로 제한되지만, 슬라이더 최대 범위보다 큰 값도 그대로 사용할 수 있다. 이때 슬라이더는 빠른 조정용 범위 끝에 머물고, 실제 적용값은 입력칸과 저장 YAML에 유지된다. 파라미터가 화면 아래에 있으면 왼쪽 패널 위에서 마우스 휠이나 스크롤바로 이동한다.

GUI에서 `Save`를 누르면 현재 화면에 보이는 trajectory가 `global_waypoints.json`, `global_waypoints.csv`, `centerline.csv`, `metadata.json`으로 저장된다.

GUI 파라미터 값은 별도 조작 없이 변경 즉시 다음 YAML에 자동 저장된다.

```text
offline_trajectory_generator/gui_params.yaml
```

다음 실행 때는 이 YAML에 저장된 값을 다시 불러온다. 다른 설정 파일을 쓰고 싶으면 `--params-yaml`을 넘긴다.

```bash
python3 offline_trajectory_generator/trajectory_gui.py \
  --params-yaml /tmp/my_trajectory_gui_params.yaml
```

CLI로 바로 파일만 생성하려면 다음 명령을 사용한다.

```bash
python3 offline_trajectory_generator/generate_global_trajectory.py \
  --map-yaml monte_carlo_localization/maps/slam_map.yaml \
  --output-dir /tmp/offline_traj_slam_map \
  --debug-image
```

출력 디렉터리를 생략하면 기본값은 다음과 같다.

```text
offline_trajectory_generator/output/<map_yaml_file_name>/
```

## 3. 주요 옵션

- `--waypoint-step`: 최종 waypoint 간격이다. 기본값은 `0.1` m이다.
- `--optimizer-step`: 내부 centerline/RT lane 계산 간격이다. 이 간격이 곧 최소 곡률 최적화의 변수 개수를 결정하므로, 너무 작게(예: `0.05`) 잡으면 변수가 수백~천 개로 늘어 최적화가 반복 한도에 걸리고 느려진다. 기본값은 `0.2` m이며, 보통 `0.15`~`0.25` 범위면 충분하다.
- `--safety-width`: 차량 폭과 안전 여유를 포함한 폭이다.
- `--boundary-margin`: 벽에서 추가로 띄울 거리이다.
- `--max-speed`: waypoint 속도 상한이다. 기본값은 `4.0` m/s이다.
- `--min-speed`: waypoint 속도 하한이다. 기본값은 `1.0` m/s이다.
- `--max-lateral-accel`: 곡률 기반 속도 계산에 쓰는 횡가속도 한계이다.
- `--median-kernel`: SLAM map의 작은 점 노이즈를 제거하는 median 필터 크기이다. 홀수 값으로 사용된다.
- `--morph-kernel`: map cleanup에 쓰는 픽셀 커널 크기이다. 노이즈가 많은 map에서는 키우되, 너무 크면 좁은 통로가 사라질 수 있다.
- `--morph-open-iterations`: 작은 free-space 점 노이즈를 제거하는 반복 횟수이다.
- `--morph-close-iterations`: 끊긴 free-space를 메우는 반복 횟수이다.
- `--skeleton-prune-iterations`: skeleton의 막다른 가지를 반복적으로 잘라내는 횟수이다.
- `--min-skeleton-component-area`: 너무 작은 skeleton 조각을 버리는 최소 픽셀 수이다.
- `--min-centerline-angle`: 경로가 한 점에서 되돌아가는 핀 형태를 제거하기 위한 최소 진행 각도이다.
- `--spike-filter-iterations`: 핀 제거 필터를 반복하는 횟수이다.
- `--reverse`: 생성된 waypoint 주행 방향을 반대로 뒤집는다.
- `--debug-image`: map 이미지 위에 centerline과 global trajectory를 그린 PNG를 저장한다.
- `--optimizer centerline`: 기본값이다. SLAM map에서 안정적으로 동작한다.
- `--optimizer mincurv`: scipy 기반 최소 곡률 보정을 시도한다. map 품질에 따라 튜닝이 필요할 수 있다.
- `--max-optimizer-iter`: 최소 곡률 최적화(L-BFGS-B)의 최대 반복 횟수이다. 기본값은 `200`이다. `optimizer-step`을 적정 범위로 두면 보통 이 한도 안에서 정상 수렴한다. 변수가 많거나 map이 어려운 경우에만 한도에 도달하는데, 이때도 그때까지 찾은 최적 경로를 그대로 사용하므로 별도 경고를 출력하지 않는다. 경고가 자주 보인다면 `optimizer-step`을 키우는 것이 정석적인 해결책이다.
- `--no-straighten-straights`: 직선 후보 구간을 직선으로 보정하는 후처리를 끈다.
- `--straight-kappa-threshold`: 이 값보다 작은 절대 곡률을 직선 후보로 본다. 기본값은 `0.2` rad/m이다.
- `--straight-min-length`: 직선 후보로 인정할 최소 구간 길이이다.
- `--straight-clearance-margin`: 직선 보정 검증에 추가로 요구하는 벽 여유 거리이다.
- `--straight-blend-length`: 직선 보정 구간 양끝에서 원래 경로와 직선을 부드럽게 섞는 길이이다.

ROS map의 trinary 회색 unknown 영역은 기본적으로 주행 가능 영역에서 제외된다. 회색 unknown까지 free-space로 쓰고 싶을 때만 GUI의 `Unknown as free`를 켠다.

직선 보정은 곡률이 낮은 구간의 내부 waypoint를 endpoint 직선 위로 당긴 뒤, distance transform으로 전체 직선이 free-space와 clearance 조건을 만족하는 경우에만 적용한다. 따라서 SLAM 노이즈 때문에 직선 구간의 곡률이 흔들리는 경우 속도 프로파일이 더 높게 잡힌다. 경로가 너무 많이 당겨지면 `straight_kappa_threshold`를 낮추고, 직선화가 부족하면 값을 올린다.

## 4. 생성 절차

1. SLAM으로 map YAML과 이미지 파일을 저장한다.
2. 위 실행 명령으로 `global_waypoints.json`을 만든다.
3. `debug_overlay.png`를 열어서 경로가 트랙 중앙을 따라가는지 확인한다.
4. 방향이 반대이면 같은 명령에 `--reverse`를 추가해서 다시 생성한다.
5. 속도가 너무 높거나 낮으면 `--max-speed`, `--min-speed`, `--max-lateral-accel`을 조정한다.

GUI를 사용할 때는 3-5단계를 화면에서 바로 확인한 뒤 `Save`만 누르면 된다.

## 5. 출력 파일 설명

`global_waypoints.csv` 컬럼:

```text
id,s,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,d_left,d_right
```

CSV 포맷은 `src/new_map_con/maps/fuck_f1.csv`와 같은 10개 컬럼 구조를 사용한다. `d_left`/`d_right`는 각 waypoint에서 트랙 좌/우 경계까지의 거리[m]이며, `--width-mode`로 계산 방식을 선택한다:

- `hybrid` (기본, 권장): 방향별 robust raycast로 좌/우 경계를 개별 측정한다. 최소 2픽셀 연속 벽을 요구해 단일 픽셀 노이즈/구멍을 무시하고, ray가 gap으로 새어 max 거리에 도달하면 distance transform 최근접 거리로 대체한다. 커브에서 raceline이 안쪽을 파고들면 좌/우 비대칭이 그대로 반영된다.
- `distance`: distance transform 기반. 빠르지만 최근접 벽 한 값만 주므로 `d_left == d_right`가 되어 좌/우 구분이 불가능하다.
- `raycast`: 방향별 raycast만 사용(robust 게이트 포함). gap 새어나감 보정은 없다.

`d_left`/`d_right`는 `opponent_detector`의 회피 안전 마진 계산과 `new_map_con`의 경로 경계 판정에 직접 사용되므로, 좌/우 비대칭이 필요한 회피 로직에는 `hybrid` 모드를 사용해야 한다. `--max-width-distance`는 raycast 상한이며 트랙 폭에 맞춰 설정한다(F1TENTH 실내 트랙은 3.0m 권장). `d_m`, `s_m` 등 나머지 ROS waypoint 필드는 `global_waypoints.json` 안에 유지된다.

CSV의 `x_m`, `y_m`은 선택한 ROS map YAML의 `resolution`과 `origin`이 적용된 map frame 좌표이다. RViz에서 map과 path가 어긋나면 generator에 넣은 map YAML과 f1sim/map server가 띄운 map YAML이 같은 파일인지 먼저 확인한다.

`global_waypoints.json` 주요 필드:

- `global_traj_wpnts_iqp`: 실제 주행에 사용할 waypoint 배열
- `centerline_waypoints`: 추출된 centerline waypoint 배열
- `est_lap_time`: 속도 프로파일 기반 예상 랩타임
- `map_info_str`: 입력 map과 생성 설정 요약
- `*_markers` (`global_traj_markers_iqp`, `trackbounds_markers`, `centerline_markers`, `global_traj_markers_sp`): RViz 시각화용 `visualization_msgs/MarkerArray`. waypoint 배열로부터 자동 생성한다 — raceline은 LINE_STRIP + 속도색 SPHERE_LIST(파랑 느림→빨강 빠름), 트랙 경계는 좌/우 LINE_STRIP(waypoint pose와 `d_left/d_right`로 계산). 모든 마커는 `frame_id=map`. `global_planning`의 `global_trajectory_publisher_node`가 이 마커를 그대로 `/global_waypoints/markers` 등으로 재발행하므로 별도 노드 없이 RViz에서 바로 보인다. (마커 스타일 상수는 `generate_global_trajectory.py`의 마커 헬퍼 상단에 있다.)

## 6. 의존성

현재 환경에서는 `numpy`, `opencv`, `PyYAML`, `scipy`가 사용된다.

```bash
python3 -m pip install -r offline_trajectory_generator/requirements.txt
```

ROS 2, `rclpy`, `quadprog`, `skimage`는 필요하지 않다.

GUI는 Python 표준 라이브러리인 `tkinter`를 사용한다. Ubuntu에서 `tkinter`가 빠져 있으면 다음 패키지를 설치한다.

```bash
sudo apt install python3-tk
```
