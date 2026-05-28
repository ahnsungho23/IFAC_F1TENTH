# Offline Trajectory Generator

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

처음부터 특정 map을 열고 싶으면 다음처럼 넘긴다.

```bash
python3 offline_trajectory_generator/trajectory_gui.py \
  --map-yaml monte_carlo_localization/maps/slam_map.yaml
```

화면 왼쪽에는 map 경로, 저장 위치, trajectory 파라미터가 표시된다. 오른쪽에는 map 이미지 위에 centerline과 RT lane이 함께 표시된다. 슬라이더나 체크박스를 바꾸면 잠시 후 자동으로 다시 계산되어 overlay가 갱신된다.

각 숫자 파라미터는 슬라이더 왼쪽에 현재 값이 표시되고, 슬라이더 오른쪽 입력칸에 값을 직접 입력할 수도 있다. 입력값은 해당 파라미터의 허용 범위와 단위에 맞게 자동으로 반올림/제한된다.

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
- `--optimizer-step`: 내부 centerline/RT lane 계산 간격이다. 값이 작을수록 더 촘촘하지만 느리다.
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

ROS map의 trinary 회색 unknown 영역은 기본적으로 주행 가능 영역에서 제외된다. 회색 unknown까지 free-space로 쓰고 싶을 때만 GUI의 `Unknown as free`를 켠다.

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
id,s_m,d_m,x_m,y_m,d_right,d_left,psi_rad,kappa_radpm,vx_mps,ax_mps2
```

`global_waypoints.json` 주요 필드:

- `global_traj_wpnts_iqp`: 실제 주행에 사용할 waypoint 배열
- `centerline_waypoints`: 추출된 centerline waypoint 배열
- `est_lap_time`: 속도 프로파일 기반 예상 랩타임
- `map_info_str`: 입력 map과 생성 설정 요약

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
