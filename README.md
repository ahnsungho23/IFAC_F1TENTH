# 2026 IFAC F1TENTH

F1TENTH 자율주행 스택입니다. 위치추정(MCL), 글로벌 플래닝, 로컬 플래닝(장애물 회피), 제어를 포함합니다.

시뮬레이터는 별도 워크스페이스(`~/sim_ws`)의 `f1tenth_gym_ros`를 사용합니다.

---

## 1. 사전 요구사항

- ROS 2 Humble
- 시뮬레이터 워크스페이스 `~/sim_ws` (`f1tenth_gym_ros` 빌드 완료)
- 이 저장소가 `~/2026_IFAC`에 위치

---

## 2. 빌드

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
```

> **`--symlink-install`을 반드시 붙이세요.**
> 이 워크스페이스는 symlink-install 모드로 통일되어 있습니다. 일반 `colcon build`와 섞어 쓰면
> `AMENT_CMAKE_SYMLINK_INSTALL` 캐시가 엇갈려 `f110_msgs`에서 심볼릭 링크 충돌로 빌드가 깨집니다.
> 이미 깨졌다면 `rm -rf build install` 후 위 명령으로 다시 빌드하면 됩니다.

symlink-install 덕분에 `config/*.yaml`과 `launch/*.launch.py`는 수정 후 **재빌드 없이** 바로 반영됩니다.

---

## 3. 실행 순서

터미널 6개를 아래 순서대로 띄웁니다. 순서가 중요합니다 — 4장의 주의사항을 먼저 읽어보세요.

각 터미널 공통 준비:

```bash
source /opt/ros/humble/setup.zsh
source install/setup.zsh   # bash 사용 시 setup.bash
```

### 터미널 1 — 시뮬레이터 (gym bridge)

```bash
cd ~/sim_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### 터미널 2 — 위치추정 (Monte Carlo Localization)

`/pf/pose/odom`을 발행합니다. 이후 모든 노드가 이 토픽에 의존합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch particle_filter_cpp mcl_launch.py mod:=sim map_name:=fuck_f1 use_rviz:=true
```

| 인자 | 값 | 설명 |
|---|---|---|
| `mod` | `sim` | 시뮬레이션 모드 (`/ego_racecar/odom` 사용, sim time 활성) |
| `map_name` | `fuck_f1` | `monte_carlo_localization/maps/fuck_f1.yaml` |
| `use_rviz` | `true` | RViz 동시 실행 |

### 터미널 3 — 글로벌 플래너

`global_waypoints.json`을 읽어 `/global_waypoints`를 발행하고, `/car_state/frenet/odom`을 계산합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch global_planner global_planning.launch.py
```

### 터미널 4 — 로컬 플래너 (장애물 회피)

`/map`의 점유 격자에서 장애물을 찾아 최소자승 3차 스플라인 회피 경로를 만듭니다.

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch local_planning local_planning.launch.py
```

### 터미널 5 — 웨이포인트 퍼블리셔

회피 웨이포인트를 받아 `/local_waypoints`로 중계합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 run wpnt_publisher wpnt_publisher
```

### 터미널 6 — 제어

L1 Guidance + Steering LUT 기반 조향/속도 제어. `force_autonomous:=true`면 조이스틱 없이 즉시 자율주행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_control control_sim.launch.py force_autonomous:=true
```

---

## 4. 주의사항

### 4.1 반드시 `~/2026_IFAC`에서 실행할 것

`planning/global_planner/config/global_planning.yaml`의 경로가 **상대경로**입니다.

```yaml
map_dir: "planning/global_planner/data"
optimizer_command: "python3 planning/global_planner/vendor/gb_optimizer/src/global_planner_node_ros2.py"
```

따라서 워크스페이스 루트가 아닌 곳에서 터미널 3을 실행하면 파일을 찾지 못합니다.

### 4.2 `global_waypoints.json`이 먼저 있어야 함

`global_trajectory_publisher_node`는 시작할 때 JSON을 **딱 한 번만** 읽습니다. 파일이 없거나 파싱에 실패하면
`/global_waypoints`를 아예 발행하지 않고, 그 뒤의 로컬 플래너와 제어가 전부 대기 상태로 멈춥니다.

현재는 `planning/global_planner/data/global_waypoints.json`이 커밋되어 있어 그대로 쓰면 됩니다.
맵을 바꿔 새로 생성했다면 터미널 3을 재시작해야 반영됩니다.

### 4.3 `/local_waypoints`에 퍼블리셔가 둘입니다

터미널 4의 `local_planner_node`(`publish_standalone_local: true`가 기본)와 터미널 5의 `wpnt_publisher`가
**같은 토픽 `/local_waypoints`를 동시에 발행**합니다. 제어 노드는 두 스트림을 섞어서 받게 됩니다.

둘 중 하나만 쓰려면:

- `planning/local_planning/config/local_planning.yaml`에서 `publish_standalone_local: false`로 두거나
- 터미널 5를 띄우지 않습니다

### 4.4 파라미터 파일을 명시하고 싶다면

각 launch 파일은 설치된 share 디렉터리의 YAML을 기본값으로 씁니다. symlink-install이므로 이 기본값은
소스의 YAML을 그대로 가리킵니다. 굳이 지정할 필요는 없지만, 다른 파일을 쓰려면:

```bash
ros2 launch global_planner global_planning.launch.py \
  params_file:=$HOME/2026_IFAC/planning/global_planner/config/global_planning.yaml

ros2 launch local_planning local_planning.launch.py \
  params_file:=$HOME/2026_IFAC/planning/local_planning/config/local_planning.yaml
```

파일명은 `local_planning.yaml`입니다 (`local_plannng.yaml` 아님).

---

## 5. 토픽 흐름

```
[터미널 1] f1tenth_gym_ros
     │  /scan, /ego_racecar/odom, /map
     ▼
[터미널 2] particle_filter_cpp ──► /pf/pose/odom
                                        │
     ┌──────────────────────────────────┤
     ▼                                  ▼
[터미널 3] global_planner
     ├─ global_trajectory_publisher_node ──► /global_waypoints
     └─ frenet_odom_node ────────────────► /car_state/frenet/odom
                                        │
     ┌──────────────────────────────────┘
     ▼
[터미널 4] local_planning
     ├──► /local_path, /local_planning/path, /local_planning/markers
     ├──► /local_waypoints            ◄── 4.3 참고 (중복)
     └──► /planner/avoidance/otwpnts
                    │
                    ▼
[터미널 5] wpnt_publisher ──► /local_waypoints, /local_waypoints/path
                    │
                    ▼
[터미널 6] f1tenth_control
     /global_waypoints + /local_waypoints 구독
     ──► /drive_autonomous ──(Mux)──► /drive
```

---

## 6. 패키지 구성

| 패키지 | 역할 |
|---|---|
| `f110_msgs` | `WpntArray`, `OTWpntArray` 등 공용 메시지 |
| `monte_carlo_localization` (`particle_filter_cpp`) | 파티클 필터 위치추정 |
| `planning/global_planner` | 글로벌 경로 발행 + Frenet 좌표 변환 |
| `planning/local_planning` | 정적 장애물 회피 로컬 경로 생성 |
| `planning/state_machine` | 주행 상태 관리 |
| `wpnt_publisher` | 로컬 웨이포인트 중계 |
| `f1tenth_control` | L1 Guidance 조향 제어, AEB, 수동/자율 Mux |
| `lap_timer` | 랩타임 측정 |
| `offline_trajectory_generator` | 오프라인 경로 생성 GUI (ROS 노드 아님) |

세부 문서는 각 패키지의 `docs/` 및 [planning/PLANNING_PIPELINE.md](planning/PLANNING_PIPELINE.md)를 참고하세요.
