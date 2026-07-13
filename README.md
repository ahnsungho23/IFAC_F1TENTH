# 2026 IFAC F1TENTH

F1TENTH 자율주행 스택입니다. 위치추정(MCL), 글로벌 플래닝, 로컬 플래닝(장애물 회피), 제어를 포함합니다.

시뮬레이터는 별도 워크스페이스(`~/sim_ws`)의 `f1tenth_gym_ros`를 사용합니다.

---

## 프로젝트 구조

ROS 2 Humble workspace for the 2026 IFAC F1TENTH stack. ROS packages live under `src/`.

```text
2026_IFAC/
├── AGENTS.md                     # repo-wide agent rules
├── CLAUDE.md                     # repo-wide working rules
├── README.md
│
├── f110_msgs/                    # shared interface package (custom messages)
│   ├── msg/
│   │   ├── CarState.msg
│   │   ├── CarStateStamped.msg
│   │   ├── GapData.msg
│   │   ├── LapData.msg
│   │   ├── OTWpntArray.msg
│   │   ├── Obstacle.msg
│   │   ├── ObstacleArray.msg
│   │   ├── OppWpnt.msg
│   │   ├── OpponentTrajectories.msg
│   │   ├── OpponentTrajectory.msg
│   │   ├── PidData.msg
│   │   ├── ProjOppPoint.msg
│   │   ├── ProjOppTraj.msg
│   │   ├── StateMachine.msg
│   │   ├── Wpnt.msg
│   │   └── WpntArray.msg
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── README.md
│
├── offline_trajectory_generator/ # offline global-trajectory + centerline generator (standalone Python, GUI)
│   ├── generate_global_trajectory.py
│   ├── trajectory_gui.py
│   ├── gui_params.yaml
│   ├── requirements.txt
│   ├── output/                   # generated maps (e.g. fuck_f1/: global_waypoints.{csv,json}, centerline.csv, metadata.json)
│   ├── AGENTS.md
│   └── README.md / README_en.md
│
├── src/                          # ROS 2 packages
│   ├── f1tenth_control/          # vehicle control: AEB, gap follower, IMU stability, MPC, steering, joy teleop
│   │   ├── config/aeb_params.yaml
│   │   ├── control_code/         # *.cpp nodes + MAP controller reference / steer lookup (Python)
│   │   ├── include/f1tenth_control/
│   │   ├── launch/               # aeb / control_real / control_sim / joy
│   │   ├── vesc_appconf.xml, vesc_mcconf.xml
│   │   ├── CMakeLists.txt, package.xml, CLAUDE.md, WORKLOG.md
│   │
│   ├── global_planning/          # global trajectory publisher + CLCS frenet odom
│   │   ├── config/global_planning.yaml
│   │   ├── docs/frenet_odom_node.md
│   │   ├── include/global_planning/   # clcs_frenet_converter.hpp, *_node.hpp, readwrite_global_waypoints.hpp
│   │   ├── src/                        # frenet_odom_node, global_planning_node, global_trajectory_publisher_node, clcs_frenet_converter, readwrite_global_waypoints
│   │   ├── test/test_clcs_frenet_converter.cpp
│   │   ├── vendor/
│   │   │   ├── commonroad_clcs/   # (vendored) CommonRoad-CLCS C++ curvilinear coordinate core
│   │   │   └── gb_optimizer/      # (vendored) TUM global_racetrajectory_optimization + trajectory_planning_helpers (Python)
│   │   ├── AGENTS.md, CMakeLists.txt, package.xml
│   │
│   ├── local_planning/           # static-obstacle avoidance local planner (least-squares spline)
│   │   ├── config/local_planning.yaml
│   │   ├── docs/local_planner.md
│   │   ├── include/local_planning/local_planner_node.hpp
│   │   ├── launch/local_planning.launch.py
│   │   ├── src/local_planner_node.cpp
│   │   ├── AGENTS.md, CMakeLists.txt, package.xml
│   │
│   ├── opponent_detector/        # opponent detection + tracking + overtake planner
│   │   ├── config/opponent_detector.yaml
│   │   ├── docs/                 # opponent_detector_node.md, sim_test_commands.md
│   │   ├── include/opponent_detector/  # frenet_projector, obstacle_tracker, opponent_detector_node, overtake_planner
│   │   ├── src/                        # frenet_projector, obstacle_tracker, opponent_detector_node, overtake_planner
│   │   ├── rviz/opponent_detector.rviz
│   │   ├── test/                       # commit_lock, offpath_recovery, synthetic_opponent, trail_to_overtake
│   │   ├── AGENTS.md, CMakeLists.txt, package.xml, README(.en).md
│   │
│   ├── state_machine/            # driving-mode state machine (GLOBAL / AVOID / OVERTAKE)
│   │   ├── config/state_machine.yaml
│   │   ├── include/state_machine/state_machine_node.hpp
│   │   ├── launch/state_machine.launch.py
│   │   ├── src/state_machine_node.cpp
│   │   ├── AGENTS.md, CMakeLists.txt, package.xml
│   │
│   ├── wpnt_publisher/           # /state-driven local waypoint selection/publisher
│   │   ├── src/wpnt_publisher.cpp
│   │   ├── config/ docs/ launch/ (empty)
│   │   ├── CMakeLists.txt, package.xml, LICENSE
│   │
│   ├── new_map_con/              # map controller + opponent simulator/drive controller
│   │   ├── config/config.yaml
│   │   ├── docs/map_controller_node.md
│   │   ├── launch/               # new_map_con, opponent_simulator
│   │   ├── maps/                 # 6 files: fuck_f1.{csv,yaml,pgm}, oct28.csv, ...
│   │   ├── src/                  # map_controller_node, opponent_drive_controller, opponent_simulator_node
│   │   ├── test/                 # copyright / flake8 / pep257
│   │   ├── AGENTS.md, CMakeLists.txt, setup.py, setup.cfg, package.xml, README(.en).md
│   │
│   ├── monte_carlo_localization/ # particle-filter localization (C++)
│   │   ├── config/mcl_config.yaml
│   │   ├── include/particle_filter_cpp/  # particle_filter.hpp, utils.hpp
│   │   ├── launch/mcl_launch.py
│   │   ├── maps/                 # 2 files (e.g. fuck_f1.{pgm,yaml})
│   │   ├── rviz/particle_filter.rviz
│   │   ├── src/                  # particle_filter.cpp, utils.cpp
│   │   ├── CMakeLists.txt, package.xml, README.md
│   │
│   ├── lap_referee/              # lap refereeing (C++)
│   │   ├── config/lap_referee.yaml
│   │   ├── docs/lap_referee_node.md
│   │   ├── launch/lap_referee.launch.py
│   │   ├── src/lap_referee_node.cpp
│   │   ├── AGENTS.md, CMakeLists.txt, package.xml, README(.en).md
│   │
│   └── lap_timer/                # lap timing + RViz HUD (Python)
│       ├── config/params.yaml
│       ├── lap_timer/            # __init__.py, lap_timer_node.py
│       ├── launch/lap_timer.launch.py
│       ├── rviz/lap_hud.rviz
│       ├── resource/lap_timer
│       ├── test/                 # copyright / flake8 / pep257
│       ├── setup.py, setup.cfg, package.xml
│
└── third_party/
    └── python_libs/
        └── steering_lookup/      # steering-angle lookup lib (ament_python)
            ├── cfg/              # lookup tables (*.csv) + analyse_tires.py
            ├── steering_lookup/  # __init__.py, lookup_steer_angle.py
            ├── resource/, test/
            └── package.xml, setup.py, setup.cfg, LICENSE, README.md
```

**Notes**

- Build/runtime artifacts are excluded: `build/`, `install/`, `log/`, `.matplotlib/`, and hidden folders (`.git`, `.vscode`, …).
- `f110_msgs` is the shared interface package. Prefer these types (and `std_msgs`) for inter-node communication.
- `global_planning/vendor/` holds vendored upstream code (CommonRoad-CLCS C++ core, TUM race-trajectory optimizer) — collapsed here; see each vendor subdir's own LICENSE/README.
- Large asset dirs (`*/maps/`, `offline_trajectory_generator/output/`) are summarized by file count/example rather than listed in full.
- Package roles (one line each) are annotations, not part of the on-disk layout.

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

기본 주행은 터미널 7개를 아래 순서대로 띄웁니다. 순서가 중요합니다 — 4장의 주의사항을 먼저 읽어보세요.
상대차 검출·추월까지 보려면 §3-1의 터미널 8·9를 **추가로** 띄웁니다(터미널 7은 유지).

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
ros2 launch global_planning global_planning.launch.py
```

### 터미널 4 — 로컬 플래너 (장애물 회피)

`/map`의 점유 격자에서 장애물을 찾아 최소자승 3차 스플라인 회피 경로를 만듭니다.

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch local_planning local_planning.launch.py
```

### 터미널 5 — 상태 머신 (state machine)

`/car_state/frenet/odom`·`/avoid_waypoints`·`/overtake_waypoints`·`/global_waypoints`를 종합해 주행 상태(GLOBAL/AVOID/OVERTAKE)를 판정하고 `/state`로 발행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch state_machine state_machine.launch.py
```

### 터미널 6 — 웨이포인트 퍼블리셔

회피 웨이포인트를 받아 `/local_waypoints`로 중계합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 run wpnt_publisher wpnt_publisher
```

### 터미널 7 — 제어

L1 Guidance + Steering LUT 기반 조향/속도 제어. `force_autonomous:=true`면 조이스틱 없이 즉시 자율주행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_control control_sim.launch.py force_autonomous:=true
```

---

## 3-1. (선택) 상대차 검출·추월 시나리오 — 터미널 8·9

> 아래 두 터미널은 **상대차 검출/추월을 볼 때만** 추가로 띄웁니다. 기본 주행에는 필요 없습니다.
>
> **전제 2가지**
> 1. 터미널 1의 gym 시뮬을 **`num_agent: 2`** (sim_ws의 `config/sim.yaml`)로 띄워야 상대차량이 스폰됩니다. 1-agent면 상대차가 아예 없어 RViz에도 안 보이고 검출도 안 됩니다. (sim.yaml 수정 후 gym 브리지를 **재실행**해야 반영됨)
> 2. 이 2-agent 브리지는 **에고·상대 둘 다 `drive`를 발행해야 물리 스텝**을 돕니다. 따라서 **터미널 7(에고 제어)을 그대로 유지**해야 하며, 8·9는 교체가 아니라 **추가**입니다.

### 터미널 8 — 상대차 주행 (opponent simulator)

global 라인을 0.8배속으로 따라가도록 f1sim 상대차량에 `/opp_drive`를 발행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch new_map_con opponent_simulator.launch.py
```

### 터미널 9 — 상대차 검출기 (opponent detector)

에고 `/scan`으로 상대차를 검출해 `/perception/obstacles`·`/proj_opponent_trajectory`를 발행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch opponent_detector opponent_detector.launch.py simulator:=true
```

RViz에서 `/perception/obstacles/markers`(빨강=동적 상대차, 파랑=정적)를 Add 하면 검출 결과가 보입니다.

---

## 4. 주의사항

### 4.1 반드시 `~/2026_IFAC`에서 실행할 것

`src/global_planning/config/global_planning.yaml`의 경로가 **상대경로**입니다.

```yaml
# global_trajectory_publisher_node: JSON을 <output_base_dir>/<map_name>/global_waypoints.json 에서 읽음
output_base_dir: "offline_trajectory_generator/output"
map_name: "fuck_f1"
```

따라서 워크스페이스 루트가 아닌 곳에서 터미널 3을 실행하면 파일을 찾지 못합니다.

### 4.2 `global_waypoints.json`이 먼저 있어야 함

`global_trajectory_publisher_node`는 시작할 때 JSON을 **딱 한 번만** 읽습니다. 파일이 없거나 파싱에 실패하면
`/global_waypoints`를 아예 발행하지 않고, 그 뒤의 로컬 플래너와 제어가 전부 대기 상태로 멈춥니다.

읽는 위치는 `<output_base_dir>/<map_name>/global_waypoints.json`(기본 `offline_trajectory_generator/output/fuck_f1/`)입니다.
**맵을 바꾸려면** yaml의 `map_name`만 그 맵 이름(= `offline_trajectory_generator/output/` 하위 폴더명)으로 바꾸면 됩니다.
새 맵을 생성했거나 파일을 교체했다면 터미널 3을 재시작해야 반영됩니다.

### 4.4 파라미터 파일을 명시하고 싶다면

각 launch 파일은 설치된 share 디렉터리의 YAML을 기본값으로 씁니다. symlink-install이므로 이 기본값은
소스의 YAML을 그대로 가리킵니다. 굳이 지정할 필요는 없지만, 다른 파일을 쓰려면:

```bash
ros2 launch global_planning global_planning.launch.py \
  params_file:=$HOME/2026_IFAC/src/global_planning/config/global_planning.yaml

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
[터미널 3] global_planning
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
                    ├──► [터미널 5] state_machine
                    │        /car_state/frenet/odom·/global_waypoints·/avoid_waypoints·/overtake_waypoints 구독
                    │        ──► /state (GLOBAL / AVOID / OVERTAKE)
                    ▼
[터미널 6] wpnt_publisher ──► /local_waypoints, /local_waypoints/path
                    │
                    ▼
[터미널 7] f1tenth_control
     /global_waypoints + /local_waypoints 구독
     ──► /drive_autonomous ──(Mux)──► /drive
```
