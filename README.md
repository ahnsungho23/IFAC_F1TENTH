# 2026 IFAC F1TENTH

F1TENTH 자율주행 스택입니다. 위치추정(MCL), 글로벌 플래닝, 로컬 플래닝(장애물 회피), 제어를 포함합니다.

**대상 배포판: ROS 2 Jazzy** (Ubuntu 24.04). Jazzy 포팅 및 검증 내역은 [§6. ROS 2 Jazzy 포트 검증](#6-ros-2-jazzy-포트-검증)을 참고하세요.

시뮬레이터는 별도 워크스페이스(`~/f1sim_C`)의 `f1tenth_gym_ros`를 사용합니다.
시뮬레이터 맵(`f1tenth_gym_ros/config/sim.yaml`의 `map_path`)과 MCL 맵, 글로벌 웨이포인트 생성 맵 **세 곳은 반드시 같은 맵**이어야 합니다.


slam launch방법 로컬에서
cd ~/slam_toolbox
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

slam 저장방법 로컬에서
cd ~/slam_toolbox
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 run nav2_map_server map_saver_cli \
    -f ~/slam_toolbox/map
    
딴 맵을 로컬에서 젯슨으로 맵 전송
scp ~/slam_toolbox/map.png ~/slam_toolbox/map.yaml \
    miru@10.1.1.3:~/2026_IFAC/src/monte_carlo_localization/maps/
    
    
offline gui 파일을 로컬에서 젯슨으로 전송
scp -r ~/2026_IFAC/offline_trajectory_generator/output/map \
    miru@10.1.1.3:~/2026_IFAC/offline_trajectory_generator/output/

터미널1
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch particle_filter_cpp mcl_launch.py mod:=real map_name:=ifac_track

터미널2
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch global_planning global_planning.launch.py

터미널3
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch local_planning local_planning.launch.py

터미널4
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch state_machine state_machine.launch.py

터미널5
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source ~/f1tenth_ws/install/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_control control_real.launch.py



rosbag
cd ~/miru/2026_IFAC

source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

ros2 bag record \
  /tf \
  /odom \
  /scan \
  /pf/pose/odom \
  /debug/l1_lookahead \
  /sensors/imu/raw \
  /estop_lock \
  /commands/motor/brake \
  /joy \
  /debug/l1_lookahead \
  /drive_autonomous\ 
  /drive

---

## 프로젝트 구조

ROS 2 Jazzy workspace for the 2026 IFAC F1TENTH stack. ROS packages live under `src/`.

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
│   ├── output/                   # generated maps (e.g. ifac_track/: global_waypoints.{csv,json}, centerline.csv, metadata.json) — gitignored
│   ├── AGENTS.md
│   └── README.md / README_en.md
│
├── src/                          # ROS 2 packages
│   ├── f1tenth_control/          # 차량 제어: MAP(L1 Guidance+Steering LUT)/MPPI 이중 컨트롤러, MAP/MPPI 셀렉터, LUT 실측 보정 (teleop Mux 없음 — 실차 f1tenth_stack 담당)
│   │   ├── control_code/         # control_map_node, control_mppi_node(+solver_cpu/gpu), drive_source_selector, realcar_dashboard_node, odom_calib_node, lut_calibrator_node, sim_imu_bridge_node, gap_follower, imu_stability_controller, steer lookup(Python/CSV)
│   │   ├── include/f1tenth_control/
│   │   ├── launch/               # control_real / control_sim / dashboard / lut_calibration
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
│   ├── obstacle_detector/        # layered LiDAR obstacle detection (static /static_obs + opponent /opp_obs)
│   │   ├── config/obstacle_detector.yaml
│   │   ├── docs/                 # obstacle_detector_node.md, sim_test_commands.md
│   │   ├── include/obstacle_detector/  # frenet_projector, obstacle_tracker, obstacle_detector_node
│   │   ├── src/                        # frenet_projector, obstacle_tracker, obstacle_detector_node
│   │   ├── rviz/obstacle_detector.rviz
│   │   ├── test/                       # synthetic_opponent
│   │   ├── AGENTS.md, CMakeLists.txt, package.xml, README(.en).md
│   │
│   ├── static_obstacle_map/      # persistent confirmed-static OccupancyGrid composer
│   │   ├── config/static_obstacle_map.yaml
│   │   ├── docs/static_obstacle_map_node.md
│   │   ├── include/static_obstacle_map/
│   │   ├── launch/static_obstacle_map.launch.py
│   │   ├── src/
│   │   ├── test/
│   │   ├── AGENTS.md, CMakeLists.txt, package.xml, README.md
│   │
│   ├── state_machine/            # driving-mode FSM + final local waypoint selector
│   │   ├── config/state_machine.yaml
│   │   ├── include/state_machine/state_machine_node.hpp
│   │   ├── launch/state_machine.launch.py
│   │   ├── src/state_machine_node.cpp
│   │   ├── docs/state_machine_node.md
│   │   ├── AGENTS.md, CMakeLists.txt, package.xml
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
│   │   ├── maps/                 # ifac_track.{png,yaml} + ifac_track.md
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

- ROS 2 Jazzy (Ubuntu 24.04)
- 시뮬레이터 워크스페이스 `~/f1sim_C` (`f1tenth_gym_ros` Jazzy 빌드 완료)
- 이 저장소가 `~/2026_IFAC`에 위치
- 글로벌 웨이포인트 생성 완료 (§4.2 참고 — 없으면 스택 전체가 대기 상태에 머뭅니다)

---

## 2. 빌드

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install
```

> **`--symlink-install`을 반드시 붙이세요.**
> 이 워크스페이스는 symlink-install 모드로 통일되어 있습니다. 일반 `colcon build`와 섞어 쓰면
> `AMENT_CMAKE_SYMLINK_INSTALL` 캐시가 엇갈려 `f110_msgs`에서 심볼릭 링크 충돌로 빌드가 깨집니다.
> 이미 깨졌다면 `rm -rf build install` 후 위 명령으로 다시 빌드하면 됩니다.

> **주의**: `config/*.yaml`·`launch/*.launch.py`는 대부분 패키지가 `install(DIRECTORY ...)`로 설치하므로
> symlink-install이어도 **실제 심볼릭 링크가 아닙니다** (ament의 symlink 훅은 `install(FILES/TARGETS)`만
> 가로챕니다). launch/config를 수정했다면 해당 패키지를 다시 빌드해야 반영됩니다:
> `colcon build --symlink-install --packages-select <pkg>`

---

## 3. 실행 순서

기본 주행은 터미널 6개를 아래 순서대로 띄웁니다. 순서가 중요합니다 — 4장의 주의사항을 먼저 읽어보세요.
상대차 검출·추월까지 보려면 §3-1의 터미널 7·8을 **추가로** 띄웁니다(터미널 6은 유지).

> **원클릭 실행**: Terminator가 설치되어 있으면 `./sim/open_sim.sh`(터미널 1~6 분할) 또는
> `./sim/open_sim.sh --opp`(터미널 1~8 분할) 한 번으로 전체 스택을 띄울 수 있습니다.
> 자세한 내용은 `sim/README.md` 참고.

각 터미널 공통 준비:

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh   # bash 사용 시 setup.bash
```

### 터미널 1 — 시뮬레이터 (gym bridge)

먼저 `~/f1sim_C/f1tenth_gym_ros/config/sim.yaml`의 `map_path`가 스택과 같은 맵을 가리키는지 확인하세요
(확장자 없는 절대경로, 예: `$HOME/2026_IFAC/src/monte_carlo_localization/maps/ifac_track` — gym의 YAML은 `$HOME`을 펼치지 않으므로 실제 값은 펼쳐서 적습니다).

```bash
cd ~/f1sim_C
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### 터미널 2 — 위치추정 (Monte Carlo Localization)

`/pf/pose/odom`을 발행합니다. 이후 모든 노드가 이 토픽에 의존합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch particle_filter_cpp mcl_launch.py mod:=sim map_name:=ifac_track use_rviz:=true
```

| 인자 | 값 | 설명 |
|---|---|---|
| `mod` | `sim` | 시뮬레이션 모드 (`/ego_racecar/odom` 사용, sim time 활성) |
| `map_name` | `ifac_track` | `monte_carlo_localization/maps/ifac_track.yaml` |
| `use_rviz` | `true` | RViz 동시 실행 |

> **초기 위치 지정(필수)**: MCL은 전역 초기화로 시작하므로 RViz의 **2D Pose Estimate**로
> 시작 위치를 찍어줘야 정확히 수렴합니다. RViz 없이(헤드리스) 돌릴 때는 아래처럼 직접 발행하세요.
> `/initialpose`는 gym 브리지도 구독하므로 **차량 텔레포트와 MCL 초기화가 동시에** 일어납니다.
> (주행 중 재초기화할 때는 차량을 먼저 정지시킨 뒤 두 번 발행하면 확실합니다 — 첫 발행의 텔레포트
> 순간 odom 점프가 MCL에 반영되는 것을 두 번째 발행이 정리해 줍니다.)
>
> ```bash
> ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
>   '{header: {frame_id: map}, pose: {pose: {position: {x: -0.427, y: 0.456}, orientation: {z: 0.3651, w: 0.9310}}}}'
> ```
>
> 위 좌표는 `ifac_track` 레이스라인 위의 스폰 인접 지점(헤딩 정렬)입니다.

### 터미널 3 — 글로벌 플래너

`global_waypoints.json`을 읽어 `/global_waypoints`를 발행하고, `/car_state/frenet/odom`을 계산합니다.
글로벌/로컬 플래너의 기본 맵 이름은 실차용 `map`이므로, 시뮬에서는 `F1_MAP=ifac_track`으로
시뮬 트랙을 명시해야 합니다 (`./sim/open_sim.sh` 사용 시 자동 설정).

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
F1_MAP=ifac_track ros2 launch global_planning global_planning.launch.py
```

### 터미널 4 — 로컬 플래너 (장애물 회피)

`/static_obs`(obstacle_detector Layer 2의 확정 정적 장애물, `f110_msgs/ObstacleArray`)를 받아
글로벌 라인의 Frenet `d(s)`만 수정한 회피 경로(`/avoid_waypoints`)를 만듭니다. 이 launch가
wall-only 레퍼런스 맵 서버와 **obstacle_detector를 기본 포함**(`start_obstacle_detector:=true`)해서 띄웁니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
F1_MAP=ifac_track ros2 launch local_planning local_planning.launch.py
```

### 터미널 5 — 상태 머신 (state machine)

`/car_state/frenet/odom`·`/avoid_waypoints`·`/overtake_waypoints`·`/global_waypoints`를 종합해
주행 상태(GLOBAL/AVOID/OVERTAKE)를 판정하고 `/state`로 발행합니다. 같은 노드가 현재 상태에
맞는 경로를 `/local_waypoints`와 `/local_waypoints/path`로 선택 발행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch state_machine state_machine.launch.py
```

### 터미널 6 — 제어

L1 Guidance + Steering LUT 기반 조향/속도 제어(MAP). 나란히 MPPI 컨트롤러도 항상 구동되며
조이스틱 RB 버튼으로 즉시 전환됩니다. **실차와 시뮬은 launch 파일이 다릅니다** — `control_sim.launch.py`는
`sim_imu_bridge_node`(gym이 IMU를 발행하지 않아 odom→IMU를 중계), `control_real.launch.py`는
`ackermann_to_vesc_node`(최종 `/drive`→VESC 모터/서보 명령 변환)를 각각 갖고 있어 노드 구성 자체가
다르기 때문입니다. 인자 하나로 합치면 환경을 잘못 고를 경우 안전 관련 노드가 조용히 빠진 채
기동될 위험이 있어 의도적으로 분리해뒀습니다.

**시뮬 — `control_sim.launch.py`**

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_control control_sim.launch.py
```

기동 즉시 자율주행합니다. teleop Mux(수동/자율/E-stop)는 이 저장소에 없고(2026-07-29 제거),
`drive_source_selector`가 자율 명령을 `/drive`로 직결합니다.

**실차 — `control_real.launch.py`**

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source ~/f1tenth_ws/install/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_control control_real.launch.py
```

⚠️ `ackermann_to_vesc_node`가 `vesc_ackermann` 패키지(f1tenth_stack 소속, `2026_IFAC`가 아니라
별도 워크스페이스 `~/f1tenth_ws`에 설치됨)에 의존합니다 — `~/f1tenth_ws/install/setup.zsh`를
같이 소싱하지 않으면 `Package 'vesc_ackermann' not found`로 실행이 실패합니다.

전제: **f110 단축어(`f1tenth_stack`)로 라이다·조이스틱·VESC 드라이버가 먼저 떠 있어야 합니다**
(`/scan`, `/joy`, VESC IMU 등). 이 launch는 그 위에서 제어 로직(MAP) +
`drive_source_selector`(MAP/MPPI 선택)만 담당합니다. 수동/자율/E-stop Mux(teleop)와
`ackermann_to_vesc_node`는 f1tenth_stack이 담당합니다.

---

## 3-1. (선택) 상대차 검출·추월 시나리오 — 터미널 7·8

> 아래 두 터미널은 **상대차 검출/추월을 볼 때만** 추가로 띄웁니다. 기본 주행에는 필요 없습니다.
>
> **전제 2가지**
> 1. 터미널 1의 gym 시뮬을 **`num_agent: 2`** (`~/f1sim_C/f1tenth_gym_ros/config/sim.yaml`)로 띄워야 상대차량이 스폰됩니다. 1-agent면 상대차가 아예 없어 RViz에도 안 보이고 검출도 안 됩니다. (sim.yaml 수정 후 gym 브리지를 **재실행**해야 반영됨)
> 2. 이 2-agent 브리지는 **에고·상대 둘 다 `drive`를 발행해야 물리 스텝**을 돕니다. 따라서 **터미널 6(에고 제어)을 그대로 유지**해야 하며, 7·8은 교체가 아니라 **추가**입니다.

### 터미널 7 — 상대차 주행 (opponent simulator)

global 라인을 0.8배속으로 따라가도록 f1sim 상대차량에 `/opp_drive`를 발행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch new_map_con opponent_simulator.launch.py
```

### 터미널 8 — 장애물·상대차 검출기 (obstacle detector)

에고 `/scan`을 레이어드 파이프라인(Layer 1 맵 필터 → Layer 2 정적 추적 → Layer 3 동적 분리)으로
처리해 정적 장애물은 `/static_obs`, 상대차는 `/opp_obs`(전방 최근접 1대)로 발행합니다.
둘 다 `f110_msgs/ObstacleArray`이며 Cartesian `(x,y)`·Frenet `(s,d)`·크기가 채워집니다.
정적 장애물의 장기 기억과 회피 판단은 로컬 플래너가 담당합니다.

> ⚠️ 터미널 4(local_planning launch)가 obstacle_detector를 **기본 포함**해서 이미 띄웁니다.
> 터미널 8을 수동으로 추가하면 검출기가 중복 기동되므로, 터미널 4를
> `start_obstacle_detector:=false`로 띄웠을 때만 아래를 실행하세요.
> (`./sim/open_sim.sh --opp`는 run.sh의 kill_pattern이 중복을 자동 정리합니다.)

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch obstacle_detector obstacle_detector.launch.py simulator:=true
```

RViz에서 `/perception/obstacles/markers`(빨강=동적 상대차, 파랑=정적)를 Add 하면 검출 결과가 보입니다.

---

## 3-2. (선택) 컨트롤 파트 보조 런치 — 실차 전용

> 아래 런치들은 필요 시 별도 터미널에서 추가로 띄웁니다(대체 아님). **대시보드는 각자 노트북에서**(원격 모니터링), **LUT 캘리브레이션은 차량에서 `control_real.launch.py`와 함께** 띄웁니다.

### 대시보드 (원격) — 차량/조이스틱 상태 확인 (선택)

젯슨 연산을 아끼려고 **대시보드는 차가 아니라 각자 노트북에서** 띄웁니다(젯슨은 토픽만 발행,
렌더링은 노트북). 무선에선 DDS 기본 멀티캐스트 디스커버리가 막혀 **Fast DDS Discovery Server**로
붙습니다.

**1) 디스커버리 서버** — 차량(젯슨) 쪽에서 상시 1개만 띄웁니다(이미 떠 있으면 생략):

```bash
fastdds discovery -i 0 -l 10.1.1.3 -p 11811
```

**2) 노트북에서 대시보드** — 각자 실행(젯슨과 같은 네트워크):

```bash
ROS_DISCOVERY_SERVER="10.1.1.3:11811" ros2 launch f1tenth_control dashboard.launch.py mode:=real
```

> 안 뜨면 `ROS_SUPER_CLIENT=true`도 함께 걸어보세요. 유선(피트)에선 멀티캐스트가 되므로 env 없이
> `mode:=real`만으로도 붙습니다. (`mode:=sim`은 시뮬 로컬 뷰어 — 실차엔 빈 화면)

E-Stop on/off, 주행 모드(MANUAL/AUTONOMOUS/ESTOP), 알고리즘(MAP/MPPI), 스로틀·조향 %, 현재
속도·ERPM·종/횡가속도를 실시간 표시하는 표시 전용 뷰어입니다. 안 띄워도 주행에는 영향 없습니다.

### LUT 캘리브레이션 — 실측 조향 LUT 갱신 (트랙 시험 주행 시 필수)

```bash
ros2 launch f1tenth_control lut_calibration.launch.py
```

**실차로 트랙을 시험 주행할 때는 항상 이 런치를 함께 켜두세요.** 실측 요레이트·속도·조향각으로
Steering LUT를 갱신하는 관찰 전용 노드로, `/drive`를 발행하지 않아 주행 제어에는 영향이 없습니다.
결과는 `~/f1tenth_lut_calibration/`에 주행할 때마다 누적 저장되며, 다음 주행에 반영하려면
`control_real.launch.py`에 다음과 같이 지정합니다:

```bash
ros2 launch f1tenth_control control_real.launch.py \
  lookup_table_file:=$HOME/f1tenth_lut_calibration/NUC6_glc_pacejka_lookup_table_calibrated.csv
```

---

## 4. 주의사항

### 4.1 반드시 `~/2026_IFAC`에서 실행할 것

`src/global_planning/config/global_planning.yaml`의 경로가 **상대경로**입니다.

```yaml
# global_trajectory_publisher_node: JSON을 <output_base_dir>/<map_name>/global_waypoints.json 에서 읽음
output_base_dir: "offline_trajectory_generator/output"
map_name: "map"          # 실차 기본값. F1_MAP 환경변수(launch 인자)가 설정돼 있으면 이를 우선함
```

따라서 워크스페이스 루트가 아닌 곳에서 터미널 3을 실행하면 파일을 찾지 못합니다.

### 4.2 `global_waypoints.json`이 먼저 있어야 함

`global_trajectory_publisher_node`는 시작할 때 JSON을 **딱 한 번만** 읽습니다. 파일이 없거나 파싱에 실패하면
`/global_waypoints`를 아예 발행하지 않고, 그 뒤의 로컬 플래너와 제어가 전부 대기 상태로 멈춥니다.

읽는 위치는 `<output_base_dir>/<map_name>/global_waypoints.json`(기본 `offline_trajectory_generator/output/<F1_MAP>/`)입니다.
**맵을 바꾸려면** `F1_MAP` 환경변수(또는 yaml의 `map_name`)만 그 맵 이름(= `offline_trajectory_generator/output/` 하위 폴더명)으로 바꾸면 됩니다.
새 맵을 생성했거나 파일을 교체했다면 터미널 3을 재시작해야 반영됩니다.

`output/`은 gitignore 대상이라 클린 체크아웃에는 없습니다. 아래처럼 생성하세요 (`torch` 미설치 환경에서는
`--optimizer mincurv`만 가능):

```bash
cd ~/2026_IFAC
python3 offline_trajectory_generator/generate_global_trajectory.py \
  --map-yaml src/monte_carlo_localization/maps/ifac_track.yaml \
  --output-dir offline_trajectory_generator/output/ifac_track \
  --optimizer mincurv --raceline-smooth-sigma 3.0 \
  --max-speed 0.5 --min-speed 0.4 --max-lateral-accel 1.0
```

> 속도 상한을 `0.5 m/s`로 두는 이유: gym 시뮬 폐루프 검증 기준값입니다. 합성 복도 맵에서는
> 코너 요레이트가 커지면(≈0.6 rad/s 이상) MCL 파티클이 복도 대칭성 때문에 뒤집힐 수 있고,
> 트랙 코너 반경(~1 m)이 작아 고속에서는 L1 추종이 코너를 잘라먹습니다. 실차/실맵에서는
> 기존 튜닝 값을 그대로 쓰면 됩니다.

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
     ├──► /local_planning/path
     └──► /avoid_waypoints
                    │
                    ▼
[터미널 5] state_machine
     /car_state/frenet/odom·/global_waypoints·/avoid_waypoints·/overtake_waypoints 구독
     ├──► /state (GLOBAL / AVOID / OVERTAKE)
     └──► /local_waypoints, /local_waypoints/path
                    │
                    ▼
[터미널 6] f1tenth_control
     /global_waypoints, /local_waypoints 구독 (실차: f110 스택이 /scan, /joy, VESC IMU 별도 공급)
     ├─ control_map_node  (MAP: L1 Guidance+Steering LUT)  ──► /drive_autonomous ─┐
     └─ control_mppi_node (MPPI: 샘플링 기반, 나란히 상시구동) ──► /drive_mppi ───┤
                                                                                   ▼
                                             drive_source_selector (RB로 MAP/MPPI 선택)
                                                                                   │
                                                                                   ▼
                                                                                /drive
                                                (시뮬: gym_bridge가 직접 구독 / 실차: ackermann_to_vesc_node → VESC)
```

---

## 6. ROS 2 Jazzy 포트 검증

이 브랜치(`jazzy_main`)는 `backup/jetson-20260725`의 Humble 기반 스택을
**ROS 2 Jazzy (Ubuntu 24.04)** 로 포팅하고 실제 빌드·주행으로 검증한 결과에,
2026-07-29의 obstacle_detector 교체·teleop 제거·MCL 개선(아래 참고)을 반영한 것입니다.
(검증일: 2026-07-29, x86_64 데스크톱 / 포트 커밋은 `jazzy-port-test`·`jazzy_port_main`과 공유)

### 6.1 포팅 변경 사항

| 분류 | 내용 |
|---|---|
| tf2 헤더 | deprecated `.h` → `.hpp` 5개 파일 (particle_filter, opponent_simulator, opponent_detector 등) — Kilted 대비 겸용 |
| CMake | 전 패키지 `ament_target_dependencies` → modern `target_link_libraries` (9개 패키지 27곳) |
| rosdep 키 | `nlohmann_json`→`nlohmann-json-dev`, 가짜 `openmp` 제거 (global_planning), 기존 waypoint selector의 `nav_mags` 오타 수정, 미선언 `angles` 제거 (MCL), 미사용 `pcl_ros`/`pcl_conversions` 제거 (MCL) |
| lap_timer | `rviz_2d_overlay_msgs` 무가드 import 제거 — Jazzy에 바이너리 미배포라 실행 즉시 죽던 것을 선택적 import로 전환 |
| vendor | `gb_optimizer`(ROS1 catkin)에 `COLCON_IGNORE` 명시 |
| MCL sim 튜닝 | sim 모드 한정 motion_dispersion/smoothing 오버라이드 (`mcl_launch.py`, 실차 값 불변) |
| 데이터 복구 | git에서 유실됐던 `ifac_track.{png,yaml}` 맵 복구 |
| 포맷 | `builtin_interfaces/Time` 로그 포맷 `-Wformat` 경고 수정 |
| backup 머지 포트 (07-29) | MCL lib에 `${f110_msgs_TARGETS}` 링크 추가(backup이 MCL에 WpntArray 도입), 신규 문서·스크립트의 humble 표기 → jazzy, `sim/run.sh`가 `F1_MAP`을 export해 글로벌/로컬 플래너 맵 통일 |
| 깨진 패키지 복구 | backup 팁은 **커밋된 미해결 충돌 마커**(`f971351` 머지 잔재)와 유실 파일 때문에 4개 패키지가 컴파일 불가였음 — ① local_planning: `raceline_spline_planner.cpp` 구현 유실+마커 ② state_machine: hpp/cpp 선언 불일치 ③ opponent_detector: cpp·yaml 등 6개 파일에 마커 ④ f110_msgs: `Obstacle.msg`의 Cartesian 확장 유실. 전부 main 복구 커밋 `b47c785`와 동일 내용(Jazzy 포트 포함, `jazzy-port-test` 검증본)으로 교체. f1tenth_control의 `WORKLOG.md`·`vesc_mcconf.xml` 마커는 젯슨 최신(HEAD쪽)으로 해소 |
| 검출기 교체 포트 (07-29) | main 최신(`0dea9d2`)의 **obstacle_detector**(레이어드 검출, `/static_obs`·`/opp_obs`) + 재구성 **local_planning**(safe_corridor 제거, 노드 결합)으로 교체 — `opponent_detector` 패키지 삭제. 두 패키지 CMake를 modern `target_link_libraries`로 전환, obstacle_detector tf2 헤더 `.h`→`.hpp` 4곳, local_planning package.xml에 launch 실사용 exec_depend(nav2_map_server·nav2_lifecycle_manager·particle_filter_cpp) 보강, 문서·sim 스크립트의 opponent_detector 참조 일괄 갱신 |

### 6.2 검증 결과

- **빌드**: `build/ install/ log/` 전부 삭제 후 클린 빌드 — **12개 패키지 전부 성공, 경고 0건** (2분 18초, Ninja)
- **런타임**: gym 시뮬레이터(f1sim_C, Jazzy 네이티브) + 7노드 풀스택으로 **자율주행 3랩 연속 완주**
  - 랩타임 ~102 s (지시 속도 0.5 m/s 정속)
  - MCL 위치추정 오차: 최대 0.18 m, 평상시 1~6 cm (320 s 연속 추적)
  - 레이스라인 횡편차 |d| ≤ 0.36 m, 전 코너 통과
  - 전 토픽 발행 확인: `/scan` 49 Hz, `/pf/pose/odom` 30 Hz, `/car_state/frenet/odom` 21 Hz, `/local_waypoints` 21 Hz, `/drive` 38 Hz 등
- **backup 머지 재검증 (2026-07-29, 이 브랜치)**: `backup/jetson-20260725`(666e6b1)에 Jazzy
  포트를 머지하고 컴파일 불가 패키지(local_planning·state_machine·opponent_detector·
  f110_msgs — 아래 §6.1 "local_planning 복구" 참고)를 복구한 뒤 동일 절차로 재검증 —
  클린 빌드 12/12 성공(포트 코드 경고 0건, 벤더 CLCS·LTO 잡음 제외), 폐루프 **3랩 연속
  완주**, MCL 오차 평균 0.032 m / 최대 0.183 m, 레이스라인 |d| 중앙값 0.014 m(순간 스파이크
  1회 0.48 m 제외 시 ≤ 0.09 m), 전 7노드 에러 로그 0건. (터미널 4가 opponent_detector·
  레퍼런스 맵 서버를 기본 포함하는 구조 — §3 터미널 4·9 참고)
- **검출기 교체 재검증 (2026-07-29, `jazzy_port_main`에서 수행 — 이 브랜치에 반영)**:
  main 최신(`0dea9d2`)의 obstacle_detector + 재구성 local_planning으로 교체한 뒤 재검증 —
  클린 빌드 12/12 경고 0건(2분 6초), `colcon test`(local_planning gtest+lint 9/9) 통과,
  합성 장애물 파이프라인 테스트(`static_obs_pipeline_test.py`, `/scan`→`/static_obs`→
  `/avoid_waypoints`) PASS, gym 폐루프 3랩 연속 완주(0.5 m/s), MCL 오차 평균 0.014 m /
  최대 0.130 m, 검출기 스캔 드롭 0·오탐 0, `/drive` 39 Hz·`/local_waypoints` 30 Hz,
  전 노드 에러 로그 0건. (gym 워크스페이스가 `~/f1sim_C` → `~/sim_ws`로 이동됨)
- **속도 한계 스윕 (2026-07-29)**: ifac_track 웨이포인트를 `--max-speed`만 바꿔 재생성
  (라인 기하 동일, a_lat 1.0)하고 각 190 s(6~12랩) 폐루프로 MCL 추적을 비교 —
  | max_speed | 랩 | GT 대비 오차 max | 판정 |
  |---|---|---|---|
  | 3.0 | 6 | 0.28 m | ✅ 클린 |
  | **4.0** | 7 | **0.23 m** | ✅ **검증 상한** |
  | 4.5 | 7 | 1.41 m | ⚠️ 복도 스파이크 |
  | 5.0 | 12 | 1.71 m | ⚠️ 복도 스파이크(완주는 함) |
  4.5부터 y≈0.1 장복도 직선에서 **종방향(along-corridor) 오차가 랩마다 0.4~1.7 m 스파이크**
  (평행벽 기하라 진행방향 관측성이 약함 — 코너 진입에서 재고정). 코너는 a_lat 1.0 기준
  0.4~1 m/s라 요레이트 플립은 발생하지 않았다. 구 0.5 프로파일은
  `output/ifac_track_maxspeed0.5_backup/`에 보관.
- **MCL pose fusion EKF (2026-07-29)**: 위 복도 한계를 출력단 EKF로 해소 — 휠 odom 포즈
  델타(laser 프레임 변환 포함)로 30 Hz 예측 + MCL 기대 포즈 보정, R = 파티클 공분산에
  **차체 종방향만 25배 불신**(복도에서 파티클이 틀린 곳에 좁게 수렴해 공분산이 모호성을
  과소평가하므로). 마할라노비스 게이트 + 연속 기각 시 재고정 안전망 포함. 재스윕 결과 —
  | max_speed | 실측 vmax | GT 오차 p50 / 정상상태 max | 판정 |
  |---|---|---|---|
  | 5.0 | 5.00 | 1.2 cm / 8 cm | ✅ (EMA 시절 1.7 m 스파이크 → 소멸) |
  | 6.0 | 5.92 | 1.2 cm / 10 cm | ✅ |
  | 7.0 | **6.21** | 1.1 cm / 10.5 cm | ✅ — **한계는 이제 MCL이 아니라 max_accel 3.0·직선 길이** |
  시뮬 기본 프로파일은 max_speed 7.0(실측 vmax 6.2, 랩 ~27 s). 더 올리려면 웨이포인트
  `--max-accel`/`--max-lateral-accel` 상향이 필요하며 이는 제어 안전 검증과 한 세트.
  구현: `use_pose_ekf`(기본 true, false면 구 EMA), 파라미터는 `mcl_config.yaml` 참고.
  ⚠️ 실차 전제: VESC 휠 odom 병진 오차 ~0.3%(`ekf_trans_error_rate` 유효값 1%로 설정),
  odom yaw는 조향 명령 합성이므로 실차 셰이크다운에서 `ekf_rot_*` 재튜닝 여지 있음.

### 6.3 시뮬 검증에서 확인된 주의사항

1. **초기 위치는 반드시 지정** — 헤드리스에서는 `/initialpose` 직접 발행 (§3 터미널 2 참고).
   전역 초기화만으로는 반복 복도 형상 때문에 엉뚱한 위치에 수렴할 수 있습니다.
2. **gym은 벽 충돌을 물리적으로 막지 않습니다** — 제어가 이탈하면 차가 벽을 통과해 맵 밖으로 나가고,
   그 순간부터 스캔이 맵과 매칭되지 않아 MCL이 복구 불가능하게 발산합니다.
3. **시뮬 맵·MCL 맵·웨이포인트 맵 3곳 일치**는 절대 조건입니다 (sim.yaml `map_path` 확인).
4. gym 브리지는 `/clock`을 발행하지 않습니다 — `use_sim_time`이 sim 모드에서 true로 설정되지만
   타이머가 전부 wall clock 기반이라 동작에는 지장이 없음을 확인했습니다.
