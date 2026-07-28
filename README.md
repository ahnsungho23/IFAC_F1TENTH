# 2026 IFAC F1TENTH

F1TENTH 자율주행 스택입니다. 위치추정(MCL), 글로벌 플래닝, 로컬 플래닝(장애물 회피), 제어를 포함합니다.

시뮬레이터는 별도 워크스페이스(`~/sim_ws`)의 `f1tenth_gym_ros`를 사용합니다.


slam launch방법 로컬에서
cd ~/slam_toolbox
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

slam 저장방법 로컬에서
cd ~/slam_toolbox
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 run nav2_map_server map_saver_cli \
    -f /home/haejun/slam_toolbox/map
    
딴 맵을 로컬에서 젯슨으로 맵 전송
scp ~/slam_toolbox/map.png ~/slam_toolbox/map.yaml \
    miru@10.1.1.3:~/2026_IFAC/src/monte_carlo_localization/maps/
    
    
offline gui 파일을 로컬에서 젯슨으로 전송
scp -r ~/2026_IFAC/offline_trajectory_generator/output/map \
    miru@10.1.1.3:~/2026_IFAC/offline_trajectory_generator/output/

터미널1
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch particle_filter_cpp mcl_launch.py mod:=real map_name:=map

터미널2
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch global_planning global_planning.launch.py

터미널3
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch local_planning local_planning.launch.py

터미널4
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch state_machine state_machine.launch.py

터미널5
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch state_machine state_machine.launch.py

터미널6
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 run wpnt_publisher wpnt_publisher

터미널7
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source ~/f1tenth_ws/install/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_control control_real.launch.py



rosbag
cd ~/miru/2026_IFAC

source /opt/ros/humble/setup.zsh
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
│   ├── f1tenth_control/          # 차량 제어: MAP(L1 Guidance+Steering LUT)/MPPI 이중 컨트롤러, joy Mux, LUT 실측 보정
│   │   ├── control_code/         # control_map_node, control_mppi_node(+solver_cpu/gpu), joy_teleop_monitor(Mux), teleop_dashboard_node, lut_calibrator_node, sim_imu_bridge_node, gap_follower, imu_stability_controller, steer lookup(Python/CSV)
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

L1 Guidance + Steering LUT 기반 조향/속도 제어(MAP). 나란히 MPPI 컨트롤러도 항상 구동되며
조이스틱 RB 버튼으로 즉시 전환됩니다. **실차와 시뮬은 launch 파일이 다릅니다** — `control_sim.launch.py`는
`sim_imu_bridge_node`(gym이 IMU를 발행하지 않아 odom→IMU를 중계), `control_real.launch.py`는
`ackermann_to_vesc_node`(최종 `/drive`→VESC 모터/서보 명령 변환)를 각각 갖고 있어 노드 구성 자체가
다르기 때문입니다. 인자 하나로 합치면 환경을 잘못 고를 경우 안전 관련 노드가 조용히 빠진 채
기동될 위험이 있어 의도적으로 분리해뒀습니다.

**시뮬 — `control_sim.launch.py`**

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_control control_sim.launch.py force_autonomous:=true
```

`force_autonomous:=true`면 조이스틱 없이 즉시 자율주행합니다. 생략하면 MANUAL로 시작하며
조이스틱 LB 버튼으로 AUTONOMOUS 전환이 필요합니다.

**실차 — `control_real.launch.py`**

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source ~/f1tenth_ws/install/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_control control_real.launch.py
```

⚠️ `ackermann_to_vesc_node`가 `vesc_ackermann` 패키지(f1tenth_stack 소속, `2026_IFAC`가 아니라
별도 워크스페이스 `~/f1tenth_ws`에 설치됨)에 의존합니다 — `~/f1tenth_ws/install/setup.zsh`를
같이 소싱하지 않으면 `Package 'vesc_ackermann' not found`로 실행이 실패합니다.

전제: **f110 단축어(`f1tenth_stack`)로 라이다·조이스틱·VESC 드라이버가 먼저 떠 있어야 합니다**
(`/scan`, `/joy`, VESC IMU 등). 이 launch는 그 위에서 제어 로직(MAP+MPPI) + Mux
(`joy_teleop_monitor`) + `ackermann_to_vesc_node`만 담당하며, 자체적으로 조이스틱을 기동하지
않습니다. 기본 시작 모드는 MANUAL(조이스틱 LB로 AUTONOMOUS 전환).

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

에고 `/scan`으로 장애물·상대차를 검출해 `/perception/obstacles`,
`/perception/static_obstacles/cartesian`, `/proj_opponent_trajectory`를 발행합니다.
정적 장애물은 Cartesian `(x,y)`, Frenet `(s,d)`, 최대 반지름 `radius`로 로컬 플래너에 전달합니다. IMU/odom 기반 스캔
deskew와 불확실성 기반 추적을 사용하며, 정적 장애물의 장기 기억과
회피 판단은 로컬 플래너가 담당합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch opponent_detector opponent_detector.launch.py simulator:=true
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
     /global_waypoints, /local_waypoints 구독 (실차: f110 스택이 /scan, /joy, VESC IMU 별도 공급)
     ├─ control_map_node  (MAP: L1 Guidance+Steering LUT)  ──► /drive_autonomous ─┐
     └─ control_mppi_node (MPPI: 샘플링 기반, 나란히 상시구동) ──► /drive_mppi ───┤
                                                                                   ▼
                                             joy_teleop_monitor (Mux — RB로 MAP/MPPI 선택)
                                                                                   │
                                                                                   ▼
                                                                                /drive
                                                (시뮬: gym_bridge가 직접 구독 / 실차: ackermann_to_vesc_node → VESC)
```
