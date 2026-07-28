# CLAUDE.md

This file defines the working rules for Claude in this repository. These rules apply to the entire repository unless a more specific agent instruction file exists in a subdirectory.

## Communication

- All user-facing replies must be written in Korean unless the user explicitly requests another language.
- Keep repository guidance in English when practical to reduce token usage and make instructions compact.

## Target Environment

- Target platform: ROS 2 Jazzy.
- New ROS 2 runtime code must be written in C++.
- Use Python only where ROS 2 conventionally requires it, such as `launch.py` files or build/config helper scripts.
- Actively check and use relevant aliases from `~/.zshrc` when running build, test, launch, or debugging commands.

## Message Policy

- Prefer existing `f110_msgs` message types for inter-node communication.
- Prefer ROS 2 standard `std_msgs` types when a standard type is sufficient.
- Before creating a new message type, first verify that the requirement cannot be cleanly represented with `f110_msgs` or `std_msgs`.

## Parameter Policy

- Check every hard-coded value before adding it to source code.
- Move configurable values into YAML parameter files whenever practical.
- Configurable values include topic names, frame names, loop rates, thresholds, gains, file paths, modes, algorithm tuning values, and feature toggles.
- Nodes that require parameters must provide a matching YAML parameter file.
- Declare parameters and safe defaults clearly in the C++ node, while keeping operational values adjustable through YAML.

## Launch Policy

- Any node that requires parameters must provide a `launch.py` file.
- The `launch.py` file must load the matching YAML parameter file.
- Ensure `CMakeLists.txt` installs launch files and parameter files so the node can run with:
  `ros2 launch <package> <launch_file>.py`

## Node-Level Instruction Policy

- Whenever creating a new node, create a node-level or package-level `AGENTS.md` in the most relevant directory for that node.
- Whenever modifying an existing node, check for the nearest applicable `AGENTS.md`. If it is missing, create one. If it exists but is outdated, update it.
- Node-level `AGENTS.md` files must describe the node-specific rules, package layout, message choices, parameter files, launch files, and documentation expectations.
- Node-level instructions may add constraints, but must not weaken or contradict the root repository instructions.
- Always follow the closest applicable node-level instructions before editing a node.

## Node Documentation

- When adding a new node or significantly changing an existing node, write or update Markdown documentation inside the relevant package.
- The document must include:
  - Node purpose
  - Operating principle
  - Subscribed topics and published topics
  - Message types
  - Main parameters and YAML file location
  - How to run the node
  - `ros2 launch` example
- Use a clear filename that includes the node name, such as `docs/<node_name>.md`, unless the package already has a better local convention. Make sure to write documents step-by-step using Korean.

## Implementation Checklist

Before finishing any ROS 2 node change, verify:

- Runtime code is implemented in C++.
- `f110_msgs` or `std_msgs` were preferred for message usage.
- Configurable values were moved to YAML where practical.
- Nodes requiring parameters include a `launch.py` file.
- `CMakeLists.txt` installs binaries, launch files, parameter files, and documentation as needed.
- The nearest node-level or package-level `AGENTS.md` exists and is current.
- Node Markdown documentation explains operation and execution.
- The implemented code was built and run after completion.
- Runtime behavior was checked, and any launch/build/runtime issues were debugged before reporting completion.

## Simulation Run Order (실행 순서)

기본 주행은 터미널 7개를 아래 순서대로 띄웁니다. 순서가 중요합니다.
상대차 검출·추월까지 보려면 터미널 8·9를 **추가로** 띄웁니다(터미널 7은 유지).

> **원클릭 실행**: Terminator가 설치되어 있으면 `./sim/open_sim.sh`(터미널 1~7 분할) 또는
> `./sim/open_sim.sh --opp`(터미널 1~9 분할) 한 번으로 전체 스택을 띄울 수 있습니다.
> 자세한 내용은 `sim/README.md` 참고.

### 터미널 1 — 시뮬레이터 (gym bridge)

`~/f1sim_C/f1tenth_gym_ros/config/sim.yaml`의 `map_path`가 스택과 같은 맵(확장자 없는 절대경로,
예: `.../2026_IFAC/src/monte_carlo_localization/maps/ifac_track`)인지 먼저 확인합니다.

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

기동 후 RViz **2D Pose Estimate**로 초기 위치를 반드시 지정합니다. 헤드리스로 돌릴 때는 대신
`/initialpose`를 직접 발행합니다 (gym 브리지 텔레포트 + MCL 초기화 동시 수행, README §3 참고):

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: -0.427, y: 0.456}, orientation: {z: 0.3651, w: 0.9310}}}}'
```

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

`/perception/static_obstacles/cartesian`의 정적 장애물을 CLCS로 투영해 글로벌 라인의 Frenet
`d(s)`만 수정한 회피 경로(`/avoid_waypoints`)를 만듭니다. 이 launch가 wall-only 레퍼런스 맵
서버와 **opponent_detector를 기본 포함**(`start_opponent_detector:=true`)해서 띄웁니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
F1_MAP=ifac_track ros2 launch local_planning local_planning.launch.py
```

### 터미널 5 — 상태 머신 (state machine)

`/car_state/frenet/odom`·`/avoid_waypoints`·`/overtake_waypoints`·`/global_waypoints`를 종합해 주행 상태(GLOBAL/AVOID/OVERTAKE)를 판정하고 `/state`로 발행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch state_machine state_machine.launch.py
```

### 터미널 6 — 웨이포인트 퍼블리셔

회피 웨이포인트를 받아 `/local_waypoints`로 중계합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 run wpnt_publisher wpnt_publisher
```

### 터미널 7 — 제어

L1 Guidance + Steering LUT 기반 조향/속도 제어. `force_autonomous:=true`면 조이스틱 없이 즉시 자율주행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_control control_sim.launch.py force_autonomous:=true
```

---

### (선택) 상대차 검출·추월 시나리오 — 터미널 8·9

> 아래 두 터미널은 **상대차 검출/추월을 볼 때만** 추가로 띄웁니다. 기본 주행에는 필요 없습니다.
>
> **전제 2가지**
> 1. 터미널 1의 gym 시뮬을 **`num_agent: 2`** (`~/f1sim_C/f1tenth_gym_ros/config/sim.yaml`)로 띄워야 상대차량이 스폰됩니다. 1-agent면 상대차가 아예 없어 RViz에도 안 보이고 검출도 안 됩니다. (sim.yaml 수정 후 gym 브리지를 **재실행**해야 반영됨)
> 2. 이 2-agent 브리지는 **에고·상대 둘 다 `drive`를 발행해야 물리 스텝**을 돕니다. 따라서 **터미널 7(에고 제어)을 그대로 유지**해야 하며, 8·9는 교체가 아니라 **추가**입니다.

### 터미널 8 — 상대차 주행 (opponent simulator)

global 라인을 0.8배속으로 따라가도록 f1sim 상대차량에 `/opp_drive`를 발행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch new_map_con opponent_simulator.launch.py
```

### 터미널 9 — 상대차 검출기 (opponent detector)

에고 `/scan`으로 상대차와 정적 장애물을 검출해 `/perception/obstacles`,
`/perception/static_obstacles/cartesian`, `/proj_opponent_trajectory`를 발행합니다. 로컬
플래너는 정적 장애물의 Cartesian `(x,y)`와 최대 반지름을 받아 CLCS로 트랙 위상을 보존해 투영한 뒤
Cartesian `x_m/y_m` 회피 경로를 생성합니다.

> ⚠️ 터미널 4(local_planning launch)가 opponent_detector를 **기본 포함**해서 이미 띄웁니다.
> 터미널 9를 수동으로 추가하면 검출기가 중복 기동되므로, 터미널 4를
> `start_opponent_detector:=false`로 띄웠을 때만 아래를 실행하세요.
> (`./sim/open_sim.sh --opp`는 run.sh의 kill_pattern이 중복을 자동 정리합니다.)

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch opponent_detector opponent_detector.launch.py simulator:=true
```

RViz에서 `/perception/obstacles/markers`(빨강=동적 상대차, 파랑=정적)를 Add 하면 검출 결과가 보입니다.
