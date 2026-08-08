# CLAUDE.md

This file defines the working rules for Claude in this repository. These rules apply to the entire repository unless a more specific agent instruction file exists in a subdirectory.

## Communication

- All user-facing replies must be written in Korean unless the user explicitly requests another language.
- Keep repository guidance in English when practical to reduce token usage and make instructions compact.

## Target Environment

- Target platform: Ubuntu 24.04 with ROS 2 Jazzy.
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

기본 주행은 터미널 8개를 아래 순서대로 띄웁니다. 순서가 중요합니다.

### 터미널 1 — 시뮬레이터 (gym bridge)

```bash
cd ~/sim_ws
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

### 터미널 3 — 글로벌 플래너

`global_waypoints.json`을 읽어 `/global_waypoints`를 발행하고, `/car_state/frenet/odom`을 계산합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch global_planning global_planning.launch.py
```

### 터미널 4 — 장애물 검출기

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch obstacle_detector obstacle_detector.launch.py simulator:=true use_sim_time:=true
```

### 터미널 5 — 로컬 플래너 (장애물 회피)

`/map`의 점유 격자에서 장애물을 찾아 최소자승 3차 스플라인 회피 경로를 만듭니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch local_planning local_planning.launch.py
```

### 터미널 6 — 상태 머신 (state machine)

`/car_state/frenet/odom`·`/avoid_waypoints`·`/overtake_waypoints`·`/global_waypoints`를 종합해 주행 상태(GLOBAL/AVOID/OVERTAKE)를 판정하고 `/state`로 발행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch state_machine state_machine.launch.py
```

### 터미널 7 — 웨이포인트 퍼블리셔

회피 웨이포인트를 받아 `/local_waypoints`로 중계합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch wpnt_publisher wpnt_publisher.launch.py
```

### 터미널 8 — 제어

L1 Guidance + Steering LUT 기반 조향/속도 제어. `force_autonomous:=true`면 조이스틱 없이 즉시 자율주행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_control control_sim.launch.py force_autonomous:=true
```

---

### (선택) `new_map_con` 단독 컨트롤러

글로벌 플래너와 기본 제어기를 종료한 뒤 CSV waypoint 발행과 pure-pursuit 제어를 한 노드로
시험할 때 사용합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch new_map_con new_map_con.launch.py simulator:=true
```
