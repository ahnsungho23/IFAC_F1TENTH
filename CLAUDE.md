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

기본 주행은 터미널 7개를 아래 순서대로 띄웁니다. 순서가 중요합니다.
(구 터미널 4=검출기 단독·구 터미널 7=wpnt_publisher는 폐지 — 아래 각 절 참고)

### 터미널 1 — 시뮬레이터 (gym bridge)

```bash
cd ~/f1sim_C
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

🔴 **시뮬 맵은 이 저장소 밖에 있습니다** — `~/f1sim_C/f1tenth_gym_ros/config/sim.yaml`의
`map_path`가 가리키는 파일로 gym이 **스캔을 만듭니다**. 이 맵이 MCL 맵과 다르면 `/scan`이
`/map`과 전혀 안 맞고, RViz에서 스캔이 트랙에서 멀리 떨어져 보입니다 (2026-08-18 실측).

트랙이 바뀌면 **세 곳을 같이** 바꿔야 합니다:

| 무엇 | 어디 |
|---|---|
| gym 스캔 생성용 맵 | `~/f1sim_C/f1tenth_gym_ros/maps/` + `config/sim.yaml`의 `map_path` |
| MCL 맵 | `src/monte_carlo_localization/maps/map.{png,yaml}` |
| raceline | `offline_trajectory_generator/output/map/` |

🔴 **스폰 포즈는 raceline의 `wpnts[0]`과 정확히 같아야 합니다.** MCL은
`auto_init_from_waypoints: true`로 **`wpnts[0]`에 파티클을 뿌립니다**(`particle_filter.cpp`
의 `start_wp = msg->wpnts[0]`). gym 스폰이 그와 다르면 MCL이 그 차이만큼 어긋난 채
시작하고 수렴하지 못합니다 — 2026-08-18에 스폰을 s=5.01로 두었더니 정확히 4.84 m
(= wpnts[0]까지 거리)만큼 틀어졌습니다.

현재값은 `sx: -0.706042, sy: 15.188966, stheta: -2.469165` = 새 라인 `wpnts[0]`입니다.
라인을 다시 만들면 이 세 값도 새 `wpnts[0]`으로 같이 바꾸십시오.

### 터미널 2 — 위치추정 (Monte Carlo Localization)

`/pf/pose/odom`을 발행합니다. 이후 모든 노드가 이 토픽에 의존합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch particle_filter_cpp mcl_launch.py mod:=sim map_name:=map use_rviz:=true
```

| 인자 | 값 | 설명 |
|---|---|---|
| `mod` | `sim` | 시뮬레이션 모드 (`/ego_racecar/odom` 사용, sim time 활성) |
| `map_name` | `map` | `monte_carlo_localization/maps/map.yaml` (기본값이라 생략 가능) |
| `use_rviz` | `true` | RViz 동시 실행 |

### 터미널 3 — 글로벌 플래너

`global_waypoints.json`을 읽어 `/global_waypoints`를 발행하고, `/car_state/frenet/odom`을 계산합니다.

⚠️ **맵 이름은 이제 `map` 하나로 통일됐습니다** (2026-08-18). MCL·global·local 세
노드의 기본값이 모두 `map`이고, 환경변수 `F1_MAP`으로 한 번에 바꿉니다. 옛 `ifac_track`
맵은 삭제됐으니 그 이름을 넘기면 파일을 못 찾습니다. **젯슨 `~/.zshrc`에 `F1_MAP=ifac_track`이
남아 있지 않은지 확인하십시오** — 남아 있으면 세 노드가 다시 어긋납니다.

launch는 `global_waypoints.json`을 **cwd 상대경로**로 읽으므로 `cd ~/2026_IFAC` 상태에서
실행해야 합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch global_planning global_planning.launch.py
```

### 터미널 4 — 장애물 검출기 (⚠️ 기본 생략)

**터미널 5의 `local_planning.launch.py`가 기본값(`start_obstacle_detector:=true`)으로
검출기를 함께 실행하므로, 이 터미널을 따로 띄우면 검출기가 2중 실행됩니다.** 두 인스턴스가
`/static_obs`에 서로 다른 트랙 ID·stamp를 교차 발행해 로컬 플래너의 커밋 경로가 계속
무효화됩니다 (중복 감지 시 검출기가 ERROR 로그를 출력함). 검출기를 단독으로 띄우고 싶으면
터미널 5에서 `start_obstacle_detector:=false`를 함께 넘기십시오.

```bash
# 단독 실행이 꼭 필요할 때만 (터미널 5에 start_obstacle_detector:=false 필요):
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch obstacle_detector obstacle_detector.launch.py simulator:=true use_sim_time:=true
```

### 터미널 5 — 로컬 플래너 (장애물 회피)

`/map`의 점유 격자에서 장애물을 찾아 최소자승 3차 스플라인 회피 경로를 만듭니다.

🔴 **`simulator:=true`를 반드시 붙이십시오** (2026-08-20). 런치 기본값이 실차(`false`)로
바뀌었습니다. 빠뜨리면 검출기가 `/pf/pose/odom`을 구독해 시뮬에서 ego odom을 못 받습니다.
반대로 **실차에서는 인자 없이** 띄우면 됩니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch local_planning local_planning.launch.py simulator:=true
```

### 터미널 6 — 상태 머신 (state machine)

`/car_state/frenet/odom`·`/avoid_waypoints`·`/overtake_waypoints`·`/global_waypoints`를 종합해 주행 상태(GLOBAL/AVOID/OVERTAKE)를 판정하고 `/state`로 발행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch state_machine state_machine.launch.py
```

### ~~터미널 7 — 웨이포인트 퍼블리셔~~ (폐지)

**wpnt_publisher는 state_machine에 통합되어 소스가 삭제된 유령 패키지입니다**
(`/local_waypoints`는 터미널 6의 state_machine_node가 직접 발행 — 2026-08-13
터미널 7 없이 8랩 완주로 검증). `install/`에 남은 옛 빌드 잔재로만 실행 가능했으며,
클린 빌드 후에는 어차피 실행되지 않습니다. **띄우지 마십시오.**

### 터미널 7 — 제어 (구 터미널 8)

L1 Guidance + Steering LUT 기반 조향/속도 제어. 기동 즉시 자율주행합니다
(구 `force_autonomous` 인자는 2026-07-29 폐지 — 넘겨도 무시됨).

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch f1tenth_control control_sim.launch.py
```

---

### ~~(선택) `new_map_con` 단독 컨트롤러~~ (폐지)

**`new_map_con`은 삭제된 패키지입니다** (`3f50623 remove: delete obsolete new_map_con
package`). `wpnt_publisher`와 같은 상태입니다 — 소스는 없고 `install/`의 옛 빌드 잔재로만
실행됩니다. 클린 빌드 후에는 실행되지 않습니다. **띄우지 마십시오.**

### 삭제된 패키지의 빌드 잔재 (정리 필요)

의도적으로 삭제됐지만 `install/`에 남아 **아직 `ros2 launch`로 뜨는** 패키지가 있습니다.
소스가 없으므로 옛 코드가 조용히 돌게 되어 위험합니다. 워크스페이스를 정리하려면:

```bash
cd ~/2026_IFAC
rm -rf src/new_map_con install/new_map_con install/wpnt_publisher \
       build/new_map_con build/wpnt_publisher
```

`src/new_map_con/`에는 `launch/__pycache__/*.pyc`만 남아 있습니다(소스 없음).
