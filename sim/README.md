# sim/ — 시뮬레이션 원클릭 실행 (Terminator)

루트 `CLAUDE.md`의 "Simulation Run Order"(터미널 7개 + 선택 2개)를 매번 손으로 띄우는 대신,
Terminator 한 창에 분할 화면으로 한 번에 띄우는 스크립트 모음입니다.
`parkm_combine` 브랜치의 `sim/` 런처 구조를 이 브랜치의 노드 구성
(local_planning + state_machine + f1tenth_control + obstacle_detector)에 맞게 옮긴 것입니다.

## 빠른 시작

```bash
# 기본 주행 (터미널 1~7, 7분할)
~/2026_IFAC/sim/open_sim.sh

# 상대차 검출 시나리오 (터미널 1~8, 8분할)
~/2026_IFAC/sim/open_sim.sh --opp
```

`open_sim.sh --opp`는 검출기만 추가하므로 별도 `/opp_drive` 발행자가 필요합니다. 저장소의
`opponent_simulator`를 함께 실행하려면 아래 전용 크루즈 실행기를 사용합니다.

## 2대 크루즈 시나리오

`launch_cruise.zsh`는 에고 차량의 전체 IFAC 스택과 별도로 `opponent_simulator`를 실행한다.
상대차는 `/centerline_waypoints`를 Pure Pursuit로 추종하며 `/opp_drive`만 발행하므로 에고
`/drive`와 충돌하지 않는다.

```bash
# 기본 맵
~/Desktop/launch_cruise.zsh

# 상대차 속도 배율 변경
~/Desktop/launch_cruise.zsh --speed 0.7

# 장애물 GUI 사용
~/Desktop/launch_cruise.zsh --gui
```

`open_sim.sh --opp`는 검출기만 추가하는 일반 레이아웃이고, 상대차 자동 주행까지 포함하려면
위의 전용 크루즈 실행기를 사용한다.

Terminator가 없으면 먼저 설치합니다: `sudo apt install terminator`

## 구성 파일

| 파일 | 역할 |
|---|---|
| `open_sim.sh` | Terminator를 전용 설정으로 실행하는 런처. `--opp`로 8분할 선택 |
| `run.sh <role>` | 패인 하나가 실행하는 스크립트. Jazzy + 워크스페이스 소싱 → 이전 잔류 프로세스 정리 → 상류 노드 대기 → launch 실행. Ctrl-C 시 셸로 전환 |
| `f1sim.terminator` | 7분할 레이아웃 (기본 주행) |
| `f1sim_opp.terminator` | 8분할 레이아웃 (상대차 검출기 포함) |

## 패인(role) ↔ CLAUDE.md 터미널 대응

| # | role | 실행 내용 | 대기(초) |
|---|---|---|---|
| 1 | `f1sim` | `ros2 launch f1tenth_gym_ros gym_bridge_launch.py` (f1sim_C) | 0 |
| 2 | `mcl` | `ros2 launch particle_filter_cpp mcl_launch.py mod:=sim map_name:=<맵> use_rviz:=true` | 3 |
| 3 | `global` | `ros2 launch global_planning global_planning.launch.py` | 6 |
| 4 | `local` | `ros2 launch local_planning local_planning.launch.py simulator:=true` | 8 |
| 5 | `state` | `ros2 launch state_machine state_machine.launch.py` | 9 |
| 6 | `wpnt` | `ros2 run wpnt_publisher wpnt_publisher` | 10 |
| 7 | `control` | `ros2 launch f1tenth_control control_sim.launch.py` | 11 |
| 8 | `oppdet` | `ros2 launch obstacle_detector obstacle_detector.launch.py simulator:=true` (`--opp` 전용) | 13 |

대기 시간은 상류 노드가 먼저 뜨도록 순서를 보장하기 위한 것으로, 패인에서 Ctrl-C 한 번이면
대기를 건너뛰고 즉시 실행됩니다.

각 role은 수동으로도 실행할 수 있고, 뒤에 `name:=value` 인자를 덧붙이면 해당 launch에
전달됩니다:

```bash
~/2026_IFAC/sim/run.sh mcl use_rviz:=false
~/2026_IFAC/sim/run.sh stop        # 스택 전체 종료 (gym 브리지 포함)
KEEP_SIM=1 ~/2026_IFAC/sim/run.sh stop   # gym 브리지는 남기고 나머지만 종료
```

## 환경변수 오버라이드

| 변수 | 기본값 | 설명 |
|---|---|---|
| `SIM_MAP_NAME` | `ifac_track` | MCL 맵 이름 (`monte_carlo_localization/maps/<이름>.yaml`) |
| `F1SIM_WS` | `~/f1sim_C` → `~/sim_ws` → `~/f1tenth_gym` 순서로 탐색 | f1tenth_gym_ros 워크스페이스 경로 |
| `MCL_RVIZ` | `true` | MCL 자체 RViz 창 켜기/끄기 (`0`이면 끔) |
| `IFAC_WS` | 스크립트 위치에서 자동 계산 | 이 저장소의 워크스페이스 루트 |

예: `SIM_MAP_NAME=<map> ./sim/open_sim.sh`

## 상대차 시나리오(`--opp`) 전제 조건

1. gym 브리지를 **`num_agent: 2`**(`~/f1sim_C/f1tenth_gym_ros/config/sim.yaml`)로 띄워야 상대차가 스폰됩니다.
   sim.yaml 수정 후에는 패인 1에서 Ctrl-C → 재실행으로 브리지를 다시 띄우세요.
2. 2-agent 브리지는 에고·상대 **둘 다** drive를 받아야 물리 스텝을 돕습니다. 패인 7의 에고
   제어와 별도로, 이 저장소 밖의 발행자가 `/opp_drive`를 제공해야 합니다.
3. RViz에서 `/perception/obstacles/markers`를 Add 하면 검출 결과(빨강=동적, 파랑=정적)가 보입니다.

## 동작 원리 요약

- `open_sim.sh`는 레이아웃 파일의 `@SIMDIR@`를 이 디렉토리의 절대 경로로 치환한 임시 설정을
  만들어 `terminator --no-dbus -g <설정> -l <레이아웃>`으로 실행합니다. 개인
  `~/.config/terminator/config`는 건드리지 않습니다.
- `--no-dbus`가 없으면 이미 떠 있는 Terminator DBus 서버가 호출을 가로채 `-g`/`-l`을 무시하고
  빈 창만 열기 때문에 필수입니다.
- `run.sh`는 role별로 자기 노드의 **이전 잔류 프로세스만** 골라 종료한 뒤 실행합니다
  (중복 실행으로 latched 토픽이 꼬이는 문제 방지). 자기 자신과 조상 프로세스는 절대 건드리지
  않습니다.
- launch가 종료(Ctrl-C)되면 패인은 소싱된 대화형 zsh로 전환되어 곧바로 `ros2 topic echo` 등
  디버깅에 쓸 수 있습니다.
