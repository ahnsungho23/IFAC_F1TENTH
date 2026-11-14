# 2026 IFAC F1TENTH

> **이 저장소는 팀 전체의 자율주행 스택입니다.**
> 제 담당은 **[`src/local_planning`](src/local_planning/) — 정적 장애물 회피 로컬 플래너**입니다.
> 그 외 패키지(위치추정·글로벌 플래닝·인지·상태 머신·제어 등)는 다른 팀원이 담당하며,
> 스택 전체를 실행하고 통합 검증하기 위해 함께 포함되어 있습니다.

1/10 스케일 자율주행 레이싱 차량(F1TENTH)을 위한 ROS 2 스택입니다.
위치추정 → 글로벌 경로 → 장애물 인지 → 로컬 회피 경로 → 상태 판단 → 제어까지 전체 파이프라인을 포함합니다.

- **대상 환경**: ROS 2 Jazzy (Ubuntu 24.04), 런타임 노드는 C++
- **시뮬레이터**: 별도 워크스페이스 `~/f1sim_C`의 `f1tenth_gym_ros`
- **실차 실행**: [`real/README.md`](real/README.md) 참고 (SLAM 맵 생성, 젯슨 전송, 실차 터미널 구성, rosbag 등)

---

## 1. 담당 파트: `local_planning`

정적 장애물을 만났을 때 글로벌 레이스라인을 기준으로 회피 경로를 만드는 C++ 노드
(`local_planner_node`)입니다.

- **레이스라인 잠금(race-line locking)**: 자유공간에서 새 경로를 탐색하지 않고, 글로벌 웨이포인트의
  순서를 그대로 유지한 채 각 점의 Frenet 횡오프셋 `d(s)`만 수정합니다. 트랙 조각이 지도상
  가깝게 붙어 있어도 다른 구간으로 경로가 점프하지 않습니다.
- **입력**: `/global_waypoints`, `/car_state/frenet/odom`, 인지 모듈이 확정한 정적 장애물
  (`/confirmed_static_obs`), 확정 전 장애물(`/static_obs`, 감속 힌트 전용), `/state`
- **출력**: 회피 경로 `/avoid_waypoints` (`f110_msgs/OTWpntArray`), 시각화용 `/local_planning/path`
- **주기**: 40 Hz (25 ms)

자세한 알고리즘(코리도어 계산, 5차 프로파일 후보 생성, 검증, 합류)과 파라미터는
[`src/local_planning/README.md`](src/local_planning/README.md)와
[`src/local_planning/docs/local_planner.md`](src/local_planning/docs/local_planner.md)에 정리되어 있습니다.

---

## 2. 시스템 구성

| 패키지 | 역할 |
|---|---|
| [`kinematic_localization`](src/kinematic_localization/) | Kinematic-ICP 기반 위치추정. `/pf/pose/odom`과 `map→odom` TF 발행 |
| [`global_planning`](src/global_planning/) | 오프라인 생성된 레이스라인을 `/global_waypoints`로 발행, CLCS 기반 Frenet 좌표 `/car_state/frenet/odom` 계산 |
| [`obstacle_detector`](src/obstacle_detector/) | LiDAR 레이어드 검출. 정적 장애물 `/static_obs`·`/confirmed_static_obs`, 상대차 `/opp_obs` |
| [`local_planning`](src/local_planning/) | **(담당)** 정적 장애물 회피 경로 `/avoid_waypoints` |
| [`state_machine`](src/state_machine/) | 주행 상태(GLOBAL / AVOID / CRUISE) 판정, 최종 경로 `/local_waypoints` 선택 |
| [`f1tenth_control`](src/f1tenth_control/) | L1 Guidance + Steering LUT(MAP) / MPPI 이중 제어기, `/drive` 발행 |
| [`new_map_con`](src/new_map_con/) | 시뮬용 상대차 주행기 등 |
| [`lap_referee`](src/lap_referee/), [`lap_timer`](src/lap_timer/) | 랩 판정·랩타임 표시 |
| [`f110_msgs`](f110_msgs/) | 공용 메시지 정의 (`WpntArray`, `ObstacleArray`, `StateMachine` 등) |
| [`offline_trajectory_generator`](offline_trajectory_generator/) | 점유격자 맵 → 최소곡률 레이스라인·속도 프로파일 생성 (C++ 엔진 + Python GUI) |

### 토픽 흐름

```
 f1tenth_gym_ros (시뮬) / 센서 드라이버 (실차)
   │  /scan, odom
   ▼
 kinematic_localization ──► /pf/pose/odom
   │
   ▼
 global_planning ──► /global_waypoints, /car_state/frenet/odom
   │
   ├──────────────► obstacle_detector ──► /static_obs, /confirmed_static_obs, /opp_obs
   │                        │
   ▼                        ▼
 local_planning (담당) ──► /avoid_waypoints
   │
   ▼
 state_machine ──► /state (GLOBAL / AVOID / CRUISE), /local_waypoints
   │
   ▼
 f1tenth_control ──► /drive  (시뮬: gym 브리지 / 실차: VESC)
```

### 디렉터리 구조

```text
2026_IFAC/
├── src/                          # ROS 2 패키지 (위 표 참고)
├── f110_msgs/                    # 공용 메시지 패키지
├── offline_trajectory_generator/ # 오프라인 레이스라인 생성기 (output/은 gitignore)
├── third_party/python_libs/      # steering_lookup (조향 LUT 라이브러리)
├── sim/                          # 시뮬 원클릭 실행 스크립트 (Terminator)
├── real/                         # 실차 원클릭 실행 스크립트 + 실차 수동 실행 메모
├── docs/                         # 저장소 공통 문서 (Jazzy 포트 검증 기록 등)
├── AGENTS.md, CLAUDE.md          # 저장소 작업 규칙
└── README.md
```

각 패키지의 노드 설명·파라미터·실행 방법은 패키지 안의 `README.md` 또는 `docs/`에 있습니다.

---

## 3. 빌드

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install
```

- **`--symlink-install`을 항상 붙입니다.** 일반 빌드와 섞으면 `f110_msgs`에서 심볼릭 링크 충돌로
  빌드가 깨집니다. 깨졌다면 `rm -rf build install` 후 다시 빌드합니다.
- `config/*.yaml`, `launch/*.launch.py`는 symlink-install이어도 복사 설치되므로, 수정 후에는
  해당 패키지를 다시 빌드해야 반영됩니다:
  `colcon build --symlink-install --packages-select <pkg>`

---

## 4. 시뮬레이션 실행

**사전 준비**

1. `~/f1sim_C`에 `f1tenth_gym_ros`가 빌드되어 있어야 합니다.
2. 글로벌 웨이포인트(`offline_trajectory_generator/output/<맵>/global_waypoints.json`)가 있어야 합니다
   (아래 §5.2).
3. 시뮬레이터 맵(`~/f1sim_C/f1tenth_gym_ros/config/sim.yaml`의 `map_path`), 위치추정 맵,
   웨이포인트 생성 맵은 **반드시 같은 트랙**이어야 합니다.

**원클릭 실행 (권장)**: Terminator가 설치되어 있으면 아래 한 줄로 전체 스택을 띄웁니다.
자세한 내용은 [`sim/README.md`](sim/README.md) 참고.

```bash
./sim/open_sim.sh          # 기본 주행 (터미널 1~6)
./sim/open_sim.sh --opp    # 상대차 포함 (터미널 1~8, sim.yaml의 num_agent: 2 필요)
```

**수동 실행**: 각 터미널에서 `cd ~/2026_IFAC && source /opt/ros/jazzy/setup.zsh && source install/setup.zsh`
후 아래 순서대로 실행합니다 (터미널 1만 `~/f1sim_C`에서 실행).

| # | 역할 | 명령 |
|---|---|---|
| 1 | 시뮬레이터 | `ros2 launch f1tenth_gym_ros gym_bridge_launch.py` |
| 2 | 위치추정 | `ros2 launch kinematic_localization kinematic_localization.launch.py map_name:=ifac_track use_sim_time:=true` |
| 3 | 글로벌 플래너 | `F1_MAP=ifac_track ros2 launch global_planning global_planning.launch.py` |
| 4 | 로컬 플래너 (+검출기) | `ros2 launch local_planning local_planning.launch.py simulator:=true` |
| 5 | 상태 머신 | `ros2 launch state_machine state_machine.launch.py` |
| 6 | 제어 | `ros2 launch f1tenth_control control_sim.launch.py` |
| 7 | (선택) 상대차 | `ros2 launch new_map_con opponent_simulator.launch.py` |

- 터미널 4는 obstacle_detector를 함께 띄웁니다. **`simulator:=true`를 빠뜨리면** 검출기가 실차용
  odom 토픽을 기다리며 상대차 검출이 꺼집니다.
- 초기 위치는 `/global_waypoints`의 첫 점(스타트라인)으로 자동 초기화됩니다. 수동 지정은 RViz의
  **2D Pose Estimate** 또는 `/initialpose` 발행으로 합니다.
- RViz에서 `/perception/obstacles/markers`를 추가하면 검출 결과(빨강=상대차, 파랑=정적)가 보입니다.

---

## 5. 주의사항

### 5.1 워크스페이스 루트(`~/2026_IFAC`)에서 실행

`global_planning`의 웨이포인트 경로가 상대경로(`offline_trajectory_generator/output`)이므로
다른 디렉터리에서 실행하면 파일을 찾지 못합니다.

### 5.2 글로벌 웨이포인트가 먼저 있어야 함

글로벌 플래너는 기동 시 `offline_trajectory_generator/output/<F1_MAP>/global_waypoints.json`을
**한 번만** 읽습니다. 파일이 없으면 `/global_waypoints`가 발행되지 않아 이후 노드가 모두 대기합니다.
맵을 바꾸거나 파일을 교체했다면 글로벌 플래너를 재시작해야 합니다. `output/`은 gitignore 대상이므로
클린 체크아웃에서는 직접 생성합니다:

```bash
cd ~/2026_IFAC
offline_trajectory_generator/bin/generate_global_trajectory \
  --map-yaml src/kinematic_localization/maps/ifac_track.yaml \
  --output-dir offline_trajectory_generator/output/ifac_track \
  --optimizer mincurv --raceline-smooth-sigma 3.0 \
  --max-speed 0.5 --min-speed 0.4 \
  --velocity-limits-csv offline_trajectory_generator/config/velocity_limits.csv
```

### 5.3 위치추정 맵은 두 종류

위치추정은 `maps/<맵>.kissmap`(포인트 맵), 글로벌·로컬 플래너는 `maps/<맵>.yaml`(점유격자)을 읽습니다.
둘 다 `src/kinematic_localization/maps/`에 있어야 하고 같은 트랙이어야 합니다.
점유격자에서 `scripts/pgm_to_kissmap.py`로 `.kissmap`을 만들 수 있습니다.

### 5.4 파라미터 파일 지정

각 launch는 패키지의 `config/*.yaml`을 기본으로 씁니다. 다른 파일을 쓰려면 `params_file:=` 인자를 넘깁니다:

```bash
ros2 launch local_planning local_planning.launch.py simulator:=true \
  params_file:=$HOME/2026_IFAC/src/local_planning/config/local_planning.yaml
```

---

## 6. 개발 이력

- Humble → **ROS 2 Jazzy 포팅** 및 빌드·폐루프 주행 검증 기록:
  [`docs/jazzy_port_validation.md`](docs/jazzy_port_validation.md)
- 2026-08-20: 위치추정을 MCL(`particle_filter_cpp`)에서 Kinematic-ICP(`kinematic_localization`)로 교체
  (토픽·TF 인터페이스는 동일)
