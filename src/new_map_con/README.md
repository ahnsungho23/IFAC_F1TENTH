# new_map_con

> 보관용 패키지입니다. 현재 워크스페이스에서는 `COLCON_IGNORE`로 빌드·검색 대상에서 제외되어
> 실행 경로에 연결되지 않습니다.

> 🌐 **한국어** · [English](README_en.md)

waypoint 추종 컨트롤러 패키지. `map_controller`(C++) 노드가 global/local 웨이포인트를 pure-pursuit로
추종하며 `/drive`를 발행한다. 차량(vehicle)과 시뮬레이터(simulator) 토픽 프로파일을 모두 지원한다.

노드 동작 원리의 상세 설명은 [`docs/map_controller_node.md`](docs/map_controller_node.md) 참고.

## 1. 작동 원리

1. 시작 시 `global_waypoints_csv`(기본 `maps/fuck_f1.csv`, 8컬럼)를 읽어 `/global_waypoints`로 latched 발행한다.
2. 제어 주기(`control_rate_hz`)마다 현재 pose에서 가장 가까운 웨이포인트를 찾고, 속도 기반
   lookahead 거리만큼 앞의 점을 목표로 pure-pursuit 조향을 계산한다.
3. 목표 속도는 (속도 lookahead 지점의) 웨이포인트 `vx_mps`를 횡오차·곡률로 보정해 명령한다.
4. `use_local_waypoints`가 켜져 있고 `/local_waypoints`가 최신이면 그것을, 아니면 global로 fallback한다.
5. 조향은 가감속·속도에 따라 스케일하고 변화율/최대각으로 제한한다.

## 1.1 세부 파이프라인

```
/pf/pose/odom ──┐
  (or /ego_racecar/odom)
                │
/local_waypoints ──┐
  (f110_msgs/WpntArray) │
                │      │         ┌─────────────────────────────┐
/odom ──────────┼──────┼────────▶│ map_controller              │
                │      │         │ (new_map_con)               │
/state ─────────┼──────┼────────▶│                             │
                ▼      ▼         │  pure-pursuit + speed ctrl  │
                ┌──────┐         │                             │
                │ pose │         └──────┬──────────────────────┘
                │+speed│                │
                └──────┘                ├──▶ /drive (AckermannDriveStamped)
                                        ├──▶ /global_waypoints (WpntArray, latched)
                                        └──▶ steering/lookahead markers (viz)
```

시작 시 CSV 경로 파일을 로드하여 `/global_waypoints`를 latch(유지) 발행하며, 이 토픽은 global_planning, wpnt_publisher, opponent_detector가 소비합니다.

## 2. 구독 / 발행 토픽

`simulator` 파라미터로 토픽 프로파일이 바뀐다(아래는 기본 프로파일 토픽).

| 방향 | 토픽(시뮬/차량) | 메시지 타입 | 설명 |
|------|------------------|-------------|------|
| 구독 | `/local_waypoints` | `f110_msgs/WpntArray` | 지역 경로(있으면 우선) |
| 구독 | `/ego_racecar/odom` / `/pf/pose/odom` | `nav_msgs/Odometry` | pose |
| 구독 | `/ego_racecar/odom` / `/odom` | `nav_msgs/Odometry` | speed |
| 구독 | `/state` | `std_msgs/String` | 상태머신(선택) |
| 구독 | (off) / `/sensors/imu/raw` | `sensor_msgs/Imu` | 가속도(조향 스케일, 선택) |
| 발행 | `/drive` | `ackermann_msgs/AckermannDriveStamped` | 주행 명령 |
| 발행 | `/global_waypoints` | `f110_msgs/WpntArray` | global 경로(latched) |
| 발행 | `steering`, `lookahead_point`, `my_waypoints`, `l1_distance` | `visualization_msgs/*`, `geometry_msgs/Point` | 디버그 마커 |

## 3. 주요 파라미터

전체 목록과 기본값: [`config/config.yaml`](config/config.yaml). 노드에 안전 기본값이 선언되어 있다.

| 파라미터 | 의미 | 기본값 |
|----------|------|--------|
| `global_waypoints_csv` | 추종할 raceline CSV 경로 | `maps/fuck_f1.csv` |
| `simulator` | 시뮬레이터 토픽 프로파일 사용 | `false` |
| `control_rate_hz` | 제어 주기 | `40.0` |
| `min/max_lookahead_distance`, `lookahead_gain`, `lookahead_speed_gain` | lookahead 계산 | `1.5`/`5.0`/`0.5`/`0.3` |
| `wheelbase`, `max_steering_angle`, `max_steering_delta` | 조향 기하/제한 | `0.33`/`0.42`/`0.4` |
| `lateral_error_coeff`, `curvature_scale` | 속도 보정 | `1.0`/`0.8` |
| `use_local_waypoints`, `fallback_to_global_waypoints` | 경로 선택 | `true`/`true` |

## 4. 빌드 및 실행

```bash
cd ~/2026_IFAC
colcon build --packages-select new_map_con
source install/setup.zsh

# 차량(기본 프로파일)
ros2 launch new_map_con new_map_con.launch.py

# 시뮬레이터 프로파일 (먼저 다른 터미널에서 f1sim 실행)
ros2 launch new_map_con new_map_con.launch.py simulator:=true
```

다른 raceline CSV로 주행(예: dl_speed_optimizer 결과):

```bash
ros2 run new_map_con map_controller --ros-args \
  --params-file src/new_map_con/config/config.yaml \
  -p simulator:=true \
  -p global_waypoints_csv:=<optimized CSV 절대경로> \
  -p package_resource_root:=src/new_map_con
```

## 5. 참고

- node-level 규칙: [`AGENTS.md`](AGENTS.md)
- 동작 원리 문서: [`docs/map_controller_node.md`](docs/map_controller_node.md)
- 이 컨트롤러가 추종하는 raceline은 `offline_trajectory_generator`/`dl_speed_optimizer`가 만든다.
