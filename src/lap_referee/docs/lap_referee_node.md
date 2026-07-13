# lap_referee 노드

## 1. 목적

`lap_referee`는 f1tenth_gym_ros 시뮬레이터에서 **한 번의 주행(롤아웃)을 심판하고 기록**하는 노드이다.
DL 속도 최적화기(`dl_speed_optimizer`)가 후보 속도 프로파일의 성능(랩타임)과 안전성(충돌 여부)을
자동으로 측정하기 위해 사용한다.

f1tenth gym 브리지는 시뮬레이터 내부의 충돌/랩 상태를 ROS 토픽으로 다시 내보내지 않는다.
따라서 이 노드는 **ROS로 관측 가능한 신호만으로** 주행 결과를 복원한다.

## 2. 동작 원리

한 프로세스가 한 롤아웃을 담당한다. 노드는 다음 순서로 동작한다.

1. 기준 raceline CSV(`waypoints_csv`)를 읽어 트랙 길이와 각 웨이포인트의 호 길이 `s`를 다시 계산한다.
2. `odom`/`scan`/`drive`를 구독하며 50 Hz 타이머로 상태를 갱신한다.
3. 차량 속도가 `start_speed_threshold`를 처음 넘는 순간을 출발 시각(t0)으로 잡는다(출발 전 정지 구간은 기록하지 않는다).
4. 가장 가까운 웨이포인트 인덱스의 전진을 누적해 트랙 진행거리를 구한다(되감기/노이즈는 무시).
5. 매 샘플마다 시간/자세/속도/명령속도/조향/최소 LiDAR/가장가까운 인덱스/횡오차/`s`를 trace에 적재한다.
6. 다음 조건 중 하나로 롤아웃을 종료한다.
   - `lap_complete`: 누적 진행거리가 `track_length × lap_fraction` 이상.
   - `collision`: 최소 LiDAR 거리가 `collision_scan_threshold` 미만(벽 충돌).
   - `off_track`: 횡오차 절댓값이 `off_track_threshold` 초과(경로 이탈).
   - `stuck`: 명령속도는 높은데(`stuck_cmd_threshold`↑) 실제 속도가 낮은(`stuck_speed_threshold`↓) 상태가 `stuck_time_sec` 지속(충돌 후 차량 정지).
   - `timeout`: 주행 시간이 `max_episode_time_sec` 초과.
   - `no_start`: 차량이 끝내 출발하지 않음.
7. 종료 시 결과를 **원자적으로** 파일에 기록하고(임시 파일→rename), `stop_vehicle_on_exit`가 켜져 있으면 0속도 `/drive`를 발행한 뒤, `shutdown_on_terminate`가 켜져 있으면 프로세스를 종료한다.

종료 직후 출발 그레이스(`startup_grace_sec`) 동안에는 충돌 판정을 건너뛰어 런치 직후 오판을 막는다.

## 3. 구독/발행 토픽

| 방향 | 토픽 | 메시지 타입 | 기본값 | 설명 |
|------|------|-------------|--------|------|
| 구독 | `odom_topic` | `nav_msgs/msg/Odometry` | `/ego_racecar/odom` | 자세·속도 |
| 구독 | `scan_topic` | `sensor_msgs/msg/LaserScan` | `/scan` | 벽 근접/충돌 감지 (gym 브리지는 ego scan을 네임스페이스 없이 `/scan`으로 발행) |
| 구독 | `drive_topic` | `ackermann_msgs/msg/AckermannDriveStamped` | `/drive` | 명령 속도(stuck 판정용) |
| 발행 | `drive_topic` | `ackermann_msgs/msg/AckermannDriveStamped` | `/drive` | 종료 시 0속도 정지(선택) |

메시지는 모두 ROS 2 표준(`nav_msgs`/`sensor_msgs`) 및 차량 제어 표준 `ackermann_msgs`를 사용한다.

## 4. 주요 파라미터와 YAML 위치

파라미터 파일: `src/lap_referee/config/lap_referee.yaml`

| 파라미터 | 의미 | 기본값 |
|----------|------|--------|
| `waypoints_csv` | 기준 raceline CSV 경로(필수) | `''` |
| `output_dir` | 결과 파일 디렉터리 | `/tmp/lap_referee` |
| `output_prefix` | 결과 파일 접두어 | `rollout` |
| `start_speed_threshold` | 출발 판정 속도 [m/s] | `0.4` |
| `collision_scan_threshold` | 충돌 판정 최소 LiDAR [m] | `0.13` |
| `off_track_threshold` | 이탈 판정 횡오차 [m] | `1.5` |
| `stuck_speed_threshold` / `stuck_cmd_threshold` / `stuck_time_sec` | stuck 판정 | `0.2` / `0.8` / `0.7` |
| `lap_fraction` | 랩 완주 판정 비율 | `0.97` |
| `max_episode_time_sec` | 최대 주행 시간 [s] | `60.0` |

## 5. 출력 파일

- `<output_dir>/<prefix>_summary.json` — 롤아웃 요약(아래 필드).
- `<output_dir>/<prefix>_trace.csv` — `t,x,y,yaw,v,cmd_v,cmd_steer,min_scan,nearest_idx,lat_err,s`.

`summary.json` 필드: `schema`, `terminated`, `collided`, `lap_completed`, `lap_time_s`,
`progress_m`, `track_length_m`, `start_index`, `crash_x`, `crash_y`, `crash_s`, `last_lat_err`,
`min_clearance_m`, `mean_speed_mps`, `max_speed_mps`, `n_samples`, `trace_csv`.

## 6. 실행 방법

먼저 워크스페이스를 빌드하고 소스한다.

```bash
cd ~/2026_IFAC
colcon build --packages-select lap_referee
source install/setup.zsh   # zsh 기준 (alias: sc)
```

기준 raceline CSV를 지정해 단독 실행한다(런치는 `waypoints_csv:=`를 반드시 넘겨야 한다).

```bash
ros2 launch lap_referee lap_referee.launch.py \
  waypoints_csv:=$HOME/2026_IFAC/src/new_map_con/maps/fuck_f1.csv \
  output_dir:=/tmp/lap_referee output_prefix:=rollout
```

또는 `ros2 run`으로 직접(최적화기는 이 방식을 사용한다):

```bash
ros2 run lap_referee lap_referee --ros-args \
  --params-file $(ros2 pkg prefix lap_referee)/share/lap_referee/config/lap_referee.yaml \
  -p waypoints_csv:=$HOME/2026_IFAC/src/new_map_con/maps/fuck_f1.csv \
  -p output_dir:=/tmp/lap_referee -p output_prefix:=rollout
```

전체 폐루프(시뮬레이터 + 컨트롤러 + 심판)는 `dl_speed_optimizer`가 자동으로 묶어 실행한다.
시각화가 필요하면 별도 터미널에서 `f1sim`을 띄운다.
