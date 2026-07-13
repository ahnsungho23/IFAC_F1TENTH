# lap_referee

> 🌐 **한국어** · [English](README_en.md)

f1tenth gym 시뮬레이터에서 **한 번의 주행(롤아웃)을 심판·기록**하는 C++ 노드 패키지.
충돌/완주/이탈/정지/타임아웃을 판정하고 랩타임과 궤적을 파일로 남긴다. `dl_speed_optimizer`의
ROS 백엔드가 후보 속도 프로파일을 자동 평가할 때 사용한다.

상세 동작 설명: [`docs/lap_referee_node.md`](docs/lap_referee_node.md).

## 1. 작동 원리

브리지가 충돌을 ROS 토픽으로 알리지 않으므로, **관측 가능한 신호만으로** 결과를 복원한다.

1. 기준 raceline CSV(`waypoints_csv`)로 트랙 길이·각 점의 `s`를 계산한다.
2. `odom`/`scan`/`drive`를 구독하며 50 Hz로 상태를 갱신, 속도가 임계값을 넘는 순간을 t0로 잡는다.
3. 가장 가까운 웨이포인트 인덱스의 전진을 누적해 진행거리를 구한다.
4. 종료: `lap_complete`(진행거리≥트랙×`lap_fraction`) / `collision`(LiDAR 최소거리<임계) /
   `off_track`(횡오차 초과) / `stuck`(명령은 높은데 정지) / `timeout` / `no_start`.
5. 종료 시 결과를 원자적으로 파일에 쓰고(임시→rename), 0속도 `/drive`를 발행한 뒤 프로세스를 종료한다.

한 프로세스 = 한 롤아웃. 오케스트레이터는 프로세스 종료로 완료를 감지한다.

## 1.1 세부 파이프라인

```
/ego_racecar/odom ─────────┐
  (nav_msgs/Odometry)       │     ┌─────────────────────────────┐
                             ├────▶│ lap_referee                  │
/scan ───────────────────────┤     │                             │
  (sensor_msgs/LaserScan)    │     │ Rollout judge: collision/   │
                             │     │ complete/off_track/stuck/    │
/drive ──────────────────────┘     │ timeout/no_start            │
  (AckermannDriveStamped)          └──────┬──────────────────────┘
                                          │
                                          ├──▶ result JSON + trace CSV (file)
                                          └──▶ /drive (0-speed stop, then exit)
```

하나의 프로세스 = 하나의 롤아웃. dl_speed_optimizer가 이 노드를 평가 백엔드로 사용합니다.

## 2. 구독 / 발행 토픽

| 방향 | 토픽 | 타입 | 기본값 |
|------|------|------|--------|
| 구독 | `odom_topic` | `nav_msgs/Odometry` | `/ego_racecar/odom` |
| 구독 | `scan_topic` | `sensor_msgs/LaserScan` | `/scan` (gym 브리지는 ego scan을 `/scan`으로 발행) |
| 구독 | `drive_topic` | `ackermann_msgs/AckermannDriveStamped` | `/drive` |
| 발행 | `drive_topic` | `ackermann_msgs/AckermannDriveStamped` | 종료 시 0속도 정지(선택) |

## 3. 주요 파라미터

전체: [`config/lap_referee.yaml`](config/lap_referee.yaml).

| 파라미터 | 의미 | 기본값 |
|----------|------|--------|
| `waypoints_csv` | 기준 raceline CSV(필수) | `''` |
| `output_dir` / `output_prefix` | 결과 파일 위치/접두어 | `/tmp/lap_referee` / `rollout` |
| `collision_scan_threshold` | 충돌 판정 최소 LiDAR [m] | `0.13` |
| `off_track_threshold` | 이탈 판정 횡오차 [m] | `1.5` |
| `stuck_speed_threshold` / `stuck_cmd_threshold` / `stuck_time_sec` | 정지 판정 | `0.2`/`0.8`/`0.7` |
| `lap_fraction` / `max_episode_time_sec` | 완주 비율 / 최대 시간 | `0.97` / `60.0` |

## 4. 출력 파일

- `<output_dir>/<prefix>_summary.json` — 롤아웃 요약(`terminated`, `collided`, `lap_time_s`, `crash_s` 등).
- `<output_dir>/<prefix>_trace.csv` — `t,x,y,yaw,v,cmd_v,cmd_steer,min_scan,nearest_idx,lat_err,s`.

JSON 스키마는 `dl_speed_optimizer`(`dl_speed_opt/evaluators.py`의 `SimRunner._parse_outputs`)와의 계약이다.

## 5. 빌드 및 실행

```bash
cd ~/2026_IFAC
colcon build --packages-select lap_referee
source install/setup.zsh

# 단독 실행(런치는 waypoints_csv를 반드시 넘긴다)
ros2 launch lap_referee lap_referee.launch.py \
  waypoints_csv:=$HOME/2026_IFAC/src/new_map_con/maps/fuck_f1.csv \
  output_dir:=/tmp/lap_referee output_prefix:=rollout
```

전체 폐루프(시뮬+컨트롤러+심판)는 `dl_speed_optimizer`의 `--backend ros`가 자동으로 묶어 실행한다.

## 6. 참고

- node-level 규칙: [`AGENTS.md`](AGENTS.md)
- 상세 문서: [`docs/lap_referee_node.md`](docs/lap_referee_node.md)
