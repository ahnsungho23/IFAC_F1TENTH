# map_controller 노드

## 1. 목적

`map_controller`는 offline trajectory generator가 만든 CSV를 읽어 `/global_waypoints`를 발행하고, 차량 위치와 waypoint를 이용해 `/drive` 명령을 계산하는 C++ ROS 2 노드이다.

## 2. 동작 원리

1. 시작할 때 `global_waypoints_csv` 파라미터의 CSV 파일을 읽는다.
2. CSV는 `id,s,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2` 형식을 기본으로 사용한다.
3. 읽은 CSV를 `f110_msgs/WpntArray`로 변환해 `/global_waypoints`에 transient local QoS로 발행한다.
4. `/local_waypoints`가 들어오면 local waypoint를 우선 사용한다.
5. local waypoint가 없거나 오래되면 설정에 따라 global waypoint로 fallback한다.
6. 현재 pose, 속도, waypoint 곡률, 횡오차를 이용해 목표 속도와 pure-pursuit 조향각을 계산한다.
7. 계산된 명령을 `ackermann_msgs/AckermannDriveStamped`로 발행한다.

## 3. 구독 토픽

- `/local_waypoints`: `f110_msgs/WpntArray`
- pose topic: `nav_msgs/Odometry`
- speed topic: `nav_msgs/Odometry`
- `/state`: `f110_msgs/StateMachine` (state_machine 패키지와 동일, QoS는 transient_local)
- imu topic: `sensor_msgs/Imu`

토픽 이름은 모두 `config/config.yaml`에서 바꿀 수 있다. `pose_topic`, `speed_topic`, `drive_topic`, `imu_topic`을 비워두면 `simulator` 값에 따라 simulator topic profile 또는 vehicle topic profile을 자동으로 사용한다.

## 4. 발행 토픽

- `/global_waypoints`: `f110_msgs/WpntArray`
- `/drive`: `ackermann_msgs/AckermannDriveStamped`
- `steering`: `visualization_msgs/Marker`
- `lookahead_point`: `visualization_msgs/Marker`
- `my_waypoints`: `visualization_msgs/MarkerArray`
- `l1_distance`: `geometry_msgs/Point`

## 5. 주요 파라미터

파라미터 파일 위치:

```text
src/new_map_con/config/config.yaml
```

주요 항목:

- `global_waypoints_csv`: 읽을 waypoint CSV 경로이다. 상대 경로이면 패키지 share 디렉터리 기준으로 해석한다. 기본값은 f1sim의 `fuck_f1.yaml` map과 같은 좌표계를 쓰는 `maps/fuck_f1.csv`이다.
- `package_resource_root`: `global_waypoints_csv` 같은 상대 리소스 경로를 먼저 찾을 package root이다. 빈 값이면 install된 package share를 사용한다.
- `simulator`: simulator topic profile을 사용할지 결정한다.
- `simulator_pose_topic`, `simulator_speed_topic`, `simulator_imu_topic`, `simulator_drive_topic`: `simulator: true`일 때 쓰는 topic이다.
- `vehicle_pose_topic`, `vehicle_speed_topic`, `vehicle_imu_topic`, `vehicle_drive_topic`: `simulator: false`일 때 쓰는 topic이다.
- `pose_topic`, `speed_topic`, `imu_topic`, `drive_topic`: 비어 있지 않으면 `simulator` profile보다 우선 적용되는 직접 override topic이다.
- `publish_global_waypoints`: CSV에서 읽은 global waypoint를 발행할지 결정한다.
- `use_local_waypoints`: `/local_waypoints`를 제어에 우선 사용할지 결정한다.
- `fallback_to_global_waypoints`: local waypoint가 없을 때 global waypoint로 제어할지 결정한다.
- `control_rate_hz`: 제어 루프 주기이다.
- `min_lookahead_distance`, `max_lookahead_distance`: L1 lookahead 거리 제한이다.
- `lookahead_gain`, `lookahead_speed_gain`: 속도에 따른 lookahead 거리 계산 계수이다.
- `wheelbase`: pure-pursuit 조향 계산에 사용하는 차량 축거이다.
- `max_steering_angle`, `max_steering_delta`: 조향각 및 조향 변화량 제한이다.
- `lateral_error_coeff`: 횡오차 기반 속도 감속 강도이다.
- `default_track_bound`: CSV에 `d_left`, `d_right`가 없을 때 채울 기본 track bound 값이다.

## 6. 실행 방법

빌드:

```bash
colcon build --packages-up-to new_map_con --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
```

환경 설정:

```bash
source install/setup.zsh
```

기본 설정으로 실행:

```bash
ros2 launch new_map_con new_map_con.launch.py
```

launch 파일은 source workspace가 보이면 기본 `params_file`과 `package_resource_root`를 `src/new_map_con`으로 잡는다. 그래서 `cb --symlink-install` 이후에는 `src/new_map_con/config/config.yaml`이나 `src/new_map_con/maps/*.csv`를 수정하고 다시 launch하면 별도 rebuild 없이 반영된다.

시뮬레이터 토픽 프로파일로 실행:

```bash
ros2 launch new_map_con new_map_con.launch.py simulator:=true
```

f1sim과 같이 실행할 때는 f1sim의 `sim.yaml`에 설정된 `map_path`와 `global_waypoints_csv`가 같은 map YAML에서 만들어진 파일인지 먼저 확인한다. 현재 기본 설정은 둘 다 `fuck_f1` map 기준이다.

다른 CSV를 쓰려면 `config/config.yaml`의 `global_waypoints_csv`를 바꾼다. 예를 들어 offline trajectory generator 출력 파일을 직접 쓰려면 홈 기준 경로를 넣는다. 노드가 선행 `~`/`$HOME`을 직접 펼치므로 사용자 이름을 박아 넣을 필요가 없다.

```yaml
global_waypoints_csv: $HOME/2026_IFAC/offline_trajectory_generator/output/fuck_f1_1/global_waypoints.csv
```

## 7. offline trajectory generator와의 연결

offline trajectory generator의 `global_waypoints.csv`는 `src/new_map_con/maps/fuck_f1.csv`와 같은 8개 컬럼 형식으로 저장된다.

```text
id,s,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2
```

따라서 `global_waypoints_csv`에 해당 CSV 경로를 지정하면 `new_map_con`에서 바로 읽어 `/global_waypoints`로 발행할 수 있다. 단, RViz에서 path가 map 위에 올라오려면 generator에서 선택한 map YAML과 f1sim/map server가 띄우는 map YAML의 `resolution`과 `origin`이 같아야 한다.
