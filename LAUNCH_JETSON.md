# Jetson 및 로컬 실행 안내

이 문서는 `~/2026_IFAC` 자율주행 스택을 실차에서 실행할 때의 터미널 구성을 정리한 문서입니다.

## 1. 실행 위치 구분

### Jetson SSH 터미널

실차의 센서와 차량 제어에 직접 연결되는 노드는 Jetson에서 실행합니다.

- f110 센서/VESC 드라이버
- `/scan`, `/odom`, `/joy`, IMU 관련 토픽 제공 노드
- MCL, 플래닝, 상태 머신, 웨이포인트, 제어 노드

SSH 접속 예시:

```bash
ssh miru@10.1.1.3
```

Jetson의 각 터미널에서 공통으로 실행합니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=0
```

시각화 출력을 제어하는 노드를 실행할 터미널에서는 다음 변수도 선언합니다. 변수는 터미널마다
독립적이므로 MCL, Global planning, Local planning, State machine, Control 터미널에서 각각
선언해야 합니다.

```zsh
PROFILE="$PWD/src/f1tenth_control/config/runtime_visualization.yaml"
```

`runtime_profile` 인자를 생략하면 설치 공간의 기본 YAML을 읽지만, 위처럼 소스 YAML을 명시하면
파일 수정 결과를 재빌드하지 않고 노드 재시작만으로 시험할 수 있습니다.

터미널 7의 실차 제어는 추가로 VESC 패키지 워크스페이스를 소싱해야 합니다.

```zsh
source ~/f1tenth_ws/install/setup.zsh
```

### 로컬 PC 터미널

다음 작업은 로컬 PC에서 실행합니다.

- `slam_toolbox` 실행
- RViz 실행 및 SLAM 조작
- 지도 저장
- 생성된 지도와 오프라인 생성 결과를 Jetson으로 복사
- 필요 시 대시보드 실행

로컬 PC가 Jetson의 ROS 2 토픽을 보려면 양쪽 장비의 ROS 환경과 네트워크 설정이 일치해야 합니다.

```zsh
source /opt/ros/jazzy/setup.zsh
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=0
```

## 2. SLAM으로 지도 만들기

SLAM은 로컬 PC에서 실행하지만, LiDAR와 odometry 데이터는 Jetson의 센서 드라이버가 제공합니다.
따라서 다음 데이터가 먼저 살아 있어야 합니다.

```text
/scan
odometry 토픽
TF: base_link -> laser
```

프레임 이름은 장비 설정에 따라 `base_link`, `laser` 또는 `ego_racecar/base_link`,
`ego_racecar/laser`일 수 있습니다.

### 2.1 Jetson에서 센서 확인

Jetson에서 f110 센서/VESC 드라이버를 실행한 후 확인합니다.

```zsh
ros2 topic hz /scan
ros2 topic list | grep -E 'scan|odom|imu|joy'
ros2 run tf2_tools view_frames
```

아래 명령이 계속 `Invalid frame ID`를 출력하면 센서 드라이버, odometry 또는 TF가 아직 실행되지 않은 것입니다.

```zsh
ros2 run tf2_ros tf2_echo base_link laser
```

### 2.2 로컬 PC에서 SLAM 실행

```zsh
cd ~/slam_toolbox
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=0
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

RViz에서 LaserScan이 보이지 않으면 `/scan` 토픽 이름과 SLAM 설정의 scan topic, 그리고 TF 프레임 이름을 확인합니다.

차량을 천천히 주행시키며 지도를 만든 뒤 SLAM을 종료하기 전에 지도를 저장합니다.

### 2.3 로컬 PC에서 지도 저장

```zsh
cd ~/slam_toolbox
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=0
mkdir -p ~/slam_toolbox/maps
ros2 topic echo /map --once
ros2 run nav2_map_server map_saver_cli \
  -f ~/slam_toolbox/maps/map \
  --ros-args -p map_subscribe_transient_local:=true
```

생성되는 주요 파일은 다음과 같습니다.

```text
~/slam_toolbox/maps/map.yaml
~/slam_toolbox/maps/map.pgm 또는 map.png
```

### 2.4 지도를 Jetson으로 전송

로컬 PC에서 실행합니다.

```zsh
scp ~/slam_toolbox/maps/map.png ~/slam_toolbox/maps/map.yaml \
  miru@10.1.1.3:~/2026_IFAC/src/monte_carlo_localization/maps/
```

실제 저장 확장자가 `map.pgm`이면 `map.png` 대신 `map.pgm`을 전송합니다. MCL 실행 시 `map_name:=map`으로 지정하면
해당 디렉터리의 `map.yaml`을 사용합니다.

## 3. 실차 구동 전제 조건

실차 주행 전 다음 항목을 확인합니다.

1. Jetson에서 f110 센서/VESC 드라이버가 실행 중입니다.
2. `/scan`, `/joy`, odometry, IMU 토픽이 발행됩니다.
3. LiDAR와 차량 프레임 사이의 TF가 존재합니다.
4. MCL용 지도와 `global_waypoints.json`이 준비되어 있습니다.
5. `~/f1tenth_ws/install/setup.zsh`에 `vesc_ackermann`이 설치되어 있습니다.

확인 명령:

```zsh
ros2 topic hz /scan
ros2 topic hz /ego_racecar/odom
ros2 topic echo /joy --once
ros2 run tf2_tools view_frames
```

## 4. 실차 구동 터미널 1~7 — 모두 Jetson SSH에서 실행

아래 명령은 모두 Jetson에 SSH로 접속한 각각의 터미널에서 실행합니다. 터미널 1의 센서 드라이버는 별도 f110 단축어/launch로 먼저 실행되어 있어야 합니다.

아래 예시는 지도 이름이 `map`인 경우입니다. 다른 지도를 사용할 때는 MCL의 `map_name`과
Global/Local planning의 `F1_MAP`을 모두 같은 이름으로 변경해야 합니다.

### 터미널 1 — 센서 및 차량 드라이버

프로젝트에 설치된 f110 드라이버 실행 명령을 사용합니다. 이 단계에서 `/scan`, `/joy`, VESC IMU, odometry 등이 발행되어야 합니다.

```zsh
# 실제 장비에 맞는 f110 드라이버 단축어 또는 launch 실행
```

### 터미널 2 — MCL

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=0
PROFILE="$PWD/src/f1tenth_control/config/runtime_visualization.yaml"

ros2 launch particle_filter_cpp mcl_launch.py \
  mod:=real \
  map_name:=map \
  use_rviz:=true \
  publish_odom_base_tf:=true \
  runtime_profile:="$PROFILE"
```

대회 중 상태와 초기 위치를 계속 확인할 수 있도록 `use_rviz:=true`로 RViz를 함께 실행합니다.
SSH에서 실행할 때는 Jetson의 그래픽 세션에 접근할 수 있도록 `DISPLAY` 설정 또는 X11 전달이
필요합니다. RViz는 같은 ROS domain에 연결된 로컬 PC에서 별도로 실행해도 됩니다.

8월 8일 이전 실차 설정과 동일하게 MCL이 `map -> odom`과 `odom -> base_link`를 모두
발행하도록 `publish_odom_base_tf:=true`를 사용합니다. 터미널 1의 `vesc_to_odom_node`도
`odom -> base_link`를 발행하도록 설정되어 있다면 발행자가 중복되어 TF가 흔들릴 수 있으므로,
둘 중 한 노드만 해당 TF를 발행하도록 설정해야 합니다.

### 터미널 3 — Global planning

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=0
PROFILE="$PWD/src/f1tenth_control/config/runtime_visualization.yaml"

F1_MAP=map ros2 launch global_planning global_planning.launch.py \
  runtime_profile:="$PROFILE"
```

공통 프로파일은 Global planning의 MarkerArray/Lattice 출력만 제어하며 waypoint와 Frenet
odometry 데이터 출력은 유지합니다.

### 터미널 4 — Local planning

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=0
PROFILE="$PWD/src/f1tenth_control/config/runtime_visualization.yaml"

F1_MAP=map ros2 launch local_planning local_planning.launch.py \
  simulator:=true \
  runtime_profile:="$PROFILE"
```

### 터미널 5 — State machine

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=0
PROFILE="$PWD/src/f1tenth_control/config/runtime_visualization.yaml"

ros2 launch state_machine state_machine.launch.py \
  runtime_profile:="$PROFILE"
```

### 터미널 6 — 사용하지 않음

현재 `state_machine_node`가 `/local_waypoints`를 직접 발행하므로 별도 `wpnt_publisher`를
실행하지 않습니다. 패키지나 실행 파일이 디스크에 존재하는 것만으로 런타임 성능이 저하되지는
않습니다.

### 터미널 7 — 실차 제어

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source ~/f1tenth_ws/install/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=0
PROFILE="$PWD/src/f1tenth_control/config/runtime_visualization.yaml"

ros2 launch f1tenth_control control_real.launch.py \
  runtime_profile:="$PROFILE"
```

`control_real.launch.py`는 자체적으로 LiDAR, 조이스틱, VESC 드라이버를 실행하지 않습니다. 터미널 1의 드라이버가 먼저 실행되어야 합니다.

## 5. 실행 순서와 점검

권장 순서는 다음과 같습니다.

```text
센서/VESC 드라이버
  -> /scan, odom, IMU, /joy, TF 확인
  -> MCL
  -> global planning
  -> local planning
  -> state machine
  -> control_real
```

문제가 발생하면 먼저 다음을 확인합니다.

```zsh
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /pf/pose/odom
ros2 topic echo /drive_mode --once
ros2 topic info /tf --verbose
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_tools view_frames
```

`/scan` 또는 `/odom`이 없으면 MCL 문제가 아니라 센서 드라이버/ROS 네트워크 문제입니다.
`/pf/pose/odom`이 없으면 MCL의 지도 로딩, 초기화, `/scan`, `/odom` 입력을 확인합니다.
`odom -> base_link`는 터미널 1의 VESC odometry에서 발행자가 정확히 하나여야 합니다.
`/drive_mode`는 자율 모드에서 `autonomous`여야 제어기의 engage gate가 열립니다.

## 6. 시뮬레이터를 사용하는 경우

시뮬레이터는 Jetson 실차 드라이버 대신 별도 시뮬레이터 워크스페이스에서 실행합니다. 시뮬레이션 제어에는 다음 launch를 사용합니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
PROFILE="$PWD/src/f1tenth_control/config/runtime_visualization.yaml"

ros2 launch f1tenth_control control_sim.launch.py \
  runtime_profile:="$PROFILE"
```

시뮬레이션에서는 `control_real.launch.py`를 사용하지 않습니다. 실차용 `control_real.launch.py`는 VESC 하드웨어 명령 변환을 포함하기 때문입니다.
