# 실차 시각화 출력 통합 제어

## 1. 목적

`runtime_visualization.yaml` 한 파일에서 2026_IFAC 주행 스택의 RViz·디버그 출력을
일괄 제어한다. 연결된 launch는 패키지 고유 YAML을 먼저 읽고 이 파일을 마지막에 읽는다.
따라서 경로 계획·제어 튜닝값은 유지되고, 이 파일에 명시된 출력 스위치만 덮어쓴다.

파일 위치:

```text
src/f1tenth_control/config/runtime_visualization.yaml
```

설치 후 위치 확인:

```bash
ros2 pkg prefix f1tenth_control
```

## 2. 실차 경량 설정

저장소 기본 프로파일은 다음 출력을 끈다.

- MCL particle pose와 particle cloud
- MCL의 5 Hz `/map` 미러
- obstacle MarkerArray
- `/local_planning/path`
- `/local_waypoints/path`
- `/debug/l1_lookahead`
- 선택 노드의 static-map·map-controller·lap-timer 시각화

`/pf/pose/odom`, `/global_waypoints`, `/static_obs`, `/avoid_waypoints`, `/state`,
`/local_waypoints`, `/drive_autonomous`, `/drive` 같은 주행 토픽은 유지된다.

`global_planning` 시각화 출력은 이번 변경 범위에서 제외했으며 기존 동작을 유지한다.

디버깅할 때는 필요한 항목만 `true`로 바꾸고 관련 노드를 재시작한다. 노드들은 이 스위치를
기동 시 읽으므로 파일 편집만으로 실행 중인 프로세스가 동적으로 바뀌지는 않는다.

## 3. 공통 실행 준비

각 터미널에서 다음을 먼저 실행한다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
PROFILE="$PWD/src/f1tenth_control/config/runtime_visualization.yaml"
```

launch 인자를 생략해도 설치된 기본 프로파일을 자동으로 읽지만, 아래처럼 명시하면 실제로
어느 파일을 적용하는지 분명하고 소스 YAML 편집 결과를 재빌드 없이 시험할 수 있다.

## 4. 시뮬레이션 실행 명령

시뮬레이터 자체는 기존 gym bridge 명령으로 먼저 실행한다. 이후 저장소 노드는 다음 순서로
실행한다. 별도 `wpnt_publisher`는 실행하지 않는다.

```bash
ros2 launch particle_filter_cpp mcl_launch.py \
  mod:=sim map_name:=ifac_track use_rviz:=false runtime_profile:="$PROFILE"
```

```bash
F1_MAP=ifac_track ros2 launch global_planning global_planning.launch.py
```

```bash
F1_MAP=ifac_track ros2 launch local_planning local_planning.launch.py \
  simulator:=true runtime_profile:="$PROFILE"
```

```bash
ros2 launch state_machine state_machine.launch.py runtime_profile:="$PROFILE"
```

```bash
ros2 launch f1tenth_control control_sim.launch.py runtime_profile:="$PROFILE"
```

상대차가 필요할 때만 다음 명령을 추가한다. 이 노드는 RViz 출력을 만들지 않으므로 공통
프로파일 인자가 필요 없다.

```bash
ros2 launch new_map_con opponent_simulator.launch.py
```

## 5. 실차 실행 명령

외부 `f1tenth_stack` 하드웨어 bringup 이후 다음 순서로 실행한다.

```bash
ros2 launch particle_filter_cpp mcl_launch.py \
  mod:=real map_name:=map use_rviz:=false runtime_profile:="$PROFILE"
```

```bash
F1_MAP=map ros2 launch global_planning global_planning.launch.py
```

```bash
F1_MAP=map ros2 launch local_planning local_planning.launch.py \
  simulator:=false runtime_profile:="$PROFILE"
```

```bash
ros2 launch state_machine state_machine.launch.py runtime_profile:="$PROFILE"
```

```bash
ros2 launch f1tenth_control control_real.launch.py runtime_profile:="$PROFILE"
```

## 6. 적용 확인

경량 프로파일에서는 다음 토픽이 없어야 한다.

```bash
ros2 topic list | grep -E '/pf/viz|/(static_obs|opp_obs|adaptive_obstacle_map)/markers$|/local_.*/path|/debug/l1_lookahead|/lap_(time_text|hud)$'
```

핵심 토픽은 유지되어야 한다.

```bash
ros2 topic list | grep -E '/pf/pose/odom|/global_waypoints$|/static_obs$|/avoid_waypoints$|/state$|/local_waypoints$|/drive$'
```

MCL의 `publish_map: false`는 particle filter의 반복 `/map` 미러만 끈다. MCL launch의
`nav2_map_server`가 제공하는 latched `/map`은 남을 수 있다. `/map`의 실제 publisher는 다음으로
확인한다.

```bash
ros2 topic info -v /map
```
