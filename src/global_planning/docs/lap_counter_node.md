# lap_counter_node

## 1. 노드 목적

`lap_counter_node`는 `/car_state/frenet/odom`의 Frenet `s`가 트랙 끝 구간에서 시작 구간으로
랩핑되는 순간을 검출한다. 검출할 때마다 완주 랩 수를 1 증가시키고 `/lap_count`로 발행한다.

## 2. 동작 원리

1. `nav_msgs/msg/Odometry.pose.pose.position.x`에서 현재 `s`를 읽는다.
2. 첫 번째 정상 샘플은 이전 `s`와 기준 시각을 초기화하는 데만 사용한다.
3. 아래 조건을 모두 만족하면 한 랩이 끝난 것으로 판정한다.
   - 이전 `s >= finish_s_min`
   - 현재 `s <= start_s_max`
   - 직전 랩핑 관측 후 `min_lap_time_sec` 이상 경과
4. 랩 수를 1 증가시키고 `std_msgs/msg/Int32`로 `/lap_count`에 발행한다.

`/lap_count` publisher는 reliable + transient-local QoS를 사용한다. 따라서 같은 durability로
구독하는 늦은 subscriber도 가장 최근 랩 수를 받을 수 있다. 노드 시작 시 초기값 `0`도 한 번 발행한다.

`finish_s_min`은 트랙의 마지막 `s`보다 조금 작은 값으로, `start_s_max`는 출발선 직후 구간으로
설정한다. 두 임계값을 멀리 두면 출발선 주변의 `s` 노이즈로 중복 카운트되는 것을 막을 수 있다.

## 3. 구독 및 발행 토픽

| 방향 | 토픽 기본값 | 메시지 타입 | 사용 필드 |
| --- | --- | --- | --- |
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | `pose.pose.position.x = s` |
| 발행 | `/lap_count` | `std_msgs/msg/Int32` | `data = 완료한 랩 수` |

## 4. 주요 파라미터

파라미터 파일은 `src/global_planning/config/global_planning.yaml`이다.

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `frenet_odom_topic` | `/car_state/frenet/odom` | Frenet odometry 입력 토픽 |
| `lap_count_topic` | `/lap_count` | 현재 랩 수 출력 토픽 |
| `finish_s_min` | `10.0` | 랩핑 직전 이전 `s`의 최솟값 [m] |
| `start_s_max` | `0.5` | 랩핑 직후 현재 `s`의 최댓값 [m] |
| `min_lap_time_sec` | `3.0` | 연속 랩핑 카운트 사이의 최소 시간 [s] |
| `initial_lap_count` | `0` | 노드 시작 시 랩 수 |

`finish_s_min`은 반드시 `start_s_max`보다 커야 한다. `min_lap_time_sec`은 MCL 또는 Frenet
projection의 순간적인 점프가 짧은 시간 안에 여러 랩으로 계산되는 것을 억제한다.

## 5. 빌드 및 실행

```zsh
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
colcon build --packages-select global_planning
source install/setup.zsh
ros2 launch global_planning global_planning.launch.py
```

`global_planning.launch.py`는 trajectory publisher와 Frenet odometry 변환기만 실행한다.
lap counter는 **state_machine과 함께 기동**된다 (`state_machine.launch.py`가
`lap_counter.launch.py`를 include). lap counter만 단독으로 실행하려면 다음 launch를 사용한다.

```zsh
ros2 launch global_planning lap_counter.launch.py
```

다른 터미널에서 현재 랩 수를 확인한다.

```zsh
source /opt/ros/humble/setup.zsh
source ~/2026_IFAC/install/setup.zsh
ros2 topic echo /lap_count --qos-durability transient_local
```

입력 `s`가 정상적으로 랩핑되는지는 다음 명령으로 확인한다.

```zsh
ros2 topic echo /car_state/frenet/odom --field pose.pose.position.x
```

임계값을 바꾸려면 `config/global_planning.yaml`의 `lap_counter_node.ros__parameters`를 수정한 뒤
노드를 다시 실행한다.
