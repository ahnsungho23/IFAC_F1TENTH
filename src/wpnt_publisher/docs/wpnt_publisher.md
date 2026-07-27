# wpnt_publisher 노드

## 목적

`wpnt_publisher`는 주행 상태에 맞는 경로를 골라 제어기에 전달할
`/local_waypoints`와 RViz용 `/local_waypoints/path`를 발행합니다.

## 동작 원리

1. `STATE_GLOBAL`에서는 현재 Frenet 인덱스 다음의 글로벌 웨이포인트를 선택합니다.
2. `STATE_AVOID`에서는 TTL 안의 정적 회피 경로를 선택합니다.
3. `STATE_OVERTAKE`에서는 기존 동적 추월 경로를 선택합니다.
4. 회피 경로가 없을 때는 글로벌 웨이포인트를 선택합니다.
5. 이 노드는 안전 정지 글로벌 패스를 생성하거나 발행하지 않습니다.

## 토픽과 메시지

| 구분 | 토픽 | 메시지 |
|---|---|---|
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` |
| 구독 | `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` |
| 구독 | `/overtake_waypoints` | `f110_msgs/msg/OTWpntArray` |
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` |
| 구독 | `/state` | `f110_msgs/msg/StateMachine` |
| 발행 | `/local_waypoints` | `f110_msgs/msg/WpntArray` |
| 발행 | `/local_waypoints/path` | `nav_msgs/msg/Path` |

## 주요 파라미터

- YAML: `config/wpnt_publisher.yaml`
- `waypoint_num`: 출력할 전방 웨이포인트 수
- `avoid_path_ttl_sec`: 마지막 정적 회피 경로의 최대 사용 시간
- 글로벌·정적 회피·오도메트리·상태·출력 토픽 이름도 YAML에서 바꿀 수 있습니다.

## 실행

```bash
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch wpnt_publisher wpnt_publisher.launch.py
```

상태와 출력 속도를 확인합니다.

```bash
ros2 topic echo /state --once
ros2 topic echo /local_waypoints --once
```
