# lap_timer 노드

## 1. 목적

`lap_timer`는 Frenet 오도메트리의 종방향 좌표 `s`가 시작선을 통과하는 시점을 검출해 현재 랩과
최고 랩타임을 계산하고 RViz HUD로 표시하는 노드입니다.

## 2. 동작 원리

1. `/car_state/frenet/odom`의 `pose.pose.position.x`를 기본 `s` 값으로 사용합니다.
2. `s`가 트랙 끝에서 시작 구간으로 wrap되거나 exact-zero 조건을 통과하면 랩 완료 후보로 봅니다.
3. `min_lap_time`보다 짧은 후보는 노이즈로 무시합니다.
4. `/drive`의 속도와 조향각을 받아 HUD와 진단 토픽에 함께 표시합니다.

## 3. 토픽과 메시지

| 구분 | 기본 토픽 | 메시지 |
|---|---|---|
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` |
| 구독 | `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` |
| 발행 | `/lap_time` | `std_msgs/msg/Float64` |
| 발행 | `/best_lap_time` | `std_msgs/msg/Float64` |
| 발행 | `/lap_hud` | `rviz_2d_overlay_msgs/msg/OverlayText` |
| 발행 | `/speed`, `/steer` | `std_msgs/msg/Float32` |

## 4. 파라미터

파라미터 파일은 `config/params.yaml`입니다. 주요 항목은 `odom_topic`, `drive_topic`,
`drive_msg_type`, `start_window`, `wrap_threshold`, `min_lap_time`, `exact_zero_mode`,
`hud_rate`, `speed_unit`입니다.

## 5. 실행

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch lap_timer lap_timer.launch.py
```

GUI가 없는 환경에서는 RViz를 끄고 실행합니다.

```bash
ros2 launch lap_timer lap_timer.launch.py use_rviz:=false
```

랩타임 발행을 확인합니다.

```bash
ros2 topic echo /lap_time --once
```
