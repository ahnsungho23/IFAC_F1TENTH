# lap_timer 노드

## 목적과 동작

`lap_timer`는 `/car_state/frenet/odom`의 Frenet `s` 래핑을 감지해 랩타임을 계산한다.
`/drive`에서 속도와 조향 정보를 받아 선택적인 RViz HUD를 갱신한다. 차량 제어에는 관여하지
않는 측정·표시 전용 노드다.

## 토픽

- 구독: `/car_state/frenet/odom` (`nav_msgs/msg/Odometry`)
- 구독: `/drive` (`ackermann_msgs/msg/AckermannDriveStamped`)
- 발행: `/lap_time`, `/best_lap_time` (`std_msgs/msg/Float64`)
- 발행: `/speed`, `/steer` (`std_msgs/msg/Float32`)
- 선택 발행: `/lap_time_text` (`visualization_msgs/msg/Marker`)
- 선택 발행: `/lap_hud` (`rviz_2d_overlay_msgs/msg/OverlayText`, 설치된 경우)

## 파라미터와 실행

기본 파라미터는 `config/params.yaml`에 있다. 공통
`f1tenth_control/config/runtime_visualization.yaml`의 `lap_timer.ros__parameters.rviz_text`가
마지막에 적용된다. `publish_hud=false`이면 `/lap_hud`와 HUD timer를 생성하지 않는다.
일반 숫자 토픽 `/speed`, `/steer`는 유지한다.

```bash
ros2 launch lap_timer lap_timer.launch.py use_rviz:=false
```

RViz 표시가 필요할 때 공통 프로파일의 `rviz_text`를 `true`로 바꾸고 다음처럼 실행한다.

```bash
ros2 launch lap_timer lap_timer.launch.py use_rviz:=true
```
