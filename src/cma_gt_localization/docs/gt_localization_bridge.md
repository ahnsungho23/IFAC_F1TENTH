# GT localization bridge

## 목적

이 노드는 CMA planner tuning 시뮬레이션에서만 simulator의 실제 ego pose를 기존 MCL 출력 인터페이스로 연결한다. MCL 알고리즘과 실차 launch는 변경하지 않는다.

## 동작 원리

1. f1tenth gym bridge가 `/ego_racecar/odom`에 `map` 기준 `ego_racecar/base_link` pose를 발행한다.
2. 이 노드는 입력 timestamp와 pose를 그대로 복사해 `/pf/pose/odom`으로 발행한다.
3. 기본 `twist_mode: mcl_compatible`은 기존 particle filter 출력과 같이 `twist.linear.x`만 복사한다.
4. TF는 발행하지 않는다. simulator가 이미 `map -> ego_racecar/base_link -> ego_racecar/laser`를 소유하기 때문이다.

## 토픽과 메시지

| 방향 | 토픽 | 타입 | 용도 |
|---|---|---|---|
| 구독 | `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | simulator GT ego state |
| 발행 | `/pf/pose/odom` | `nav_msgs/msg/Odometry` | 기존 localization consumer 호환 출력 |

장애물 GT, scenario manifest, baked-map obstacle geometry는 입력하지 않는다. 장애물 pipeline은 계속 `/scan -> obstacle_detector -> /static_obs`만 사용한다.

## 주요 파라미터

설정 파일은 `config/gt_localization_bridge.yaml`이다.

- `input_topic`, `output_topic`: GT 입력과 MCL 호환 출력
- `expected_input_frame`, `expected_input_child_frame`: simulator frame 검증
- `output_frame`, `output_child_frame`: downstream 인터페이스 frame
- `reject_frame_mismatch`: 예상 frame과 다른 메시지를 폐기
- `twist_mode`: `mcl_compatible` 또는 `passthrough`
- `qos_depth`: reliable QoS queue 깊이

## 실행 방법

먼저 simulator를 실행한 뒤 다음을 실행한다.

```bash
ros2 launch cma_gt_localization gt_localization_bridge.launch.py
```

CMA runner에서는 `localization_mode: ground_truth`일 때만 이 launch를 사용한다. `localization_mode: mcl`이면 기존 `particle_filter_cpp/mcl_launch.py`가 실행된다. 두 localization provider를 동시에 실행하면 안 된다.

