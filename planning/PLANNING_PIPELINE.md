# Planning/Control Pipeline Architecture

이 문서는 현재 레포지토리 기준으로 planning-control 데이터 흐름을 정리한 아키텍처 메모입니다.

## 1) End-to-End Flow

```text
[Localization]
monte_carlo_localization
  ↓
/pf/pose/odom
nav_msgs/msg/Odometry

[Global Planning]
global_planner_node (C++)
  입력:
    /map
    /pf/pose/odom
  처리:
    occupancy grid 처리
    centerline 추출
    boundary 계산
    minimum curvature trajectory 생성
  출력:
    /global_waypoints
    global_waypoints.json

[Global Waypoint Republish]
global_trajectory_publisher_node (C++)
  입력:
    global_waypoints.json
  출력:
    /global_waypoints

[Frenet Index]
frenet_odom_node (C++)
  입력:
    /pf/pose/odom
    /global_waypoints
  처리:
    현재 차량과 가장 가까운 global waypoint index 계산
  출력:
    /car_state/frenet/odom

[Waypoint Slicing]
wpnt_publisher
  입력:
    /global_waypoints
    /car_state/frenet/odom
  처리:
    현재 위치 기준으로 앞쪽 waypoint 일부 선택
  출력:
    /local_waypoints

[Control]
new_map_con / MAP_controller.py
  입력:
    /global_waypoints
    /local_waypoints
  처리:
    steering, speed 계산
  출력:
    drive command
```

## 2) Node Responsibilities

### `monte_carlo_localization`
- 역할: 차량 localization
- 출력: `/pf/pose/odom` (`nav_msgs/msg/Odometry`)

### `global_planner_node` (C++)
- 역할: 맵 기반 global trajectory 생성
- 입력:
  - `/map`
  - `/pf/pose/odom`
- 처리:
  - occupancy grid 처리
  - centerline 추출
  - boundary 계산
  - minimum curvature trajectory 생성
- 출력:
  - `/global_waypoints`
  - `global_waypoints.json`
- 구현 예시:
  - 소스: `planning/src/global_planner_node.cpp`
  - 헤더: `planning/include/planning/global_planner_node.hpp`
  - 실행 파일: `global_planner_node`

### `global_trajectory_publisher_node` (C++)
- 역할: 저장된 global trajectory 재배포
- 입력:
  - `global_waypoints.json`
- 출력:
  - `/global_waypoints`
- 구현 예시:
  - 소스: `planning/src/global_trajectory_publisher_node.cpp`
  - 헤더: `planning/include/planning/global_trajectory_publisher_node.hpp`
  - 실행 파일: `global_trajectory_publisher_node`

### `frenet_odom_node` (C++)
- 역할: 차량의 현재 global waypoint 인덱스 생성
- 입력:
  - `/pf/pose/odom`
  - `/global_waypoints`
- 처리:
  - 현재 차량과 가장 가까운 global waypoint index 계산
- 출력:
  - `/car_state/frenet/odom`
- 구현 예시:
  - 소스: `planning/src/frenet_odom_node.cpp`
  - 헤더: `planning/include/planning/frenet_odom_node.hpp`
  - 실행 파일: `frenet_odom_node`

### `wpnt_publisher`
- 역할: global waypoint를 local waypoint 구간으로 슬라이싱
- 입력:
  - `/global_waypoints`
  - `/car_state/frenet/odom`
- 처리:
  - 현재 위치 기준 앞쪽 waypoint 일부 선택
- 출력:
  - `/local_waypoints`

### `new_map_con / MAP_controller.py`
- 역할: 추종 제어
- 입력:
  - `/global_waypoints`
  - `/local_waypoints`
- 처리:
  - steering, speed 계산
- 출력:
  - drive command

## 3) Interface Notes

- `global_planner_node`와 `global_trajectory_publisher_node`는 동시에 `/global_waypoints`를 publish하지 않도록 운용 모드를 분리하는 것을 권장합니다.
- `frenet_odom_node.py`의 출력은 `wpnt_publisher`가 기대하는 인덱스 포맷(closest waypoint index)과 일치해야 합니다.
- control 안정성을 위해 `/global_waypoints`와 `/local_waypoints`의 frame/QoS 정책을 일관되게 유지하는 것이 중요합니다.
