# local_planning

글로벌 Race Line의 순서를 고정한 정적 장애물 회피 패키지입니다.
정적 장애물은 `obstacle_detector`의 `/static_obs`에서 map-frame Cartesian AABB로 받고, 중심
CLCS 투영으로 트랙 branch를 고정한 뒤 실제 Race Line 선분과 가장 가까운 AABB 면의 `|d|`를
blocking 판정에 사용합니다. 그 사각형의 종·횡 범위로 트랙 위상을 보존한 회피선을 만듭니다.
결과는 Cartesian `x_m/y_m`이 채워진 `/avoid_waypoints`로 발행합니다.

장애물이 나타나면 자유공간에서 새 경로를 검색하지 않습니다. 현재 글로벌 waypoint 구간을 그대로
선택하고 Frenet `d(s)`만 장애물 반대쪽으로 이동한 뒤 cubic spline을 맞춥니다. 따라서 스네이크처럼
서로 다른 트랙 조각이 지도상 가까이 붙어 있어도 다른 조각으로 경로가 점프하지 않습니다.
선택된 경로는 최신 AABB에도 안전한 동안 geometry를 그대로 유지하며, 안전정지는 즉시 latch하고
연속 안전 판정 뒤에만 해제해 perception 흔들림이 경로 모드 진동으로 전달되지 않게 합니다.
첫 장애물 군집은 짧은 준비 감속 동안 ID와 AABB 합집합을 안정화한 뒤 양쪽을 비교합니다. 실제
횡이동 전에는 새 장애물에 따라 방향을 다시 고를 수 있지만, 회피 진입 뒤에는 방향을 고정합니다.
첫 회피의 merge 뒤에 별도 blocking 군집이 있으면 중간에 global로 복귀하지 않고 준비 감속부터
새 maneuver를 이어서 생성하며, 이때 완료된 첫 maneuver의 방향 잠금은 다음 계획에 전달하지
않습니다.

핵심 동작, 토픽, 파라미터, 실행 및 검증 절차는
[`docs/local_planner.md`](docs/local_planner.md)에 단계별로 정리되어 있습니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch local_planning local_planning.launch.py
```

기본 출력은 `/avoid_waypoints`입니다. 최종 `/local_waypoints` 선택은 `/state`를 구독하는
`wpnt_publisher`가 담당하므로 `publish_standalone_local`의 기본값은 `false`입니다.
