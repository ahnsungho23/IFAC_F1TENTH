# local_planning

글로벌 Race Line의 순서를 고정한 정적 장애물 회피 패키지입니다.
정적 장애물은 `/perception/static_obstacles/cartesian`의 map-frame 중심 `(x,y)`와 원 반지름으로 받고, CLCS를 이용해
트랙 위상을 보존한 회피선을 만든 뒤 Cartesian `x_m/y_m`이 채워진 `/avoid_waypoints`를 발행합니다.

장애물이 나타나면 자유공간에서 새 경로를 검색하지 않습니다. 현재 글로벌 waypoint 구간을 그대로
선택하고 Frenet `d(s)`만 장애물 반대쪽으로 이동한 뒤 cubic spline을 맞춥니다. 따라서 스네이크처럼
서로 다른 트랙 조각이 지도상 가까이 붙어 있어도 다른 조각으로 경로가 점프하지 않습니다.

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
