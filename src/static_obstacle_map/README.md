# static_obstacle_map

Confirmed 정적 장애물을 주행 동안 map 좌표로 기억하고, 저장된 `Obstacle.msg` 전체를
`/adaptive_obstacle_map` (`f110_msgs/msg/ObstacleArray`)으로 발행하는 ROS 2 Jazzy C++
패키지다. 저장 장애물은 RViz용 `/adaptive_obstacle_map/markers`
(`visualization_msgs/msg/MarkerArray`)로도 발행한다.

장애물 형상은 최초 confirmed AABB를 보존하고, 연속 확인된 외곽 경계만 bounded union으로
추가한다. 따라서 통과 뒤의 부분 scan이 앞에서 저장한 형상을 덮어쓰지 않으며, 최대 대각선
상한을 넘는 확장은 기존 형상을 움직이지 않고 거부한다. 기본 설정에서는 detector의 일시적인
static/dynamic 분류 진동도 confirmed 저장 항목을 삭제하지 않는다.

이 패키지는 `/map`을 구독하지 않으며 `nav_msgs/msg/OccupancyGrid`를 합성하거나 발행하지 않는다.

설정과 실행 절차는 [docs/static_obstacle_map_node.md](docs/static_obstacle_map_node.md)를 참고한다.
