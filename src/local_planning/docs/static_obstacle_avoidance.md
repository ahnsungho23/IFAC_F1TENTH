# 정적 장애물 회피 로컬 플래너 (`static_obstacle_avoidance_node`)

## 1. 노드 목적 (Purpose)
`static_obstacle_avoidance_node`는 인지(Comception) 모듈에서 전방 정적 장애물을 감지했을 때, Frenet 좌표계 상에서 부드러운 회피 궤적 세그먼트를 생성하고 최적의 경로를 선정하여 발행하는 ROS 2 C++ 노드입니다.

## 2. 핵심 동작 원리 (Operating Principle)

1. **상황 판단 및 로컬패스 선택적 발행**
   - 전방 탐색 거리 내에 차량 주행 코리도를 차단하는 정적 장애물이 없을 경우, 로컬 회피 경로를 생성하지 않고 글로벌 패스가 유지되도록 불필요한 발행을 중단(또는 비활성화 빈 경로 발행)합니다.
2. **5차 다항식(Quintic Polynomial) 보간 및 글로벌 경로 수렴**
   - 회피를 수행할 때는 차량의 현재 위치 및 횡편차/속도 조건을 출발점으로 하고, 장애물 우회 후 글로벌 경로 중심선($(d, d', d'') = (0, 0, 0)$)에 부드럽게 수렴하는 세그먼트를 생성합니다.
3. **비용함수(Cost Function) 평가**
   - 다중 후보 경로에 대해 **안전성 비용($J_{\text{obs}}$)**, **횡방향 편차 비용($J_{\text{lat}}$)**, **경로 평활도 비용($J_{\text{smooth}}$)**을 합산하여 충돌 없는 최소 비용 경로를 선택합니다.
4. **횡가속도 기반 스로틀(속도) 프로파일 조정**
   - 회피 곡률 $\kappa(s)$ 에 따라 최대 허용 횡가속도 $a_{\text{lat,max}}$를 넘지 않도록 목표 속도 $v_x(s) = \min(v_{\text{global}}(s), \sqrt{a_{\text{lat,max}} / |\kappa(s)|})$로 자동 감속합니다.

## 3. 구독 및 발행 토픽 (Subscribed & Published Topics)

| 방향 | 토픽 명 | 메시지 타입 | 설명 |
|---|---|---|---|
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` | 글로벌 레퍼런스 레이스라인 |
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | 차량 Frenet 좌표계 상태 ($s, d, v_s, v_d$) |
| 구독 | `/obstacles` | `f110_msgs/msg/ObstacleArray` | Comception 감지 장애물 목록 |
| 발행 | `/planner/avoidance/otwpnts` | `f110_msgs/msg/OTWpntArray` | `wpnt_publisher`로 전송되는 회피 세그먼트 |
| 발행 | `/local_planning/avoidance_path` | `nav_msgs/msg/Path` | RViz 궤적 시각화 |
| 발행 | `/local_planning/candidate_paths` | `visualization_msgs/msg/MarkerArray` | 후보 경로 시각화 |

## 4. 파라미터 파일 (`config/local_planning.yaml`)

- `planning_frequency`: 플래너 실행 주기 (기본: 20.0 Hz)
- `lookahead_distance`: 정적 장애물 전방 탐색 범위 (기본: 15.0 m)
- `corridor_width`: 주행 코리도 폭 (기본: 0.65 m)
- `safety_margin`: 장애물 경계 여유 마진 (기본: 0.28 m)
- `max_lat_accel`: 허용 최대 횡가속도 (기본: 6.0 m/s²)
- `weight_obs`, `weight_lat`, `weight_smooth`: 비용함수 가중치

## 5. 실행 방법

```bash
ros2 launch local_planning static_obstacle_avoidance.launch.py
```
