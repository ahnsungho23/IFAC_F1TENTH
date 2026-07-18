# 로컬 플래닝 패키지 (`local_planning`)

본 패키지는 IFAC 2026 자율주행 레이싱 대회 차량을 위한 2가지 로컬 플래너 노드를 제공합니다.

1. **`static_obstacle_avoidance_node`** (신규 구현): 인지(Comception) 모듈의 정적 장애물 정보(`/obstacles`) 및 Frenet 좌표계를 이용한 5차 다항식(Quintic Spline) 기반 선택적 회피 세그먼트 생성 플래너.
2. **`local_planner_node`** (기존 구현): 점유 격자 지도(`/map`) 기반 가중 최소자승법(Least-Squares) 3차 다항식 스플라인 플래너.

---

## 1. 정적 장애물 회피 플래너 (`static_obstacle_avoidance_node`) 핵심 구현 사항

본 노드는 차량 주행 중 인지(Comception) 모듈로부터 전방 정적 장애물을 감지했을 때, 상황을 자율적으로 판단하여 부드러운 회피 궤적 세그먼트를 생성하고 제어기에 전달합니다. 다음 5가지 요구사항을 완벽히 만족합니다.

### ① 세그먼트 단위 경로 발행 (Segment Path Publishing)
- 전체 트랙을 매번 재생성하지 않고, 차량 현재 위치($s_{\text{car}}$)부터 정적 장애물 회피 기동을 거쳐 글로벌 패스로 수렴하는 지점($s_{\text{end}}$)까지의 **로컬 웨이포인트 세그먼트(`f110_msgs::msg::OTWpntArray`)** 만 발행합니다.

### ② 글로벌 패스로의 $C^2$ 수렴 보장 (Global Path Convergence)
- 회피 기동 후 경로 끝부분이 기존 글로벌 레이스라인과 완벽히 수렴하도록 **2단계 5차 다항식(Quintic Polynomial Spline)** 을 사용합니다.
- 궤적 끝점의 경계조건을 글로벌 경로 중심선 $(d_f, d'_f, d''_f) = (0, 0, 0)$ 으로 강제하여 **위치($C^0$), 기울기($C^1$), 곡률($C^2$) 연속성**을 보장합니다.

### ③ 곡률 기반 스로틀(속도) 프로파일 동적 조정 (Throttle Adjustment)
- 회피 곡률 $\kappa(s)$ 로 인한 허용 최대 횡가속도 $a_{\text{lat,max}}$ 초과 및 슬립을 방지하기 위해 각 웨이포인트의 목표 속도를 자동 조율합니다.
  $$v_x(s) = \min\left(v_{\text{global}}(s), \sqrt{\frac{a_{\text{lat,max}}}{|\kappa(s)| + \epsilon}}\right)$$

### ④ 상황 판단에 따른 선택적 로컬패스 발행 (Conditional Publishing)
- 로컬 플래너가 스스로 전방 상황을 판단합니다.
- 경로 차단 장애물이 없거나 장애물이 주행 코리도(`corridor_width`) 밖으로 멀리 떨어져 있어 **글로벌 패스를 유지해도 안전한 경우, 로컬 패스 발행을 생략(또는 빈 경로 발행)**하여 `wpnt_publisher`가 글로벌 패스(`/global_waypoints`)를 유지하도록 합니다.

### ⑤ 다중 후보 경로 생성 및 비용함수(Cost Function) 평가
- 장애물 좌/우 회피 오프셋($d_{\text{target}}$) 및 복귀 길이 조합으로 다중 후보 궤적 $P_i(s)$ 를 생성합니다.
- 안전성, 횡편차, 평활도를 평가하는 총 비용함수 $J_{\text{total}} = w_{\text{obs}} J_{\text{obs}} + w_{\text{lat}} J_{\text{lat}} + w_{\text{smooth}} J_{\text{smooth}}$ 를 통해 최적 경로 $P^*$ 를 선택합니다.

---

## 2. 전체 시스템 데이터 플로우 (`static_obstacle_avoidance_node`)

```
       [ Comception (LiDAR 인지) ]
                   │
                   ▼  /obstacles (f110_msgs/msg/ObstacleArray)
┌──────────────────┴─────────────────────────────────────────────────┐
│               static_obstacle_avoidance_node (본 노드)               │
│                                                                     │
│  1. 전방 차단 정적 장애물 판단 (코리도 침범 여부)                         │
│  2. 차단 장애물 없음 -> 로컬패스 발행 생략 (글로벌패스 유지)                 │
│  3. 차단 장애물 감지 -> 5차 다항식 회피 세그먼트 생성 + 비용함수 최적화      │
└──────────────────┬─────────────────────────────────────────────────┘
                   │ /planner/avoidance/otwpnts (f110_msgs/msg/OTWpntArray)
                   ▼
       [ 웨이포인트 퍼블리셔 (wpnt_publisher) ]
                   │ /local_waypoints (f110_msgs/msg/WpntArray)
                   ▼
       [ MAP 컨트롤러 (new_map_con : Pure Pursuit L1) ]
                   │ /drive (AckermannDriveStamped)
                   ▼
            [ VESC MK VI 구동 제어 ]
```

---

## 3. 토픽 인터페이스 (`static_obstacle_avoidance_node`)

### 구독 토픽 (Subscriptions)
| 토픽 이름 | 메시지 타입 | 설명 |
|---|---|---|
| `/global_waypoints` | `f110_msgs/msg/WpntArray` | 글로벌 레이스라인 기준 웨이포인트 |
| `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | 차량의 Frenet 좌표계 상태 ($s, d, v_s, v_d$) |
| `/obstacles` | `f110_msgs/msg/ObstacleArray` | Comception 감지 장애물 목록 |

### 발행 토픽 (Publications)
| 토픽 이름 | 메시지 타입 | 설명 |
|---|---|---|
| `/planner/avoidance/otwpnts` | `f110_msgs/msg/OTWpntArray` | `wpnt_publisher`로 전송되는 회피 세그먼트 |
| `/local_planning/avoidance_path` | `nav_msgs/msg/Path` | 최종 선정된 회피 경로 (RViz 시각화용) |
| `/local_planning/candidate_paths` | `visualization_msgs/msg/MarkerArray` | 생성된 후보 경로 목록 (RViz 시각화용) |

---

## 4. 파라미터 및 실행 명령어

### 파라미터 (`config/local_planning.yaml`)
`static_obstacle_avoidance_node` 및 `local_planner_node` 두 노드의 파라미터가 모두 정의되어 있습니다.
- `planning_frequency`: 플래너 실행 주기 (기본: 20.0 Hz)
- `lookahead_distance`: 전방 장애물 탐색 거리 (기본: 15.0 m)
- `corridor_width`: 차량 주행 코리도 폭 (기본: 0.65 m)
- `safety_margin`: 안전 여유 마진 (기본: 0.28 m)
- `max_lat_accel`: 최대 허용 횡가속도 (기본: 6.0 m/s²)
- `weight_obs`, `weight_lat`, `weight_smooth`: 비용함수 가중치

### 빌드 명령어
```bash
colcon build --packages-select local_planning --symlink-install
```

### 실행 명령어
1. **정적 장애물 회피 노드 (`static_obstacle_avoidance_node`) 실행**:
   ```bash
   ros2 launch local_planning static_obstacle_avoidance.launch.py
   ```
2. **격자 지도 기반 기존 로컬 플래너 (`local_planner_node`) 실행**:
   ```bash
   ros2 launch local_planning local_planning.launch.py
   ```
