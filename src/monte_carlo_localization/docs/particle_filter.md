# Monte Carlo Localization (particle_filter_node) 문서

## 1. 노드 개요 (Node Purpose)

`particle_filter_node`는 Monte Carlo Localization(MCL) 파티클 필터를 기반으로 2차원 점유 격자 지도(Occupancy Grid Map) 상에서 차량의 위치(Pose: x, y, yaw)를 추정하는 C++ ROS 2 노드입니다.
실시간 라이다 스캔 정보와 오도메트리, 글로벌 트래젝터리 시작 포즈를 통합하여 `map -> odom` TF 변환 및 차량 추정 포즈(`/pf/pose/odom`)를 발행합니다.

---

## 2. 동작 원리 (Operating Principle)

1. **파티클 필터링 (Resampling & Motion & Sensor Model)**:
   - 차량의 동작 모델(Bicycle kinematics motion model)에 기반하여 파티클들을 이동시킵니다.
   - 라이다 스캔 데이터와 맵 상의 Raycasting을 비교하여 4-component 빔 센서 모델 기반으로 파티클 가중치를 계산하고 다항 리샘플링(Multinomial Resampling)을 실행합니다.

2. **자동 시작 위치 정합 (Auto Initial Pose Seeding)**:
   - `/global_waypoints` 토픽 수신 시 첫 번째 웨이포인트(Start Line Pose)를 차량 출발 위치로 자동 인식하여 파티클 분포를 시작 포즈로 초기화하고 빠른 수렴(Fast Convergence) 모드를 활성화합니다.
   - 이로 인해 RViz 기동 후 수동으로 2D Pose Estimate를 클릭하지 않아도 라이다 스캔 위치가 맵/글로벌 패스 시작점에 자동으로 맞춰집니다.

3. **Pose Fusion EKF (출력단 융합, 2026-07-29 추가)**:
   - 휠 odom 포즈 델타(laser 프레임 변환 포함)로 매 주기(30 Hz) 포즈를 예측하고, MCL 기대
     포즈를 측정으로 보정합니다. 측정 노이즈 R은 파티클 가중 공분산에 **차체 종방향만
     25배 불신**을 더한 값 — 평행벽 복도에서 라이다가 진행방향을 관측하지 못해 생기던
     종방향 표류(0.8~1.7 m/랩)를 차단하고, 코너에서 회전된 잔여 오차를 고게인으로 잡습니다.
   - 마할라노비스 게이트로 MCL 순간 글리치를 걸러내고, 연속 기각이 길어지면 MCL로
     재고정하는 안전망이 있습니다. `use_pose_ekf: false`면 구 EMA 스무딩으로 돌아갑니다.

4. **TF 및 Odometry 발행**:
   - 추정된 포즈를 바탕으로 `map -> odom` TF 트랜스폼을 브로드캐스팅합니다.

---

## 3. 구독 및 발행 토픽 (Topics & Message Types)

### Subscribed Topics
- `/scan` (`sensor_msgs/msg/LaserScan`): 라이다 센서 측정 데이터
- `/odom` (`nav_msgs/msg/Odometry`): 차량 바퀴/추정 오도메트리 데이터
- `/global_waypoints` (`f110_msgs/msg/WpntArray`): 글로벌 경로 웨이포인트 (자동 위치 초기화용)
- `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): RViz 2D Pose Estimate 수동 초기화 포즈

### Published Topics
- `/pf/pose/odom` (`nav_msgs/msg/Odometry`): MCL 최종 추정 포즈 오도메트리
- `/pf/viz/inferred_pose` (`geometry_msgs/msg/PoseStamped`): 시각화용 추정 포즈
- `/pf/viz/particles` (`geometry_msgs/msg/PoseArray`): RViz 파티클 군집 시각화
- `/map` (`nav_msgs/msg/OccupancyGrid`): 맵 서버로부터 전달받은 지도 정보 재발행

---

## 4. 주요 파라미터 및 YAML 위치 (Main Parameters & YAML)

- **YAML 파일 위치**: `config/mcl_config.yaml`
- **주요 파라미터**:
  - `auto_init_from_waypoints` (`bool`, 기본값: `true`): `/global_waypoints` 시작 포즈 기반 자동 초기 포즈 세팅 활성화 여부
  - `max_particles` (`int`, 기본값: `1000`): 파티클 개수
  - `scan_topic` (`string`, 기본값: `/scan`): 라이다 스캔 토픽 이름
  - `odom_topic` (`string`, 기본값: `/odom`): 오도메트리 토픽 이름
  - `publish_map_odom_tf` (`bool`, 기본값: `true`): `map -> odom` TF 발행 여부
  - `use_pose_ekf` (`bool`, 기본값: `true`): 출력단 pose fusion EKF 활성화 (false = 구 EMA)
  - `ekf_trans_error_rate` (`double`, 기본값: `0.01`): 주행거리 대비 휠 odom 병진 오차율
  - `ekf_meas_long_inflation` (`double`, 기본값: `25.0`): 차체 종방향 측정 불신 배율
  - 나머지 EKF/스무딩 파라미터는 `mcl_config.yaml`의 주석 참고

---

## 5. 실행 방법 및 launch 예시 (How to Run & ros2 launch)

### 빌드 명령어
```bash
colcon build --packages-select particle_filter_cpp
```

### 실행 명령어 (ros2 launch)
- **실차 모드**:
  ```bash
  export F1_MAP=map
  ros2 launch particle_filter_cpp mcl_launch.py mod:=real map_name:=map
  ```
- **시뮬레이션 모드**:
  ```bash
  ros2 launch particle_filter_cpp mcl_launch.py mod:=sim map_name:=map use_rviz:=true
  ```
