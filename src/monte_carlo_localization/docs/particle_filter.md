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
   - 휠 odom 포즈 델타(laser 프레임 변환 포함)로 매 주기(real/bag 40 Hz, sim 30 Hz) 포즈를 예측하고, MCL 기대
     포즈를 측정으로 보정합니다. 측정 노이즈 R은 파티클 가중 공분산에 **차체 종방향만
     25배 불신**을 더한 값 — 평행벽 복도에서 라이다가 진행방향을 관측하지 못해 생기던
     종방향 표류(0.8~1.7 m/랩)를 차단하고, 코너에서 회전된 잔여 오차를 고게인으로 잡습니다.
   - 마할라노비스 게이트로 MCL 순간 글리치를 걸러내고, 연속 기각이 길어지면 MCL로
     재고정하는 안전망이 있습니다. `use_pose_ekf: false`면 구 EMA 스무딩으로 돌아갑니다.

4. **TF 및 Odometry 발행**:
   - 추정된 포즈를 바탕으로 `map -> odom` TF 트랜스폼을 브로드캐스팅합니다.

5. **구조 결함 수정 (2026-08-03, D1~D6)**:
   - **D1**: 최종 출력(`get_current_pose()`)이 EKF 모드에서 `ekf_state_`를 우선 반환. 이전에는
     odom 추적 경로가 출력을 우회했고, 그 경로의 odom 델타가 map 프레임 회전 없이 더해져
     부팅 헤딩 φ만큼 어긋난 델타가 매 주기 적용됐음. fallback odom 추적 경로도 SE(2)
     합성(앵커 시점 프레임 요 차이 회전 + `normalize_angle`)으로 수정.
   - **D2**: `timer_update()` 내 잉여 `state_lock_.unlock()` 제거 (미소유 뮤텍스 unlock = UB).
   - **D3**: `odomCB`의 속도 대입·`update_odom_pose()` 호출을 `state_lock_` 안으로 이동 (데이터 레이스 수정).
   - **D5**: 지연 보상이 `mcl_processing_time_ × delay_compensation_factor`(실제 지연과 무관한
     base)를 쓰던 것을 **실측 lidar age(`now - scan stamp`, 0~0.2 s clamp)**로 교체. EKF 모드의
     발행 stamp도 내용(odom 최신 시각)에 맞춰 odom 시각으로 수정. `delay_compensation_factor`는 은퇴(미사용).
   - **D6**: 모션 노이즈를 map 좌표축에 직접 더하던 것을 **차체 프레임(종/횡)에서 생성 후
     갱신 헤딩으로 회전**하도록 수정. 이제 `motion_dispersion_x/y`가 의도대로 종방향/횡방향을
     의미함. ⚠️ 기존 sim/real 튜닝값은 map 프레임 전제였으므로 **재튜닝 필요**.
   - **D4**: `obs_px_` 변환 시 무효 레이(NaN/inf/0)를 max range로 처리(이전엔 "매우 가까운 벽"으로
     오해석), 중복 `last_steady_time` 대입 정리, dead 파라미터 `fine_timing`을 YAML에서 삭제.

6. **라이다 갭 대응 + 불가능 포즈 연속성 (2026-08-03, run_0803_2 시리즈 대응)**:
   - **갭 중 파티클 전파**: 라이다 스트림이 끊기는 동안(CASE 2: odom only) 파티클 구름도
     odom 기반 모션 모델로 함께 전파. 이전에는 파티클이 갭 시작 위치에 얼어붙어, 갭 종료 후
     첫 MCL이 수 초 전 위치로 평가돼 가중치 붕괴 → 포즈 스냅이 갭 직후에 집중 발생
     (bag 분석: 점프 19회 중 16회가 스캔 갭 직후).
   - **재앵커 가드 (MCL 연속성)**: EKF 게이트 force-accept(연속 기각 시 MCL 재고정) 대상이
     물리적으로 불가능한 위치(벽/미지 깊숙이, `is_pose_permissible` — 반경 15 cm 내 free 없음)면
     재고정하지 않음. 갭/맵 불일치 후 MCL이 벽 속 모드로 수렴했을 때 추정 전체를 벽 속에
     박는 최악 경로만 차단하는 좁은 가드 (일반 보정 경로는 건드리지 않음 — 넓은 비토는
     벽 밀착 주행이 많은 타이트 맵에서 회복을 막아 오히려 악화함을 재생으로 확인).

7. **스캔 강건화 — 다이낯믹 환경 대응 (2026-08-03, S2+§9)**:
   - **S2 (per-ray likelihood floor)**: 센서 모델 열(기대 거리)별 최댓값의
     `ray_likelihood_floor_ratio`(0.05)배를 하한으로 적용. 맵과 일부 다른 레이(가구 이동,
     사람 등)가 있어도 정답 가설이 레이 하나당 66배씩 깎여 급사하지 않음 (최대 20배로 제한).
   - **§9 (스캔 품질 연동 R)**: 최대 가중치 파티클 기준 outlier 비율(3·sigma_hit 초과 레이)과
     ESS를 매 주기 계산해, 맵 불일치 구간에서는 EKF 측정 노이즈 R을 연속적으로 부풀림
     (`use_scan_quality_r`, 실차만 기본 활성). 진동/환경 변화 구간을 odom 우세로 버티고
     지나면 자동 복귀. 하드 게이트와 달리 부분 신뢰라 복구 불능이 없음.

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

- **YAML 파일 위치**: `config/mcl_config.yaml` (real/bag), `config/mcl_config_sim.yaml` (sim 전용 — mod:=sim일 때 launch가 선택. launch는 값을 오버라이드하지 않고 YAML이 단일 소스)
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
