# ROS 2 Jazzy 포트 검증 기록

> 루트 `README.md`의 §6에서 분리한 이력 문서입니다 (2026-07-29 기준 기록, 이후 위치추정은 MCL → Kinematic-ICP로 교체됨).

이 브랜치(`jazzy_main`)는 `backup/jetson-20260725`의 Humble 기반 스택을
**ROS 2 Jazzy (Ubuntu 24.04)** 로 포팅하고 실제 빌드·주행으로 검증한 결과에,
2026-07-29의 obstacle_detector 교체·teleop 제거·MCL 개선(아래 참고)을 반영한 것입니다.
(검증일: 2026-07-29, x86_64 데스크톱 / 포트 커밋은 `jazzy-port-test`·`jazzy_port_main`과 공유)

## 1. 포팅 변경 사항

| 분류 | 내용 |
|---|---|
| tf2 헤더 | deprecated `.h` → `.hpp` 5개 파일 (particle_filter, opponent_simulator, opponent_detector 등) — Kilted 대비 겸용 |
| CMake | 전 패키지 `ament_target_dependencies` → modern `target_link_libraries` (9개 패키지 27곳) |
| rosdep 키 | `nlohmann_json`→`nlohmann-json-dev`, 가짜 `openmp` 제거 (global_planning), 기존 waypoint selector의 `nav_mags` 오타 수정, 미선언 `angles` 제거 (MCL), 미사용 `pcl_ros`/`pcl_conversions` 제거 (MCL) |
| lap_timer | `rviz_2d_overlay_msgs` 무가드 import 제거 — Jazzy에 바이너리 미배포라 실행 즉시 죽던 것을 선택적 import로 전환 |
| MCL sim 튜닝 | sim 모드 한정 motion_dispersion/smoothing 오버라이드 (`mcl_launch.py`, 실차 값 불변) |
| 데이터 복구 | git에서 유실됐던 `ifac_track.{png,yaml}` 맵 복구 |
| 포맷 | `builtin_interfaces/Time` 로그 포맷 `-Wformat` 경고 수정 |
| backup 머지 포트 (07-29) | MCL lib에 `${f110_msgs_TARGETS}` 링크 추가(backup이 MCL에 WpntArray 도입), 신규 문서·스크립트의 humble 표기 → jazzy, `sim/run.sh`가 `F1_MAP`을 export해 글로벌/로컬 플래너 맵 통일 |
| 깨진 패키지 복구 | backup 팁은 **커밋된 미해결 충돌 마커**(`f971351` 머지 잔재)와 유실 파일 때문에 4개 패키지가 컴파일 불가였음 — ① local_planning: `raceline_spline_planner.cpp` 구현 유실+마커 ② state_machine: hpp/cpp 선언 불일치 ③ opponent_detector: cpp·yaml 등 6개 파일에 마커 ④ f110_msgs: `Obstacle.msg`의 Cartesian 확장 유실. 전부 main 복구 커밋 `b47c785`와 동일 내용(Jazzy 포트 포함, `jazzy-port-test` 검증본)으로 교체. f1tenth_control의 `WORKLOG.md`·`vesc_mcconf.xml` 마커는 젯슨 최신(HEAD쪽)으로 해소 |
| 검출기 교체 포트 (07-29) | main 최신(`0dea9d2`)의 **obstacle_detector**(레이어드 검출, `/static_obs`·`/opp_obs`) + 재구성 **local_planning**(safe_corridor 제거, 노드 결합)으로 교체 — `opponent_detector` 패키지 삭제. 두 패키지 CMake를 modern `target_link_libraries`로 전환, obstacle_detector tf2 헤더 `.h`→`.hpp` 4곳, local_planning package.xml에 launch 실사용 exec_depend(nav2_map_server·nav2_lifecycle_manager·particle_filter_cpp) 보강, 문서·sim 스크립트의 opponent_detector 참조 일괄 갱신 |

## 2. 검증 결과

- **빌드**: `build/ install/ log/` 전부 삭제 후 클린 빌드 — **12개 패키지 전부 성공, 경고 0건** (2분 18초, Ninja)
- **런타임**: gym 시뮬레이터(f1sim_C, Jazzy 네이티브) + 7노드 풀스택으로 **자율주행 3랩 연속 완주**
  - 랩타임 ~102 s (지시 속도 0.5 m/s 정속)
  - MCL 위치추정 오차: 최대 0.18 m, 평상시 1~6 cm (320 s 연속 추적)
  - 레이스라인 횡편차 |d| ≤ 0.36 m, 전 코너 통과
  - 전 토픽 발행 확인: `/scan` 49 Hz, `/pf/pose/odom` 30 Hz, `/car_state/frenet/odom` 21 Hz, `/local_waypoints` 21 Hz, `/drive` 38 Hz 등
- **backup 머지 재검증 (2026-07-29, 이 브랜치)**: `backup/jetson-20260725`(666e6b1)에 Jazzy
  포트를 머지하고 컴파일 불가 패키지(local_planning·state_machine·opponent_detector·
  f110_msgs — 아래 §6.1 "local_planning 복구" 참고)를 복구한 뒤 동일 절차로 재검증 —
  클린 빌드 12/12 성공(포트 코드 경고 0건, 벤더 CLCS·LTO 잡음 제외), 폐루프 **3랩 연속
  완주**, MCL 오차 평균 0.032 m / 최대 0.183 m, 레이스라인 |d| 중앙값 0.014 m(순간 스파이크
  1회 0.48 m 제외 시 ≤ 0.09 m), 전 7노드 에러 로그 0건. (터미널 4가 opponent_detector·
  레퍼런스 맵 서버를 기본 포함하는 구조 — §3 터미널 4·9 참고)
- **검출기 교체 재검증 (2026-07-29, `jazzy_port_main`에서 수행 — 이 브랜치에 반영)**:
  main 최신(`0dea9d2`)의 obstacle_detector + 재구성 local_planning으로 교체한 뒤 재검증 —
  클린 빌드 12/12 경고 0건(2분 6초), `colcon test`(local_planning gtest+lint 9/9) 통과,
  합성 장애물 파이프라인 테스트(`static_obs_pipeline_test.py`, `/scan`→`/static_obs`→
  `/avoid_waypoints`) PASS, gym 폐루프 3랩 연속 완주(0.5 m/s), MCL 오차 평균 0.014 m /
  최대 0.130 m, 검출기 스캔 드롭 0·오탐 0, `/drive` 39 Hz·`/local_waypoints` 30 Hz,
  전 노드 에러 로그 0건. (gym 워크스페이스가 `~/f1sim_C` → `~/sim_ws`로 이동됨)
- **속도 한계 스윕 (2026-07-29)**: ifac_track 웨이포인트를 `--max-speed`만 바꿔 재생성
  (라인 기하 동일, 제거 전 scalar 설정의 `a_lat=1.0`)하고 각 190 s(6~12랩) 폐루프로 MCL 추적을 비교했다.
  현재 생성기는 `velocity_limits.csv`의 `max_lateral_accel_mps2` 열을 사용하므로 아래 표는 과거 조건의
  측정 기록이다.

  | max_speed | 랩 | GT 대비 오차 max | 판정 |
  |---|---|---|---|
  | 3.0 | 6 | 0.28 m | ✅ 클린 |
  | **4.0** | 7 | **0.23 m** | ✅ **검증 상한** |
  | 4.5 | 7 | 1.41 m | ⚠️ 복도 스파이크 |
  | 5.0 | 12 | 1.71 m | ⚠️ 복도 스파이크(완주는 함) |
  4.5부터 y≈0.1 장복도 직선에서 **종방향(along-corridor) 오차가 랩마다 0.4~1.7 m 스파이크**
  (평행벽 기하라 진행방향 관측성이 약함 — 코너 진입에서 재고정). 코너는 당시 `a_lat=1.0` 기준
  0.4~1 m/s라 요레이트 플립은 발생하지 않았다. 구 0.5 프로파일은
  `output/ifac_track_maxspeed0.5_backup/`에 보관.
- **MCL pose fusion EKF (2026-07-29)**: 위 복도 한계를 출력단 EKF로 해소 — 휠 odom 포즈
  델타(laser 프레임 변환 포함)로 30 Hz 예측 + MCL 기대 포즈 보정, R = 파티클 공분산에
  **차체 종방향만 25배 불신**(복도에서 파티클이 틀린 곳에 좁게 수렴해 공분산이 모호성을
  과소평가하므로). 마할라노비스 게이트 + 연속 기각 시 재고정 안전망 포함. 재스윕 결과 —
  | max_speed | 실측 vmax | GT 오차 p50 / 정상상태 max | 판정 |
  |---|---|---|---|
  | 5.0 | 5.00 | 1.2 cm / 8 cm | ✅ (EMA 시절 1.7 m 스파이크 → 소멸) |
  | 6.0 | 5.92 | 1.2 cm / 10 cm | ✅ |
  | 7.0 | **6.21** | 1.1 cm / 10.5 cm | ✅ — **한계는 이제 MCL이 아니라 max_accel 3.0·직선 길이** |
  시뮬 기본 프로파일은 max_speed 7.0(실측 vmax 6.2, 랩 ~27 s). 더 올리려면
  `velocity_limits.csv`의 `max_accel_mps2`/`max_lateral_accel_mps2` 열을 상향해야 하며 이는
  제어 안전 검증과 한 세트.
  구현: `use_pose_ekf`(기본 true, false면 구 EMA), 파라미터는 `mcl_config.yaml` 참고.
  ⚠️ 실차 전제: VESC 휠 odom 병진 오차 ~0.3%(`ekf_trans_error_rate` 유효값 1%로 설정),
  odom yaw는 조향 명령 합성이므로 실차 셰이크다운에서 `ekf_rot_*` 재튜닝 여지 있음.

## 3. 시뮬 검증에서 확인된 주의사항

1. **초기 위치는 반드시 지정** — 헤드리스에서는 `/initialpose` 직접 발행 (§3 터미널 2 참고).
   전역 초기화만으로는 반복 복도 형상 때문에 엉뚱한 위치에 수렴할 수 있습니다.
2. **gym은 벽 충돌을 물리적으로 막지 않습니다** — 제어가 이탈하면 차가 벽을 통과해 맵 밖으로 나가고,
   그 순간부터 스캔이 맵과 매칭되지 않아 MCL이 복구 불가능하게 발산합니다.
3. **시뮬 맵·MCL 맵·웨이포인트 맵 3곳 일치**는 절대 조건입니다 (sim.yaml `map_path` 확인).
4. gym 브리지는 `/clock`을 발행하지 않습니다 — `use_sim_time`이 sim 모드에서 true로 설정되지만
   타이머가 전부 wall clock 기반이라 동작에는 지장이 없음을 확인했습니다.
