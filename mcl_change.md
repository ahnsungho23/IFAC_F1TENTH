# 구(舊) MCL → 현(現) MCL 변경 대조표

`monte_carlo_localization` 패키지(`particle_filter` 노드)가 **2026-08-15 "MCL 근본 개편"**
전후로 어떻게 바뀌었는지 정리한 문서입니다.

| 구분 | 기준 커밋 | 시점 |
|---|---|---|
| **구 MCL** | `a37294d` (`particle_filter: 정지 중 입자 확산 차단 — 이동량 비례 노이즈`) | 2026-08-15 개편 직전 |
| **현 MCL** | `cf972f3` (HEAD, `nhw_ifac`) | 2026-08-17 |

전체 변경 규모 (`git diff a37294d HEAD -- src/monte_carlo_localization`):

```
 AGENTS.md                                     |  15 +-
 config/mcl_config.yaml                        |  27 +-
 config/mcl_config_sim.yaml                    |   5 +
 docs/particle_filter.md                       |  10 +-
 include/particle_filter_cpp/particle_filter.hpp |  31 +-
 launch/mcl_launch.py                          |  15 +-
 maps/ifac_track.{md,png,yaml}                 |  삭제
 maps/map.{png,yaml}                           |  교체
 src/particle_filter.cpp                       | 491 +++++++++++++-------
 12 files changed, 456 insertions(+), 218 deletions(-)
```

변경을 만든 커밋 계보:

| 커밋 | 날짜 | 내용 |
|---|---|---|
| `59cd44a` | 08-15 | **근본 개편 본체 (T1~T7)** — ESS 게이트, 멀티스레드, 클러스터 포즈, 헬스 토픽 |
| `ce1bb19` | 08-15 | 빌드 오류 정리(`rclcpp::TimerOptions` 제거) + 재생성 타이머에 콜백그룹 배선 누락 수리 + launch nav2 폴백 |
| `5e5f957` | 08-16 | YAML 튜닝 (모션 노이즈·EKF 노이즈 축소) |
| `9525d73` | 08-16 | **C1/C2/M5 후속 수리** — `pre_resample_pose_` 도입, 콜백 락 정리, EKF 게이트 원복 |
| `4872770` | 08-16 | 맵 이미지 갱신 |
| `2e58225` | 08-17 | **데드락 수리** — `unique_lock` 함수 스코프 재귀 lock 문제 |

---

## 1. 알고리즘 순서 자체가 바뀌었습니다 (가장 큰 변화)

### 구 MCL — 매 사이클 무조건 리샘플

`src/particle_filter.cpp`의 `MCL()`은 아래 순서로 돌았습니다.

1. **리샘플링** (`std::discrete_distribution`, 무조건 실행)
2. 모션 모델 예측 (`proposal_distribution_`에 적용)
3. 센서 모델 → `weights_` **덮어쓰기**
4. 가중치 정규화 + ESS 계산
5. 긴급복구
6. `particles_.swap(proposal_distribution_)`

문제는 **매 사이클 리샘플이 강제**라는 점입니다. 리샘플이 끝나면 `weights_`는 전부 `1/N`이
되므로, 센서가 준 정보가 한 프레임만 살고 버려집니다. 정지 중이거나 스캔 정보가 부족한
구간에서 입자 다양성이 계속 갈려나가 발산했습니다.

### 현 MCL — ESS 게이트 리샘플 + 가중치 누적

`MCL()` 순서가 정석 SIR 구조로 재배치됐습니다.

1. **모션 모델 예측** — `particles_`에 직접 적용
2. **센서 모델** — `weights_` 갱신 (아래 누적 규칙 적용)
3. 정규화 (합이 0이면 `1/N`으로 균등 리셋 — 구버전엔 없던 방어)
4. **ESS 계산** → `ess_ratio_ = (1/Σw²) / N`
5. `pre_resample_pose_ = expected_pose(weights_)` **캐시** (§3 참조)
6. **조건부 리샘플** — `ess_ratio_ < ess_threshold` (기본 0.5) **또는** fast-convergence 중일 때만
7. 긴급복구 (`particles_`에 직접 재시드)

가중치 누적은 `calculate_particle_weights()`에서 `was_resampled_in_last_step_` 플래그로
갈라집니다.

```cpp
double sensor_w = std::pow(weight, squash_factor);
if (was_resampled_in_last_step_) {
    weights[i] = sensor_w;   // 리샘플 직후 → 새로 시작
} else {
    weights[i] *= sensor_w;  // 리샘플 스킵 → 프레임 간 우도 누적
}
```

즉 리샘플을 건너뛴 프레임에서는 센서 우도가 곱연산으로 쌓여, 여러 프레임에 걸친 정보가
살아남습니다.

---

## 2. 단일 스레드 → MultiThreadedExecutor + 콜백 그룹 격리 (T2/T3)

### 구 MCL

```cpp
rclcpp::spin(std::make_shared<particle_filter_cpp::ParticleFilter>());
```

`update_timer_`(40 Hz MCL), `map_timer_`(**200 ms**, `/map` 재발행), 라이다/오도메트리/
RViz 콜백이 **전부 한 스레드에 직렬**로 묶여 있었습니다. `/map` 재발행 같은 무거운 작업이
MCL 루프를 최대 1.3초까지 블로킹하는 공백이 발생했습니다.

### 현 MCL

```cpp
rclcpp::executors::MultiThreadedExecutor executor;
auto node = std::make_shared<particle_filter_cpp::ParticleFilter>();
executor.add_node(node);
executor.spin();
```

콜백 그룹 두 개로 물리적으로 분리했습니다.

| 콜백 그룹 | 소속 |
|---|---|
| `update_cb_group_` | `update_timer_`(MCL 40 Hz), `laser_sub_`, `odom_sub_` |
| `map_viz_cb_group_` | `map_timer_`(**2000 ms**로 연장), `health_timer_`(1000 ms), `pose_sub_`(`/initialpose`), `click_sub_`, `waypoints_sub_` |

- `map_timer_` 주기: **200 ms → 2000 ms** (0.5 Hz). `/map`은 `transient_local` QoS라 잦은
  재발행이 불필요합니다.
- 15 Hz 스타트업 타이머를 40 Hz로 재생성하는 경로에도 `update_cb_group_`을 넘기도록
  수리했습니다 (`ce1bb19` — 개편 본체에서 이 한 곳이 누락돼 재생성 후 기본 그룹으로
  떨어졌습니다).

---

## 3. 포즈 추정: 전역 가중평균 → Primary Cluster 가중평균 (T6)

### 구 MCL — 전역 가중평균

```cpp
for (int i = 0; i < MAX_PARTICLES; ++i) {
    pose[0] += weights_[i] * particles_(i, 0);
    pose[1] += weights_[i] * particles_(i, 1);
    sum_sin += weights_[i] * std::sin(particles_(i, 2));
    sum_cos += weights_[i] * std::cos(particles_(i, 2));
}
pose[2] = std::atan2(sum_sin, sum_cos);
```

이 트랙은 좌우 대칭 복도라 입자 구름이 두 갈래(bimodal)로 갈리는 일이 자주 생깁니다.
전역 평균은 그 **사잇값(= 벽 속 빈 공간)** 을 추정 포즈로 내보냈습니다.

### 현 MCL — 최고 가중치 입자 주변 클러스터만 평균

`expected_pose(const std::vector<double>& weights)` 오버로드가 새로 생겼고, 계산 방식이
바뀌었습니다.

1. 최고 가중치 입자를 기준점(`ref_x, ref_y, ref_theta`)으로 잡음
2. `dist <= cluster_radius`(0.5 m) **AND** `|dtheta| <= cluster_yaw_thres`(0.5 rad) 인
   입자만 골라 가중평균
3. 클러스터 밖 가중치 비율을 `bimodality_ratio_ = clamp(1 - cluster_weight_sum, 0, 1)`로 기록
4. 클러스터 가중치 합이 `1e-9` 이하면 기준 입자 포즈를 그대로 반환

### 여기서 파생된 C1 버그와 `pre_resample_pose_`

클러스터 방식은 "최고 가중치 입자"에 의존합니다. 그런데 리샘플 직후 `weights_`는 전부
`1/N`이라 "최고 가중치 탐색"이 사실상 **인덱스 0번 입자 고정**이 됩니다. 개편 본체(`59cd44a`)는
리샘플 **후**에 `expected_pose()`를 불렀기 때문에, 매 사이클 무작위 표본 하나를 추정 포즈로
발행하는 상태였습니다 (대칭 복도에서 발행 포즈가 모드 사이를 무작위 점프).

`9525d73`에서 수리했습니다.

- `MCL()`이 **리샘플 직전**(센서로 informed된 가중치 상태)에 `pre_resample_pose_`를 캐시
- `timer_update()`는 `expected_pose()` 재호출 대신 `pre_resample_pose_`를 읽음
- 긴급복구 재시드도 `pre_resample_pose_` 사용
- `publish_health()`도 `pre_resample_pose_` 재사용 (락 쥔 채 O(N) 재계산 제거 — M5)
- 무인자 `expected_pose()`는 하위 호환용으로만 남김

---

## 4. 신설: 헬스 진단 토픽 `/pf/health` (T4)

구 MCL에는 없던 토픽입니다. `health_timer_`(1000 ms, `map_viz_cb_group_`)가
`std_msgs/msg/String`으로 JSON을 내보냅니다.

```json
{"cycle_p50_ms":2.31,"last_gap_ms":25.0,"ess_ratio":0.612,
 "sigma_pos_m":0.043,"sigma_yaw_deg":1.82,"bimodality":0.031,"outlier_ratio":0.045}
```

| 필드 | 의미 |
|---|---|
| `cycle_p50_ms` | MCL 사이클 평균 소요 시간 |
| `last_gap_ms` | 직전 포즈 발행 공백 (`publish_tf()`에서 `last_pose_pub_stamp_` 기준 측정) |
| `ess_ratio` | ESS / N |
| `sigma_pos_m`, `sigma_yaw_deg` | 입자 퍼짐 (가중 표준편차) |
| `bimodality` | Primary Cluster 밖 가중치 비율 |
| `outlier_ratio` | 최대 가중치 입자 기준 outlier 레이 비율 |

확인:

```bash
ros2 topic echo /pf/health
```

`publish_tf()`에도 발행 공백 측정 코드가 추가됐습니다.

```cpp
rclcpp::Time pub_now = this->get_clock()->now();
if (last_pose_pub_stamp_.nanoseconds() != 0) {
    last_publish_gap_ms_ = (pub_now - last_pose_pub_stamp_).seconds() * 1000.0;
}
last_pose_pub_stamp_ = pub_now;
```

---

## 5. 스레드 안전성 수리 (C2 + 08-17 데드락)

멀티스레드로 넘어가면서 락 구조를 다시 짜야 했고, 그 과정에서 두 건이 더 고쳐졌습니다.

### (1) `timer_update()` 락: 수동 `try_lock()` → RAII

**구 MCL**

```cpp
if (state_lock_.try_lock()) {
    ...
    state_lock_.unlock();
}
```

블록 안에 힙 할당·Eigen 연산·로거가 전부 있어, 예외가 던져지면 `unlock()`이 스킵됩니다.
그러면 **이후 모든 사이클의 `try_lock`이 영구히 실패** — MCL은 조용히 멈추는데 stale pose
발행 경로는 살아 있어 겉보기엔 정상으로 보입니다.

**현 MCL**

```cpp
std::unique_lock<std::mutex> mcl_state_lock(state_lock_, std::try_to_lock);
if (mcl_state_lock.owns_lock()) { ... }
```

### (2) 2026-08-17 데드락 — 함수 스코프 `unique_lock` 재귀 lock

`mcl_state_lock`은 `if` **밖**에 선언돼 있으므로 **함수 스코프**입니다. `if` 블록을 닫아도
락은 풀리지 않는데, 그 아래 publish 블록이 같은 `std::mutex`를 다시 잠그면 **같은 스레드의
재귀 lock**이 됩니다. `std::mutex`는 non-recursive라 그 자리에서 영구 정지합니다.

실제 증상 연쇄:

```
초기화 로그(Odometry tracking initialized)까지 정상
  → timer_update 첫 호출에서 정지
  → update/map/health 타이머 전부 정지 (같은 실행기)
  → 전 토픽 발행 0 → map->odom TF 미발행
  → RViz "Frame [map] does not exist"
  → 고정 프레임 없어 2D Pose Estimate 클릭도 발행 안 됨 (교착)
```

수리:

```cpp
if (mcl_state_lock.owns_lock()) {
    mcl_state_lock.unlock();   // 명시적 해제
}
```

### (3) publish 구간을 하나의 락으로 통합

**구 MCL**은 타임스탬프 선택 부분만 짧게 락을 잡고, `get_current_pose()` /
`publish_tf()` / `visualize()`는 **락 없이** 호출했습니다. 단일 스레드에서는 문제가 없었지만,
멀티스레드에서는 `clicked_pose`/`waypointsCB`(`map_viz_cb_group_` 스레드)와 `ekf_state_`,
`odom_pose_`, `inferred_pose_`, `particles_`, `weights_`, `rng_`를 동시에 만져 찢어진 읽기와
RNG 동시 소비가 발생합니다.

**현 MCL**은 `publish_lock` 하나로 `get_current_pose()` → 타임스탬프 선택 → `publish_tf()` →
`visualize()` 전체를 감쌌습니다. (하위 함수들은 내부에서 락을 잡지 않는 순수 읽기 함수라
재귀 데드락 없음)

### (4) 콜백 내부 락 추가

`clicked_pose()` / `waypointsCB()`도 `map_viz_cb_group_` 스레드에서 도는 만큼,
`initialize_odom_tracking()` · `ekf_initialized_` · `inferred_pose_` ·
`fast_convergence_*` · `auto_init_done_` 갱신과 `visualize()` 호출을 각각
`std::lock_guard`로 묶었습니다. (`initialize_particles_pose()`는 자체 락을 잡고 반환하므로
그 뒤에 묶어 재귀 데드락을 피합니다)

---

## 6. 파라미터 변경

### 신설 (`mcl_config.yaml`, `mcl_config_sim.yaml` 양쪽)

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `ess_threshold` | `0.5` | `ESS/N`이 이 값보다 작을 때만 리샘플 (T5) |
| `cluster_radius` | `0.5` | Primary Cluster 위치 반경 [m] (T6) |
| `cluster_yaw_thres` | `0.5` | Primary Cluster 각도 허용치 [rad] (T6) |

### 값 변경 (`mcl_config.yaml`, 실차)

| 파라미터 | 구 | 현 | 이유 |
|---|---|---|---|
| `max_viz_particles` | 20 | **200** | 구름 분포 진단용 시각화 확장 (T7) |
| `motion_dispersion_x` | 0.10 | **0.05** | 모션 노이즈 축소 |
| `motion_dispersion_y` | 0.02 | **0.01** | 모션 노이즈 축소 |
| `motion_dispersion_theta` | 0.20 | **0.06** | 3.4° 이하로 회전 노이즈 대폭 안정화 |
| `ekf_rot_error_rate` | 0.10 | **0.05** | 요 프로세스 노이즈 축소 |
| `ekf_meas_yaw_std_floor` | 0.02 | **0.04** | 측정 요 과신 방지 |

`mcl_config_sim.yaml`은 위 3개 신설 파라미터만 추가됐고, 기존 값 변경은 없습니다.

### 되돌린 값 (원복 이력, 문서화 가치 있음)

- `ekf_gate_force_accept` / `ekf_gate_force_accept_dist`: 개편 중 `20 / 1.5` → `120 / 6.0`으로
  6배 완화했다가 **`20 / 1.5`로 원복**했습니다. 완화가 필요했던 진짜 원인은 §3의 C1 버그
  (리샘플 직후 균등 가중치로 무작위 입자를 가리켜 게이트가 상시 기각)였기 때문입니다.
  C1을 고쳤으니 기각 빈도가 원래 수준으로 돌아올 것으로 예상 — 재측정 후 여전히 상시
  기각되면 다른 원인이 있다는 뜻입니다.
- `publish_extrapolation_sec`: 개편 중 `0.00`으로 껐다가 **`0.09`로 원복**했습니다. 끄면
  ~90 ms 출력 지연이 그대로 컨트롤러 피드백 경로에 실려 L1 감쇠를 깎습니다
  (`pose_lpf` 58 ms와 합쳐 0.9 Hz 횡진동으로 발현).

---

## 7. 맵 파일 정리

구 MCL은 `maps/`에 두 트랙이 공존했습니다.

| 구 MCL | 현 MCL |
|---|---|
| `ifac_track.png` / `ifac_track.yaml` / `ifac_track.md` | **삭제** |
| `map.png` (2684 B), `map.yaml` (`origin: [0,0,0]`) | `map.png` (5697 B), `map.yaml` (`origin: [-2.347, -1.767, 0]`) |

구 `ifac_track.yaml`은 `resolution: 0.025`, `origin: [-20.3043, -1.4273, 0]`, `free_thresh: 0.25`
였고, 현 `map.yaml`은 `resolution: 0.050`, `free_thresh: 0.196`입니다.

> ⚠️ **실행 명령이 달라집니다.** `mcl_launch.py`의 `map_name` 기본값은 `map`이고,
> `ifac_track.yaml`은 더 이상 존재하지 않습니다. 저장소 루트 `CLAUDE.md`의 실행 순서에
> 적혀 있는 `map_name:=ifac_track`은 현재 트리에서는 파일이 없어 실패합니다. 지금은
> 인자 없이 (또는 `map_name:=map`으로) 띄우십시오.
>
> ```bash
> ros2 launch particle_filter_cpp mcl_launch.py mod:=sim use_rviz:=true
> ```

---

## 8. Launch 변경 (`mcl_launch.py`)

`nav2_lifecycle_manager` 패키지가 없는 환경을 위한 폴백이 추가됐습니다.

- **구**: `map_server` + `lifecycle_manager` 노드를 무조건 반환
- **현**: `get_package_share_directory('nav2_lifecycle_manager')`를 먼저 시도하고, 실패하면
  `map_server`만 띄운 뒤 `ExecuteProcess`로 CLI 명령을 순차 실행

```bash
sleep 1.0; ros2 lifecycle set /particle_filter_map_server configure; \
sleep 0.5; ros2 lifecycle set /particle_filter_map_server activate
```

파라미터 오버라이드 금지 원칙(launch는 배선만, 튜닝값은 YAML만)은 변경 없이 유지됩니다.

---

## 9. 헤더 변경 요약 (`particle_filter.hpp`)

| 구분 | 추가된 것 |
|---|---|
| include | `std_msgs/msg/string.hpp` |
| 함수 | `Eigen::Vector3d expected_pose(const std::vector<double>& weights)` (오버로드), `void publish_health()` |
| 파라미터 멤버 | `ESS_THRESHOLD`, `CLUSTER_RADIUS`, `CLUSTER_YAW_THRES` |
| 상태 멤버 | `was_resampled_in_last_step_`, `bimodality_ratio_`, `pre_resample_pose_`, `last_publish_gap_ms_`, `last_pose_pub_stamp_` |
| 콜백 그룹 | `update_cb_group_`, `map_viz_cb_group_` |
| 퍼블리셔 | `health_pub_` |
| 타이머 | `health_timer_` |

---

## 10. 한눈에 보는 대조표

| 항목 | 구 MCL (`a37294d`) | 현 MCL (HEAD) |
|---|---|---|
| 실행기 | `rclcpp::spin` (단일 스레드) | `MultiThreadedExecutor` + 콜백 그룹 2개 |
| MCL 순서 | 리샘플 → 모션 → 센서 | 모션 → 센서 → ESS 게이트 → 조건부 리샘플 |
| 리샘플 조건 | 매 사이클 무조건 | `ess_ratio < 0.5` 또는 fast-convergence 중 |
| 가중치 | 매 프레임 덮어쓰기 | 리샘플 스킵 프레임에서 곱연산 누적 |
| 포즈 추정 | 전 입자 가중평균 | 최고 가중치 주변 Primary Cluster 가중평균 |
| 포즈 계산 시점 | 리샘플 **후** | 리샘플 **전** 캐시 (`pre_resample_pose_`) |
| `/map` 재발행 | 200 ms | 2000 ms |
| 진단 | 없음 | `/pf/health` (1 Hz JSON) |
| `timer_update` 락 | 수동 `try_lock`/`unlock` (예외 시 영구 잠김) | `unique_lock(try_to_lock)` + 명시적 unlock |
| publish 구간 락 | 타임스탬프만 부분 락 | 전 구간 단일 락 |
| RViz/waypoint 콜백 락 | 없음 | `lock_guard`로 보호 |
| 시각화 입자 수 | 20 | 200 |
| 맵 | `map` + `ifac_track` 2종 | `map` 1종 |

---

## 참고 명령

```bash
# 코드 차이 전체
git diff a37294d HEAD -- src/monte_carlo_localization

# 개편 본체만
git show 59cd44a

# 헬스 모니터링
ros2 topic echo /pf/health
```
