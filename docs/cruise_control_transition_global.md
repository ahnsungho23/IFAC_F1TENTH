# 크루즈 컨트롤 이식 제안서 (개정 3판) — 파일별 작업 지시서

**개정일**: 2026-08-21 (야간)
**대상**: `adaptive_global` (현재 HEAD `b2cd573`)
**이 판의 목적**: **어느 파일을 통째로 복사해 오고, 어느 파일의 어느 줄을 어떻게 고칠지**를
파일 단위로 확정한다. 배경 설명은 최소화하고 작업 지시만 남긴다.

> 2판 대비 정정 3건 (실측 검증 결과 반영)
> - `GapData` 누락 필드 **13개 → 11개** (13은 컴파일 오류 *줄 수*였다)
> - "필드 추가는 소비자를 안 깬다" → **9개 패키지 전부 재빌드 필요**
> - "교차공분산 0.0은 보수적" → **반대다. 과소평가다** (§B-5에 수치)

---

## 목차

- [0. 현재 위치 — 무엇이 이미 끝났나](#0-현재-위치--무엇이-이미-끝났나)
- [A. 복사해 올 파일](#a-복사해-올-파일)
- [B. 수정할 파일 — 위치와 내용](#b-수정할-파일--위치와-내용)
- [C. 새로 작성할 파일](#c-새로-작성할-파일)
- [D. 작업 순서와 검증](#d-작업-순서와-검증)
- [E. 가져오면 안 되는 것](#e-가져오면-안-되는-것)
- [부록 — 근거 확인 명령](#부록--근거-확인-명령)

---

## 0. 현재 위치 — 무엇이 이미 끝났나

이식 경로가 초판 설계와 달랐다. `transition_global` → `adaptive_global` 직접이 아니라:

```
transition_global ──(08/21 sync)──▶ sungho_main(aa07fe9) ──(폴더 덮어쓰기)──▶ adaptive_global 워킹트리
```

### ✅ 이미 워킹트리에 들어와 있는 것 (재작업 불필요)

| 파일 | 상태 |
|---|---|
| `src/f1tenth_control/control_code/cruise_controller.cpp` | 신규 배치됨 |
| `src/f1tenth_control/control_code/cruise_controller_node.cpp` | 신규 배치됨 |
| `src/f1tenth_control/include/f1tenth_control/cruise_controller.hpp` | 신규 배치됨 |
| `src/f1tenth_control/config/cruise_controller.yaml` | 신규 배치됨 |
| `src/f1tenth_control/test/test_cruise_controller.cpp` | 신규 배치됨 (**18케이스 전부 통과 확인**) |
| `src/f1tenth_control/CRUISE_TUNING_GUIDE.md` | 신규 배치됨 |
| `src/f1tenth_control/CMakeLists.txt` | 크루즈 타깃·설치 규칙 반영됨 |
| `src/f1tenth_control/control_code/control_map_node.cpp` | `/cruise_speed_limit` 구독 + 상한 적용 + `max_speed` 콜백 |
| `src/f1tenth_control/launch/_control_common.py` | 크루즈 런치 인자 + `build_cruise_controller_node` |

🔴 **전부 미커밋이다.** `git reset --hard`에 통째로 날아간다 — 이 세션에서 실제로 한 번 잃었다.
작업 시작 전 `git add -A src/f1tenth_control && git commit` 하거나 `git stash push -u`로 보존할 것.

### ❌ 아직 안 된 것 — 이 문서가 다루는 범위

| # | 항목 | 성격 |
|---|---|---|
| A-1 | `f110_msgs/msg/GapData.msg` 복사 | 🔴 빌드 블로커 |
| B-1 | `f110_msgs/msg/Obstacle.msg` 필드 삽입 | 🔴 빌드 블로커 |
| C-1 | `src/f1tenth_control/docs/cruise_controller_node.md` 신규 | 🔴 install 블로커 |
| B-2~4 | `interference_distance_m` 정합 (3파일) | 🔴 리밋 사이클 |
| B-5 | 불확실성 시간 전파 비활성화 (2파일) | 🔴 과소평가 |
| B-6 | 빈 `/opp_obs` 계약 (3파일) | 🔴 안전 |
| B-7 | `state_machine.yaml` 오참조 정정 (5곳) | ⚠️ 문서 |

---

## A. 복사해 올 파일

### A-1. `f110_msgs/msg/GapData.msg` — **전체 교체**

```bash
git show transition_global:f110_msgs/msg/GapData.msg > f110_msgs/msg/GapData.msg
```

현재 3필드 → 14필드 + 상수 5개. **누락은 11개**다:
`active_constraint`, `raw_gap`, `effective_gap`, `desired_gap`, `sigma_gap`, `horizon_tau`,
`ego_speed`, `opponent_speed`, `feedback_speed`, `braking_speed`, `speed_limit`.

🔑 **전체 교체가 안전한 이유**: legacy 3필드(`gap_diff`/`vs_diff`/`gap_int`)가 **맨 앞에
이름 그대로** 남는다. 기존 소비자의 소스는 그대로 컴파일된다.

⚠️ **그래도 재빌드는 전부 필요하다.** msg 타입 해시가 바뀌므로 `f110_msgs`를 `package.xml`에
선언한 **9개 패키지 전부**를 다시 빌드해야 한다:
`global_planning`, `map_creator`, `f1tenth_control`, `local_planning`, `static_obstacle_map`,
`new_map_con`, `state_machine`, `obstacle_detector`, `kinematic_localization`.

### A-2. 복사해 올 파일은 이것 하나뿐이다

나머지는 **수정(§B)** 또는 **신규 작성(§C)**이다. 특히:

- `src/f1tenth_control/docs/` 는 **어느 브랜치에도 없다** — `transition_global`·`sungho_main`·
  `adaptive_global` 셋 다 `git ls-tree` 0건. 복사가 아니라 §C-1로 새로 써야 한다.
- `state_machine`·`obstacle_detector`의 확률 술어 파일들은 Phase 2(선택) 범위다. 이 문서는
  **지금 실차에 올리기 위한 최소 집합**만 다룬다.

---

## B. 수정할 파일 — 위치와 내용

### B-1. 🔴 `f110_msgs/msg/Obstacle.msg` — 8줄 삽입 (전체 교체 금지)

⚠️ **`transition_global` 버전으로 덮으면 안 된다.** `adaptive_global`에만 있는
`is_interfering`(35~37행)이 사라지고 `state_machine`·`obstacle_detector`가 동시에 깨진다.

**위치**: 32행 `float64 vd_var` 다음, 33행 `bool is_static` 앞.

```diff
 float64 vs_var
 float64 vd_var
+# Cross-covariances of the Frenet constant-velocity tracker blocks. Together with
+# (s_var, vs_var) and (d_var, vd_var) they form valid PSD 2x2 blocks, which lets a consumer
+# propagate the obstacle's positional uncertainty over a prediction horizon:
+#   var_s(t) = s_var + 2*t*s_vs_cov + t^2*vs_var
+# A producer that does not fill them leaves 0.0, which reads as "no cross-correlation" and
+# degrades to the previous diagonal-only propagation.
+float64 s_vs_cov
+float64 d_vd_cov
 bool is_static
 bool is_visible
 # True when the dynamic opponent overlaps the ego driving corridor and its current or predicted
 # longitudinal gap is small enough to interfere with ego driving.
 bool is_interfering
```

🔑 **이 위치인 이유**: `transition_global`과 같은 자리라 나중에 두 브랜치를 합칠 때
충돌이 안 난다. 결과 파일은 두 브랜치의 **상위집합**이 된다.

ℹ️ `d_vd_cov`는 **컴파일 블로커가 아니다**(참조하는 코드 0건). PSD 쌍이므로 같이 넣는다.

---

### B-2. 🔴 `src/obstacle_detector/config/obstacle_detector.yaml` — 116행

```diff
-    interference_distance_m: 1.0               # 간섭 진입 후면 간격 [m]
+    interference_distance_m: 5.0               # 간섭 진입 후면 간격 [m]
+                                               # cruise trailing_gap(5.0)과 같거나 커야 한다.
+                                               # 작으면 CRUISE 진입->5 m 확보->간섭 해제->
+                                               # GLOBAL 재가속->재진입 리밋 사이클이 돈다.
```

`interference_distance_margin_ratio: 0.20`(117행)이 같은 ID를 **6.0 m**까지 붙잡으므로
히스테리시스는 확보된다.

⚠️ **반대 방향(크루즈 `trailing_gap`을 1.0으로 인하)은 권하지 않는다** — 1.0 m 추종은
검출 지연과 제동거리를 덮지 못한다.

### B-3. 🔴 `src/obstacle_detector/src/obstacle_detector_node.cpp` — 196행

**yaml만 고치면 안 된다.** 코드 기본값이 남아 있으면 yaml 없이 노드를 띄웠을 때 조용히
1.0으로 돌아간다.

```diff
-    this->declare_parameter<double>("interference_distance_m", 1.0);
+    this->declare_parameter<double>("interference_distance_m", 5.0);
```

### B-4. `src/obstacle_detector/docs/obstacle_detector_node.md` — 420행

```diff
-... | ego corridor 횡겹침과 현재/예측 후면 간격으로 `is_interfering` 판정. 기본 1.0 m 진입, 같은 ID는 1.2 m에서 해제 |
+... | ego corridor 횡겹침과 현재/예측 후면 간격으로 `is_interfering` 판정. 기본 5.0 m 진입, 같은 ID는 6.0 m에서 해제 (f1tenth_control cruise `trailing_gap`과 한 쌍) |
```

243~244행은 값을 안 적고 파라미터명만 쓰므로 수정 불필요.

---

### B-5. 🔴 불확실성 시간 전파 끄기 — 2파일

**왜**: `σ_g² = s_var + 2τ·cov + τ²·vs_var`인데 생산자가 없어 `cov = 0`이 실린다. CV 칼만은
예측 단계에서 `P₀₁ ← P₀₁ + dt·P₁₁`로 교차항이 **양수**로 자라므로, 0을 쓰는 것은 미래 위치
분산의 **과소평가**다. 실측 대표값(σ_s = 0.2 m, σ_vs = 0.707 m/s, `uncertainty_sigma` 2.0):

| τ | cov=0 (현재) | cov=+최대 | 과소평가 | 미확보 간격 |
|---|---|---|---|---|
| 0.0 | 0.200 | 0.200 | 0.0% | 0.000 m |
| 0.5 | 0.406 | 0.552 | 26.4% | 0.292 m |
| **1.0 (현재 운영값)** | **0.735** | **0.906** | **18.9%** | **0.341 m** |

🔑 **설정 파일 자신이 이미 모순이다.** `cruise_controller.yaml` 51~53행 주석이
*"둘 다 0.0 = 전파 없음 = 구 거동과 비트 동일**(착지 기본값)**"*이라 해 놓고 바로 아래에서
1.0으로 켠다. 아래 수정은 **주석이 원래 지시한 값으로 되돌리는 것**이다.

#### B-5a. `src/f1tenth_control/config/cruise_controller.yaml` — 54~55행

```diff
-    gap_uncertainty_horizon_max: 1.0   # tau_max [s]. CV 유효 지평(interference_horizon_sec과 정렬)
-    opp_speed_confidence_z: 1.0        # z_v. A/B 활성화
+    # 🔴 착지 기본값 0.0 (위 주석의 지시대로). 검출기가 s_vs_cov를 생산하기 전까지는
+    #    cov 항이 0으로 들어가 sigma_g를 최대 26% 과소평가한다. Phase 2에서 생산자를
+    #    붙인 뒤 1.0으로 켤 것.
+    gap_uncertainty_horizon_max: 0.0   # tau_max [s]
+    opp_speed_confidence_z: 0.0        # z_v
```

#### B-5b. `src/f1tenth_control/launch/_control_common.py` — 96행·100행

**yaml만 고치면 무효다** — 런치가 같은 이름의 파라미터를 덮어쓴다.

```diff
         DeclareLaunchArgument(
-            'gap_uncertainty_horizon_max', default_value='1.0',
+            'gap_uncertainty_horizon_max', default_value='0.0',
             description='상대차 위치·속도 공분산 시간 전파 지평 상한 [s]. '
                         '0=전파 없음(착지 기본값). 검출기가 s_vs_cov를 생산한 뒤에만 켤 것'
         ),
         DeclareLaunchArgument(
-            'opp_speed_confidence_z', default_value='1.0',
+            'opp_speed_confidence_z', default_value='0.0',
             description='상대차 속도 하한 계산에 쓰는 표준편차 배수. 0=하한 미적용'
         ),
```

---

### B-6. 🔴 빈 `/opp_obs` 계약 — 3파일 (실차 전 필수)

#### 무엇이 문제인가

빈 배열 **한 프레임**에 속도 상한이 최대속도로 풀린다. 두 코드가 맞물린 결과다:

```cpp
// state_machine_node.cpp:411 — 빈 배열이면 any_of가 false → 디바운스 없이 즉시 false
opponent_interfering_ = std::any_of(msg->obstacles.begin(), msg->obstacles.end(), ...);
```
```cpp
// cruise_controller_node.cpp:228-231 — 상태가 CRUISE가 아니면 즉시 최대속도
if (!state_seen_ || state_ != f110_msgs::msg::StateMachine::STATE_CRUISE) {
    publishLimit(maximum_speed_);
    return;                        // ← 240행 clear_confirm_sec 검사에 도달조차 못 한다
}
```

🔑 **`clear_confirm_sec: 1.00`은 CRUISE 안에 머무는 동안에만 평가된다.** 상태 게이트가 그
앞에서 단락시키므로, 상태가 먼저 빠져나가면 확인 시간이 **구조적으로 무효**다.

🔑 **`opponent_stale_timeout_sec: 0.3`은 이걸 못 막는다.** 그건 **토픽 침묵**만 잡는데,
빈 배열도 `last_opponent_time_`을 갱신하므로 "신선함"으로 취급된다.

#### 설계 — 세 경우를 구분한다

| 경우 | 지금 | 바꾼 뒤 |
|---|---|---|
| 빈 배열 / 간섭 없는 배열 | 즉시 해제 | **확인 시간 경과 후** 해제 |
| 토픽 침묵 | `opponent_stale_timeout_sec` 후 해제 | 동일 (변경 없음) |
| 간섭 있는 배열 | 즉시 설정 | 동일 (진입은 즉시가 맞다) |

#### B-6a. `src/state_machine/include/state_machine/state_machine_node.hpp` — 108행 뒤

```diff
   bool opponent_seen_{false};
   bool opponent_interfering_{false};
+  // 빈/무간섭 /opp_obs가 연속으로 관측되기 시작한 시각. 간섭 해제는 이 시점부터
+  // opponent_clear_confirm_sec가 지나야 확정된다(진입은 즉시). 토픽 침묵은 별도로
+  // last_opponent_time_ + opponent_stale_timeout_sec가 잡는다.
+  std::optional<rclcpp::Time> opponent_clear_since_;
+  double opponent_clear_confirm_sec_{1.0};
```

`<optional>` 인클루드가 없으면 같이 추가한다.

#### B-6b. `src/state_machine/src/state_machine_node.cpp` — 405~415행 전체 교체

```cpp
void StateMachineNode::on_opponent(const f110_msgs::msg::ObstacleArray::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }
  opponent_seen_ = true;
  const auto stamp = now();
  const bool interfering_now = std::any_of(
    msg->obstacles.begin(), msg->obstacles.end(),
    [](const auto & obstacle) {return !obstacle.is_static && obstacle.is_interfering;});

  if (interfering_now) {
    // 진입은 즉시다 — 늦게 반응해서 얻을 게 없다.
    opponent_interfering_ = true;
    opponent_clear_since_.reset();
  } else if (!opponent_clear_since_.has_value()) {
    // 해제 후보 첫 프레임. 아직 상태를 바꾸지 않는다.
    opponent_clear_since_ = stamp;
  } else if ((stamp - *opponent_clear_since_).seconds() >= opponent_clear_confirm_sec_) {
    // 확인 시간을 채웠을 때만 해제한다. 한 프레임 결측으로 상한이 풀리지 않는다.
    opponent_interfering_ = false;
  }
  last_opponent_time_ = stamp;
}
```

같은 파일 **65행 근처**(파라미터 선언부)와 **100행 근처**(로드부)에 추가:

```cpp
declare_parameter<double>("opponent_clear_confirm_sec", 1.0);   // 65행 근처
opponent_clear_confirm_sec_ =                                    // 100행 근처
  get_parameter("opponent_clear_confirm_sec").as_double();
```

**124행 근처** 검증 블록에도 한 줄 추가한다(기존 `opponent_stale_timeout_sec` 검증과 같은 형태):

```cpp
if (!std::isfinite(opponent_clear_confirm_sec_) || opponent_clear_confirm_sec_ < 0.0) {
  throw std::invalid_argument("opponent_clear_confirm_sec must be finite and non-negative");
}
```

#### B-6c. `src/state_machine/config/state_machine.yaml` — 30행 뒤

```diff
     opponent_stale_timeout_sec: 0.3
+    # 빈(또는 간섭 없는) /opp_obs가 이 시간 동안 연속돼야 간섭 해제를 확정합니다.
+    # 진입은 즉시입니다. 한 프레임 결측으로 CRUISE가 풀려 속도 상한이 최대속도로
+    # 되돌아가는 것을 막습니다.
+    # 🔑 f1tenth_control cruise_controller.yaml의 clear_confirm_sec(1.00)과 맞출 것 —
+    #    상태가 먼저 빠져나가면 그쪽 확인 시간이 구조적으로 무효가 됩니다.
+    opponent_clear_confirm_sec: 1.0
```

#### B-6d. 테스트 — `src/state_machine/test/` 에 3케이스 추가

```
□ 빈 배열 1프레임      → opponent_interfering_ 유지, 상태 CRUISE 유지
□ 빈 배열 1.0초 지속   → 해제, 상태 GLOBAL
□ 토픽 침묵 0.3초      → stale 경로로 해제 (빈 배열 경로와 독립임을 확인)
```

---

### B-7. `state_machine.yaml` 오참조 5곳 정정

`state_machine.yaml`에 `interference_distance_m`은 **존재하지 않는다**(grep 0건).
소유자는 `obstacle_detector`다. 5곳 전부 대상을 바꾸고 값을 5.0으로 맞춘다.

| 파일 | 행 | 고칠 내용 |
|---|---|---|
| `src/f1tenth_control/AGENTS.md` | 32 | `state_machine.yaml`'s → `obstacle_detector.yaml`'s |
| `src/f1tenth_control/CRUISE_TUNING_GUIDE.md` | 389 | 경로 교체 + **"(현재 5.0)"** 유지 (B-2 적용 후 참이 됨) |
| `src/f1tenth_control/CRUISE_TUNING_GUIDE.md` | 446 | 경로 교체 |
| `src/f1tenth_control/config/cruise_controller.yaml` | 22 | 경로 교체 |
| `src/f1tenth_control/include/f1tenth_control/cruise_controller.hpp` | 91~92 | `see state_machine.yaml` → `see obstacle_detector.yaml` |

⚠️ B-2를 적용하지 않은 채 문서만 고치면 "5.0"이 거짓이 된다. **B-2 → B-7 순서로 할 것.**

---

## C. 새로 작성할 파일

### C-1. 🔴 `src/f1tenth_control/docs/cruise_controller_node.md` — 신규

**두 가지 이유로 필수다.**

1. **install 블로커.** `CMakeLists.txt:134`가 `install(DIRECTORY docs ...)`인데 디렉터리가
   없다. CMake 최소 재현 결과 **치명적 실패**다:
   ```
   file INSTALL cannot find ".../docs": No such file or directory.
   install 종료코드 = 1
   ```
   🔴 **`sungho_main` 트리에도 `docs/`가 없다** — 그 브랜치에서도 install이 실패한다.
   08/21 sync가 빠뜨린 것이므로 `adaptive_global`에서 새로 써야 한다.
   (`adaptive_global` 원본 CMakeLists에는 이 줄이 아예 없었다 = 덮어쓰기가 들여온 블로커.)

2. **저장소 규칙.** `CLAUDE.md:53-64` "Node Documentation" — 새 노드는
   `docs/<node_name>.md`를 한국어로 단계별 작성해야 한다.

**포함할 항목** (CLAUDE.md 규정):

```
□ 노드 목적            전방 상대차 간격 → 종방향 속도 상한
□ 동작 원리            desired_gap = s0 + T*v / 제동식 분해 / min(feedback, braking)
□ 구독 토픽            /opp_obs, /car_state/frenet/odom, /global_waypoints, /state
□ 발행 토픽            /cruise_speed_limit (std_msgs/Float64), /cruise/gap_data (f110_msgs/GapData)
□ 메시지 타입          위 각 항목에 명시
□ 주요 파라미터·YAML   config/cruise_controller.yaml (경로 명시)
□ 실행 방법            ros2 launch f1tenth_control control_real.launch.py cruise_enable:=true
□ 🔑 불변식            desired_gap <= obstacle_detector.yaml interference_distance_m
□ 🔑 실패 모드         state != CRUISE면 즉시 maximum_speed (B-6 계약과 함께 읽을 것)
```

⚠️ **빈 파일이나 `.gitkeep`으로 때우지 말 것.** install은 통과하지만 규칙 위반이고,
이 노드는 속도 상한을 쥐고 있어 문서 없이 튜닝하면 위험하다.

---

## D. 작업 순서와 검증

순서에 의존성이 있다. **B-2 → B-7**, **A-1/B-1 → 전체 재빌드** 두 곳이 특히 그렇다.

### D-1. 실행 순서

```bash
cd ~/2026_IFAC

# 0) 미커밋 크루즈 이식분부터 보존한다 (§0)
git add -A src/f1tenth_control && git commit -m "f1tenth_control: sungho_main 08/21 크루즈 스택 이식 + map_creator 정합 패치"

# 1) 빌드 블로커 — msg
git show transition_global:f110_msgs/msg/GapData.msg > f110_msgs/msg/GapData.msg   # A-1
#    Obstacle.msg 32행 뒤에 8줄 삽입                                                  # B-1

# 2) install 블로커 — 문서
mkdir -p src/f1tenth_control/docs
#    docs/cruise_controller_node.md 작성                                             # C-1

# 3) 불변식·안전 (코드/설정)
#    B-2 detector yaml 116행 / B-3 detector cpp 196행 / B-4 detector 문서 420행
#    B-5a cruise yaml 54~55행 / B-5b _control_common.py 96·100행
#    B-6a~c state_machine 3파일

# 4) 문서 정합 (B-2 적용 후에)
#    B-7 오참조 5곳

# 5) 전체 재빌드 — msg 해시가 바뀌었으므로 부분 빌드 금지
colcon build --symlink-install

# 6) 테스트
colcon test --packages-select f1tenth_control state_machine
colcon test-result --verbose
```

### D-2. 합격 기준

| 단계 | 확인 |
|---|---|
| 빌드 | `colcon build` 전체 성공 (**install 단계까지**) |
| 유닛 | `test_cruise_controller` 18케이스 + B-6d 신규 3케이스 통과 |
| 기동 | `ros2 node list \| grep cruise_controller_node` |
| 상한 발행 | `ros2 topic echo /cruise_speed_limit --once` |
| 진단 | `ros2 topic echo /cruise/gap_data` — `active_constraint` / `raw_gap` → `effective_gap` vs `desired_gap` |
| 🔑 리밋 사이클 없음 | `ros2 topic echo /state` — GLOBAL↔CRUISE 왕복이 없어야 한다 |
| 🔑 한 프레임 결측 내성 | 상대차를 잠깐 가려도 `/cruise_speed_limit`이 `maximum_speed`로 안 튀어야 한다 |

⚠️ **이 환경은 Humble이고 대상은 Jazzy다.** 여기서는 컴파일 오류 재현과 유닛테스트까지만
가능하다. 최종 빌드·런타임 합격은 **젯슨(Jazzy)에서 별도로** 판정할 것.

### D-3. 이후 (선택, 지금 하지 않는다)

```
□ Phase 2  검출기가 s_vs_cov / d_vd_cov 실제 생산 (Cauchy-Schwarz 클립 필수)
           → 그 뒤에 B-5를 1.0으로 되돌린다
□ Phase 2  간섭 판단을 state_machine으로 이관 (확률 술어)
           🔴 이관 시 allow_avoid_transition 파라미터 콜백 복원 필수 (map_creator 의존)
□ Phase 3  2-agent 시뮬 (num_agent: 2 + opponent_simulator)
□ Phase 3  sim/launch_cruise.zsh 죽은 맵 경로 수리 (monte_carlo_localization, ifac_track)
```

---

## E. 가져오면 안 되는 것

1. **`Obstacle.msg` 전체 교체** — `is_interfering`이 사라져 detector·state_machine 동시 파손.
2. **`state_machine/` 디렉터리 통째 이관** (Phase 2 전) — `transition_global` 버전은
   `allow_avoid_transition` 파라미터 콜백을 삭제했고, `map_creator`가 그걸 쓴다
   (`map_creator.yaml:13` `disable_avoid_after_swap`). 맵 스왑 후 회피 게이트가 조용히 죽는다.
3. **`obstacle_detector`의 `is_interfering` 제거** (Phase 2 전) — 소비자가 살아 있다.
4. **`local_planning` P3 재작성** — 크루즈와 무관하고 `map_creator`가 링크하는
   `raceline_planner` export 구조를 건드린다.
5. **U1 그립 클램프 / MPPI 제거 / `kinematic_localization` 마이그레이션** — 크루즈와 별개거나 이미 되어 있다.
6. **`src/f1tenth_control/f110_msgs/` 중첩 사본** — `colcon`이 `src/f1tenth_control`을 패키지로
   인식한 순간 하위를 안 훑으므로 **한 번도 빌드된 적이 없다.** 사람 눈에는 정본처럼 보여서
   `.msg`를 고칠 때 루트와 조용히 갈라진다. 이번 덮어쓰기로 19개 파일이 삭제됐다 — 되살리지 말 것.

---

## 부록 — 근거 확인 명령

```bash
# 빌드 블로커가 msg 2건뿐임을 확인 (13줄 / 12멤버 / cruise_controller_node.cpp 단일 파일)
colcon build --packages-select f1tenth_control 2>&1 | grep 'error:' \
  | grep -oP "has no member named .\K[a-z_]+" | sort -u

# docs 설치 블로커가 치명적임을 확인
grep -n "install(DIRECTORY docs" src/f1tenth_control/CMakeLists.txt
ls -d src/f1tenth_control/docs                       # 없음
git ls-tree --name-only sungho_main:src/f1tenth_control | grep docs   # 그쪽에도 없음

# 불변식 위반 확인
grep -n "interference_distance_m" src/obstacle_detector/config/obstacle_detector.yaml
grep -n "'trailing_gap'" -A2 src/f1tenth_control/launch/_control_common.py

# state_machine.yaml 오참조 확인 (그 파라미터가 거기 없음)
grep -n "interference" src/state_machine/config/state_machine.yaml    # 0건
grep -rn "state_machine.yaml.*interference_distance_m" src/f1tenth_control/

# 빈 프레임 즉시 해제 경로 확인
sed -n '405,415p' src/state_machine/src/state_machine_node.cpp
sed -n '228,231p' src/f1tenth_control/control_code/cruise_controller_node.cpp

# 전파 설정이 주석과 모순임을 확인
sed -n '51,55p' src/f1tenth_control/config/cruise_controller.yaml

# f110_msgs 의존 패키지 전수 (재빌드 대상)
grep -l "f110_msgs" src/*/package.xml
```
