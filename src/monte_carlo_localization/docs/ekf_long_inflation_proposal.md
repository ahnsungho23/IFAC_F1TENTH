# MCL 종방향 보정 복원 제안서 (v3)

- **작성**: 2026-08-17 (v1) → v2 (1차 리뷰 반영) → **v3** (2차 리뷰 반영, 동일)
- **대상**: `particle_filter_cpp` (`src/monte_carlo_localization/`)
- **근거 데이터**: `analysis_outputs/mcl_scanmatch_0816/` (bag `rosbag2_2026_08_16-15_46_36`),
  `analysis_outputs/run_0816_201629/` (bag `run_0816_201629`), 코드 감사·교차검증
  `analysis_outputs/run_0816_201629/mcl_eval_results.json`

### v2 → v3 주요 변경 (2차 리뷰 수용)

| # | 변경 | 사유 (전부 재현 검증됨) |
|---|---|---|
| 1 | §4 성능 예측을 **중앙차분 지표 기준으로 재산정**: e_ss 0.25 → **~0.33 m**, inflation>6 구간 37 → **47.9 %** | v2의 예측이 구(전방차분 float) 지표로 계산돼 있었음 |
| 2 | 관측성 계산 위치: sensor_model 내부 → **raw_pose 산출 직후** + recovery 사이클 폴백 | emergency recovery(`:1050-1057`)가 센서모델 뒤에 파티클을 교체 — 중앙차분은 기존 `ranges_` 정렬이 불필요하므로 raw_pose에서 계산이 정합적 |
| 3 | 의사코드 순서 교정: **s_scan을 먼저 계산** → long_inf 결정 → 적용 → s² → update. **P1에도 스캔품질 게이트** 추가 | 현행 s_scan은 `:1463-1473`의 지역변수(인플레이션 적용 뒤) — v2 의사코드는 스코프 불일치. 나쁜 스캔에서 44→6이면 s²가 양쪽에 곱해져도 상대 신뢰 7.3배 증가는 그대로 |
| 4 | "히스테리시스" 실체화: P1 = EMA + 연속 스케줄, **P2 발동 허가 = Schmitt 래치** | v2는 단어만 있고 메커니즘 없었음 |
| 5 | P3 정밀화: `lowspeed_trans_std_scale`로 개명(σ 계수 — **분산은 최대 ~100배**, 0.6 m/s에서 Q 약 46배·1.2에서 65배), `sigma_lat`도 결합 상승 명기, "여유 2.5배" → "레이싱 최저속 3.0 대비 **속도 분리** 2.5배" | `:1197-1198` σ_long 식·횡방향 결합 확인 |
| 6 | Stage 0 스펙 확정: raw_pose 공분산 Jacobian 변환, 시간축 필드 6종, 진단 메시지 = `diagnostic_msgs/DiagnosticArray`, `ekf_update` 결과 enum, **차체 종방향 스칼라 = u-투영 정의** | 파티클은 laser 프레임(`:1828`), `ekf_update`는 void+중간 return(`:1213`), K(0,0)은 map x축 게인 |
| 7 | **신규 리스크 승격**: R을 낮추면 "게이트 연속 기각 20회(0.5 s) → free-space 앨리어스로 force reset" 경로가 새로 열림 — 검증 지표·정책 개편 | `is_pose_permissible`은 벽 속만 거부 |
| 8 | 재현성 강화: 시드 + **재생률 고정 + 구성당 3회 반복**, "비트 동일" → 고정 이벤트 시퀀스 단위테스트 수치 동일 | wall timer·`try_lock`(`:1382`)·startup 15→40 Hz 전환으로 스레드 스케줄 의존 |
| 9 | 맵 출처 규정: 캘리브레이션·채점은 **bag `/map` 추출본** 사용 + 해시·origin 기록. ⚠️ 부수 발견: 저장소 `maps/map.png`(md5 `b0be...`, origin `[0,0,0]`)는 **운용 맵(md5 `b7a0...`, origin `[-5.283,-1.147]`)과 다른 파일** | 분석 스크립트의 경로 하드코딩 지적 → 검증 중 확인 |
| 10 | §12 신설: 파일 구조·수정 명단 | — |

<details><summary>v1 → v2 변경 이력 (접힘)</summary>

P4를 전제조건(Stage 0)으로 승격 / "전 구간 6" 사실 정정 / 지표 정수 셀·EMA 재설계 /
P2 명칭·AND 조건 강화 / P3 유지 + 임계 1.2 / 파라미터 이중정의 제거·enable 3분리·기본 false /
합격 기준 \|Δ출력−Δodom\| 교체 / 안전성 주장 조건부화 / random_seed 신설
</details>

---

## 1. 요약

`/pf/pose/odom`이 실제 위치보다 **상시 약 0.5 m 앞서** 발행된다.

- **확정**: ① 기동 스톨의 팬텀 전진 odom 주입(+0.735 m/기동) ② `ekf_meas_long_inflation: 44`
  에 의한 종방향 보정 차단 메커니즘 존재(v=0 게인 붕괴 포함) ③ 게이트·재고정 안전망 발화 불능.
- **미확정**: 관측된 0.5 m의 소재지 — (a) raw MCL 정확 + EKF 차단 (b) raw MCL도 편향
  (c) 혼합. **raw MCL이 발행되지 않아 분리 불가** → Stage 0/1이 선행돼야 하는 이유.

| 단계 | 내용 | 상태 |
|---|---|---|
| Stage 0 | P4: raw MCL + EKF 내부 진단 토픽, random_seed | 스펙 확정(§5) — 구현 착수 가능 |
| Stage 1 | 두 bag 재생으로 raw MCL 정확성 검증 (3분법 판별) | Stage 0 후 |
| Stage 2 | 상수 인플레이션 스윕 {44,25,12,9,6,2} | Stage 1 후 |
| Stage 3 | P1: 관측성 조건부 인플레이션 (중앙차분 지표 + 합성 복도 캘리브레이션) | 스윕 후 |
| Stage 4 | P2′: 조건부 정지 종방향 게인 부스트 | Stage 3 후 |
| Stage 5 | P3: 저속 odom 불신 (임계 1.2, 유지 확정) | Stage 3과 병행 가능 |
| 실차 활성화 | **force-accept 앨리어스 시험(§8) 통과 전 보류** | — |

---

## 2. 문제 정의 — 실측 증상

| 항목 | bag 15:46 (482프레임) | bag 20:16 (435프레임) |
|---|---|---|
| 위치 오차 \|Δ\| 중앙값 | **0.544 m** | **0.529 m** (전체 0.556) |
| p90 / 최대 | 0.899 / 1.281 m | 0.915 / 1.212 m |
| 종방향 비중 | **87 %** (MCL이 앞섬) | 동일 경향 |
| 속도 상관 | lon = +0.0025·v − 0.391, **R² = 0.000** | — |
| v=0 정지 중 | **0.50~0.59 m 동결** | 출발 전 +0.184 m 선재 |
| 맵 정합률 (레이 10 cm 이내) | MCL 0.28 vs 참조 0.94 | MCL 0.22 vs 참조 0.86 |

**제어 파급** (bag 15:46): L1 `sin η` 부호 반전 11.3 %, 조향 요구 오염 중앙값 7.7°,
풀락 좌우 진자(0.5 s 주기) → 두 bag 모두 e-stop 종료.

> **참조 궤적의 한계**: 스캔매칭은 알고리즘은 독립이지만 같은 LiDAR·같은 맵을 쓰므로
> 절대 참값은 아니다. 제어가 맵 프레임 레이스라인을 추종하므로 맵-상대 위치가 목적에
> 맞는 기준이며, 정합률 0.94 vs 0.28의 격차는 이 한계로 뒤집히지 않는다. 유보 구간:
> 출발 구역(스캔매처 쌍안정, ±0.18 m)과 맵 불일치 구간(GAP1~3, 최대 0.70 m).

---

## 3. 원인 분석

### 3-1. [확정] 주입: 센서리스 기동 스톨의 팬텀 전진

bag 20:16 기동 구간(5.3~10.15 s):

- 스톨 4회 (6.75/7.96/9.02/10.08 s) — rpm이 명령의 27 %에서 급락, 전류 60~62 A
  스파이크, 40 A 포화 3.26 s. **vx 부호 반전 0회** → "전후 진동"이 아니라 FOC 관측기
  desync의 팬텀 양의 ERPM.
- 휠 odom 적분 **1.404 m** vs 참 변위 **0.669 m** (+0.735 m, 2.1배). 스톨 코스팅 중엔
  반대로 언더카운트(참 0.213 vs 휠 0.001 m).
- MCL 종방향 오차 +0.184 → **+0.45~0.48 m 계단 성장** 후 지속.

EKF 예측은 odom 포즈 델타를 무검증 가산 — 방어는 1주기 1 m 가드뿐(`:1171-1174`).

### 3-2. [확정] 차단 메커니즘의 존재

`particle_filter.cpp:1452`: 차체 종방향 측정 분산 ×44 (σ ×6.63).

- 정상상태 폐합: `e_ss = (0.012/0.03)·6.63·σ_cloud·s_scan ≈ 0.53 m` (σ=0.10, s=2) —
  실측 0.53~0.56 재현, 속도 약분으로 무상관까지 재현.
- v=0 동결: `Q ∝ ds` → 플로어 6.25e-8, K ≈ 3.8e-4/사이클 (τ ≈ 66 s, s_scan 부풀림 시
  130 s+). **raw MCL 정확성과 무관하게 성립.**
- 안전망 발화 불능: maha² ≈ 0.5 ≪ 25 → 상시 수용 → force_accept(20회/1.5 m) 죽은 경로.
- `use_scan_quality_r`: 틀린 포즈일수록 R 최대 ×20 추가 (악순환).

### 3-3. [미확정] 오차의 소재지 — 3분법

센서모델 완화(σ_hit 0.35 m, floor 0.05, squash 0.4, 61레이)로 **구름 자체도 편향
가능**. 상태 (a) raw 정확/EKF 차단 → 인플레이션 완화로 해결, (b) raw도 편향 →
센서모델·리샘플링 수정 필요, (c) 혼합. **판별은 Stage 0+1로만 가능.**

---

## 4. 설계 의도와 전제 검증

44의 의도 — "특징점 없는 (고속) 직선에서 odom 신뢰" — 는 정당하며(08-03 복도 스냅
방어 실적), v3도 유지한다. 문제는 **무조건부**라는 것뿐이다.

레이스라인 165점 관측성(§4-1의 **중앙차분 ±10 cm·≥2셀 지표** 기준, v3 재산정):

| 지표 | 값 |
|---|---|
| 종방향 정보 레이 | min 4 / p10 6 / med 25 (61레이 중) |
| 복도성(< 8) | **27 / 165 = 16 %** |
| 스케줄(8↔24) 적용 시 inflation | med 6.0 / mean 20.1 / p90 **44** / **6 초과 47.9 %** |
| 예상 정상상태 오차 (동일 가정) | **≈ 0.33 m** (mean √inflation 4.08 기준) |

> v2의 "37 % / 0.25 m"는 구(전방차분) 지표로 계산된 값이라 폐기한다. 0.33 m는
> lo/hi를 아직 캘리브레이션하지 않은 보수적 추정이며, Stage 3에서 합성 복도로 lo/hi를
> 확정한 뒤 재산정한다. 참고로 개선 폭의 절반 이상은 지표와 무관한 저관측성 구간
> 외(코너·특징 구간)에서 나온다.

### 4-1. 지표의 수치 결함 — 재설계 근거 (v2 유지)

| 판정 기준 | min | p10 | med | 복도성(<8) |
|---|---|---|---|---|
| \|Δr\| > 0.050 (v1, float) | 9 | 16 | 31 | 0 |
| \|Δr\| > 0.051 (+1 mm) | **1** | 4 | 14 | 다수 |
| 정수 셀 ≥ 1 (전방차분) | 17 | 22 | 40 | 0 |
| **중앙차분 ±10 cm, ≥ 2셀 (채택)** | 4 | 6 | 25 | 27 |

float `>0.05`는 1셀 케이스에서 부동소수점 반올림에 따라 레이마다 비일관 판정(수치적
정의 불량). 채택 지표도 국소 민감도 휴리스틱이지 엄밀한 관측성(Fisher 조건부 정보)이
아니므로 — 레이 상관·yaw/횡방향 혼동·전역 다봉성 비구분 — lo/hi는 반드시 합성 복도
캘리브레이션으로 확정한다.

---

## 5. 제안 — 단계별 실행 계획

### Stage 0 — P4: 관측 토픽 + 재현성 (전제조건, 거동 무변경)

**`/pf/viz/raw_pose`** (`geometry_msgs/PoseWithCovarianceStamped`):

- 내용: EKF 이전 파티클 기대포즈(지연보상 전 raw). 프레임: base_link (laser 레버암
  `:1828` 차감).
- 공분산: 파티클 가중 공분산을 laser→base **Jacobian 변환** `Cov_base = J·Cov_laser·Jᵀ`
  (레버암 0.27 m × yaw 분산 결합 — 효과는 수 mm~cm 수준이지만 명세로 고정).
- stamp: `last_lidar_time_`. ⚠️ 시간축 캐비앗: 현행 파티클 전파는 스캔 타임스탬프가
  아니라 steady-clock dt + 최신 twist(`:1024`, `:1356`)이므로 raw_pose는 엄밀한
  스캔 시각 상태가 아니다. Stage 1 해석 시 아래 진단의 시간 필드로 보정하고, 근본
  해결(스캔 타임스탬프 기반 모션)은 §10 후속(AE-HYU 이식 ③)과 연결된다.

**`/pf/viz/ekf_diag`** (`diagnostic_msgs/DiagnosticArray` — Header 보유, 루트
CLAUDE.md 메시지 정책상 신규 타입보다 표준 우선. KeyValue로 기록):

- 시간: scan stamp / 최신 odom stamp / timer now / lidar_age / motion dt
- 포즈: 지연보상 전·후 z, EKF 상태
- 스칼라(**차체 종방향 = u-투영**, u = [cos ψ, sin ψ], K(0,0) 같은 map축 성분 금지):
  `innov_long = uᵀ(z−x)_xy`, `R_long = uᵀR_xy u`, `P_long = uᵀP_xy u`, `K_long = uᵀK_xy u`
- 판정: maha², `ekf_update` 결과 enum {ACCEPTED, GATE_REJECTED, FORCE_REANCHORED,
  FORCE_TARGET_NOT_PERMISSIBLE, RESET} — 현행 `void ekf_update`(`:1213`, 게이트 시
  중간 return)를 **결과 구조체 반환으로 변경**
- 상태: info_rays(raw/EMA), 적용 long_inf, s_scan, 정지/저속/recovery 플래그

**`random_seed`** (0 = 현행 `random_device`(`:24`), >0 = 고정).

### Stage 1 — raw MCL 정확성 검증

두 bag 재생(고정 시드·고정 재생률·구성당 3회)으로 raw_pose vs EKF 출력 vs 스캔매칭
3자 대조. raw가 참조 ≤0.15 m면 상태 (a) → Stage 2~5 진행. raw도 0.3 m+ 편향이면
상태 (b)/(c) → 센서모델·리샘플링 트랙(AE-HYU ESS 게이트 + low-variance 이식)을 본
계획에 편입하고 인플레이션 완화 단독 진행 금지.

### Stage 2 — 상수 스윕

`ekf_meas_long_inflation ∈ {44, 25, 12, 9, 6, 2}` — adaptive 없이 R 효과 분리,
게이트 기각률·**force reanchor 발화**·오차의 트레이드오프 곡선 확보 (§7 수치의 실증).

### Stage 3 — P1: 관측성 조건부 인플레이션

- **지표**: 정수 셀 중앙차분 (±`long_obs_half_span_m`=0.10, 총 span 0.20 m,
  `≥ long_obs_cells_min`=2셀) + 시간 EMA. **P1 스케줄은 연속(래치 없음)**,
  P2′ 발동 허가만 Schmitt 래치(§ Stage 4).
- **계산 위치** (v3 교정): `raw_pose = expected_pose()` 직후, **raw_pose 기준으로**
  ±half_span 재캐스팅. 중앙차분은 기존 `ranges_` 정렬이 불필요하므로 sensor_model
  내부일 이유가 없고, emergency recovery(`:1050-1057`) 이후의 실제 측정 분포와
  일치한다. **recovery 발동 사이클은 adaptive 비활성(44 폴백)**.
- **P1 자체 발동 조건** (v3 신설): `s_scan < adaptive_scan_quality_max` — 관측성은
  맵 기준 계산이라 실제 스캔이 맵과 다른 구간(환경 변화)에서는 특징이 "있다고" 오판
  할 수 있으므로, 스캔품질 나쁠 때는 44 폴백.
- **캘리브레이션**: 합성 복도 맵(무특징 평행벽)에서 확실히 lo 미만, 실트랙 코너에서
  확실히 hi 초과가 되도록 lo/hi 확정 → §4 성능 재산정.
- CPU: 젯슨 wall-time 실측 (레이 수 기준 +0.4 %(2회 캐스팅)이나 호출 오버헤드 별도).

### Stage 4 — P2′: 조건부 정지 종방향 게인 부스트

- **발동 조건 (전부 AND)**: 정지 상태머신 활성 **AND** `long_observable_` 래치 참
  (Schmitt: EMA ≥ hi에서 set, ≤ lo에서 clear — **복도 정지 시 미발동**) **AND**
  `s_scan < standstill_scan_quality_max` **AND** odom 신선(수신 간격 < 0.2 s).
- 적용: `long_inf = min(long_inf, standstill_long_inflation)` — 낮추는 방향으로만.
- **정지 상태머신** (ROS time 기준): 진입 `|v| < 0.05`가 0.3 s 지속 / 해제
  `|v| > 0.15` / odom 미수신·시간 역행(bag 루프) 시 리셋 / 기동 직후 비활성.
- 정직한 기대치: inflation 2 기준 τ ≈ 14 s(s=1)~28 s(s=2) — "출발 전 소거 보장"이
  아니라 정지 시간에 비례(그리드 대기 수십 초에서 실효). raw MCL 자체가 편향이면
  무효 — Stage 1 결과에 종속.

### Stage 5 — P3: 저속 odom 불신 (유지 확정, 임계 1.2)

bag 20:16 기동 크리프(5.3~10.15 s) 휠 적분 1.404 m의 속도대역 분해:

| vx 대역 | 적분 | 비중 |
|---|---|---|
| < 0.6 | 0.630 m | 44.9 % ← v1 임계 0.6의 커버리지 |
| 0.6~1.2 | 0.773 m | 55.1 % (스톨 램프 스파이크 0.70~1.08) |
| ≥ 1.2 | 0 m | 0 % |

실주행 v<1.2 체류 0.00 s (두 bag), 실주행 최저 속도 3.02/3.35 m/s → 임계 1.2는
팬텀 대역 100 % 커버, 레이싱과 **속도 분리 2.5배** (팬텀 최대 1.08 대비 여유가 아님).

- **의미 정밀화** (v3): `ekf_trans_error_rate`는 σ 계수다 — ×10은 **분산 최대
  ~100배**이고, 40 Hz·플로어 포함 실효는 0.6 m/s에서 종방향 **Q 약 46배**, 1.2에서
  **약 65배**, 완전 정지(ds=0)에선 플로어만 남아 무변화. `sigma_lat = 0.3·sigma_long`
  (`:1198`) 결합으로 **횡방향 노이즈도 함께 상승**함을 명기.
- 정직한 한계: 팬텀 평균값 자체는 그대로 가산된다(차단 아님) — 효과는 크리프 중
  매 스캔(~194회/4.85 s) 회수 게인 상향이며, **raw MCL이 유효할 때만**(Stage 1) 작동.
- 2차 업그레이드(필요 시): `/sensors/core` rpm·전류 직접 스톨 검출.

---

## 6. 구현 상세

```cpp
// ── timer_update, MCL() 완료 후 (v3: 계산 순서 전면 교정) ───────────────────────
Eigen::Vector3d raw_pose = expected_pose();

// [0] 스캔 품질을 '먼저' 계산 (현행 :1463-1473의 지역 계산을 앞으로 이동)
double s_scan = 1.0;
if (USE_SCAN_QUALITY_R) s_scan = 1.0 + K_OUT * q_term + K_ESS * ess_term;

// [1] 종방향 관측성 — raw_pose 기준 ±half_span 중앙차분 (정수 셀), recovery 시 무효
int changed = count_long_info_rays(raw_pose, LONG_OBS_HALF_SPAN_M, LONG_OBS_CELLS_MIN);
long_obs_ema_ = ema(long_obs_ema_, changed, LONG_OBS_EMA_ALPHA);
// P2' 허가용 Schmitt 래치 (P1 스케줄은 연속)
if (!long_observable_ && long_obs_ema_ >= LONG_OBS_RAYS_HI) long_observable_ = true;
else if (long_observable_ && long_obs_ema_ <= LONG_OBS_RAYS_LO) long_observable_ = false;

// [2] long_inf 결정 — 기존 ekf_meas_long_inflation(44)이 최대값/폴백
double long_inf = EKF_MEAS_LONG_INFLATION;
bool obs_valid = !emergency_recovery_fired_ && (s_scan < ADAPTIVE_SCAN_QUALITY_MAX);
if (USE_ADAPTIVE_LONG_INFLATION && obs_valid) {
    double t = std::clamp((long_obs_ema_ - LONG_OBS_RAYS_LO) /
                          (LONG_OBS_RAYS_HI - LONG_OBS_RAYS_LO), 0.0, 1.0);
    long_inf = EKF_MEAS_LONG_INFLATION * (1.0 - t) + LONG_INFLATION_MIN * t;
}
if (USE_STANDSTILL_LONG_GAIN && standstill_ && long_observable_
    && s_scan < STANDSTILL_SCAN_QUALITY_MAX && odom_fresh_)
    long_inf = std::min(long_inf, STANDSTILL_LONG_INFLATION);

// [3] R 구성: 종방향 인플레이션 → [4] 전체 s_scan² → [5] EKF 갱신(결과 enum 기록)
cov_body(0,0) = std::max(cov_body(0,0) * long_inf, POS_FLOOR * POS_FLOOR);
...
meas_cov *= s_scan * s_scan;
EkfUpdateResult res = ekf_update(z, meas_cov);        // void → 결과 구조체 (Stage 0)

// ── ekf_predict_from_odom (:1197 부근, Stage 5) ────────────────────────────────
const double rate = (USE_LOWSPEED_PROCESS_NOISE &&
                     std::abs(current_velocity_) < LOWSPEED_THRESH_MPS)
                    ? EKF_TRANS_ERROR_RATE * LOWSPEED_TRANS_STD_SCALE
                    : EKF_TRANS_ERROR_RATE;
const double sigma_long = rate * ds + EKF_TRANS_FLOOR_MPS * dt_nom;   // 실제 사용 지점
```

### 파라미터 (v3)

| 파라미터 | 코드 기본 | 실차 YAML (검증 후) | 비고 |
|---|---|---|---|
| `ekf_meas_long_inflation` | 25.0 (기존) | 44.0 (기존 유지) | adaptive 최대값/폴백 겸용 |
| `use_adaptive_long_inflation` | **false** | true | Stage 3 통과 후 |
| `long_inflation_min` | 6.0 | Stage 2 스윕으로 확정 | 1 ≤ min ≤ max 검증 |
| `long_obs_rays_lo` / `hi` | 8 / 24 | 합성 복도 캘리브레이션 후 | hi > lo 검증 |
| `long_obs_half_span_m` | 0.10 (**±0.10, 총 0.20 m**) | 〃 | v2 `delta_m` 개명 |
| `long_obs_cells_min` / `ema_alpha` | 2 / 0.2 | 〃 | |
| `adaptive_scan_quality_max` | 1.5 | 〃 | **P1 폴백 게이트 (v3 신설)** |
| `use_standstill_long_gain` | **false** | true | Stage 4 통과 후 |
| `standstill_long_inflation` | 2.0 | 〃 | |
| `standstill_speed_mps`/`exit_mps`/`hold_sec` | 0.05/0.15/0.3 | 〃 | ROS time 기준 FSM |
| `standstill_scan_quality_max` | 1.5 | 〃 | |
| `use_lowspeed_process_noise` | **false** | true | Stage 5 |
| `lowspeed_thresh_mps` | 1.2 | 1.2 | §5 실측 근거 |
| `lowspeed_trans_std_scale` | 10.0 | 10.0 | **σ 계수** (분산 ≤100배), v2 개명 |
| `random_seed` | 0 | 0 (A/B 시 고정) | |
| `publish_raw_pose` / `publish_ekf_diag` | true | true | Stage 0 |

- sim YAML에 enable 3종 **명시적 false**. 파라미터는 생성자 1회 읽기 — 변경 시
  **노드 재시작 필요**, 롤백 = flag false + 재시작 (재빌드 불필요).
- 기동 시 범위 검증: `hi > lo`, `1 ≤ min ≤ ekf_meas_long_inflation`, `half_span > 0`,
  `cells_min ≥ 1`, `scale ≥ 1`, 속도·시간 ≥ 0 — 위반 시 경고 + 해당 기능 자동 비활성.

---

## 7. 안전성 논증 (조건부)

1. **복도 보존은 검증 대상**: P1은 합성 복도에서 분류기가 lo 미만임을 확인한 뒤에만
   "복도에서 44 유지"를 주장할 수 있다. P2′는 `long_observable_` 래치로 복도 정지 시
   미발동. P3는 저속 한정.
2. **게이트 재무장은 경계값**: inflation 6·σ 0.1·s 2에서 2.5 m 스냅 maha² ≈ 26 vs
   문턱 25 — s_scan 4.5면 5.1로 수용. Stage 0 진단의 실제 P/R/s_scan 로그로 확인.
3. **[v3 신설] force-accept 앨리어스 경로 — R 완화가 여는 신규 리스크**:
   현행(44)은 게이트가 전부 수용하므로 force reset이 없지만, R을 낮추면
   `앨리어스 측정 → 게이트 연속 기각 20회(40 Hz ≈ 0.5 s) → free-space 앨리어스로
   ekf_reset → 수 m 스냅` 경로가 살아난다. `is_pose_permissible`은 벽 속만 거부한다.
   따라서 **force-reset 정책 자체를 별도 안전 과제로 편성**한다 — 후보: force_accept
   횟수/거리 상향, 재고정 전 N사이클 측정 정합(연속 저혁신) 요구, adaptive 활성 중
   재고정 시 long_inf 일시 44 복귀. §8 지표로 채점한다.

---

## 8. 검증 계획

### 재생 A/B (고정 시드 + **고정 재생률** + 구성당 **3회 반복**, 스캔매칭 채점)

| 구성 | 내용 |
|---|---|
| A | 현행 (상수 44) |
| B | 상수 스윕 {25,12,9,6,2} |
| C | P1 / D | P1+P2′ / E | P1+P2′+P3 |

× bag 2개 + **합성 복도 시나리오** (평행벽 맵, 2.5 m 앨리어스 측정 주입).
채점 맵은 **bag `/map` 추출본**을 쓰고 해시·resolution·origin을 결과에 기록한다
(⚠️ 저장소 `maps/map.png`는 운용 맵과 다른 파일 — origin `[0,0,0]` — 채점에 사용 금지).

### 합격 기준 (v3 — force reset 분리 계상)

| 지표 | 기준 |
|---|---|
| 참조 대비 오차 중앙값 | ≤ 0.30 m (§4 재산정 기준, lo/hi 확정 후 갱신) |
| 코너 구간 오차 중앙값 (별도 집계) | ≤ 0.30 m |
| 1주기 순보정량 \|Δ출력 − Δodom\| (**명시적 재고정 제외**) | p99 ≤ 0.05 m, 최대 ≤ 0.15 m |
| **force reanchor 횟수 — 정상 bag** | **0회** (발생 시 그 자체로 불합격) |
| 합성 복도 앨리어스: reanchor 횟수·이동량 | 현행(44) 대비 악화 없음 |
| 기동 완료 + 8 s 잔차 | ≤ 0.2 m (현행 0.45~0.48) |
| 게이트 연속 기각 | 참고 지표로만 (force reset이 카운터를 자르므로 단독 안전 지표 아님) |

### 단위 테스트 (오프라인 결정론 — "노드 출력 비트 동일" 대신 고정 이벤트 시퀀스 수치 동일)

복도/끝벽/코너 관측성, 5 cm 양자화 경계, Schmitt 래치 set/clear, 정지 FSM
진입·해제·시간 역행, odom stale, flag 전부 false 시 현행 수식과 수치 동일.

---

## 9. 리스크와 롤백

| 리스크 | 대응 |
|---|---|
| raw MCL 자체 편향 (상태 b/c) | Stage 1 판별 — 해당 시 센서모델·리샘플링 트랙 편입, 인플레이션 완화 단독 진행 금지 |
| **force-accept 앨리어스 스냅 (R 완화의 신규 경로)** | §7-3 정책 과제 + §8 앨리어스 시험 통과 전 실차 보류 |
| 관측성 오판 (맵≠환경 구간) | P1 자체에 s_scan 폴백(v3) + 합성 복도 캘리브레이션 + 최악에도 min=6 |
| 게이트 오기각 | Stage 2 스윕에서 기각률 실측, `ekf_gate_chi2` 재조정 |
| 예상 밖 거동 | flag false + 재시작 → 현행 복원 (단위 테스트로 수식 동일성 보장) |

주의: 첫 bag의 주 오차 성장은 코너형(0.04→0.57 m) — 코너 구간 별도 집계의 이유.

## 10. 후속 과제 (본 제안과 별건)

- AE-HYU 통째 교체 반대(근거: `mcl_eval_results.json`). 이식 후보: ① RTR odom-delta
  모션모델 ② ESS 게이트 + low-variance 리샘플링(상태 b/c 시 본 계획 편입)
  ③ **스캔 타임스탬프 정합 TF 모션 계산** — Stage 0 시간축 캐비앗(§5)의 근본 해결.
- 맵 정비: 운용 맵 ↔ 저장소 `maps/` 불일치 해소, GAP1(0.70 m) 구간 재매핑.
- VESC 오픈루프 기동 시퀀스 개선 (하드웨어 파트).

## 11. 작업량 추정

| 단계 | 규모 |
|---|---|
| Stage 0 | ~120줄 (토픽 2 + `ekf_update` 결과 구조체 + 시간 필드 + seed) |
| Stage 1 | 재생 파이프라인 재사용, 반나절 |
| Stage 2 | 스크립트 반나절, 재생 6런 × 2 bag × 3회 |
| Stage 3 | ~100줄 + 합성 맵 제작·캘리브레이션 1일 |
| Stage 4 | ~50줄 (ROS-time FSM 포함) |
| Stage 5 | ~10줄 |
| 단위 테스트 + 문서 | 1~1.5일 |

---

## 12. 파일 구조 — 수정·신규 명단

```
src/monte_carlo_localization/
├── src/
│   └── particle_filter.cpp                [수정] Stage 0/3/4/5 본체:
│                                            · ekf_update void → EkfUpdateResult 반환 (:1213)
│                                            · s_scan 계산 위치 이동 (:1463 → R 구성 전)
│                                            · count_long_info_rays() 신설 (raw_pose 기준)
│                                            · 정지 FSM·Schmitt 래치·저속 σ 스케일 (:1197)
│                                            · raw_pose/ekf_diag 발행, random_seed (:24)
├── include/particle_filter_cpp/
│   └── particle_filter.hpp                [수정] 멤버(long_obs_ema_, long_observable_,
│                                            standstill_), EkfUpdateResult 구조체·enum,
│                                            신규 파라미터 선언
├── config/
│   ├── mcl_config.yaml                    [수정] 신규 파라미터 17개 (실차값, §6 표)
│   └── mcl_config_sim.yaml                [수정] enable 3종 명시 false
├── launch/
│   └── mcl_launch.py                      [변경 불필요] (launch는 YAML 미오버라이드 —
│                                            진단 토픽 remap 필요 시에만 손댐)
├── maps/
│   └── synthetic_corridor.{png,yaml}      [신규] 무특징 평행벽 캘리브레이션 맵
│                                            ⚠️ 기존 maps/map.png는 운용 맵과 다름 — 채점 금지
├── test/
│   └── test_long_inflation.cpp            [신규] §8 단위 테스트 (gtest)
├── CMakeLists.txt                         [수정] diagnostic_msgs 링크, test 타겟,
│                                            합성 맵 install
├── package.xml                            [수정] <depend>diagnostic_msgs</depend> 추가
│                                            (f110_msgs는 기존 보유)
├── AGENTS.md                              [수정] 노드 정책 갱신 (루트 CLAUDE.md 정책 —
│                                            신규 파라미터·토픽·검증 절차 반영)
└── docs/
    ├── particle_filter.md                 [수정] 신규 토픽·파라미터·동작 문서화
    └── ekf_long_inflation_proposal.md     [본 문서]

analysis_outputs/mcl_ab/                    (검증 인프라 — 저장소 분석 영역, 신규)
├── make_corridor_map.py                   합성 복도 생성 + 앨리어스 주입 시나리오
├── replay_ab.sh                           bag × 구성 매트릭스 재생 (시드·재생률 고정)
├── grade_run.py                           스캔매칭 채점 (run_0816_201629/work/match_scan.py 재사용,
│                                            맵 = bag /map 추출본 + 해시 기록)
└── configs/                               구성 A~E YAML 오버레이
```

**구현 순서 요약**: `package.xml`/`CMakeLists` → `hpp`(구조체·파라미터) →
`cpp` Stage 0 → 빌드·bag 스모크 → `mcl_ab/` 채점 인프라 → Stage 1 판정 →
이후 단계는 판정 결과에 따라.
