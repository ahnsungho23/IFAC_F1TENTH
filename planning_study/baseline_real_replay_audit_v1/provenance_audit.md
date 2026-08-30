# P3 oracle 전 provenance audit

## 결론

이 corpus에서 exact historical Git commit은 네 bag 모두 확인되지 않았다. 네 bag 모두 현재
`research_instrumentation_v1`과 동일한 wire schema를 쓰고, 기록된 P3 payload는
`local_planning_p3_cycle/1`, `TEST_ACTIVE` 계열임을 직접 확인했다. 일부 planner runtime
parameter도 `/rosout`에서 복원할 수 있다. 따라서 네 bag의 historical source status는 모두
`SOURCE_VERSION_PARTIALLY_INFERRED`이며 `EXACT_SOURCE_VERSION_KNOWN`이 아니다.

현재 baseline 통계의 해석 대상은 다음 중 **B**다.

- **A. ORIGINAL HISTORICAL PLANNER BEHAVIOR**: 녹화 당시 실행 중이던 코드와 당시 설정이 만든
  `/avoid_waypoints`, `/local_planning/p3_shadow`, safe-stop 및 drive/state 결과.
- **B. FROZEN-BASELINE REPLAY BEHAVIOR**: `research_instrumentation_v1`의 exact binary/config가
  bag에 기록된 `/global_waypoints`, obstacle, Frenet odometry, state 입력을 받아 새로 계산한 결과.

기록 시점의 perception/localization/global-planning 산출물을 B의 입력으로 사용했으므로 B도
완전히 새로 생성한 end-to-end current-stack 결과는 아니다. 그러나 planner source/config와
failure distribution은 frozen tag의 것이다. 기존 baseline 통계와 `footprint_track_bound`
107,946건은 B로만 해석한다.

## 1. Footprint history

### 1.1 Git 계보

아래 표의 main-line integration commit은 모두 현재 tag의 ancestor다. `reason evidence`에 별도로
적은 commit은 같은 변경의 상세 근거를 남긴 다른 branch의 commit이며 현재 tag ancestor가 아닐
수 있다. 따라서 상세 근거 commit과 production 계보 commit을 같은 것으로 오인하면 안 된다.

| Date (KST) | Production-line commit | Old | New | 의미와 recoverable reason |
|---|---|---|---|---|
| 2026-08-08 | `30e7304372d216098a0b356c05fe635f4f8afd44` | half width `0.121`; safety `0.030`; no tracking reserve | half width `0.1435` (full width `0.287`); safety `0.0582293`; tracking reserve `0.140` | Subject가 `ego차량 크기 수정`; diff는 `0.287 m`를 실차 폭이라고 명시한다. 물리 폭은 커졌다. |
| 2026-08-08 | `2b35daccd7556fa448377ea9df268f3609428bd6` | safety `0.0582293`; wall reserve 없음 | safety `0.0147893`; wall reserve `0.04` | CMA-ES 적용값. 이때 track bound는 아직 vehicle-center limit으로 해석했다. |
| 2026-08-12 | `107368013a812b49ff518f3767555b8f3c3eb3ac` | length parameter 없음; center `d`만 track interval에 검사; `d_left/right`를 이미 차체 폭이 반영된 center limit으로 간주 | length `0.56`; `0.56 x 0.287 m` 회전 직사각형 네 corner 검사; `d_left/right`를 reference-to-physical-boundary 거리로 재해석; `footprint_track_bound` hard rejection 추가 | Commit subject는 `1`이라 설계 이유는 복원 불가. diff와 주석은 물리 경계 semantics 및 rotated-corner projection을 명시한다. |
| 2026-08-13 | `70c5a9d830a97844c365d2194e2c0cf49ef94225` | branch-local pre-sync state | 위 rotated footprint 구현 동기화 | 현재 계보의 package sync. 8/12 introduction과 동일 계열 구현이다. |
| 2026-08-20 | `4437eea0912e4d15c32214385d303794b4be6292` | wall `0.04`; localization reserve `0.06`; ranking wall room은 center 기준 | wall `0.10`; localization reserve `0.12`; ranking wall room은 body+tracking tube 기준 | 상세 근거는 non-ancestor `ac17f4a...`: 실제 벽 접촉과 계획/실제 pose residual을 근거로 wall `0.10` 선택. Hard footprint gate는 wall margin을 한 번만 적용하고, ranking 변경은 feasible set을 바꾸지 않는다. |
| 2026-08-20 | `5895516f4fc8b15d0621b24d3f71de92b29327eb` | half width `0.1435`; safety `0.0147893`; localization `0.12`; tracking `0.14` | half width `0.15`; safety `0`; localization `0`; tracking `0.20` | 물리 폭 변경의 상세 근거는 non-ancestor `4e10e8f...`: measured `0.287 m`를 operational `0.300 m`로 올림, 좌우 각 `6.5 mm` 증가. |
| 2026-08-21 | `b2cd573fc6a0ae27c15300d5c70952638ec4dea0` | half width `0.15`; zero safety/localization | half width `0.1435`; safety `0.0147893`; localization `0.12`; tracking `0.14` | `max value` commit에서 이전 설정으로 되돌림. 상세 reason은 commit에 없다. |
| 2026-08-23 | `1b9e314177e4c219f5ff7b8c9d676a05a3ae7606` | half width `0.1435`; safety `0.0147893`; wall `0.10`; localization `0.12`; tracking `0.14` | half width `0.15`; safety `0`; wall `0.04`; localization/tracking `0`; reserve mode `none` | 상세 근거는 non-ancestor `42b2109...`: competition free-width `0.50 m` 예산에서 wall/obstacle margin을 재배분. 물리 차체를 줄인 변경은 아니다. |
| 2026-08-24 | `d3ca5b5902baf1c390881a930074d4a44285a28b` | safety `0` | safety `0.05` | 상세 근거는 non-ancestor `9035dd9...`: obstacle 실물 크기 확인과 충돌 여유를 반영. Track-footprint wall gate가 아니라 obstacle clearance 변경이다. |
| 2026-08-26 | `f2f6c8a29d2b6ee41e97cbfba6939548cc6c5092`, `191703d26d8a5ec1ff778f9d606ce7560a8a529d` | safety `0.05` | safety `0.08` | `191703d...`는 Jetson 실물 튜닝 반영이라고 명시한다. Track-footprint wall gate에는 safety margin이 들어가지 않는다. |

집중 기간 2026-08-16~26에 `vehicle_length_m=0.56`은 바뀌지 않았다. 회전 직사각형
construction과 physical-track projection도 이 기간에 의미 변경이 확인되지 않았다. 반면
`vehicle_half_width_m`은 `0.1435→0.15→0.1435→0.15`, wall margin은
`0.04→0.10→0.04`, obstacle safety margin은 `0.0147893→0→0.05→0.08`로 바뀌었다.

`eba25b148a78cefc33d45c3e7e6963de0ae7ea70`의 `vehicle_length 0.58 vs 0.56`
수정은 production planner 값 변경이 아니라 `stuck_case_harness`가 운영 YAML과 달랐던 것을
`0.56`으로 맞춘 진단 하니스 수정이다. 이를 과거 production vehicle length가 `0.58`이었다는
증거로 쓰면 안 된다.

### 1.2 사용자 기억과의 관계

Git이 직접 증명하는 물리 치수 변경은 full width `0.287→0.300 m`로, 더 커지는 방향이다.
그러므로 “비정상적으로 큰 physical footprint를 작게 고쳤다”는 사건은 확인되지 않았다.

`[INFERENCE]` 사용자가 기억하는 “큰 footprint 수정”은 physical rectangle보다 유효 벽 배제
폭을 말했을 가능성이 있다. 정렬된 자세에서 단순 폭으로 보면 wall 포함 폭이 한때
`0.300 + 2*0.10 = 0.500 m`였고, 8/23 이후 `0.300 + 2*0.04 = 0.380 m`가 됐다.
Git은 이 margin 축소와 이유는 증명하지만, 사용자 기억이 바로 이 사건을 가리킨다는 사실은
증명하지 않는다.

### 1.3 현재 frozen footprint의 정확한 semantics

`research_instrumentation_v1`의 runtime-effective 값은 다음과 같다.

- `vehicle_length_m = 0.56`
- `vehicle_half_width_m = 0.15` (full width `0.30`)
- `wall_safety_margin_m = 0.04`
- `safety_margin_m = 0.08` (obstacle side only)
- `obstacle_reserve_mode = none`, `localization_reserve_m = 0`, tracking fallback `0`

즉 frozen tag는 old center-only validator나 half width `0.1435`/wall `0.10` 조합이 아니라,
8/12의 rotated physical footprint와 8/23 이후 half width `0.15`/wall `0.04`, 8/26 이후
obstacle safety `0.08` 조합을 사용한다.

waypoint pose를 `(p, psi)`라 하면 네 corner는

```text
q_ab = p + R(psi) [a*0.28, b*0.15]^T,  a,b in {-1,+1}
```

이다. 각 corner를 같은 track branch의 인접 reference segment에 투영하고 그 위치의
interpolated `d_left=L`, `d_right=R`, corner lateral coordinate `d_q`를 구한다. Hard gate는

```text
clearance(q) = min(L - 0.04 - d_q, d_q + R - 0.04)
footprint_clearance = min_q clearance(q)
reject footprint_track_bound iff footprint_clearance < -epsilon
```

이다. width가 `0.05 m` 이하인 잘못된 bound만 fallback half width `1.50 m`로 대체한다.
`safety_margin_m=0.08`과 obstacle tracking reserve는 이 wall gate에 들어가지 않는다.
반대로 obstacle collision은 현재 obstacle face에서 `0.15+0.08=0.23 m`를 inset한다.

## 2. Recording-version heterogeneity

### 2.1 공통으로 확정되는 것

네 bag 모두 다음 type/hash가 동일하며 현재 tag build의 custom message hash와 일치한다.

| Topic | Type | RIHS01 hash | 확인되는 schema |
|---|---|---|---|
| `/confirmed_static_obs` | `f110_msgs/msg/ObstacleArray` | `3cbd63999030f9341a58672c586f326970619261c0ff331d475d7873c4363d96` | Cartesian AABB/radius와 Frenet `s_start/end`, `d_right/left`, center, velocity/variance/covariance, static/visible/interfering fields |
| `/global_waypoints` | `f110_msgs/msg/WpntArray` | `745ac277daef96dee4d9beff2534e7d10b29d936ab55d8895d4cea06cdfa3ad4` | `id,s,d,x,y,d_right,d_left,psi,kappa,vx,ax` |
| `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | `3cc97dc7fb7502f8714462c526d369e35b603cfc34d946e3f2eda2766dfec6e0` | standard Odometry |
| `/pf/pose/odom` | `nav_msgs/msg/Odometry` | same as above | standard Odometry |
| `/local_planning/p3_shadow` | `std_msgs/msg/String` | `df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18` | JSON payload는 네 bag 모두 `schema=local_planning_p3_cycle/1`, `mode=TEST_ACTIVE` |

동일 wire schema는 current replay compatibility를 증명하지만 exact source commit을 증명하지
않는다. P3 JSON에도 Git SHA/source version field는 없었다.

### 2.2 Bag별 직접 증거

| Bag | Direct historical evidence | Global waypoint provenance | Version status |
|---|---|---|---|
| `2026-08-24 19:00:30 KST` | Planner startup at 19:04:55/19:05:21: half width `0.150`, safety `0.000`, reserve off, wall `0.040`; restart at 19:06:00/19:08:00: same except safety `0.050`. Planner parameter events `0`. Length와 Git SHA는 기록되지 않음. | 185 points; full waypoint payload digest `ded35f...`; `/map_infos`: absolute `kinematic_localization/maps/map.yaml`, mincurv, estimated lap `9.355 s`. | `SOURCE_VERSION_PARTIALLY_INFERRED` |
| `2026-08-24 19:24:17 KST` | Four recorded startups (19:25:24~19:33:42): half width `0.150`, safety `0.080`, reserve off, wall `0.040`. 18 parameter events are drive-mode manager only. Length와 Git SHA는 기록되지 않음. | Same `ded35f...` payload and `9.355 s` provenance as first bag. | `SOURCE_VERSION_PARTIALLY_INFERRED` |
| `2026-08-25 09:23:32 KST` | P3 schema/mode와 custom message hashes는 확인. Planner startup log와 parameter event는 없음. 따라서 footprint parameters와 Git SHA는 미확정. | 185 points; full payload digest `b9a0cd...`; field별 digest상 앞 두 bag과 `vx_mps/ax_mps2`만 다르고 geometry/bounds는 동일. `/map_infos`: relative `monte_carlo_localization/maps/map.yaml`, estimated lap `9.939 s`. | `SOURCE_VERSION_PARTIALLY_INFERRED` |
| `2026-08-25 09:59:22 KST` | Recording 말미 10:17:38 startup만 half width `0.150`, safety `0.080`, reserve off, wall `0.040` 확인. 그 전 대부분의 config는 startup evidence 없음. Historical rosout 자체에 `safe-stop path rejected: footprint_track_bound`가 있어 rotated footprint hard gate 계열이 실행됐음은 직접 확인. 유일한 parameter event는 state machine `allow_avoid_transition=false`. Git SHA는 없음. | Same `b9a0cd...` payload and `9.939 s` provenance as third bag. | `SOURCE_VERSION_PARTIALLY_INFERRED` |

첫 bag은 한 recording 안에서도 safety margin이 바뀌었다. 따라서 “한 bag = 한 historical
configuration”도 성립하지 않는다. 둘째와 넷째 bag에 frozen tag와 같은 세 개의 출력 parameter가
보이더라도 startup log가 출력하지 않는 vehicle length, exact source, validator detail까지
동일하다고 확대 해석하지 않는다.

## 3. Interpretation contract

### A. Original historical behavior로 말할 수 있는 것

기록된 `/avoid_waypoints`, historical `/local_planning/p3_shadow`, rosout, state, drive command가
그 timestamp에 실제로 발행됐다는 사실은 말할 수 있다. Startup log가 존재하는 epoch에는 그
로그가 직접 밝힌 parameter도 함께 말할 수 있다.

말할 수 없는 것은 exact historical commit, startup evidence가 없는 epoch의 full config,
또는 historical output failure가 특정 current-source line 때문에 발생했다는 인과다. 녹화 날짜와
Git commit 날짜의 선후관계만으로 차량에 배포된 commit을 선택하지 않는다.

### B. Frozen-baseline replay behavior로 말할 수 있는 것

Replay는 detached clean source `6ba46dbe...`, isolated binary, tag의 production YAML을 사용했다.
Instrumentation overlay는 연구 출력 설정만 추가하고 footprint/margin을 override하지 않았다.
각 run의 `metadata.json` effective snapshot은 `0.56`, `0.15`, `0.08`, `0.04`, reserve off를
직접 기록한다. 네 run startup log도 half width `0.150`, obstacle safety `0.080`, wall `0.040`을
동일하게 확인한다.

재검증한 replay executable SHA-256은
`b74f00dab72a7f1d536da0fff95a19b57fd1920da4be144b3f052980406d689c`, production YAML
SHA-256은 `0a99dfce38a8552d40bb8903e49ad19bb06dcf7252490a66abc8fa187b4d432d`, effective
parameter snapshot ID는 `sha256:e9d8fb8aa0ec8c939f684c77926d79dfd7052c909e7a6415aa31a19bf4b0c4b6`다.

따라서 현재 `planner_baseline_summary.csv`, `validator_failure_distribution.csv`, stage/runtime/join
artifact는 B의 통계다. A와 B가 우연히 같은 parameter 일부를 공유하는 bag/epoch가 있어도 두
실험 정의는 합쳐지지 않는다.

## 4. `footprint_track_bound` 재해석

Frozen replay의 dominant first failure `footprint_track_bound=107,946`은
`research_instrumentation_v1`의 `0.56 x 0.30 m` rotated rectangle과 physical track bound,
wall inset `0.04 m` 조합에서 발생했다. Replay에서 obstacle safety margin `0.08 m`은 이 failure
predicate에 들어가지 않는다.

이 수치를 historical planner의 옛 half width `0.1435`, 한때의 wall `0.10`, 또는 알려지지 않은
배포 config 탓으로 돌리면 안 된다. Historical original에서 같은 failure string이 관측된 경우도
그 timestamp의 recorded historical outcome일 뿐이며, 현재 107,946건과 같은 모집단/설정이 아니다.

## 5. Corpus claim suitability

네 bag 모두 frozen-baseline replay에는 적합하고 실제 full replay가 완료됐다. Original historical
behavior 주장은 `RECORDED_OUTPUT_ONLY` 조건에서만 적합하다. Exact algorithm/config equivalence,
historical counterfactual, 또는 commit-level 재현 주장에는 네 bag 모두 부적합하다. 상세 per-bag
field는 `corpus_provenance.csv`에 보존했다.

이번 audit에서는 planner/P3 mapping을 바꾸지 않았고 P3 oracle도 실행하지 않았다.
