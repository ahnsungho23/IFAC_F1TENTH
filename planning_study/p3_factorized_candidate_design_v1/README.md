# P3 factorized candidate-generation design study v1

## 결론

DEVELOPMENT 40개 중 기존 P3 family 안에 strict hard-valid 해가 있던 23개를 대상으로, 현재 template에 묶여 있던 lateral factor와 transition factor를 오프라인에서 재조합했다. 기전 분리용 DEVELOPMENT counterfactual 기준으로 `d_mid`만 분리하면 9/23, 현재 event에 이미 있던 transition pair만 교차결합하면 5/23, 둘을 합치면 13/23을 회복했다. 두 변화의 합집합은 기존 진단의 13개 `TEMPLATE_OR_TRANSITION_SELECTION` 사례와 정확히 일치한다.

그러나 이것은 배포 가능한 정책 성능이 아니다. `A_UNCOUPLE_D_MID_DEV_COUNTERFACTUAL`의 대체 `d_mid`와 `FULL_LxT_DEV_UPPER_BOUND`의 lateral factor는 이미 열린 DEVELOPMENT oracle 결과에서 가져온 hindsight이다. production factor만 이용한 `A_EXISTING_D_MID_POOL`은 1/23만 회복했다. 따라서 현재 가장 타당한 연구 문제는 **pre-planning geometry로 작은 factor subset을 선택하는 bounded set-valued policy**이며, 이 문서에서는 정책·threshold를 만들지 않았다.

## 범위와 증거 계약

- repository HEAD: `80ae205fd470f537bd1e5449fb906c5548f6653a`
- 사용한 corpus: `DVE001..DVE040` DEVELOPMENT 40개만 사용
- oracle-valid: strict hard-valid P3가 존재하는 23개
- clearance gate: `STRICT_ONLY`
- `VALIDATION_UNSEEN` 및 `FINAL_HOLDOUT_UNSEEN`: 읽거나 계산하지 않음
- split manifest: 변경하지 않음
- production planner, P3 family, validator, ranker, lifecycle, vehicle parameter, collision model, speed shaping, YAML: 변경하지 않음
- 실행 harness SHA-256: `8b23f2b6df54c2e93794ee087f3ebd7a14f9921544af4f97eef755535e045e7e`
- harness source SHA-256: `c3aa74b1aa76ca87899fbe3805fe8fa972b7d6168ff1d9fff360914d48d38594`
- 모든 40개 event에서 direct reconstruction한 baseline path-digest multiset이 기록된 production candidate와 exact parity를 만족해야만 ablation을 계속하도록 했다.

입력 파일과 SHA, variant 정의는 [execution_manifest.json](execution_manifest.json)에 고정했다. 분석은 [analyze_factorized_design.py](analyze_factorized_design.py)가 재현하며, 결과 숫자의 요약은 [analysis_summary.json](analysis_summary.json)에 있다. harness wall time은 subprocess 시작·출력·파일 I/O와 잔여 잡음을 포함하므로 production callback latency가 아니다.

## 1. 현재 coupling 구조

### Factor 표현

한 direct P3 요청을 다음처럼 기록했다.

\[
L_i=(\mathrm{side},d_{target},d_{mid},\mathrm{probe/root\ lineage}),\qquad
T_j=(\alpha_{entry},\alpha_{exit},\mathrm{template\ semantics})
\]

\[
q_{ij}=(L_i,T_j)=(\mathrm{side},d_{target},d_{mid},\alpha_{entry},\alpha_{exit}).
\]

단, 이는 구현을 분석하기 위한 factor 표기이지 완전한 수학적 독립성을 뜻하지 않는다. 실제 5개 station은

\[
(z_0,z_1,z_2,z_3,z_4)
=f(ego\ d,d_{target},\alpha_{entry},\alpha_{exit},cluster,side)
\]

이고 offset은 `(ego_d, d_target, d_mid, d_target, 0)`이다. 즉 transition이 station을 바꾸며, analytic `d_mid` root도 해당 station과 probe equation에 의존한다. source의 `stationsFor()`가 이 결합을 명시한다([p3_shadow.cpp](../../src/local_planning/src/p3_shadow.cpp), lines 805–863).

따라서 B의 cross-pairing에서 옮겨진 수치 `d_mid`는 새 transition에 대한 analytic root라는 보장은 없다. 다만 동일한 다섯 자유도와 동일한 P3 spline reconstruction/validator를 쓰므로 **기존 P3 family의 direct candidate**인 것은 유지된다.

`entry_scale/exit_scale`만으로 실제 meter 길이가 완전히 정해지는 것도 아니다. side가 outside line인지에 따른 multiplier, cluster 위치, track horizon, ego speed 기반 tail이 exit option과 realized station을 바꾼다. 그래서 CSV에는 scale과 함께 `entry_length_m/exit_length_m`를 보존했다([p3_shadow.cpp](../../src/local_planning/src/p3_shadow.cpp), lines 844–863, 1578–1606).

### Production 흐름

```text
ego + obstacle/track corridor
        |
        +-- side/component/target
        |
        +-- template이 entry/exit를 선택 --------------------+
        |                                                    |
        +-- 그 entry/exit로 z0..z4 구성                       |
        |                                                    |
        +-- template별 probe/zero-interface -> d_mid/root ----+
                                                             |
              exact tuple (side,target,middle,entry,exit)
                              |
                 positive-segment guard -> P3 -> validator
                              |
                    stage order/cap/first success
```

현재는 독립적인 `L set`과 `T set`을 만든 뒤 곱하지 않는다. template이 full tuple을 한 번에 제안한다. M1은 exact five-tuple만 deduplicate하고, 서로 다른 template에서 관측된 lateral/transition factor를 교차결합하지 않는다([p3_shadow.cpp](../../src/local_planning/src/p3_shadow.cpp), lines 2113–2155).

| stage | lateral-shape factor | transition factor | 현재 coupling / 불가능 조합 |
|---|---|---|---|
| M0-V1 | 한 side target, curvature-continuity `s_probe,d_probe`, 해당 station에서 푼 active-outer `d_mid` root | entry ladder와 3개 exit 비율 | 각 `(entry,exit)`마다 station과 root를 다시 계산한다. root와 다른 transition의 임의 교차결합은 없음. side당 frozen cap 16 |
| M0-V2 | zero-interface는 `d_mid=d_target`; analytic option은 component/probe root | boundary/quarter/center/analytic template별 hard-coded entry/exit | template이 지정하지 않은 `(d_target,d_mid,entry,exit)` 조합은 생성 불가. M0-V1 실패 뒤, 자체 12 및 callback 잔여 예산 안에서만 실행 |
| M1 zero | boundary-inset `d_target`, `d_mid=d_target` | `SHORT=(entry_min,exit_min)`, `SPAN=(entry_quarter,exit_span)` | 같은 target에 다른 `d_mid`를 붙일 수 없음. 다른 M0/M1 transition을 붙일 수 없음 |
| M1 analytic | branch-near/far target과 bottleneck-center probe에서 푼 active/all-inactive root | `NEAR_LONG=(entry_max,exit_max)`, `FAR_SPAN=(entry_quarter,exit_span)`, 조건부 `NEAR_BISECTED` | root는 해당 template station에 종속. 이미 event에 존재하는 다른 transition과 교차결합하지 않음 |

근거 source:

- candidate cap은 M0-V1 16, M0 extension 12, callback total 24이다([p3_shadow.cpp](../../src/local_planning/src/p3_shadow.cpp), lines 603–608).
- M0-V1은 target/probe를 정한 뒤 entry/exit별 station에서 root를 풀고 바로 검증한다([p3_shadow.cpp](../../src/local_planning/src/p3_shadow.cpp), lines 2357–2447).
- M0-V2 template의 zero-interface와 analytic option은 lines 1935–2047에 고정되어 있다.
- M1 context의 `entry_min/max/quarter`, `exit_min/max/span`, boundary inset은 lines 2050–2110에 만들어진다.
- M1 zero의 `d_mid=d_target`과 고정 transition은 lines 2253–2283에 나타난다.
- M1 analytic root는 선택된 transition으로 station을 만든 다음 probe equation을 푼다(lines 2192–2251).
- M1은 M0가 hard-valid를 못 찾았을 때만 남은 total-24 budget으로 호출된다(lines 407–488).

[current_factor_coupling.csv](current_factor_coupling.csv)는 40개 DEVELOPMENT failure에서 실제 관측한 production full tuple 249개를 한 행씩 분해한다. 관측 stage는 M0-V1 81, M0-V2 12, M1 156개였다. 이 파일의 `currently_impossible_or_skipped`는 각 template 때문에 만들 수 없는 결합을 명시한다.

## 2. Offline factorized variants

| variant | factor source와 의미 | 온라인 해석 가능 여부 |
|---|---|---|
| `BASELINE_PRODUCTION` | 기록된 production full tuple exact reconstruction | 가능, parity 기준선 |
| `A_EXISTING_D_MID_POOL` | M1 zero target/side에 같은 event production의 same-side `d_mid`를 붙이고 원 transition 유지 | 현재 factor만 쓰는 진단. 선택 규칙은 아직 없음 |
| `A_UNCOUPLE_D_MID_DEV_COUNTERFACTUAL` | 각 M1 zero tuple에 같은 side/target/transition에서 가장 가까운 이미 알려진 strict oracle-valid `d_mid` 하나 추가 | 불가능. A 기전의 DEVELOPMENT oracle upper bound |
| `B_CROSS_PAIR_TRANSITIONS` | production lateral tuple × 같은 event의 production transition pair | factor는 현재 관측 가능. 전수 곱은 온라인 예산에 부적합 |
| `AB_EXISTING_FACTOR_BASIS` | A-existing lateral pool × production transition pool | 현재 factor 기반 전수 upper diagnostic |
| `AB_DEV_COUNTERFACTUAL` | A-counterfactual lateral pool × production transition pool | A+B 기전 분리용 DEVELOPMENT upper bound |
| `FULL_LxT_DEV_UPPER_BOUND` | production 및 모든 strict oracle-valid lateral tuple × production transition pool | 순수 oracle upper bound. 배포 정책이 아님 |

모든 조합은 같은 direct P3 reconstruction과 같은 validator로 평가했다. exact binary `double`을 유지했다. 일부 analytic root는 `1e-12`보다 작은 차이에도 다른 sampled-path digest를 만들었기 때문에, factor key를 반올림했을 때 깨진 parity를 허용하지 않았다.

## 3. Ablation 결과

| variant | 총 후보 = validator | guard reject | hard-valid 후보 | hard-valid/validator | 회복/23 | 중복 path 수(비율) |
|---|---:|---:|---:|---:|---:|---:|
| production | 249 | 0 | 0 | 0.00% | 0 | 32 (12.85%) |
| A, current factor pool | 417 | 0 | 2 | 0.48% | 1 | 39 (9.35%) |
| A, DEVELOPMENT counterfactual | 265 | 0 | 16 | 6.04% | 9 | 35 (13.21%) |
| B, cross transitions | 2,181 | 0 | 63 | 2.89% | 5 | 51 (2.34%) |
| A+B, current factor basis | 4,077 | 0 | 69 | 1.69% | 7 | 61 (1.50%) |
| A+B, DEVELOPMENT counterfactual | 2,226 | 0 | 81 | 3.64% | 13 | 54 (2.43%) |
| full L×T DEVELOPMENT upper bound | 21,978 | 0 | 9,051 | 41.18% | 23 | 1,318 (6.00%) |

Episode 회복:

- A counterfactual 9개: `DVE001, DVE006, DVE015, DVE017, DVE019, DVE022, DVE023, DVE029, DVE038`
- B 5개: `DVE012, DVE018, DVE024, DVE029, DVE031`
- A와 B의 교집합: `DVE029`
- A+B counterfactual 13개: 위 합집합 전체
- current-factor A는 `DVE019` 한 개만 회복
- full upper bound는 oracle-valid 23개 전부 회복

기존 template taxonomy에서 primary `SELECTION_FAILURE_WITH_EXISTING_OPTION`은 4개였지만 B가 5개를 회복한 것은 모순이 아니다. `DVE029`는 primary class가 A의 missing option이면서 B 경로로도 회복되는 중첩 사례다. [recovery_by_variant.csv](recovery_by_variant.csv)는 40개 모든 event의 후보·validator·hard-valid 수와 boolean 회복을 함께 제공한다.

## 4. Candidate explosion과 runtime

| variant | 후보/evaluation p50 | p95 | p99 | max | harness 증가시간 p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| production | 3 | 30.0 | 30.0 | 30 | 0.00127 s | 0.00606 s | 0.00962 s | 0.01018 s |
| A current | 4.5 | 64.3 | 72.44 | 74 | 0.00201 s | 0.00993 s | 0.01235 s | 0.01310 s |
| A counterfactual | 4 | 30.0 | 31.22 | 32 | 0.00263 s | 0.01101 s | 0.01429 s | 0.01536 s |
| B | 4 | 526.35 | 644.38 | 667 | 0.00294 s | 0.07504 s | 0.12197 s | 0.14585 s |
| A+B current | 5 | 1,023.70 | 1,259.76 | 1,305 | 0.00224 s | 0.14366 s | 0.23745 s | 0.28810 s |
| A+B counterfactual | 4 | 527.80 | 655.69 | 667 | 0.00304 s | 0.07862 s | 0.12562 s | 0.15516 s |
| full L×T | 33 | 1,898.05 | 5,375.34 | 7,440 | 0.01276 s | 0.31330 s | 0.83304 s | 1.03177 s |

전수 factorization은 production total cap 24와 비교할 수 없을 정도로 크다. 특히 B만으로도 p95 526.35, max 667 validator 호출이 필요했다. duplicate **비율**이 B에서 낮아진 것은 공간이 효율적이라는 뜻이 아니다. 절대 중복 path는 production 32개에서 B 51개, full 1,318개로 증가했다. 자세한 per-event 값은 [factorization_ablation.csv](factorization_ablation.csv), path digest 중복은 [duplicate_analysis.csv](duplicate_analysis.csv), 모든 요청/결과는 [factorized_candidate_space.csv](factorized_candidate_space.csv)에 있다.

Runtime은 같은 머신에서 empty harness wall time을 뺀 진단치다. subprocess와 CSV 출력 비용 때문에 알고리즘 latency의 절대값으로 사용할 수 없지만, 후보 폭증과 계산량 증가 방향은 validator-call 수와 일치한다.

## 5. 작은 bounded basis

### Oracle-upper-bound transition coverage

correct lateral factor가 마법처럼 주어진다는 조건에서, strict hard-valid full-factorial row의 exact transition pair를 maximum set coverage로 골랐다.

| transition-pair budget | 23개 중 조건부 coverage | 해석 |
|---:|---:|---|
| 1 | 16/23 (69.6%) | DEVELOPMENT oracle upper bound |
| 2 | 21/23 (91.3%) | DEVELOPMENT oracle upper bound |
| 3 | 22/23 (95.7%) | DEVELOPMENT oracle upper bound |
| 4 | 23/23 (100%) | DEVELOPMENT oracle upper bound |

2-pair 결과는 end-to-end 온라인 회복률이 아니다. correct lateral factor를 이미 안다는 조건이 숨어 있다. 선택된 exact pair와 covered/uncovered event는 [candidate_budget_analysis.csv](candidate_budget_analysis.csv)에 있다.

### Bounded lateral option의 한계

A-counterfactual로 회복 가능한 9개에서 필요한 `d_mid - d_target` exact 값을 hindsight set-cover로 고르면, 1/2/3/4개 delta가 각각 2/3/4/5개만 덮었다. 반면 production에 실제 있던 same-side `d_mid` pool은 A 단독으로 1/23만 회복했다. 즉 몇 개의 전역 상수 delta를 추가하는 것만으로는 9개 lateral coupling failure 대부분을 보존하지 못한다.

결론은 다음과 같다.

- transition factor는 2–4개의 작은 DEVELOPMENT upper-bound basis가 존재한다.
- lateral factor는 현재 관측 factor나 작은 global exact-delta basis만으로 충분하다는 증거가 없다.
- 따라서 작은 bounded candidate set의 핵심 미해결 항목은 **geometry에서 event-specific lateral option을 생성/선택하는 것**이다.
- validation 전에 후보 budget과 selection rule을 DEVELOPMENT에서 고정해야 한다. 이 연구에서는 고정하지 않았다.

## 6. Geometry conditioning 가능성

[geometry_conditioning_features.csv](geometry_conditioning_features.csv)는 oracle-valid 23개에 대해 planning 전에 관측 가능한 ego/obstacle/corridor/reference geometry만 기록한다. outcome은 분석용 recovery route로 붙였고 ML이나 threshold fitting은 하지 않았다.

가장 뚜렷한 제한적 패턴은 B-only 4개였다.

- 4/4가 reference-curvature `SIGN_CHANGE`
- ego-to-obstacle start 중앙값 0.155 m, 범위 0.112–0.157 m
- obstacle longitudinal span 중앙값 0.021 m, 범위 0.018–0.023 m
- track-width range 중앙값 1.338 m, 범위 0.650–1.438 m

A-only 8개는 obstacle span 중앙값 0.206 m이었지만 curvature pattern은 `NEAR_ZERO` 2, `POSITIVE` 3, `NEGATIVE` 1, `SIGN_CHANGE` 2로 이질적이었다. full-only 10개도 거리·곡률·corridor 폭이 넓게 분산됐다. 이는 가까운 짧은 obstacle와 curvature sign change가 transition cross-pair 필요성을 예고할 가능성을 보여주지만 표본이 4개이고 event domain 상관도 있으므로 threshold 근거가 아니다.

물리적으로 해석 가능한 가설은 다음 단계의 후보 descriptor로 유지할 수 있다.

- obstacle까지 남은 거리와 obstacle 길이: entry/exit transition에 쓸 longitudinal 공간
- 다음 obstacle까지 merge 거리: long exit 허용 여부
- chosen-side free width와 bottleneck width: lateral target/middle의 허용 여유
- entry/obstacle/exit reference curvature 및 sign change: Frenet offset을 Cartesian curvature로 변환할 때의 부담
- ego `d`, `d_target`, speed: 필요한 lateral 변화량과 동역학 부담

하지만 현재 DEVELOPMENT 결과만으로 어느 feature가 독립적으로 원인인지, 어떤 threshold가 재현되는지는 확인되지 않았다. `[INFERENCE]` geometry-conditioned selection은 가능성이 있으나 아직 frozen online policy는 없다.

## 7. Research framing

H1–H4의 지지·반증·예상 budget·미해결점을 [research_framing_comparison.md](research_framing_comparison.md)에 비교했다.

Primary recommendation은 **H4: geometry-conditioned bounded factorized P3 candidate generation**이다. A+B가 13개 인공 coupling failure를 직접 회복하지만 전수 곱은 p95 수백 개이므로, factorization 자체보다 관측 geometry에서 작은 set을 고르는 문제가 실제 연구 기여와 실행 가능성을 동시에 결정한다. H3는 mechanism을 가장 직접적으로 검증하는 fallback framing이다.

이 추천은 repository 내부 DEVELOPMENT 증거에 대한 방법론적 추천이다. 외부 선행연구 조사를 수행하지 않았으므로 학술적 novelty를 확정하지 않는다.

## README 필수 질문에 대한 답

1. **현재 artificial coupling 제거가 의미 있는 비율을 회복하는가?**  그렇다. oracle-informed A+B 기전 분리는 13/23(56.5%)을 회복하며, 기존 13개 template/transition 사례와 정확히 일치한다. 다만 production factor만의 A+B 전수 조합은 7/23이고, 온라인 정책 성능으로 볼 수 없다.

2. **A/B/A+B 각각 얼마인가?**  A counterfactual 9/23, B 5/23, A+B counterfactual 13/23이다. A와 B는 `DVE029` 한 개가 겹친다. 현재 factor만 쓴 A는 1/23이다.

3. **naive explosion은 얼마나 큰가?**  B는 p95 526.35/max 667, A+B counterfactual은 p95 527.8/max 667, full L×T는 p95 1,898.05/max 7,440 후보와 같은 수의 validator 호출을 만들었다. production은 p95/max 30이었다.

4. **작은 bounded set으로 대부분의 oracle coverage를 유지할 수 있는가?**  transition만 보면 correct lateral factor 조건에서 2개 pair가 21/23을 덮는다. 그러나 이는 hindsight upper bound이며, 작은 global lateral delta 4개는 A 대상 9개 중 5개만 덮는다. end-to-end bounded online coverage는 아직 입증되지 않았다.

5. **어느 부분이 geometry-conditioned selection을 요구하는가?**  주로 event-specific lateral option의 생성/선택, 그리고 수백 개로 폭증하는 transition cross-product에서 소수 pair를 고르는 부분이다. 단순히 global pair 몇 개를 더하는 것만으로는 correct lateral factor를 알 수 없다.

6. **가장 강한 연구 방향이 geometry-conditioned bounded factorized P3 generation인가?**  현재 DEVELOPMENT 내부 증거로는 그렇다. 단, 이는 primary hypothesis이지 검증 완료된 알고리즘이나 novelty claim이 아니다.

7. **다음에 풀 exact algorithmic problem은 무엇인가?**  pre-planning feature vector `x`에서 baseline을 항상 포함하면서 lateral subset `L_K(x)`와 transition subset `T_J(x)`를 골라

   \[
   Q(x)=Q_{production}(x)\cup\{(L_i,T_j):L_i\in L_K(x),T_j\in T_J(x)\},
   \quad |Q(x)|\le B
   \]

   를 만들고, DEVELOPMENT hard-valid episode coverage를 최대화하되 candidate/validator p99 budget `B`를 지키는 **bounded set-valued selection problem**이다. exact-tuple 및 path-digest dedup, 기존 construction guard/validator, baseline fallback을 유지해야 한다. 특히 `L_K(x)`는 oracle `d_mid`를 조회하지 않고 기존 P3 자유도와 pre-planning geometry만으로 만들어야 한다. `K,J,B`와 selection rule은 다음 DEVELOPMENT 단계에서 고정한 뒤 unseen validation에 처음 적용해야 한다.

## 산출물 안내

- [current_factor_coupling.csv](current_factor_coupling.csv): production 후보의 L/T 분해 및 현 coupling
- [factorized_candidate_space.csv](factorized_candidate_space.csv): 모든 variant의 direct request, lineage, validator 결과
- [factorization_ablation.csv](factorization_ablation.csv): event × variant 집계
- [recovery_by_variant.csv](recovery_by_variant.csv): 40개 event별 회복 비교
- [candidate_budget_analysis.csv](candidate_budget_analysis.csv): p50/p95/p99/max, set-cover upper bounds
- [duplicate_analysis.csv](duplicate_analysis.csv): digest 기준 중복
- [geometry_conditioning_features.csv](geometry_conditioning_features.csv): oracle-valid 23개의 observable geometry
- [research_framing_comparison.md](research_framing_comparison.md): H1–H4 비교와 추천
- [plots/](plots/): recovery, explosion, duplication, geometry, bounded coverage 그림
