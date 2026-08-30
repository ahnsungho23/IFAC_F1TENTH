# VALIDATION v1 integrity and failure diagnosis

## 결론

이 진단은 frozen validation 37개만 사용했고 `FINAL_HOLDOUT_UNSEEN`은 열지 않았다. H4-A,
production planner, exact validator, ranker 및 parameter는 수정하지 않았다. 검증에는 기존 frozen
exact-validator harness(`SHA-256 8b23f2b6df54c2e93794ee087f3ebd7a14f9921544af4f97eef755535e045e7e`)를
그대로 사용했다.

핵심 결론은 다음과 같다.

1. VUE036은 `ORACLE_REFINEMENT_MISS`다. 유효점은 domain과 factor space 안에 있었지만
   top-30 coarse seed 선택에서 주변 셀이 잘려 refinement가 그 구역을 방문하지 않았다.
2. 현재 oracle의 `NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN`은 P3 가능성의 신뢰할 수 있는 upper-bound
   certificate가 아니다. 기존 label은 변경하지 않았고 sampled-search 결과로만 유지한다.
3. H4-A exact hard-valid recovery는 14개지만, 보수적인 연구용 `USABLE_VALID_P3` 기준을 통과하는
   것은 10개다.
4. 11개 central unrecovered는 candidate-budget truncation 2개, probe/root issue 5개,
   multi-parameter interaction 4개다. 9개에는 oracle best lateral pair가 현 factor basis에 없다.
5. H4-A와 H3의 central recovery는 모두 13/24이지만 성공 집합은 같지 않다. H4-A only는 VUE027,
   H3 only는 VUE013이다.

## 범위와 protocol

- 입력: validation-v1의 37개 episode
- 분석 후 protocol role: `VALIDATION_SEEN_AFTER_V1`
- 원 frozen split: 수정하지 않음
- holdout 접근: 없음
- 기존 oracle/result artifact: 수정하거나 소급 relabel하지 않음
- 새 overlay: [validation_seen_after_v1_manifest.csv](validation_seen_after_v1_manifest.csv)

`run_diagnosis.py`는 validation 전용 manifest와 이미 공개된 DEVELOPMENT artifact만 읽는다. 37개
validation input에서 frozen factor pool 전체를 exact validator로 평가한 것은 selector redesign이
아니라 K=24 바깥 witness와 factor-basis coverage를 구분하기 위한 offline diagnosis다.

## 1. VUE036 oracle miss

### exact valid witness

H3, H4-A, H4-B가 모두 같은 path digest `95a7394f79189d58`을 복원했다.

| 항목 | 값 |
|---|---:|
| side | RIGHT |
| `d_target` | -0.6725895769471937 m |
| `d_mid` | -0.7246579807893281 m |
| entry factor | 0.5145810930150512 |
| exit factor | 0.4971684162574945 |
| target source | `COMPONENT_QUARTER_C0` |
| mid source | `TARGET_CENTER_HALF` |
| H3 / H4-A / H4-B rank | 23 / 13 / 5 |
| constructed points | 29 |
| exact hard validator | PASS |

RIGHT component는 `[-0.9850000000,-0.5684527693]`다. near에서 far 방향 1/4 지점이
`d_target=-0.6725895769`이고 bottleneck center `-0.7767263846`와 target의 중간값이
`d_mid=-0.7246579808`이다. Entry/exit pair는 같은 event의 production
`M1/ZERO_BOUNDARY_SHORT`에 이미 존재하던 pair다.

이 witness는 analytic root나 production template을 재생한 것이 아니다. Frozen factorized method가
`(side,d_target,d_mid,entry,exit)`를 direct P3로 복원한 것이다. 따라서 selector witness에는
`s_probe`, `d_probe`, root index가 없다. 비교를 위해 production의 세 경로는 다음과 같다.

| production template | `d_target` | `d_mid` | probe/root | entry / exit | 결과 |
|---|---:|---:|---|---|---|
| `ZERO_BOUNDARY_SHORT` | -0.5749613 | -0.5749613 | ZERO interface | 0.514581 / 0.497168 | curvature invalid |
| `ZERO_BOUNDARY_SPAN` | -0.5749613 | -0.5749613 | ZERO interface | 0.713669 / 0.497168 | curvature invalid |
| `FAR_SPAN` | -0.9850000 | -0.7767264 | `s_probe=1.135823`, corridor-center/all-inactive root | 0.713669 / 0.497168 | curvature invalid |

### miss mechanism

Frozen oracle의 RIGHT target domain과 `d_mid∈[-1.5,1.5]`, production factor pair에는 witness가
포함된다. 그러나 exact tuple은 0.05 m coarse grid/production anchors에 없었다.

- coarse+anchor requests: 6,558
- refinement requests: 1,823
- nearest same-factor coarse point: `(-0.65,-0.70)`, 2-D distance 0.033441 m
- witness 주변 coarse ranks: 63, 64, 75, 76(strict), 160, 161, 172, 173(relaxed)
- refinement seed: violation-score 상위 30개만
- top-30 boundary와 주변 셀의 score가 모두 `(2,21.638047...)`인 tie였으나 request-order상
  주변 셀은 rank 63 이후였다.
- 실제 refinement request 중 witness의 ±0.04 m window를 덮는 점: 0
- frozen 0.01 m resolution을 같은 주변에 적용한 별도 진단: 64점 중 8점 hard-valid

따라서 exact witness가 격자점이 아니었던 사실만으로 `ORACLE_RESOLUTION_GAP`이라 하지 않는다.
기존 0.01 m 격자도 해당 지역에서는 유효점을 찾았다. 직접 원인은 top-30 seed/window가 그 지역에
도달하지 못한 `ORACLE_REFINEMENT_MISS`다. Domain gap, factor-space mismatch, construction guard,
digest dedup, validator implementation mismatch 증거는 없다. 상세 수치는
[vue036_oracle_miss_diagnosis.csv](vue036_oracle_miss_diagnosis.csv)에 있다.

## 2. HARD_VALID와 USABLE_VALID

### 정의

`HARD_VALID_P3`는 기존 exact hard validator의 `hard_valid=1`과 빈 failure reason이다. 이 정의와
production validator는 변경하지 않는다.

본 연구 진단에서 제안하는 `USABLE_VALID_P3_DIAGNOSIS_V1`은 다음의 교집합이다.

```text
exact_hard_valid
AND footprint/track, lateral-slope, signed-curvature, curvature-rate hard margin >= 0
AND minimum_normalized_safety_slack >= 0
AND obstacle diagnostic margin >= 0
AND exit_reaches_next_obstacle == false
AND braking_deficit_m <= 1e-9
```

이는 기존에 기록된 물리량만 쓰는 보수적인 연구 평가 기준이다. Negative aggregate diagnostic을
곧바로 hard collision이라고 소급 판정하지 않으며, 오직 “실사용 가능 recovery” 집계에서 제외한다.
`velocity_loss`는 품질 비교용으로 보존하지만 frozen 물리 threshold가 없으므로 임의 cutoff를
추가하지 않았다.

### 결과

| 기준 | known exact-feasible 25개 | predeclared oracle-feasible 24개 |
|---|---:|---:|
| A. H4-A hard-valid recovery | 14/25 | 13/24 |
| B. H4-A usable-valid recovery | 10/25 | 10/24 |

Hard-valid지만 usable에서 제외된 것은 VUE009, VUE011, VUE014, VUE036이다.

- VUE009/011/014: negative obstacle/slack + next-obstacle exit conflict
- VUE036: negative obstacle/slack; exit conflict와 braking deficit은 없음
- positive braking deficit: 0/14
- velocity loss: 전 14개에 기록됐지만 threshold 미정
- complete factor pool 안에서 위 네 사례를 대체할 usable witness: 0

모든 recovery의 원 수치는 [hard_vs_usable_valid.csv](hard_vs_usable_valid.csv)에 있다.

## 3. 11개 oracle-feasible H4-A unrecovered

Complete existing factor pool을 H4-A ordering 그대로 exact-validate했다.

| primary mechanism | 수 | event |
|---|---:|---|
| candidate-budget truncation | 2 | VUE013(rank 38), VUE025(rank 221) |
| probe/root issue | 5 | VUE004, VUE008, VUE018, VUE026, VUE032 |
| multi-parameter interaction | 4 | VUE010, VUE012, VUE020, VUE035 |

VUE013/025는 pool 안에 hard-valid가 있지만 frozen geometry ordering이 K=24 밖으로 밀었다. 따라서
secondary mechanism은 `GEOMETRY_RULE_GENERALIZATION_FAILURE`다. 나머지 9개는 oracle best
`(d_target,d_mid)` pair가 factor pool에 없으며 `LATERAL_FACTOR_MISS`가 secondary evidence다.
이는 새 lateral factor나 rule을 제안하는 결과가 아니다.

DEVELOPMENT의 H4-A K24 unrecovered 7개는 probe 계열 2, transition/template 1, other 4였다.
Validation에서는 probe/root 5, multi-parameter 4, budget 2로 관측 메커니즘이 이동했다. 양 split의
원 taxonomy granularity가 다르므로 이 비교는 descriptive다. Event별 증거는
[validation_unrecovered_taxonomy.csv](validation_unrecovered_taxonomy.csv), split 비교는
[development_vs_validation_mechanisms.csv](development_vs_validation_mechanisms.csv)에 있다.

## 4. H4-A가 H3와 동률이 된 이유

DEVELOPMENT oracle-feasible 23개에서 H3 K24는 13개, H4-A K24는 16개를 회복했다. Validation
24개에서는 둘 다 13개다. 그러나 validation 성공 집합은 교집합 12개이고, H4-A only VUE027,
H3 only VUE013으로 한 건씩 교환됐다.

관측된 세 메커니즘이 함께 설명한다.

1. **Binary geometry feature의 분별력 감소.** Oracle-best side에서 entry distance 평균이
   0.530 m에서 0.246 m로 줄고(SMD -0.349), max reference curvature는 0.484에서
   0.546 rad/m로 증가했다(SMD +0.355). 결과적으로 H4-A `desired_entry=long` rule은
   DEVELOPMENT 19/23(82.6%)에서 validation 23/24(95.8%)로 거의 상수가 됐다.
2. **Fixed basis가 같은 주요 mode를 덮음.** 공통 recovery가 12개이고 각 method가 한 사례만
   독점하여 net result가 같아졌다.
3. **K=24 truncation.** 두 method 모두 매 feasible event에서 24개를 채웠다. 특히 VUE013의
   H4-A valid witness는 rank 38이라 H3만 회복했다. VUE025도 H4-A rank 221에 valid가 있었다.

`[INFERENCE]` 따라서 “geometry 자체가 쓸모없다”기보다, frozen binary thresholds가 shifted
validation geometry에서 순위 정보를 충분히 나누지 못하고 K=24와 결합한 것이 더 정확한
설명이다. 동시에 나머지 9개 miss는 full factor pool에도 유효 후보가 없으므로 K만 늘리는 것으로
전체 문제가 해결되지는 않는다. 분포 수치는
[geometry_distribution_comparison.csv](geometry_distribution_comparison.csv)에 있다.

## 5. Oracle completeness와 기존 결과

Validation의 기존 oracle-negative 13개에 complete existing factor pool을 추가로 적용했을 때
hard-valid witness는 VUE036 하나에서만 나왔다. 나머지 12개에는 그 제한된 pool 안에서 추가
witness가 없었다. 이것은 12개가 P3-infeasible이라는 증명이 아니다.

동일 coarse/top-30-refine 구조를 사용한 과거 `NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN`도 구조적으로
같은 false-negative 가능성의 영향을 받는다. 다만 직접 증명된 사례는 VUE036 하나뿐이다.
기존 결과는 **소급 relabel하지 않았으며**, event별 보존 상태는
[oracle_negative_case_audit.csv](oracle_negative_case_audit.csv)에 있다. 상세 계약은
[oracle_completeness_audit.md](oracle_completeness_audit.md)를 따른다.

## 요청 질문에 대한 답

1. **왜 oracle이 VUE036을 놓쳤는가?** Domain이나 resolution 자체가 아니라, 같은 violation
   score tie에서 주변 coarse 셀이 top-30 seed 밖으로 밀려 refinement window가 유효 영역에
   도달하지 못한 `ORACLE_REFINEMENT_MISS`다.
2. **현재 oracle이 upper bound로 충분히 신뢰 가능한가?** 아니다. VUE036이 false negative를
   직접 증명한다. 현재 값은 sampled-search lower bound on found feasibility이지 completeness
   upper bound가 아니다.
3. **H4-A recovery 중 usable은 몇 개인가?** 전체 exact hard-valid 14개 중 10개다. 기존
   oracle-feasible 24개 기준으로는 10/24다.
4. **왜 H4-A가 H3 대비 우위를 잃었는가?** Validation에서 geometry rule이 거의 항상 long-entry를
   선택해 분별력이 감소했고, fixed basis가 대부분 같은 mode를 덮었으며, H4-A의 VUE013 valid가
   K=24 밖으로 밀렸기 때문이다.
5. **무엇이 generalize되지 않았는가?** H4-A binary transition ordering과 bounded lateral factor
   coverage다. 2개는 ordering/K 결합, 9개는 factor-basis coverage 부족으로 남았다.
6. **다음 단계는 무엇인가?** 순서는 oracle repair와 success-metric repair가 먼저이고, 그 후
   method redesign 여부를 판단하는 조합이다. 불완전 oracle과 hard-only metric 상태에서 H4-A v2를
   설계하면 target label 자체가 흔들린다. 이 작업에서는 어떤 redesign도 구현하지 않았다.

## 산출물

- [vue036_oracle_miss_diagnosis.csv](vue036_oracle_miss_diagnosis.csv)
- [oracle_completeness_audit.md](oracle_completeness_audit.md)
- [hard_vs_usable_valid.csv](hard_vs_usable_valid.csv)
- [validation_unrecovered_taxonomy.csv](validation_unrecovered_taxonomy.csv)
- [development_vs_validation_mechanisms.csv](development_vs_validation_mechanisms.csv)
- [geometry_distribution_comparison.csv](geometry_distribution_comparison.csv)
- [validation_seen_after_v1_manifest.csv](validation_seen_after_v1_manifest.csv)
- [oracle_negative_case_audit.csv](oracle_negative_case_audit.csv)
- [run_diagnosis.py](run_diagnosis.py)
