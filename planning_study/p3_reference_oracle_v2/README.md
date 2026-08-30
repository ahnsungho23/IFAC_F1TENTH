# Reference Oracle v2 + usable-path evaluation contract

## 결론

Reference Oracle v2를 기존 P3 family와 exact validator만 사용해 구축하고, 이미 본 데이터 86개(PILOT 9 + DEVELOPMENT 40 + VALIDATION_SEEN_AFTER_V1 37)를 전수 재계산했다. 총 **42,378,897**개의 선언 요청을 빠짐없이 제출했고, **12,356,965**개 constructed path에 대해 같은 수의 exact-validator 실행을 수행했다. Coverage, exact evaluator lineage, production candidate digest/verdict parity는 모두 **86/86 PASS**다. 사전 verification은 VUE036을 포함해 **7/7 PASS**, frozen H3/H4-A/H4-B artifact candidate parity는 **231/231 PASS**다.

이 결과의 completeness는 [동결된 유한 search domain](oracle_v2_spec.md)에 대해서만 성립한다. 연속 P3 공간 전체에 대한 수학적 완전성이나 실시간 적용 가능성을 주장하지 않는다.

Production planner, P3 family, mapping, validator, ranking, parameter, K, H3, H4-A v1, H4-B는 변경하지 않았다.

## Oracle v2 범위와 계산량

| Seen scope | Events | Declared requests | Constructed / validator executions | Hard-feasible events | Usable-feasible events |
|---|---:|---:|---:|---:|---:|
| PILOT_SEEN_DEVELOPMENT_DATA | 9 | 4,080,202 | 2,390,688 | 5 | 5 |
| DEVELOPMENT | 40 | 18,513,014 | 3,680,722 | 23 | 17 |
| VALIDATION_SEEN_AFTER_V1 | 37 | 19,785,681 | 6,285,555 | 25 | 19 |
| **ALL SEEN** | **86** | **42,378,897** | **12,356,965** | **53** | **41** |

누적 C++ harness wall time은 약 2,086.2 s였다. Request 수와 valid row 수는 중복 path를 포함할 수 있으므로 per-event unique digest 수도 `computation_summary.csv`에 별도로 기록했다.

## VUE036 verification

Old oracle의 `NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN`은 false negative였다. 원인은 이미 진단된 `ORACLE_REFINEMENT_MISS`, 즉 coarse 결과 중 top-30만 refinement seed로 남긴 heuristic이 작은 feasible island를 제외한 것이다.

Oracle v2는 VUE036의 **153,524**개 선언 요청을 모두 평가해 75,264개 path를 construct/validate했고, **52 hard-valid / 52 usable-valid** 요청을 찾았다.

- v2 best: `d_target=-0.70`, `d_mid=-0.73`, entry `0.51458109301505117`, exit `0.49716841625749453`, digest `ba6514964316af56`
- 이미 알려진 frozen-selector witness: `d_target=-0.67258957694719368`, `d_mid=-0.72465798078932808`, 같은 entry/exit, digest `95a7394f79189d58`
- witness의 frozen 선택 순위: H3 23, H4-A 13, H4-B 5

두 경로 모두 hard-valid이며 exit conflict가 없고 braking deficit가 0이라 usable-valid다. Negative obstacle margin/slack은 hard collision horizon과 다른 full-obstacle diagnostic 범위에서 계산되는 ranking diagnostic이므로 별도 safety failure로 재해석하지 않았다. 상세 행은 `vue036_recovery.csv`에 있다.

## Old oracle label correction

| Scope | Old hard-positive | Oracle v2 hard-positive | Old negative -> v2 positive |
|---|---:|---:|---:|
| PILOT | 4/9 | 5/9 | 1: V2E09 |
| DEVELOPMENT | 23/40 | 23/40 | 0 |
| VALIDATION_SEEN_AFTER_V1 | 24/37 | 25/37 | 1: VUE036 |
| **전체** | **51/86** | **53/86** | **2** |

Old negative 35개 중 2개, 즉 **5.7%**가 v2 positive로 바뀌었다. V2E09는 old search가 v2 best tuple을 요청하지 않은 coverage gap이고, VUE036은 refinement miss다. 기존 artifact/label은 덮어쓰지 않았으며 비교표는 `old_vs_v2_oracle_labels.csv`다.

수치상의 underestimate는 2 event지만, false negative가 실제 존재하므로 old oracle은 P3 feasibility upper bound로 사용할 수 없다.

## Frozen evaluation contract

- `HARD_VALID_P3 := exact frozen hard_valid == 1`
- `USABLE_VALID_P3 := HARD_VALID_P3 AND !exit_reaches_next_obstacle AND braking_deficit_m <= 1e-9`

추가 두 조건은 기존 production ranking의 물리적 의미와 같은 epsilon을 사용한다. Obstacle margin, normalized slack, velocity loss, center margin 등에는 새 threshold를 만들지 않았다. 정확한 식과 hard/ranking/diagnostic 책임 분리는 [diagnostic semantics audit](diagnostic_semantics_audit.md)와 [evaluation contract](evaluation_contract.md)에 있다.

## Frozen method 재평가

Recovery 분모는 production hard-valid가 0인 episode 중, 각각 oracle-v2 hard-feasible 또는 usable-feasible인 episode다.

| Scope / method | K | Hard recovery | Usable recovery | Candidate rows | Validator executions |
|---|---:|---:|---:|---:|---:|
| DEVELOPMENT / production | actual | 0/23 | 0/17 | 175 | 175 |
| DEVELOPMENT / H3 | 24 | 13/23 (56.5%) | 8/17 (47.1%) | 550 | 472 |
| DEVELOPMENT / H4-A | 24 | **16/23 (69.6%)** | **11/17 (64.7%)** | 550 | 436 |
| DEVELOPMENT / H4-B | 12 | **16/23 (69.6%)** | **11/17 (64.7%)** | 276 | 223 |
| VALIDATION_SEEN / production | actual | 0/25 | 0/19 | 171 | 171 |
| VALIDATION_SEEN / H3 | 24 | 14/25 (56.0%) | 10/19 (52.6%) | 600 | 526 |
| VALIDATION_SEEN / H4-A | 24 | **14/25 (56.0%)** | **11/19 (57.9%)** | 600 | 470 |
| VALIDATION_SEEN / H4-B | 12 | **14/25 (56.0%)** | **11/19 (57.9%)** | 300 | 231 |

DEVELOPMENT hard-success set에서 H3의 13개는 전부 H4-A와 겹치며, H4-A-only는 DVE012, DVE014, DVE024다. VALIDATION_SEEN hard-success는 intersection 13, H3-only VUE013, H4-A-only VUE027로 순증가가 없다. Usable-success는 DEVELOPMENT intersection 8 + H4-A-only 3, VALIDATION_SEEN intersection 10 + H4-A-only VUE027이다.

따라서 H4-A는 DEVELOPMENT에서는 genuine advantage가 있지만, validation seen에서 hard-valid 우위는 재현되지 않았고 usable-valid는 1 event만 앞선다. H4-B가 절반 K로 H4-A와 동일한 success set을 낸 사실도 H4-A의 현재 geometry rule 자체보다 factor basis와 top-K 선택 문제가 더 중요할 가능성을 지지한다. 이는 seen-data 진단이며 holdout 성능 주장이 아니다.

## Full/factorized coverage와 남은 miss

Complete frozen factor pool은 DEVELOPMENT hard 18/23, usable 13/17을 덮었고 VALIDATION_SEEN hard 16/25, usable 11/19를 덮었다. 따라서 complete factor pool도 Reference Oracle v2의 upper bound가 아니다. Reference Oracle v2만 선언된 finite domain의 reference upper bound 역할을 한다.

H4-A가 놓친 oracle-v2 hard-feasible episode 18개의 descriptive taxonomy는 다음과 같다.

| Mechanism | DEVELOPMENT | VALIDATION_SEEN | Evidence rule |
|---|---:|---:|---|
| Candidate-budget truncation | 2 | 2 | Complete pool에 valid가 있으나 첫 H4-A 순위가 K=24 밖 |
| Probe/root or `d_mid` selection | 1 | 5 | 같은 production target/transition은 있으나 필요한 midpoint가 없음 |
| Multi-parameter or factor-space coverage | 4 | 4 | v2 best target/lateral tuple 자체가 frozen factor basis 밖 |
| Isolated transition-only miss | 0 | 0 | 확인되지 않음 |
| Isolated lateral-pair-only miss | 0 | 0 | 별도 원인으로 분리되지 않음 |

Budget 사례는 DVE029(rank 62), DVE039(rank 116), VUE013(rank 38), VUE025(rank 221)다. 나머지는 K를 키우는 것만으로 복구되지 않는다. 개별 근거는 `remaining_failure_taxonomy.csv`에 있다. `MULTI_PARAMETER_OR_FACTOR_SPACE_COVERAGE`는 현재 evidence로 두 원인을 더 세밀하게 식별할 수 없어 합쳐 둔 descriptive label이다.

## 다음 연구 판단

1. **Old oracle underestimate:** old negative의 5.7%(2/35)가 false negative였다. 절대 수는 작아도 upper-bound 신뢰성은 깨졌다.
2. **Meaningful upper bound:** Oracle v2는 선언 finite domain 안에서 의미 있는 reference upper bound다. Complete frozen factor pool은 dev hard 78.3%, validation hard 64.0%만 덮어 upper bound가 아니다.
3. **H4-A vs H3:** dev에서는 +3 hard/+3 usable의 실제 우위가 있지만 validation hard는 동률이고 usable은 +1뿐이다. H4-A v1의 일반화 우위는 강하지 않다.
4. **남은 원인:** K truncation 4, probe/root-or-midpoint 6, multi-parameter/factor-space 8이다. 독립 transition-only 원인은 관찰되지 않았다.
5. **연구 방향:** H4-A v1의 개별 규칙을 미세 조정하기보다, 더 일반적인 geometry-conditioned factor ranking/top-K selection과 factor-space coverage를 연구하는 편이 근거가 강하다. 이 문서는 새 방법을 설계하거나 threshold를 조정하지 않는다.
6. **Holdout 전 필수 보완:** oracle grid/bounds/transition-set 민감도를 seen data에서만 사전 고정하고, ephemeral `/tmp` harness를 source+build recipe와 함께 versioned freeze하며, 방법/metric/SHA를 완전히 동결하고, final holdout은 단 한 번의 immutable pipeline으로 평가해야 한다. 연속 공간 completeness나 factor-pool upper bound라는 과장된 표현도 금지해야 한다.

## Frozen provenance

| Artifact | SHA-256 |
|---|---|
| `oracle_v2_spec.json` | `62b63b8ab05d5a73565398141bd05a9db4a102541855fb2e3634d85b18a6bffe` |
| `evaluation_contract.json` | `226b1b44a9bea6adf26715658f36ae8e7b2c322e030f363270728f9e044b1dad` |
| exact validator harness binary | `8b23f2b6df54c2e93794ee087f3ebd7a14f9921544af4f97eef755535e045e7e` |
| harness source snapshot | `d7050e61a1f2c582fa3a08fc5f94cad4c8f378b4bc2a5ce1b11162174180e69a` |
| dataset split manifest | `c57cfe8e57dfca4bb318d30047e8f7215f6994e88a39babfbdbb10ce7637b2f4` |
| frozen method implementation | `900a262e49436fa75a1567d49b7823ef4e4cb7f08d313389cd2de26c647c3722` |

`computation_summary.csv`의 aggregate 행에도 요구된 네 SHA를 반복 기록했다.

## Holdout boundary note

Oracle runner는 split manifest를 structured data로 열지 않고 raw-byte SHA만 계산하며, materialized seen input 9+40+37만 주소화한다. Oracle 계산·label·method 비교에는 FINAL_HOLDOUT event ID, feature, outcome, input을 전혀 사용하지 않았다.

다만 사전 scope 점검 중 수동 진단에서 split manifest를 role별 count로만 집계해 `FINAL_HOLDOUT_UNSEEN=37`이라는 역할 수는 노출되었다. Holdout 행의 event ID/feature/outcome은 출력·분석하지 않았고 이후 어떤 계산에도 사용하지 않았다. Manifest와 final-holdout assignment는 변경하지 않았다. 이 경계 사건을 숨기지 않고 기록한다.

