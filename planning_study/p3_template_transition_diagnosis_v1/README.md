# P3 template / transition failure diagnosis v1

## 결론

이 진단은 이미 공개된 **DEVELOPMENT 40 episode**만 사용했다. 새로운 oracle을 실행하지 않았고, `VALIDATION_UNSEEN` 및 `FINAL_HOLDOUT_UNSEEN`은 읽지 않았다.

13개 `TEMPLATE_OR_TRANSITION_SELECTION` 사례의 최소 복구 경로는 두 종류였다.

- **9/13 `COVERAGE_FAILURE_MISSING_OPTION`**: M1 `ZERO_BOUNDARY_SHORT/SPAN`의 동일한 side, `d_target`, entry/exit scale을 유지하고 `d_mid`만 바꾸면 hard-valid P3가 존재했다. Production zero-interface 후보는 `d_mid=d_target`만 나타냈으므로 필요한 full lateral tuple이 없었다. 이는 scale coverage miss가 아니다.
- **4/13 `SELECTION_FAILURE_WITH_EXISTING_OPTION`**: `d_target`과 `d_mid`를 그대로 유지하고, 그 이벤트의 다른 production 후보가 이미 사용한 entry/exit scale pair와 교차 결합하면 hard-valid P3가 존재했다. 즉 새 transition scale이 아니라 기존 option의 조합/선택 실패다.

따라서 13건의 지배적 최소 원인은 “새 scale 부족”이 아니라 **lateral tuple과 기존 transition option을 제한적으로 결합한 candidate-generation coverage**다. 9:4 기준으로는 missing full option이 우세하지만, 성공한 scale pair 자체는 13건 모두 해당 이벤트의 production option set 안에 있었다.

## 범위와 재현 계약

- Repository HEAD: `80ae205fd470f537bd1e5449fb906c5548f6653a`
- 입력: `p3_mapping_research_corpus_v1`의 `mapping_failure_taxonomy.csv`, `development_oracle_summary.csv`, `raw_oracle/production_candidates.csv`, `raw_oracle/oracle_valid_candidates.csv`, `raw_oracle/inputs/DVE*.event`
- geometry 복원: 앞 단계의 read-only event parser/corridor helpers를 재사용했다.
- event selection: `mapping_failure_taxonomy.csv`에서 이미 `TEMPLATE_OR_TRANSITION_SELECTION` 또는 `OTHER`로 열린 DEVELOPMENT ID만 선택했다.
- 실행 결과: new oracle search `0`, validation/holdout row read `0`.
- 분석 스크립트: [`analyze_template_transition.py`](analyze_template_transition.py)

핵심 입력 SHA-256은 다음과 같다.

| 입력 | SHA-256 |
|---|---|
| `mapping_failure_taxonomy.csv` | `b0f90425e3114cbc8bd4c8397d619f17af79f8f269f87d86d2720aa1a967ae12` |
| `development_oracle_summary.csv` | `6494eb5c8f7f60c3c09bcc2806657c3f668a31acb4267695a00c97625ebbfbc9` |
| `raw_oracle/production_candidates.csv` | `43c047813a6b74316b43796b2d1cc8312f537d489c4d9bcea95bb3224ed1130e` |
| `raw_oracle/oracle_valid_candidates.csv` | `6dadbf4f845539e10b255cbc7e1150cd3ced47c780041dedb0397cc96f21a0d9` |
| `raw_oracle/execution_provenance.json` | `67c430fa6a7839bcb27b715e3f5bec90a170cd1a427197df2d8735028ca36cd7` |

Oracle pilot은 각 이벤트에서 production이 실제 생성한 `(entry_scale, exit_scale)` pair만 P3 direct oracle에 전달했다. 따라서 아래 oracle 성공을 근거로 “새 scale을 발견했다”고 해석할 수 없다. 비교에서 full tuple은

```text
(side, d_target, d_mid, entry_scale, exit_scale)
```

이며, scale option 존재와 full tuple 존재를 구분했다.

## 1. 13개 사례의 exact decomposition

길이는 `L_entry=z1-z0`, `L_exit=z4-z3`로 복원했다. 다음 표는 strict oracle-valid 후보 중 production 후보와의 변경 coordinate 수, 이어서 총 길이/측방 변경량이 최소인 경로다. 모든 production 대표는 M1 zero-interface이고 probe가 없는 `M1_BRANCH_COMPLETE_ACTIVE_SET_CLOSURE` 경로다.

| ID | production | 최소 복구 경로 | entry scale, 길이 변화 | exit scale, 길이 변화 | `d_target` | `d_mid` production → oracle | 판정 |
|---|---|---|---|---|---:|---:|---|
| DVE001 | M1/ZERO_BOUNDARY_SPAN | d_mid freedom | 0.713669, +0.000 m | 0.497168, +0.000 m | -0.471806 | -0.471806 → -0.500000 | missing option |
| DVE006 | M1/ZERO_BOUNDARY_SPAN | d_mid freedom | 0.713669, +0.000 m | 0.497168, +0.000 m | 0.162656 | 0.162656 → 0.200000 | missing option |
| DVE012 | M1/ZERO_BOUNDARY_SHORT | existing scale cross | 0.514581, +0.000 m | 0.497168 → 0.604987, **+0.270 m** | -0.417266 | unchanged | existing-option selection |
| DVE015 | M1/ZERO_BOUNDARY_SHORT | d_mid freedom | 0.514581, +0.000 m | 0.497168, +0.000 m | -0.341835 | -0.341835 → -0.350000 | missing option |
| DVE017 | M1/ZERO_BOUNDARY_SHORT | d_mid freedom | 0.514581, +0.000 m | 0.497168, +0.000 m | -0.341835 | -0.341835 → -0.350000 | missing option |
| DVE018 | M1/ZERO_BOUNDARY_SHORT | existing scale cross | 0.514581, +0.000 m | 0.497168 → 0.604987, **+0.270 m** | -0.404842 | unchanged | existing-option selection |
| DVE019 | M1/ZERO_BOUNDARY_SHORT | d_mid freedom | 0.514581, +0.000 m | 0.497168, +0.000 m | -0.642876 | -0.642876 → -0.650000 | missing option |
| DVE022 | M1/ZERO_BOUNDARY_SPAN | d_mid freedom | 0.713669, +0.000 m | 0.497168, +0.000 m | 0.159922 | 0.159922 → 0.250000 | missing option |
| DVE023 | M1/ZERO_BOUNDARY_SHORT | d_mid freedom | 0.514581, +0.000 m | 0.497168, +0.000 m | 0.535064 | 0.535064 → 0.550000 | missing option |
| DVE024 | M1/ZERO_BOUNDARY_SHORT | existing scale cross | 0.514581, +0.000 m | 0.497168 → 3.947357, **+8.655 m** | -0.462797 | unchanged | existing-option selection |
| DVE029 | M1/ZERO_BOUNDARY_SPAN | d_mid freedom | 0.713669, +0.000 m | 0.497168, +0.000 m | -0.409740 | -0.409740 → -0.450000 | missing option |
| DVE031 | M1/ZERO_BOUNDARY_SPAN | existing scale cross | 0.713669 → 1.310934, **+0.071 m** | 0.497168 → 3.947357, **+8.655 m** | -0.399376 | unchanged | existing-option selection |
| DVE038 | M1/ZERO_BOUNDARY_SPAN | d_mid freedom | 0.713669, +0.000 m | 0.497168, +0.000 m | -0.478327 | -0.478327 → -0.500000 | missing option |

DVE029에는 secondary recovery도 있다. `d_target=d_mid=-0.409740`을 유지하고 entry/exit scale을 `(0.713669, 0.497168)`에서 `(1.310934, 0.593390)`으로 바꾸는 기존-scale cross도 valid였다. 그러나 최소 변경 coordinate 수가 1인 `d_mid`-only 경로가 있으므로 primary는 missing option으로 정했다.

정밀 category는 다음과 같다.

- DVE012, DVE018, DVE024: `EXIT_TRANSITION_TOO_SHORT | TEMPLATE_FAMILY_SELECTION_MISS | INTERACTION_WITH_PROBE`
- DVE031: `ENTRY_TRANSITION_TOO_SHORT | EXIT_TRANSITION_TOO_SHORT | ENTRY_EXIT_COMBINATION_MISS | TEMPLATE_FAMILY_SELECTION_MISS | INTERACTION_WITH_PROBE`
- 나머지 9건: `TEMPLATE_FAMILY_SELECTION_MISS | INTERACTION_WITH_D_MID | OTHER_TEMPLATE_MECHANISM`
- DVE029의 secondary route에는 entry/exit-too-short 및 probe interaction 증거도 있다.
- `ENTRY_TRANSITION_TOO_LONG`, `EXIT_TRANSITION_TOO_LONG`, `SCALE_COVERAGE_MISS`는 이 13건에서 지지되지 않았다.

모든 exact production/oracle scalar와 기록된 margin은 [`production_vs_oracle_template.csv`](production_vs_oracle_template.csv), 최소 경로와 secondary 경로는 [`template_case_summary.csv`](template_case_summary.csv), 최종 category는 [`transition_failure_taxonomy.csv`](transition_failure_taxonomy.csv)에 있다.

`oracle_*_margin` 열은 raw oracle의 진단 metric을 그대로 보존했다. 일부 `hard_valid=1` 행에서도 `obstacle_margin_m` 또는 `minimum_normalized_safety_slack`가 음수이므로, 이 진단에서는 그 파생 metric의 부호를 별도의 validator pass/fail로 재해석하지 않았다. authoritative hard-valid 증거는 evaluator의 `hard_valid=1`과 빈 `first_failure_reason`이다.

## 2. Existing option인가, missing option인가

두 질문은 서로 다른 단위로 답해야 한다.

1. **Scale-pair 단위:** 13/13 모두 성공 pair가 해당 이벤트의 production option set에 있었다. 새 scale coverage가 필요했다는 증거는 0건이다.
2. **Full-tuple 단위:** 성공 full tuple은 13/13 모두 production에 없었다. 그중 4건은 기존 M0 scale pair를 동일 lateral tuple에 cross-select하지 않은 경우이고, 9건은 zero-interface가 필요한 `d_mid != d_target`을 표현하지 않은 경우다.

따라서 “기존 discrete option이 충분한가?”의 답은 **transition scale 집합만 보면 yes, 현재의 결합된 candidate option 집합을 보면 no**다.

## 3. Candidate-generation 이전 geometry

[`geometry_features.csv`](geometry_features.csv)는 ego/obstacle/corridor/reference geometry만 사용한다. obstacle interval은 raw 및 inflation 후 값을 모두 보존하고, corridor bottleneck은 폭·station·center·active obstacle ID를 기록했다. “available merge distance”는 다음 visible obstacle까지의 거리이며, 다음 obstacle이 없으면 15 m lookahead에서 right-censored로 표시했다.

최소 복구 경로별 기술통계(min / median / max)는 다음과 같다.

| feature | d_mid freedom 9건 | existing-scale cross 4건 |
|---|---:|---:|
| ego→obstacle start [m] | 0.035 / 0.327 / 0.924 | **0.112 / 0.155 / 0.157** |
| obstacle span [m] | 0.024 / 0.197 / 0.271 | **0.018 / 0.021 / 0.023** |
| corridor bottleneck width [m] | 0.114 / 0.398 / 0.939 | 0.200 / 0.735 / 0.747 |
| max absolute reference curvature [rad/m] | 0.000 / 0.424 / 0.664 | 0.304 / 0.304 / 0.606 |
| track-width range [m] | 0.063 / 0.150 / 0.650 | 0.163 / 0.406 / 1.338 |
| required exit length [m] | 1.247 / 1.247 / 3.072 | **1.518 / 5.710 / 9.902** |

관찰 가능한 반복은 다음 정도까지다.

- 기존-scale cross 4건은 모두 obstacle start가 ego로부터 약 0.11–0.16 m이고, obstacle span이 약 0.018–0.023 m이며, curvature extractor가 모두 `SIGN_CHANGE`로 분류했다. 모두 production보다 긴 exit가 필요했다.
- DVE012/DVE018은 거의 같은 geometry domain으로, exit를 1.247 m에서 1.518 m로 늘리는 동일 패턴이다. 두 episode를 완전히 독립적인 geometry 증거 2개로 과대계수하면 안 된다.
- DVE024는 높은 reference curvature(최대 0.606 rad/m), 좁은 bottleneck(0.200 m), 다음 visible obstacle까지 0.989 m인 상태에서도 9.902 m exit가 valid였다. DVE031은 큰 track-width 변화(1.338 m)와 9.902 m exit가 같이 나타났다.
- “merge distance가 부족하면 짧은 exit” 패턴은 관찰되지 않았다. 4건 중 3건은 15 m에서 censored였고, 한 건은 merge distance가 약 0.989 m였지만 긴 exit가 필요했다.
- d_mid-freedom 9건은 curvature, corridor 폭, merge distance가 넓게 퍼져 있어 이 13건만으로 단일 geometry threshold를 지지하지 않는다.

`[INFERENCE]` 매우 가까운 obstacle start와 curvature sign-change가 기존 M0 transition option을 M1 lateral tuple과 함께 검토할 유용한 predictor 후보로 보인다. 그러나 표본은 4건이고 DVE012/DVE018이 유사 장면이므로 정책 결론이 아니라 다음 DEVELOPMENT hypothesis일 뿐이다.

## 4. 6개 OTHER 재분류

| ID | refined class | 최소 counterfactual evidence |
|---|---|---|
| DVE002 | `INTERACTION_OF_MULTIPLE_EXISTING_PARAMETERS` | 같은 scale에서 `d_target +0.0936 m`, `d_mid -0.0464 m`; target/mid/scale 단일 변경 valid 없음 |
| DVE007 | `INTERACTION_OF_MULTIPLE_EXISTING_PARAMETERS` | 같은 scale에서 `d_target -0.0024 m`, `d_mid +0.3138 m`; 단일 변경 valid 없음 |
| DVE020 | `INTERACTION_OF_MULTIPLE_EXISTING_PARAMETERS` | `d_target +0.0275 m`, `d_mid -0.0293 m` 및 entry scale 변경; 단일 변경 valid 없음 |
| DVE030 | `INTERACTION_OF_MULTIPLE_EXISTING_PARAMETERS` | 같은 scale에서 `d_target -0.0109 m`, `d_mid -0.1942 m`; 단일 변경 valid 없음 |
| DVE032 | `INTERACTION_OF_MULTIPLE_EXISTING_PARAMETERS` | 같은 scale에서 `d_target +0.0801 m`, `d_mid -0.1199 m`; 단일 변경 valid 없음 |
| DVE037 | `TRUE_UNCLASSIFIED_MECHANISM__D_TARGET_ONLY_COVERAGE_GAP` | 같은 `d_mid=0.537983` 및 scale에서 `d_target 0.8225→0.6000`만 바꾸면 valid |

6건 모두 exact evaluator의 production hard-valid count가 0이므로 valid production 후보를 ranking/lifecycle이 잘못 선택한 사례는 아니다. DVE037은 target-only coverage는 분리되지만 source가 probe/root/transition 중 무엇인지 직접 고립되지 않아 true-unclassified로 남겼다. 나머지 5건도 단일 parameter counterfactual이 없어 probe 또는 transition으로 억지 재분류하지 않았다. 상세 값은 [`other_case_refinement.csv`](other_case_refinement.csv)에 있다.

## 5. Deterministic bounded set coverage

“configuration”은 `(entry_scale, exit_scale)` pair로 정의하고 12 decimal로 canonicalize했다. 각 pair가 oracle-valid한 episode set을 만든 뒤, budget 1–5에서 **모든 조합을 열거**하여 최대 episode coverage를 선택했다. 동률은 scale-pair tuple의 lexicographic order로 결정했다.

이 결과는 **oracle-valid `d_target,d_mid`가 이미 주어졌다는 조건부 transition coverage**다. Production mapping이 해당 lateral tuple을 생성한다는 뜻이 아니다.

| pair budget | exact selected pairs | covered / 23 | 13 template cases | empirical production candidates / validator calls | one-lateral-tuple request upper bound over 40 |
|---:|---|---:|---:|---:|---:|
| 1 | (0.514581, 0.497168) | 16 / 23 = 69.6% | 11 / 13 = 84.6% | 37 / 37 | 40 |
| 2 | + (1.310934, 0.604987) | 21 / 23 = 91.3% | 13 / 13 = 100% | 46 / 46 | 80 |
| 3 | (0.514581,0.497168), (0.514581,0.604987), (1.310934,7.397546) | 22 / 23 = 95.7% | 13 / 13 = 100% | 63 / 63 | 120 |
| 4 | (0.514581,0.497168), (0.514581,0.604987), (1.310934,6.677830), (1.310934,7.397546) | 23 / 23 = 100% | 13 / 13 = 100% | 64 / 64 | 160 |
| 5 | budget-4 동률 optimum에 (0.514581,0.593390) 포함 | 23 / 23 = 100% | 13 / 13 = 100% | 65 / 65 | 200 |

“empirical” 열은 현재 40-event production CSV에서 선택 pair와 일치한 기존 후보/validator 행을 다시 센 replay proxy다. 현재 전체 production constructed candidate는 249개였다. 반면 마지막 열은 각 이벤트·pair마다 lateral tuple 하나만 요청한다고 가정한 단순 상한이다. 미래 selector의 target/root multiplicity가 정의되지 않았으므로 이것을 실제 새 generator의 정확한 계산량으로 읽으면 안 된다. 원자료는 [`configuration_coverage.csv`](configuration_coverage.csv)와 [`candidate_budget_estimate.csv`](candidate_budget_estimate.csv)에 있다.

## 6. Refined DEVELOPMENT taxonomy

23개 oracle-valid episode를 evidence가 직접 지지하는 수준에서 정리하면 다음과 같다.

| refined mechanism | count | oracle-valid 23 대비 |
|---|---:|---:|
| `D_PROBE_DOMINANT` | 3 | 13.0% |
| `S_PROBE_DOMINANT` | 1 | 4.3% |
| template: missing d_mid/full option | 9 | 39.1% |
| template: existing scale-option cross-selection | 4 | 17.4% |
| multi-parameter interaction | 5 | 21.7% |
| true-unclassified d_target-only coverage | 1 | 4.3% |

Oracle-valid episode 중 probe selection에 직접 귀속되는 비율은 **4/23=17.4%**다. 전체 DEVELOPMENT 40을 분모로 하면 4/40=10.0%이며, 나머지 17/40은 `NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN`이므로 이 진단에서 mechanism을 부여하지 않았다.

## 7. 연구 방향에 대한 8개 답

1. **13건의 정확한 원인:** 9건은 M1 zero-interface의 `d_mid=d_target` 제약으로 필요한 full lateral option이 없었고, 4건은 기존 M0 transition pair를 동일 M1 lateral tuple과 교차 선택하지 못했다.
2. **기존 option 선택인가 coverage 부족인가:** 최소 경로 기준 full-option coverage 부족이 9/13으로 우세하고 existing-option selection이 4/13이다. 단, transition scale 자체는 13/13 이미 존재했다.
3. **geometry와 필요한 구성의 반복 관계:** existing-scale cross 4건은 모두 매우 가까운/짧은 obstacle과 curvature sign-change를 공유하며 더 긴 exit가 필요했다. 표본 중복과 작은 n 때문에 predictor 후보이지 freeze 가능한 rule은 아니다.
4. **작은 bounded set의 coverage:** lateral tuple이 주어진다는 조건에서 1 pair가 69.6%, 2 pair가 91.3%, 4 pair가 100%의 23 oracle-success episode를 덮었다. 2 pair는 template 13건을 모두 덮었다.
5. **probe selection 비율:** direct evidence 기준 oracle-valid의 4/23=17.4%(전체 40의 10.0%)다.
6. **최종 DEVELOPMENT taxonomy:** 위 표와 같이 probe 3+1, template 9+4, multi-parameter 5, d_target-only unclassified 1이며 oracle-domain no-valid 17은 별도다.
7. **broader geometry-conditioned bounded P3 generation을 지지하는가:** **DEVELOPMENT-stage 연구 방향으로는 yes**다. 작은 scale set의 조건부 coverage와 반복 geometry가 동기를 제공한다. 그러나 핵심 miss가 scale 추가보다 lateral/transition cross-composition이므로, 단순히 scale 후보만 늘리는 연구로 축소하면 evidence와 어긋난다. validation/holdout을 열기 전 policy freeze와 preregistered test가 필요하다.
8. **다음에 시험할 가장 방어 가능한 hypothesis:** 아래 4개다. 아직 구현하거나 tuning하지 않았다.

## 8. 다음 hypothesis 후보

1. **Bounded cross-composition:** scene별 M1 lateral anchor와 이미 계산된 M0 transition pair를 제한된 수로 교차 결합하면, 새 scale을 도입하지 않고 existing-option selection 4건을 회복할 수 있다.
2. **Zero-interface d_mid freedom:** `d_mid=d_target` 하나뿐인 zero-interface에 소수의 bounded `d_mid` option을 허용하면 9건의 missing full option을 회복할 수 있다. 값/선택법은 아직 미정이다.
3. **Geometry gate for transition reuse:** obstacle-start distance, curvature sign-change, bottleneck width, track-width change를 사용해 short/medium/long existing exit pair 중 소수를 고르면 무조건 Cartesian product보다 작은 budget으로 유지할 수 있다.
4. **Joint lateral option hypothesis:** 단일 parameter로 분리되지 않은 OTHER 5건에는 `d_target`과 `d_mid`를 공동으로 고르는 bounded option이 필요하며, probe-only 또는 scale-only 변경으로는 충분하지 않다.

이 네 hypothesis는 DEVELOPMENT 관찰에서 나온 연구 질문일 뿐이며, 새 mapping 설계·성능 주장·validation 결과가 아니다.

## 산출물

- [`template_case_summary.csv`](template_case_summary.csv)
- [`production_vs_oracle_template.csv`](production_vs_oracle_template.csv)
- [`transition_failure_taxonomy.csv`](transition_failure_taxonomy.csv)
- [`geometry_features.csv`](geometry_features.csv)
- [`other_case_refinement.csv`](other_case_refinement.csv)
- [`configuration_coverage.csv`](configuration_coverage.csv)
- [`candidate_budget_estimate.csv`](candidate_budget_estimate.csv)
- [`plots/`](plots/)

Planner, mapping, validator, ranking, parameter, source build, commit 및 push는 변경하거나 실행하지 않았다.
