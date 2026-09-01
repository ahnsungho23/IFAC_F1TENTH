# GQSC/P3 연구 상태 재구성

## 1. 체크포인트와 계보

- 현재: branch `research`, HEAD `9a55216a2089a78f3a2b4c7080bf2680bca7444e`.
- 시작 시 working tree는 clean이었고 `GQSC_DATA_ROOT`는 `/home/sungho/Documents/GitHub/GQSC_RECOVERY_20260901/research_data/F1TENTH_research_data`로 정확히 resolve됐으며 허용된 `rosbags_original/` root가 존재했다. 이 task에서는 rosbag을 열거나 재평가하지 않았다.
- Validation v1 진단과 Oracle v2/R3 pre-checkpoint의 권위 commit: `1fe52ed5ea6c64d1deb2dcb9059e3ca64fa5f83d`, tag `p3_r3_k12_pre_holdout`.
- 마지막 OS 이전 과학 상태: `ca621fd9d7c7aa0e3d30ee9c0e43a1599b98da79` (`GQSC_S1_LIVE_RUNTIME_ENVIRONMENT_LIMITED`). 그 직전 공개된 smoke checkpoint는 `0dfb345` / `paper/gqsc-s1-smoke-v1`이다.
- `ca621fd..HEAD`에서 frozen contract, P3 shadow/generator, R3, raceline planner 핵심 blob은 동일하다. 현재 HEAD의 후속 변경은 ROS 2 Humble/runtime/map 복구가 중심이며 이 진단에서 planner 알고리즘 변경으로 해석하지 않는다.

실제 Git graph에서 확인한 주요 tag: `research_instrumentation_v1`→`6ba46db`, `research_instrumentation_v2`→`80ae205`, `p3_r3_k12_pre_holdout`→`1fe52ed`, `p3_r3_k12_final_offline_v1`→`41391cf`(tag target만 확인, 안전 경계상 결과 내용은 진단 근거로 열람하지 않음), `p3_r3_k12_functional_integration_v1`→`17d853f`, `p3_r3_k12_exact_optimized_v2`→`ec7aca4`, `gqsc_v3_geometry_general_closed_loop_candidate`→`8a8dcd9`, `gqsc_s1_pre_smoke_v1`→`534e5e6`, `paper/baseline`→`217ebae`, `paper/gqsc-s1-smoke-v1`→`0dfb345`.

## 2. 권위 아티팩트

| 아티팩트 | producer / split | commit/tag | 지위 |
|---|---|---|---|
| `planning_study/p3_geometry_conditioned_validation_v1/` | `run_frozen_validation.py`; 37 validation episodes | `1fe52ed` / `p3_r3_k12_pre_holdout`에 보존 | 최초 Validation v1 결과 |
| `planning_study/p3_validation_v1_diagnosis/` | `run_diagnosis.py`; validation 37만 사용 | `1fe52ed` / `p3_r3_k12_pre_holdout` | VUE036 및 초기 hard/usable 진단; 일부 분모는 후속 Oracle v2가 대체 |
| `planning_study/p3_reference_oracle_v2/` | `run_reference_oracle_v2.py`; pilot 9, development 40, validation 37의 이미-seen 범위 | `1fe52ed` / `p3_r3_k12_pre_holdout` | 최신 Oracle/metric/accounting 권위 |
| `planning_study/p3_geometry_factor_ranking_v2/` | `run_factor_ranking_v2.py`; seen development+validation factor ranking | `1fe52ed` / `p3_r3_k12_pre_holdout` | R3 선택 및 post-diagnosis seen 성능 |
| `planning_study/p3_r3_k12_integration_v1/` | integration/parity harness | `17d853f` / `p3_r3_k12_functional_integration_v1` | 통합 증거 |
| `planning_study/p3_r3_k12_exact_topk_optimization_v2/` | `generate_tables.py`; exact optimization/parity | `ec7aca4` / `p3_r3_k12_exact_optimized_v2` | exact Top-K 최적화 증거 |
| `planning_study/gqsc_s1_closed_loop_candidate_v1/` 및 `gqsc_s1_callback_architecture_v1/` | `run_s1_integration_study.py`, `run_callback_architecture_study.py`; frozen S1 parity/runtime | `534e5e6` / `gqsc_s1_pre_smoke_v1` | 현재 후보/콜백 구조의 권위 |
| `planning_study/gqsc_s1_closed_loop_smoke_v1/` | 4-case deterministic smoke | `0dfb345` / `paper/gqsc-s1-smoke-v1` | 제한된 폐루프 기능 증거 |
| `planning_study/gqsc_s1_live_realtime_v1/` | observation-only live timing | `ca621fd` | 마지막 pre-migration 실시간성 진단 |

Validation 분류의 직접 근거는 `planning_study/p3_validation_v1_diagnosis/validation_seen_after_v1_manifest.csv:2-38`이다. 37행 모두 원 split은 `VALIDATION_UNSEEN`, 진단 이후 overlay는 `VALIDATION_SEEN_AFTER_V1`이며 split 수정은 없다. 이 재구성은 그 37건으로 새 선택·최적화를 하지 않았다.

## 3. VUE036

결론은 **P3-family infeasibility가 아니라 구 Oracle의 refinement miss/false negative**이다.

- 당시 production은 3개 candidate를 construct/validate했지만 hard-valid가 0이었다 (`planning_study/p3_reference_oracle_v2/frozen_method_event_results.csv:302`). 첫 returned production path는 RIGHT, `d_target=d_mid=-0.574961319743...`, entry `0.514581093015...`, exit `0.497168416257...`, `ZERO_BOUNDARY_SHORT`였고 directional-curvature gate에서 실패했다 (`planning_study/p3_geometry_conditioned_validation_v1/validation_oracle_summary.csv:37`).
- witness: RIGHT, `d_target=-0.672589576947...`, `d_mid=-0.724657980790...`, entry factor `0.514581093016...`, exit factor `0.497168416257...`, digest `95a7394f79189d58`; exact hard-valid이다 (`planning_study/p3_validation_v1_diagnosis/vue036_oracle_miss_diagnosis.csv:2-4`).
- 동일 witness의 production-template rank는 H3 23, H4-A 13, H4-B 5였다. 이는 family witness/rank 증거이지 당시 production 성공을 의미하지 않는다 (`.../vue036_oracle_miss_diagnosis.csv:2-4`).
- 구 Oracle은 coarse 0.05, top-30 주위 ±0.04/0.01 refinement를 사용했고 해당 local cell을 refinement하지 않았다. local 0.01 grid의 8/64가 valid였다 (`.../vue036_oracle_miss_diagnosis.csv:5`).
- 그래서 domain, transition resolution, family, validator, guard/filter mismatch가 아니라 `ORACLE_REFINEMENT_MISS`로 판정했다 (`planning_study/p3_validation_v1_diagnosis/oracle_completeness_audit.md:13-39`).
- Oracle v2 finite exhaustive search에서는 153,524 request, 75,264 validation, hard/usable witness 52개를 확인했다 (`planning_study/p3_reference_oracle_v2/README.md:22-32`).

## 4. V2E09

V2E09도 **구 Oracle search-coverage false negative**이다. Oracle v2에서 111,906 request, 38,290 validation, hard/usable 8개를 확인했다. best tuple은 RIGHT, `d_target=-0.043039680887...`, `d_mid=-0.402674917276...`, entry `0.713669411656...`, exit `0.497168416257...`, digest `b82e32da7c4203eb`이다 (`planning_study/p3_reference_oracle_v2/old_vs_v2_oracle_labels.csv:10`). 현재 frozen S1 parity에서는 여전히 hard/usable 0이므로 Oracle label은 고쳐졌지만 selector coverage 문제는 남았다 (`planning_study/gqsc_s1_closed_loop_candidate_v1/frozen_parity.csv:10`).

## 5. Oracle 완전성 결론

- 구 Oracle은 deterministic sampled search였지만 infeasibility certificate가 아니었다 (`planning_study/p3_validation_v1_diagnosis/oracle_completeness_audit.md:5-7`).
- Oracle v2는 선언된 finite tuple domain을 top-N/refinement pruning 없이 exhaustive하게 열거한다. 연속 공간 전체나 P3 밖 family의 완전성을 주장하지 않는다 (`planning_study/p3_reference_oracle_v2/oracle_v2_spec.md:8-33`).
- 구 Oracle negative 35건 중 false negative는 2건(`V2E09`, `VUE036`, 5.7%)이었다. 구 positive→v2 negative는 0건이므로 확인된 false positive도 0건이다. validation hard-feasible은 24→25, pilot은 4→5로 교정됐다 (`planning_study/p3_reference_oracle_v2/README.md:34-45`; `planning_study/p3_reference_oracle_v2/old_vs_v2_oracle_labels.csv:2-87`).
- 따라서 구 Oracle 기반 upper bound와 infeasible 판정은 폐기/재해석해야 한다. 최신 비교의 분모는 hard 25, usable 19이다 (`planning_study/p3_reference_oracle_v2/README.md:58-71`).

## 6. HARD_VALID 계열과 USABLE_VALID_P3

정의는 다음처럼 정리된다.

1. `HARD_VALID`는 결과 ledger의 `hard_valid==1`, 즉 해당 평가기가 hard constraint를 모두 통과했다는 원시 판정이다. 초기 문서에서는 Oracle/production 공통 표기였으나, 평가기 버전을 명시하지 않으면 비교가 모호하다.
2. 최신 `HARD_VALID_P3`는 **exact production P3 hard validator에서 `hard_valid==1`이고 failure reason이 비어 있음**이다 (`planning_study/p3_reference_oracle_v2/evaluation_contract.md:6-10`). hard gate는 entry continuity, 최소 point 수, reference ordering, center/rotated-footprint track bounds, horizon-limited obstacle collision, slope, curvature-rate, directional-curvature이다 (`planning_study/p3_reference_oracle_v2/diagnostic_semantics_audit.md:13-27`).
3. 최신 `USABLE_VALID_P3`는 `HARD_VALID_P3`이면서 next-obstacle exit conflict가 없고 braking deficit가 `<=1e-9`인 경우다 (`planning_study/p3_reference_oracle_v2/evaluation_contract.md:12-23`). obstacle margin, braking slack, velocity 등 diagnostic 값이 음수라는 이유만으로 추가 탈락시키지 않는다 (`.../evaluation_contract.md:25-35`; `.../diagnostic_semantics_audit.md:43-51`).
4. 초기 diagnosis의 `USABLE_VALID_P3_DIAGNOSIS_V1`은 25 hard-feasible 모두를 분모로 사용하고 VUE009/011/014/036을 보수적으로 제외해 H4-A `10/25`로 보고했다 (`planning_study/p3_validation_v1_diagnosis/README.md:95-127`). 이 정의/분모는 최신 계약의 H4-A `11/19`로 대체됐다.

권장 metric mapping:

- 수학적 family capability: 선언 domain 안 `HARD_VALID_P3` 존재 여부와, 필요하면 별도 `USABLE_VALID_P3` 존재 여부를 함께 보고한다.
- Oracle completeness: finite domain의 `HARD_VALID_P3` 존재를 exhaustive하게 증명한다. usable completeness는 별도 usable predicate로 보고한다.
- production selector capability: Oracle-v2 hard/usable feasible 집합에 대한 selector의 hard/usable recovery를 둘 다 보고한다.
- 실제 closed-loop usefulness: `USABLE_VALID_P3`는 필요 조건이지 충분 조건이 아니다. lifecycle, controller consumption, publication/tracking, collision/margin, callback latency까지 별도 폐루프 증거가 필요하다.

## 7. Validation v1 교정 회계와 H4-A

최소 숫자는 [validation_v1_corrected_accounting.csv](validation_v1_corrected_accounting.csv)에 있다.

- 총 37, production success 0 (`planning_study/p3_geometry_conditioned_validation_v1/README.md:34-50`).
- 최초 구 Oracle: hard-feasible 24, negative 13; H3/H4-A/H4-B 모두 13/24. H4-A-only VUE027, H3-only VUE013으로 net improvement가 없었다 (`.../README.md:54-97`, `:168-178`).
- known-exception sensitivity에서 VUE036을 추가하면 H3/H4-A 모두 14/25였다 (`.../README.md:75-97`).
- Oracle v2 교정: hard-feasible 25/negative 12, usable-feasible 19/non-usable 18. H3 `14/25 hard`, `10/19 usable`; H4-A 및 H4-B `14/25 hard`, `11/19 usable` (`planning_study/p3_reference_oracle_v2/README.md:58-71`).

H4-A의 원 정의는 fixed lateral priority에 geometry-conditioned transition ordering을 더한 것이다. entry는 `D_entry<0.25`, `max_curvature>0.5`, 또는 obstacle distance `>2`이면 long을 우선하고, exit는 merge `<2`면 short, merge `>=2`이면서 curvature sign change 또는 track-width variation `>0.6`이면 long을 우선한다. score는 transition error + `0.20*lateral priority`이다 (`planning_study/p3_geometry_conditioned_method_v1/README.md:54-84`). 개발에서는 H4-A K24가 `16/23`; validation에서는 최초 `13/24`, 교정 후 `14/25`로 H3와 동일했다 (`.../p3_geometry_conditioned_method_v1/README.md:119-135`; `.../p3_geometry_conditioned_validation_v1/README.md:75-97`).

교정 후 H4-A hard 미회복 11건의 정확한 행별 상태는 [unresolved_cases.csv](unresolved_cases.csv)다. taxonomy는 probe/root 또는 `d_mid` 선택 5건, multi-parameter/factor-space coverage 4건, candidate-budget truncation 2건이다 (`planning_study/p3_reference_oracle_v2/remaining_failure_taxonomy.csv:9-19`). 이 중 Oracle-v2 usable target은 8건이다.

## 8. Development 대 validation mechanism shift

- development 미회복 7건: probe 2, transition 1, other 4. validation 미회복 11건: budget 2, multi-parameter 4, probe 5 (`planning_study/p3_validation_v1_diagnosis/development_vs_validation_mechanisms.csv:2-6`).
- 전체 validation은 development보다 obstacle/entry longitudinal distance가 짧고(mean `0.250` vs `0.751`, SMD `-0.470`), max curvature가 높았다(`0.551` vs `0.459`, SMD `+0.503`). Oracle-positive subset에서도 같은 방향이었다 (`planning_study/p3_validation_v1_diagnosis/geometry_distribution_comparison.csv:4-11`, `:19-26`).
- H4-A의 desired-entry-long 비율은 development `0.826`에서 validation `0.958`로 올라 binary rule이 거의 상수화됐다 (`.../geometry_distribution_comparison.csv:29`).

따라서 “geometry가 무효”가 아니라, 개발 실패 mode에 맞춘 binary ordering이 validation의 짧은 entry/높은 curvature 및 factor/probe/budget coverage miss를 충분히 분리하지 못했다는 결론이 가장 잘 지지된다 (`planning_study/p3_validation_v1_diagnosis/README.md:154-173`). 이는 관측된 mechanism+geometry shift와 incomplete candidate coverage의 조합이며, Oracle/production guard 문제가 H4-A의 net-zero를 설명한다는 증거는 없다.

## 9. 진단 이후 현재 코드가 해결한 것

- R3 K12는 validation에서 `19/25 hard`, `18/19 usable`로 H4-A의 `14/25`, `11/19`를 개선했고 남은 usable miss는 VUE035 하나였다 (`planning_study/p3_geometry_factor_ranking_v2/recovery_by_budget.csv:35,47`; `.../README.md:62-80,99-129`). 이 분석은 이미-seen validation을 사용했으므로 새 unseen claim이나 추가 모델 선택 근거가 아니다.
- frozen S1은 H4-A hard 미회복 11건 중 VUE004/008/013/018/020/025/026/032를 hard-recover하고, Oracle-v2 usable인 8건 중 VUE004/008/018/020/025/026/032의 7건을 usable-recover한다 (`planning_study/gqsc_s1_closed_loop_candidate_v1/frozen_parity.csv:54,58,63,68,70,75,76,82`). VUE010/012/035는 hard miss이며 VUE035만 Oracle-v2 usable target이다 (`.../frozen_parity.csv:60,62,85`).
- 4개 deterministic low-speed scenario는 collision 0과 lifecycle/publication/tracking gate를 통과했다. 그러나 fresh-generation 16-sample evaluator p95 `29.939 ms`, full callback p95 `33.421 ms`로 25 ms를 넘었다 (`planning_study/gqsc_s1_closed_loop_smoke_v1/README.md:54-101`).
- 마지막 live 진단에서 같은 frozen query는 standalone warm p95 `17.006 ms`, live evaluator `27.802 ms`, callback `30.904 ms`; performance core p95 `22.008 ms`, efficiency core p95 `30.952 ms`였다. 결론은 알고리즘 실패가 아니라 `GQSC_S1_LIVE_RUNTIME_ENVIRONMENT_LIMITED`이며 realtime p95 qualification은 아직 없다 (`planning_study/gqsc_s1_live_realtime_v1/README.md:50-104,128-150`, commit `ca621fd`).

## 10. 신뢰/교정/미해결

신뢰:

- Oracle v2의 선언 finite domain 완전성, exact hard/usable 계약, VUE036/V2E09 false-negative 판정.
- Validation 37의 `VALIDATION_SEEN_AFTER_V1` 분류.
- frozen S1의 offline parity와 제한된 4-case smoke 기능 결과.

교정:

- 구 Oracle hard-feasible validation 분모 24→25.
- early diagnosis H4-A usable `10/25`→최신 계약 `11/19`.
- “구 Oracle negative = family infeasible/upper bound” 해석 폐기.

미해결:

- frozen S1의 배치 조건별 live fresh-generation p95 25 ms 충족 여부.
- 현재 S1이 miss하는 VUE035형 usable multi-parameter/factor-space coverage class와 V2E09형 search-coverage class의 mechanism-general 해결.
- 4-case smoke 밖의 대규모 폐루프 일반화. 단, 이미-seen validation 37을 재최적화에 사용해서는 안 된다.

UNKNOWN:

- 현재 증거만으로 배치 대상 host 전반에서 p95 25 ms를 보장할 수 없다.
- VUE035/V2E09의 한 사례에 맞추지 않고도 어떤 최소 selector 변경이 일반화되는지는 아직 증명되지 않았다.
