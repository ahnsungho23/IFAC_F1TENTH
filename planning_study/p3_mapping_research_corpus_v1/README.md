# P3 Mapping Research Corpus Freeze and Development Diagnosis v1

## 결론

`research_instrumentation_v2`(`80ae205fd470f537bd1e5449fb906c5548f6653a`)의 frozen replay에서
확보한 193개 oracle-eligible failure episode를 **oracle 결과를 보기 전에 episode 단위로 동결**했다.
기존 pilot 9개는 영구적으로 `PILOT_SEEN_DEVELOPMENT_DATA`이며, 나머지 184개를
`DEVELOPMENT` 110개, `VALIDATION_UNSEEN` 37개, `FINAL_HOLDOUT_UNSEEN` 37개로 나눴다.

DEVELOPMENT 중 사전 동결한 40개만 pilot-v2와 동일한 deterministic P3 oracle로 조사했다.
40개 모두 exact lineage와 production candidate-digest reconstruction parity를 통과했고,
23개에서 hard-valid P3가 발견됐다. 그 23개를 직접 진단한 결과는 다음과 같다.

| mechanism | count | oracle-valid 중 share | 전체 검색 episode 중 share |
|---|---:|---:|---:|
| `D_PROBE_DOMINANT` | 3 | 13.0% | 7.5% |
| `S_PROBE_DOMINANT` | 1 | 4.3% | 2.5% |
| `JOINT_S_AND_D_PROBE` | 0 | 0% | 0% |
| `ROOT_FILTERING_DOMINANT` | 0 | 0% | 0% |
| `TEMPLATE_OR_TRANSITION_SELECTION` | 13 | 56.5% | 32.5% |
| `OTHER` | 6 | 26.1% | 15.0% |
| `NOT_IDENTIFIABLE` | 0 | 0% | 0% |

따라서 DEVELOPMENT evidence만으로는 **새로운 `d_probe` policy를 primary research target으로
바로 채택하는 것은 NO-GO**다. `D_PROBE_DOMINANT`는 세 독립 failure episode에서 반복됐지만
모두 동일 bag/reference domain에 속한다. 반면 transition/zero-interface template 관련 evidence가
13개로 훨씬 많다. 이는 `d_probe` 단독 논문보다 probe와 transition/template coverage를 함께 보는
더 넓은 mapping 연구가 우선임을 시사한다. 이는 개발 진단이며 새 mapping이나 parameter를
구현·학습·튜닝한 결과가 아니다.

## 1. Frozen dataset roles

| role | count | oracle outcome 사용 여부 |
|---|---:|---|
| `PILOT_SEEN_DEVELOPMENT_DATA` | 9 | 기존 pilot 결과가 이미 알려진 개발 자료 |
| `DEVELOPMENT` | 110 | 이 중 사전 선택된 40개만 이번 oracle 실행 |
| `VALIDATION_UNSEEN` | 37 | 결과 미검사 |
| `FINAL_HOLDOUT_UNSEEN` | 37 | 결과 미검사 |

09:23 short recording의 유일한 eligible episode는 기존 `V2E08`이므로 pilot role에만 남는다.
이를 validation/holdout으로 재사용하지 않았다.

최종 freeze 값은 다음과 같다.

- seed: `p3_mapping_research_corpus_v1_seed_20260830`
- algorithm: `episode_iterative_stratification_v1`
- source eligible manifest SHA-256:
  `257577c116eb07064e5a3cc367b660a9eee4f5f26460b3582af2afc7205145b0`
- frozen split manifest SHA-256:
  `c57cfe8e57dfca4bb318d30047e8f7215f6994e88a39babfbdbb10ce7637b2f4`
- selected DEVELOPMENT manifest SHA-256:
  `552fd15d91d02e2d450585ec625392a8cb1a50a1f0146b2b65d3562dddf67cb4`

### Exact splitting algorithm

1. 기존 pilot summary의 `(bag, callback_sequence, evaluation_sequence)`와 정확히 일치하는 9개를
   먼저 격리한다. pilot oracle outcome은 split feature로 사용하지 않는다.
2. 나머지 184개에서 production candidate 생성 전에 알 수 있는 다음 category token만 만든다.
   bag, reference snapshot, obstacle side, obstacle count, maximum obstacle-width bin, ego-speed bin,
   local reference-curvature magnitude/sign bin, lifecycle context.
3. 희귀 token 조합부터 처리한다. 각 episode를 한 role에 추가했을 때 role별 목표 비율에 대한
   token별 incremental squared imbalance가 가장 작아지는 role을 고른다.
4. 정확한 capacity `110/37/37`을 강제하며 동률은
   `SHA256(seed | episode_id | role)`로 결정한다.
5. DEVELOPMENT 110개에 동일 알고리즘을 다시 적용해 `ORACLE_SELECTED=40`, `NOT_SELECTED=70`을
   고정한다. callback-level random split은 사용하지 않는다.

한 episode당 earliest eligible evaluator snapshot 하나만 사용하므로 같은 failure episode의 인접
callback이 서로 다른 split에 들어갈 수 없다. 선택된 40개의 bag 분포는 19:00 bag 20개,
19:24 bag 12개, 09:59 bag 8개이며, obstacle-side bin은 RIGHT/MIXED-or-CENTER/LEFT가
20/15/5개다.

`run_development_oracle.py`는 `development_oracle_manifest.csv`만 읽는다. validation 및 final
holdout manifest row나 그 oracle outcome은 실행 입력으로 로드하지 않았다.

## 2. Oracle contract and computation

pilot-v2 domain을 줄이지 않았다.

- existing P3 family와 production entry/exit transition pair만 사용
- STRICT 및 기존 RELAXED target domain
- `d_mid in [-1.5, 1.5] m`
- coarse grid `0.05 m`
- production anchor request 포함
- violation score 상위 30개 주변 `+-0.04 m`, `0.01 m` refinement
- 동일 request 제거와 reference/evaluator log의 exact caching만 적용
- frozen validator, footprint, collision model, vehicle parameters 그대로 사용

| metric | result |
|---|---:|
| DEVELOPMENT episodes searched | 40 |
| conclusive | 40 |
| oracle valid | 23 |
| no valid P3 found in domain | 17 |
| total requests | 853,531 |
| constructed paths | 202,664 |
| validator executions | 202,664 |
| hard-valid grid rows | 11,497 |
| harness search runtime sum | 31.594 s |

모든 40개에서 evaluator lineage, constructed digest multiset, returned digest multiset이 일치했다.
request 수와 constructed/validator 수는 동일 개념이 아니다. domain 밖 또는 construction 불가
request 때문에 853,531개 request 중 202,664개만 실제 path와 validator execution으로 이어졌다.

Frozen source evidence:

- `p3_shadow.cpp`: `cbbcc6e181d0d416b3d3be47b0be5b2cbfd4cf39ddd34cc1ac223989e9257ca5`
- `p3_analytic_solver.hpp`: `e410106266984237f6b1e3286b56aa0093d8ae394a66cc838dbcb2d16c3b8081`
- operational YAML: `0a99dfce38a8552d40bb8903e49ad19bb06dcf7252490a66abc8fa187b4d432d`
- detached oracle harness: `8b23f2b6df54c2e93794ee087f3ebd7a14f9921544af4f97eef755535e045e7e`
- pilot-v2 runner: `ed0d8a54cd8d3e3452c78f336c378f384e60a74f1d641ffd6f05208acdc51198`

## 3. Cause-classification contract

각 oracle-success episode에 다음 우선순위를 적용했다.

1. production과 같은 `d_target/d_mid`가 다른 기존 transition pair에서 valid이면
   `TEMPLATE_OR_TRANSITION_SELECTION`.
2. production과 같은 side/`d_target`/entry/exit, 다른 `d_mid`의 STRICT oracle path가 있고
   exact `z0..z4`가 같으면 analytic probe 진단을 수행한다.
3. oracle `d_mid`가 current branch/root bound를 통과하지 못하면 `ROOT_FILTERING_DOMINANT`.
4. 통과한다면 production `s_probe`에서
   `Delta d_probe = d_oracle(s_probe_prod)-d_probe_prod`를 계산한다.
5. current `d_probe(s)` policy를 유지한 501-point station sweep에서 residual zero가 있으면
   `S_PROBE_DOMINANT`, 없으면 `D_PROBE_DOMINANT`.
6. analytic probe가 없는 zero-interface template에서 같은 target/scales의 다른 `d_mid`가
   필요하면 `TEMPLATE_OR_TRANSITION_SELECTION`.
7. 위 요소를 하나로 고립할 수 없으면 `OTHER`; lineage/parity가 깨지면 `NOT_IDENTIFIABLE`.

13개 template/transition 사례는 다시 두 종류다. 5개는 exact production `d_target/d_mid`가
다른 기존 transition scale에서 valid였고, 8개는 probe가 없는 zero-interface template이
`d_mid=d_target`으로 제한한 반면 같은 target/scales의 다른 `d_mid`가 oracle-valid였다.

## 4. D_PROBE_DOMINANT geometry

| event | stage | side | `s_probe` m | production `d_probe` m | required `d_probe` m | correction m | corridor width m |
|---|---|---|---:|---:|---:|---:|---:|
| DVE004 | M1 NEAR_LONG | LEFT | 0.06536 | +0.55931 | +0.15922 | -0.40010 | 0.82638 |
| DVE005 | M0-V1 | RIGHT | 0.13915 | -0.47115 | -0.45000 | +0.02115 | 0.31885 |
| DVE014 | M0-V1 | RIGHT | 0.16806 | -0.45068 | -0.41000 | +0.04068 | 0.52682 |

분포는 signed `[-0.40010, +0.02115, +0.04068] m`, absolute median `0.04068 m`다.
세 correction 모두 `|d|`를 줄이는, 즉 바깥쪽 anchor에서 reference line 방향으로 이동하는
방향이었다.

- 두 M0-V1 사례는 obstacle-adjacent free-space boundary로부터 정확히 `0.08 m` inset된
  `d_probe`를 사용했고 필요한 inward correction은 각각 2.12 cm, 4.07 cm였다.
- M1 사례는 폭 `0.826 m`인 LEFT branch의 center `+0.559 m`를 사용했으나 oracle path가 같은
  station에서 요구한 값은 `+0.159 m`였다.
- 세 사례 모두 positive-curvature 구간(`0.304..0.693 rad/m`)이며 short obstacle cluster 안의
  probe였다. 그러나 모두 19:24 bag의 같은 reference snapshot domain이고 표본이 3개뿐이므로
  curvature나 corridor width와 correction 크기의 일반적 상관을 주장할 수 없다.
- exact footprint-valid lateral interval은 candidate yaw에 따라 달라 pre-planning 단계에서
  하나의 candidate-independent interval로 계산할 수 없다. 대신 CSV에는 half-width와 wall
  margin을 이미 뺀 corridor center interval을 보존하고, 이 한계를 명시했다.

## 5. Station counterfactual

세 `D_PROBE_DOMINANT` 사례는 current anchor policy를 유지한 station sweep의 최소 residual이
각각 `0.39931`, `0.02115`, `0.04068 m`이고 zero crossing이 없었다. 따라서 이 세 사례에서는
`s_probe`만 바꾸는 것으로 선택된 oracle path를 회복하지 못했다.

반면 DVE039는 현 production station에서 correction `+0.03107 m`가 필요했지만, current
M0 anchor policy가 oracle path와 만나는 station이 cluster 안에 존재했다. 501-point scan의
minimum residual은 `7.62e-5 m`이며 sign crossing도 확인돼 `S_PROBE_DOMINANT`로 분류했다.
이 사례는 09:59 bag/reference domain에서 나온다.

## 6. GO / NO-GO questions

### 1. D_PROBE_DOMINANT가 여러 episode와 bag/reference domain에서 반복되는가?

여러 독립 failure episode에서는 반복된다(3개). 그러나 세 개 모두 19:24 bag과
`ref_3f5d487564f4ee3d`에 속하므로 **multiple bag/reference-domain 반복은 아니다**.

### 2. lateral anchor selection을 primary research target으로 삼을 만큼 빈도가 큰가?

현재 DEVELOPMENT evidence만으로는 **NO-GO**다. 전체 검색의 7.5%, oracle-success의 13.0%이며
단일 domain에 국한된다. 반복 가능한 secondary hypothesis로는 충분하지만 primary claim에는
unseen validation 이전의 domain breadth가 부족하다.

### 3. s_probe selection도 material bottleneck인가?

그렇지만 현재 빈도는 낮다. DVE039 한 건(전체 2.5%, oracle-success 4.3%)에서 기존 lateral
anchor policy를 유지한 station 변경만으로 point constraint를 맞출 수 있었다. 세 d-probe
사례에서는 station-only recovery가 없었다.

### 4. root filtering과 transition-template selection 때문에 broader mapping paper가 필요한가?

transition/template은 중요하다. 23개 oracle-success 중 13개(56.5%)다. 이번 40개 DEVELOPMENT
subset에서는 root-filtering 반복은 없었지만 기존 pilot의 localization-stress V2E08 evidence는
`PILOT_SEEN_DEVELOPMENT_DATA`로 별도 유지된다. 현재 강한 DEVELOPMENT 결과는
`d_probe` 단독보다 **probe + template/transition coverage**를 다루는 연구 범위를 지지한다.

### 5. required d_probe correction과 일관된 pre-planning geometry 변수는 무엇인가?

현재 일관된 것은 correction 방향뿐이다. 세 경우 모두 selected-side outward anchor를
reference line 쪽으로 되돌려야 했다. 두 M0 사례는 0.08 m continuity inset, 한 M1 사례는 wide
corridor midpoint와 연결된다. obstacle-side, curvature sign, cluster-relative station도 같지만
단일 reference domain 때문에 독립적인 연관성으로 해석할 수 없다.

### 6. 다음 DEVELOPMENT 연구 hypothesis

아래는 구현안이 아니라 다음 실험에서 반증해야 할 `[INFERENCE]` hypothesis다.

1. `[INFERENCE]` M0의 고정 0.08 m continuity inset이 일부 short-cluster curve에서 필요한
   `d_mid`보다 과도하게 outward anchor를 만든다.
2. `[INFERENCE]` wide one-sided corridor에서 M1 midpoint anchor보다 ego/target continuity와의
   관계가 `d_mid` feasibility를 더 잘 설명한다.
3. `[INFERENCE]` curvature-critical 한 점 외의 existing corridor station도 함께 평가하면
   DVE039 유형의 station aliasing을 줄일 수 있다.
4. `[INFERENCE]` zero-interface의 `d_mid` 자유도와 기존 entry/exit scale coverage가 probe
   mapping보다 더 큰 개발 bottleneck인지 별도 ablation으로 검증해야 한다.

어떤 hypothesis도 이번 작업에서 fitting, parameter selection, source modification으로
구현하지 않았다.

## 7. Artifacts

- `dataset_split_manifest.csv`: pilot/development/validation/final-holdout 전체 193개 freeze
- `development_oracle_manifest.csv`: outcome-blind로 고정한 DEVELOPMENT 40개
- `development_oracle_summary.csv`: 40개 lineage/parity/oracle 요약
- `mapping_failure_taxonomy.csv`: oracle-success 23개 원인 분류와 근거
- `dprobe_dominant_cases.csv`: D_PROBE 3개 direct diagnosis
- `dprobe_geometry_features.csv`: candidate-generation 이전 geometry descriptor
- `dprobe_required_correction.csv`: fixed-s correction과 station counterfactual 요약
- `station_counterfactual_scan.csv`: analytic 사례의 501-point current-policy scan
- `computation_summary.csv`: event별 및 전체 request/path/validator/runtime
- `plots/`: mechanism frequency, bag outcome, correction, geometry, residual plot
- `raw_oracle/`: frozen input, full valid rows, failure distribution, search domain, parity evidence
- `split_freeze.json`: split/source manifest SHA 및 outcome-blind 계약

## Scope boundary

모든 localization label은 계속 `LOC_UNKNOWN`이다. validation 및 final holdout oracle outcome은
검사하지 않았다. production P3 family, mapping, solver, validator, ranking, footprint, collision
model, lifecycle, vehicle parameter, speed shaping은 수정하지 않았다. 새로운 mapping을 구현하거나
production parameter를 조정하지 않았으며 commit/push도 수행하지 않았다.
