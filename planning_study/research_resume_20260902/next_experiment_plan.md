# 다음 연구 질문과 실험 계획

Validation 37건은 전부 `VALIDATION_SEEN_AFTER_V1`이다. 아래 실험은 이 37건으로 parameter/model을 선택하지 않으며, 금지 분할을 사용하지 않는다.

## 1순위 — frozen S1 live fresh-generation 실행환경 qualification

**단일 최우선 질문:** 같은 frozen S1 query의 25 ms 초과 tail이 CPU core 배치와 ROS/simulator contention으로 재현·설명되는가?

- 가설: frozen method나 candidate set을 바꾸지 않아도 performance-core 격리/명시 배치에서 live fresh-generation callback p95가 `<=25 ms`가 된다.
- failure class: deployment scheduling / execution-environment tail latency.
- 기대 개선: live evaluator p95 `27.802 ms`, callback p95 `30.904 ms`에서 callback p95 `<=25 ms`; method digest 및 `B128/K12/12`는 불변.
- 최소 실험: 동일 build·frozen SHA·instrumentation-off에서 planner만 (a) unpinned, (b) performance core pinned로 A/B한다. workload는 node-isolated, ROS-only, simulator+ROS 3개를 사전 고정한다. 각 condition에서 warm-up 뒤 fresh-generation callback `>=200`개, 독립 repeat `5`회; stage timing, CPU/core migration, budgets, digest, failure/lifecycle/publication을 기록한다.
- 성공 기준: 모든 predeclared workload의 performance-core condition에서 fresh full-callback p95 `<=25 ms`; p99와 max는 별도 공개; `max_generated<=128`, `max_validated<=12`, selected<=12, digest/parity 불변, collision/lifecycle failure 0.
- 실패 해석: performance-core에서도 p95가 넘으면 환경만으로 설명되지 않으며 그때 exact behavior-preserving callback optimization을 별도 설계한다. 이 단계에서는 알고리즘을 변경하지 않는다.
- validation overfit 위험: 낮음. synthetic/deterministic runtime workload만 사용한다.
- 이미-seen validation 소비: 없음.

근거: `planning_study/gqsc_s1_live_realtime_v1/README.md:50-104,128-150` (commit `ca621fd`); `planning_study/gqsc_s1_closed_loop_smoke_v1/README.md:88-101`.

## 2순위 — VUE035형 selector-coverage의 synthetic mechanism test

- 가설: 현 frozen selector의 유일한 Oracle-v2-usable H4-A 잔여 miss는 family expressiveness/guard가 아니라 multi-parameter/factor-space coverage 부족이다.
- failure class: multi-parameter/factor-space coverage under bounded selector budget.
- 기대 개선: held-out synthetic geometry cell에서 Oracle-v2 hard+usable witness가 존재할 때 frozen S1 recovery gap을 재현하고, 변경 전 원인을 factor provenance/rank/quota로 국소화한다.
- 최소 실험: development에서 이미 정의한 corridor/curvature/entry-distance 범위로 synthetic grid를 사전 선언한다. VUE035 수치를 fitting target으로 쓰지 않는다. exact Oracle-v2를 진단 labeler로만 사용하고 frozen `B128/K12`의 generated/validated/rank/provenance를 비교한다.
- 성공 기준: 독립 synthetic cell 여러 개에서 동일한 coverage mechanism이 재현되고, hard validator/guard가 아닌 generation/ranking stage에서 witness가 소실됨을 확인한다. 재현 전에는 selector를 변경하지 않는다.
- validation overfit 위험: 중간. VUE035는 확인용으로만 남기고 설계/선택은 development+synthetic에서 끝내야 한다.
- 이미-seen validation 소비: 새 선택에는 없음; 최종 고정 후 1회 확인만 가능하며 결과는 seen confirmation으로 표기한다.

근거: `planning_study/p3_reference_oracle_v2/remaining_failure_taxonomy.csv:19`; `planning_study/gqsc_s1_closed_loop_candidate_v1/frozen_parity.csv:85`.

## 3순위 — V2E09형 old-search-gap synthetic coverage audit

- 가설: Oracle v2가 교정한 V2E09형 tuple은 현재 frozen S1의 probe/factor basis가 체계적으로 낮은 coverage를 갖는 별도 class다.
- failure class: selector search coverage; Oracle label 문제는 이미 해결됨.
- 기대 개선: 사례 자체를 튜닝하지 않고도 development/synthetic에서 같은 side/transition coupling의 miss를 재현하고, VUE035 class와 공통 selector 결함인지 분리한다.
- 최소 실험: V2E09 best tuple을 직접 후보에 삽입하지 않는다. development-derived synthetic parameter sweep에서 Oracle-v2 witness existence와 frozen S1 generation/selection을 교차표로 만든다.
- 성공 기준: 최소 3개의 독립 synthetic events에서 동일 mechanism을 재현하거나, 재현 실패 시 V2E09를 single-case unresolved로 유지한다.
- validation overfit 위험: 낮음(해당 case는 pilot seen). 사례별 패치 위험은 높으므로 synthetic 재현이 필수다.
- 이미-seen validation 소비: 없음.

근거: `planning_study/p3_reference_oracle_v2/old_vs_v2_oracle_labels.csv:10`; `planning_study/gqsc_s1_closed_loop_candidate_v1/frozen_parity.csv:10`.

Oracle v2 구현 변경, P3 family 변경, weight/threshold/grid 변경, CMA-ES, label 변경은 어느 실험에도 포함하지 않는다.
