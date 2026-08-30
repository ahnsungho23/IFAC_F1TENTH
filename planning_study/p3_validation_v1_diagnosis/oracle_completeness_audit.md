# Frozen oracle completeness audit

## 판정

VUE036의 miss class는 `ORACLE_REFINEMENT_MISS`다. Frozen oracle은 feasible P3를 발견하는
deterministic bounded search이지만, `NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN`을 완전한 infeasibility
certificate로 해석할 수 없다.

## VUE036 배제 진단

| 후보 원인 | 판정 | 근거 |
|---|---|---|
| `ORACLE_DOMAIN_GAP` | 배제 | valid `d_target`, `d_mid`, side가 strict domain 안에 있음 |
| `ORACLE_RESOLUTION_GAP` | 직접 원인 아님 | 같은 0.01 m lattice를 주변에 적용해 64점 중 8점 hard-valid |
| `ORACLE_REFINEMENT_MISS` | **확정** | 주변 coarse 셀 최상 rank 63; top-30 seed에 없고 refinement coverage 0 |
| `FACTOR_SPACE_MISMATCH` | 배제 | valid entry/exit가 enumerated production pair |
| `FILTERING/DEDUP_MISS` | 배제 | valid digest가 oracle output에 생성된 뒤 제거된 흔적이 아니라 요청 자체가 없음 |
| `IMPLEMENTATION_MISMATCH` | 배제 | 같은 frozen harness가 exact tuple을 29-point hard-valid path로 재구성 |
| construction guard | 배제 | exact witness construction 성공, guard reject 없음 |

Coarse grid의 witness 주변 8개 strict/relaxed 셀은 violation rank
`63|64|75|76|160|161|172|173`이었다. Rank 30 경계와 이 셀들의 score는 모두
`(violation_count=2, maximum_normalized_violation=21.638047477...)`로 같았다. Stable ordering에서
앞선 request가 top-30을 채워, 유효 pocket 주변 셀이 seed가 되지 않았다. 기존 refinement는 각
seed의 `d_target,d_mid`에 ±0.04 m, 0.01 m grid를 놓으므로 seed 탈락 후에는 해당 pocket을 방문할
경로가 없다.

## Exact witness lineage

- event: VUE036
- path digest: `95a7394f79189d58`
- `d_target=-0.6725895769471937`
- `d_mid=-0.7246579807893281`
- entry/exit: `0.5145810930150512 / 0.4971684162574945`
- selected ranks: H3 23, H4-A 13, H4-B 5
- source: component quarter target + target/bottleneck-center half mid
- transition source: production M1 `ZERO_BOUNDARY_SHORT` pair
- template/probe/root: direct factorized P3 request; analytic-root lineage 없음
- exact validator: hard-valid

## 기존 oracle-negative 결과 계약

이 진단은 기존 CSV의 classification을 변경하지 않았다.

1. Validation oracle-negative 13개 중 complete existing factor pool에서 추가 hard-valid가 나온 것은
   VUE036 하나다.
2. 나머지 12개는 **그 factor pool에서는** 추가 witness가 없었다. 이는 continuous P3 domain의
   infeasibility 증명이 아니다.
3. DEVELOPMENT의 기존 oracle-negative 사례에서도 frozen H3/H4-A/H4-B selected budget이 추가
   witness를 제공한 기록은 없지만, 같은 search topology를 썼다면 VUE036형 false negative를
   배제할 수 없다.
4. 따라서 과거 label은 historical result로 보존하고, 향후 분석에서는
   `NO_VALID_FOUND_BY_FROZEN_SAMPLED_ORACLE` 의미로 제한해서 읽어야 한다. 공식 relabel은 oracle
   repair 후 별도 version에서 수행해야 한다.

## Upper-bound 사용 가능성

현재 oracle 성공 수는 search가 실제 찾은 feasible 사건 수의 lower bound다. 반대로 oracle-negative
수를 family infeasibility의 upper bound처럼 사용하는 것은 허용되지 않는다. Upper-bound 주장에는
최소한 다음 중 하나가 필요하다.

- domain 전체에 대한 completeness certificate,
- adaptive subdivision/interval proof로 미탐색 pocket을 통제한 search,
- 독립 search의 교차 검증과 convergence evidence.

이는 다음 단계의 요구사항이지 이번 진단에서 oracle algorithm을 변경했다는 뜻이 아니다.

## 재현 및 integrity

`run_diagnosis.py`는 frozen exact-validator harness SHA를 먼저 확인하고 validation input 37개만
사용한다. Complete factor-pool audit은 기존 `build_pool()`과 H4-A score를 그대로 호출하되 K를
자르기 전 순위를 관찰한다. 원 frozen split, oracle outputs, H4-A spec/implementation은 수정하지
않는다. `FINAL_HOLDOUT_UNSEEN`은 읽지 않는다.
