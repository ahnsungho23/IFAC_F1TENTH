# Research framing comparison

## 해석 범위

이 비교는 `PILOT_SEEN_DEVELOPMENT_DATA`와 `DVE001..DVE040 DEVELOPMENT`만 사용한다. 40개 실패 중 23개에 기존 oracle domain 안의 strict hard-valid P3가 있었고, 그 23개의 primary mechanism은 `D_PROBE_DOMINANT=3`, `S_PROBE_DOMINANT=1`, `TEMPLATE_OR_TRANSITION_SELECTION=13`, `OTHER=6`이다. unseen validation/holdout은 읽지 않았다.

“예상 novelty”는 repository 내부의 잠재적 기여를 뜻한다. 이번 연구는 외부 문헌 검토가 아니므로 선행연구 대비 학술적 신규성을 확정하지 않는다.

## H1 — `d_probe` selection improvement

### 지지하는 DEVELOPMENT 증거

- 23개 oracle-valid 실패 중 3개가 direct diagnosis에서 `D_PROBE_DOMINANT`이었다.
- corridor anchor가 analytic root와 `d_mid`를 바꾸므로 물리적·수학적 인과 경로가 명확하다.

### 반대 또는 제한 증거

- 3/23만 primary `D_PROBE_DOMINANT`이고, 13/23은 template/transition coupling이었다.
- 본 factorization의 A+B는 probe를 바꾸지 않고도 13개 coupling 사례를 회복했다.
- full L×T 회복은 oracle lateral value를 포함하므로 `d_probe`만 고치면 된다는 증거가 아니다.

### 예상 기여와 budget

- 기여 후보: corridor geometry와 analytic equation을 이용한 anchor 선택 개선.
- budget 목표: 현재 cap 24 안에서 소수 probe anchor/root를 추가하는 형태가 바람직하지만, 필요한 anchor 수는 이 연구로 정해지지 않았다.

### 주요 미해결점

- 3개가 반복 가능한 geometry rule로 묶이는지, `s_probe` 또는 root branch 변화와 독립적으로 `d_probe` 효과만 분리할 수 있는지 불명확하다.

## H2 — transition/template selection improvement

### 지지하는 DEVELOPMENT 증거

- primary `TEMPLATE_OR_TRANSITION_SELECTION`이 13/23이다.
- production lateral tuple에 같은 event의 production transition을 cross-pair한 B가 5/23을 회복했다. 이 중 4개는 primary `SELECTION_FAILURE_WITH_EXISTING_OPTION`, 한 개 `DVE029`는 A/B 중첩이다.
- correct lateral factor 조건의 hindsight set cover에서 1/2/3/4 transition pair가 16/21/22/23개를 덮었다.

### 반대 또는 제한 증거

- B만으로 회복한 것은 5/23이며, 나머지 8개 template 사례는 주로 M1 zero의 `d_mid=d_target` coupling을 풀어야 했다.
- naive B는 p95 526.35, max 667 validator 호출이라 production 적용이 불가능하다.
- 2-pair 21/23은 correct lateral factor를 이미 안다는 조건부 oracle upper bound다.

### 예상 기여와 budget

- 기여 후보: 기존 transition option의 template 간 재사용 및 bounded selection.
- 동일 lateral당 2–4개 transition이 DEVELOPMENT upper bound상 유망하지만, 전체 candidate budget은 production cap 24를 설계 목표로 삼아 다시 제한해야 한다. 24가 충분하다는 증거는 아직 없다.

### 주요 미해결점

- correct lateral option 없이 transition만 골라서는 coverage가 제한된다. event별 transition을 geometry로 고르는 온라인 규칙도 없다.

## H3 — factorized P3 candidate generation

### 지지하는 DEVELOPMENT 증거

- A counterfactual 9개와 B 5개의 합집합이 13개 template/transition 사례를 정확히 회복했다.
- 이는 현재 full-tuple template coupling이 기존 P3 family의 feasible 조합을 구조적으로 누락한다는 직접적인 mechanism evidence다.
- full DEVELOPMENT L×T upper bound는 oracle-valid 23/23을 동일 spline family와 validator로 재현했다.

### 반대 또는 제한 증거

- A counterfactual과 full L×T는 oracle lateral factor를 사용한다. 온라인 lateral generator가 아니다.
- production factor만의 A는 1/23, A+B current는 7/23에 그쳤다.
- full L×T는 총 21,978, p99 5,375.34, max 7,440 후보를 만들어 실시간 후보 생성기로 사용할 수 없다.
- analytic root에서 가져온 `d_mid`를 다른 transition과 결합하면 새 transition의 probe equation root라는 의미는 사라진다. 유효성은 direct validator가 다시 보장해야 한다.

### 예상 기여와 budget

- 기여 후보: lateral-shape proposal과 longitudinal transition proposal의 책임 분리, bounded recombination, 중복 제거.
- candidate/validator budget은 명시적으로 bounded해야 하며 공정한 초기 설계 목표는 current total cap 24 보존이다. DEVELOPMENT가 24에서의 최종 coverage를 아직 증명하지 않는다.

### 주요 미해결점

- oracle 없이 작은 lateral pool을 어떻게 생성할지, 어떤 cross-pair를 cap 안에 넣을지가 해결되지 않았다.

## H4 — geometry-conditioned bounded factorized P3 candidate generation

### 지지하는 DEVELOPMENT 증거

- H3의 13개 회복은 factorization의 필요성을, naive p95 수백 개는 bounded selection의 필요성을 동시에 보인다.
- B-only 4개는 모두 curvature sign change였고, obstacle start 거리와 obstacle span도 좁은 범위에 모였다. transition 선택에 물리적으로 해석 가능한 pre-planning signal이 있을 가능성이 있다.
- transition upper-bound basis는 2–4개로 작아, geometry selector가 correct subset을 골라낼 경우 계산량을 줄일 여지가 있다.

### 반대 또는 제한 증거

- B-only 표본은 4개뿐이고 같은 bag/input domain 상관을 배제하지 못했다.
- A-only 8개의 curvature/corridor 조건은 이질적이며, 작은 global `d_mid` delta 4개도 그중 일부만 덮었다.
- full-only 10개에는 probe-dominant, other, s-probe 사례가 섞여 있어 하나의 selector로 해결된다는 증거가 없다.
- 현재 결과는 geometry feature와 outcome의 연관이지 frozen threshold나 unseen 일반화 증거가 아니다.

### 예상 기여와 budget

- 잠재 기여: corridor/obstacle/reference geometry로 `L_K(x)`와 `T_J(x)`를 선택하는 해석 가능한 bounded set-valued policy.
- 설계 목표는 baseline 포함, exact/path dedup, validator calls `<=B`, production total cap 24와 비교 가능한 p99 budget이다. `K,J,B`의 실제 값은 아직 미정이다.

### 주요 미해결점

- outcome leakage 없이 lateral option을 만드는 방법, geometry feature의 domain 일반화, fixed budget 아래 coverage/runtime trade-off, 그리고 unseen에서의 최초 검증이 남아 있다.

## 비교표

| framing | 직접 지지 범위 | 핵심 반증/제한 | 예상 budget | 가장 큰 미해결점 |
|---|---:|---|---|---|
| H1 `d_probe` | 3/23 primary | 13개 coupling 사례를 설명 못함 | 소수 anchor, 정확한 수 미정 | `s_probe`/branch와 독립 효과 |
| H2 transition | B 5/23; 조건부 2 pair 21/23 | correct lateral을 가정, naive B 폭증 | lateral당 2–4 upper-bound pair | 온라인 pair 선택 |
| H3 factorized | A+B 13/23 mechanism recovery | oracle lateral 의존, full max 7,440 | explicit cap 필요; 목표 24 | bounded lateral generation |
| H4 geometry-conditioned bounded | H3 회복 + 폭증 회피 필요; 제한적 geometry pattern | 표본 작고 threshold/generalization 미검증 | `K×J`를 포함해 total `B<=24`를 초기 목표 | leakage-free selector와 unseen 검증 |

## 추천

Primary framing은 **H4: geometry-conditioned bounded factorized P3 candidate generation**으로 한다.

정확한 다음 문제는 scalar parameter regression이 아니라 다음 constrained set selection이다.

\[
\pi:x\mapsto (L_K(x),T_J(x)),
\]

\[
Q(x)=Q_{production}(x)\cup(L_K(x)\times T_J(x)),
\quad |Q(x)|\le B,
\]

여기서 `x`는 planning 전에 관측 가능한 ego/obstacle/corridor/reference geometry이고, objective는 DEVELOPMENT episode hard-valid coverage를 높이면서 validator-call p99와 duplicate를 제한하는 것이다. production baseline 후보는 항상 포함하고, 기존 P3 constructor/guard/validator가 최종 권위를 유지한다. oracle outcome은 DEVELOPMENT에서 policy 설계·freeze에만 쓰고 online input으로 쓰면 안 된다.

Fallback framing은 **H3: bounded factorized P3 candidate generation**이다. geometry separation이 재현되지 않으면, 먼저 factor ownership과 bounded enumeration 자체를 기여 대상으로 삼고 deterministic coverage/budget을 연구할 수 있다.

다음 단계 전에 반드시 DEVELOPMENT에서 feature 정의, lateral proposal rule, transition subset rule, `K/J/B`, tie-break, dedup, fallback을 동결해야 한다. 그 뒤에만 `VALIDATION_UNSEEN`을 처음 열어야 하며, holdout은 마지막까지 유지해야 한다.

