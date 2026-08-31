# GQSC v3 predeclared candidate policies

## Pre-evaluation declaration

`V3_POLICY_SET_PREDECLARED_BEFORE_DEVELOPMENT_OUTCOME_COMPARISON`

이 문서는 아래 v3 후보들의 hard/usable 결과를 실행하기 전에 작성되었다. 후보 정의에는
VALIDATION_SEEN, success-control outcome, FINAL_HOLDOUT 정보가 들어가지 않는다. 앞선
DEVELOPMENT-only deficiency phase는 clean standalone bank에서 모든 teacher hard/usable lateral이
빠진 사건이 DVE039 하나뿐임을 보였다. 다른 DEVELOPMENT missing lateral은 서로 다른 normalized
영역에 있었고 해당 사건에는 다른 clean-bank hard/usable lateral이 이미 존재했다.

따라서 v3는 새 lateral operator를 추가하지 않는다. 특히
`COMPONENT_HALF_FAR002_INWARD015`와 그 전용 transition priority는 모든 v3 후보에서
비활성이다. `MIN_OUTWARD`도 DEVELOPMENT에서 MIN과 MAX를 구분한 hard 효과가 DVE023 한
건뿐이므로 후보에서 제외한다.

모든 후보가 공유하는 계약은 다음과 같다.

- direct lateral bank: clean standalone v1 그대로
- v2 event-specific component replacement: OFF
- pair proxy budget: B128
- P3 reconstruction: K12 이하
- exact validator: 12회 이하
- lexicographic quota: 10
- coverage quota: 2
- exact preconstruction-shape deduplication
- production path family, reconstruction, validator, final rank: unchanged

## P0 — `V3_CLEAN_V1_BASELINE`

비교 기준이다. clean v1의 original coverage stream을 그대로 사용한다. coverage shape가
lexicographic Top-10과 중복될 수 있다. 새 상수나 새 rule이 없다.

## P1 — `V3_DISJOINT_PROXY_COVERAGE`

lexicographic Top-10의 exact preconstruction shapes를 coverage eligibility에서 제외한다. 남은
candidate 중 기존 dimensionless/proxy coverage order로 2개를 고른다. 두 slot을 새 shape에
사용한다는 일반적인 computational diversity 원리이며 lateral/transition 특정 class를 예약하지
않는다.

## P2 — `V3_SIDE_BALANCED_DISJOINT`

양쪽 side가 모두 존재하면 RIGHT와 LEFT에서 각각 1개의 best disjoint coverage shape를 고른다.
한 side에 eligible candidate가 없으면 남은 slot은 P1의 general disjoint coverage로 채운다.

```text
for side in [RIGHT, LEFT]:
    select best eligible non-lex exact shape from that side, quota 1
fill remaining quota with general disjoint coverage
```

RIGHT-first는 기존 deterministic side ordering과 일치하는 tie convention일 뿐, RIGHT에 더 많은
quota를 주지 않는다. 양쪽이 있으면 결과는 1+1이다.

## P3 — `V3_NORMALIZED_MAXIMIN_DISJOINT`

각 candidate를 side-domain과 transition geometry에 정규화한다. domain의 zero-near endpoint를
`n`, 반대 endpoint를 `f`라 하고

\[
u_t=\frac{d_{target}-n}{f-n},\qquad
u_m=\frac{d_{mid}-n}{f-n}.
\]

`e`, `x`는 이미 geometry minimum/maximum으로 정규화된 entry/exit coordinate이고,
`sigma=0` for RIGHT, `1` for LEFT다. 두 candidate의 거리는

\[
D^2=(\Delta\sigma)^2+(\Delta u_t)^2+(\Delta u_m)^2+
    (\Delta e)^2+(\Delta x)^2.
\]

construction guard를 통과하고 lexicographic shape와 exact-disjoint인 candidate 중, 이미 선택된
lexicographic+coverage set까지의 최소 `D^2`가 최대인 candidate를 greedy하게 2개 고른다.
동률은 기존 lexicographic proxy/stable tie를 쓴다. 모든 coordinate weight는 1이며 outcome으로
fit하지 않는다.

## DEVELOPMENT 선택 순서

하나의 v3 후보는 다음 predeclared lexicographic preference로 선택한다.

1. event/map-specific parameter가 없을 것
2. B128/K12/validator-12 bound와 deterministic exact dedup을 지킬 것
3. DEVELOPMENT에서 clean P0 대비 usable recovery를 잃지 않을 것
4. DEVELOPMENT usable recovery가 클 것
5. DEVELOPMENT hard recovery가 클 것
6. candidate diversity가 높을 것
7. 동일하면 method complexity가 낮은 순서 `P0 < P1 < P2 < P3`

Success controls는 이 선택에 사용하지 않는다. DEVELOPMENT에서 하나를 선택하고 JSON/SHA로
lock한 뒤에만 regression gate로 한 번 평가한다. 그 결과가 나쁘더라도 이 task에서 policy를
수정하거나 다시 선택하지 않는다.
