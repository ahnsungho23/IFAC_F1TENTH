# GQSC v2 pre-freeze parameter provenance audit

## 판정

`GQSC_V2_HAS_MAP_OR_EVENT_SPECIFIC_PARAMETERIZATION`

이 판정은 구현이 event ID를 읽는다는 뜻이 아니다. 온라인 식은 event ID나 validator 결과를
읽지 않는 일반적인 side-relative geometry 연산이다. 그러나 그 식에 들어간 절대 길이
`0.02 m`, `0.15 m`의 **결합**과 그 연산자의 transition 우선순위는 DEVELOPMENT의 명명된
사례 DVE039를 재구성하도록 도입되었고, `MIN_OUTWARD` 선택도 DEVELOPMENT의 DVE023을
회복한 ablation 결과로 결정되었다. 따라서 "event literal이 source에 없음"과 "parameter
provenance가 event-independent임"은 같은 명제가 아니다.

이번 감사에서는 method를 변경하거나 새 tuning/ablation을 실행하지 않았다.
`FINAL_HOLDOUT_UNSEEN`은 열람하지 않았다. 근거는 현재 research-only source, 이전에 생성된
standalone v1/v2 artifact, frozen R3 factor-space 문서, 현재 parameter뿐이다.

## 분류 규약

| 번호 | 분류 | 이 문서의 판정 기준 |
|---:|---|---|
| 1 | `PHYSICAL` | 차량 치수, 조향 한계, 안전 요구량으로부터 source에 명시된 식으로 유도됨 |
| 2 | `DIMENSIONLESS_GEOMETRIC` | corridor/component 길이에 정규화되어 geometry와 함께 자동 scaling됨 |
| 3 | `COMPUTATIONAL` | 후보 수, budget, index, 수치 비교 tolerance처럼 계산량/결정성만 규정함 |
| 4 | `DEVELOPMENT_TUNED_GENERAL` | DEVELOPMENT ablation으로 고른 일반 hyperparameter이며 특정 event/map 하나의 좌표를 재구성하는 것이 목적은 아님 |
| 5 | `EVENT_OR_MAP_SPECIFIC` | 명명된 event 또는 특정 map geometry의 실패를 고치도록 값이나 조합을 선택함 |

숫자 원자가 이전 bank에 존재한 경우에도 **v2에서 그 숫자가 사용된 방식**을 주 분류로 삼았다.
원자 자체의 과거 provenance는 별도 열에 보존한다. 이 구분이 없으면 기존 general grid에 있던
숫자를 특정 event의 좌표와 결합한 사실이 가려진다.

## 현재 선택된 v2 규칙

첫 connected component를 `C_0=[a,b]`라 하고, 두 끝 중 `(abs(value), value)`가 작은 끝을
`near`, 다른 끝을 `far`라 하자. 그러면 현재 replacement operator는

\[
q=\operatorname{sign}(far-near),\qquad
c=near+\frac{1}{2}(far-near),
\]

\[
d_{target}=\operatorname{clip}_{D}(c+0.02q),
\]

\[
d_{mid}=\operatorname{clip}_{[-1.5,1.5]}(d_{target}-0.15q).
\]

즉 `0.02 m`는 component midpoint에서 **far 방향**, `0.15 m`는 그 target에서
**inward/near 방향**이다. 기존 `COMPONENT_HALF_MID_MINUS_015` 한 자리를 교체하므로 lateral
bound를 늘리지 않는다. Source authority는
[`p3_r3_k12.cpp`](../../src/local_planning/src/p3_r3_k12.cpp)의
`directSideLaterals()`이다.

`ZERO_INTERFACE_EQUAL_MIN_OUTWARD`에서는 side domain `D=[l,u]`의 zero-near end를 `n`,
반대 end를 `f`라 하고

\[
r=\frac{d_{target}-n}{f-n},\qquad
\Delta_{out}=\operatorname{sign}(f-n)(d_{mid}-d_{target})
\]

로 둔다. lexicographic Top-10과 exact-shape 중복이 없는 pool에서 다음을 순서대로 뽑는다.

```text
zero_interface(row) :=
    transition == SHORT_ENTRY_SHORT_EXIT
    and -epsilon <= r <= 1/32 + epsilon

reserve[0] := coverage order에서 가장 좋은
              zero_interface and |d_mid-d_target| <= epsilon 후보 1개

reserve[1] := 아직 선택되지 않은 zero_interface 후보 중
              Delta_out > epsilon인 후보의 최소 Delta_out 1개
              (동률은 기존 lexicographic proxy tie)

빈 slot := 기존 general coverage order로 채움
```

따라서 `MIN_OUTWARD`는 metric threshold가 아니라 positive outward 후보들의 **순서 통계량**이다.
반대로 zero-interface band의 `1/32`는 side-domain 폭에 정규화된 threshold다.

## v2에서 새로 선택된 상수와 규칙

### Lateral replacement operator

| 상수/규칙 | 주 분류 | 정확한 origin과 식 | geometry scaling | 선택 dataset | map scale 변경 | map-independent 방어 가능성 |
|---|---|---|---|---|---|---|
| component midpoint `1/2` | 2 `DIMENSIONLESS_GEOMETRIC` | `c=near+0.5(far-near)`. 기존 component-fraction bank에도 있던 midpoint | component 폭과 선형 scaling | v2 이전 R3/standalone v1 geometry basis | 불필요 | 가능. 단위 없는 affine fraction |
| far-direction `0.02 m` | **5 `EVENT_OR_MAP_SPECIFIC`** | 이전 R3 target-offset bank의 `0.02 m` 원자를 DVE039 component center에 결합: `d_target=clip_D(c+0.02q)` | **안 됨**. component 폭과 무관한 절대 meter | 결합은 DEVELOPMENT DVE039 분석에서 선택. 원자는 이전 seen-designed general bank에 존재 | scale-similarity를 보존하려면 재표현/retuning 필요 | 현재 근거로 불가. 같은 F1TENTH metric scale에서만 경험적 hyperparameter로 방어 가능 |
| inward-direction `0.15 m` | **5 `EVENT_OR_MAP_SPECIFIC`** | 이전 R3 mid-offset bank의 `0.15 m` 원자를 위 target과 결합: `d_mid=clip(d_target-0.15q,-1.5,1.5)` | **안 됨**. component 폭과 무관한 절대 meter | 결합은 DEVELOPMENT DVE039 분석에서 선택. 원자는 이전 seen-designed general bank에 존재 | scale-similarity를 보존하려면 재표현/retuning 필요 | 현재 근거로 불가. 차량 반폭과의 수치 일치는 유도 근거가 아님 |
| direction magnitude `1`과 zero-near 기준 `0` | 2 `DIMENSIONLESS_GEOMETRIC` | `q=copysign(1,far-near)`; `near=argmin(abs(endpoint),endpoint)` | 부호 convention이므로 scale invariant | geometry convention, dataset tuning 아님 | 불필요 | 가능 |
| `d_mid` clamp `[-1.5,1.5] m` | 4 `DEVELOPMENT_TUNED_GENERAL` (상속) | frozen R3 factor-space의 target-relative mid bound `kTargetBoundM`; v2 신설값은 아님 | 안 됨 | pre-existing seen-designed R3 factor space | 큰 scale/차종 변경 시 재검증·retuning 필요 | 물리 유도식이 없어 보편적 map-independent 값으로는 불가 |
| 기존 operator **1개 교체** | 3 `COMPUTATIONAL` | `COMPONENT_HALF_MID_MINUS_015`를 찾아 in-place replacement; lateral 수 증가 없음 | 해당 없음 | DEVELOPMENT contribution ablation: 교체 대상은 7회 construct, hard/usable 각 3개였지만 unique recovery 0 | 불필요 | 계산 bound 보존 규칙으로 가능 |
| replacement 전용 transition ordering | **5 `EVENT_OR_MAP_SPECIFIC`** | `LONG/MEDIUM`, `LONG/SHORT`, `SHORT/SHORT`, `MEDIUM/SHORT`, `LONG/LONG` 순. B128의 초기 wave에 DVE039용 operator를 노출하도록 도입 | transition 값 자체는 geometry scaling하지만 이 우선순위는 scaling 법칙이 아님 | DEVELOPMENT DVE039; 같은 operator가 V2E05도 회복한 것은 사후 generality evidence | geometry 분포가 바뀌면 재검증 필요 | event ID 없이 실행되지만 선택 provenance 때문에 현 상태로는 불충분 |

중요하게, DVE039의 component 폭은 `0.3856462017192975 m`이다. 따라서 해당 event에서
`0.02/W=0.0518610060`, `0.15/W=0.3889575454`이며 기존의 정확한 `1/32`, `1/2` 같은
component fraction이 아니다. 식은 side-relative이지만 두 offset의 크기는 component-relative가
아니다.

### `ZERO_INTERFACE_EQUAL_MIN_OUTWARD`

| 상수/규칙 | 주 분류 | 정확한 origin과 식 | geometry scaling | 선택 dataset | map scale 변경 | map-independent 방어 가능성 |
|---|---|---|---|---|---|---|
| near-band lower `0` | 2 `DIMENSIONLESS_GEOMETRIC` | normalized fraction `r>=-epsilon`; zero-near domain interface 자체 | 자동 scaling | geometry convention | 불필요 | 가능 |
| near-band upper `1/32` | 2 `DIMENSIONLESS_GEOMETRIC` | `r<=(1/32)+epsilon`; 이전 component fraction 및 zero-interface inset basis에서 상속 | side-domain 폭의 `3.125%`로 자동 scaling | 숫자는 pre-existing geometry basis. 이 band를 reserve 대상으로 고른 것은 DVE023/VUE009/VUE014/SCE008 구조 진단 | 불필요 | 숫자 자체는 가능. 다만 reserve 대상으로의 선택은 seen-event informed |
| `SHORT_ENTRY_SHORT_EXIT`, 즉 `transition_index==0` | **5 `EVENT_OR_MAP_SPECIFIC`** (규칙) | displacement된 8개 teacher-hard candidate 전부가 short/short였다는 진단에 따라 filter | transition station 값은 geometry scaling, categorical filter는 그대로 | DVE023 + 이미 노출된 VUE009/VUE014/SCE008 구조 진단 | 값 retuning은 없으나 새 geometry 분포에서 재검증 필요 | 현재 evidence는 네 명명 event에 묶여 있어 보편성 주장은 불가 |
| equality tolerance `epsilon=1e-9` | 3 `COMPUTATIONAL` | global `kEpsilon`; `|d_mid-d_target|<=epsilon`의 수치 비교 tolerance | binary64 equality 보조이며 geometry scaling 목적 아님 | 기존 계산 convention, dataset tuning 아님 | 일반적 meter-scale map에서 불필요 | 가능 |
| equal-lateral class에 reserve를 주는 규칙 | **5 `EVENT_OR_MAP_SPECIFIC`** | `|d_mid-d_target|<=epsilon` class를 첫 reserve 대상으로 지정 | class 자체는 scale invariant | DVE023/VUE009/VUE014/SCE008 구조 진단 | 값 retuning은 없으나 새 분포에서 재검증 필요 | 현재 evidence로는 불충분 |
| outward positivity `Delta_out>epsilon`의 부호 규칙 | 2 `DIMENSIONLESS_GEOMETRIC` | side-relative outward 방향만 허용; tolerance는 위 computational row | 부호와 ordering은 positive scale에 invariant | structural geometry rule; 새 magnitude threshold 없음 | 불필요 | 식 자체는 가능 |
| **minimum** positive outward | **5 `EVENT_OR_MAP_SPECIFIC`** | `MIN`과 `MAX`를 DEVELOPMENT에서 비교. MIN만 DVE023 hard를 회복(`25/49`), MAX는 baseline과 같은 `24/49` | ordinal이므로 scale invariant | DEVELOPMENT_49, 실질적 구분 evidence는 DVE023 한 건 | 수치 retuning은 없으나 분포 변경 시 rule 재검증 필요 | 일반 형식은 깔끔하지만 선택 evidence가 한 명명 event이므로 현재는 불가 |
| equal `1` slot + outward `1` slot이라는 수 | 3 `COMPUTATIONAL` | frozen coverage quota 2를 `1+1`로 분할 | 해당 없음 | K12/coverage-2 계산 envelope | 불필요 | 계산 bound로 가능 |
| 두 slot을 equal/outward에 각각 배정하는 semantic rule | **5 `EVENT_OR_MAP_SPECIFIC`** | 위 `1+1`의 의미를 두 seen-derived shape class로 고정 | 해당 없음 | DEVELOPMENT ablation; structural hypothesis에는 VUE/SCE exposure 있음 | 새 geometry 분포에서 재검증 필요 | 현재 evidence로는 불충분 |
| 빈 reserve fallback | 3 `COMPUTATIONAL` | 필요한 class가 없으면 남은 `2-selected` slot을 기존 general coverage로 채움 | 해당 없음 | defensive bounded implementation | 불필요 | 가능 |

`1/32`와 outward sign/minimum에는 절대 meter threshold가 없다. 따라서 reserve policy의
**수학적 형태**는 scale-aware다. 하지만 `SHORT_SHORT + [0,1/32] + equal/min-outward`라는
class 조합의 선택 provenance는 named seen events에 의존한다. 이 감사의 최종 category 5 판정은
바로 이 차이를 반영한다.

## 상속된 계산/기하 상수

다음 값은 standalone v2가 새로 tuning한 값은 아니지만, 선택된 v2의 실제 bound와 생성 공간을
정의하므로 누락하지 않는다.

| 값 | 분류 | origin/역할 | scaling 및 retuning |
|---|---|---|---|
| `B=128` | 3 `COMPUTATIONAL` | v1부터의 pair-proxy budget; v2 DEVELOPMENT ablation도 모두 B128 | map scale과 무관. geometry 복잡도 분포가 달라지면 coverage는 재평가해야 하나 길이 scaling parameter는 아님 |
| allowed `B={64,96,128}` | 3 `COMPUTATIONAL` | native selector API의 허용 budget menu | map scale과 무관 |
| lateral retain `{36,48,64}` 및 hard max `64` | 3 `COMPUTATIONAL` | B64/B96/B128별 retain cap; side당 bank cap 32, RIGHT/LEFT interleave 후 64 | map scale과 무관 |
| transition family max `7` | 3 `COMPUTATIONAL` | short/medium/long의 bounded family set | family 내부 `1/4`, `1/2`, `1/64`는 category 2 geometry fractions |
| M0-like target grid의 점 개수 `5` | 3 `COMPUTATIONAL` | v1 direct anchor의 bounded anchor count | map scale과 무관 |
| M0-like target grid 간격 `/4` | 2 `DIMENSIONLESS_GEOMETRIC` | `a+i(b-a)/4`, `i=0..4` | domain과 자동 scaling |
| zero-interface direct anchor `/64` | 2 `DIMENSIONLESS_GEOMETRIC` | v1 `near+(far-near)/64` | domain과 자동 scaling |
| per-side lateral cap `32` | 3 `COMPUTATIONAL` | v1 bank cap; replacement는 수를 늘리지 않음 | map scale과 무관 |
| `K=12`, lexicographic `10`, coverage `2` | 3 `COMPUTATIONAL` | frozen R3-K12 quota를 상속 | map scale과 무관; 계산/coverage trade-off는 별도 연구 대상 |
| reconstruction max `12`, validator max `12` | 3 `COMPUTATIONAL` | K12 hard computation contract | map scale과 무관 |
| direct transition fractions entry `1/4`, exit-near `1/64`, exit-medium `1/2` | 2 `DIMENSIONLESS_GEOMETRIC` | v1 `e0+(e1-e0)/4`, `x0+(x1-x0)/64`, `x0+(x1-x0)/2` | entry/exit geometry와 자동 scaling |
| global comparison `epsilon=1e-9` | 3 `COMPUTATIONAL` | exact/deterministic comparison tolerance | 일반적인 meter-scale map에서 retuning 불필요 |

## `0.02 m`, `0.15 m`의 물리/기하 재표현 가능성

### `0.15 m`

현재 실차 config의 `vehicle_half_width_m`가 우연히 `0.15 m`이고 frozen R3 geometry도 같은
차량 반폭을 쓴다. 그러나 다음 이유로 v2의 inward `0.15 m`를 차량 반폭에서 유도했다고
볼 수 없다.

1. operator source는 parameter나 vehicle geometry를 읽지 않고 literal `0.15`를 사용한다.
2. frozen R3 factor-space에는 이미 일반 mid-offset set의 한 원자로 `±0.15 m`가 있었다.
3. component corridor를 만들 때 track은 이미 `vehicle_half_width(0.15)+wall_margin(0.04)`로
   inset되고, obstacle envelope도 이미 `vehicle_half_width(0.15)+safety_margin(0.08)`로
   확장된다. 즉 component geometry에는 footprint/clearance가 선반영돼 있다.
4. standalone v2 문서는 `0.15`를 vehicle half width가 아니라 DVE039 teacher lateral을
   재구성하는 pre-existing bank offset으로 설명한다.
5. simulation/operational margin 조합은 별도로 달라질 수 있어, 숫자 일치만으로 동일한 물리
   의미가 보존되지 않는다.

따라서 `0.15 m = vehicle_half_width_m`라는 식은 현재 코드의 **유도식이 아니라 사후적 수치
동일성**이다. 이를 물리식으로 선언하면 이미 inset된 corridor에 footprint 의미를 다시 부여하는
이중 해석 위험도 있다.

### `0.02 m`

현재 값들로는 `0.02=0.5*wall_safety_margin_m(0.04)` 또는
`0.02=0.25*safety_margin_m(0.08)`라고 쓸 수 있고, 기존 R3 proxy에도 `0.02 m` clearance
preference가 있다. 그러나 어느 관계도 operator source/spec/artifact에 유도 근거로 기록되어
있지 않다. simulation config에서는 wall/safety margin이 달라져 이 비율도 유지되지 않는다.
그러므로 이런 등식은 모두 `[INFERENCE]`인 사후 fitting이며 `PHYSICAL` 근거가 아니다.

현재 확인 가능한 가장 정확한 설명은 다음과 같다.

```text
0.02 m: pre-existing general target-offset atom
0.15 m: pre-existing general mid-offset atom
their component-midpoint/far/inward composition: DVE039-motivated v2 rule
```

두 값을 새 식으로 바꾸는 것은 이 감사 범위를 벗어나는 method 변경이다. 현 상태에서는
차량/clearance로부터 유도된 값이나 component 폭에 정규화된 값으로 **정직하게 재표현할 수
없다**.

## Dataset provenance와 ablation evidence

| 결정 | 사용된 evidence | 확인 결과 |
|---|---|---|
| displacement 구조 | DVE023 `DEVELOPMENT`; VUE009/VUE014 `VALIDATION_SEEN_AFTER_V1`; SCE008 seen success control | 8개 teacher-hard candidate 모두 B128 안, short/short, normalized near band `[0,1/32]`, exit-conflict proxy true |
| MIN vs MAX outward | `DEVELOPMENT_49` ablation | MIN이 DVE023을 회복해 hard `24->25`; MAX는 `24`로 회복 없음 |
| component far002/inward015 | `DEVELOPMENT_49`, 특히 DVE039 | DVE039의 `(-0.6496768991,-0.4996768991)` lateral을 component midpoint에서 정확히 재구성 |
| replacement 손실/추가 gain | `DEVELOPMENT_49` contribution ablation | 교체된 operator unique hard/usable event 0; 새 operator는 DVE039 외 V2E05도 회복 |
| 결합 policy 선택 | `DEVELOPMENT_49` | v1 `24 hard/21 usable`에서 `27/23`; DVE023과 DVE039 회복 |
| 선택 후 확인 | `VALIDATION_SEEN_37`, `SUCCESS_CONTROLS_18` | validation hard/usable `19/18` 유지, hard success control `17->18`; VUE/SCE가 이미 구조 설계에 노출됐다는 제한은 artifact에 명시됨 |

즉 v2 policy는 DEVELOPMENT score로 골랐지만 완전한 blind design은 아니었다. 특히
DVE039와 DVE023은 각각 operator와 `MIN_OUTWARD` 선택을 직접 구분한 named DEVELOPMENT
events다. V2E05라는 추가 gain은 single-event literal보다 넓은 작동 가능성을 보여주지만,
그것만으로 original parameterization의 event-specific provenance를 소거하지는 않는다.

## Source/history trace

- `0.02` target-offset atom과 `0.15` mid-offset atom은 production-integrated frozen R3 source의
  commit `17d853f3399fb68ed468bc30602633d39f59d7a0`
  (`2026-08-31 02:54:15 +0900`, `Integrate frozen R3-K12 recovery into local planner`)에 이미
  존재한다. 이 값들은 standalone v2가 처음 만든 숫자가 아니다.
- standalone v1 operator 정의는
  [`direct_operator_definitions.md`](../gqsc_standalone_direct_seed_v1/direct_operator_definitions.md)에
  B128, Top-10+2, component fraction, direct transition fraction을 명시한다.
- standalone v2의 결합 근거는
  [`dve039_lateral_analysis.md`](../gqsc_standalone_coverage_repair_v2/dve039_lateral_analysis.md),
  policy 선택 근거는
  [`diversity_policy_ablation.csv`](../gqsc_standalone_coverage_repair_v2/diversity_policy_ablation.csv)와
  [`operator_extension_ablation.csv`](../gqsc_standalone_coverage_repair_v2/operator_extension_ablation.csv)다.
- 현재 v2 연구 변경은 아직 pre-freeze working-tree 상태이므로 v2 전용 commit hash나 frozen
  policy SHA는 존재하지 않는다. 기존 atom commit을 v2 policy provenance와 혼동하면 안 된다.

## 최종 답

1. `0.02 m`와 `0.15 m`는 기존 general factor bank에 있던 절대 offset 원자이지만, standalone
   v2에서 둘을 component midpoint/far/inward 방향으로 결합한 규칙은 DVE039에 의해 선택됐다.
2. 두 값은 현재 source에서 vehicle geometry, safety margin, corridor 폭으로부터 유도되지 않으며
   map/vehicle scale에 따라 자동 scaling되지 않는다.
3. `1/32` near band, side-relative outward sign, component midpoint `1/2`, transition fractions는
   dimensionless geometry로 표현되어 scale-aware다.
4. `MIN_OUTWARD`, short/short band reserve, `1+1` semantic slot 배치는 named seen-event evidence로
   선택됐다. 계산량 `B128`, `K12`, `10+2`, lateral `64`, transition `7`은 computational이다.
5. 그러므로 현재 method를 단순히 "geometrically justified" 또는 "general hyperparameters only"
   로 분류할 근거는 부족하다. pre-freeze provenance 분류는
   **`GQSC_V2_HAS_MAP_OR_EVENT_SPECIFIC_PARAMETERIZATION`**이다.
