# Frozen method specification

## 상태

이 문서는 `VALIDATION_UNSEEN`을 열기 전에 DEVELOPMENT에서 동결할 수 있는 offline method prototype을 정의한다. P3 spline family, reconstruction, validator, ranker, lifecycle, vehicle/collision parameter와 production YAML은 그대로 둔다.

machine-readable authority는 [method_spec.json](method_spec.json)이다. 이 문서는 같은 내용을 사람이 검토할 수 있게 풀어쓴다.

## Factor 정의

완전한 direct P3 tuple은 다음과 같다.

\[
q=(L,T)=(side,d_{target},d_{mid},\alpha_{entry},\alpha_{exit}).
\]

`L`은 side, `d_target`, `d_mid`와 두 값의 생성 provenance다. `T`는 같은 event의 production이 실제 생성한 unique entry/exit scale pair다. 새로운 station 수, spline 차수, knot semantics는 추가하지 않는다.

### Lateral pool

각 side에서 다음 값을 사용한다.

1. production의 모든 unique `(d_target,d_mid)`.
2. production의 모든 unique `d_target`.
3. corridor connected component의 near-inset, 1/4, center, 3/4 target.
4. 각 target에 대해 `d_mid`를 target, bottleneck center, target-center 중간, target의 0.75배, target의 0.5배로 만든다.

component endpoint `near/far`는 `(|d|,d)` 순으로 정한다. near-inset은 `near+(far-near)/64`다. LEFT domain은 `[0,1.5]`, RIGHT는 `[-1.5,0]`이고 component는 최대 두 개다.

### Transition pool

해당 evaluation의 production candidate에 이미 나타난 모든 unique `(entry_scale,exit_scale)`만 사용한다. event 안의 min/max로 각각 `[0,1]`에 정규화한다. 값이 하나뿐이면 0이다.

### 제거한 coupling

production template이 허용한 exact full tuple만 재사용하는 제한을 제거한다. 위 L과 T를 factor pool에서 조합하지만, production이 이미 검증한 exact tuple은 proposed stage에서 다시 실행하지 않는다. 새로운 P3 family가 아니라 기존 direct P3 자유도의 재조합이다.

## Pre-planning geometry

[geometry_features.csv](geometry_features.csv)는 candidate validation 전에 얻을 수 있는 다음 값을 보존한다.

- ego `d`, speed
- obstacle start distance, longitudinal span
- inflated obstacle lateral interval
- chosen-side free interval/width
- corridor bottleneck interval/center/width/station
- available entry 및 merge/exit distance
- entry/obstacle/exit reference curvature
- maximum absolute curvature와 curvature sign change
- track-width variation

oracle classification, hard-valid outcome, first-failure, margin은 selector input이 아니다.

## H3 fixed bounded basis

geometry outcome에 따라 순서를 바꾸지 않는다. lateral priority와 다음 fixed transition archetype priority를 합산한다.

```text
(short entry, short exit)
(long entry, short exit)
(short entry, long exit)
(long entry, long exit)
(mid entry, mid exit)
```

각 transition의 priority는 `archetype index + normalized Euclidean distance`의 최소값이다. full tuple score와 tie-break는 JSON에 고정했다.

## H4-A geometry-conditioned transition

Lateral pool의 고정 priority는 유지하고 transition 목표만 geometry로 정한다.

\[
e^*=1\quad\text{if }D_{entry}<0.25\;\lor\;|\kappa|_{max}>0.5\;\lor\;D_{obs}>2.0,
\]

그 외에는 `e*=0`이다.

\[
x^*=0\quad\text{if }D_{merge}<2.0,
\]

그 외에는 curvature sign change 또는 track-width variation `>0.6 m`이면 `x*=1`, 아니면 0이다.

\[
E_T=|e-e^*|+|x-x^*|,
\]

\[
score_{H4A}=E_T+0.20P_L.
\]

secondary tie-break는 `E_T`, lateral priority, RIGHT-before-LEFT, full-tuple lexical order다.

물리적 의미는 짧거나 높은 곡률의 entry에는 긴 ramp를, merge 공간이 부족하면 짧은 exit를, sign-changing/width-varying 구간에는 긴 exit를 우선하는 것이다. obstacle이 2 m보다 멀면 긴 entry를 구성할 longitudinal 공간이 있다고 본다.

## H4-B geometry-conditioned lateral and transition

H4-A의 transition score와 다음 lateral 목표를 함께 쓴다.

- bottleneck width `<0.25 m`: narrow corridor.
- obstacle start `>2.0 m`: component 3/4를 desired target/mid로 사용.
- 그 외 target: narrow이면 bottleneck center, 아니면 near-inset.
- 그 외 mid: obstacle span `>=0.12 m` 또는 narrow이면 bottleneck center, 아니면 candidate target.

\[
E_L=\frac{|d_t-d_t^*|+|d_m-d_m^*|}{\max(0.1,w_{bottleneck})},
\]

\[
score_{H4B}=E_L+1.5E_T+0.05P_L.
\]

tie-break는 `E_L`, `E_T`, RIGHT-before-LEFT, full-tuple lexical order다.

## Budget, deduplication, fallback

- score 오름차순 exact tuple 중 처음 K개만 reconstruction 대상으로 선택한다.
- 평가 K는 `4,8,12,16,24`다.
- exact tuple을 먼저 deduplicate한다.
- reconstruction 후 동일 path digest는 첫 path만 exact validator에 전달한다.
- production을 먼저 실행한다. production hard-valid가 1개 이상이면 proposed stage는 실행하지 않고 기존 선택을 유지한다.
- production hard-valid가 0일 때만 proposed stage를 실행한다.
- proposed stage도 실패하면 기존 safe-stop/fallback/lifecycle authority를 그대로 유지한다.

현재 detached audit harness는 reconstruction과 validator를 한 호출에서 수행하므로 실제 audit 실행은 duplicate도 검증했다. 결과 CSV는 raw harness execution과 frozen method가 요구하는 unique-digest validator count를 모두 기록한다. production 구현 전에는 reconstruction/digest와 validator 사이의 dedup 경계를 별도로 보존해야 한다.

## Frozen prototype 선택

- primary: `H4A_GEOMETRY_TRANSITION`, `K=24`
- fallback/efficiency ablation: `H4B_GEOMETRY_LATERAL_TRANSITION`, `K=12`

선정 근거와 exact SHA는 [selected_prototype_spec.json](selected_prototype_spec.json) 및 sidecar SHA 파일에 기록한다.

