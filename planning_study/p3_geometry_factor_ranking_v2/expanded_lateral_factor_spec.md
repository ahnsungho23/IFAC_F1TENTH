# Expanded lateral factor specification v2

## 범위

이 factor space는 기존 direct P3 tuple

\[
q=(L,T)=((side,d_{target},d_{mid}),(\alpha_{entry},\alpha_{exit}))
\]

만 사용한다. 새 spline family, 새 transition 값, oracle 좌표 삽입은 없다. Online 생성 입력은 ego, 현재 obstacle snapshot, reference path, strict corridor뿐이다. Machine-readable authority는 [factor_space_spec.json](factor_space_spec.json)이다.

## Target factor

Strict-valid side마다 blocking-cluster 전 구간에서 연결된 free-space component `C_j=[a_j,b_j]`를 계산한다. `near_j`는 절댓값이 작은 끝, `far_j`는 반대 끝이다.

\[
d_{target}=near_j+f(far_j-near_j)
\]

여기서 `f`는 다음 고정 집합이다.

`{0, 1/64, 1/32, 1/16, 1/8, 3/16, 1/4, 3/8, 1/2, 5/8, 2/3, 3/4, 7/8, 15/16, 63/64, 1}`

여기에 다음 geometry anchor를 합친다.

- same-side production `d_target`와 `±{0.01,0.02,0.04,0.06,0.08,0.10} m`
- bottleneck center
- obstacle-span midpoint의 corridor center
- blocking span 안 reference/corridor sample의 center trend

모든 target은 strict side bounds로 clip하고 exact binary64 값으로 deduplicate한다.

## Mid factor

각 target `t`와 corridor-center anchor `c_i`에서 다음을 생성한다.

\[
d_{mid}\in\left\{
t,\;t+\lambda(c_i-t),\;\rho t,\;t+\delta
\right\}.
\]

- `lambda ∈ {1/8,1/4,3/8,7/16,1/2,3/4,1}`
- `rho ∈ {1/4,1/2,3/4}`
- `delta ∈ ±{0.01,0.02,0.04,0.05,0.08,0.10,0.15,0.20} m`
- exact production `(d_target,d_mid)` pair도 보존
- 최종 `d_mid`는 `[-1.5,1.5] m`로 clip

`7/16`은 특정 oracle 좌표를 직접 넣은 것이 아니다. target과 관측 corridor-center 사이를 일정 비율로 나누는 deterministic basis이며, seen 설계에서 coarse `3/8`과 `1/2` 사이의 공백을 줄이기 위해 추가했다.

## Transition factor

`T`는 해당 evaluator input에서 production이 실제 생성한 unique exact `(entry_scale,exit_scale)` pair 전부다. Entry/exit 값을 새로 만들거나 서로 독립된 임의 grid로 확장하지 않는다.

## Full product를 만들지 않는다는 의미

모든 `L`과 `T` 조합에 대해 cheap priority는 계산하지만, P3 reconstruction과 exact validator는 top-K에만 사용한다. 따라서 cheap pair count는 event별 seen 범위에서 `1,012–71,506`이지만 selected K12의 expensive work는 event당 P3 12개, validator 최대 12회다.

## Deduplication

순서는 다음과 같다.

1. exact lateral tuple dedup
2. exact full tuple dedup
3. `stationsFor()`를 cheap하게 복원한 `(side,d_target,d_mid,z0..z4)` shape dedup
4. P3 reconstruction 후 path digest dedup

마지막 단계의 duplicate도 이미 실제 P3로 구성됐으므로 K를 소비한다. 다만 같은 digest는 첫 path만 logical exact validator에 전달한다.

## 알려진 한계

- factor 수 자체는 큰 cheap hypothesis set이며 hard real-time benchmark는 수행하지 않았다.
- reference-waypoint discrete proxy는 exact Cartesian curvature/footprint validator가 아니다.
- seen data에서 만든 basis이므로 unseen 일반화는 final holdout 전까지 주장하지 않는다.
- `s_probe`나 analytic root를 새로 푸는 정책이 아니다. 기존 P3의 direct degrees of freedom을 geometry-derived factor로 제안하는 recovery layer다.
