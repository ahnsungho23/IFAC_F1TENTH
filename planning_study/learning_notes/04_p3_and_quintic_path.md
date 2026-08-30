# P3와 quintic 회피 경로

## 1. 이 장의 핵심

[CURRENT IMPLEMENTATION] 현재 P3는 장애물 좌우에 통과 가능한 Frenet corridor를 만들고, 다섯 station의 횡오프셋을 정한 뒤 네 개의 quintic Hermite segment로 `C²`인 `d(s)`를 구성한다. 후보를 무제한 최적화하지 않고, M0-first와 M1 closure template을 정해진 순서로 제안하며 전체 constructed candidate는 최대 24개다([`p3_shadow.cpp:42`](../../src/local_planning/src/p3_shadow.cpp#L42), [`p3_shadow.cpp:310`](../../src/local_planning/src/p3_shadow.cpp#L310)).

이것이 중요한 이유는 “P3=5차 다항식”이 아니기 때문이다. 5차 다항식은 경로 표현이고, P3의 특징은 corridor, analytic root, branch filtering, M0/M1 candidate policy, exact validation, lexicographic ranking, lifecycle과의 결합에 있다.

## 2. 입력과 출력

[CURRENT IMPLEMENTATION] `P3ShadowEvaluator::run()`의 개념적 입력은 다음과 같다([`p3_shadow.cpp:75`](../../src/local_planning/src/p3_shadow.cpp#L75)).

- ego `s,d,speed`
- confirmed/guarded static obstacle envelopes
- ordered global reference와 track width
- entry/exit/offset/vehicle/validation parameters
- snapshot stamp, epoch, reference generation 같은 lineage

출력은 단일 path만이 아니다.

- 생성된 candidate trace의 ordered list
- 각 candidate의 identity, path digest, side, knot parameter, rejection reason
- M0/M1 count와 validator call count
- selected candidate와 selected path
- 실패 classification
- runtime·margin diagnostic

## 3. corridor부터 시작한다

[CURRENT IMPLEMENTATION] 각 station에서 먼저 track center가 들어갈 수 있는 횡구간을 계산한다. 그 구간에서 inflated obstacle interval을 빼고, 왼쪽 후보라면 장애물의 왼쪽에 연결되는 구간, 오른쪽 후보라면 오른쪽에 연결되는 구간을 고른다. station 사이에 interval overlap이 계속 존재해야 connected corridor다([`p3_shadow.cpp:791`](../../src/local_planning/src/p3_shadow.cpp#L791)).

```text
d
left wall  ─────────────────────────
           [ left free interval ]
obstacle       [ forbidden ]
           [ right free interval ]
right wall ─────────────────────────
                   s →
```

[INFERENCE] corridor는 연속 공간 전체의 완전한 증명이라기보다, reference/obstacle/start/end 등 선택된 station의 interval 구조를 사용한 candidate construction domain이다. 마지막 안전 판정은 reconstructed Cartesian waypoint의 exact validator가 맡는다.

## 4. 다섯 station과 다섯 offset

변수를 먼저 정의한다.

- `z0`: entry 시작 station
- `z1`: target offset에 도달하는 apex station
- `z2`: `z1`과 obstacle cluster end의 중간 station
- `z3`: padded cluster end
- `z4`: exit가 끝나 `d=0`으로 합류하는 station
- `d_e`: 현재 ego lateral offset
- `d_t`: obstacle 통과 target offset
- `d_m`: corridor analytic condition을 맞추는 middle offset

[CURRENT IMPLEMENTATION] station 배열은 다음 구조다([`p3_shadow.cpp:525`](../../src/local_planning/src/p3_shadow.cpp#L525)).

```text
z0             z1             z2             z3             z4
entry start    apex           middle station cluster end    merge
 d_e ───────→  d_t ───────→   d_m ───────→   d_t ───────→   0
```

정확한 offset vector는

`[d_e, d_t, d_m, d_t, 0]`

이다([`p3_shadow.cpp:1053`](../../src/local_planning/src/p3_shadow.cpp#L1053)). `z2`의 값이 단순히 `d_t`로 고정되지 않는 것이 핵심이다. analytic solver가 bottleneck probe에서 corridor 조건을 맞추도록 `d_m` root를 구한다.

`z0`와 `z1`은 entry scale과 현재 장애물까지의 거리로 정해진다. 목표 횡이동에 비해 장애물이 너무 가까우면 필요한 최소 전이 길이를

`required_transition = |d_t-d_e| / maximum_lateral_slope`

로 구하고 `z0=0`에서 시작하도록 fallback한다([`p3_shadow.cpp:540`](../../src/local_planning/src/p3_shadow.cpp#L540)). 이는 통과를 보장하는 것이 아니다. 그래도 obstacle box를 침범하면 validator가 기각한다.

### 경로 모양을 바꾸는 파라미터

[CURRENT IMPLEMENTATION] P3 파라미터는 독립적인 "부드러움 손잡이" 하나가 아니라 station과 offset을 함께 바꾼다. 주요 영향은 다음과 같다([`local_planning.yaml:384`](../../src/local_planning/config/local_planning.yaml#L384), [`p3_shadow.cpp:494`](../../src/local_planning/src/p3_shadow.cpp#L494)).

- entry scale을 키우면 일반적으로 횡이동을 더 앞에서 시작해 같은 `|d_t-d_e|`를 더 긴 `s` 구간에 분배한다.
- exit scale을 키우면 obstacle 뒤에서 `d=0`으로 돌아가는 구간이 길어져 merge가 완만해질 수 있다.
- target offset의 절댓값을 키우면 장애물과의 횡방향 여유는 늘 수 있지만, station 간격이 같다면 `|d'|`와 곡률 요구가 커질 수 있다.
- `maximum_lateral_slope`를 키우면 짧은 entry도 허용하지만, 이것은 Cartesian 곡률이나 차량 실현성을 자동 보장하지 않는다.
- `post_merge_lookahead`와 `post_merge_min_time`은 merge 뒤 출력 tail 길이를 정하며, 앞선 네 quintic segment의 boundary condition 자체를 바꾸는 파라미터는 아니다.

[GENERAL THEORY] 횡변위 `Δd`를 길이 `h`에 걸쳐 같은 무차원 profile로 만들면 chain rule 때문에 대략 `d'∝Δd/h`, `d''∝Δd/h²`로 스케일한다. 따라서 transition length가 절반이면 기울기 크기는 약 두 배, 두 번째 미분 크기는 약 네 배가 될 수 있다. 이것이 장애물이 늦게 보이거나 entry 구간이 짧을 때 path가 급해지는 수학적 이유다. 실제 P3는 다중 segment와 reference curvature를 사용하므로 이 비율은 직관적 scaling이지 최종 Cartesian curvature의 exact formula는 아니다.

## 5. 왜 quintic인가

[GENERAL THEORY] 하나의 polynomial segment를 normalized coordinate `t=(s-z_i)/h`, `0≤t≤1`로 쓰자.

`d(t)=c0+c1t+c2t²+c3t³+c4t⁴+c5t⁵`

양 끝에서 지정하고 싶은 조건은 여섯 개다.

- 시작: `d(0)=d0`, `d_s(0)=v0`, `d_ss(0)=a0`
- 끝: `d(1)=d1`, `d_s(1)=v1`, `d_ss(1)=a1`

계수도 여섯 개이므로 quintic이 자연스럽다. cubic은 위치와 1차 미분 네 조건에는 맞지만 양 끝 2차 미분까지 독립적으로 지정할 자유도가 부족하다.

## 6. 현재 코드의 quintic Hermite 계수

[CURRENT IMPLEMENTATION] 현재 구현은 `t`에 대한 계수에 interval length `h`를 미리 반영한다([`p3_shadow.cpp:594`](../../src/local_planning/src/p3_shadow.cpp#L594)).

먼저

```text
c0 = d0
c1 = h v0
c2 = 0.5 h² a0
r0 = d1 - c0 - c1 - c2
r1 = h v1 - c1 - 2c2
r2 = h² a1 - 2c2
```

를 정의하면

```text
c3 =  10r0 - 4r1 + 0.5r2
c4 = -15r0 + 7r1 - r2
c5 =   6r0 - 3r1 + 0.5r2
```

이다. `d_s`를 구할 때는 `t` 미분 뒤 `1/h`를 곱해야 한다. 코드의 `segmentD1()`도 이 chain rule을 사용한다([`p3_shadow.cpp:735`](../../src/local_planning/src/p3_shadow.cpp#L735)).

## 7. C0, C1, C2

[GENERAL THEORY]

- `C0`: knot 양쪽에서 위치 `d`가 같다. 경로가 끊기지 않는다.
- `C1`: 위치와 1차 미분 `d'`가 같다. 진행 방향이 갑자기 꺾이지 않는다.
- `C2`: 위치·1차·2차 미분 `d''`가 같다. lateral shape의 굽힘 변화가 knot에서 불연속하지 않는다.

[CURRENT IMPLEMENTATION] P3는 다섯 knot에서 하나의 derivative와 acceleration 배열을 먼저 만든 뒤, 이 값을 양옆 segment의 공통 boundary condition으로 넘긴다. 내부 derivative는 양쪽 secant가 같은 부호일 때 weighted harmonic derivative를, 부호가 바뀌면 0을 쓴다. 내부 acceleration은

`a_i = 2(slope_next-slope_previous)/(h_previous+h_next)`

이다([`p3_shadow.cpp:619`](../../src/local_planning/src/p3_shadow.cpp#L619), [`p3_analytic_solver.hpp:73`](../../src/local_planning/include/local_planning/p3_analytic_solver.hpp#L73)). 끝 knot derivative/acceleration은 배열 초기값 0이다.

이렇게 공유한 knot state 때문에 네 segment가 `C²`로 연결된다. 다만 `d(s)`가 C²라는 사실만으로 Cartesian curvature rate나 차량 steering rate까지 자동으로 안전하다는 뜻은 아니다. reference curvature와 sampling·normal offset이 함께 작용하므로 최종 geometry 검증이 별도로 필요하다.

## 8. textbook quintic과 현재 P3의 차이

[GENERAL THEORY] 흔한 lane-change 예제는 한 개의 segment에서 시작과 끝의 `d',d''`를 모두 0으로 둔다.

`d(t)=d0+(d1-d0)(10t³-15t⁴+6t⁵)`

이것은 quintic Hermite의 특수한 smoothstep이다.

[CURRENT IMPLEMENTATION] 현재 P3는 한 개 smoothstep이 아니다. 다섯 knot, 네 segment이며 내부 derivative와 acceleration을 harmonic/secant rule로 계산한다. 또한 `d_m`을 corridor probe 조건에서 analytic root로 풀고 branch sign·bound·forward residual을 검사한다([`p3_analytic_solver.hpp:226`](../../src/local_planning/include/local_planning/p3_analytic_solver.hpp#L226)). 따라서 “시작과 끝만 잇는 일반 textbook quintic”으로 설명하면 P3의 핵심을 놓친다.

## 9. M0와 M1의 실제 의미

[CURRENT IMPLEMENTATION] evaluator는 RIGHT, LEFT 순으로 side를 평가하고 M0 baseline에서 canonical corridor/probe mapping을 먼저 시도한다. hard-valid M0가 없으면 남은 전체 cap 안에서 M0 extension을 시도하고, 그래도 없을 때 M1을 호출한다([`p3_shadow.cpp:121`](../../src/local_planning/src/p3_shadow.cpp#L121), [`p3_shadow.cpp:186`](../../src/local_planning/src/p3_shadow.cpp#L186)).

M1은 connected component마다 다음 template을 round-robin 순서로 offer한다([`p3_shadow.cpp:1867`](../../src/local_planning/src/p3_shadow.cpp#L1867)).

1. `ZERO_BOUNDARY_SHORT`
2. `ZERO_BOUNDARY_SPAN`
3. `NEAR_LONG`
4. `FAR_SPAN`
5. 필요한 경우 `NEAR_BISECTED`

[INFERENCE] M0/M1은 polynomial 차수를 뜻하지 않는다. 둘 다 같은 5-knot C² reconstruction과 exact validator를 쓰며, corridor의 어떤 active-set/branch/template 후보를 어떤 순서로 제안하는지가 다르다.

## 10. candidate reconstruction

[CURRENT IMPLEMENTATION] 각 global waypoint의 ego-forward station에서 profile `d(s)`를 평가하고 normal shift로 `x,y`를 만든다. path end는 `z4` 뒤에

`tail=max(post_merge_lookahead, |ego_speed|×post_merge_min_time)`

만큼 추가된다([`p3_shadow.cpp:1045`](../../src/local_planning/src/p3_shadow.cpp#L1045)). 그 뒤 Cartesian geometry 재계산, avoidance speed shaping, hard validation을 거쳐 trace에 path digest와 지표가 저장된다.

## 11. 단순화한 숫자 예제

[GENERAL THEORY] 이해를 위해 현재 다중-segment P3가 아니라, 길이 `h=4 m`에서 `d0=0`, `d1=0.6 m`, 양 끝 `d'=d''=0`인 단일 textbook segment를 보자.

`d(t)=0.6(10t³-15t⁴+6t⁵)`

- 시작 `t=0`: `d=0`
- 중간 `t=0.5`: `d=0.3 m`
- 끝 `t=1`: `d=0.6 m`
- 중간의 `d'(s)=0.28125` 정도다.

평균 기울기는 `0.6/4=0.15`지만 중간의 순간 기울기는 더 크다. 따라서 목표 offset/transition length 비만 보는 것으로 maximum slope를 보장할 수 없고 sampled path의 실제 slope를 검사해야 한다.

[CURRENT IMPLEMENTATION] 실제 P3에서는 이 예제 네 개가 서로 다른 `h`, 내부 `d',d''`, `d_m`을 공유하며 이어진다고 생각하면 된다.

## 12. 자주 혼동하는 개념

- quintic은 P3 전체가 아니라 path representation의 한 요소다.
- knot 5개는 candidate 5개라는 뜻이 아니다.
- `d(s)`의 `C²`와 Cartesian curvature의 `C²`는 같은 문장이 아니다.
- analytic root는 최종 안전 해답이 아니라 corridor condition을 맞춘 candidate proposal이다.
- candidate cap 24는 모든 가능한 `d(s)` 공간을 완전 탐색한다는 뜻이 아니다.

## 반드시 설명할 수 있어야 하는 질문

1. 다섯 station과 `[d_e,d_t,d_m,d_t,0]`는 각각 무엇을 뜻하는가?
2. quintic이 여섯 boundary condition에 자연스러운 이유는 무엇인가?
3. 현재 P3가 textbook single smoothstep과 다른 점은 무엇인가?
4. knot state 공유가 C²를 만드는 이유는 무엇인가?
5. M0/M1과 polynomial 차수를 혼동하면 안 되는 이유는 무엇인가?
6. analytic root 뒤에도 exact validator가 필요한 이유는 무엇인가?
