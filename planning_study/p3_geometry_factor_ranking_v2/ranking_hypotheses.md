# Ranking hypotheses

세 hypothesis 모두 같은 expanded factor space와 exact P3 harness를 사용한다. 차이는 reconstruction 전 순위뿐이다.

## R1 — feasibility lexicographic

순위 tuple은 다음 오름차순이다.

1. later-obstacle corridor conflict proxy
2. corridor max violation 존재 여부와 값
3. slope excess 존재 여부와 값
4. corridor violation 합
5. curvature proxy
6. corridor-center error
7. negative minimum clearance
8. shape energy
9. stable tie

Weight fitting이 거의 없고 feasibility hierarchy가 명시적이라는 장점이 있다. Combined seen usable은 K8/12/16/24에서 `29/32/33/33` of 36이다.

## R2 — balanced physics score

\[
S=40v_{max}+2v_{sum}+8e_{slope}+0.12q_{curv}
+0.25e_{center}+2\max(0,0.02-c_{min})+0.02E+0.002P.
\]

Exit-conflict proxy를 먼저 비교하고 그 뒤 `S`를 비교한다. `0.8` slope limit와 `0.02 m` clearance preference를 제외한 계수는 SEEN 설계용 heuristic이며 safety threshold가 아니다. Combined seen usable은 `27/30/33/33`이다.

## R3 — lexicographic with coverage reserve

Selected hypothesis다. R1 stream과 coverage stream을 별도로 best-first 생성한다. Coverage stream은 R2 score에 factor-space 거리 보상을 준다.

\[
D=0.5I(side\ne side_j)+\frac{|t-t_j|}{1.5}
+\frac{|m-m_j|}{3}+0.2|e-e_j|+0.2|x-x_j|,
\]

\[
S_{coverage}=S-2\min(1,\min_j D).
\]

K12 quota는 R1 fully reconstructed 10개 뒤 coverage 2개다. General schedule은 R1 1–10, coverage 1–2, R1 11–18, coverage 3–6, 나머지 R1, 나머지 coverage다. Validation outcome은 quota switching에 사용하지 않는다.

Combined seen usable은 `29/32/33/34`, hard는 `31/36/38/39`다. K12에서 R1과 usable 회복 수는 같지만 R3는 predeclared budget case DVE039를 회복하고 hard recovery가 35에서 36으로 늘었다. DVE002를 반대로 잃으므로 이는 pure superset이 아니다.

## Stable tie

모든 hypothesis의 마지막 tie는 다음 순서다.

`source_priority → lateral_factor_index → transition_index → RIGHT before LEFT → exact configuration key`

## 선택 근거

R3 K12를 선택했다.

- H4-B K12 대비 combined usable `22/36 → 32/36`
- R3 K16은 `33/36`으로 한 건만 증가하므로 K12를 선호
- 세 bag 모두에서 recovery: `12 + 12 + 8`
- success control에서는 production-first로 18/18 모두 v2 미호출
- remaining four usable misses는 full expanded pool에 usable path가 있으나 budget/ranking 밖인 설명 가능한 miss

이는 SEEN 설계 결과이며 final holdout 성능 보장이 아니다.
