# Selected v2 method freeze

## Decision

`R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12`를 **final holdout에 한 번 평가할 v2 candidate로 동결**한다. 이번 작업에서는 `FINAL_HOLDOUT_UNSEEN`을 읽거나 실행하지 않았다. Machine-readable authority는 [selected_v2_method_spec.json](selected_v2_method_spec.json)이다.

## Production-first activation

```text
production hard-valid exists
  -> production result 사용
  -> v2 factor generation/ranking/reconstruction/validation 0회

production hard-valid absent
  -> v2 R3-K12 recovery stage
  -> 실패하면 기존 lifecycle/fallback authority 유지
```

V2는 replacement planner가 아니다.

## Exact K12 algorithm

1. Strict-valid side context와 expanded lateral factors를 생성한다.
2. 해당 evaluation의 production transition pair와 결합해 cheap pair descriptor를 계산한다.
3. Exact tuple과 exact preconstruction shape `(side,d_target,d_mid,z0..z4)`를 deduplicate한다.
4. R1 feasibility-lexicographic stream과 coverage stream을 만든다.
5. R1 stream에서 construction guard를 통과한 P3 10개를 만든다.
6. Coverage stream에서 construction guard를 통과한 P3 2개를 만든다.
7. 총 12개의 fully reconstructed P3에서 path digest가 같은 경우 첫 path만 exact validator에 전달한다.
8. Frozen hard validator와 frozen USABLE predicate를 적용한다.

Construction guard failure는 K를 소비하지 않는다. Path digest duplicate는 이미 P3가 만들어졌으므로 K를 소비하지만 validator를 다시 호출하지 않는다.

## R1 order

다음 tuple 오름차순이다.

`exit conflict proxy → max corridor violation flag/value → slope excess flag/value → violation sum → curvature proxy → center error → -minimum clearance → shape energy → stable tie`

## Coverage order

기본 score는

\[
40v_{max}+2v_{sum}+8e_s+0.12q_c+0.25e_c
+2\max(0,0.02-c_{min})+0.02E+0.002P
\]

이고 이미 선택한 factor와의 normalized distance `D`에 대해 `score-2*min(1,D)`를 greedy하게 최소화한다. Exact formula와 normalization은 JSON authority에 고정했다.

## Tie and dedup freeze

- tie: `source_priority`, `lateral_factor_index`, `transition_index`, RIGHT-before-LEFT, exact configuration lexical
- numeric tuple identity: binary64 hex exact equality
- preconstruction shape identity: exact side/target/mid/stations
- validator dedup: first path digest

## K 선택

Combined seen primary usable recovery:

| K | R3 usable | hard |
|---:|---:|---:|
| 8 | 29/36 | 31/48 |
| **12** | **32/36** | **36/48** |
| 16 | 33/36 | 38/48 |
| 24 | 34/36 | 39/48 |

K16은 usable 한 건만 더 회복한다. 요청된 simplicity/budget criterion에 따라 K12를 선택했다.

## Frozen validity contract

```text
HARD_VALID_P3
AND !exit_reaches_next_obstacle
AND braking_deficit_m <= 1e-9
```

변경하지 않았다.

## Compute contract

DEVELOPMENT+VALIDATION_SEEN_AFTER_V1 77 failure event에서:

- fully reconstructed P3: `924 = 77*12`
- unique path digest / logical validator: `852`
- per-event reconstruction: 정확히 12
- per-event logical validator: 최대 12
- cheap lateral factors: 104,155 total
- cheap pair priorities: 694,600 total

Cheap pool은 full P3로 materialize하지 않는다. Runtime 수치는 detached offline harness라 production deadline 증거가 아니다.

## Seen evidence

| corpus | hard | usable |
|---|---:|---:|
| DEVELOPMENT | 17/23 | 14/17 |
| VALIDATION_SEEN_AFTER_V1 | 19/25 | 18/19 |
| combined seen | 36/48 | 32/36 |
| pilot seen auxiliary | 4/5 | 4/5 |

H4-B K12 combined usable `22/36`보다 10 episode 증가했다. Recovery는 세 bag에 `12/12/8`개로 분포한다.

## Remaining four usable misses

- DVE002: coverage quota 때문에 R1에서 K12 회복되던 factor를 잃은 ranking interaction
- DVE005: 더 큰 K의 R1/R3에서는 회복되는 budget/rank miss
- DVE022: expanded pool usable 58개, 첫 usable rank 51–64
- VUE035: expanded pool usable 32개, coverage rank 16이지만 K12 coverage quota는 2

따라서 네 건 모두 현재 증거에서는 factor-space/P3-family 부재가 아니라 bounded ranking/quota miss다.

## Freeze hashes

| authority | SHA-256 |
|---|---|
| selected method JSON | `7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc` |
| factor-space JSON | `949971060ad43d09eb859030409076bd36f14a9f544fd36a784b9413e900b2e6` |
| offline evaluator | `2cd87b4c9faf9e9f84c96295f137b4edeed21fafc9abeccf48298b603f075c03` |
| sensitivity audit | `13c024ce6a926e6056c649fc6591227ae8f196df0b0cbfce458ca184a6036dff` |

Sidecar는 [selected_v2_method_spec.sha256](selected_v2_method_spec.sha256), [factor_space_spec.sha256](factor_space_spec.sha256), [run_factor_ranking_v2.sha256](run_factor_ranking_v2.sha256)에 있다. Production source, validator, ranker, parameter는 변경하지 않았다.
