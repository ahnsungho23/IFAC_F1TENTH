# GQSC v3 true coverage regression mechanism audit v1

## 결론

`GQSC_V3_MIXED_COVERAGE_LIMIT`

Frozen `V3_SIDE_BALANCED_DISJOINT`와 exact validator를 바꾸지 않고 provenance ladder를
재구성했다. method JSON SHA-256은
`965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780`이다.

가장 중요한 정정은 property의 **200 miss가 200 coverage miss가 아니라는 것**이다.

- 101: blocking obstacle이 아닌 정상 global-line 사례인데 wrapper가 legacy reason을 보존하지 않은 결과-semantic regression
- 3: valid side geometry가 없어 direct transition set이 empty가 된 invariant return
- 83: conservative 1D corridor만 존재하고 exact five-knot P3 witness는 확인되지 않음
- 13: exact same-P3 hard-valid witness가 실제 존재하는 genuine GQSC coverage miss

13개 exact property miss는 Top-12 displacement 9, B128 truncation 2, lateral operator miss 2다.
별도의 true production/raceline regression 7 frame은 Top-12 1, transition operator 5,
parameter/input mismatch 1이다. 합친 exact-proven 20 snapshot은 selection-only 10/20,
pair-policy 2/20, geometry operator 7/20, parameter/interface 1/20이며 proven P3 representation
limit은 0/20이다. property와 production에서 지배 원인이 다르므로 단일 selection 또는 단일
operator 원인 대신 `MIXED`가 정확하다.

## Audit contract

- A: exact lateral factor 존재
- B: exact transition factor 존재
- C: 두 factor가 frozen pair ordering에 들어갈 수 있음
- D: materialized B128 pair pool에 존재
- E: ordered 10+2 Top-12에 존재
- F: frozen GQSC가 실제 reconstruct/validate
- G: final selected path

구현은 full Cartesian pair container를 만들지 않고 transition waves로 B128을 채운다. 따라서 C는
operator-pair eligibility, D는 실제 bounded materialization이다. E 밖 witness는 audit-only direct
reconstruction adapter로 같은 constructor/validator를 통과시켰고 planner decision에는 넣지 않았다.

## 1. 가장 이른 provenance 단계

Exact property 13개 기준: A에서 2건, D에서 2건, E에서 9건이 처음 탈락했다. Production/raceline
7 frame은 B에서 5건, E에서 1건, geometry parameter contract 전에 1건이 탈락했다.
Dedup, reconstruction mismatch, validator-input mismatch-after-reconstruction은 관찰되지 않았다.

## 2. 주된 원인은 무엇인가

전체 exact-proven snapshot에서는 selection-only가 50%로 가장 크지만, production regressions는
transition coverage가 5/7로 지배적이다. 따라서 **ranking만의 문제도, operator만의 문제도 아니다**.
B128 자체의 고유 miss는 property 2건으로 작지만 실재한다.

## 3. 새 operator 필요성

필요한 exact evidence는 lateral 2 property case와 transition 5 production frame에서만 확인됐다.
그러나 이 audit은 어떤 operator도 설계하거나 추가하지 않았다. 83 corridor-only cases에는 exact P3
witness가 없으므로 새 operator 필요 사례로 세지 않았다.

## 4. B128/K12 완화 효과

- K/selection policy만 바꾸어 회복 가능한 증거: 10/20 exact snapshots
- B128 pair policy/budget까지 바꾸어 회복 가능한 증거: 추가 2/20
- operator 없이 B/K만 늘려도 회복되지 않는 증거: transition/lateral 7/20

큰 B/K를 실제 실행하지 않았으므로 이는 ladder상 fixability이지 성능 재평가가 아니다.

## 5. Geometry distribution shift

`geometry_distribution_success_vs_miss.csv`에 538개 전부의 corridor width, ego d/speed,
cluster station/span, side-domain width 분포를 나눴다. selection miss는 valid geometry와 K=12를
모두 소비하는 반면, 101 semantic miss와 3 invariant case는 geometry bank 자체가 비어 있다.
즉 “miss geometry가 전부 더 좁다”는 단일 shift는 성립하지 않는다. exact operator/selection miss와
nonblocking/corridor-only population이 혼합돼 있다.

## 6. 원칙적인 다음 단계

원인별로 분리해야 한다: (1) nonblocking result contract 복구, (2) hard-coded frozen geometry와
runtime parameter contract 일치, (3) Top-12/B128 selection coverage 연구, (4) repeated transition
mechanism 연구. P3 family 변경 근거는 현재 없다. 이 순서는 진단 결론이며 구현 제안이나 tuning이 아니다.

## Source evidence and limits

- direct transition construction/bound: `src/local_planning/src/p3_r3_k12.cpp:1736`, `:2047`
- bounded wave/pair materialization: `src/local_planning/src/p3_r3_k12.cpp:2121`
- frozen geometry constants: `src/local_planning/src/p3_shadow.cpp:1285`, `:1323`
- property oracle is independent 1D reachability, not P3: `src/local_planning/test/test_rule_property.cpp:27`, `:99`

`[INFERENCE]` Design intent behind the hard-coded frozen geometry values is not recoverable from
these executions. This audit reports their observed contract mismatch, not why they were chosen.

No FINAL_HOLDOUT path was listed, opened, parsed, or executed. No closed-loop replay was run.
GQSC, validator, ranking, B128/K12, parameters, and production decision behavior were not changed.
