# FINAL HOLDOUT — immutable R3-K12 evaluation v1

상태: `FINAL_HOLDOUT_OPENED_AND_EVALUATED_NO_LONGER_UNSEEN`. 동결 checkpoint
`1fe52ed5ea6c64d1deb2dcb9059e3ca64fa5f83d` (`p3_r3_k12_pre_holdout`)에서 사전 선언된 one-shot runner만 사용했다.
holdout open: `2026-08-30T15:38:48.269142+00:00`, evaluation end: `2026-08-30T16:30:09.430490+00:00`. Production planner, validator,
R3 ranking, parameter, lifecycle는 수정하지 않았고 commit/push도 하지 않았다.

## Primary result

Reference Oracle v2에서 usable-feasible인 production failure 19개 중 R3-K12가
17개를 usable recovery했다. 비율은 `0.894737`,
Wilson 95% CI는 `[0.686059,
0.970641]`이다. Hard recovery는 17/20
(`0.850000`, Wilson 95% CI `[0.639581, 0.947631]`)이다. Usable 기준 Oracle-infeasible
production failure는 18개였다(그중 FHE014는 hard-feasible이지만 usable-infeasible).
Seen 결과는 usable 32/36,
hard 36/48이었으며 holdout 차이는 각각
`+0.585 pp`,
`+10.000 pp`이다.

## Frozen comparison

| Method | Hard | Usable | Candidates | Validators |
|---|---:|---:|---:|---:|
| H3_FIXED_BOUNDED_K24 | 14 | 14 | 888 | 821 |
| H4A_GEOMETRY_TRANSITION_K24 | 15 | 15 | 888 | 755 |
| H4B_GEOMETRY_LATERAL_TRANSITION_K12 | 13 | 13 | 444 | 366 |
| R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12 | 17 | 17 | 444 | 393 |

동일한 Oracle-usable denominator 19개에서 H4-B K12는 13개(68.4%), R3-K12는
17개(89.5%)를 회복했다. R3가 추가 회복한 4개는 FHE016, FHE017, FHE028,
FHE035이며, H4-B의 13개 성공은 모두 R3 성공과 겹친다. 따라서 동일 reconstructed
candidate cap K=12에서 R3가 `+4 events`, `+21.1 percentage points`였다. 다만 exact
validator 호출은 H4-B 366회, R3 393회였고 둘 다 event당 최대 12회였다.

R3의 contract budget은 episode당 fully reconstructed candidate 최대 12, path-digest
dedup 뒤 logical exact-validator 최대 12이다. `raw_harness_validator_executions`는 두 rank
stream을 검증하기 위한 offline 연구 audit overhead이며 deployable K=12 예산과 구분했다.
37개 모두 정확히 12개를 reconstruct했고 logical validator는 총 393회, event 최대
12회였다. 기록된 R3 runtime(p50 46.97 ms, p95 121.19 ms, p99 156.63 ms, max
174.64 ms)은 두 ranking stream을 함께 검사하는 standalone 연구 harness 시간이라
production callback latency로 해석할 수 없다. Oracle은 23,636,273 requests 중
9,094,829 paths를 construct/validate했고 총 wall time은 2,220.24 s였다.

17개 R3 best-usable path의 최소 margin은 center-track 0.1700 m,
footprint-track 0.01349 m, obstacle 0.01117 m, lateral-slope 0.13865,
signed-curvature 0.38259 rad/m, curvature-rate 17.42483 rad/m²였다. 최소 normalized
safety slack은 0.007448이고 braking deficit은 17개 모두 0이었다. 전체 min/p50/max는
`safety_margin_summary.csv`에 있다. 이는 exact snapshot validator diagnostics이며
closed-loop robustness margin을 뜻하지 않는다.

## Required conclusions

1. **R3-K12 final usable recovery:** 17/19, Wilson CI는 위와 같다.
2. **Seen 대비 유지:** 절대 차이는 `+0.585 pp`이다. 이는 기술통계이며 별도 허용폭을 사후 정의하지 않았다.
3. **여러 bag/input domain:** usable 성공은 3개 bag, 2개 reference domain에 걸친다.
4. **H3/H4-A/H4-B 비교:** `final_method_comparison.csv`의 exact 동일-event 결과를 따른다.
5. **주요 실패 메커니즘:** `{"FACTOR_SPACE_COVERAGE_MISS": 2, "NO_USABLE_P3_IN_REFERENCE_ORACLE_V2_DOMAIN": 18, "R3_K12_USABLE_RECOVERY": 17}`. 두 miss는 FHE022와 FHE033이다. 여기서 `FACTOR_SPACE_COVERAGE_MISS`는 **Oracle best usable exact tuple이 동결 R3 factor pool에 없었다**는 제한된 기술 분류다. R3 pool 전체에 다른 usable tuple이 없었다는 증명은 아니다.
6. **Probe/root 관련 잔여 실패:** 이 평가만으로 설계 의도를 추측하지 않는다. 두 exact-tuple 부재는 probe/root 좌표 또는 다른 factor 좌표 차이를 포함할 수 있으나, probe/root를 단독 원인으로 확정하지 않는다.
7. **계산 예산:** K=12 및 logical validator cap은 지켜졌다. Oracle은 상한 평가용 offline finite-domain 계산이므로 production budget과 직접 비교하지 않는다.
8. **Generalization evidence:** usable 비율은 seen 88.9%에서 holdout 89.5%로 `+0.585 pp`였고 3개 bag/2개 reference domain에서 성공했으므로, 이 동결 corpus의 exact-snapshot P3 recovery에는 generalization을 지지하는 evidence가 있다. 그러나 19개 usable-feasible denominator의 Wilson CI가 넓고 동일 기록 corpus 기반이므로 통계적 유의성이나 광범위한 환경 일반화는 주장하지 않는다.
9. **지원되는/지원되지 않는 주장:** 지원되는 주장은 “동결 finite-domain Oracle 기준에서 R3-K12가 H4-B K12보다 4개 더 usable recovery했고 K/validator cap을 지켰다”이다. 지원되지 않는 주장은 continuous P3 completeness, 새 트랙·새 센서 조건 일반화, closed-loop 안정성, real-time production latency, 실제 차량 안전성, production superiority이다.
10. **Holdout 상태:** 37개 행은 이제 unseen이 아니며 같은 결과에 대한 재튜닝·재실행은 금지한다.

## Closed-loop simulation decision

R3-K12는 **동결 method 그대로 closed-loop simulation 평가로 진행할 연구 후보로는 준비됐다**.
이는 production integration 승인이 아니다. 다음 단계는 이 holdout을 다시 튜닝에 쓰지 않고,
별도의 closed-loop scenario·runtime·lifecycle/safe-stop interaction·repeatability gate를 통과하는지
검증하는 것이다. 실패해도 이 final holdout에 맞춘 R3 수정/재실행은 허용되지 않는다.

Exact lineage, production constructed/returned digest multiset parity, Oracle coverage와
production validator parity는 전 event에서 fail-closed로 검증했다. 개별 수치와 domain,
failure taxonomy, 계산량은 동명 CSV에 기록했다.
