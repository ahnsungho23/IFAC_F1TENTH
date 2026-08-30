# P3 Family Oracle Pilot v1

## 결론

이 파일럿에서는 **hard-valid P3 oracle solution이 0/5개**였다. Exact candidate-producing
sub-evaluation 입력 parity를 입증한 E02와 E04는 각각
`NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN`으로 분류했다. E01, E03, E05는 callback-global
instrumentation만으로 해당 하위 평가의 입력 lineage를 복원할 수 없어 `ORACLE_INCONCLUSIVE`다.
따라서 이번 결과에는 `PILOT_EVIDENCE_OF_MAPPING_MISS`가 하나도 없으며, 이 5개만으로
`s_probe`/`d_probe` mapping 연구로 진행하는 것은 **NO-GO**다.

| event | exact baseline callback | input parity | oracle validator executions | hard-valid | classification |
|---|---:|---|---:|---:|---|
| E01 | 20068 | unresolved | 0 | 0 | ORACLE_INCONCLUSIVE |
| E02 | 7557 | exact | 59206 | 0 | NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN |
| E03 | 8697 | unresolved | 0 | 0 | ORACLE_INCONCLUSIVE |
| E04 | 10562 | exact | 9824 | 0 | NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN |
| E05 | 22672 | unresolved | 0 | 0 | ORACLE_INCONCLUSIVE |

## 해석 계약과 입력 선정

이 실험은 original historical `/avoid_waypoints`가 아니라 tag `research_instrumentation_v1`
(`6ba46dbe90139d184c066740a3cbbf191d41cc61`)의 **FROZEN-BASELINE REPLAY BEHAVIOR**를 다룬다. 네 bag의 historical commit은
모두 정확히 알려져 있지 않다. E01/E02는 waypoint digest `b9a0cd...`(185 points,
monte_carlo_localization provenance), E03/E04/E05는 `ded35f...`(185 points,
kinematic_localization provenance)를 사용했다. 전체 문자열은 `event_manifest.csv`에 보존했다.

5개 이벤트는 모두 `|localization join offset| <= 25 ms`이고 pose sample gap이 `<= 50 ms`여서
pilot의 temporal coverage 조건은 통과했다. 이 조건은 localization quality threshold가 아니며,
label은 전부 `LOC_UNKNOWN`이다. **어떤 이벤트도 `LOC_CLEAN`이라 부르지 않는다.**

Rounded elapsed가 아니라 baseline의 exact `(ros_time_ns, callback_sequence)`를 사용했다.
다만 한 planning callback에는 plan/P0/safe-stop escape의 복수 P3 evaluation이 합산되고,
`CANDIDATE_EVENT`에는 evaluator별 ego/obstacle lineage key가 없다. 따라서 detached frozen-tag
capture로 동일 후보의 **반환 path-digest 집합이 정확히 일치**할 때만 oracle을 실행했다.
E02와 E04는 Jaccard 1.0 exact parity, 나머지는 parity 미확립이므로 탐색하지 않았다.

## Frozen family와 탐색 계약

- Frozen footprint: length `0.56 m`, half-width `0.15 m`, wall margin `0.04 m`, obstacle
  safety margin `0.08 m`.
- Knot 구조는 `[z0,z1,z2,z3,z4]`, offset은
  `[d_ego,d_target,d_mid,d_target,0]` 그대로다. Station 생성은
  `p3_shadow.cpp:668-698`, C2 quintic reconstruction은 `p3_shadow.cpp:762-786`, 실제 candidate
  reconstruction/validation 연결은 `p3_shadow.cpp:1203-1309`를 그대로 사용했다.
- Rotated rectangular footprint와 physical track projection은
  `raceline_spline_planner.cpp:2284-2345`의 production validator를 사용했다.
- `d_target`: exact strict/relaxed side-domain의 두 endpoint를 정렬한 닫힌 구간. 이벤트별
  수치는 `oracle_summary.csv`의 JSON field에 있다.
- `d_mid`: frozen `maximum_target_offset_m=1.5 m`에 맞춰 `[-1.5,1.5] m`.
- Coarse grid: `d_target`, `d_mid` 모두 `0.05 m`; 구간 endpoint와 production anchors 포함.
- Refinement: normalized hard-violation score가 작은 coarse seed 30개 각각의 `+-0.04 m`
  neighborhood를 `0.01 m`로 재탐색.
- Side: exact context에서 valid인 LEFT/RIGHT. Gate: STRICT/RELAXED.
- Entry/exit: 해당 callback에서 관측된 frozen production pair만 사용했다. 새로운 template이나
  entry/exit semantics를 만들지 않았다.
- Sampling은 deterministic이며 각 candidate는 production `makeC2Profile`, geometry/speed
  finalization, `validateP3ShadowPath`를 통과했다. Knot order가 성립하지 않는 request는
  production과 같은 `non-positive quintic-Hermite segment` construction guard로 분리했다.

`oracle_candidates.csv`는 bounded equivalent다. 모든 ANCHOR, 모든 REFINE, 이벤트별
near-valid score 상위 500 COARSE row를 보존한다. 전체 count와 분포는 `oracle_summary.csv`, 전체
run context/nearest-invalid는 `oracle_run_manifest.json`에 보존했다.

## 이벤트 결과

### E02

Coarse 57,066 + refinement 2,140 = **59,206** paths가 모두 constructed/validated되었고
hard-valid는 0이다. 전부 `footprint_track_bound`가 첫 hard failure이고 전부 path waypoint 0에서
실패했다. sampled domain의 가장 큰 footprint-track margin도
`-0.076929 m`로 음수다. Near-valid multi-constraint score가 가장 작은
후보는 `(d_target,d_mid)=(0.36,0.11) m`, footprint margin `-0.078545 m`였다. 즉 이 snapshot은
production mapping이 특정 root를 놓친 증거가 아니라, frozen footprint가 P3 경로의 시작부터
track validation을 통과하지 못하는 증거다.

### E04

23,344 requests 중 **9824**개가 construct/validate되었고
13520개는 knot-order construction guard로 path가 되지 않았다. Validated
path의 hard-valid는 0이며 footprint-track margin의 최대값도
`-0.040614 m`다. First failure는 `footprint_track_bound` 9,752개와
left-control-steering curvature 72개이며, 9,680개는 waypoint 0에서 실패했다. Near-valid
후보 `(d_target,d_mid)=(-0.438802,-0.45) m`의 footprint margin은 `-0.047997 m`였다.

이 footprint 결과는 **frozen replay footprint**에 대한 것이며, historical planner가 당시 어떤
footprint를 사용했는지에 대한 주장이 아니다.

### E01 / E03 / E05

Production callback에는 각각 candidate가 있었지만 exact sub-evaluation snapshot을 현재 schema로
직접 join할 수 없었다. Callback 끝의 latest obstacle/ego 또는 nearby replay capture를 넣었을 때
반환 path-digest 집합이 baseline과 일치하지 않았다. 따라서 다른 입력으로 oracle을 돌려 결론을
만드는 대신 `ORACLE_INCONCLUSIVE`로 중단했다. 이는 P3 family infeasibility나 mapping miss의
증거가 아니다.

## 요청된 GO/NO-GO 질문

1. **5개 중 hard-valid oracle solution:** 0개.
2. **Existing P3 family 안에서 infeasible로 보이는 것:** exact parity가 있는 2개(E02, E04).
   분류명은 실험 계약대로 `NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN`이며 continuous-domain 수학적
   완전성 증명으로 확대하지 않는다.
3. **Inconclusive:** 3개(E01, E03, E05). 이유는 search 해상도가 아니라 evaluator 입력 lineage
   parity 미확립이다.
4. **Oracle-success에서 production이 놓친 region:** success가 없으므로 없음.
5. **Mapping이 family expressiveness보다 병목이라는 반복 증거:** 없음. 판정 가능한 두 건은
   오히려 넓힌 `(d_target,d_mid)` domain에서도 frozen footprint validation을 못 통과했다.
6. **`s_probe`/`d_probe` mapping 연구 진행 근거:** 이 5건만으로는 **NO-GO**. 먼저
   evaluator-level snapshot ID/ego/obstacle lineage를 instrumentation에 추가하거나, exact parity를
   더 많이 확보한 뒤 별도 pilot을 해야 한다. 이 제안은 새 mapping 구현 승인이 아니다.

## 산출물 안내

- `event_manifest.csv`: exact event, localization coverage, provenance, frozen footprint,
  callback-global production count, probe/mapping values, parity.
- `oracle_summary.csv`: domain/resolution/request/constructed/validator/runtime/classification.
- `oracle_candidates.csv`: bounded candidate-level evidence.
- `production_vs_oracle.csv`: production mapping 범위와 oracle outcome 비교.
- `nearest_invalid_summary.csv`: exact-parity 이벤트의 near-invalid top 10.
- `plots/`: 이벤트별 path/feasible-region 그림. Inconclusive 이벤트는 판정 중단 이유를 표시한다.
- `paths/`: E02/E04 nearest-invalid `d(s)`/Cartesian samples.
- `method_manifest.csv`, `oracle_run_manifest.json`: source/input/harness hashes와 full run metadata.

Production planner, mapping, parameter에는 변경이 없고 commit/push도 수행하지 않았다.
