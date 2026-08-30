# P3 Oracle Pilot v2

## 결론

`research_instrumentation_v2` frozen baseline의 준비 단계 rank 1–9를 결과와 무관하게 모두
실행했다. 9개 evaluator input 모두 exact instrumentation-v2 lineage join, 전체 constructed
candidate digest multiset, production-returned digest multiset, production anchor의 hard-valid
판정·first failure·hard margin 재구성 parity를 통과했다.

9건 모두 결론 가능했고, 4건에서 frozen production은 hard-valid 후보가 0이지만 동일한 기존
P3 family의 deterministic oracle은 hard-valid 경로를 찾았다. 이 4건에만
`PILOT_EVIDENCE_OF_MAPPING_MISS`를 부여한다. 이는 pilot evidence이며 최종
`MAPPING_MISSED_EXISTING_P3` dataset label이 아니다. 나머지 5건은
`NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN`이고, finite grid 밖에도 valid P3가 없다는 뜻은 아니다.

## Frozen input 및 parity gate

- baseline tag/commit: `research_instrumentation_v2` /
  `80ae205fd470f537bd1e5449fb906c5548f6653a`
- source/config aggregate SHA-256:
  `55a078d810ad39551a491e04a84b559394873cbf01319195a996f2cde2100521` /
  `6009b3211a50dce7159b95553b7dbea1417eb20b50dbc6fb716947973a2e334a`
- exact lineage: callback/evaluation role/pass, 네 snapshot ID, source stamp, obstacle sequence,
  source epoch, reference generation이 preparation row와 v2 evaluation/candidate stream에서 일치.
- reconstruction: evaluator의 exact core obstacle geometry와 해당 reference snapshot을 사용했다.
  64개 production constructed candidate 전부의 path digest, 반환 여부, hard-valid, first failure,
  center/footprint/obstacle/slope/curvature/rate margin이 일치했다.
- full `Obstacle.msg` raw field를 event JSON에서 다시 hash했다고 주장하지 않는다. 그 exact
  identity는 v2 snapshot ID가 보존하고, offline evaluator-relevant reconstruction은 모든
  production candidate의 digest/verdict/margin parity로 별도 검증했다.
- 9건 모두 localization label은 `LOC_UNKNOWN`이다. V2E08은 제외하지 않고
  `LOCALIZATION_STRESS_CANDIDATE` provenance를 유지했다.

상세 증거는 `lineage_parity.csv`, `production_anchor_parity.csv`, `inputs/`에 있다. production
source/config/parameter는 수정하지 않았고, `/tmp` detached v2 source에 audit-only public adapter와
출력 harness만 추가해 production construction/finalization/validator를 호출했다.

## Oracle domain

v1과 동일한 deterministic broad search다.

1. 각 event에서 production `inspectP3OracleContext()`가 허용한 모든 STRICT/RELAXED 및
   LEFT/RIGHT side domain을 사용한다.
2. `d_target`은 각 side domain의 양 끝과 0.05 m 격자, production target anchor를 사용한다.
3. `d_mid`는 `[-1.5,1.5] m`의 0.05 m 격자와 production mid anchor를 사용한다.
4. `(entry_scale,exit_scale)`은 그 event에서 production이 실제 사용한 pair만 허용한다. 새 path
   family나 새 transition scale을 넣지 않았다.
5. coarse violation score 상위 30점을 중심으로 `d_target,d_mid` 각각 ±0.04 m, 0.01 m 격자를
   refine한다.
6. 모든 구성 경로는 frozen production spline reconstruction, geometry recomputation,
   `validateCandidate()`를 그대로 통과한다. best valid는 기존 7-key production rank 순서를
   사용한다.

정확한 event별 target domain, transition pair, request 수는 `search_domain.json`, 방법 계약은
`method_manifest.csv`에 있다. `total requests`는 construction guard에서 경로가 만들어지지 않은
요청도 포함하며, `constructed paths`와 `validator executions`는 실제 실행 수다.

## Event별 결과

`prod C/R/V`는 production constructed/returned/hard-valid, `oracle Q/P/V/H`는
request/constructed-path/validator-execution/hard-valid-row다. runtime은 release audit harness의
coarse+refine search wall time이며 로그 parsing/plot 시간을 포함하지 않는다.

| event | bag | elapsed s | callback/eval | ego `(s,d,v)` m | prod C/R/V | wp0 footprint margin m | oracle Q/P/V/H | runtime s | classification |
|---|---|---:|---|---|---:|---:|---:|---:|---|
| V2E01 | rosbag2_2026_08_24-19_24_17 | 721.860 | 28019/2 | `(5.827,-0.359,0.183)` | 9/3/0 | +0.4131 | 30346/2596/2596/244 | 0.602 | ORACLE_VALID_P3_EXISTS |
| V2E02 | rosbag2_2026_08_24-19_00_30 | 920.020 | 33976/2 | `(14.502,+0.050,0.000)` | 3/3/0 | +0.6309 | 9858/3810/3810/610 | 0.784 | ORACLE_VALID_P3_EXISTS |
| V2E03 | rosbag2_2026_08_25-09_59_22 | 157.790 | 6394/2 | `(23.434,-0.081,0.000)` | 9/3/0 | +0.2646 | 22820/5140/5140/14 | 1.261 | ORACLE_VALID_P3_EXISTS |
| V2E04 | rosbag2_2026_08_25-09_59_22 | 671.170 | 25383/2 | `(8.335,+1.123,0.000)` | 6/6/0 | +0.0192 | 15208/15208/15208/0 | 3.377 | NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN |
| V2E05 | rosbag2_2026_08_24-19_00_30 | 577.840 | 20689/2 | `(2.076,-0.818,0.000)` | 2/2/0 | +0.0289 | 7716/6724/6724/0 | 0.536 | NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN |
| V2E06 | rosbag2_2026_08_24-19_00_30 | 786.320 | 28885/2 | `(34.226,-0.480,0.000)` | 22/16/0 | +0.4644 | 52232/52232/52232/0 | 10.119 | NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN |
| V2E07 | rosbag2_2026_08_24-19_00_30 | 552.200 | 19683/2 | `(0.696,+0.010,0.000)` | 2/2/0 | +0.6742 | 6474/3994/3994/0 | 0.428 | NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN |
| V2E08 | rosbag2_2026_08_25-09_23_32 | 6.410 | 358/2 | `(14.697,-0.008,0.000)` | 9/3/0 | +0.5698 | 17906/8402/8402/1449 | 1.083 | ORACLE_VALID_P3_EXISTS |
| V2E09 | rosbag2_2026_08_25-09_59_22 | 538.290 | 20095/2 | `(43.810,+0.022,0.281)` | 2/2/0 | +0.5304 | 5628/2404/2404/0 | 0.197 | NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN |

합계는 168,188 requests, 100,510 constructed paths, 100,510 validator executions,
2,317 hard-valid rows, search wall time 18.388 s다.

### Exact obstacle geometry

| event | evaluator obstacle summary |
|---|---|
| V2E01 | id145 `s=[5.969,5.992], c=5.980, d=[-0.227,-0.208]` |
| V2E02 | id228 `s=[14.685,14.902], c=14.794, d=[-0.331,-0.212]` |
| V2E03 | id54 `s=[23.783,23.896], c=23.840, d=[-0.243,-0.045]` |
| V2E04 | id445 `s=[20.889,21.111], c=21.000, d=[-0.690,-0.191]` |
| V2E05 | id79 `s=[1.165,1.261], d=[+0.257,+0.307]`; id104 `s=[2.639,3.085], d=[+0.100,+0.502]` |
| V2E06 | id189 `s=[34.342,34.371], d=[-0.334,-0.306]`; id192 `s=[36.816,37.914], d=[+0.271,+1.047]`; id185 `s=[38.836,38.850], d=[-0.121,-0.108]` |
| V2E07 | id79 `s=[0.913,0.930], d=[+0.057,+0.080]`; id85 `s=[1.019,1.370], d=[-0.671,-0.337]`; repeated id79 `s=[3.669,3.870], d=[+0.141,+0.336]` |
| V2E08 | id110 `s=[14.682,15.086], c=14.884, d=[+0.228,+0.809]` |
| V2E09 | id319 `s=[44.090,44.294], c=44.192, d=[+0.197,+0.433]` |

### Production P3 parameters

아래는 exact candidate rows의 unique 요약이다. 반올림 전 값, stage/template/root/rule,
candidate별 `(d_target,d_mid,s_probe,d_probe,entry,exit)`는 `production_candidates.csv`와
`oracle_summary.csv`의 `production_candidate_parameters`에 보존했다. `none`은 ZERO_INTERFACE
closure 등 probe가 없는 production candidate다.

| event | unique production `d_target` | unique production `d_mid` | unique `(s_probe,d_probe)` |
|---|---|---|---|
| V2E01 | -0.456880, -0.463764 | -0.536880, -0.463764, -0.677190 | (0.153068,-0.536880), (0.153068,-0.677190) |
| V2E02 | +0.160312, +0.150000 | +0.160312, +0.414194 | (0.291567,+0.414194) |
| V2E03 | +0.185335, +0.199587 | +0.265335, +0.199587, +0.739982 | (0.405122,+0.265335), (0.388574,+0.641417) |
| V2E04 | +0.161289, +0.330625, +0.511250 | same as target | none |
| V2E05 | -0.162266 | -0.162266 | none |
| V2E06 | +0.150000, -0.355117, -0.409851, -0.468234, +0.160898, +0.324375, +0.498750, -0.585000, +0.847500 | +0.206416/+0.206433/+0.206434, -0.355117, -0.409851, -0.468234, +0.160898, +0.324375, +0.498750, +0.484559 | (4.618363,+0.201617), (4.616943,+0.484559), (4.616943,-0.468234) |
| V2E07 | +0.320764 | +0.320764 | none |
| V2E08 | -0.150000, -0.159336 | -0.150002/-0.150000, -0.159336, -0.552723 | (0.351274,-0.150000), (0.351274,-0.374868) |
| V2E09 | -0.159727 | -0.159727 | none |

## Oracle-success details

`track`은 minimum footprint-track margin이고 center-track은 별도 diagnostic이다. `parameter
distance`는 nearest production candidate와 raw 4-D Euclidean
`(d_target,d_mid,entry_scale,exit_scale)` / 2-D Euclidean `(d_target,d_mid)` 순서다. 서로 단위가
다른 4-D raw distance를 물리 거리로 해석하면 안 된다.

| event | gate/side | best target | best mid | entry/exit | track | obstacle | slope | curvature | rate | braking deficit | normalized slack | parameter distance 4-D/2-D | relation |
|---|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| V2E01 | STRICT/RIGHT | -0.456880 | -0.463764 | 0.514581/7.397546 | +0.1560 | +1.5000 | +0.7161 | +0.6973 | +19.2250 | 0.0000 | +0.1046 | 0.0731/0.0731 | SAME_D_TARGET_MISSED_D_MID |
| V2E02 | STRICT/LEFT | +0.150000 | +0.110000 | 1.310934/6.630707 | +0.0832 | +0.0917 | +0.5937 | +0.3241 | +18.2229 | 0.0000 | +0.0611 | 0.3042/0.3042 | SAME_D_TARGET_MISSED_D_MID |
| V2E03 | STRICT/LEFT | +0.200000 | +0.190000 | 0.514581/6.620728 | +0.3180 | +0.0068 | +0.0255 | +0.2311 | +14.4592 | 0.0000 | +0.0046 | 0.0767/0.0767 | SAME_D_TARGET_AND_D_MID_OTHER_P3_DOF |
| V2E08 | STRICT/RIGHT | -0.160000 | -0.120000 | 0.514581/7.397546 | +0.4100 | +0.0489 | +0.4549 | +0.0773 | +18.1075 | 0.0000 | +0.0326 | 0.0316/0.0316 | SAME_D_TARGET_MISSED_D_MID |

V2E01/V2E02/V2E08에서는 oracle valid set 안에 production과 **같은 `d_target` 및 같은
entry/exit scale**을 쓰면서 production이 만들지 않은 `d_mid`에서 valid가 되는 점이 실제로
있다. 따라서 세 event의 production failure는 P3 family 부재가 아니라 probe/root mapping이
선택한 `d_mid` coverage와 직접 관련된 반복 pilot evidence다. 다만 이 실험은 `d_mid`를 직접
grid search했으므로 `s_probe`와 `d_probe` 중 어느 요소가 원인인지는 분리하지 않는다.

V2E03은 다른 경우다. valid set에 production의 exact `d_target=d_mid=0.1995871`이 존재하지만,
production은 그 tuple을 short exit `0.497168`과만 결합했다. oracle valid는 이미 production에
존재하던 long exit `6.620728` pair와의 재조합에서 나온다. 따라서 이 event를
`(s_probe,d_probe)->d_mid` miss로 세지 않고 transition-template/tuple association bottleneck으로
분류했다.

각 success의 path/feasible-region plot:

- V2E01: `plots/V2E01_best_valid_path.png`, `plots/V2E01_feasible_region.png`
- V2E02: `plots/V2E02_best_valid_path.png`, `plots/V2E02_feasible_region.png`
- V2E03: `plots/V2E03_best_valid_path.png`, `plots/V2E03_feasible_region.png`
- V2E08: `plots/V2E08_best_valid_path.png`, `plots/V2E08_feasible_region.png`

정확한 best hard margins와 nearest production tuple은 `oracle_summary.csv`, 모든 2,317 valid
rows는 `oracle_valid_candidates.csv`, path samples는 `paths/`에 있다.

## No-valid events와 first failures

- V2E04: 15,208/15,208 constructed paths가 모두 `footprint_track_bound` 첫 실패다.
- V2E06: 52,232/52,232가 모두 inflated static-obstacle collision 첫 실패다.
- V2E05: footprint 5,012가 지배적이며 non-positive construction 992, left curvature 762가 뒤를
  잇는다.
- V2E07: non-positive construction 2,480, slope 2,456, left/right curvature가 주 병목이다.
- V2E09: non-positive construction 3,224, left curvature 1,748, slope 358이 주 병목이다.

전체 reason/count는 `oracle_first_failure_distribution.csv`, normalized violation score 기준 event별
근접 invalid 10개는 `nearest_invalid_candidates.csv`에 있다. 특히 V2E04와 V2E06은 현재 finite
domain에서 각각 footprint/corridor와 obstacle geometry가 우세하므로, 모든 failure를 mapping
miss로 일반화하면 안 된다.

## Aggregate A–F

A. Conclusive events: **9/9**. `ORACLE_INCONCLUSIVE`는 0건이다.

B. Oracle success / conclusive event ratio: **4/9 = 44.4%**.

C. Success는 **4건이고 네 bag 모두**에 하나씩 존재하며, 두 reference snapshot domain
(`ref_3f5d...`, `ref_4dc...`)에 걸친다. V2E08 stress candidate를 빼도 두 08-24 bag과 09:59 bag,
즉 세 bag에 success가 남는다.

D. Success 4건 중 요청된 분류는 `same d_target / missed d_mid` **3**, `different d_target needed`
**0**, `both differ` **0**이다. 나머지 **1**은 `same d_target and d_mid / other P3 DOF`이며
V2E03의 transition-scale association이다. 분류는 best 한 점이 아니라 전체 valid set과 exact
production tuple을 비교했다.

E. **반복 evidence는 `(s_probe,d_probe)->d_mid` root-mapping 연구를 지지한다.** 세 event,
세 bag, 두 reference domain에서 동일 target/transition scales에 다른 mid가 valid였다. 그러나
현재 pilot은 `s_probe` 대 `d_probe`의 개별 책임을 식별하지 않으며 새 mapping을 제안하거나
구현하지 않았다.

F. 동시에 다른 병목도 명확하다. V2E03은 transition-template association, V2E04는
footprint/track corridor, V2E06은 obstacle/corridor collision이 우세하다. V2E05/V2E07/V2E09는
construction positivity, slope, curvature, footprint가 섞인다. 따라서 결과는 “mapping만 고치면
모든 failure가 해결된다”거나 “no-valid 5건은 P3 family가 수학적으로 표현 불가능하다”는
증거가 아니다.

## Artifacts

- `oracle_summary.csv`: event별 production/oracle count, best valid, hard margins, nearest tuple,
  classification/evidence.
- `lineage_parity.csv`, `production_anchor_parity.csv`: acceptance gates.
- `production_candidates.csv`: exact production P3 candidate parameters/probes.
- `oracle_valid_candidates.csv`: 모든 hard-valid oracle rows.
- `nearest_invalid_candidates.csv`, `oracle_first_failure_distribution.csv`: no-valid evidence.
- `search_domain.json`, `method_manifest.csv`, `oracle_run_manifest.json`: 재현 계약과 SHA.
- `inputs/`, `paths/`, `plots/`: exact evaluator reconstruction input, best paths, requested plots.

P3 family, spline construction, production mapping, validator, ranking, vehicle parameter,
lifecycle, speed shaping은 변경하지 않았다. 새 mapping도 구현하지 않았다.
