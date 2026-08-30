# FIRST REAL-ROSBAG BASELINE AUDIT v1

## 범위와 비개입 계약

- Baseline tag/commit: `research_instrumentation_v1` / `6ba46dbe90139d184c066740a3cbbf191d41cc61`
- Source/config aggregate SHA: `09fb8c4980b277d9d0ee07b6337a8d4797f77a252616611c27745e15157d135b` / `6009b3211a50dce7159b95553b7dbea1417eb20b50dbc6fb716947973a2e334a`
- Read-only localization extractor: `tools/planning_research/rosbag_localization_audit/extract_localization_features.py`, SHA-256 `3257c0132e60b293f00c59b37c026e761cc40c623e17245b7c822b1cc2df7dbb`
- Production YAML과 `p3_mode=TEST_ACTIVE`를 그대로 사용했습니다.
- replay input은 기록된 `/global_waypoints`, `/confirmed_static_obs`, `/static_obs`, `/car_state/frenet/odom`, `/state`만 1.0x로 재생했습니다.
- `s_probe`, `d_probe`, `d_target`, spline/solver/generator/validator/ranker/speed/lifecycle/safe-stop/perception/localization 코드는 변경하지 않았습니다.
- 네 recording의 장소는 확정하지 않고 `USER_RECOLLECTION_COMPETITION_SITE` metadata로만 남겼습니다.
- P3 oracle과 새 mapping은 실행하지 않았습니다.
- 녹화 당시 historical source는 네 bag 모두 exact commit이 확인되지 않았습니다. 이 문서의
  planner 통계는 historical `/avoid_waypoints`를 재집계한 값이 아니라 **frozen-baseline replay
  behavior**입니다. 상세 footprint history, bag별 config/schema 증거와 claim boundary는
  [`provenance_audit.md`](provenance_audit.md)를 따릅니다.

## Replay 절차

1. 각 MCAP SHA-256을 직접 계산하고 metadata message/topic/file 합을 검사했습니다.
2. 기존 localization extractor가 각 MCAP을 EOF까지 `SequentialReader` READ_ONLY로 읽었습니다.
3. 8/25 09:23 bag의 첫 20초 smoke에서 planning 940, candidate 248, drop 0을 확인했습니다.
4. tag의 clean detached worktree에서 source/config provenance를 만들고 exact tag source를 isolated build했습니다.
5. 네 bag을 각각 별도 ROS domain에서 1.0x 전체 재생하고 clean shutdown 후 JSONL을 집계했습니다.
6. startup/pre-clock과 마지막 frozen-clock tail은 baseline denominator에서 제외했습니다. `ready_for_baseline`은 global reference, ego Frenet state, obstacle source를 모두 받은 event입니다.

`blocking_context`는 explicit field가 baseline schema에 없으므로 constructed candidate, active/fresh/continued lifecycle, safe-stop, 또는 non-benign P3 failure가 있는 ready callback의 operational proxy입니다. 이는 obstacle message 존재 횟수와 동일하지 않습니다.
모든 event count는 callback 단위이며, 같은 failure/safe-stop episode의 반복 callback을 독립 episode로 해석하지 않습니다. Episode 성격의 변화량은 `temporal_stability.csv`에 별도로 기록했습니다.

## Corpus integrity/capability

| Bag | User-provided SHA match | Duration [s] | Messages | Current replay inputs | Full replay |
|---|---:|---:|---:|---:|---:|
| `rosbag2_2026_08_24-19_00_30` | 1 | 944.197 | 967747 | 1 | 1 |
| `rosbag2_2026_08_24-19_24_17` | 1 | 769.070 | 775242 | 1 | 1 |
| `rosbag2_2026_08_25-09_23_32` | 1 | 53.798 | 59314 | 1 | 1 |
| `rosbag2_2026_08_25-09_59_22` | N/A | 1117.075 | 1126487 | 1 | 1 |

네 번째 bag은 지정된 `rosbags_original/recent` 원본만 사용했습니다. 사용자가 비교 기준 SHA를 제공하지 않았으므로 match는 N/A이며, 이번 audit에서 계산한 SHA-256은 `270ab6852ae7ddae137da712fc51ef774818a58e51defe64422fc9805928df5e`입니다. metadata-only `/home/sungho/rosbags` copy는 열지 않았습니다.

### Provenance interpretation

네 bag의 historical source status는 모두 `SOURCE_VERSION_PARTIALLY_INFERRED`다. Common wire
schema와 P3 diagnostic family는 확인되지만 Git SHA는 기록되지 않았다. 특히 첫 bag은 녹화 중
planner startup config가 `safety_margin_m=0.000`에서 `0.050`으로 바뀌었고, 둘째 bag은
`0.080`을 기록했다. 셋째 bag에는 planner startup config가 없으며, 넷째 bag은 녹화 말미의
한 startup만 `0.080`을 확인한다. Bag별 상세 field는 `corpus_provenance.csv`에 있다.

Frozen replay는 이 historical config를 재사용하지 않았다. 네 run 모두 exact tag의 effective
snapshot `vehicle_length=0.560`, `vehicle_half_width=0.150`, `safety_margin=0.080`,
`wall_safety_margin=0.040`, reserve off를 사용했다. 따라서 아래 failure distribution은 녹화 당시
옛 footprint의 결과가 아니라 이 frozen footprint의 결과다.

## Planner descriptive baseline

| Bag | Ready events | Blocking proxy | Fresh | Continued | Constructed actual | Validator actual | Hard-valid | No-candidate | Safe-stop entries |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `rosbag2_2026_08_24-19_00_30` | 27759 | 16124 | 293 | 160 | 82179 | 127455 | 7105 | 4042 | 164 |
| `rosbag2_2026_08_24-19_24_17` | 29825 | 13270 | 534 | 681 | 55512 | 77488 | 9353 | 1390 | 91 |
| `rosbag2_2026_08_25-09_23_32` | 2030 | 196 | 0 | 0 | 200 | 261 | 0 | 5 | 1 |
| `rosbag2_2026_08_25-09_59_22` | 43180 | 18689 | 634 | 365 | 74993 | 109503 | 9819 | 1703 | 101 |

전체 ready callback 102,794개 중 blocking-context proxy는 48,279개입니다. actual constructed=212,884, actual validator entry=314,707, hard-valid return=26,277, no-candidate=7,140입니다. Retained cap 24는 **한 evaluator가 반환하는 `P3ShadowResult`의 cap**이지 callback-global cap이 아닙니다. 여러 evaluator와 lifecycle/safe-stop validation을 합친 실제 callback 최대치는 constructed 294, `validateCandidate()` entry 422, callback-global returned count 294였습니다.

## Runtime summary

단위는 wall-clock microseconds이며 instrumentation/replay 환경에서 측정한 descriptive 값입니다.

| Bag | Metric | p50 [us] | p95 [us] | p99 [us] | Max [us] |
|---|---|---:|---:|---:|---:|
| `rosbag2_2026_08_24-19_00_30` | `callback_total` | 181.064 | 22406.839 | 81895.586 | 514925.308 |
| `rosbag2_2026_08_24-19_00_30` | `planning_total` | 0.000 | 19655.479 | 76724.459 | 475875.199 |
| `rosbag2_2026_08_24-19_24_17` | `callback_total` | 165.862 | 13443.244 | 45620.157 | 421584.932 |
| `rosbag2_2026_08_24-19_24_17` | `planning_total` | 0.000 | 10915.278 | 41083.570 | 379995.045 |
| `rosbag2_2026_08_25-09_23_32` | `callback_total` | 108.249 | 437.656 | 525.015 | 54504.875 |
| `rosbag2_2026_08_25-09_23_32` | `planning_total` | 1.220 | 4.330 | 8.096 | 52561.522 |
| `rosbag2_2026_08_25-09_59_22` | `callback_total` | 188.192 | 6580.809 | 52032.599 | 349391.388 |
| `rosbag2_2026_08_25-09_59_22` | `planning_total` | 0.000 | 5524.593 | 46451.124 | 308159.443 |

## Stage usage

| Stage | Invocation proxy | Events with construction | Constructed | Hard-valid | Published fresh selections |
|---|---:|---:|---:|---:|---:|
| M0_V1 | 63153 | 5034 | 85001 | 8691 | 686 |
| M0_V2 | 2129 | 2129 | 50959 | 3327 | 441 |
| M1 | 4931 | 4931 | 76924 | 1640 | 334 |

M0-V1 invocation proxy는 strict+relaxed evaluation-pass count입니다. M0-V2/M1은 schema에 zero-construction invocation flag가 없으므로 ‘후보가 하나 이상 구성된 callback’ 하한만 보고합니다.

## First validation failures

| First failure | Reason | Candidate count |
|---|---|---:|
| `TRACK_BOUNDARY` | `footprint_track_bound` | 107946 |
| `TRACK_BOUNDARY` | `d-offset leaves the global waypoint track bounds` | 32800 |
| `GEOMETRY` | `shifted race line exceeds left control steering curvature` | 23021 |
| `GEOMETRY` | `shifted race line exceeds right control steering curvature` | 14635 |
| `GEOMETRY` | `quintic d-offset exceeds maximum_lateral_slope` | 12911 |
| `OBSTACLE_COLLISION` | `d-offset intersects an inflated static-obstacle box` | 7675 |
| `GEOMETRY` | `path entry is discontinuous from the current ego d` | 238 |

분모는 `validator_executed=true`이면서 hard-valid가 아닌 CANDIDATE_EVENT입니다. 후보별 최초 실패만 계수하므로 callback no-candidate나 safe-stop entry 횟수와 같은 통계가 아닙니다.
또한 callback-global `validate_candidate_executed_total_actual`에는 retention/lifecycle/safe-stop escape validator가 포함되지만, 이들은 새 CANDIDATE_EVENT를 만들지 않으므로 위 failure-reason 분포에는 포함되지 않고 실행 횟수에만 포함됩니다.

`footprint_track_bound`는 frozen tag의 `0.56 x 0.30 m` rotated rectangle을 physical track
boundary에 projection하고 wall margin `0.04 m`를 한 번 적용한 결과다. Historical planner의
unknown/older footprint config 때문에 생겼다고 해석하지 않는다. Obstacle safety margin
`0.08 m`도 이 wall predicate에는 들어가지 않는다.

## Temporal stability

| Bag | Path identity changes | Side changes | Invalidations | Safe-stop entries/exits | Global handoff entries |
|---|---:|---:|---:|---:|---:|
| `rosbag2_2026_08_24-19_00_30` | 12160 | 62 | 174 | 164/164 | 512 |
| `rosbag2_2026_08_24-19_24_17` | 14939 | 85 | 355 | 91/91 | 621 |
| `rosbag2_2026_08_25-09_23_32` | 4 | 0 | 0 | 1/1 | 3 |
| `rosbag2_2026_08_25-09_59_22` | 14747 | 64 | 398 | 101/100 | 1120 |

전체 invalidation 927회 중 `STALE_SOURCE` 616회, `FRENET_ODOMETRY_STALE` 287회, `WRAP_OR_BACKWARD_PROGRESS` 14회이며 나머지 exact reason은 `temporal_stability.csv`의 JSON field에 보존했습니다.

## Join 검증

- CANDIDATE_EVENT exact `(run_id, callback_sequence)` join: 212,884 matched, 0 unmatched.
- PLANNING_EVENT → nearest localization feature 최대 bag별 p99 offset: 82.913583 s.
- 전체 planning event 중 nearest localization offset ≤50 ms 비율: 0.751268. 큰 offset은 pose가 없는 leading/trailing/gap 구간이며 가까운 것처럼 보간하지 않았습니다.
- Logger drop: 15; full replay clean shutdown: 4/4.
- 모든 localization label은 `LOC_UNKNOWN`; empirical percentile은 bag 내부 수동 검토 순위이며 품질 threshold가 아닙니다.

## Localization temporal availability

| Bag | Leading unavailable [s] | Trailing unavailable [s] | Pose span / bag | Max pose sample gap [s] |
|---|---:|---:|---:|---:|
| `rosbag2_2026_08_24-19_00_30` | 0.470 | 0.016 | 0.999486 | 8.920 |
| `rosbag2_2026_08_24-19_24_17` | 62.388 | 0.001 | 0.918878 | 22.523 |
| `rosbag2_2026_08_25-09_23_32` | 0.061 | 0.020 | 0.998484 | 0.051 |
| `rosbag2_2026_08_25-09_59_22` | 65.449 | 90.173 | 0.860688 | 4.366 |

따라서 candidate↔planning exact join과 달리 localization join은 모든 callback에서 동시 관측을 보장하지 않습니다. offset을 그대로 보존하고 coverage 밖 event를 quality evidence로 사용하지 않아야 합니다.

## Top-ranked localization windows

### rosbag2_2026_08_24-19_00_30

- elapsed `942.068s`, ns `1787566572677944563`: top metric `abs_d_jump_m`, mean/max empirical percentile 0.9583/0.9977.
- elapsed `630.947s`, ns `1787566261556871755`: top metric `backward_s_jump_m`, mean/max empirical percentile 0.9364/0.9937.
- elapsed `624.505s`, ns `1787566255114844537`: top metric `abs_d_jump_m`, mean/max empirical percentile 0.9346/0.9948.

### rosbag2_2026_08_24-19_24_17

- elapsed `199.793s`, ns `1787567319921359101`: top metric `abs_d_jump_m`, mean/max empirical percentile 0.9654/0.9977.
- elapsed `308.095s`, ns `1787567428222892061`: top metric `localization_minus_imu_yaw_rate_radps`, mean/max empirical percentile 0.9492/0.9964.
- elapsed `34.943s`, ns `1787567155071621475`: top metric `backward_s_jump_m`, mean/max empirical percentile 0.9425/0.9977.

### rosbag2_2026_08_25-09_23_32

- elapsed `0.000s`, ns `1787617412472255573`: top metric `scan_pose_offset_s`, mean/max empirical percentile 0.9923/1.0000.
- elapsed `1.000s`, ns `1787617413472696030`: top metric `projection_branch_jump_proxy_m`, mean/max empirical percentile 0.9461/0.9995.
- elapsed `46.634s`, ns `1787617459105983403`: top metric `distance_jump_m`, mean/max empirical percentile 0.9252/0.9977.

### rosbag2_2026_08_25-09_59_22

- elapsed `546.835s`, ns `1787620174509706623`: top metric `backward_s_jump_m`, mean/max empirical percentile 0.9737/0.9978.
- elapsed `395.525s`, ns `1787620023199773614`: top metric `abs_d_jump_m`, mean/max empirical percentile 0.9626/0.9988.
- elapsed `74.721s`, ns `1787619702395723730`: top metric `localization_minus_imu_yaw_rate_radps`, mean/max empirical percentile 0.9467/0.9924.

선정은 각 bag의 다중 feature empirical percentile 평균 순 top-N이며, 0.5초 이내 중복 중심을 제거했습니다. 이는 suspect 우선 검토 제안이지 LOC_SUSPECT 판정이 아닙니다.

## Comparatively stable-looking planner-failure review candidates

- `rosbag2_2026_08_25-09_59_22` elapsed `525.790s`, kind `NO_CANDIDATE`, fallback `SAFE_STOP_AUTHORITY`, localization mean/max empirical percentile 0.1138/0.6734.
- `rosbag2_2026_08_25-09_59_22` elapsed `196.110s`, kind `NO_CANDIDATE`, fallback `SAFE_STOP_AUTHORITY`, localization mean/max empirical percentile 0.1159/0.6734.
- `rosbag2_2026_08_24-19_00_30` elapsed `222.080s`, kind `NO_CANDIDATE`, fallback `SAFE_STOP_AUTHORITY`, localization mean/max empirical percentile 0.1396/0.6685.
- `rosbag2_2026_08_25-09_59_22` elapsed `121.490s`, kind `NO_CANDIDATE;SAFE_STOP_ENTRY`, fallback `NO_HARD_VALID_M1_CANDIDATE`, localization mean/max empirical percentile 0.1404/0.6822.
- `rosbag2_2026_08_24-19_00_30` elapsed `923.530s`, kind `NO_CANDIDATE`, fallback `SAFE_STOP_AUTHORITY`, localization mean/max empirical percentile 0.1479/0.6685.
- `rosbag2_2026_08_24-19_00_30` elapsed `291.880s`, kind `NO_CANDIDATE;SAFE_STOP_ENTRY`, fallback `BOUNDARY_HANDOFF_UNRESOLVED`, localization mean/max empirical percentile 0.1514/0.6685.
- `rosbag2_2026_08_24-19_00_30` elapsed `673.580s`, kind `NO_CANDIDATE`, fallback `SAFE_STOP_AUTHORITY`, localization mean/max empirical percentile 0.1540/0.6685.
- `rosbag2_2026_08_24-19_24_17` elapsed `657.080s`, kind `NO_CANDIDATE`, fallback `SAFE_STOP_AUTHORITY`, localization mean/max empirical percentile 0.1584/0.6478.

이 목록은 failure-join 중 localization feature 평균 순위가 낮은 순서입니다. ‘localization과 무관’이라는 인과 판정이나 LOC_CLEAN label은 아닙니다.

## Artifacts

- `corpus_manifest.csv`: path/SHA/tag/provenance/integrity
- `corpus_provenance.csv`, `provenance_audit.md`: historical source status, footprint Git history,
  per-bag config/schema evidence, original-vs-frozen interpretation contract
- `bag_capabilities.csv`: exact topics/types/counts와 replay capability
- `raw_localization_features/*.csv`: 기존 extractor의 LOC_UNKNOWN 원행
- `localization_summary.csv`, `localization_suspect_windows.csv`: threshold-free summary/top-N windows
- `planning_events_merged.csv`: callback-global counters/runtime/candidate aggregate/localization nearest join
- `planner_baseline_summary.csv`, `validator_failure_distribution.csv`, `stage_usage.csv`
- `runtime_summary.csv`, `temporal_stability.csv`, `planner_localization_join.csv`
- `selected_example_plots/*_localization_overview.png`: bag별 10-panel overview

## 요청 질문 10개에 대한 답

1. **네 bag 모두 integrity가 정상인가?** 예. 네 MCAP 모두 EOF sequential read와 metadata topic/message/file 일관성 검사가 끝났습니다. 첫 세 bag은 사용자 제공 SHA와 일치했고, 기준 SHA가 없던 네 번째는 계산값만 기록했습니다.
2. **실제 planner replay가 가능한 bag은 몇 개인가?** capability상 4/4, 실제 full instrumented replay도 4/4 완료했습니다.
3. **Instrumentation event와 localization feature가 join되는가?** Candidate는 exact join 212,884/212,884, unmatched=0입니다. Localization은 timestamp nearest join 구조는 동작하지만 ≤50ms 비율이 0.751268이고 worst-bag p99가 82.913583s여서 temporal coverage는 불완전합니다. 큰 gap event를 동시 localization evidence로 취급하면 안 됩니다.
4. **Localization anomaly가 눈에 띄는 구간은 어디인가?** 위 top-ranked window와 `localization_suspect_windows.csv`에 timestamp range로 제시했습니다. 모두 LOC_UNKNOWN입니다.
5. **Localization과 무관해 보이는 P3 failure가 실제로 존재하는가?** [DESCRIPTIVE] frozen-baseline replay에서 상대적으로 낮은 localization empirical-rank 문맥과 겹친 no-candidate/safe-stop failure 후보는 존재합니다. 다만 현재 evidence만으로 localization 무관이라는 인과 판정은 할 수 없고, historical planner failure와도 동일시하지 않습니다.
6. **Actual candidate/validator 계산량은?** 전체 ready callback에서 constructed 212,884, validator 314,707, hard-valid 26,277입니다. 한 evaluator의 retained cap 24와 달리 callback-global 관측 최대치는 constructed 294, validator 422였습니다. bag별 callback p50/p95/p99/max는 `planner_baseline_summary.csv`에 있습니다.
7. **M0-V1/V2/M1은 얼마나 사용됐는가?** 위 stage table과 `stage_usage.csv`가 constructed/hard-valid/published-selection을 구분합니다. zero-construction invocation은 schema 한계상 M0-V2/M1에서 하한만 보고합니다.
8. **Mapping/oracle 연구를 시작할 clean-looking P3 failure가 존재하는가?** [DESCRIPTIVE] frozen-baseline replay에 수동 검토할 comparatively stable-looking failure 후보 pool은 존재합니다. LOC_CLEAN이 동결되지 않았고 historical source도 exact-known이 아니므로 ‘clean failure 확정’은 아닙니다.
9. **다음 단계로 localization policy freeze가 가능한가?** 아직 불가능합니다. 네 bag의 threshold-free feature와 수동 후보만으로 CLEAN/SUSPECT/BAD policy를 동결할 근거가 부족합니다.
10. **Mapping miss라고 주장할 수 있는가?** 아니오. 이번 단계에서는 P3 oracle이나 새 mapping을 전혀 실행하지 않았으므로 어떤 event도 mapping miss로 부르면 안 됩니다.
