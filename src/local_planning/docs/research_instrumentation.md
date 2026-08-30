# Local planner research instrumentation

## 목적과 비개입 계약

이 기능은 P3 후보 생성·검증·순위·lifecycle을 사후 연구할 수 있도록 event를 복사해
기록합니다. `research_instrumentation_enable`의 C++/YAML 기본값은 `false`입니다. OFF이면
research logger/cycle을 만들지 않으며 P3 생성, `s_probe`, `d_probe`, `d_target`, root solver,
validator, ranker, lifecycle, safe-stop, 발행 경로를 읽거나 바꾸지 않습니다.

ON일 때도 계획 코드가 research field를 의사결정 입력으로 읽지 않습니다. 각 callback에
stack-owned `PlanningResearchCycle` 하나를 만들고 planner에 non-owning pointer를 잠시
연결합니다. callback 종료 시 bounded queue에 `try_lock`으로 복사 없이 이동합니다. mutex를
즉시 얻지 못하거나 queue가 가득 차면 callback을 기다리게 하지 않고 event를 버립니다.
writer thread만 JSONL 파일을 쓰며 synchronous fsync는 없습니다. node/logger 소멸 시 남은
queue를 drain하고 stream을 flush합니다.

## Run 준비와 provenance

ON run에서 logger가 필수로 검증하는 항목은 non-empty output root와 run ID, Git commit,
source SHA, config SHA입니다. node는 빈 run ID를 단조시계 기반 ID로 자동 생성하며,
scenario ID와 clean worktree의 빈 dirty-status 문자열은 선택 사항입니다.

| Parameter | Default | 역할 |
|---|---:|---|
| `research_instrumentation_enable` | `false` | 전체 계측 gate |
| `research_all_violation_audit_enable` | `false` | first-failure 뒤 non-authoritative 전체 위반 pass |
| `research_output_root` | `/tmp/local_planning_research` | run directory의 부모 |
| `research_run_id` | empty | directory/join run key; empty이면 node가 단조시계 기반 ID 생성 |
| `research_scenario_id` | empty | 실험자가 부여하는 시나리오 key |
| `research_git_commit` | empty | 실행 source의 Git HEAD; ON에서는 필수 |
| `research_git_dirty_status` | empty | 실행 source의 porcelain status |
| `research_source_sha256` | empty | relevant source manifest aggregate; ON에서는 필수 |
| `research_config_sha256` | empty | config manifest aggregate; ON에서는 필수 |
| `research_parameter_snapshot_id` | empty | 실험 요청 snapshot ID; empty이면 effective params의 FNV-1a ID |
| `research_queue_capacity` | `128` | callback→writer bounded queue 크기 |

`tools/planning_research/prepare_instrumented_run.py`가 Git/source/config manifest와 두 번째
ROS parameter overlay를 새 디렉터리에 만듭니다. logger는 run directory가 이미 있으면
덮어쓰지 않고 실패합니다. `metadata.json`은 schema version, 위 provenance, 이름순으로
직렬화한 실제 declared/effective parameter 전체를 저장합니다.

## 파일과 join key

- `metadata.json`: run 단위 provenance와 schema `local_planning_research/1`
- `planning_events.jsonl`: callback당 `PLANNING_EVENT` 1행
- `candidate_events.jsonl`: 실제 구성된 후보당 `CANDIDATE_EVENT` 1행
- `run_summary.json`: clean shutdown 여부와 accepted/written/dropped 최종 누계

join key는 `(run_id, callback_sequence)`입니다. 한 callback에서 strict/relaxed evaluator가
둘 다 돌 수 있으므로 candidate에는 `clearance_pass`도 포함됩니다.

## PLANNING_EVENT schema

| 그룹 | 필드 |
|---|---|
| header/provenance join | `schema_version`, `event_type`, `run_id`, `scenario_id`, `callback_sequence`, `ros_time_ns`, `obstacle_source_stamp_ns`, `obstacle_sequence`, `source_epoch`, `reference_generation`, `git_commit`, `source_sha256`, `config_sha256`, `effective_parameter_snapshot_id` |
| ego | `ego.x`, `ego.y`, `ego.yaw`, `ego.pose_source`, `ego.s`, `ego.d`, `ego.speed_mps` |
| lifecycle | `p3_mode`, `lifecycle_state`, `lifecycle_owner`, `continuation_result`, `invalidation_reason`, `selected_side`, `selected_candidate_identity`, `selected_path_digest`, `fallback_reason`, `active`, `continued`, `fresh`, `backup`, `safe_stop`, `global_handoff` |
| callback-global counters | `strict_attempted`, `relaxed_attempted`, `m0_v1_constructed_right`, `m0_v1_constructed_left`, `discarded_side_candidate_count`, `m0_v2_constructed`, `m1_constructed`, `constructed_total_actual`, `validate_candidate_executed_total_actual`, `hard_valid_total_actual`, `returned_candidate_count`, `lifecycle_revalidation_count`, `safe_stop_escape_evaluator_count` |
| runtime µs | `callback_total`, `planning_total`, `corridor_extraction`, `probe_anchor_selection`, `analytic_root_solve`, `spline_reconstruction`, `geometry_recomputation`, `velocity_shaping`, `candidate_measurement`, `hard_validation`, `ranking`, `lifecycle_revalidation` |
| logging | `evaluation_count`, `dropped_log_count` |

local planner는 Cartesian localization topic을 직접 받지 않습니다. 따라서 `ego.x/y`는 recorded
Frenet `(s,d)`와 가장 가까운 global reference normal로 복원하며, yaw는 reference yaw와
Frenet odometry quaternion의 heading error를 합칩니다. 이때 `ego.pose_source`는
`FRENET_REFERENCE_RECONSTRUCTION`입니다. 이는 `/pf/pose/odom` 원본과 동일하다고 주장하는
필드가 아닙니다.

`validate_candidate_executed_total_actual`은 `validateCandidate()` 함수 진입점에서 직접
증가합니다. 그러므로 P3 strict/relaxed뿐 아니라 같은 callback의 P0 retention, lifecycle
guard/raw, safe-stop escape가 호출한 validator까지 포함합니다. `hard_valid_total_actual`은
그 함수가 최종 `true`에 도달한 횟수입니다. 공개 `candidate_count <= 24`와 다른 계산량입니다.
`safe_stop_escape_evaluator_count`는 정지점 탐침마다 공용 `anyFeasibleCandidateFrom()`에
진입하기 직전에 증가합니다. lifecycle count는 decision에 기록된 guarded/raw validation
attempt boolean의 합입니다.

`callback_total`은 callback 시작부터 queue 제출 직전까지입니다. `planning_total`은 callback
안에서 실제 호출된 P3 evaluator들의 기존 total 합이므로 P0/safe-stop을 포함한 wall-clock
총합으로 해석하지 않습니다. phase 합도 겹칠 수 있으므로 `callback_total`과 동일할 필요가
없습니다.

## CANDIDATE_EVENT schema

| 그룹 | 필드 |
|---|---|
| join | `schema_version`, `event_type`, `run_id`, `scenario_id`, `callback_sequence`, `clearance_pass` |
| identity | `generator_stage`, `template`, `side`, `generation_order`, `candidate_identity`, `logical_identity`, `path_digest`, `component`, `mapping_source`, `source_cell`, `analytic_branch_regime`, `returned_by_policy`, `discarded_side` |
| P3 geometry | `d_target`, `s_probe`, `d_probe`, `d_mid`, `root_index`, `root_type`, `probe_location_rule`, `probe_anchor_rule`, `entry_scale`, `exit_scale`, `z0..z4`, `point_count` |
| validation | `validator_executed`, `validation_authority`, `hard_valid`, `first_failure_enum`, `first_failure_name`, `first_failure_reason`, `failure_waypoint_index`, `failure_obstacle_id`, `all_observed_violation_flags` |
| margin/peak | `center_track_m`, `footprint_track_m`, `obstacle_m`, `peak_lateral_slope`, `lateral_slope_margin`, `peak_positive_curvature_radpm`, `peak_negative_curvature_radpm`, `signed_curvature_margin_radpm`, `peak_curvature_rate_radpm2`, `curvature_rate_margin_radpm2` |
| exact 7-key rank tuple | `exit_reaches_next_obstacle`, `braking_feasible`, `braking_deficit_m`, `velocity_loss`, `minimum_normalized_safety_slack`, `global_path_deviation_m`, `generation_index` |
| outcome | `final_rank`, `selected`, `evaluation_selected` |
| candidate-attributable runtime µs | `spline_reconstruction`, `geometry_recompute`, `velocity_shaping`, `candidate_measurement`, `hard_validation` |

probe/root 선택은 여러 후보가 공유하므로 한 후보에 비용을 중복 귀속하지 않습니다.
`candidate.runtime_us.probe_anchor/root_solve`는 `null`이고 evaluation/callback aggregate가
권위값입니다.

M0-V1은 우/좌 side를 모두 `research_all_candidates`에 복사한 뒤 production이 고르지 않은
side를 `discarded_side=true`, `returned_by_policy=false`로 남깁니다. M0-V2/M1도 실제 append된
후보를 남깁니다. M1 analytic candidate는 버리던 `Probe*`에서 `s_probe`, `d_probe`, 두 rule과
source root index/type을 복사합니다. `ZERO_INTERFACE`처럼 probe가 없는 closure는 의도적으로
`s_probe/d_probe=null`입니다. 이 복사는 생성 뒤 research 구조에만 이루어집니다.

production verdict는 기존 `validateCandidate()`의 첫 failure와 early-return 순서 그대로입니다.
전체 위반 audit를 켜면 그 verdict를 얻은 뒤 별도 pass가 동일 path/visible obstacle을 읽어
`all_observed_violation_flags`를 수집합니다. 이 pass는 validator count, hard-valid, rank,
lifecycle, publication에 연결되지 않으며 runtime 비용 때문에 기본 OFF입니다.

## 확인 한계

- candidate event는 새로 구성된 후보를 기록합니다. continuation suffix는 새 candidate가
  아니므로 planning lifecycle counter/runtime에만 나타납니다.
- `validation_authority=GUARD`는 fresh candidate construction validator입니다. lifecycle의
  guarded/raw/retention attempt는 planning event에 계수되며 candidate를 중복 생성하지 않습니다.
- queue에서 빠진 callback은 복구하지 않습니다. 다음 planning event의 누적
  `dropped_log_count`와 run 종료 후 마지막 값으로 손실을 확인합니다.
