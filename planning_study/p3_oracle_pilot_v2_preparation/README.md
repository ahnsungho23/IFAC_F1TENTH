# P3 Oracle Pilot v2 Preparation

## 결론

ORACLE PILOT v2 준비를 완료했다. **expensive P3 oracle은 실행하지 않았다.** 네 real bag의
최종 instrumentation-v2 frozen replay에서 반복 `NO_CANDIDATE` episode 266개를 얻었고, 그중
193개 episode에서 exact evaluator lineage, localization temporal coverage, obstacle context,
candidate-specific waypoint-0 footprint gate를 모두 통과한 최초 snapshot을 확보했다.

상위 10개를 포함한 전체 순위는 `oracle_eligible_snapshots.csv`에 있다. 이 결과는 oracle을
실행할 수 있는 입력 후보 목록이지 mapping miss, P3-family infeasibility, localization-clean
label의 증거가 아니다.

## 1. Waypoint-0 판정

현재 P3의 waypoint 0은 실제 fixed ego pose가 아니다. 다음 ordered global-reference sample을
고르고 그 sample에 P3 offset을 적용한 candidate-specific future path point다.

\[
i_0=\operatorname{nextReferenceIndex}(s_e),\quad
\Delta s_0=\operatorname{forwardDistance}(s_e,s^{ref}_{i_0}),
\]

\[
d_0=P_3(\Delta s_0;d_e,d_{target},d_{mid}),\quad
(x_0,y_0)=r_{i_0}+d_0 n_{i_0}.
\]

heading도 ego yaw가 아니다. operational `analytic_path_geometry_enable=true`에서는 첫 다섯
candidate `x/y` sample의 local cubic derivative로 `psi_0=atan2(y',x')`를 다시 계산한다.
EgoFrenetState에는 yaw 자체가 없다.

- E02: ego `(s,d)=(9.290700,+1.035777) m`, first sample
  `(s,d)=(9.529387,+1.034623) m`, `Delta s0=0.238687 m`.
- E04: ego `(s,d)=(21.445466,-1.157887) m`, first sample
  `(s,d)=(21.568176,-1.156159) m`, `Delta s0=0.122710 m`.

두 경우 모두 `z0` 뒤 첫 P3 segment 안이다. 첫 segment 끝값은 `d_target`이고 그 끝의
derivative/acceleration은 다음 secant, 즉 `d_mid`까지 사용하므로 두 unknown 모두 waypoint-0
position/tangent와 rotated footprint test에 영향을 줄 수 있다. 따라서 v1 E02/E04의
waypoint-0 `footprint_track_bound`는 candidate의 첫 sampled point 실패이지, 실제 물리 ego
footprint 실패가 아니다. 자세한 source/math/numeric evidence는 `waypoint0_semantics.md`에 있다.

## 2. Research instrumentation v2

schema를 `local_planning_research/2`로 올렸고 기존 default gate는 그대로 `false`다. OFF일 때
logger/research cycle/snapshot hash/추가 waypoint-0 측정은 생성되지 않는다. production P3
mapping, generator, analytic solver, validator verdict, ranking, parameter, lifecycle, safe-stop,
publication은 연구 필드를 읽지 않는다.

새 `evaluation_events.jsonl`은 top-level `evaluateP3Shadow()` 호출마다 callback-local
`evaluation_sequence`를 부여한다. STRICT/RELAXED는 같은 invocation sequence를 공유하고
`clearance_pass`로 구분된다. 다음 evaluator 호출은 같은 callback이어도 새 sequence다.

각 evaluation/candidate에는 다음 lineage가 기록된다.

- `callback_sequence`, `evaluation_sequence`, `evaluation_role`, `clearance_pass`
- `input_snapshot_id`, `ego_snapshot_id`, `obstacle_snapshot_id`,
  `reference_snapshot_id`
- source stamp, obstacle sequence, source epoch, reference generation
- evaluator가 실제 받은 ego `s/d/speed`, obstacle 핵심 geometry
- 그 evaluation에 속한 candidate path digest 목록, stage/template
- 각 candidate 첫 sampled waypoint의 `s/d/x/y/yaw`, center margin, footprint margin

role은 direct TEST_ACTIVE/SHADOW, production `PLAN_PRIMARY`, `SAFE_STOP_ESCAPE`를 구분한다.
production 내부에는 기존 `p0_failure_reason="PLAN"`을 그대로 전달하고 research role은 별도
write-only 인자로 분리했다. exact join key는
`(run_id, callback_sequence, evaluation_sequence, clearance_pass)`다. 전체 schema 계약은
`src/local_planning/docs/research_instrumentation.md`에 있다.

## 3. OFF/ON parity

- 10개 frozen scenario stream, 281개 result record: OFF/ON byte-identical.
- 양쪽 SHA-256:
  `213dde6f525fe4fb1e627024cde34d12d825b4abbb370b6d53c5174cb5d23128`.
- `test_research_instrumentation`: 2/2 passed.
- `test_p3_production_parity`: 12/12 passed.
- package CTest: 26개 중 24개 passed. `test_local_planner_node`는 project overlay와
  `ROS_LOG_DIR=/tmp/p3_v2_ctest_ros`로 재실행해 passed. 남은 두 기존 failure는
  `stuck_case_harness safety_margin_m=0.05` 대 operational YAML `0.08`, control steering-right
  `0.41` 대 local-planning YAML `0.361`의 pre-existing contract mismatch다. 요청된 parameter
  변경 금지에 따라 수정하지 않았다.
- source aggregate:
  `55a078d810ad39551a491e04a84b559394873cbf01319195a996f2cde2100521`.
- operational config aggregate, unchanged:
  `6009b3211a50dce7159b95553b7dbea1417eb20b50dbc6fb716947973a2e334a`.

전체 `colcon build --symlink-install`은 source/test compilation 뒤 기존 CMake가 참조하는 미존재
`src/local_planning/scripts/check_tracking_lut.py`의 install 단계에서 실패했다. 사용자 삭제물을
복원하지 않았고, 생성된 final targets를 `cmake --build`로 다시 빌드한 뒤 위 unit/parity test를
실행했다. 이 결손은 v2 변경과 무관하다.

## 4. Four-bag frozen replay

| bag | cycles accepted/written | dropped | in-bag planning events | PLAN strict eval | repeated episodes | eligible representatives |
|---|---:|---:|---:|---:|---:|---:|
| 2026-08-24 19:00:30 | 34,955 / 34,955 | 0 | 34,853 | 5,385 | 109 | 96 |
| 2026-08-24 19:24:17 | 29,852 / 29,852 | 0 | 29,750 | 3,645 | 71 | 57 |
| 2026-08-25 09:23:32 | 2,296 / 2,296 | 0 | 2,195 | 5 | 1 | 1 |
| 2026-08-25 09:59:22 | 43,036 / 43,036 | 1 | 42,933 | 2,971 | 85 | 39 |

모두 clean shutdown했다. 09:59 bag의 callback 1개는 bounded nonblocking logger의 `try_lock`
drop이다. callback sequence gap에서 episode를 끊었으며, gap을 가로질러 연속 실패로 합치지
않았다. 선정 row는 각각 evaluation/candidate event가 완전 존재하고 lineage가 일치한다.

## 5. Episode 및 eligibility 계약

`NO_CANDIDATE` callback은 exact `PLAN_PRIMARY/STRICT` evaluation이 constructed/returned
candidate를 만들었지만 evaluation hard-valid가 0이고, callback이 candidate를 선택하거나
continue하지 못한 경우다. 같은 source epoch에서 callback sequence가 연속인 실패를 묶고,
길이 2 이상만 episode로 인정했다.

episode별 earliest callback 중 아래를 모두 만족하는 첫 evaluation 하나를 대표로 뽑았다.

1. 네 snapshot ID와 source lineage가 존재한다.
2. evaluation의 candidate row count/path-digest multiset/네 snapshot ID가 candidate stream과
   정확히 일치한다.
3. nearest localization pose offset 절댓값 `<=25 ms`, pose sample gap `<=50 ms`다. 이는 temporal
   coverage gate일 뿐 quality threshold가 아니며 label은 계속 `LOC_UNKNOWN`이다.
4. evaluator obstacle context가 비어 있지 않다.
5. production constructed/returned candidate가 있고 hard-valid는 0이다.
6. returned production candidate 중 적어도 하나의 candidate-specific waypoint-0 footprint
   margin이 `>=-1e-9 m`다.

waypoint-0 orientation은 candidate마다 달라 하나의 snapshot-level 고정 margin은 존재하지
않는다. CSV의 대표 margin은 returned candidate 중 최대값이며, 최소값과 valid candidate 수도
함께 기록했다. eligibility 제외 사유는 valid waypoint-0 candidate 부재 2,050 evaluations,
localization temporal coverage 미달 480 evaluations였다.

## 6. Ranked oracle-eligible snapshots — top 10

`constructed/returned/valid`는 exact PLAN evaluation의 actual count다. `prior safe-stop`은 직전
callback의 safe-stop 상태다. 상위 9개는 episode offset 0이며 직전 callback에 safe-stop/boundary
degradation이 없었다. PLAN evaluation은 같은 callback의 fallback 생성보다 먼저 실행되므로,
callback 종료 summary가 safe-stop이어도 evaluator 입력 자체는 fallback path 생성 전이다.

| rank | bag | elapsed s | cb/eval | ego `(s,d,v)` | wp0 max margin m | constructed/returned/valid | lifecycle / fallback | obstacle summary | prior safe-stop |
|---:|---|---:|---|---|---:|---|---|---|---|
| 1 | 08-24 19:24:17 | 721.860 | 28019/2 | `(5.827,-0.359,0.183)` | +0.4131 | 9/3/0 | IDLE / BOUNDARY_HANDOFF_UNRESOLVED | id145 `s=[5.97,5.99] d=[-0.23,-0.21]` | no |
| 2 | 08-24 19:00:30 | 920.020 | 33976/2 | `(14.502,+0.050,0.000)` | +0.6309 | 3/3/0 | IDLE / BOUNDARY_HANDOFF_UNRESOLVED | id228 `s=[14.69,14.90] d=[-0.33,-0.21]` | no |
| 3 | 08-25 09:59:22 | 157.790 | 6394/2 | `(23.434,-0.081,0.000)` | +0.2646 | 9/3/0 | IDLE / BOUNDARY_HANDOFF_UNRESOLVED | id54 `s=[23.78,23.90] d=[-0.24,-0.04]` | no |
| 4 | 08-25 09:59:22 | 671.170 | 25383/2 | `(8.335,+1.123,0.000)` | +0.0192 | 6/6/0 | IDLE / NO_HARD_VALID_M1_CANDIDATE | id445 `s=[20.89,21.11] d=[-0.69,-0.19]` | no |
| 5 | 08-24 19:00:30 | 577.840 | 20689/2 | `(2.076,-0.818,0.000)` | +0.0289 | 2/2/0 | IDLE / BOUNDARY_HANDOFF_UNRESOLVED | id79 and id104 | no |
| 6 | 08-24 19:00:30 | 786.320 | 28885/2 | `(34.226,-0.480,0.000)` | +0.4644 | 22/16/0 | IDLE / no-static-obstacle callback fallback | ids189,192,185 in exact PLAN input | no |
| 7 | 08-24 19:00:30 | 552.200 | 19683/2 | `(0.696,+0.010,0.000)` | +0.6742 | 2/2/0 | IDLE / NO_HARD_VALID_M1_CANDIDATE | 3 obstacle records, including repeated id79 | no |
| 8 | 08-25 09:23:32 | 6.410 | 358/2 | `(14.697,-0.008,0.000)` | +0.5698 | 9/3/0 | IDLE / BOUNDARY_HANDOFF_UNRESOLVED | id110 `s=[14.68,15.09] d=[+0.23,+0.81]` | no |
| 9 | 08-25 09:59:22 | 538.290 | 20095/2 | `(43.810,+0.022,0.281)` | +0.5304 | 2/2/0 | IDLE / BOUNDARY_HANDOFF_UNRESOLVED | id319 `s=[44.09,44.29] d=[+0.20,+0.43]` | no |
| 10 | 08-24 19:24:17 | 439.420 | 17300/1 | `(1.351,-0.284,0.000)` | +0.2970 | 2/2/0 | IDLE / SAFE_STOP_AUTHORITY | ids18,17 | yes |

Top-10의 localization join offset 절댓값은 `0.619–14.613 ms`, pose gap은
`18.867–31.625 ms`이고 모두 `LOC_UNKNOWN`이다. exact obstacle JSON, all snapshot IDs,
stage/template, first-failure distribution은 CSV에 보존했다. 특히 rank 6의 callback-global
fallback 문자열과 exact PLAN evaluator obstacle input이 다르므로, oracle input은 반드시
evaluation row의 snapshot ID/obstacles를 사용해야 한다.

## 7. 산출물

- `waypoint0_semantics.md`: E02/E04 source/math/numeric diagnosis.
- `parity_summary.csv`: OFF/ON 및 unit-test evidence.
- `replay_manifest.csv`: 네 final replay의 schema/cycle/drop/count.
- `failure_episodes.csv`: 반복 실패 episode와 earliest eligible 위치.
- `oracle_eligible_snapshots.csv`: 193개 전체 ranked snapshot 및 exact lineage.
- `eligibility_exclusion_distribution.csv`: failure-evaluation 제외 사유.
- `preparation_summary.json`: machine-readable 계약과 집계.

P3 oracle, new mapping, mapping search는 실행하지 않았다. 따라서 이 준비 결과로 mapping miss를
주장할 수 없다. 다음 단계는 별도 승인 후 이 ranked exact-input snapshot을 oracle pilot 입력으로
사용하는 것이다.
