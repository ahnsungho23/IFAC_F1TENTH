# Frozen R3-K12 closed-loop integration v1

## 결론

동결된 `R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12`를 production P3의 **post-ladder recovery
stage**로 통합했다. strict/relaxed production P3가 성공하면 기존 결과를 즉시 반환하고 R3는
호출하지 않는다. 둘 다 실패할 때만 R3가 기존 P3 family 안에서 최대 12개 경로를 재구성하고,
unique path마다 기존 exact validator를 한 번 호출한다.

이미 본 PILOT/DEVELOPMENT/VALIDATION snapshot 86개에서 frozen offline R3와 integrated R3의
factor 순서, reconstruction 수, path digest, validator/hard/usable verdict, 최종 순서와 선택
candidate가 86/86 exact 일치했다. 동일 snapshot을 `plan()`의 재측정·재검증·기존 downstream
rank까지 통과시킨 선택 path digest도 frozen hard recovery 40/40에서 기대값과
일치했다. production-success
control 18개도 pre-integration detached
baseline과 선택 결과, candidate sequence와 path digest가 모두 일치했고, R3 호출과 추가 R3
validator는 0이었다.

FINAL_HOLDOUT 결과는 method 선택이나 구현 조정에 사용하지 않았다. 이 작업에서는 holdout event
파일 또는 결과 CSV 내용을 읽지 않았고, parity corpus도 이미 본 86개 snapshot만 사용했다.

## 구현 경계

- 동결 method SHA-256:
  `7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc`
- quota: feasibility-ranked 10 + coverage-reserve 2, `K=12`
- construction guard failure: K를 소비하지 않음
- constructed digest duplicate: K를 소비하지만 validator를 재호출하지 않음
- R3 constructed candidate 최대: 12
- R3 exact-validator 호출 최대: 12
- acceptance authority: 기존 `validateCandidate()` 그대로
- R3 failure: R3 이전 production fallback/safe-stop 의미 유지
- 파라미터/YAML/차량 치수/validator/ranking/lifecycle/speed shaping 변경: 없음

정확한 실행 계약은 [integration_contract.md](integration_contract.md)에 정리했다.

## 검증 결과

| 검증 | 결과 |
|---|---:|
| Frozen offline vs integrated R3, seen-only | 86/86 exact |
| Existing `plan()` downstream selected digest | 40/40 frozen hard recoveries exact |
| Instrumentation OFF vs ON decision output | 86/86 exact |
| Frozen hard-recovery outcome reproduction | 40/40 exact |
| 그중 usable-valid가 존재하는 event | 36 |
| Production-success non-interference | 18/18 exact |
| Success event의 R3 호출 / 추가 validator | 0 / 0 |
| Seen failure의 최대 constructed / validator | 12 / 12 |
| Release full rule-property test | PASS, 490.90 s |
| 나머지 신규/기존 회귀와 lint | 24/24 PASS |

[offline_vs_integrated_parity.csv](offline_vs_integrated_parity.csv)는 86개 전체 parity를,
[recovery_reproduction.csv](recovery_reproduction.csv)는 frozen hard recovery 40개의 재현을,
[production_success_noninterference.csv](production_success_noninterference.csv)는 성공 control을
담는다.

## Instrumentation

기본값 `research_instrumentation_enable=false`는 바뀌지 않았다. ON일 때만 R3 invocation,
method/SHA, pre-ranking factor 수, lexicographic/coverage 선택, digest 중복, constructed/validator/
hard/usable count, factor lineage, 선택 candidate, phase runtime과 failure 뒤 fallback을 기록한다.
OFF/ON 비교에서 runtime 열을 제외한 결과 열은 86/86 동일했다.

## Build와 회귀 테스트

격리된 Release direct-CMake build는 모든 target을 빌드했고, full-sample
`test_rule_property`는 490.90초에 통과했다. R3 계산으로 기존 60초 timeout을 넘으므로 표본이나
assertion은 바꾸지 않고 이 test의 CTest timeout만 900초로 명시했다. 이를 제외한 24개 CTest도
모두 통과했다.

두 contract test 실패는 현재 branch의 R3 변경 전부터 존재하며 이번 금지 범위라 수정하지 않았다.

- `params_match_yaml`: `stuck_case_harness safety_margin_m=0.05`, operational YAML `0.08`
- `control_contract_match`: local YAML right steering `0.361`, control source `0.41`

격리 colcon은 source compile을 완료했지만 install manifest가 HEAD부터 존재하지 않는
`src/local_planning/scripts/check_tracking_lut.py`를 요구해 install 단계에서 실패했다. R3 source나
link failure가 아니며, direct-CMake build와 tests는 위와 같이 통과했다. 상세 상태는
[test_summary.csv](test_summary.csv)에 있다.

## Runtime smoke와 다음 단계 경계

Release, instrumentation OFF, 단일-process 순차 harness 86개에서 R3 total runtime은 p50
118.950 ms, p95 1457.743 ms, p99/max 3164.845 ms였다. 이는 heterogeneous recorded snapshot을
사용한 deterministic integration smoke이지 closed-loop benchmark가 아니다. 그럼에도 25 ms
planning period보다 큰 event가 있으므로 **실시간 closed-loop readiness는 아직 입증되지 않았다**.
이 결과를 보고 method, K, quota 또는 규칙을 조정하지 않았다. 요청대로 대규모 simulation
benchmark는 실행하지 않았다. 수치는 [runtime_smoke.csv](runtime_smoke.csv)에 있다.

## Reproduction

Release harness를 빌드한 뒤 다음 두 read-only checker를 실행한다.

```bash
python3 src/local_planning/test/check_p3_r3_k12_parity.py \
  --harness <build>/p3_r3_k12_integration_harness --jobs 1

python3 src/local_planning/test/check_p3_r3_k12_success_noninterference.py \
  --baseline-harness <detached-pre-integration>/baseline_selected_harness \
  --integrated-harness <build>/p3_r3_k12_integration_harness
```

`frozen_artifacts.sha256`는 method spec, Reference Oracle v2 spec, evaluation contract, dataset
split manifest와 frozen offline implementation의 SHA-256을 고정한다.
