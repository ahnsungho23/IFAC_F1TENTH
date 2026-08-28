# Test results

- Capture: `2026-08-29T04:21:08+09:00`
- Commit: `f1e1bba9550923ca1ac48a52c44c82b35b707aad`
- Isolated directory: `/tmp/ifac_cleanup_baseline.I4Q0UW`

## Build

| 실행 | 결과 | 비고 |
|---|---|---|
| Repository alias `cb --packages-select local_planning` | FAIL | 기존 cache `Unix Makefiles`와 alias `Ninja` generator 불일치. Cache 삭제 안 함 |
| `/tmp` isolated Ninja build | PASS | `local_planning` 1 package, 54.7 s |

첫 실패는 source compile failure가 아니라 기존 build cache generator 충돌이다. 기존 cache를
삭제하거나 덮어쓰지 않고 격리 build로 전환했다.

## Parameter/config contracts

| CTest | 결과 | 관측 |
|---|---|---|
| `params_match_yaml` | FAIL | `safety_margin_m`: harness `0.05`, operational YAML `0.08` |
| `velocity_limits_match_csv` | PASS | CSV/YAML/C++/harness 종방향 표 일치 |
| `control_contract_match` | FAIL | real right steering: planner YAML `0.361`, control launch `0.41` |

총 3개 중 1 pass, 2 fail. 이 불일치는 baseline 이전 상태이며 이번 작업에서 수정하지 않았다.

## gTest results

| Binary/selection | 수 | 결과 | 핵심 coverage |
|---|---:|---|---|
| `test_candidate_rank` | 8 | PASS | ranking priority, epsilon, strict weak ordering |
| `test_raceline_spline` representative | 7 | PASS | no-blocking, left/right, deterministic selection, order, rejection, safe-stop |
| `test_p3_production_parity` | 10 | PASS | frozen scenario recovery/failure, M0/M1, impossible gap |
| `test_p3_maneuver_lifecycle` | 19 | PASS | fresh/continuation/completion/invalidation/guard/raw validation |
| `test_safe_stop_lifecycle` | 12 | PASS | latch/release/debounce/empty/stale/blind timeout determinism |
| `test_local_planner_node` | 7 | PASS | authoritative snapshot, explicit empty, reference generation, latch gate |

총 63개 gTest가 통과했다.

`test_local_planner_node` 실행 중 sandbox가 UDP socket/getifaddrs를 제한해 Fast DDS 경고가
반복됐지만, localhost transport로 7개 테스트는 모두 통과했다. 이 결과는 제한된 테스트
환경의 결과이며 실제 DDS 네트워크 품질 검증이 아니다.

### Selected raceline tests

- `IgnoresObstacleWithEnoughRawRacelineClearance`
- `PlanReturnsAvoidanceForAPassableObstacle`
- `UsesRightSideWhenLeftTrackSpaceIsInsufficient`
- `RepeatedCandidateSelectionIsBitDeterministic`
- `SafeStopReasonReportsTheActualRejectionNotAPhantomSideLock`
- `BuildsCollisionFreeStopWhenBothSidesAreClosed`
- `ShiftsOnlyOrderedGlobalRaceLineSamples`

## Deterministic representative harness

입력:

- `pinch_success.stream`
- `failing_cluster1.stream`
- `straight_margin_crawl.stream`

결과:

| 항목 | 값 |
|---|---|
| TSV lines | 56 |
| Run 1 SHA-256 | `3a482780d133957d3846a2fb6737e55e2745c559b99fdaef69d9378ea5a17054` |
| Run 2 SHA-256 | `3a482780d133957d3846a2fb6737e55e2745c559b99fdaef69d9378ea5a17054` |
| Deterministic in repeated run | YES |

요약 관측:

| Scenario/frame | 후보 | Selected | Digest | 결과 |
|---|---:|---|---|---|
| `pinch_success/0` | 6 | `M0_V1_LEFT_f7dc6511c639d3d3` | `f7dc6511c639d3d3` | hard-valid |
| `failing_cluster1/0` | 24 | `NONE` | `NONE` | `NO_HARD_VALID_M1_CANDIDATE` |
| `straight_margin_crawl/0` | 6 | `M0_V1_RIGHT_f32da8afed800e55` | `f32da8afed800e55` | hard-valid |
| `straight_margin_crawl/1` | 15 | `M0_V2_LEFT_ZERO_INTERFACE_BOUNDARY_INSET_b7a84c60e16dd787` | `b7a84c60e16dd787` | hard-valid |

전체 output hash는 후보 row 순서와 개별 identity/digest/hard-valid/rejection reason까지 포함한다.

## Deliberately not run

- Full simulator and controller stack
- CMA-ES or optimization
- Large rosbag replay
- `test_rule_property`
- Package-wide lint suite
- Real vehicle runtime

이 항목들은 baseline 목표에 비해 비용이 크거나 현재 요청 범위를 넘으므로 제외했다.
