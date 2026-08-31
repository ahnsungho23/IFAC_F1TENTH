# GQSC-S1 callback multi-evaluation / fallback architecture audit v1

## 결론

`GQSC_S1_CALLBACK_ARCHITECTURE_READY_FOR_SMOKE`

Frozen S1 생성법은 바꾸지 않았다. Canonical method SHA-256은 여전히
`670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776`이고, 기준 v3 SHA는
`965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780`이다. 변경은 두 가지
동작동일 계산 제거뿐이다.

1. `ego(s,d,v)`, ordered obstacles, reference/planner revision이 정확히 같은 safe-stop probe는
   같은 callback의 S1 결과를 재사용한다.
2. safe-stop은 경로 자체가 아니라 hard-valid 경로의 **존재 여부**만 필요하다. S1이 이미 같은
   exact validator로 Top-12를 판정했으므로 `would_recover` 인증서를 사용하고, 같은 경로를
   `measure + validate`하는 두 번째 심판을 제거했다. 실제 제동 경로 검증은 제거하지 않았다.

## 관측된 호출 구조

104개 allowed-seen callback의 변경 전 fresh S1 횟수는 `1회 62 / 2회 40 / 3회 2`였다. 추가
44회는 모두 `SAFE_STOP_ESCAPE`였으며:

- 31회는 primary와 ego/obstacles/reference가 모두 같아 재사용 가능했다.
- 13회는 `v=0`인 서로 다른 가상 정지 상태였다.
- 서로 다른 13회 중 hard-valid escape가 존재한 것은 V2E06 한 번뿐이었다.
- 44회 모두 이 corpus에서는 최종 path/kind를 바꾸지 않았다. V2E06도 escape metadata만
  바꾸었고 최종 braking result는 `NO_SAFE_PATH`였다.

변경 후 fresh S1 횟수는 `1회 91 / 2회 13 / 3회 0`이다. 논리적 safe-stop probe 자체는
`0회 62 / 1회 40 / 2회 2`로 동일하다. 상세 lineage와 분류는
`evaluations_per_callback.csv`, `multi_evaluation_classification.csv`,
`extra_evaluation_counterfactual.csv`에 있다.

## Safe-stop 의미

추가 평가는 제동 가능성이나 이미 생성된 제동 경로의 충돌 검사 대체물이 아니다. 후보 정지점
`s_stop = s_ego + max(0, obstacle_start_forward - safe_stop_buffer)`에서 정지한 뒤, 같은 S1
family와 exact validator로 다시 출발할 수 있는지를 묻는다. 실패하면 현재 ego 위치를 확인하고,
그 위치가 가능할 때만 최대 8회 이분탐색으로 가장 늦은 탈출 가능 정지점을 찾는다. 결과는
`stop_at`과 `safe_stop_escape_verified`에 소비된다. 실제 braking prefix는 그 뒤 별도로 만들고
`validateCandidate()`로 검증한다. 따라서 서로 다른 가상 ego의 fresh 평가를 단순 삭제하는 것은
현재의 정지위치/재출발 계약을 바꾼다.

## Runtime

Release, 두 번 warmup 뒤 104 snapshots x 7회 = 728 samples, research instrumentation OFF이다.

| scope | p50 | p90 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|
| S1 evaluator | 11.150 ms | 17.815 ms | 20.214 ms | 20.953 ms | 26.946 ms |
| primary planning | 11.706 ms | 18.702 ms | 20.939 ms | 22.160 ms | 27.916 ms |
| active continuation/revalidation | 0.475 ms | 0.616 ms | 0.633 ms | 0.657 ms | 0.669 ms |
| safe-stop/fallback | 0.000 ms | 8.988 ms | 12.329 ms | 19.128 ms | 22.803 ms |
| full callback model | 13.925 ms | 21.533 ms | **22.840 ms** | 32.220 ms | 34.678 ms |

변경 전 callback p95/p99/max는 43.208/55.006/55.968 ms였다. p95는 25 ms 목표를 통과했다.
p99는 아직 25 ms를 넘으므로 이것은 large benchmark나 실시간 자격의 최종 증명이 아니라,
작은 저속 closed-loop smoke를 시작할 수 있는 gate 통과다. 원자료는 `runtime_breakdown.csv`다.

## 기능·안전 gate

- frozen stateless parity: 104/104 exact, 두 전체 실행의 raw output도 동일
- hard/usable events: 62/54로 불변
- sequential lifecycle: 62/62
- evaluation-local B128/K12/validator12 불변
- raceline regression: 88/88
- production scenario: 기존과 동일한 10/12; 실패는 같은
  `LayoutBReplanRecovers`, `SamePinchGeometryAtTwoDistances`
- observed callback-global `validateCandidate()` 최대: 73 -> 25

`post_change_parity.csv`가 gate 근거다. production validator의 판정식, S1 proposal/ordering,
P3 reconstruction, ranking/selection, lifecycle, fallback, controller contract는 바꾸지 않았다.

## Protocol

이 audit는 9 pilot-seen + 40 DEVELOPMENT + 37 `VALIDATION_SEEN_AFTER_V1` + 18 success-control만
사용했다. `FINAL_HOLDOUT` 내용은 열람하지 않았고 large closed-loop benchmark와 smoke는 실행하지
않았다. 재생성은 Release build 뒤 다음 명령으로 한다.

```bash
python3 planning_study/gqsc_s1_callback_architecture_v1/run_callback_architecture_study.py
```
