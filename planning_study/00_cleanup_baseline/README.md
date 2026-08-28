# Local planning cleanup behavior baseline

이 디렉터리는 local planning 리팩터링 전 동작 기준선이다. Production source, parameter,
P3 수식, 후보 순서, ranking, validator, lifecycle, topic/message 계약은 수정하지 않았다.

## Baseline identity

- Branch: `planning_cleanup`
- Commit: `f1e1bba9550923ca1ac48a52c44c82b35b707aad`
- Remote: `ifac_f1tenth/planning_cleanup`
- Capture time: `2026-08-29T04:21:08+09:00`
- Fetch 후 local/remote divergence: `0 0`

소스·설정 및 입력 stream의 SHA-256은 [baseline_manifest.md](baseline_manifest.md)에 있다.

## 실행한 검사

기존 CTest/gTest/하니스만 사용했다. Full simulation, CMA-ES, rosbag replay,
`test_rule_property`, 전체 lint suite는 실행하지 않았다.

- `/tmp` 격리 build에서 `local_planning` package 빌드
- parameter/config 계약 CTest 3개
- `test_candidate_rank`: 8개 전부
- `test_raceline_spline`: 대표 7개
- `test_p3_production_parity`: 10개 전부
- `test_p3_maneuver_lifecycle`: 19개 전부
- `test_safe_stop_lifecycle`: 12개 전부
- `test_local_planner_node`: 7개 전부
- `p3_production_parity_harness`: 대표 stream 3개를 반복 실행

gTest는 총 63개가 통과했다. Parameter/config CTest는 3개 중 1개가 통과했고 2개는
baseline 시점부터 실패한다. 실패 내용은 값을 고치지 않고 [test_results.md](test_results.md)에
그대로 기록했다.

## 동작 불변 조건

리팩터링 후 다음이 동일해야 한다.

- 대표 stream의 selected candidate identity와 selected path digest
- 후보 수와 생성 순서
- 각 후보 identity, path digest, hard-valid 판정, rejection reason
- 선택 후보와 ranking 순서
- 생성 경로의 waypoint 수와 벡터 순서
- waypoint의 `s/d/x/y/psi/kappa/vx/ax` 값
- no-blocking, avoidance, safe-stop 판정
- immutable committed suffix의 identity, digest, continuation/invalidation 판정
- safe-stop release 조건과 우선순위
- 주요 diagnostic schema와 안정 필드
- parameter 이름·값과 config/launch hash

대표 P3 하니스 출력은 56줄이며, 두 번 연속 같은 SHA-256을 냈다.

```text
3a482780d133957d3846a2fb6737e55e2745c559b99fdaef69d9378ea5a17054
```

필드별 정확한 계약과 비교 한계는 [behavior_contract.md](behavior_contract.md)를 따른다.

## 아직 자동 비교할 수 없는 항목

- ROS topic으로 실제 발행된 `/avoid_waypoints` 전체 메시지의 byte-level equality
- publish 직전 raw slowdown overlay가 적용된 최종 waypoint 전체 dump
- `header.stamp`, callback sequence 등이 포함된 diagnostic JSON 전체 문자열 equality
- state machine까지 포함한 safe-stop/fallback/commitment end-to-end 순서
- launch override가 적용된 실제 실행 중 parameter dump
- 완전히 빈 장애물 입력에 대한 frozen scenario stream

현재 하니스의 path digest는 waypoint 수와 순서 및 핵심 8개 double 필드를 포함하지만,
waypoint `id`, `d_left`, `d_right`, message header, `OTWpntArray` metadata는 포함하지 않는다.
따라서 offline selected path digest와 실제 `/avoid_waypoints` 전체 메시지가 같다고 확대 해석하면
안 된다.

## Regression 확인 방법

명령은 [baseline_commands.md](baseline_commands.md)에 있다. 리팩터링 후에는 다음 순서로
확인한다.

1. `/tmp` 격리 디렉터리에서 `local_planning`만 빌드한다.
2. 기록된 gTest/CTest를 동일하게 실행한다.
3. 대표 세 stream의 하니스 출력 SHA-256이 위 값과 같은지 확인한다.
4. 두 기존 parameter 계약 실패가 다른 실패로 변하지 않았는지 확인한다.
5. `src/local_planning/config`, launch, message/topic 계약에 의도하지 않은 diff가 없는지 확인한다.

이 baseline은 리팩터링 비교용이다. 현재 production 거동이 물리적으로 완전하거나 모든 기존
계약 테스트가 통과한다는 인증은 아니다.
