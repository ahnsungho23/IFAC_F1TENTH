# Integration diff audit

## 기준과 의도된 변경

- 기준 commit: `8a8dcd93b43d4b34ec797bef23df4d8435422e59`
- 의도된 production 변화: fresh P3-family candidate generator를 frozen
  `V3_SIDE_BALANCED_DISJOINT` GQSC v3로 교체
- 의도적으로 보존: continuation/lifecycle, exact validator, final comparator, speed shaping,
  fallback/safe-stop, messages/topics, controller contract, vehicle/config values

Production adapter는 `evaluateP3ShadowUnguarded()`에서 frozen standalone selector를 B128로 직접
호출합니다. legacy strict/relaxed M0/M1 ladder 및 exact R3 호출은 이 entry에서 제거했고, 명시적
research adapter API만 유지했습니다. `generateP3Candidates()`는 frozen result의 final rank를 기존
downstream comparator tie index로 전달합니다.

## Frozen identity verification

```text
sha256(gqsc_v3_geometry_general_method.json)
= 965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780

prevalidation lock
= ea30a441695732e87d43d6863e1a31992dbfdd45749d324047495b6c96824185
```

- CMake configure는 canonical JSON 누락 또는 SHA mismatch에서 실패합니다.
- `test_gqsc_v3_frozen_contract.py`가 header identity, B128/K12/validator-12 및 frozen policy option을
  고정합니다.
- 노드는 startup에 두 SHA와 세 상한을 출력합니다.
- 기준 commit 대비 `src/local_planning/src/p3_r3_k12.cpp` diff는 0입니다.
- seen-only 104/104에서 production adapter와 frozen research API의 proposal/candidate/verdict/
  selected digest가 exact였습니다.

Canonical JSON의 source-provenance hash 중 selector 구현 `p3_r3_k12.cpp`, selector header,
`p3_shadow.hpp`, selector unit test는 그대로 일치합니다. `p3_shadow.cpp`와 integration harness의
byte hash는 main adapter와 parity mode를 추가했으므로 의도적으로 달라졌습니다. 즉 method SHA는
canonical serialization의 identity이고 integration adapter 자체가 예전 source hash와 같다는
주장은 하지 않습니다. 그 경계를 104개 exact behavioral parity와 explicit no-legacy-seed gate로
검증했습니다.

## Non-method diff boundary

기준 commit 대비 아래 경로의 diff byte count는 0입니다.

- `src/local_planning/config`
- `src/local_planning/launch`
- `src/f1tenth_control`
- `src/obstacle_detector`
- `src/kinematic_localization`

새 message, topic, parameter, launch argument는 추가하지 않았습니다. vehicle length/width,
safety/wall margin, steering/control parameter도 바꾸지 않았습니다. production instrumentation은
기본 OFF이며 startup identity log 외의 문자열/hash serialization을 hot path에 추가하지 않았습니다.

## Source/test/artifact 변경 목록

- `gqsc_v3_frozen_contract.hpp`: canonical identity와 frozen options
- `p3_shadow.cpp`: primary fresh entry 배선과 canonical trace metadata
- `raceline_spline_planner.cpp`: frozen final-order tie index 전달
- `local_planner_node.cpp`: startup SHA/bound log
- `CMakeLists.txt`, `test_gqsc_v3_frozen_contract.py`: SHA/cap gate
- integration harness와 seen-only study runner: parity, sequence, Release timing
- package AGENTS/한국어 node 문서: 현재 production ownership 갱신

## Verification 결과

통과:

- Release build
- canonical SHA contract test
- `test_p3_r3_k12` (frozen selector)
- `test_p3_maneuver_lifecycle`
- `test_safe_stop_lifecycle`
- `test_local_planner_node` (7/7, `ROS_LOG_DIR=/tmp/...`)
- velocity-limit CSV contract
- stateless GQSC parity 104/104
- `git diff --check`

Production scenario test는 legacy research-trace assertion을 명시적 legacy API로 이동한 뒤 12개 중
9개가 통과했습니다. 다음 3개 recovery test는 frozen V3 coverage 차이로 실패하며 method를
수정하지 않았습니다.

- `PassingScenariosKeepRecoveringOnEveryLayout`: `passing_mixed` frame 1
- `LayoutBReplanRecovers`: `layoutB_failing` frame 0, 1, 2
- `SamePinchGeometryAtTwoDistances`: `pinch_failure` frame 0, 1

`test_raceline_spline`은 production GQSC entry에 맞춰 legacy-M1 invocation assertion만 현재
B128/K12 assertion으로 갱신한 뒤 88개 중 8개가 실패했습니다. 실패 범주는 다음과 같습니다.

- frozen selection이 기존 wall/obstacle 균형점 또는 committed side를 재현하지 않음: 2개
- custom test parameter에서 direct transition set이 비어 fail-closed: 4개
- committed-envelope 및 tight-gap recovery 차이: 2개

`test_rule_property`의 deterministic conservative oracle 실험은 feasible layout 538개 중 200개
planner failure로 기존 ceiling 99를 초과했습니다. false pass는 0이었지만 recovery coverage
non-regression은 만족하지 못했습니다. 이는 method를 튜닝하지 않고 frozen candidate를 main으로
교체했을 때 드러난 method/interface 회귀이며 숨기거나 test ceiling을 올리지 않았습니다.

따라서 exact validator, lifecycle 및 safe-stop source/단위 계약이 unchanged인 것은 확인했지만,
generator-dependent production behavior가 전부 unchanged라고 주장하지 않습니다.

기존 repository 검사 중 다음 두 불일치는 이 작업 전부터 알려졌고 요청대로 유지했습니다.

- harness `safety_margin_m=0.05` vs YAML `0.08`
- right steering contract `0.361` vs 다른 검사 기대값 `0.41`

## FINAL_HOLDOUT 경계

Study runner는 허용된 네 seen-only input directory만 열거하며 wildcard parent discovery가 없습니다.
FINAL_HOLDOUT event/manifest/result file은 읽거나 실행하지 않았습니다. 초기 repository-wide file
inventory에서 경로명이 표시된 것 외에 content access는 없었습니다.
