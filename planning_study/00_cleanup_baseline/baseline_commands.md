# Baseline commands

모든 명령은 repository root에서 실행한다. Production source/config를 수정하지 않는다.

## 1. Git identity

```bash
git status --short --branch
git branch --show-current
git rev-parse HEAD
git fetch ifac_f1tenth planning_cleanup
git rev-list --left-right --count HEAD...ifac_f1tenth/planning_cleanup
git rev-parse ifac_f1tenth/planning_cleanup
```

Fetch만 허용한다. Cleanup baseline 확인 중 merge/push를 실행하지 않는다.

## 2. Source/config fingerprint

```bash
sha256sum \
  src/local_planning/src/*.cpp \
  src/local_planning/include/local_planning/*.hpp \
  src/local_planning/config/* \
  src/local_planning/launch/* \
  src/local_planning/CMakeLists.txt \
  src/local_planning/package.xml
```

## 3. Isolated build

기존 workspace의 `build/local_planning`은 Unix Makefiles cache이고 `cb` alias는 Ninja를
강제한다. Cache를 삭제하지 말고 `/tmp`에 새 build를 만든다.

```bash
set -e
source /opt/ros/jazzy/setup.zsh
source /home/sungho/Documents/GitHub/2026_IFAC/install/setup.zsh
baseline_root=$(mktemp -d /tmp/ifac_cleanup_baseline.XXXXXX)
colcon --log-base "$baseline_root/log" build \
  --base-paths /home/sungho/Documents/GitHub/2026_IFAC/src/local_planning \
  --packages-select local_planning \
  --build-base "$baseline_root/build" \
  --install-base "$baseline_root/install" \
  --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
source "$baseline_root/install/setup.zsh"
```

이 캡처에서 `baseline_root`는 `/tmp/ifac_cleanup_baseline.I4Q0UW`였다.

## 4. Parameter/config contract tests

```bash
ctest --test-dir "$baseline_root/build/local_planning" \
  --output-on-failure \
  -R '^(params_match_yaml|velocity_limits_match_csv|control_contract_match)$'
```

현재 baseline의 예상 결과는 1 pass, 2 fail이다. 실패 signature는
[test_results.md](test_results.md)를 참고한다.

## 5. Representative planner tests

```bash
"$baseline_root/build/local_planning/test_candidate_rank"

"$baseline_root/build/local_planning/test_raceline_spline" \
  --gtest_filter='RacelineSplinePlanner.IgnoresObstacleWithEnoughRawRacelineClearance:RacelineSplinePlanner.PlanReturnsAvoidanceForAPassableObstacle:RacelineSplinePlanner.UsesRightSideWhenLeftTrackSpaceIsInsufficient:RacelineSplinePlanner.RepeatedCandidateSelectionIsBitDeterministic:RacelineSplinePlanner.SafeStopReasonReportsTheActualRejectionNotAPhantomSideLock:RacelineSplinePlanner.BuildsCollisionFreeStopWhenBothSidesAreClosed:RacelineSplinePlanner.ShiftsOnlyOrderedGlobalRaceLineSamples'

"$baseline_root/build/local_planning/test_p3_production_parity"
"$baseline_root/build/local_planning/test_p3_maneuver_lifecycle"
"$baseline_root/build/local_planning/test_safe_stop_lifecycle"

export ROS_LOG_DIR="$baseline_root/ros_log"
"$baseline_root/build/local_planning/test_local_planner_node"
```

## 6. Representative deterministic P3 fingerprint

```bash
"$baseline_root/build/local_planning/p3_production_parity_harness" \
  src/local_planning/test/data/p3_scenarios/pinch_success.stream \
  src/local_planning/test/data/p3_scenarios/failing_cluster1.stream \
  src/local_planning/test/data/p3_scenarios/straight_margin_crawl.stream \
  | sha256sum
```

예상 SHA-256:

```text
3a482780d133957d3846a2fb6737e55e2745c559b99fdaef69d9378ea5a17054  -
```

같은 명령을 두 번 실행해 hash가 같은지도 확인한다. Hash가 다르면 후보 순서, identity,
path digest, 판정 또는 rejection reason 중 하나가 달라진 것이다.

## 7. Production contract diff guard

```bash
git diff -- \
  src/local_planning/config \
  src/local_planning/launch \
  src/local_planning/package.xml

git status --short --branch
```

리팩터링 diff에서 config/launch/topic/message 계약 변화가 보이면 cleanup과 분리한다.
