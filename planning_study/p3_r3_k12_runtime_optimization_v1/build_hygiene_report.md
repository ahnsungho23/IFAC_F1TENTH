# Build hygiene report

## 원인

`src/local_planning/CMakeLists.txt`는 `scripts/check_tracking_lut.py`를 install 대상으로 선언했지만
파일은 존재하지 않았다. Git history에서 이 도구는 2026-08-30 commit
`ae128159011c5aa743015e7e2441495170c95b38`에서 명시적으로 삭제됐다. 삭제 commit은 CMake install
선언을 함께 제거하지 않았다. 따라서 분류는 `STALE_CMAKE_INSTALL_DECLARATION`이다.

## 수정

누락 파일을 과거에서 복원하지 않고 stale `install(PROGRAMS ...)` 두 줄만 제거했다. node source,
planner parameter, launch, runtime install target은 바꾸지 않았다.

## 검증

- `/tmp`의 독립 Release colcon build/install: 성공, 1 package finished.
- 기존 실패 지점인 install 단계: 성공.
- local_planning tests: known mismatch 두 건을 제외한 25/25 성공.
- 첫 node-test 실패는 sandbox가 `$HOME/.ros/log`에 쓸 수 없어서였으며,
  `ROS_LOG_DIR=/tmp/p3_r3_k12_ros_logs`로 재실행해 성공했다.
- task가 보존하라고 지정한 기존 mismatch는 그대로다:
  `safety_margin_m` harness 0.05 vs YAML 0.08,
  right steering planner 0.361 vs control 0.41.
- config directory는 functional-integration tag 대비 diff가 없다.

컴파일 중 기존 `raceline_spline_planner.cpp` 미사용 항목과 기존 test narrowing warning이 있었으나
이번 R3 최적화에서 새 warning은 남기지 않았다.
