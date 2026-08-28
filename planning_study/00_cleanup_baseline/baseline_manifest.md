# Baseline manifest

## Git state

| 항목 | 값 |
|---|---|
| Branch | `planning_cleanup` |
| HEAD | `f1e1bba9550923ca1ac48a52c44c82b35b707aad` |
| Commit date | `2026-08-27T01:10:02+09:00` |
| Commit subject | `trajectory_generator: 결선용 속도 프로파일 (ay 8.2 / ax 5.2->5.0)` |
| Remote branch | `ifac_f1tenth/planning_cleanup` |
| Remote hash after fetch | `f1e1bba9550923ca1ac48a52c44c82b35b707aad` |
| `HEAD...remote` | `0 0` |
| Status before baseline documents | `?? .idea/`, `?? planning_study/` |

`planning_study/`와 `.idea/`는 baseline 캡처 전부터 untracked였다. Fetch만 실행했고 merge와
push는 하지 않았다.

## Runtime source SHA-256

| 파일 | SHA-256 |
|---|---|
| `src/local_planning/src/local_planner_main.cpp` | `a0dc47f990c297d7455c745b5271592a3d49fa35c901bcc7eb13c1ec6be4dfe1` |
| `src/local_planning/src/local_planner_node.cpp` | `15b705b35c9e442da41c284b41f20ab7a9a2ea09f415b21e0071c2f60fa6f7da` |
| `src/local_planning/src/obstacle_guard.cpp` | `b3a39650a06657cf0745dee6bf4469865b1b4398846e89e092398178a9db0ca7` |
| `src/local_planning/src/p3_maneuver_lifecycle.cpp` | `94fe842967abd21178a1ffb74b12983d153ec7b7eb19a36ae112c3856a01f04c` |
| `src/local_planning/src/p3_shadow.cpp` | `bcbbf382f37b238d6724d4eddad20e85e2684ec3be9981e44dd14fa394baaf2d` |
| `src/local_planning/src/raceline_spline_planner.cpp` | `3ce6dfad73e435593234409e7b285bcb726583e3f93cc1e5d158ff5b7bb497fc` |
| `src/local_planning/src/safe_stop_lifecycle.cpp` | `2721123fa8b3c355b31ee607edb148529d391d6324dad7b1abda455da350f289` |

## Contract/header SHA-256

| 파일 | SHA-256 |
|---|---|
| `include/local_planning/candidate_rank.hpp` | `5eb849808d6abad1353a5ccd29ea231af3d65550241598073e65001285cb5127` |
| `include/local_planning/handoff_latch_gate.hpp` | `467510f80e52e7e5755ed593d85e50f9a30183c5e0ec7449e88457552bba8e4e` |
| `include/local_planning/hold_recovery_gate.hpp` | `acc9257fab528ced6c99aea99563138b26a305265e41127d022105875296f0f6` |
| `include/local_planning/local_planner_node.hpp` | `d63607571f8fdc2309fe2fb1aed724f4110c9fda52a3330681984e0c6b3c0caa` |
| `include/local_planning/maneuver_memory.hpp` | `53eed51c68ddf98783deafe067239914bdfd86eef5c16706124d4792395587e0` |
| `include/local_planning/obstacle_guard.hpp` | `3a28dbed79e380d077ec723c282b1bd20e82da8fe949804eaf7c8da96ba9ca07` |
| `include/local_planning/p3_analytic_solver.hpp` | `e410106266984237f6b1e3286b56aa0093d8ae394a66cc838dbcb2d16c3b8081` |
| `include/local_planning/p3_maneuver_lifecycle.hpp` | `4a6dc677f6357a13f54704b4d433ba846f7ebc483b7a4d809ecd9d23838c759b` |
| `include/local_planning/p3_shadow.hpp` | `5c45913d018878232f768db3e0860294d7a6433b13acdf46fc7ee5d51a5c9c6e` |
| `include/local_planning/raceline_spline_planner.hpp` | `1d618a2154ed87ade0392f2a7432ec19c6247ddb7362e236608838d457c6c5e9` |
| `include/local_planning/raw_slowdown_gate.hpp` | `24a813c7757194f76a5f3db3db77540f1f9f741c8e94d31ac3ff16293376e805` |
| `include/local_planning/safe_stop_lifecycle.hpp` | `01d86c3de4477694d69924a0ac1e702c0d751aa2cf9bd39de3fee1c855144f7c` |

경로는 모두 `src/local_planning/` 기준이다.

## Parameter/config/launch SHA-256

| 파일 | SHA-256 |
|---|---|
| `config/local_planning.yaml` | `b5ebfa9fe3e3c84aacf4c58a9b4b554ef143ab3c1df0e6cd265b52ee65a7c78d` |
| `config/local_planning_sim.yaml` | `955e6e89a6554315bc23c24e5845a81c008499ba59e27a64ff5d004d8e4c4fb6` |
| `config/local_planning_velocity_limits.csv` | `af022f22e4603fb57770efe028362efb1d537f82ee8a30b78442a6421133c1c6` |
| `launch/local_planner_only.launch.py` | `7bda1fe389ad9af1dd465693b4449a7d1ce557635e9855681525e280f0713cf3` |
| `launch/local_planning.launch.py` | `c5c91e7df703294176947deac59c36eb3dbf3a323da0bff649d43c854769ef1c` |
| `CMakeLists.txt` | `4b64d7072734886e0f1520a85b179741443e118fb3529f81c09a0a11ad96f40d` |
| `package.xml` | `ea28d0f66e9545020bc4778e459ea92ddbb338cd4850e506afcc7cbd6623ed0b` |

리팩터링에서는 source hash가 바뀌는 것이 정상이다. Parameter/config/launch hash는 이번 cleanup
범위에서 그대로여야 한다.

## Representative scenario inputs

| 역할 | stream | 프레임 | SHA-256 |
|---|---|---:|---|
| 단일 장애물, 좌측 회피 | `pinch_success.stream` | 1 | `c6ef31ac3b92fe1d3c835d04eba6d92d34138266de620129c94e402abb5d733c` |
| 후보 전멸 및 rejection | `failing_cluster1.stream` | 1 | `4a47a831309a9cd9ab5d37014197bcd344355b667d1c32f1fc7424106470c94a` |
| 복수 후보, 우측/좌측 선택과 ranking | `straight_margin_crawl.stream` | 2 | `727f5db1950883c56a4789096916baec78a8f59f91901b942b3eaaa8c2656d82` |

세 stream은 현재 운영 YAML이 아니라 stream 내부에 저장된 과거 deterministic parameter
snapshot을 사용한다. 그러므로 현재 launch-effective parameter 검증을 대신하지 않는다.

## Representative output fingerprint

- 명령 출력: TSV 56줄
- 출력 SHA-256, run 1:
  `3a482780d133957d3846a2fb6737e55e2745c559b99fdaef69d9378ea5a17054`
- 출력 SHA-256, run 2:
  `3a482780d133957d3846a2fb6737e55e2745c559b99fdaef69d9378ea5a17054`

이 fingerprint에는 summary와 후보별 identity/digest/hard-valid/rejection reason 및 후보 출력
순서가 포함된다.

## Environment

| 항목 | 값 |
|---|---|
| OS/kernel | `Linux 6.8.0-138-generic x86_64` |
| ROS | `Jazzy` |
| CMake | `3.28.3` |
| Ninja | `1.11.1` |
| GCC | `13.3.0` |
| Python | `3.12.3` |
| Isolated run directory | `/tmp/ifac_cleanup_baseline.I4Q0UW` |

`/tmp` 경로는 이 캡처 실행의 임시 산출물이며 baseline의 영구 구성요소가 아니다.
