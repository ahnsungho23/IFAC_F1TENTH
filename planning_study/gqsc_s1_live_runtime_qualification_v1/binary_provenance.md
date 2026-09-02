# Runtime binary provenance gate

## Decision and revision history

`RUNTIME_BINARY_FROZEN`

`CURRENT_BINARY_OPTIMIZATION = RELEASE_EQUIVALENT` for the frozen W2/W3 Release-overlay binary.

- Workload protocol revision 0: W1 ready; W2/W3 blocked.
- Workload protocol revision 1: same-input W1/W2/W3 workloads frozen.
- Binary provenance revision 2: the recovered normal build was proven non-Release, so it was
  rejected for the absolute 25 ms qualification. A clean experiment-local Release overlay was
  built and functionally validated without changing any workload or production source.

No A/B performance run or p95/p99 calculation occurred in this gate.

## Source identity

- Build source commit: `dc33b875a938fdb7730d35fe61de43ee863b04c0`
- Branch at build: `research`; HEAD equalled `origin/research`; worktree was clean.
- There is no `src/local_planning` diff from algorithm checkpoint
  `55c61e54540fc07d57e6041655e0e59ddb8e17e8` through the build commit.
- Compiler: `/usr/bin/c++`, GCC `11.4.0` on Ubuntu 22.04.
- Frozen contract/config/event SHA-256: `21e653cf...dd40`, `4fe48035...a960`,
  `572adb59...405bf`.
- S1 method: `LEX8_GLOBAL_DISJOINT_COVERAGE4`, SHA-256 `670f39a2...b776`.

## Rejected normal build

The recovered normal `build/local_planning/CMakeCache.txt` has an empty `CMAKE_BUILD_TYPE`.
More importantly, its authoritative `flags.make` and `compile_commands.json` commands for
`local_planner_node.cpp`, `p3_r3_k12.cpp`, `p3_shadow.cpp`, `local_planner_main.cpp`, and the W1
harness contain no `-O0/-O1/-O2/-O3/-Os/-Og` and no `-DNDEBUG`. They contain only
`-Wall -Wextra -Wpedantic -std=c++17` in addition to definitions/includes.

With GCC 11.4, absence of every `-O` option means the compiler's effective default is `-O0`.

Classification: `NON_RELEASE`.

- Normal installed node: `/home/sungho/Documents/GitHub/2026_IFAC/install/local_planning/lib/local_planning/local_planner_node`
  (symlink to the normal build), SHA-256 `3059d3c5...595d`.
- Normal W1 harness: `/home/sungho/Documents/GitHub/2026_IFAC/build/local_planning/p3_r3_k12_integration_harness`,
  SHA-256 `54288540...6ede`.

Neither binary is permitted for the final absolute runtime qualification.

## Frozen Release overlay

Exact build entry point: `tools/build_release_overlay.zsh`. It builds only `local_planning` with
separate experiment paths and refuses to overwrite an existing overlay.

The interactive `cb` alias adds symlink-install and Ninja, but it is not used here: the retained
script's explicit colcon build/install/log bases and non-symlink installed executable are the
authoritative contract.

- Build: `release_overlay/_build`
- Install: `release_overlay/_install`
- Log: `release_overlay/_log`
- CMake contract: `CMAKE_BUILD_TYPE=Release`, `BUILD_TESTING=ON`,
  `CMAKE_EXPORT_COMPILE_COMMANDS=ON`
- Actual flags for `local_planner_core`, node main, and W1 harness:
  `-O3 -DNDEBUG -Wall -Wextra -Wpedantic -std=c++17`

Authoritative generated-evidence hashes:

| Artifact | SHA-256 |
|---|---|
| Release `CMakeCache.txt` | `bd9ac217a8305409802da1b04fe4ef8ebf0e04a4671984aaabf0be4c7e5680a4` |
| Release `compile_commands.json` | `46e67a27fd67b68d58c4fba48f983c2f4b8417d3a7fb395d8ed455ef1a44054f` |
| core `flags.make` | `b916da2efcda52911b2f43855b20a947452cd599bc03a4489850aa00f156e93b` |
| node-main `flags.make` | `23d3f6db74108a7e71aa4743d381309682784d566c3988054ad2900cbbf29c59` |
| W1-harness `flags.make` | `60b00081c1ab7e2a9d33c3938e4d0ca9ce0c291feb0b92b258d8cbfa9adfbb26` |
| colcon/CMake `command.log` | `f3c085bb4e433e98d6de724cc7f5615b6badfe14e0d35cdf3ba2d57fa83d50e6` |

## Frozen executable identities

| Role | Absolute path | SHA-256 | ELF Build ID |
|---|---|---|---|
| W1 standalone Release harness | `/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/release_overlay/_build/local_planning/p3_r3_k12_integration_harness` | `49503d48d96cf408ad47691a69b683e5f2a0c02947a73283994f57b2697c0fd2` | `37aec0429af821572560242e11a20dc9ff74dae7` |
| W2/W3 installed Release node | `/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/release_overlay/_install/local_planning/lib/local_planning/local_planner_node` | `54019a86e13f4dc25771628f7a3d385be8e2c6ce2a657448b3a5939823ab878a` | `69d08471db85935b1f50ceea30b401dcbb978251` |

The build-tree node has the same Build ID but a different SHA after install-time RPATH rewriting;
it is not the frozen W2/W3 executable. Both ROS workloads source the same Release overlay last,
verify `ros2 pkg prefix local_planning`, verify the installed-node hash, and then invoke
`ros2 run local_planning local_planner_node`.

## Bootstrap and fairness

The required order is ROS Humble, simulator overlay, normal IFAC dependency overlay, replay-tool
overlay, then the Release local-planning overlay last. W1 uses the Release harness whose harness TU
and linked `local_planner_core` share the same `-O3 -DNDEBUG` contract as the W2/W3 node. Thus the
W1→W2 comparison does not mix optimized and unoptimized planner code.

## Non-performance parity

- W1 Release harness: SCE018, `128/12/12`, hard/usable `9/9`, selected candidate
  `GQSC_S1_MAIN_COVERAGE_LEFT_c7b2c19bf2af9350`, digest `c7b2c19bf2af9350`, failure `NONE`,
  integrated/downstream digest match.
- W2 Release node: four fresh joined callbacks/four source epochs; exact candidate/count/digest,
  hard-valid selected lifecycle, no safe stop; timing redacted.
- W3 same Release node: the same four-callback parity contract under the frozen full-stack process
  set; timing redacted.

Only parity fields are retained under `validation/`; the W1 raw smoke output containing an incidental
wall-time field was deleted, and no timing value was interpreted.

| Retained non-performance evidence | SHA-256 |
|---|---|
| `release_w1_parity.json` | `20652fab7de7b7d0a58025a2268a129b3464f7c5d9a2eae64646672d4398936e` |
| `release_compile_flags.txt` | `0ba668589540fdeb4772a3403593ddc0e9367c7220df388f9f3a7b4fc930cbc1` |
| `release_w2_joined_smoke.jsonl` | `686b1556ff1f401af55a44ce4c03afb49400fc91cc4c2e5c7e2a412aaf402e8b` |
| `release_w2_ros_graph.txt` | `2710047520d05ebd85bce46a4be6b8ad0615c1bb9e6aa8ef7157d6b3cf3e1a4a` |
| `release_w3_joined_smoke.jsonl` | `2fd8fe7e763c735847be7b1d87001aada5d9f04017384d0c23d64b32c5b32813` |
| `release_w3_ros_graph.txt` | `e647e3df761fdbbc7973450e1be4a5f3adc3274c80892d976a31b8a1d4e4272d` |
