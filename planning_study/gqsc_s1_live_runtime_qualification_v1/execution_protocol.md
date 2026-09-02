# Execution protocol — protocol revision 2, binary provenance revision 2

## Authorization and stop rule

Revision 1 and execution attempt 0 collected no timing. Revision 2 freezes the final decision and
run-evidence contract in `decision_contract.md`. Do not start a partial A/B
run unless the full preflight matches. Any branch, hash, binary, CPU topology, QoS, graph, row
order, join, or parity mismatch stops the attempt. Do not rebuild production code, edit a parameter,
substitute an input, or select another CPU during a run.

## Environment and external tool build

Use zsh without `set -u`:

```zsh
source /opt/ros/humble/setup.zsh
source /home/sungho/sim_ws/install/setup.zsh
source /home/sungho/Documents/GitHub/2026_IFAC/install/setup.zsh
cd /home/sungho/Documents/GitHub/2026_IFAC
planning_study/gqsc_s1_live_runtime_qualification_v1/tools/build_tools.zsh
source planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_install/setup.zsh
source planning_study/gqsc_s1_live_runtime_qualification_v1/release_overlay/_install/setup.zsh
```

`build_tools.zsh` builds only `gqsc_runtime_replay` in the experiment's ignored Release build,
install, and log directories. It does not build `local_planning`.

The one-time clean planner build is exactly
`tools/build_release_overlay.zsh`. It builds only `local_planning` with
`CMAKE_BUILD_TYPE=Release`, `BUILD_TESTING=ON`, and separate ignored build/install/log paths. It
refuses to overwrite an existing overlay. The normal workspace is never the planner provider for a
qualification shell; the Release overlay must be sourced last and
`ros2 pkg prefix local_planning` must equal
`.../release_overlay/_install/local_planning`.

Before every attempt, record and verify:

```zsh
git status --short --branch
git rev-parse HEAD
sha256sum planning_study/p3_geometry_conditioned_method_v1/success_control_inputs/SCE018.event
sha256sum src/local_planning/include/local_planning/gqsc_s1_frozen_contract.hpp
sha256sum src/local_planning/config/local_planning.yaml
sha256sum planning_study/gqsc_s1_live_runtime_qualification_v1/release_overlay/_build/local_planning/p3_r3_k12_integration_harness
sha256sum planning_study/gqsc_s1_live_runtime_qualification_v1/release_overlay/_install/local_planning/lib/local_planning/local_planner_node
ros2 pkg prefix local_planning
sha256sum planning_study/gqsc_s1_live_runtime_qualification_v1/tools/gqsc_runtime_replay/src/sce018_replay_driver.cpp
sha256sum planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_install/lib/gqsc_runtime_replay/sce018_replay_driver
sha256sum planning_study/gqsc_s1_live_runtime_qualification_v1/tools/run_w1_qualification.zsh
sha256sum planning_study/gqsc_s1_live_runtime_qualification_v1/tools/run_w2_qualification.zsh
sha256sum planning_study/gqsc_s1_live_runtime_qualification_v1/tools/run_w3_qualification.zsh
sha256sum planning_study/gqsc_s1_live_runtime_qualification_v1/tools/analyze_qualification.py
lscpu -e=CPU,CORE,SOCKET,NODE,ONLINE,MAXMHZ,MINMHZ
taskset -pc $$
sed -n '1p' /sys/devices/system/cpu/cpu8/topology/thread_siblings_list
sed -n '1p' /sys/devices/system/cpu/cpu8/cpufreq/scaling_governor
on_ac_power
cat /proc/cmdline
```

Required identities are in `workload_freeze.md`. The parent mask must be `0-23`; CPU 8 must remain
online on core 4 with sibling `8-9`. All non-planner experiment processes are assigned
`0-7,10-23` in both conditions. A leaves only the planner unpinned; B changes only the planner to
`taskset -c 8`. The governor remains `powersave`, and `on_ac_power` must return 0 (`AC`) before and
after every run. No affinity command was executed as part of the revision-1 smoke.

## W1

Before and after each A/B pair, retain this untimed parity output:

```zsh
env -u GQSC_S1_MAIN_TIMING -u GQSC_S1_STAGE_TIMING \
  planning_study/gqsc_s1_live_runtime_qualification_v1/release_overlay/_build/local_planning/p3_r3_k12_integration_harness \
  planning_study/p3_geometry_conditioned_method_v1/success_control_inputs/SCE018.event
```

The exact future timing runner for both conditions is `tools/run_w1_qualification.zsh`. The literal
first W1 row is:

```zsh
planning_study/gqsc_s1_live_runtime_qualification_v1/tools/run_w1_qualification.zsh \
  A 1 ATTEMPT0 \
  /home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/raw/W1_SCE018_STANDALONE/R1_A_ATTEMPT0
```

Every other invocation substitutes only the condition/repeat/attempt binding from `run_schedule.csv`
and the matching literal output directory. The runner uses the same frozen harness/environment
arguments as revision 1, adds only a stopped pre-exec PID/affinity gate and AC/system evidence, and
requires exactly 200 measurement rows. The harness executes 20 warm-ups internally but does not
emit their timing rows; W2/W3 retain their 20 warm-up join rows explicitly.

## W2

The exact future runner is `tools/run_w2_qualification.zsh`. It verifies core and Release binary
hashes, confirms the Release package prefix, launches that installed `local_planner_node`, applies
only profiling enable and diagnostic detail, remaps
its public inputs/outputs to `/gqsc_runtime/*`, then runs the external collector for exactly 20+200
unique joined callbacks. No simulator process is launched.
Before starting replay, the runner resolves the actual installed-node child PID and records/verifies
its executable and A/B allowed list. It then resolves the replay child and verifies the background
mask. `affinity.txt` and `system_context.txt` are mandatory.

For example, the literal first W2 row is:

```zsh
planning_study/gqsc_s1_live_runtime_qualification_v1/tools/run_w2_qualification.zsh \
  A 1 ATTEMPT0 \
  /home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/raw/W2_SCE018_ROS_NODE_TEST_ACTIVE/R1_A_ATTEMPT0
```

Every other invocation substitutes only the condition and repeat values in the binding CSV and
uses the corresponding literal output directory
`raw/W2_SCE018_ROS_NODE_TEST_ACTIVE/R<repeat>_<condition>_ATTEMPT0`. The script accepts only A/B,
repeat 1–5, and `ATTEMPT0`/`RERUN1`, and refuses an existing output path. This is deterministic
argument expansion, not an operator-selected workload or schedule.

## W3

The exact future runner is `tools/run_w3_qualification.zsh`. Its startup order and real-project
process set are:

1. `ros2 launch f1tenth_gym_ros gym_bridge_launch.py`: `/bridge`, map server, lifecycle manager,
   RViz, ego robot state publisher; simulator config hash is frozen.
2. `ros2 launch kinematic_localization kinematic_localization.launch.py map_name:=ifac_track
   use_sim_time:=false map_frame:=map base_frame:=ego_racecar/base_link
   odom_topic:=/ego_racecar/odom map_topic:=/kinematic_localization/map
   auto_init_from_waypoints:=false`: `/kinematic_localization`.
3. One `/initialpose` at `x=-0.427`, `y=0.456`, `z=0.3651`, `w=0.9310`; both simulator and
   localization consume it.
4. `F1_MAP=ifac_track ros2 launch global_planning global_planning.launch.py
   map_name:=ifac_track`: global trajectory and Frenet odometry nodes.
5. `ros2 launch obstacle_detector obstacle_detector.launch.py simulator:=true
   use_sim_time:=false rviz:=false`: the real normal-topic detector.
6. The exact same Release-overlay qualification `local_planner_node` binary as W2, with the same
   private remaps and profiling switches. No second local planner is launched.
7. `ros2 launch state_machine state_machine.launch.py`.
8. `ros2 launch f1tenth_control control_sim.launch.py`: control map, cruise controller, simulated
   IMU bridge, and drive-source selector.
9. After the complete ready-node predicate, the same replay/collector as W2 starts.

The script validates the simulator launch/config, `ifac_track` map triplet, current global path,
production node, event, configuration, method file, and complete node set before collection. On
exit it signals the replay then all process groups and preserves every log and graph file.
Before replay begins, it records/verifies the actual planner child and every current member of each
experiment-owned non-planner process group. AC/governor/system context and graph evidence are
mandatory.

The literal first W3 row is:

```zsh
planning_study/gqsc_s1_live_runtime_qualification_v1/tools/run_w3_qualification.zsh \
  A 1 ATTEMPT0 \
  /home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/raw/W3_SCE018_FULL_STACK_CONTENTION/R1_A_ATTEMPT0
```

Other rows use the exact condition/repeat from `run_schedule.csv` and the corresponding
`raw/W3_SCE018_FULL_STACK_CONTENTION/R<repeat>_<condition>_ATTEMPT0` path.

## Planner-input isolation

For W2 and W3 the qualified planner subscribes to exactly one publisher on each of:

- `/gqsc_runtime/global_waypoints`
- `/gqsc_runtime/confirmed_static_obs`
- `/gqsc_runtime/frenet_odom`
- `/gqsc_runtime/state`

That publisher is `sce018_replay_driver`. The normal W3 `/confirmed_static_obs` publisher remains
`obstacle_detector`, and normal `/ego_racecar/odom` publisher remains `bridge`, but neither topic is
connected to the qualified planner. Graph snapshots are mandatory raw artifacts.

## Smoke-only commands

These are fixed wiring checks and never performance inputs:

```zsh
planning_study/gqsc_s1_live_runtime_qualification_v1/tools/run_w2_smoke.zsh
planning_study/gqsc_s1_live_runtime_qualification_v1/tools/run_w3_smoke.zsh
```

They collect four joined callbacks, redact all numeric timing/raw JSON, and label timing
interpretation forbidden.

## Samples, timeout, retries, and order

- W1 executes exactly 20 internal warm-ups and emits exactly the next 200 measurement rows. W2/W3
  retain exactly 20 joined rows with `phase=WARMUP` and exclude them from statistics, then retain
  exactly 200 `phase=MEASUREMENT` rows; an extra eligible row fails the attempt.
- Five A/B pairs per workload. Timeouts: W1 600 s, W2 600 s, W3 900 s.
- Invalid/rejected callbacks are retained as faults and never silently renumbered into samples.
- One `RERUN1` is allowed only for a recorded infrastructure/protocol failure, never for a latency
  value. Preserve the failed attempt. A second failure or more than one repeat requiring rerun fails
  the workload.
- Pair order is fixed independently for each workload: repeat 1 A/B, 2 B/A, 3 A/B, 4 B/A, 5 A/B.
  `run_schedule.csv` is authoritative and was frozen before any A/B output.

## Analysis and decisions

`decision_contract.md` is authoritative for timing-scope interpretation, nearest-rank quantiles,
six deadline decisions, the W3-B production decision, contrasts, strong effects, final precedence,
parity/infrastructure handling, affinity evidence, and system state. The frozen analysis entry point
is `tools/analyze_qualification.py`; only its synthetic `--self-test` is permitted before timing.
