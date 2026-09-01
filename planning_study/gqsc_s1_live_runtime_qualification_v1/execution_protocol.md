# Execution protocol

## Global stop rule

This file is a future-run contract, not authorization to run the A/B experiment. At this
checkpoint the global gate is blocked. W1 commands below are exact but must not be used to start a
partial experiment while W2/W3 remain blocked. W2/W3 deliberately contain no placeholder command.

Any hash, branch, topic schema, CPU topology, or row-status mismatch stops the run. Do not rebuild,
edit configuration, substitute a path, or change the A/B order inside a run.

## Environment bootstrap

Every shell must use zsh without `set -u`, then execute exactly:

```zsh
source /opt/ros/humble/setup.zsh
source /home/sungho/sim_ws/install/setup.zsh
source /home/sungho/Documents/GitHub/2026_IFAC/install/setup.zsh
cd /home/sungho/Documents/GitHub/2026_IFAC
```

The simulator overlay is sourced before the IFAC overlay. The following package prefixes must be
discoverable before a ROS-layer run:

```zsh
ros2 pkg prefix local_planning
ros2 pkg prefix global_planning
ros2 pkg prefix kinematic_localization
ros2 pkg prefix f1tenth_gym_ros
```

Expected prefixes are the current IFAC isolated install for the first three and
`/home/sungho/sim_ws/install/f1tenth_gym_ros` for the simulator package. The prior package failure
was an unsourced/incorrectly nounset shell, not evidence requiring a rebuild.

## Common preflight

Before any future run, record but do not repair:

```zsh
git status --short --branch
git rev-parse HEAD
sha256sum src/local_planning/include/local_planning/gqsc_s1_frozen_contract.hpp
sha256sum src/local_planning/config/local_planning.yaml
sha256sum src/local_planning/src/path_digest.cpp
lscpu -e=CPU,CORE,SOCKET,NODE,ONLINE,MAXMHZ,MINMHZ
sed -n '1p' /sys/devices/system/cpu/cpu8/topology/thread_siblings_list
sed -n '1p' /sys/devices/system/cpu/cpu8/cpufreq/scaling_governor
```

The frozen host topology is Intel i7-14650HX, 24 online logical CPUs; CPU 8 has maximum 5200 MHz,
core 4, and sibling set `8-9`. Condition B is exactly `taskset -c 8`. No experiment-owned process
may be assigned to CPU 9 during B. The observed governor is `powersave`; changing it is forbidden
inside this protocol. A topology or governor mismatch blocks the run instead of selecting a new
CPU after seeing data.

## W1

W1's primary metric is the final `callback_wall_us` field of each `GQSC_MAIN_TIMING` row. Event
parsing and planner construction happen before the timing loop. `evaluation_wall_us` and the
component fields are secondary diagnostic columns only.

Before and after every A/B pair, run this untimed parity command and retain its `SUMMARY`,
`INTEGRATED_SELECTED`, and `DOWNSTREAM_SELECTED` rows. Its profile timings are not part of the A/B
dataset:

```zsh
env -u GQSC_S1_MAIN_TIMING -u GQSC_S1_STAGE_TIMING \
  build/local_planning/p3_r3_k12_integration_harness \
  planning_study/p3_geometry_conditioned_method_v1/success_control_inputs/SCE018.event
```

Condition A command for each scheduled repeat is:

```zsh
env GQSC_S1_MAIN_TIMING=1 GQSC_S1_TIMING_WARMUP=20 GQSC_S1_TIMING_REPEATS=200 \
  build/local_planning/p3_r3_k12_integration_harness \
  planning_study/p3_geometry_conditioned_method_v1/success_control_inputs/SCE018.event
```

Condition B command for each scheduled repeat is:

```zsh
taskset -c 8 env GQSC_S1_MAIN_TIMING=1 GQSC_S1_TIMING_WARMUP=20 \
  GQSC_S1_TIMING_REPEATS=200 \
  build/local_planning/p3_r3_k12_integration_harness \
  planning_study/p3_geometry_conditioned_method_v1/success_control_inputs/SCE018.event
```

The caller must save stdout/stderr under a unique immutable name composed from the exact
`workload_id`, repeat, and condition in `run_schedule.csv`; it must never overwrite a prior
attempt. Exit 0, exactly 200 timing rows, finite non-negative callback values, expected counts,
and expected outcome are required.

## W2

No exact command is frozen. Do not reuse the historical ROS_NODE_ISOLATED command because it used
`SHADOW`, while this workload requires `TEST_ACTIVE`. A command may be added only after the neutral
external driver requirements in `workload_freeze.md` are implemented. The future exact procedure
must identify:

- the actual `ros2 launch local_planning local_planning.launch.py` invocation and the sole
  profiling parameter override;
- publishers for `/global_waypoints`, `/car_state/frenet/odom`, `/confirmed_static_obs`, and any
  required state topic, with exact message files/hashes and QoS;
- monotonic timestamp/sequence production and the deterministic fresh-lifecycle mechanism;
- collection from `/local_planning/live_runtime_profile` plus cycle diagnostic, joined by
  `callback_sequence`; and
- startup-ready, warm-up-start, measurement-start, stop, and shutdown predicates.

Until that command produces 20 warm-up and 200 post-warm-up `TEST_ACTIVE` fresh profiles without
changing input geometry, W2 is not runnable for qualification.

## W3

No exact command is frozen. `sim/run.sh` currently exposes the real stack roles `f1sim`, `mcl`,
`global`, `local`, `state`, and `control`, but listing those roles is not an exact S01 experiment.
The missing historical path and scheduler mean that executing the current stack would create a
different workload.

The future procedure must retain and hash the simulator config, map and path bytes, exact role
start order and ready checks, localization initial pose/TF adapter, S01 obstacle scheduler, seed
and noise, speed cap, profiling override, collection predicate, and shutdown order. Collection may
begin only after localization/global reference are stable and the deterministic S01 scheduler has
entered its predeclared start state; it may not begin at an operator-selected visually favorable
moment.

## Warm-up, samples, timeout, retries, and failures

- Warm-up: exactly 20 eligible fresh callbacks per condition/repeat, discarded and retained in
  raw evidence with `phase=WARMUP`.
- Measurement: exactly the next 200 eligible fresh callbacks. Additional eligible callbacks are
  protocol violations, not a pool from which to select 200.
- Repeats: five per condition and workload.
- Timeout from workload-ready state: W1 600 s, W2 600 s, W3 900 s.
- Invalid/rejected callbacks do not replace the next eligible callback silently; their identity
  and reason are retained. Collection continues until 200 valid samples or timeout.
- A repeat with fewer than 200 valid samples, parity failure, nonzero process exit, missing raw
  evidence, or unexpected extra eligible sample is failed.
- Maximum reruns: one per failed repeat, only for a recorded infrastructure/protocol failure.
  Never rerun because of a latency value. Preserve the failed attempt and name the single rerun
  `RERUN1`.
- If the rerun fails, or if more than one repeat needs a second attempt, that workload fails. Do
  not pool attempts or report fewer than five complete repeats.
- Planner outcomes/failures that performed a valid fresh evaluation remain in latency statistics
  and are classified; they are not invalidated merely for being slow or unsuccessful.

## A/B order

`run_schedule.csv` is binding independently for each workload: repeat 1 A/B, repeat 2 B/A,
repeat 3 A/B, repeat 4 B/A, repeat 5 A/B. A is unpinned; B is CPU 8 pinned. The schedule cannot be
changed after any performance output exists. Blocked rows remain `DO_NOT_RUN` until the global gate
is re-audited.
