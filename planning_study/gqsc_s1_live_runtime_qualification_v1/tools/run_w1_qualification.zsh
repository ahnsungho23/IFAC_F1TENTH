#!/usr/bin/env zsh
set -e

if (( $# != 4 )); then
  print -u2 -- "usage: $0 A|B REPEAT_ID ATTEMPT0|RERUN1 OUTPUT_DIR"
  exit 64
fi

readonly CONDITION=$1
readonly REPEAT_ID=$2
readonly ATTEMPT_ID=$3
readonly OUTPUT_DIR=$4
[[ $CONDITION == A || $CONDITION == B ]] || exit 64
[[ $REPEAT_ID == <1-5> ]] || exit 64
[[ $ATTEMPT_ID == ATTEMPT0 || $ATTEMPT_ID == RERUN1 ]] || exit 64
[[ ! -e $OUTPUT_DIR ]] || { print -u2 -- "refusing to overwrite $OUTPUT_DIR"; exit 73; }
mkdir -p $OUTPUT_DIR

readonly REPO=/home/sungho/Documents/GitHub/2026_IFAC
readonly ROOT=$REPO/planning_study/gqsc_s1_live_runtime_qualification_v1
readonly HARNESS=$ROOT/release_overlay/_build/local_planning/p3_r3_k12_integration_harness
readonly EVENT=$REPO/planning_study/p3_geometry_conditioned_method_v1/success_control_inputs/SCE018.event
readonly TIMING=$OUTPUT_DIR/timing.tsv
readonly EXPECTED_CPUS=$([[ $CONDITION == B ]] && print 8 || print 0-23)
readonly EXPECTED_POWER=AC
readonly EXPECTED_GOVERNOR=powersave
typeset timeout_pid=''
typeset harness_pid=''

cleanup() {
  local pid
  for pid in $harness_pid $timeout_pid; do
    if [[ -n $pid ]] && kill -0 $pid 2>/dev/null; then
      kill -CONT $pid 2>/dev/null || true
      kill -TERM $pid 2>/dev/null || true
      wait $pid 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

power_state() {
  if on_ac_power; then
    print AC
    return
  fi
  local status=$?
  if (( status == 1 )); then
    print BATTERY
  else
    print UNKNOWN
  fi
}

record_system_context() {
  local phase=$1
  {
    print -- "phase=$phase"
    print -- "timestamp_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    print -- "power_source=$(power_state)"
    print -- "governor=$(sed -n '1p' /sys/devices/system/cpu/cpu8/cpufreq/scaling_governor)"
    print -- "loadavg=$(</proc/loadavg)"
    print -- "kernel=$(uname -r)"
    print -- "parent_affinity=$(taskset -pc $$)"
    print -- "relevant_processes_begin"
    ps -eo pid,ppid,pgid,psr,comm,args | grep -E \
      'p3_r3_k12_integration_harness|local_planner_node|sce018_replay_driver|gym_bridge|kinematic_localization' || true
    print -- "relevant_processes_end"
  } >>$OUTPUT_DIR/system_context.txt
}

[[ $(git -C $REPO branch --show-current) == research ]] || exit 65
[[ $(taskset -pc $$) == *'0-23' ]] || exit 65
[[ $(sed -n '1p' /sys/devices/system/cpu/cpu8/topology/thread_siblings_list) == 8-9 ]] || exit 65
print -r -- '572adb59ea24f3f06bed7502eb870e57a33a5416e8630106b67bc2b1d0b405bf  '$EVENT |
  sha256sum --check --status || exit 65
print -r -- '4fe480351a80135ff2a6e4592f661ff8a5670c032e12554d85065339d16ea960  '$REPO/src/local_planning/config/local_planning.yaml |
  sha256sum --check --status || exit 65
print -r -- '21e653cf9b063c9c60843c6ebaeb06fda918046bfe6d78a7d4ba6b957bdddd40  '$REPO/src/local_planning/include/local_planning/gqsc_s1_frozen_contract.hpp |
  sha256sum --check --status || exit 65
grep -Fq '670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776' \
  $REPO/src/local_planning/include/local_planning/gqsc_s1_frozen_contract.hpp || exit 65
print -r -- '49503d48d96cf408ad47691a69b683e5f2a0c02947a73283994f57b2697c0fd2  '$HARNESS |
  sha256sum --check --status || exit 65
[[ $(power_state) == $EXPECTED_POWER ]] || exit 66
[[ $(sed -n '1p' /sys/devices/system/cpu/cpu8/cpufreq/scaling_governor) == $EXPECTED_GOVERNOR ]] || exit 66
record_system_context START

if [[ $CONDITION == B ]]; then
  taskset -c 8 timeout --signal=TERM 600 \
    env GQSC_S1_MAIN_TIMING=1 GQSC_S1_TIMING_WARMUP=20 GQSC_S1_TIMING_REPEATS=200 \
    zsh -c 'kill -STOP $$; exec "$@"' zsh $HARNESS $EVENT \
    >$TIMING 2>$OUTPUT_DIR/stderr.log &
else
  timeout --signal=TERM 600 \
    env GQSC_S1_MAIN_TIMING=1 GQSC_S1_TIMING_WARMUP=20 GQSC_S1_TIMING_REPEATS=200 \
    zsh -c 'kill -STOP $$; exec "$@"' zsh $HARNESS $EVENT \
    >$TIMING 2>$OUTPUT_DIR/stderr.log &
fi
timeout_pid=$!

for _ in {1..500}; do
  for candidate in $(pgrep -P $timeout_pid 2>/dev/null || true); do
    if [[ $(awk '/^State:/{print $2}' /proc/$candidate/status 2>/dev/null || true) == T ]]; then
      harness_pid=$candidate
      break 2
    fi
  done
  kill -0 $timeout_pid 2>/dev/null || break
  sleep 0.005
done
if [[ -z $harness_pid ]]; then
  print -u2 -- "unable to capture live W1 harness PID before completion"
  wait $timeout_pid 2>/dev/null || true
  exit 67
fi

typeset actual_cpus=$(awk '/^Cpus_allowed_list:/{print $2}' /proc/$harness_pid/status)
{
  print -- "planner_pid=$harness_pid"
  print -- "pre_exec_gate=true"
  print -- "expected_executable=$HARNESS"
  print -- "pre_exec_executable=$(readlink -f /proc/$harness_pid/exe)"
  print -n -- "command_line="
  tr '\0' ' ' </proc/$harness_pid/cmdline
  print
  grep -E '^(Name|Pid|PPid|Cpus_allowed|Cpus_allowed_list):' /proc/$harness_pid/status
  taskset -pc $harness_pid
  ps -o pid,psr,comm,args -p $harness_pid
} >$OUTPUT_DIR/affinity.txt
[[ $actual_cpus == $EXPECTED_CPUS ]] || { print -u2 -- "W1 affinity mismatch"; kill -TERM $timeout_pid 2>/dev/null || true; exit 68; }

kill -CONT $harness_pid
typeset executable_verified=0
for _ in {1..500}; do
  if [[ $(readlink -f /proc/$harness_pid/exe 2>/dev/null || true) == $HARNESS ]]; then
    {
      print -- "actual_executable=$HARNESS"
      print -n -- "actual_command_line="
      tr '\0' ' ' </proc/$harness_pid/cmdline
      print
      grep -E '^(Name|Pid|PPid|Cpus_allowed|Cpus_allowed_list):' /proc/$harness_pid/status
      taskset -pc $harness_pid
      ps -o pid,psr,comm,args -p $harness_pid
    } >>$OUTPUT_DIR/affinity.txt
    executable_verified=1
    break
  fi
  kill -0 $harness_pid 2>/dev/null || break
  sleep 0.001
done
(( executable_verified == 1 )) || { print -u2 -- "W1 executable identity was not observed"; exit 67; }

set +e
wait $timeout_pid
typeset harness_status=$?
set -e
(( harness_status == 0 )) || { print -u2 -- "W1 qualification failed; evidence: $OUTPUT_DIR"; exit $harness_status; }

[[ $(grep -c '^GQSC_MAIN_TIMING' $TIMING) == 200 ]] || exit 69
[[ $(awk -F '\t' '$1=="GQSC_MAIN_TIMING" && ($2!="SCE018" || $11!=128 || $12!=12 || $13!=12 || $14!="FRESH_SELECTED"){bad++} END{print bad+0}' $TIMING) == 0 ]] || exit 69
[[ $(power_state) == $EXPECTED_POWER ]] || exit 66
[[ $(sed -n '1p' /sys/devices/system/cpu/cpu8/cpufreq/scaling_governor) == $EXPECTED_GOVERNOR ]] || exit 66
record_system_context END
print -- "W1_QUALIFICATION_CAPTURE_COMPLETE_UNINTERPRETED"
print -- "evidence=$OUTPUT_DIR"
