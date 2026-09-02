#!/usr/bin/env zsh
set -e
setopt TYPESET_SILENT

readonly REPO=/home/sungho/Documents/GitHub/2026_IFAC
readonly ROOT=$REPO/planning_study/gqsc_s1_live_runtime_qualification_v1
readonly TOOLS=$ROOT/tools
readonly HARNESS=$ROOT/release_overlay/_build/local_planning/p3_r3_k12_integration_harness
readonly EVENT=$REPO/planning_study/p3_geometry_conditioned_method_v1/success_control_inputs/SCE018.event
readonly TEMP_ROOT=$(mktemp -d /tmp/gqsc_s1_pid_affinity_preflight.XXXXXX)
typeset timeout_pid=''
typeset harness_pid=''

cleanup() {
  if [[ -n $harness_pid ]] && kill -0 $harness_pid 2>/dev/null; then
    kill -CONT $harness_pid 2>/dev/null || true
    kill -TERM $harness_pid 2>/dev/null || true
  fi
  if [[ -n $timeout_pid ]] && kill -0 $timeout_pid 2>/dev/null; then
    kill -TERM $timeout_pid 2>/dev/null || true
    wait $timeout_pid 2>/dev/null || true
  fi
  rm -rf -- $TEMP_ROOT
}
trap cleanup EXIT INT TERM

source $TOOLS/pid_evidence.zsh
source /opt/ros/humble/setup.zsh
source /home/sungho/sim_ws/install/setup.zsh
source $REPO/install/setup.zsh
source $TOOLS/_install/setup.zsh
source $ROOT/release_overlay/_install/setup.zsh

resolve_unique_stopped_child() {
  emulate -L zsh
  setopt TYPESET_SILENT
  local parent_pid=$1
  local attempt candidate state
  local -a candidates stopped
  for attempt in {1..500}; do
    candidates=($(pgrep -P $parent_pid 2>/dev/null || true))
    stopped=()
    for candidate in $candidates; do
      [[ $candidate == <-> && -d /proc/$candidate ]] || continue
      state=$(awk '/^State:/{print $2}' /proc/$candidate/status 2>/dev/null || true)
      if [[ $state == T ]] && taskset -pc $candidate >/dev/null 2>&1; then
        stopped+=($candidate)
      fi
    done
    if (( ${#stopped} == 1 )); then
      print -r -- $stopped[1]
      return 0
    fi
    if (( ${#stopped} > 1 )); then
      print -u2 -- "ambiguous stopped W1 child count: ${#stopped}"
      return 1
    fi
    kill -0 $parent_pid 2>/dev/null || return 1
    sleep 0.005
  done
  print -u2 -- "no stopped W1 child found"
  return 1
}

preflight_w1_condition() {
  local condition=$1
  local expected_cpus=$([[ $condition == B ]] && print 8 || print 0-23)
  local actual_cpus attempt

  if [[ $condition == B ]]; then
    taskset -c 8 timeout --signal=TERM 30 \
      env -u GQSC_S1_MAIN_TIMING -u GQSC_S1_STAGE_TIMING \
      zsh -c 'kill -STOP $$; exec "$@"' zsh $HARNESS $EVENT \
      >/dev/null 2>/dev/null &
  else
    timeout --signal=TERM 30 \
      env -u GQSC_S1_MAIN_TIMING -u GQSC_S1_STAGE_TIMING \
      zsh -c 'kill -STOP $$; exec "$@"' zsh $HARNESS $EVENT \
      >/dev/null 2>/dev/null &
  fi
  timeout_pid=$!
  harness_pid=$(resolve_unique_stopped_child $timeout_pid)
  [[ $harness_pid == <-> ]]
  actual_cpus=$(awk '/^Cpus_allowed_list:/{print $2}' /proc/$harness_pid/status)
  [[ $actual_cpus == $expected_cpus ]]
  kill -CONT $harness_pid
  for attempt in {1..500}; do
    if _pid_matches_expected_process_quiet $harness_pid $HARNESS; then
      kill -STOP $harness_pid
      break
    fi
    kill -0 $harness_pid 2>/dev/null || return 1
    sleep 0.001
  done
  _pid_matches_expected_process_quiet $harness_pid $HARNESS
  verify_resolved_pid $harness_pid $HARNESS
  actual_cpus=$(awk '/^Cpus_allowed_list:/{print $2}' /proc/$harness_pid/status)
  [[ $actual_cpus == $expected_cpus ]]
  kill -TERM $harness_pid 2>/dev/null || true
  kill -CONT $harness_pid 2>/dev/null || true
  kill -TERM $timeout_pid 2>/dev/null || true
  wait $timeout_pid 2>/dev/null || true
  timeout_pid=''
  harness_pid=''
  print -- "W1_PID_AFFINITY_PREFLIGHT_PASS condition=$condition cpus=$expected_cpus"
}

$TOOLS/test_pid_resolver.zsh
preflight_w1_condition A
preflight_w1_condition B

typeset condition runner workload
for condition in A B; do
  GQSC_S1_PID_PREFLIGHT_ONLY=1 $TOOLS/run_w2_qualification.zsh \
    $condition 1 ATTEMPT0 $TEMP_ROOT/W2_$condition
done
for condition in A B; do
  GQSC_S1_PID_PREFLIGHT_ONLY=1 $TOOLS/run_w3_qualification.zsh \
    $condition 1 ATTEMPT0 $TEMP_ROOT/W3_$condition
done

if pgrep -af '[p]3_r3_k12_integration_harness|[l]ocal_planner_node|[s]ce018_replay_driver|[g]ym_bridge_launch|[k]inematic_localization.launch|[g]lobal_planning.launch|[o]bstacle_detector.launch|[s]tate_machine.launch|[c]ontrol_sim.launch'; then
  print -u2 -- "PID preflight left an experiment-owned process"
  exit 1
fi

print W1_W2_W3_PID_AFFINITY_PREFLIGHT_PASS
