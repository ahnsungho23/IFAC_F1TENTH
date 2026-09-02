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
readonly TOOLS=$ROOT/tools
readonly RELEASE_INSTALL=$ROOT/release_overlay/_install
readonly RELEASE_NODE=$RELEASE_INSTALL/local_planning/lib/local_planning/local_planner_node
readonly EVENT=$REPO/planning_study/p3_geometry_conditioned_method_v1/success_control_inputs/SCE018.event
readonly RESULT=$OUTPUT_DIR/joined_callbacks.jsonl
readonly GRAPH=$OUTPUT_DIR/ros_graph.txt
readonly BACKGROUND_CPUS=0-7,10-23
readonly EXPECTED_PLANNER_CPUS=$([[ $CONDITION == B ]] && print 8 || print 0-23)
readonly EXPECTED_POWER=AC
readonly EXPECTED_GOVERNOR=powersave
export ROS_DOMAIN_ID=82

typeset planner_pid=''
typeset driver_pid=''
typeset planner_process_pid=''
typeset driver_process_pid=''

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

find_child_executable() {
  local parent_pid=$1
  local expected=$2
  for _ in {1..500}; do
    local candidate
    for candidate in $(pgrep -P $parent_pid 2>/dev/null || true); do
      if [[ $(readlink -f /proc/$candidate/exe 2>/dev/null || true) == $expected ]]; then
        print -- $candidate
        return 0
      fi
    done
    kill -0 $parent_pid 2>/dev/null || return 1
    sleep 0.01
  done
  return 1
}

record_pid_evidence() {
  local role=$1
  local pid=$2
  local expected_executable=$3
  local expected_cpus=$4
  local actual_executable=$(readlink -f /proc/$pid/exe)
  local actual_cpus=$(awk '/^Cpus_allowed_list:/{print $2}' /proc/$pid/status)
  {
    print -- "role=$role"
    print -- "pid=$pid"
    print -- "expected_executable=$expected_executable"
    print -- "actual_executable=$actual_executable"
    print -n -- "command_line="
    tr '\0' ' ' </proc/$pid/cmdline
    print
    grep -E '^(Name|Pid|PPid|Cpus_allowed|Cpus_allowed_list):' /proc/$pid/status
    taskset -pc $pid
    ps -o pid,psr,comm,args -p $pid
  } >>$OUTPUT_DIR/affinity.txt
  [[ $actual_executable == $expected_executable && $actual_cpus == $expected_cpus ]]
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
    print -- "relevant_processes_begin"
    ps -eo pid,ppid,pgid,psr,comm,args | grep -E \
      'local_planner_node|sce018_replay_driver|gym_bridge|kinematic_localization' || true
    print -- "relevant_processes_end"
  } >>$OUTPUT_DIR/system_context.txt
}

cleanup() {
  local pid
  for pid in $driver_process_pid $driver_pid $planner_process_pid $planner_pid; do
    if [[ -n $pid ]] && kill -0 $pid 2>/dev/null; then
      kill -INT $pid 2>/dev/null || true
      for _ in {1..20}; do
        kill -0 $pid 2>/dev/null || break
        sleep 0.1
      done
      kill -TERM $pid 2>/dev/null || true
      wait $pid 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.zsh
source /home/sungho/sim_ws/install/setup.zsh
source $REPO/install/setup.zsh
source $TOOLS/_install/setup.zsh
source $RELEASE_INSTALL/setup.zsh

[[ $(git -C $REPO branch --show-current) == research ]] || exit 65
[[ $(ros2 pkg prefix local_planning) == $RELEASE_INSTALL/local_planning ]] || exit 65
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
print -r -- '54019a86e13f4dc25771628f7a3d385be8e2c6ce2a657448b3a5939823ab878a  '$RELEASE_NODE |
  sha256sum --check --status || exit 65
print -r -- 'ab014c1e56fd03189327d418e63c88c3295b150936037c7dc65e6d1905b8251b  '$TOOLS/gqsc_runtime_replay/src/sce018_replay_driver.cpp |
  sha256sum --check --status || exit 65
print -r -- '49c5fd04c105883c744f470e810fb0811fa7d36cd7094c069b2e264c3ba8cafb  '$TOOLS/_install/lib/gqsc_runtime_replay/sce018_replay_driver |
  sha256sum --check --status || exit 65
[[ $(power_state) == $EXPECTED_POWER ]] || exit 66
[[ $(sed -n '1p' /sys/devices/system/cpu/cpu8/cpufreq/scaling_governor) == $EXPECTED_GOVERNOR ]] || exit 66

typeset -a planner_prefix
if [[ $CONDITION == B ]]; then
  planner_prefix=(taskset -c 8)
else
  planner_prefix=()
fi

$planner_prefix ros2 run local_planning local_planner_node --ros-args \
  --params-file $REPO/src/local_planning/config/local_planning.yaml \
  -p live_runtime_profiling_enable:=true \
  -p p3_diagnostics_detail:=FULL \
  -r /global_waypoints:=/gqsc_runtime/global_waypoints \
  -r /confirmed_static_obs:=/gqsc_runtime/confirmed_static_obs \
  -r /static_obs:=/gqsc_runtime/raw_static_obs \
  -r /car_state/frenet/odom:=/gqsc_runtime/frenet_odom \
  -r /state:=/gqsc_runtime/state \
  -r /avoid_waypoints:=/gqsc_runtime/avoid_waypoints \
  -r /local_planning/path:=/gqsc_runtime/path \
  -r /local_planning/p3_shadow:=/gqsc_runtime/p3_cycle \
  -r /local_planning/live_runtime_profile:=/gqsc_runtime/live_profile \
  >$OUTPUT_DIR/planner.log 2>&1 &
planner_pid=$!

planner_process_pid=$(find_child_executable $planner_pid $RELEASE_NODE) || exit 67
record_pid_evidence PLANNER $planner_process_pid $RELEASE_NODE $EXPECTED_PLANNER_CPUS || exit 68
record_system_context START

taskset -c $BACKGROUND_CPUS ros2 run gqsc_runtime_replay sce018_replay_driver --ros-args \
  -p event_file:=$EVENT \
  -p output_path:=$RESULT \
  -p workload_id:=W2_SCE018_ROS_NODE_TEST_ACTIVE \
  -p condition:=$CONDITION \
  -p repeat_id:=$REPEAT_ID \
  -p attempt_id:=$ATTEMPT_ID \
  -p target_callbacks:=220 \
  -p warmup_callbacks:=20 \
  -p minimum_source_epochs:=220 \
  -p smoke_mode:=false \
  -p preflight_hold_sec:=3.0 \
  -p timeout_sec:=600.0 \
  >$OUTPUT_DIR/driver.log 2>&1 &
driver_pid=$!

driver_process_pid=$(find_child_executable \
  $driver_pid $TOOLS/_install/lib/gqsc_runtime_replay/sce018_replay_driver) || exit 67
record_pid_evidence REPLAY_DRIVER $driver_process_pid \
  $TOOLS/_install/lib/gqsc_runtime_replay/sce018_replay_driver $BACKGROUND_CPUS || exit 68
sleep 2
{
  git -C $REPO status --short --branch
  git -C $REPO rev-parse HEAD
  taskset -pc $planner_process_pid
  taskset -pc $driver_process_pid
  lscpu -e=CPU,CORE,SOCKET,NODE,ONLINE,MAXMHZ,MINMHZ
  sed -n '1p' /sys/devices/system/cpu/cpu8/topology/thread_siblings_list
  print -- '=== NODES ==='
  ros2 node list
  for topic in global_waypoints confirmed_static_obs frenet_odom state p3_cycle live_profile; do
    print -- "=== /gqsc_runtime/$topic ==="
    ros2 topic info --verbose /gqsc_runtime/$topic
  done
} >$GRAPH

set +e
wait $driver_pid
typeset driver_status=$?
set -e
driver_pid=''
driver_process_pid=''
if (( driver_status != 0 )); then
  print -u2 -- "W2 qualification failed; evidence: $OUTPUT_DIR"
  exit $driver_status
fi

grep -Fq '"kind":"complete","joined_callbacks":220,"warmup_callbacks":20,"measurement_callbacks":200,"distinct_source_epochs":220,"parity":"PASS"' $RESULT
[[ $(power_state) == $EXPECTED_POWER ]] || exit 66
[[ $(sed -n '1p' /sys/devices/system/cpu/cpu8/cpufreq/scaling_governor) == $EXPECTED_GOVERNOR ]] || exit 66
record_system_context END
print -- "W2_QUALIFICATION_CAPTURE_COMPLETE_UNINTERPRETED"
print -- "evidence=$OUTPUT_DIR"
