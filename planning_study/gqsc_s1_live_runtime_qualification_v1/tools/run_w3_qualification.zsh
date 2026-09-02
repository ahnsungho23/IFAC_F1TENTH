#!/usr/bin/env zsh
set -e
setopt TYPESET_SILENT

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
readonly PID_PREFLIGHT_ONLY=${GQSC_S1_PID_PREFLIGHT_ONLY:-0}
[[ $PID_PREFLIGHT_ONLY == 0 || $PID_PREFLIGHT_ONLY == 1 ]] || exit 64
export ROS_DOMAIN_ID=83

typeset -a process_groups
typeset -a background_groups
typeset -a background_labels
typeset driver_pid=''
typeset planner_pid=''
typeset driver_group_pid=''
typeset planner_group_pid=''

source $TOOLS/pid_evidence.zsh

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

record_pid_evidence() {
  local role=$1
  local pid=$2
  local expected_executable=$3
  local expected_cpus=$4
  verify_resolved_pid $pid $expected_executable || return 1
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

record_background_group_evidence() {
  local label=$1
  local group_id=$2
  local -a members
  members=($(ps -eo pid=,pgid= | awk -v group_id=$group_id '$2 == group_id {print $1}'))
  (( ${#members} > 0 )) || return 1
  {
    print -- "background_group=$label"
    print -- "pgid=$group_id"
  } >>$OUTPUT_DIR/affinity.txt
  local pid
  for pid in $members; do
    local actual_cpus=$(awk '/^Cpus_allowed_list:/{print $2}' /proc/$pid/status)
    {
      print -- "member_pid=$pid"
      print -- "member_executable=$(readlink -f /proc/$pid/exe)"
      print -n -- "member_command_line="
      tr '\0' ' ' </proc/$pid/cmdline
      print
      grep -E '^(Name|Pid|PPid|Cpus_allowed|Cpus_allowed_list):' /proc/$pid/status
      taskset -pc $pid
      ps -o pid,psr,comm,args -p $pid
    } >>$OUTPUT_DIR/affinity.txt
    [[ $actual_cpus == $BACKGROUND_CPUS ]] || return 1
  done
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
      'local_planner_node|sce018_replay_driver|gym_bridge|kinematic_localization|global_trajectory|frenet_odom|obstacle_detector|state_machine|control_map|cruise_controller|sim_imu_bridge|drive_source_selector' || true
    print -- "relevant_processes_end"
  } >>$OUTPUT_DIR/system_context.txt
}

start_background() {
  local label=$1
  shift
  setsid taskset -c $BACKGROUND_CPUS "$@" >$OUTPUT_DIR/$label.log 2>&1 &
  process_groups+=($!)
  background_groups+=($!)
  background_labels+=($label)
}

start_planner() {
  if [[ $CONDITION == B ]]; then
    setsid taskset -c 8 "$@" >$OUTPUT_DIR/planner.log 2>&1 &
  else
    setsid "$@" >$OUTPUT_DIR/planner.log 2>&1 &
  fi
  planner_group_pid=$!
  process_groups+=($planner_group_pid)
}

stop_group() {
  local pid=$1
  kill -0 $pid 2>/dev/null || return 0
  kill -INT -- -$pid 2>/dev/null || true
  for _ in {1..20}; do
    kill -0 $pid 2>/dev/null || return 0
    sleep 0.1
  done
  kill -TERM -- -$pid 2>/dev/null || true
  for _ in {1..10}; do
    kill -0 $pid 2>/dev/null || return 0
    sleep 0.1
  done
  kill -KILL -- -$pid 2>/dev/null || true
}

cleanup() {
  local pid
  if [[ -n $driver_group_pid ]]; then
    stop_group $driver_group_pid
  fi
  for pid in ${(Oa)process_groups}; do
    stop_group $pid
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
print -r -- '49f9e5c1ae2d1d34c7fede4189e98a65f8d2977ae5ace0562735ffd28d9bca2f  /home/sungho/sim_ws/src/f1tenth_gym_ros/config/sim.yaml' |
  sha256sum --check --status || exit 65
print -r -- 'a3f7f679b19d8598feea4a34944b5667a222109c78cc126e4e540f8a26ca0f86  /home/sungho/sim_ws/src/f1tenth_gym_ros/launch/gym_bridge_launch.py' |
  sha256sum --check --status || exit 65
print -r -- '1f1fee4eef4e0130cc92cebfc4f9148707fe6ea0e65c7f3358f00d4724fd9a00  '$REPO/src/kinematic_localization/maps/ifac_track.png |
  sha256sum --check --status || exit 65
print -r -- '7625011637ad9bd506d6d19d3bb990f2ae1c6be72ba76fa04f624f56d3f160f1  '$REPO/src/kinematic_localization/maps/ifac_track.yaml |
  sha256sum --check --status || exit 65
print -r -- '65620cf48437e8ec4b0c8d21f976bbe3517d08da868e93c015b2efbd8030ff28  '$REPO/src/kinematic_localization/maps/ifac_track.kissmap |
  sha256sum --check --status || exit 65
print -r -- 'b9cdd21fb7067ebd32b9ce2f4d2ffbd8affc96c5d2193eaba02892a772a16fad  '$REPO/src/global_planning/data/ifac_track/global_waypoints.json |
  sha256sum --check --status || exit 65
print -r -- '044fa7d6c458b6e75679260d5108fc23f982e01645a9c4851137dce2a938f009  '$REPO/src/global_planning/launch/global_planning.launch.py |
  sha256sum --check --status || exit 65
print -r -- '866bb0c1bc7fa9a8f46cfc202e2c6aea1e39b84e0cc527aace8bfcff4d9346b5  '$REPO/src/kinematic_localization/launch/kinematic_localization.launch.py |
  sha256sum --check --status || exit 65
print -r -- 'f4b83044811faa6b6a9c023d7adf410dfdf82030f2fde727af71397974723217  '$REPO/src/obstacle_detector/launch/obstacle_detector.launch.py |
  sha256sum --check --status || exit 65
print -r -- '4a97827241a0334d9c7d1c7eeb0d14167a1e29e5c354a8d357f0c13efa471c19  '$REPO/src/state_machine/launch/state_machine.launch.py |
  sha256sum --check --status || exit 65
print -r -- '99a775ca302ca8baf479d6bcc157c41b401c61ea34f4f9bb2255eeedda02687c  '$REPO/src/f1tenth_control/launch/control_sim.launch.py |
  sha256sum --check --status || exit 65
[[ $(power_state) == $EXPECTED_POWER ]] || exit 66
[[ $(sed -n '1p' /sys/devices/system/cpu/cpu8/cpufreq/scaling_governor) == $EXPECTED_GOVERNOR ]] || exit 66

unset SNAP SNAP_ARCH SNAP_COMMON SNAP_CONTEXT SNAP_COOKIE SNAP_DATA SNAP_EUID
unset SNAP_INSTANCE_NAME SNAP_LAUNCHER_ARCH_TRIPLET SNAP_LIBRARY_PATH SNAP_NAME SNAP_REAL_HOME
unset SNAP_REVISION SNAP_UID SNAP_USER_COMMON SNAP_USER_DATA SNAP_VERSION
unset GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE GIO_LAUNCHED_DESKTOP_FILE GIO_MODULE_DIR
unset GSETTINGS_SCHEMA_DIR GTK_EXE_PREFIX GTK_IM_MODULE_FILE GTK_MODULES GTK_PATH LOCPATH
export XDG_DATA_DIRS=${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-/usr/local/share/:/usr/share/:/var/lib/snapd/desktop}
export XDG_DATA_HOME=$HOME/.local/share

start_background simulator ros2 launch f1tenth_gym_ros gym_bridge_launch.py
sleep 4
start_background localization ros2 launch kinematic_localization kinematic_localization.launch.py \
  map_name:=ifac_track use_sim_time:=false map_frame:=map \
  base_frame:=ego_racecar/base_link odom_topic:=/ego_racecar/odom \
  map_topic:=/kinematic_localization/map auto_init_from_waypoints:=false
sleep 3
timeout 8 ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: -0.427, y: 0.456, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.3651, w: 0.9310}}}}' \
  >$OUTPUT_DIR/initialpose.log 2>&1
start_background global env F1_MAP=ifac_track ros2 launch global_planning global_planning.launch.py \
  map_name:=ifac_track
sleep 3
start_background detector ros2 launch obstacle_detector obstacle_detector.launch.py \
  simulator:=true use_sim_time:=false rviz:=false
start_planner ros2 run local_planning local_planner_node --ros-args \
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
  -r /local_planning/live_runtime_profile:=/gqsc_runtime/live_profile
start_background state ros2 launch state_machine state_machine.launch.py
sleep 1
start_background control ros2 launch f1tenth_control control_sim.launch.py
sleep 5

typeset nodes=''
typeset ready=0
for _ in {1..30}; do
  nodes=$(ros2 node list 2>/dev/null || true)
  if [[ $nodes == *'/bridge'* && $nodes == *'/rviz'* &&
        $nodes == *'/kinematic_localization'* &&
        $nodes == *'/global_trajectory_publisher_node'* &&
        $nodes == *'/frenet_odom_node'* && $nodes == *'/obstacle_detector'* &&
        $nodes == *'/local_planner_node'* && $nodes == *'/state_machine_node'* &&
        $nodes == *'/control_map_node'* && $nodes == *'/cruise_controller_node'* &&
        $nodes == *'/sim_imu_bridge_node'* && $nodes == *'/drive_source_selector'* ]]; then
    ready=1
    break
  fi
  sleep 0.5
done
if (( ! ready )); then
  print -r -- $nodes >$OUTPUT_DIR/nodes_failed.txt
  exit 20
fi

planner_pid=$(resolve_unique_group_executable $planner_group_pid $RELEASE_NODE) || exit 67
[[ $planner_pid == <-> ]] || exit 67
record_pid_evidence PLANNER $planner_pid $RELEASE_NODE $EXPECTED_PLANNER_CPUS || exit 68
typeset group_index
for (( group_index = 1; group_index <= ${#background_groups}; ++group_index )); do
  record_background_group_evidence \
    $background_labels[$group_index] $background_groups[$group_index] || exit 68
done
[[ $(power_state) == $EXPECTED_POWER ]] || exit 66
[[ $(sed -n '1p' /sys/devices/system/cpu/cpu8/cpufreq/scaling_governor) == $EXPECTED_GOVERNOR ]] || exit 66
record_system_context START

if (( PID_PREFLIGHT_ONLY )); then
  record_system_context END
  print -- "W3_PID_AFFINITY_PREFLIGHT_PASS condition=$CONDITION pid=$planner_pid cpus=$EXPECTED_PLANNER_CPUS background=$BACKGROUND_CPUS"
  exit 0
fi

setsid taskset -c $BACKGROUND_CPUS ros2 run gqsc_runtime_replay sce018_replay_driver --ros-args \
  -p event_file:=$EVENT \
  -p output_path:=$RESULT \
  -p workload_id:=W3_SCE018_FULL_STACK_CONTENTION \
  -p condition:=$CONDITION \
  -p repeat_id:=$REPEAT_ID \
  -p attempt_id:=$ATTEMPT_ID \
  -p target_callbacks:=220 \
  -p warmup_callbacks:=20 \
  -p minimum_source_epochs:=220 \
  -p smoke_mode:=false \
  -p preflight_hold_sec:=3.0 \
  -p timeout_sec:=900.0 \
  >$OUTPUT_DIR/driver.log 2>&1 &
driver_group_pid=$!

driver_pid=$(resolve_unique_group_executable \
  $driver_group_pid $TOOLS/_install/lib/gqsc_runtime_replay/sce018_replay_driver) || exit 67
[[ $driver_pid == <-> ]] || exit 67
record_pid_evidence REPLAY_DRIVER $driver_pid \
  $TOOLS/_install/lib/gqsc_runtime_replay/sce018_replay_driver $BACKGROUND_CPUS || exit 68
sleep 2
{
  git -C $REPO status --short --branch
  git -C $REPO rev-parse HEAD
  taskset -pc $planner_pid
  taskset -pc $driver_pid
  lscpu -e=CPU,CORE,SOCKET,NODE,ONLINE,MAXMHZ,MINMHZ
  sed -n '1p' /sys/devices/system/cpu/cpu8/topology/thread_siblings_list
  print -- '=== NODES ==='
  ros2 node list
  for topic in global_waypoints confirmed_static_obs frenet_odom state p3_cycle live_profile; do
    print -- "=== /gqsc_runtime/$topic ==="
    ros2 topic info --verbose /gqsc_runtime/$topic
  done
  print -- '=== NORMAL DETECTOR OUTPUT ==='
  ros2 topic info --verbose /confirmed_static_obs
  print -- '=== NORMAL SIM ODOMETRY ==='
  ros2 topic info --verbose /ego_racecar/odom
} >$GRAPH

set +e
wait $driver_group_pid
typeset driver_status=$?
set -e
driver_pid=''
driver_group_pid=''
if (( driver_status != 0 )); then
  print -u2 -- "W3 qualification failed; evidence: $OUTPUT_DIR"
  exit $driver_status
fi

grep -Fq '"kind":"complete","joined_callbacks":220,"warmup_callbacks":20,"measurement_callbacks":200,"distinct_source_epochs":220,"parity":"PASS"' $RESULT
[[ $(power_state) == $EXPECTED_POWER ]] || exit 66
[[ $(sed -n '1p' /sys/devices/system/cpu/cpu8/cpufreq/scaling_governor) == $EXPECTED_GOVERNOR ]] || exit 66
record_system_context END
print -- "W3_QUALIFICATION_CAPTURE_COMPLETE_UNINTERPRETED"
print -- "evidence=$OUTPUT_DIR"
