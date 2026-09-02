#!/usr/bin/env zsh
set -e

readonly REPO=/home/sungho/Documents/GitHub/2026_IFAC
readonly ROOT=$REPO/planning_study/gqsc_s1_live_runtime_qualification_v1
readonly TOOLS=$ROOT/tools
readonly RELEASE_INSTALL=$ROOT/release_overlay/_install
readonly RELEASE_NODE=$RELEASE_INSTALL/local_planning/lib/local_planning/local_planner_node
readonly RELEASE_NODE_SHA=54019a86e13f4dc25771628f7a3d385be8e2c6ce2a657448b3a5939823ab878a
readonly EVENT=$REPO/planning_study/p3_geometry_conditioned_method_v1/success_control_inputs/SCE018.event
readonly OUTPUT_DIR=$(mktemp -d /tmp/gqsc_s1_w3_smoke.XXXXXX)
readonly RESULT=$OUTPUT_DIR/joined_smoke.jsonl
readonly GRAPH=$OUTPUT_DIR/ros_graph.txt
export ROS_DOMAIN_ID=83

typeset -a process_groups
typeset driver_pid=''

start_group() {
  local label=$1
  shift
  setsid "$@" >$OUTPUT_DIR/$label.log 2>&1 &
  process_groups+=($!)
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
  if [[ -n $driver_pid ]]; then
    stop_group $driver_pid
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

[[ $(ros2 pkg prefix local_planning) == $RELEASE_INSTALL/local_planning ]] || exit 65
print -r -- "$RELEASE_NODE_SHA  $RELEASE_NODE" | sha256sum --check --status || exit 65

# Codex is launched from a confined VS Code snap. Its GTK/GIO/locale variables make the host ROS
# RViz load core20's private glibc. Remove only those inherited GUI variables in this process tree;
# no persistent user or system setting is changed.
unset SNAP SNAP_ARCH SNAP_COMMON SNAP_CONTEXT SNAP_COOKIE SNAP_DATA SNAP_EUID
unset SNAP_INSTANCE_NAME SNAP_LAUNCHER_ARCH_TRIPLET SNAP_LIBRARY_PATH SNAP_NAME SNAP_REAL_HOME
unset SNAP_REVISION SNAP_UID SNAP_USER_COMMON SNAP_USER_DATA SNAP_VERSION
unset GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE GIO_LAUNCHED_DESKTOP_FILE GIO_MODULE_DIR
unset GSETTINGS_SCHEMA_DIR GTK_EXE_PREFIX GTK_IM_MODULE_FILE GTK_MODULES GTK_PATH LOCPATH
export XDG_DATA_DIRS=${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-/usr/local/share/:/usr/share/:/var/lib/snapd/desktop}
export XDG_DATA_HOME=$HOME/.local/share

# Current sim/run.sh f1sim role, including its normal bridge/map/RViz/robot-state process set.
start_group simulator ros2 launch f1tenth_gym_ros gym_bridge_launch.py
sleep 4

# Current sim/run.sh mcl role.
start_group localization ros2 launch kinematic_localization kinematic_localization.launch.py \
  map_name:=ifac_track \
  use_sim_time:=false \
  map_frame:=map \
  base_frame:=ego_racecar/base_link \
  odom_topic:=/ego_racecar/odom \
  map_topic:=/kinematic_localization/map \
  auto_init_from_waypoints:=false
sleep 3

# Use the repository's documented ifac_track headless start pose. Both gym and KICP subscribe to
# /initialpose, so this teleports the simulator and initializes localization consistently.
timeout 8 ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: -0.427, y: 0.456, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.3651, w: 0.9310}}}}' \
  >$OUTPUT_DIR/initialpose.log 2>&1

# Current sim/run.sh global role and installed ifac_track path bundle.
start_group global env F1_MAP=ifac_track ros2 launch global_planning global_planning.launch.py \
  map_name:=ifac_track
sleep 3

# Perception load normally included by local_planning.launch.py. The simulator map server already
# owns /map, so launch only the real detector here and keep its outputs off qualification topics.
start_group detector ros2 launch obstacle_detector obstacle_detector.launch.py \
  simulator:=true use_sim_time:=false rviz:=false

# The single local planner in this W3 process set. Only its planner inputs/outputs and research
# streams are isolated; algorithm/config/mode remain production values.
start_group planner ros2 run local_planning local_planner_node --ros-args \
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

# Current sim/run.sh state/control roles. They consume the normal full-stack topics, not the
# private qualification inputs or outputs.
start_group state ros2 launch state_machine state_machine.launch.py
sleep 1
start_group control ros2 launch f1tenth_control control_sim.launch.py
sleep 5

typeset nodes=''
typeset ready=0
for _ in {1..30}; do
  nodes=$(ros2 node list 2>/dev/null || true)
  if [[ $nodes == *'/bridge'* &&
        $nodes == *'/rviz'* &&
        $nodes == *'/kinematic_localization'* &&
        $nodes == *'/global_trajectory_publisher_node'* &&
        $nodes == *'/frenet_odom_node'* &&
        $nodes == *'/obstacle_detector'* &&
        $nodes == *'/local_planner_node'* &&
        $nodes == *'/state_machine_node'* &&
        $nodes == *'/control_map_node'* &&
        $nodes == *'/cruise_controller_node'* &&
        $nodes == *'/sim_imu_bridge_node'* &&
        $nodes == *'/drive_source_selector'* ]]; then
    ready=1
    break
  fi
  sleep 0.5
done
if (( ! ready )); then
  print -u2 -- "W3 process set not ready; evidence: $OUTPUT_DIR"
  print -r -- $nodes >$OUTPUT_DIR/nodes_failed.txt
  exit 20
fi

setsid ros2 run gqsc_runtime_replay sce018_replay_driver --ros-args \
  -p event_file:=$EVENT \
  -p output_path:=$RESULT \
  -p workload_id:=W3_SCE018_FULL_STACK_CONTENTION \
  -p condition:=SMOKE \
  -p repeat_id:=0 \
  -p attempt_id:=SMOKE \
  -p target_callbacks:=4 \
  -p warmup_callbacks:=0 \
  -p minimum_source_epochs:=2 \
  -p smoke_mode:=true \
  -p preflight_hold_sec:=3.0 \
  -p timeout_sec:=40.0 \
  >$OUTPUT_DIR/driver.log 2>&1 &
driver_pid=$!

sleep 2
{
  print -- '=== NODES ==='
  ros2 node list
  print -- '=== PRIVATE GLOBAL REFERENCE ==='
  ros2 topic info --verbose /gqsc_runtime/global_waypoints
  print -- '=== PRIVATE CONFIRMED OBSTACLES ==='
  ros2 topic info --verbose /gqsc_runtime/confirmed_static_obs
  print -- '=== PRIVATE FRENET ODOMETRY ==='
  ros2 topic info --verbose /gqsc_runtime/frenet_odom
  print -- '=== PRIVATE STATE ==='
  ros2 topic info --verbose /gqsc_runtime/state
  print -- '=== PRIVATE P3 CYCLE ==='
  ros2 topic info --verbose /gqsc_runtime/p3_cycle
  print -- '=== PRIVATE LIVE PROFILE ==='
  ros2 topic info --verbose /gqsc_runtime/live_profile
  print -- '=== NORMAL DETECTOR OUTPUT ==='
  ros2 topic info --verbose /confirmed_static_obs
  print -- '=== NORMAL SIM ODOMETRY ==='
  ros2 topic info --verbose /ego_racecar/odom
} >$GRAPH

set +e
wait $driver_pid
typeset driver_status=$?
set -e
driver_pid=''

if (( driver_status != 0 )); then
  print -u2 -- "W3 smoke failed; evidence: $OUTPUT_DIR"
  exit $driver_status
fi

print -- "W3_SMOKE_PASS"
print -- "evidence=$OUTPUT_DIR"
