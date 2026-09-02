#!/usr/bin/env zsh
set -e

readonly REPO=/home/sungho/Documents/GitHub/2026_IFAC
readonly ROOT=$REPO/planning_study/gqsc_s1_live_runtime_qualification_v1
readonly TOOLS=$ROOT/tools
readonly EVENT=$REPO/planning_study/p3_geometry_conditioned_method_v1/success_control_inputs/SCE018.event
readonly OUTPUT_DIR=$(mktemp -d /tmp/gqsc_s1_w2_smoke.XXXXXX)
readonly RESULT=$OUTPUT_DIR/joined_smoke.jsonl
readonly GRAPH=$OUTPUT_DIR/ros_graph.txt
export ROS_DOMAIN_ID=82

typeset planner_pid=''
typeset driver_pid=''

cleanup() {
  if [[ -n $driver_pid ]] && kill -0 $driver_pid 2>/dev/null; then
    kill -INT $driver_pid 2>/dev/null || true
    for _ in {1..10}; do
      kill -0 $driver_pid 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM $driver_pid 2>/dev/null || true
    wait $driver_pid 2>/dev/null || true
  fi
  if [[ -n $planner_pid ]] && kill -0 $planner_pid 2>/dev/null; then
    kill -INT $planner_pid 2>/dev/null || true
    for _ in {1..10}; do
      kill -0 $planner_pid 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM $planner_pid 2>/dev/null || true
    wait $planner_pid 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.zsh
source /home/sungho/sim_ws/install/setup.zsh
source $REPO/install/setup.zsh
source $TOOLS/_install/setup.zsh

ros2 run local_planning local_planner_node --ros-args \
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

ros2 run gqsc_runtime_replay sce018_replay_driver --ros-args \
  -p event_file:=$EVENT \
  -p output_path:=$RESULT \
  -p workload_id:=W2_SCE018_ROS_NODE_TEST_ACTIVE \
  -p condition:=SMOKE \
  -p repeat_id:=0 \
  -p attempt_id:=SMOKE \
  -p target_callbacks:=4 \
  -p warmup_callbacks:=0 \
  -p minimum_source_epochs:=2 \
  -p smoke_mode:=true \
  -p preflight_hold_sec:=2.0 \
  -p timeout_sec:=30.0 \
  >$OUTPUT_DIR/driver.log 2>&1 &
driver_pid=$!

sleep 1
{
  ros2 node list
  ros2 topic info --verbose /gqsc_runtime/global_waypoints
  ros2 topic info --verbose /gqsc_runtime/confirmed_static_obs
  ros2 topic info --verbose /gqsc_runtime/frenet_odom
  ros2 topic info --verbose /gqsc_runtime/p3_cycle
  ros2 topic info --verbose /gqsc_runtime/live_profile
} >$GRAPH

set +e
wait $driver_pid
typeset driver_status=$?
set -e
driver_pid=''

if (( driver_status != 0 )); then
  print -u2 -- "W2 smoke failed; evidence: $OUTPUT_DIR"
  exit $driver_status
fi

print -- "W2_SMOKE_PASS"
print -- "evidence=$OUTPUT_DIR"
