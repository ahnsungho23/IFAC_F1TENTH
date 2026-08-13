#!/usr/bin/env bash
# Reproduce the normal-chain (gym_bridge + MCL) finals run that exposed the P3 blind-corner
# failure, using the same map / candidate / spawn pose as
# runs/cmaes_tuning/p3_hybrid_p0_cma_v1/.evaluation/normal_finals_best_evaluable_retry.
#
# usage: normal_finals_repro.sh <output_dir> [domain_id]     (domain_id must be <= 232)
#
# Reconstructed from that run's processes.txt/run_contract.txt and the launch commands in
# cmaes_tuning/simulation_runner.py.
#
# What to check in the output:
#   <out>/repro_summary.json     terminated / collided / progress_m
#   <out>/local.log              "DIAG perception ... dynamic=N translation_suppressed=N"
#                                -> the blind-corner obstacle must stay out of dynamic=
#   <out>/p3diag.log             per-callback P3 lifecycle; feed to p3_failure_window_report.py
WS=/home/sungho/Documents/GitHub/2026_IFAC
SIM_WS=/home/sungho/f1sim_C
OUT="${1:?usage: normal_finals_repro.sh <output_dir> [domain_id]}"
DOMAIN="${2:-241}"

# Default = the v1 frozen finals map. Its obstacle #1 (s=9.114) sits in the vehicle's
# full-lock curvature shadow and is impassable (safe-stop there is the correct outcome);
# for pass-through verification point BAKED_MAP at the v2 vehicle-feasible map instead:
#   BAKED_MAP=$WS/runs/cmaes_tuning/p3_hybrid_p0_cma_v2/scenarios/finals_competition_style/ifac_track
BAKED="${BAKED_MAP:-$WS/runs/cmaes_tuning/ifac_track_p3_m1_lifecycle_shadow_v1/pre_result/rule_checked_maps/finals_competition_style/ifac_track}"
CLEAN=$WS/src/monte_carlo_localization/maps/ifac_track.yaml
# The CMA best candidate carries its own copy of every planner parameter, including the LUT, so a
# candidate file frozen before the LUT was measured would silently re-run the old margins.
CAND="${CANDIDATE_YAML:-$WS/runs/cmaes_tuning/p3_hybrid_p0_cma_v1/.evaluation/best_candidate_measured_lut.yaml}"
WPCSV=$WS/offline_trajectory_generator/output/ifac_track/global_waypoints.csv
REFPARAMS=$WS/src/lap_referee/config/lap_referee.yaml
SX=-17.483476031966312; SY=5.675529517562366; STHETA=-3.024225

mkdir -p "$OUT"
export ROS_DOMAIN_ID=$DOMAIN
export ROS_LOG_DIR="$OUT/ros_logs"
mkdir -p "$ROS_LOG_DIR"

# Same transport pin the CMA lockstep framework uses. FastDDS shared-memory leaves zombie
# segments behind whenever a node is SIGKILLed; once enough pile up, participant creation and
# lifecycle service calls (map_server configure/activate) stall with no error and the chain
# comes up half-dead. Loopback UDPv4 removes that whole failure class.
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
source /opt/ros/jazzy/setup.bash
source "$SIM_WS/install/local_setup.bash" 2>/dev/null || true
source "$WS/install/setup.bash"

PIDS=()
launch() {  # launch <logname> <cmd...>
  local name="$1"; shift
  stdbuf -oL -eL "$@" > "$OUT/$name.log" 2>&1 &
  PIDS+=($!)
  echo "$name $!" >> "$OUT/processes.txt"
}

# A SIGKILLed node leaves its FastDDS shared-memory segments behind, and once a few thousand
# have piled up in /dev/shm every new node hangs inside participant creation with no error --
# the chain then comes up half-dead and the referee never sees a car. Sweep the orphans while
# nothing is running, both before and after the episode.
sweep_orphans() {
  ps -ef | grep -E "[r]os2|[g]ym_bridge|_node --ros-args|[l]ap_referee|[p]article_filter|ros2cli" \
    | grep -v grep | awk '{print $2}' | sort -u | xargs -r kill -9 2>/dev/null
  sleep 2
  if [ "$(ps -ef | grep -E '[r]os2|[g]ym_bridge|_node --ros-args' | grep -v grep | wc -l)" -eq 0 ]; then
    find /dev/shm -maxdepth 1 -name 'fastrtps_*' -delete 2>/dev/null
    find /dev/shm -maxdepth 1 -name 'sem.fastrtps_*' -delete 2>/dev/null
  fi
}

cleanup() {
  for p in "${PIDS[@]:-}"; do kill -INT "$p" 2>/dev/null; done
  sleep 8
  for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null; done
  sweep_orphans
}
trap cleanup EXIT

sweep_orphans

# Fixed sleeps are not enough to sequence this chain. particle_filter asks map_server for the map
# as soon as it starts, and if map_server has not reached ACTIVE yet it answers nothing: the filter
# then auto-initializes against an empty map ("Invalid map resolution: 0.000000") and aborts on an
# Eigen assertion. /pf/pose/odom never appears, the planner never gets an ego Frenet state, and the
# car drives the global race line straight into the obstacle while every node still looks healthy.
# So gate each stage on the topic it must produce.
wait_topic() {  # wait_topic <topic> <timeout_s>
  local topic="$1" budget="$2" waited=0
  while [ "$waited" -lt "$budget" ]; do
    if timeout 5 ros2 topic echo --no-daemon --once "$topic" >/dev/null 2>&1; then
      echo "  ready: $topic (${waited}s)"; return 0
    fi
    waited=$((waited + 5))
  done
  echo "  MISSING: $topic after ${budget}s"; return 1
}

echo "domain=$DOMAIN map=$BAKED candidate=$CAND" > "$OUT/run_contract.txt"

launch simulator ros2 launch f1tenth_gym_ros obstacle_sim_launch.py \
  "map_path:=$BAKED" map_img_ext:=.png \
  "sx:=$SX" "sy:=$SY" "stheta:=$STHETA" \
  simulator_seed:=12345 scan_noise_std:=0.01 \
  scan_publication_mode:=legacy_republish publish_scan_identity:=true \
  num_agent:=1 rviz:=false
wait_topic /ego_racecar/odom 40

launch global ros2 launch global_planning global_planning.launch.py map_name:=ifac_track
wait_topic /global_waypoints 40

launch_mcl() {
  launch mcl ros2 launch particle_filter_cpp mcl_launch.py \
    mod:=sim map_name:=ifac_track use_rviz:=false start_map_server:=true use_sim_time:=false
}
mcl_attempt=0
until wait_topic /pf/pose/odom 40; do
  mcl_attempt=$((mcl_attempt + 1))
  if [ "$mcl_attempt" -ge 3 ]; then
    echo "  MCL DID NOT COME UP - results will be invalid"
    break
  fi
  echo "  retrying MCL ($mcl_attempt) after map_server activation"
  pkill -9 -f particle_filter 2>/dev/null; sleep 3
  mv "$OUT/mcl.log" "$OUT/mcl_failed_attempt_$mcl_attempt.log" 2>/dev/null
  launch_mcl
done
wait_topic /car_state/frenet/odom 30

launch local ros2 launch local_planning local_planning.launch.py \
  "params_file:=$CAND" "reference_map:=$CLEAN" \
  simulator:=true use_sim_time:=false p3_mode:=TEST_ACTIVE
wait_topic /static_obs 40

launch state ros2 launch state_machine state_machine.launch.py
sleep 3

# Passive capture of the P3 lifecycle diagnostics stream the analysis depends on. --no-daemon
# keeps this off the ros2cli daemon, whose XMLRPC probe dies when a stale daemon is around.
launch p3diag ros2 topic echo --no-daemon --full-length --field data /local_planning/p3_shadow
sleep 1

# MCL publishes a pose as soon as it initializes but is not converged yet, and the first corner of
# this track is the tightest on the lap: starting on an unconverged pose puts the car into the wall
# before it ever reaches an obstacle, which looks exactly like a planner failure in the summary.
# Wait for MCL to actually agree with ground truth instead of sleeping a fixed amount, and do it
# BEFORE the referee starts -- its no_start_timeout_sec is 8 s.
localization_error() {
  local gt mcl
  gt=$(timeout 8 ros2 topic echo --no-daemon --once /ego_racecar/odom --field pose.pose.position 2>/dev/null)
  mcl=$(timeout 8 ros2 topic echo --no-daemon --once /pf/pose/odom --field pose.pose.position 2>/dev/null)
  python3 - "$gt" "$mcl" <<'PYEOF'
import re, sys, math
def xy(text):
    values = dict(re.findall(r"^([xy]):\s*(-?[\d.eE+-]+)", text, re.M))
    return float(values["x"]), float(values["y"])
try:
    ax, ay = xy(sys.argv[1]); bx, by = xy(sys.argv[2])
    print(f"{math.hypot(ax - bx, ay - by):.4f}")
except Exception:
    print("nan")
PYEOF
}

settle_waited=0
while [ "$settle_waited" -lt 45 ]; do
  error=$(localization_error)
  echo "  localization error ${error} m (${settle_waited}s)" | tee -a "$OUT/localization_settle.txt"
  awk -v e="$error" 'BEGIN{exit !(e < 0.05)}' 2>/dev/null && break
  sleep 5
  settle_waited=$((settle_waited + 5))
done

# The referee's default no_start window (8 s) races controller launch + node init + launch
# kick, which take 10-17 s under load; either ordering loses sometimes. Give the referee a
# 60 s window instead, start it first so the trace covers the car from s=0, then start the
# controller.
python3 - "$REFPARAMS" "$OUT/lap_referee_params.yaml" <<'PYEOF'
import sys, yaml
with open(sys.argv[1]) as handle:
    params = yaml.safe_load(handle)
for node in params.values():
    ros = node.get("ros__parameters") if isinstance(node, dict) else None
    if isinstance(ros, dict) and "no_start_timeout_sec" in ros:
        ros["no_start_timeout_sec"] = 60.0
with open(sys.argv[2], "w") as handle:
    yaml.safe_dump(params, handle, sort_keys=False)
PYEOF
launch referee ros2 launch lap_referee lap_referee.launch.py \
  "params_file:=$OUT/lap_referee_params.yaml" "waypoints_csv:=$WPCSV" \
  "output_dir:=$OUT" output_prefix:=repro
sleep 2

launch controller ros2 launch f1tenth_control control_sim.launch.py
echo "chain up; running episode"
( timeout 20 ros2 topic list > "$OUT/topics.txt" 2>&1
  timeout 10 ros2 topic hz /ego_racecar/odom > "$OUT/hz_odom.txt" 2>&1
  timeout 10 ros2 topic hz /drive > "$OUT/hz_drive.txt" 2>&1
  timeout 10 ros2 topic hz /static_obs > "$OUT/hz_static_obs.txt" 2>&1 ) &

# The referee terminates the episode itself; give it a bounded wall-clock budget.
for _ in $(seq 1 150); do
  [ -f "$OUT/repro_summary.json" ] && break
  sleep 1
done
sleep 2
echo "done"
