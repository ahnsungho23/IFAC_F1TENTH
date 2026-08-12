#!/usr/bin/env bash
# Record obstacle-free lap traces on the CLEAN ifac_track at several controller speed caps,
# for tracking_error_lut_from_traces.py. On a clean map the published path IS the global race
# line, so the referee's |lat_err| is exactly the tracking error at every sampled (speed,
# curvature) -- including the low-speed rows the race line never visits, which otherwise get
# pessimistic monotone-filled values that block the gap-driven avoidance speed cap.
#
# usage: clean_lap_lut_traces.sh <output_dir> [domain_id]
# Runs one lap per speed cap in SPEED_CAPS (empty string = no cap / race profile).
WS=/home/sungho/Documents/GitHub/2026_IFAC
SIM_WS=/home/sungho/f1sim_C
OUT="${1:?usage: clean_lap_lut_traces.sh <output_dir> [domain_id]}"
DOMAIN="${2:-226}"
SPEED_CAPS=("1.6" "2.0" "2.5" "2.9" "" "")

CLEAN_BASE=$WS/src/monte_carlo_localization/maps/ifac_track
WPCSV=$WS/offline_trajectory_generator/output/ifac_track/global_waypoints.csv
REFPARAMS=$WS/src/lap_referee/config/lap_referee.yaml
SX=-17.483476031966312; SY=5.675529517562366; STHETA=-3.024225

mkdir -p "$OUT"
export ROS_DOMAIN_ID=$DOMAIN
export ROS_LOG_DIR="$OUT/ros_logs"
mkdir -p "$ROS_LOG_DIR"
export PYTHONUNBUFFERED=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/jazzy/setup.bash
source "$SIM_WS/install/local_setup.bash" 2>/dev/null || true
source "$WS/install/setup.bash"

sweep_orphans() {
  ps -ef | grep -E "[r]os2|[g]ym_bridge|_node --ros-args|[l]ap_referee|[p]article_filter|ros2cli" \
    | grep -v grep | awk '{print $2}' | sort -u | xargs -r kill -9 2>/dev/null
  sleep 2
}

wait_topic() {
  local topic="$1" budget="$2" waited=0
  while [ "$waited" -lt "$budget" ]; do
    if timeout 5 ros2 topic echo --no-daemon --once "$topic" >/dev/null 2>&1; then
      echo "  ready: $topic (${waited}s)"; return 0
    fi
    waited=$((waited + 5))
  done
  echo "  MISSING: $topic after ${budget}s"; return 1
}

run_one() {  # run_one <label> <max_speed or empty>
  local label="$1" cap="$2"
  local run_out="$OUT/$label"
  mkdir -p "$run_out"
  local pids=()
  launch() {
    local name="$1"; shift
    stdbuf -oL -eL "$@" > "$run_out/$name.log" 2>&1 &
    pids+=($!)
  }
  sweep_orphans
  launch simulator ros2 launch f1tenth_gym_ros obstacle_sim_launch.py \
    "map_path:=$CLEAN_BASE" map_img_ext:=.png \
    "sx:=$SX" "sy:=$SY" "stheta:=$STHETA" \
    simulator_seed:=12345 scan_noise_std:=0.01 \
    scan_publication_mode:=legacy_republish publish_scan_identity:=true \
    num_agent:=1 rviz:=false
  wait_topic /ego_racecar/odom 40
  launch global ros2 launch global_planning global_planning.launch.py map_name:=ifac_track
  wait_topic /global_waypoints 40
  if [ -n "$cap" ]; then
    launch controller ros2 launch f1tenth_control control_sim.launch.py "max_speed:=$cap"
  else
    launch controller ros2 launch f1tenth_control control_sim.launch.py
  fi
  wait_topic /drive 40
  launch referee ros2 launch lap_referee lap_referee.launch.py \
    "params_file:=$REFPARAMS" "waypoints_csv:=$WPCSV" \
    "output_dir:=$run_out" output_prefix:=lap
  echo "[$label] lap running (cap=${cap:-none})"
  for _ in $(seq 1 150); do
    [ -f "$run_out/lap_summary.json" ] && break
    sleep 1
  done
  sleep 1
  for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null; done
  sleep 6
  for p in "${pids[@]}"; do kill -9 "$p" 2>/dev/null; done
  sweep_orphans
  grep -E '"terminated"|"lap_completed"|"lap_time_s"|"max_speed_mps"|"n_samples"' \
    "$run_out/lap_summary.json" 2>/dev/null || echo "[$label] NO SUMMARY"
}

index=0
for cap in "${SPEED_CAPS[@]}"; do
  index=$((index + 1))
  label=$(printf "run_%02d_cap_%s" "$index" "${cap:-race}")
  run_one "$label" "$cap"
done
echo "traces:"
ls "$OUT"/run_*/lap_trace.csv 2>/dev/null
