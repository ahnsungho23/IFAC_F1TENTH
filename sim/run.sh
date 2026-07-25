#!/usr/bin/env zsh
# =================================================================================================
# sim/run.sh <role> — source the workspaces this role needs and launch ONE sim component, then drop
# to an interactive shell so the Terminator pane stays usable after the launch is stopped (Ctrl-C).
#
# Roles (same order as the root CLAUDE.md "Simulation Run Order"):
#   f1sim | mcl | global | local | state | wpnt | control     (base 7-node loop)
#   opp | oppdet                                              (optional opponent scenario, 8-9)
#   stop | scratch
#
# Trailing `name:=value` args are appended to that role's ros2 launch command, e.g.:
#     ~/2026_IFAC/sim/run.sh mcl use_rviz:=false
#     ~/2026_IFAC/sim/run.sh oppdet
#
# Used by the Terminator layouts (sim/f1sim.terminator, sim/f1sim_opp.terminator) via
# sim/open_sim.sh; also runnable by hand from any terminal.
# =================================================================================================
emulate -L zsh
setopt no_nomatch
# ROS args use `name:=value`; zsh's `=`-expansion would rewrite e.g. `simulator:=true` into
# `simulator:/usr/bin/true` (it expands `=cmd` to the command's path). Disable it so `:=` survives.
setopt no_equals

role="${1:-scratch}"
(( $# )) && shift                       # remaining args pass through to the launch command
# Repo root derived from this script's own location (sim/run.sh -> repo root), so a clone into any
# directory name works — do NOT hard-code $HOME/2026_IFAC. $IFAC_WS overrides it if ever needed.
IFAC="${IFAC_WS:-${0:A:h:h}}"

# Track/map used by MCL. Keep in sync with the gym bridge's map (sim_ws config/sim.yaml map_path).
# Override per run:  SIM_MAP_NAME=fuck_f1 ./sim/open_sim.sh
MAP_NAME="${SIM_MAP_NAME:-ifac_track}"

# ROS 2 Jazzy underlay first, then the overlay this role needs.
source /opt/ros/jazzy/setup.zsh 2>/dev/null

# f1tenth_gym_ros lives in a separate workspace whose path differs per machine.
# $F1SIM_WS overrides the candidates.
source_sim_ws() {
  local ws
  for ws in "${F1SIM_WS:-}" "$HOME/sim_ws" "$HOME/f1tenth_gym"; do
    [[ -n "$ws" && -f "$ws/install/setup.zsh" ]] || continue
    source "$ws/install/setup.zsh"
    print -P "%F{242}(f1tenth_gym_ros workspace: $ws)%f"
    return 0
  done
  print -P "%F{red}[run.sh] no f1tenth_gym_ros workspace found — tried \$F1SIM_WS, ~/sim_ws, ~/f1tenth_gym%f"
  return 1
}

# --- stale-process cleanup ------------------------------------------------------------------------
# Leftovers are the most common source of "the stack behaves randomly": a second global_planning
# overwriting the latched /global_waypoints, an old controller still publishing /drive, or a
# previous gym bridge holding the simulator. Each role kills ONLY its own previous instance, so
# panes started side by side never kill each other. The `stop` role clears the whole stack at once.
# `pgrep -f` matches the FULL command line, so it can also hit this script's own ancestors (a pane
# whose command string mentions the pattern). Collect the ancestor chain once and never touch it.
typeset -a SELF_CHAIN
_build_self_chain() {
  local p=$$ ppid
  while [[ -n "$p" && "$p" != "0" && "$p" != "1" ]]; do
    SELF_CHAIN+=("$p")
    ppid=$(awk '{print $4}' /proc/$p/stat 2>/dev/null) || break
    [[ -z "$ppid" || "$ppid" == "$p" ]] && break
    p="$ppid"
  done
}
_build_self_chain

kill_pattern() {                          # <pattern> [pattern ...]
  local pat p killed=0
  typeset -a pids victims
  victims=()
  for pat in "$@"; do
    pids=(${(f)"$(pgrep -f -- "$pat" 2>/dev/null)"})
    for p in $pids; do
      [[ -z "$p" ]] && continue
      (( ${SELF_CHAIN[(I)$p]} )) && continue      # never kill ourselves or an ancestor
      (( ${victims[(I)$p]} )) && continue         # already queued
      victims+=("$p")
    done
  done
  for p in $victims; do
    kill "$p" 2>/dev/null && killed=$((killed + 1))
  done
  if (( killed )); then
    print -P "%F{242}[$role] cleaned up ${killed} leftover process(es)%f"
    sleep 1
    # anything that ignored SIGTERM gets SIGKILL, otherwise the new node hits "name already taken"
    for p in $victims; do kill -9 "$p" 2>/dev/null; done
  fi
  return 0
}

typeset -a PAT_SIM PAT_MCL PAT_GLOBAL PAT_LOCAL PAT_STATE PAT_WPNT PAT_CONTROL PAT_OPP PAT_OPPDET
PAT_SIM=('ros2 launch f1tenth_gym_ros' 'gym_bridge')
PAT_MCL=('ros2 launch particle_filter_cpp' 'particle_filter_node'
         'particle_filter_map_server' 'lifecycle_manager_particle_filter')
PAT_GLOBAL=('ros2 launch global_planning' 'global_planning_node'
            'global_trajectory_publisher_node' 'frenet_odom_node')
PAT_LOCAL=('ros2 launch local_planning' 'local_planner_node')
PAT_STATE=('ros2 launch state_machine' 'state_machine_node')
PAT_WPNT=('ros2 run wpnt_publisher' 'wpnt_publisher/wpnt_publisher')
PAT_CONTROL=('ros2 launch f1tenth_control' 'control_map_node' 'control_mppi_node'
             'sim_imu_bridge_node' 'joy_teleop_monitor')
PAT_OPP=('ros2 launch new_map_con opponent_simulator' 'opponent_simulator'
         'opponent_drive_controller')
PAT_OPPDET=('ros2 launch opponent_detector' 'opponent_detector_node')

typeset -a cmd
delay=0

case "$role" in
  f1sim)                                   # Terminal 1 — simulator (gym bridge + its RViz)
    source_sim_ws
    kill_pattern "${PAT_SIM[@]}"
    cmd=(ros2 launch f1tenth_gym_ros gym_bridge_launch.py)
    ;;
  mcl|localization)                        # Terminal 2 — Monte Carlo Localization -> /pf/pose/odom
    source "$IFAC/install/setup.zsh" 2>/dev/null
    kill_pattern "${PAT_MCL[@]}"
    # EVERY downstream node depends on /pf/pose/odom — without MCL the whole stack stalls.
    # MCL_RVIZ=0 turns its RViz off when the f1sim RViz window is enough.
    cmd=(ros2 launch particle_filter_cpp mcl_launch.py
         mod:=sim map_name:="$MAP_NAME" use_rviz:="${MCL_RVIZ:-true}")
    delay=3
    ;;
  global|frenet)                           # Terminal 3 — /global_waypoints + /car_state/frenet/odom
    source "$IFAC/install/setup.zsh" 2>/dev/null
    kill_pattern "${PAT_GLOBAL[@]}"
    # global_planning config may use RELATIVE paths, so run from the workspace root
    # (the `cd "$IFAC"` below guarantees that).
    cmd=(ros2 launch global_planning global_planning.launch.py)
    delay=6
    ;;
  local|avoid)                             # Terminal 4 — local planner (obstacle avoidance)
    source "$IFAC/install/setup.zsh" 2>/dev/null
    kill_pattern "${PAT_LOCAL[@]}"
    cmd=(ros2 launch local_planning local_planning.launch.py)
    delay=8
    ;;
  state)                                   # Terminal 5 — state machine -> /state
    source "$IFAC/install/setup.zsh" 2>/dev/null
    kill_pattern "${PAT_STATE[@]}"
    cmd=(ros2 launch state_machine state_machine.launch.py)
    delay=9
    ;;
  wpnt)                                    # Terminal 6 — avoid wpnts relay -> /local_waypoints
    source "$IFAC/install/setup.zsh" 2>/dev/null
    kill_pattern "${PAT_WPNT[@]}"
    # wpnt_publisher ships no launch file, so it is a plain `ros2 run`
    # (identical to the root CLAUDE.md run order). Extra `name:=value` args do NOT apply.
    cmd=(ros2 run wpnt_publisher wpnt_publisher)
    delay=10
    ;;
  control|ego)                             # Terminal 7 — L1 + Steering LUT control -> /drive
    source "$IFAC/install/setup.zsh" 2>/dev/null
    kill_pattern "${PAT_CONTROL[@]}"
    # force_autonomous:=true drives immediately without a joystick.
    cmd=(ros2 launch f1tenth_control control_sim.launch.py force_autonomous:=true)
    delay=11
    ;;
  opp|opponent)                            # Terminal 8 — opponent follows the global line -> /opp_drive
    source "$IFAC/install/setup.zsh" 2>/dev/null
    kill_pattern "${PAT_OPP[@]}"
    # Needs the gym bridge at num_agent: 2 (sim_ws config/sim.yaml) AND terminal 7 kept running —
    # the 2-agent bridge only steps physics when BOTH cars publish drive.
    cmd=(ros2 launch new_map_con opponent_simulator.launch.py)
    delay=13
    ;;
  oppdet|detector)                         # Terminal 9 — opponent detector <- ego /scan
    source "$IFAC/install/setup.zsh" 2>/dev/null
    kill_pattern "${PAT_OPPDET[@]}"
    cmd=(ros2 launch opponent_detector opponent_detector.launch.py simulator:=true)
    delay=14
    ;;
  stop|clean|kill)                         # kill every node this stack owns, then exit
    print -P "%F{cyan}[stop] clearing the stack…%f"
    kill_pattern "${PAT_OPPDET[@]}" "${PAT_OPP[@]}" "${PAT_CONTROL[@]}" "${PAT_WPNT[@]}" \
                 "${PAT_STATE[@]}" "${PAT_LOCAL[@]}" "${PAT_GLOBAL[@]}" "${PAT_MCL[@]}"
    [[ "${1:-}" == "--all" || "${KEEP_SIM:-0}" == "0" ]] && kill_pattern "${PAT_SIM[@]}"
    print -P "%F{green}[stop] done%f"
    exit 0
    ;;
  scratch|*)                               # a pre-sourced debug shell
    source "$IFAC/install/setup.zsh" 2>/dev/null
    cd "$IFAC"
    print -P "%F{cyan}[scratch] debug shell — e.g.  ros2 topic echo /state%f"
    print -P "%F{cyan}          planner out: /avoid_waypoints · /overtake_waypoints · /local_waypoints%f"
    print -P "%F{cyan}          perception : /perception/obstacles · /proj_opponent_trajectory%f"
    exec zsh -i
    ;;
esac

(( $# )) && cmd+=("$@")                  # extra `name:=value` args go to this role's launch file

cd "$IFAC"
print -P "%F{green}[$role]%f ${cmd[*]}"

if (( delay > 0 )); then
  print -P "%F{242}(waiting ${delay}s for upstream nodes… Ctrl-C skips the wait)%f"
  trap 'print -P "%F{242}(wait skipped)%f"' INT
  sleep "$delay" 2>/dev/null || true
  trap - INT
fi

"${cmd[@]}"

print -P "%F{yellow}[$role] launch exited — dropping to an interactive shell%f"
exec zsh -i
