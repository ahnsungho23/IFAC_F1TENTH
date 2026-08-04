#!/usr/bin/env zsh
# =================================================================================================
# real/run_real.sh <role> — launch ONE real-car component via SSH on the Jetson (RViz is local),
# then drop to a shell so the Terminator pane stays usable after Ctrl-C.
#
# Roles (same order as src/f1tenth_control/LAUNCH_FULLSTACK.md §3):
#   bringup | mcl | global | state | control     (on the Jetson, via ssh -t)
#   local                                     (T4 local_planning + obstacle_detector — local.sh 전용)
#   rviz                                      (Jetson, ssh -X: RViz + rosbag PAUSED 시작)
#   rvizlocal                                 (local, 본체 PC)
#   stop [--all] | scratch
#
# Trailing `name:=value` args are appended to the role's ros2 launch command, e.g.:
#     ~/2026_IFAC/real/run_real.sh control max_speed:=2.5 min_speed:=0.5     # 셰이크다운
#
# Env overrides:
#   F1_HOST=miru@10.1.1.3        # Jetson SSH target
#   F1_MAP_NAME=map              # map name (MCL map_name:= / global F1_MAP)
#
# ⚠️ non-interactive ssh never reads the Jetson's ~/.zshrc, so ROS_DOMAIN_ID/RMW are exported
# explicitly here (values from LAUNCH_FULLSTACK.md). F1_MAP is exported per-pane because the
# Jetson .zshrc may carry a wrong value (it was `ifac_track` on 2026-07-31).
# =================================================================================================
emulate -L zsh
setopt no_nomatch no_equals

role="${1:-scratch}"
(( $# )) && shift                       # remaining args pass through to the launch command

REALDIR="${0:A:h}"
IFAC="${REALDIR:h}"                     # repo root (real/run_real.sh -> repo root)
JETSON="${F1_HOST:-miru@10.1.1.3}"
MAP_NAME="${F1_MAP_NAME:-map}"

# Remote prelude shared by every Jetson pane.
RPRE="export ROS_DOMAIN_ID=67 RMW_IMPLEMENTATION=rmw_fastrtps_cpp;"
RPRE+=" source /opt/ros/jazzy/setup.zsh 2>/dev/null;"

remote() {                              # <delay> <remote command string>   (SSH_OPTS로 ssh 플래그 조정)
  local d="$1" rcmd="$2"
  (( $# >= 2 )) && shift 2
  (( $# )) && rcmd+=" $*"               # extra `name:=value` args go to the launch command
  print -P "%F{green}[$role]%f ssh $JETSON -- $rcmd"
  if (( d > 0 )); then
    print -P "%F{242}(waiting ${d}s for upstream nodes… Ctrl-C skips the wait)%f"
    trap 'print -P "%F{242}(wait skipped)%f"' INT
    sleep "$d" 2>/dev/null || true
    trap - INT
  fi
  ssh ${=SSH_OPTS:--t} "$JETSON" "$RPRE $rcmd; echo; echo '[$role] launch exited — dropping to a REMOTE shell'; exec zsh -i"
  local rc=$?
  # ssh 자체가 실패(차 오프라인/인증 실패)핏도 창이 닫히지 않게 로컬 셸로 유지
  print -P "%F{yellow}[$role] ssh session ended (rc=$rc) — dropping to a LOCAL shell (재시도: $0 $role)%f"
  exec zsh -i
}

case "$role" in
  bringup)                               # T1 — hardware bringup (VESC/lidar/joy/mux)
    remote 0 "cd ~/f1tenth_ws && source install/setup.zsh && ros2 launch f1tenth_stack bringup_launch.py"
    ;;
  mcl|localization)                      # T2 — MCL -> /pf/pose/odom  (MCL은 F1_MAP을 안 읽는다 — map_name 명시)
    remote 6 "cd ~/2026_IFAC && source install/setup.zsh && ros2 launch particle_filter_cpp mcl_launch.py mod:=real map_name:=$MAP_NAME use_rviz:=false"
    ;;
  global|frenet)                         # T3 — /global_waypoints + /car_state/frenet/odom
    remote 12 "cd ~/2026_IFAC && source install/setup.zsh && export F1_MAP=$MAP_NAME && ros2 launch global_planning global_planning.launch.py"
    ;;
  local|avoid)                           # T4 — local planner + obstacle_detector (/avoid_waypoints)
    remote 13 "cd ~/2026_IFAC && source install/setup.zsh && export F1_MAP=$MAP_NAME && ros2 launch local_planning local_planning.launch.py simulator:=false"
    ;;
  state)                                 # T5 — /state + /local_waypoints (local_planning 없음 → GLOBAL 유지, 글로벌 릴레이)
    remote 15 "cd ~/2026_IFAC && source install/setup.zsh && ros2 launch state_machine state_machine.launch.py"
    ;;
  control|ego)                           # T6 — L1 + Steering LUT -> /drive (마지막에 띄울 것)
    remote 18 "source ~/f1tenth_ws/install/setup.zsh && cd ~/2026_IFAC && source install/setup.zsh && ros2 launch f1tenth_control control_real.launch.py"
    ;;
  rviz)                                  # 젯슨 — RViz(X-forward) + rosbag 녹화(일시정지 상태로 시작)
    # 녹화는 --start-paused로 백그라운드 기동. 재개/정지는 어느 창에서나:
    #   ros2 service call /rosbag2_recorder/resume rosbag2_interfaces/srv/Resume
    #   ros2 service call /rosbag2_recorder/pause  rosbag2_interfaces/srv/Pause
    # RViz를 닫으면(Ctrl-C) 녹화도 SIGINT로 함께 종료. X-forward OpenGL이라 소프트웨어 렌더링 강제.
    SSH_OPTS="-X -t" remote 10 'source ~/f1tenth_ws/install/setup.zsh && cd ~/2026_IFAC && source install/setup.zsh && export LIBGL_ALWAYS_SOFTWARE=1 && BAG=~/rosbags/$(date +%m%d)/run_$(date +%m%d_%H%M%S) && mkdir -p ${BAG:h} && { ros2 bag record -a -s sqlite3 --start-paused -o $BAG & BAGPID=$!; echo "[rec] $BAG — PAUSED로 시작 (재개: ros2 service call /rosbag2_recorder/resume rosbag2_interfaces/srv/Resume)"; rviz2 -d $(ros2 pkg prefix particle_filter_cpp)/share/particle_filter_cpp/rviz/particle_filter.rviz; kill -INT $BAGPID 2>/dev/null; wait $BAGPID 2>/dev/null; }'
    ;;
  rvizlocal)                             # 본체 PC — RViz만 로컬에서 (X-forward가 느리면 이쪽)
    export ROS_DOMAIN_ID=67 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    source /opt/ros/jazzy/setup.zsh 2>/dev/null
    source "$IFAC/install/setup.zsh" 2>/dev/null
    ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
    print -P "%F{green}[rvizlocal]%f rviz2 (local, Fixed Frame map — 차량 정지 후 2D Pose Estimate로 초기 위치)"
    rviz2 -d "$(ros2 pkg prefix particle_filter_cpp)/share/particle_filter_cpp/rviz/particle_filter.rviz"
    print -P "%F{yellow}[rvizlocal] exited — dropping to an interactive shell%f"
    exec zsh -i
    ;;
  stop|clean|kill)                       # 이 스택의 노드만 원격 종료 (--all: bringup까지)
    print -P "%F{cyan}[stop] clearing the stack on $JETSON…%f"
    ssh "$JETSON" "pkill -f 'ros2 launch (particle_filter_cpp|global_planning|state_machine|f1tenth_control|local_planning|obstacle_detector)'; pkill -f 'ros2 bag record|rosbag2_recorder'; pkill -f 'particle_filter_node|particle_filter_map_server|lifecycle_manager_particle_filter|global_trajectory_publisher_node|frenet_odom_node|state_machine_node|control_map_node|drive_source_selector|local_planner_node|obstacle_detector_node'" 2>/dev/null
    if [[ "${1:-}" == "--all" ]]; then
      ssh "$JETSON" "pkill -f 'ros2 launch f1tenth_stack'" 2>/dev/null
    fi
    print -P "%F{green}[stop] done%f"
    exit 0
    ;;
  scratch|*)                             # Jetson에 그냥 붙는 디버그 셸
    print -P "%F{cyan}[scratch] ssh $JETSON%f"
    exec ssh -t "$JETSON"
    ;;
esac
