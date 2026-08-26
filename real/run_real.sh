#!/usr/bin/env zsh
# =================================================================================================
# real/run_real.sh <role> — launch ONE real-car component via SSH on the Jetson (RViz is local),
# then drop to a shell so the Terminator pane stays usable after Ctrl-C.
#
# Roles (실차 실행 순서 = real/local.terminator 의 pane 번호):
#   bringup   1  시각 동기화 → sudo jetson_clocks && f110
#   ping      2  라이다 이더넷 링크: 기동 1회 점검(필요시 허브 재부착) 후 감시만
#   mcl       3  kinematic_localization  map_name:=$F1_MAP_NAME
#   global    4  global_planning         map_name:=$F1_MAP_NAME
#   local     5  local_planning (+obstacle_detector)
#   state     6  state_machine
#   control   7  f1tenth_control                  (마지막에 띄울 것)
#   rviz         젯슨 ssh -X: RViz + rosbag(PAUSED 시작)   — 기본 레이아웃에는 없다
#   rvizlocal    본체 PC 에서 RViz 만
#   foxglove     젯슨에서 foxglove_bridge (ws://<젯슨>:8765) — WiFi 역압이 없는 시각화
#   time         젯슨 시계를 **이 기기**에 맞춘다 (bringup 이 자동으로 먼저 부른다)
#   stop [--all] | scratch
#
# Trailing `name:=value` args are appended to the role's ros2 launch command, e.g.:
#     ~/2026_IFAC/real/run_real.sh control max_speed:=2.5 min_speed:=0.5     # 셰이크다운
#
# Env overrides:
#   F1_HOST=miru@10.1.1.1        # Jetson SSH target
#   F1_MAP_NAME=map              # map name (mcl/global 의 map_name:= 인자)
#   F1_SCAN_HZ=5                 # foxglove role 이 만드는 /slow_scan 의 Hz
#
# 🔑 원격 명령은 **대화형 zsh**(`zsh -ic`) 안에서 돈다. `f110`·`sc` 가 젯슨 ~/.zshrc 의
#    alias 라서 비대화형 ssh 로는 안 풀리기 때문이다.
#    ⚠️ alias 는 **parse 시점**에 확장된다 — "source ~/.zshrc; sc && ros2 ..." 처럼 한 줄로
#    보내면 source 가 실행되기 전에 이미 파싱이 끝나 `sc: command not found` 가 난다.
#    `zsh -i` 가 rc 를 먼저 읽고 `-c` 문자열을 그 뒤에 파싱하는 순서라야 맞는다.
#    ROS_DOMAIN_ID/RMW 는 rc 를 읽은 **뒤에** export 해서 젯슨 ~/.zshrc 값을 이긴다.
# =================================================================================================
emulate -L zsh
setopt no_nomatch no_equals

role="${1:-scratch}"
(( $# )) && shift                       # remaining args pass through to the launch command

REALDIR="${0:A:h}"
IFAC="${REALDIR:h}"                     # repo root (real/run_real.sh -> repo root)
JETSON="${F1_HOST:-miru@10.1.1.1}"
MAP_NAME="${F1_MAP_NAME:-map}"
DOMAIN="${ROS_DOMAIN_ID:-70}"
SCAN_HZ="${F1_SCAN_HZ:-5}"          # foxglove role: /scan -> /slow_scan 솎음 주기(Hz)

# ~/.zshrc 를 읽은 뒤 덮어쓸 환경. 아래 remote() 가 zsh -ic 문자열의 맨 앞에 붙인다.
RENV="export ROS_DOMAIN_ID=$DOMAIN RMW_IMPLEMENTATION=rmw_fastrtps_cpp;"

# ── 시각 동기화 ────────────────────────────────────────────────────────────────
# 🔴 젯슨 RTC 에는 백업 배터리가 없다 — 전원을 끊으면 시계가 1970 으로 돌아간다.
#    그리고 차량망(핫스팟)에는 상위 NTP 가 없어서 systemd-timesyncd 가 "동기화됨"이라고
#    보고해도 실제로는 아무 데서도 시각을 못 받아온다.
# 🔑 bag · jetson_load.sh · 분석 스크립트가 전부 "젯슨과 이 기기가 같은 시계"를 전제한다.
#    어긋나면 두 기록을 붙였을 때 정렬이 **조용히** 틀린다.
#
# 🔴 측정법이 중요하다 — 단발 왕복으로 재면 거짓말한다.
#    오프셋 추정 `jt − (t0+t1)/2` 는 **왕복이 대칭**일 때만 맞는데, 시끄러운 WiFi 에서는
#    전혀 대칭이 아니다. 2026-08-25 실측(2단 ssh): 같은 순간을 5번 쟀는데
#      RTT 0.18 s → skew +0.13 s / RTT 0.96 s → +0.53 s / RTT 2.69 s → +1.37 s
#    처럼 **추정치가 RTT 에 그대로 비례**했다. 즉 대부분이 시계 오차가 아니라 링크 비대칭이다.
#    그래서 NTP 와 같이 여러 번 재고 **RTT 가 가장 작은 표본만** 쓴다(minimum filter).
#    오차 한계는 그 RTT 의 절반이므로 같이 찍어 준다.
TIME_SKEW_MAX="${F1_TIME_SKEW_MAX:-1.0}"   # 이 초를 넘을 때만 건드린다
TIME_SAMPLES="${F1_TIME_SAMPLES:-7}"       # 최소 RTT 를 고르기 위한 표본 수

# 짧은 탐침에만 쓰는 ssh 다중화. ⚠️ remote() 의 런치 세션에는 **일부러 안 쓴다** —
# 마스터 하나가 끊기면 패널 7개가 한꺼번에 죽기 때문.
_time_ssh=(-o ControlMaster=auto -o "ControlPath=${TMPDIR:-/tmp}/f1time-%r@%h:%p"
           -o ControlPersist=15 -o BatchMode=yes -o ConnectTimeout=8)

# _probe_skew — 최소 RTT 표본의 (skew, rtt) 를 전역 _SKEW/_RTT 에 넣는다. 실패 시 1 반환.
_probe_skew() {
  local i t0 t1 jt rtt skew got=0
  _SKEW=0; _RTT=999
  for (( i = 1; i <= TIME_SAMPLES; i++ )); do
    t0=$(date +%s.%N)
    jt=$(ssh "${_time_ssh[@]}" "$JETSON" 'date +%s.%N' 2>/dev/null) || continue
    t1=$(date +%s.%N)
    [[ -z "$jt" ]] && continue
    rtt=$(( t1 - t0 )); skew=$(( jt - (t0 + t1) / 2 ))
    got=1
    (( rtt < _RTT )) && { _RTT=$rtt; _SKEW=$skew; }
  done
  (( got )) || return 1
  return 0
}

sync_time() {
  local force="${1:-}" target
  if ! _probe_skew; then
    print -P "%F{yellow}[time] 젯슨에 붙지 못했다 — 시각 동기화 건너뜀%f"; return 0
  fi
  printf -v _SKEW '%.3f' "$_SKEW"; printf -v _RTT '%.3f' "$_RTT"
  print -P "%F{cyan}[time]%f 젯슨 − 이 기기 = ${_SKEW}s  (최소 RTT ${_RTT}s → 불확실도 ±$(printf '%.3f' $(( _RTT / 2 )))s)"
  if [[ -z "$force" ]] && (( ${_SKEW#-} < TIME_SKEW_MAX )); then
    print -P "%F{green}[time] ${TIME_SKEW_MAX}s 이내 — 그대로 둔다%f"; return 0
  fi
  # 편도 지연(최소 RTT 의 절반)만큼 앞당겨 보낸다 — 젯슨이 받는 시점에 맞도록.
  target=$(( $(date +%s.%N) + _RTT / 2 ))
  printf -v target '%.6f' "$target"
  print -P "%F{cyan}[time]%f 젯슨 시계를 이 기기에 맞춘다"
  ssh ${=SSH_OPTS:--t} "$JETSON" "sudo date -s @$target >/dev/null && echo '[time] set: '\$(date '+%F %T.%3N %Z')" \
    || { print -P "%F{yellow}[time] 설정 실패(sudo 거부?) — 계속 진행한다%f"; return 0; }
  # 실제로 먹었는지 확인. systemd-timesyncd 가 되돌리면 여기서 드러난다.
  _probe_skew || return 0
  printf -v _SKEW '%.3f' "$_SKEW"
  if (( ${_SKEW#-} < TIME_SKEW_MAX )); then
    print -P "%F{green}[time] 동기화 완료 — 잔차 ${_SKEW}s%f"
  else
    print -P "%F{yellow}[time] 아직 ${_SKEW}s 어긋나 있다.%f"
    print -P "%F{yellow}       systemd-timesyncd 가 되돌렸을 수 있다. 젯슨에서 한 번만:%f"
    print -P "%F{yellow}         sudo timedatectl set-ntp false%f"
  fi
}

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
  local inner="$RENV $rcmd"
  ssh ${=SSH_OPTS:--t} "$JETSON" "zsh -ic ${(q)inner}; echo; echo '[$role] exited — dropping to a REMOTE shell'; exec zsh -i"
  local rc=$?
  # ssh 자체가 실패(차 오프라인/인증 실패)해도 창이 닫히지 않게 로컬 셸로 유지
  print -P "%F{yellow}[$role] ssh session ended (rc=$rc) — dropping to a LOCAL shell (재시도: $0 $role)%f"
  exec zsh -i
}

case "$role" in
  bringup)                               # 1 — hardware bringup (VESC/lidar/joy/mux)
    # 시각 동기화를 여기서 먼저 한다 — 어차피 sudo 를 쓰는 창이라 비밀번호를 한 번만 묻는다.
    sync_time
    # ⚠️ `sudo` 는 비밀번호를 물을 수 있다. ssh -t 로 tty 를 주므로 프롬프트가 정상 동작한다.
    remote 0 "sudo jetson_clocks && f110"
    ;;

  time|clock)                            # 젯슨 시계를 이 기기에 맞춘다 (임계 무시하고 강제)
    sync_time --force
    print -P "%F{green}[time] done%f"
    exit 0
    ;;
  ping|link)                             # 2 — 라이다 이더넷 링크 감시 + USB-C 허브 자동 복구
    # 2026-08-25 백에서 라이다가 충돌 1.3s 뒤 Not Connected 로 죽고 끝까지 안 돌아왔다.
    # 이 창이 살아 있으면 "링크가 끊긴 것"과 "센서가 죽은 것"을 실시간으로 가를 수 있다.
    # lidar_link_watch.sh 는 **기동 시 1회만** 점검한다. "Destination Host Unreachable"
    # 이면 usbc_power.sh cycle --force 로 허브를 재부착하고 다시 확인한 뒤, 그 다음부터는
    # ping 만 흘린다(자동 복구 없음).
    # 🔴 주행 중에 자동 복구를 안 하는 이유: 같은 허브에 조이스틱이 물려 있어 사이클을
    #    돌리면 /joy 가 죽고 = 조이스틱 E-stop 을 못 쓴다. 기동 시점(차 정지·자율 미체결)
    #    에만 복구하는 게 안전하다. 주행 중 복구는 사람이 판단해 수동으로.
    #    끄려면 LIDAR_AUTOCYCLE=0, 자세한 건 real/lidar_link_watch.sh 헤더.
    remote 0 "W=~/2026_IFAC/real/lidar_link_watch.sh; [ -x \$W ] && \$W 192.168.0.10 || ping 192.168.0.10"
    ;;
  mcl|localization)                      # 3 — KICP -> /pf/pose/odom
    # 동결 맵은 maps/$MAP_NAME.kissmap (global/local 이 읽는 $MAP_NAME.yaml 점유격자와 짝이어야 한다).
    remote 6 "cd ~/2026_IFAC && sc && ros2 launch kinematic_localization kinematic_localization.launch.py map_name:=$MAP_NAME"
    ;;
  global|frenet)                         # 4 — /global_waypoints + /car_state/frenet/odom
    remote 12 "cd ~/2026_IFAC && sc && ros2 launch global_planning global_planning.launch.py map_name:=$MAP_NAME"
    ;;
  local|avoid)                           # 5 — local planner + obstacle_detector (/avoid_waypoints)
    remote 13 "cd ~/2026_IFAC && sc && ros2 launch local_planning local_planning.launch.py"
    ;;
  state)                                 # 6 — /state + /local_waypoints
    remote 15 "cd ~/2026_IFAC && sc && ros2 launch state_machine state_machine.launch.py"
    ;;
  control|ego)                           # 7 — L1 + 자전거 역모델 -> /drive (마지막에 띄울 것)
    remote 18 "cd ~/2026_IFAC && sc && ros2 launch f1tenth_control control_real.launch.py"
    ;;
  rviz)                                  # 젯슨 — RViz(X-forward) + rosbag 녹화(일시정지 상태로 시작)
    # 기본 레이아웃(local.terminator)에는 없다. 필요하면 별도 창에서:
    #   ~/2026_IFAC/real/run_real.sh rviz
    # 재개/정지는 어느 창에서나:
    #   ros2 service call /rosbag2_recorder/resume rosbag2_interfaces/srv/Resume
    #   ros2 service call /rosbag2_recorder/pause  rosbag2_interfaces/srv/Pause
    SSH_OPTS="-X -t" remote 10 'export LIBGL_ALWAYS_SOFTWARE=1 && cd ~/2026_IFAC && sc && BAG=~/rosbags/$(date +%m%d)/run_$(date +%m%d_%H%M%S) && mkdir -p ${BAG:h} && { ros2 bag record -a -s mcap --start-paused -o $BAG & BAGPID=$!; echo "[rec] $BAG — PAUSED로 시작 (재개: ros2 service call /rosbag2_recorder/resume rosbag2_interfaces/srv/Resume)"; rviz2 -d ~/2026_IFAC/src/kinematic_localization/rviz/kicp_real.rviz; kill -INT $BAGPID 2>/dev/null; wait $BAGPID 2>/dev/null; }'
    ;;
  rvizlocal)                             # 본체 PC — RViz만 로컬에서 (X-forward가 느리면 이쪽)
    # 🔴 2026-08-26 이후 이 role 은 **토픽을 하나도 못 받는다.** 젯슨 ~/.zshrc 가
    #    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST 라 DDS 가 젯슨 밖으로 안 나간다.
    #    시각화는 `run_real.sh foxglove` + 브라우저(ws://10.1.1.1:8765)를 쓸 것.
    #    이 role 을 되살리려면 젯슨의 그 export 를 지워야 한다(README 참고).
    print -P "%F{red}[rvizlocal] 경고: 젯슨 DDS 탐색이 LOCALHOST 로 묶여 있어 토픽이 안 온다 — foxglove role 을 쓸 것%f"
    export ROS_DOMAIN_ID=$DOMAIN RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    source /opt/ros/jazzy/setup.zsh 2>/dev/null
    source "$IFAC/install/setup.zsh" 2>/dev/null
    ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
    # 🔑 노이즈 많은 WiFi 전용 설정 — 모든 구독이 BEST_EFFORT·Depth 1 이다.
    #    RELIABLE 발행자 <-> BEST_EFFORT 구독자는 정상 매칭되므로 데이터는 그대로 오고,
    #    KICP(단일스레드 실행기)를 세우던 역압만 빠진다. docs/kinematic_localization.md §10-7-7
    RVIZCFG="${F1_RVIZ_CFG:-$IFAC/src/kinematic_localization/rviz/kicp_real.rviz}"
    if [[ -r "$RVIZCFG" ]]; then
      print -P "%F{green}[rvizlocal]%f rviz2 -d $RVIZCFG  (BEST_EFFORT · Fixed Frame map)"
      print -P "%F{242}(맵은 최대 10 s 뒤에 뜬다 — /map 구독이 Volatile 이라 래치 샘플을 안 받는다)%f"
      rviz2 -d "$RVIZCFG"
    else
      print -P "%F{yellow}[rvizlocal] 설정 파일 없음($RVIZCFG) — 기본 rviz2%f"
      rviz2
    fi
    print -P "%F{yellow}[rvizlocal] exited — dropping to an interactive shell%f"
    exec zsh -i
    ;;
  foxglove|fox)                          # 젯슨 — foxglove_bridge (WebSocket). 뷰어는 본체 PC 브라우저
    # 🔑 브릿지가 **젯슨 안에서** 구독하므로 DDS 가 WiFi 를 안 넘는다 — RViz 를 본체 PC 에서
    #    띄웠을 때 KICP 를 세우던 원격 RELIABLE 리더가 아예 없어진다(§10-7-7). `/tf` 도 포함.
    #    뷰어: https://app.foxglove.dev → Open connection → ws://${JETSON#*@}:8765
    # ⚠️ 젯슨에 패키지가 필요하다: sudo apt install ros-jazzy-foxglove-bridge
    # 🔑 브릿지와 함께 topic_tools throttle 을 띄워 /scan(40 Hz) -> /slow_scan 을 만든다.
    #    화이트리스트에는 /slow_scan 만 있고 /scan 은 일부러 빠져 있다 — 40 Hz LaserScan 을
    #    WiFi 로 넘기면 송신 버퍼가 밀려 화면이 뒤처지기 때문. 속도는 F1_SCAN_HZ 로 조절한다.
    #    브릿지가 끝나면 throttle 도 같이 정리한다.
    remote 10 "cd ~/2026_IFAC && sc && { ros2 run topic_tools throttle messages /scan ${SCAN_HZ} /slow_scan >/dev/null 2>&1 & THR=\$!; ros2 run foxglove_bridge foxglove_bridge --ros-args --params-file ~/2026_IFAC/real/foxglove_bridge.yaml; kill \$THR 2>/dev/null; }"
    ;;
  stop|clean|kill)                       # 이 스택의 노드만 원격 종료 (--all: bringup까지)
    print -P "%F{cyan}[stop] clearing the stack on $JETSON…%f"
    ssh "$JETSON" "pkill -f 'ros2 launch (kinematic_localization|global_planning|state_machine|f1tenth_control|local_planning|obstacle_detector)'; pkill -f 'ros2 bag record|rosbag2_recorder'; pkill -f foxglove_bridge; pkill -f 'throttle messages /scan'; pkill -f 'localization_node|global_trajectory_publisher_node|frenet_odom_node|state_machine_node|control_map_node|drive_source_selector|local_planner_node|obstacle_detector_node'" 2>/dev/null
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
