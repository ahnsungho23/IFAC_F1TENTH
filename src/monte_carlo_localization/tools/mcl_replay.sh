#!/usr/bin/env bash
# =====================================================================================
# MCL 리플레이 하네스 — 실차 백을 노드에 다시 먹여 파라미터 변경의 효과를 재측정한다.
#
# 왜 필요한가: 백만 봐서는 "필터가 자기 우도 최댓값에 못 간다"까지는 알 수 있어도,
# 그 원인이 제안분포(motion_dispersion)인지 출력단(EKF/스무딩)인지 못 가린다.
# 같은 입력에 파라미터 하나만 바꿔 돌려야 가려진다.
#
#   사용:
#     tools/mcl_replay.sh <bag> <label> [--start S] [--duration D] [--warmup W] [key=value ...]
#
#   예:
#     tools/mcl_replay.sh rosbag2_2026_08_18-19_41_22 base   --start 172.3 --duration 15.5
#     tools/mcl_replay.sh rosbag2_2026_08_18-19_41_22 lat005 --start 172.3 --duration 15.5 \
#         motion_dispersion_y=0.05
#
#   결과: runs/mcl_replay/<label>/  (채점은 tools/mcl_score.py 로)
#
# ⚠️ 이 하네스는 '수렴 후 추종오차'를 잰다. 시작 포즈를 원본 백에서 씨앗으로 주기
#    때문에 초기 수렴 성능은 측정 대상이 아니다.
# ⚠️ 원본의 /tf 는 재생하지 않는다. 거기에는 옛 map->odom 이 들어 있어 이번 실행의
#    MCL 출력과 섞인다. 노드가 실제로 조회하는 것은 /tf_static 의 base_link->laser 뿐이다.
# =====================================================================================
set -euo pipefail

BAG="${1:?사용법: mcl_replay.sh <bag> <label> [options] [key=value ...]}"
LABEL="${2:?label 이 필요합니다}"
shift 2

START=0; DURATION=0; WARMUP=25
OVERRIDES=()
while [ $# -gt 0 ]; do
  case "$1" in
    --start)    START="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --warmup)   WARMUP="$2"; shift 2 ;;
    *=*)        OVERRIDES+=("$1"); shift ;;
    *) echo "알 수 없는 인자: $1" >&2; exit 2 ;;
  esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO"
OUT="runs/mcl_replay/${LABEL}"
rm -rf "$OUT"; mkdir -p "$(dirname "$OUT")"

# 다른 세션과 섞이지 않도록 전용 도메인.
export ROS_DOMAIN_ID="${MCL_REPLAY_DOMAIN:-77}"
export ROS_LOCALHOST_ONLY=1

# ROS 의 setup.bash 는 미설정 변수를 참조하므로 -u 를 잠시 끈다.
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

PLAY_START=$(python3 -c "print(max(0.0, $START - $WARMUP))")
PLAY_DUR=$(python3 -c "print($DURATION + ($START - max(0.0, $START - $WARMUP)) if $DURATION > 0 else 0)")

# --- 1. 파라미터 파일 만들기 (원본 + 오버라이드) -------------------------------------
CFG="$(mktemp -t mcl_replay_XXXX.yaml)"
python3 - "$CFG" "${OVERRIDES[@]:-}" <<'PY'
import sys, yaml
out = sys.argv[1]
overrides = [a for a in sys.argv[2:] if a and "=" in a]
src = "src/monte_carlo_localization/config/mcl_config.yaml"
with open(src) as fh:
    cfg = yaml.safe_load(fh)
node = cfg["particle_filter"]["ros__parameters"]
for item in overrides:
    k, v = item.split("=", 1)
    if k not in node:
        raise SystemExit(f"{src} 에 '{k}' 가 없습니다 — 오타를 막기 위해 거부합니다.")
    old = node[k]
    if isinstance(old, bool):
        new = v.strip().lower() in ("1", "true", "yes")
    elif isinstance(old, int):
        new = int(v)
    elif isinstance(old, float):
        new = float(v)
    else:
        new = v
    node[k] = new
    print(f"  오버라이드 {k}: {old} -> {new}")
with open(out, "w") as fh:
    yaml.safe_dump(cfg, fh, sort_keys=False, allow_unicode=True)
PY

# --- 2. 씨앗 포즈 뽑기 ---------------------------------------------------------------
read -r SX SY SYAW < <(python3 src/monte_carlo_localization/tools/mcl_pose_at.py "$BAG" "$PLAY_START")
echo "  씨앗 포즈 @${PLAY_START}s : x=$SX y=$SY yaw=$SYAW"

# --- 좀비 청소 -----------------------------------------------------------------------
# ros2 launch 를 죽여도 자식(particle_filter_node/map_server/lifecycle_manager)은 살아남는다.
# 살아남은 노드는 **자기 시계가 멈춘 채** /pf/pose/odom 을 계속 발행하고, 다음 실행의
# 같은 이름 노드와 충돌해 측정을 통째로 오염시킨다 (2026-08-18: 리플레이 결과가 원본보다
# 3배 나빴는데 전부 좀비가 낸 값이었다). 매번 시작 전에 지우고, 끝나면 프로세스 그룹째 죽인다.
kill_stale() {
  # 실행 파일 경로로 앵커한다 — 'particle_filter_node' 만 쓰면 그 문자열이 든 셸 명령까지 잡는다.
  local pat='lib/particle_filter_cpp/particle_filter_node|lib/nav2_map_server/map_server|lib/nav2_lifecycle_manager/lifecycle_manager'
  local found
  found=$(pgrep -f "$pat" || true)
  if [ -n "$found" ]; then
    echo "  이전 실행의 노드를 정리합니다: $(echo "$found" | tr '\n' ' ')"
    echo "$found" | xargs -r kill -9 2>/dev/null || true
    sleep 1
  fi
}
kill_stale

PIDS=()
PGIDS=()
cleanup() {
  for p in "${PIDS[@]:-}"; do kill -INT "$p" 2>/dev/null || true; done
  sleep 2
  for g in "${PGIDS[@]:-}"; do kill -9 -- "-$g" 2>/dev/null || true; done
  for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null || true; done
  kill_stale
  rm -f "$CFG"
}
trap cleanup EXIT

# --- 3. MCL 기동 ---------------------------------------------------------------------
setsid ros2 launch particle_filter_cpp mcl_launch.py \
     mod:=bag map_name:=map use_rviz:=false config_file:="$CFG" \
     > "runs/mcl_replay/${LABEL}.mcl.log" 2>&1 &
LAUNCH_PID=$!
PIDS+=("$LAUNCH_PID")
PGIDS+=("$LAUNCH_PID")

echo "  MCL 기동 대기..."
for i in $(seq 1 40); do
  if ros2 node list 2>/dev/null | grep -q particle_filter; then break; fi
  sleep 0.5
done
sleep 3
DUP=$(ros2 node list 2>/dev/null | grep -c '^/particle_filter$' || true)
if [ "$DUP" != "1" ]; then
  echo "  ✗ /particle_filter 가 ${DUP}개입니다 — 측정이 오염됩니다. 중단합니다." >&2
  exit 3
fi

# --- 4. 녹화 + 씨앗 + 재생 -----------------------------------------------------------
ros2 bag record -s mcap -o "$OUT" \
     /pf/pose/odom /pf/health /scan /drive_mode /estop_lock /odom \
     > "runs/mcl_replay/${LABEL}.rec.log" 2>&1 &
REC_PID=$!
PIDS+=("$REC_PID")
sleep 2

python3 src/monte_carlo_localization/tools/mcl_seed_pose.py \
        --x "$SX" --y "$SY" --yaw "$SYAW" --repeat 2 --period 0.5 \
        > "runs/mcl_replay/${LABEL}.seed.log" 2>&1 &
PIDS+=($!)

PLAY=(ros2 bag play "$BAG" --clock 200
      --topics /scan /odom /tf_static /drive_mode /estop_lock /global_waypoints)
[ "$(python3 -c "print(1 if $PLAY_START>0 else 0)")" = "1" ] && PLAY+=(--start-offset "$PLAY_START")
[ "$(python3 -c "print(1 if $PLAY_DUR>0 else 0)")" = "1" ]  && PLAY+=(--playback-duration "$PLAY_DUR")
echo "  재생: ${PLAY[*]}"
"${PLAY[@]}" > "runs/mcl_replay/${LABEL}.play.log" 2>&1

# 순서가 중요하다.
#  1) 먼저 MCL 을 내린다. 재생이 끝나면 /clock 이 멈추는데, 노드는 그대로 살아서
#     **얼어붙은 시각으로** /pf/pose/odom 을 계속 낸다. 녹화기를 먼저 기다리면 그
#     쓰레기가 수백 개 섞여 채점을 망친다 (2026-08-18: 793개가 들어갔다).
#  2) 그 다음 녹화기를 곱게 내린다. 강제 종료하면 metadata.yaml 이 안 써진다.
sleep 0.5
for g in "${PGIDS[@]:-}"; do kill -9 -- "-$g" 2>/dev/null || true; done
kill_stale
sleep 0.5
kill -INT "$REC_PID" 2>/dev/null || true
for i in $(seq 1 20); do
  kill -0 "$REC_PID" 2>/dev/null || break
  sleep 0.5
done
kill -9 "$REC_PID" 2>/dev/null || true
if [ ! -f "$OUT/metadata.yaml" ]; then
  echo "  metadata 가 없어 reindex 합니다"
  ros2 bag reindex "$OUT" -s mcap >/dev/null 2>&1 || true
fi
echo "  완료 -> $OUT"
