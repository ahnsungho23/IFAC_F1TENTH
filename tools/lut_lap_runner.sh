#!/usr/bin/env bash
# 실차 LUT 랩 러너 — lap_referee 1랩 절차의 실수 방지 자동화.
#
# 배경(2026-08-13 사고): referee는 랩 완주/no_start/timeout 때만 스스로 종료하며 파일을
# 쓴다. Ctrl+C는 저장 없이 죽는다. 또 젯슨 핫스팟(HY_MIRU)은 AP→클라이언트 멀티캐스트가
# 불안정해 SPDP 디스커버리가 복불복이었다 → ROS_STATIC_PEERS로 유니캐스트 디스커버리 고정.
#
# 사용 (랩탑, 젯슨 스택이 떠 있는 상태에서 랩마다):
#   bash tools/lut_lap_runner.sh slow20_lap1
#   bash tools/lut_lap_runner.sh slow20_lap2
#   ...
# 절차: 실행 → "odom 수신 확인" 뜨면 자율주행(A) 시작 → 완주하면 스스로 종료·저장 확인까지 출력.
# 차를 세워두고 실행하면 8초 뒤 no_start로 자기종료 + 파일 생성 = 배선 스모크 테스트로도 사용.
# ⚠️ set -u 금지: ROS setup.bash가 미정의 변수(AMENT_TRACE_SETUP_FILES 등)를 참조해 죽는다.
PREFIX=${1:?usage: lut_lap_runner.sh <prefix> [domain] [jetson_ip] [waypoints_csv] [outdir]}
DOMAIN=${2:-70}
JET=${3:-10.1.1.1}
WS=$HOME/2026_IFAC
WP=${4:-$WS/offline_trajectory_generator/output/map/global_waypoints.csv}
OUT=${5:-$HOME/lut_traces}

export ROS_DOMAIN_ID=$DOMAIN
export ROS_STATIC_PEERS=$JET
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
ros2 daemon stop >/dev/null 2>&1 || true

test -f "$WP" || { echo "🔴 waypoints_csv 없음: $WP"; exit 1; }

echo "[1/3] /pf/pose/odom 수신 확인 (domain=$DOMAIN, static_peer=$JET, 최대 15초)..."
if ! timeout 15 ros2 topic echo /pf/pose/odom --once >/dev/null 2>&1; then
  echo "🔴 /pf/pose/odom 미수신 — 주행을 시작하지 마세요."
  echo "   진단: bash $WS/src/f1tenth_control/tools/f1net_client.sh"
  echo "   ([4]의 수신 IP 목록에 젯슨($JET)이 있어야 정상입니다 — 자기 IP만 있으면 실패)"
  exit 1
fi
echo "🟢 odom 수신 확인."
echo ""
echo "[2/3] referee 시작 — 이제 자율주행(A)을 시작하세요."
echo "      ⚠️ Ctrl+C 금지: 랩 완주 시 referee가 'lap_complete'를 찍고 스스로 종료·저장합니다."
ros2 launch lap_referee lap_referee.launch.py \
  waypoints_csv:="$WP" odom_topic:=/pf/pose/odom \
  output_dir:="$OUT" output_prefix:="$PREFIX"

echo ""
echo "[3/3] 저장 확인:"
if ls -l "$OUT/${PREFIX}_summary.json" "$OUT/${PREFIX}_trace.csv" 2>/dev/null; then
  python3 - "$OUT/${PREFIX}_summary.json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
print(f"   terminated={d['terminated']} lap_completed={d['lap_completed']} "
      f"lap_time_s={d.get('lap_time_s')} mean_v={round(d.get('mean_speed_mps', 0), 2)}")
PY
else
  echo "🔴 파일이 없습니다 — referee가 자기종료 전에 중단됐습니다. 이 랩은 다시 돌아야 합니다."
  exit 1
fi
