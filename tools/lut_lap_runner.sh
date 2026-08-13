#!/usr/bin/env bash
# 실차 LUT 랩 러너 — lap_referee 1랩 절차의 실수 방지 자동화.
#
# 배경(2026-08-13 사고): referee는 랩 완주/no_start/timeout 때만 스스로 종료하며 파일을
# 쓴다. Ctrl+C는 저장 없이 죽는다. 또 젯슨 핫스팟(HY_MIRU)은 AP→클라이언트 멀티캐스트가
# 불안정해 SPDP 디스커버리가 복불복이었다 → ROS_STATIC_PEERS로 유니캐스트 디스커버리 고정.
#
# 사용 (젯슨 스택이 떠 있는 상태에서 랩마다) — 랩탑·젯슨 어디서든 동작:
#   bash tools/lut_lap_runner.sh slow20_lap1
#   bash tools/lut_lap_runner.sh slow20_lap2
#   ...
# 권장은 "젯슨에서 실행" (Plan A, deploy_referee_to_jetson.sh 참고) — odom이 로컬이라
# wifi 디스커버리·수신 문제가 원천 제거된다. 랩탑 실행(Plan B)은 fastdds_car_client.xml
# 프로필로 참가자 인덱스 0~31까지 유니캐스트 디스커버리를 보내 복불복을 없앤다.
# 절차: 실행 → "odom 수신 확인" 뜨면 자율주행(A) 시작 → 완주하면 스스로 종료·저장 확인까지 출력.
# 차를 세워두고 실행하면 8초 뒤 no_start로 자기종료 + 파일 생성 = 배선 스모크 테스트로도 사용.
# ⚠️ set -u 금지: ROS setup.bash가 미정의 변수(AMENT_TRACE_SETUP_FILES 등)를 참조해 죽는다.
PREFIX=${1:?usage: lut_lap_runner.sh <prefix> [domain] [jetson_ip] [waypoints_csv] [outdir]}
DOMAIN=${2:-70}
JET=${3:-"10.1.1.1;10.1.1.10"}   # 피어 후보 전부 (세미콜론 목록)
WS=$HOME/2026_IFAC
WP=${4:-$WS/offline_trajectory_generator/output/map/global_waypoints.csv}
OUT=${5:-$HOME/lut_traces}

export ROS_DOMAIN_ID=$DOMAIN
export ROS_STATIC_PEERS="$JET"
# 2026-08-13 최종 원인 대응: STATIC_PEERS는 원격 참가자 인덱스 0~3까지만 닿는다
# (FastDDS maxInitialPeersRange 기본 4). 프로필이 있으면 0~31까지 유니캐스트 커버.
if [ -f "$WS/tools/fastdds_car_client.xml" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/tools/fastdds_car_client.xml"
fi
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
# 젯슨 배포본(deploy_referee_to_jetson.sh)은 lap_referee를 ~/lut_ws 오버레이에 빌드한다
if [ -f "$HOME/lut_ws/install/setup.bash" ]; then
  source "$HOME/lut_ws/install/setup.bash"
fi
ros2 daemon stop >/dev/null 2>&1 || true

# 파일 사본은 라이브 덤프 실패 시의 폴백일 뿐 — 없으면 라이브 덤프가 필수가 된다(젯슨 배포본)
if ! test -f "$WP"; then
  echo "⚠️  waypoints_csv 파일 사본 없음: $WP → 라이브 /global_waypoints 덤프 필수"
  WP=""
fi

echo "현재 네트워크: $(iwgetid -r 2>/dev/null || echo ?) / IP: $(hostname -I 2>/dev/null | awk '{print $1}')"
echo "[1/3] /pf/pose/odom 수신 확인 (domain=$DOMAIN, static_peer=$JET, 최대 15초)..."
if ! timeout 15 ros2 topic echo /pf/pose/odom --once >/dev/null 2>&1; then
  echo "🔴 /pf/pose/odom 미수신 — 주행을 시작하지 마세요. 순서대로 확인:"
  echo "   ① 젯슨 스택(T1 f110 + T2 MCL)이 떠 있고 /pf/pose/odom 을 발행 중인가"
  echo "   ② 랩탑이 젯슨과 같은 망인가 (위의 '현재 네트워크' 줄 확인)"
  echo "   ③ 진단: bash $WS/src/f1tenth_control/tools/f1net_client.sh"
  echo "      (⚠️ [4]에 자기 IP만 떠도 HY_MIRU에선 정상 — 멀티캐스트가 원래 안 흐르는 망이라"
  echo "       유니캐스트 프로필로 우회한다. [1] 네트워크·[2] ROS 환경 항목을 볼 것)"
  exit 1
fi
echo "🟢 odom 수신 확인."

# 차가 실제로 따르는 라인을 라이브로 덤프해 referee 기준으로 사용한다.
# 파일 사본(랩탑 vs 젯슨)이 다르거나 순서가 반대면 진행거리가 누적되지 않아
# lap_complete가 영원히 안 뜬다(2026-08-13 실측 사고) — 라이브 덤프가 원천 차단.
mkdir -p "$OUT"
LIVE_WP="$OUT/live_global_waypoints.csv"
# 덤프 스크립트 위치: 랩탑=저장소 tools/, 젯슨 배포본=~/lut_ws/ (deploy 스크립트가 복사)
DUMPER="$WS/tools/dump_global_waypoints.py"
[ -f "$DUMPER" ] || DUMPER="$HOME/lut_ws/dump_global_waypoints.py"
if [ -f "$DUMPER" ] && python3 "$DUMPER" "$LIVE_WP" 10; then
  WP="$LIVE_WP"
  echo "🟢 라이브 /global_waypoints 사용: $WP"
else
  if [ -z "$WP" ]; then
    echo "🔴 라이브 /global_waypoints 미수신 + 파일 사본도 없음 — referee를 시작할 수 없습니다."
    echo "   글로벌 플래너(T3)가 떠 있는지 확인하세요."
    exit 1
  fi
  echo "⚠️  /global_waypoints 미수신 — 파일 사본으로 진행: $WP"
  echo "   (파일이 젯슨 발행 라인과 다르면 랩 판정이 안 될 수 있음)"
fi
echo ""
echo "[2/3] referee 시작 — 이제 자율주행(A)을 시작하세요."
echo "      ⚠️ Ctrl+C 금지: 랩 완주 시 referee가 'lap_complete'를 찍고 스스로 종료·저장합니다."
echo "      (5초마다 HB 로그가 떠야 정상 — progress가 안 올라가면 그 줄을 그대로 보고할 것)"
# 병렬 odom 주기 기록 — referee가 굶는지(수신 두절) 외부에서 교차 검증
ros2 topic hz /pf/pose/odom > "$OUT/${PREFIX}_odom_hz.log" 2>&1 &
HZ_PID=$!
ros2 launch lap_referee lap_referee.launch.py \
  waypoints_csv:="$WP" odom_topic:=/pf/pose/odom \
  output_dir:="$OUT" output_prefix:="$PREFIX"
kill $HZ_PID 2>/dev/null; wait $HZ_PID 2>/dev/null

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
