#!/usr/bin/env bash
# lap_referee를 젯슨에서 직접 돌리기 위한 1회 배포 스크립트 (랩탑에서 실행).
#
# 왜 (2026-08-13): 랩탑에서 referee를 돌리면 odom이 wifi 디스커버리·수신에 의존해
# 복불복이 났다. referee가 젯슨 로컬이면 odom이 같은 호스트라 wifi가 데이터 경로에서
# 완전히 빠진다 — 가장 확실한 해결(Plan A). trace 타이밍 품질도 더 좋다.
#
# 사용:  bash tools/deploy_referee_to_jetson.sh [jetson_ip(기본 10.1.1.1)]
# 배포 후 (젯슨 터미널):   bash ~/lut_lap_runner.sh slow20_lap1
# 랩 종료 후 회수 (랩탑):  mkdir -p ~/lut_traces && scp "miru@<ip>:~/lut_traces/*" ~/lut_traces/
# ⚠️ 랩탑에서 lap_referee 소스나 lut_lap_runner.sh를 고치면 젯슨 사본은 자동으로
#    안 따라온다 — 이 스크립트를 다시 실행해 재배포할 것.
#
# 비밀번호를 여러 번 묻는 게 귀찮으면 먼저 1회: ssh-copy-id miru@<ip>
set -e
JET=${1:-10.1.1.1}
WS=$HOME/2026_IFAC

echo "[1/3] 소스·도구 복사 → miru@$JET"
ssh "miru@$JET" 'mkdir -p ~/lut_ws/src ~/lut_traces'
scp -r "$WS/src/lap_referee" "miru@$JET:~/lut_ws/src/"
scp "$WS/tools/dump_global_waypoints.py" "miru@$JET:~/lut_ws/"
scp "$WS/tools/lut_lap_runner.sh" "miru@$JET:~/"

echo "[2/3] 젯슨에서 lap_referee 빌드 (~1분)"
# ⚠️ ssh 원격 명령은 상대 유저의 "로그인 셸 -c"로 돈다 — 젯슨 유저(miru)는 zsh라서
# setup.bash를 zsh로 소싱하면 BASH_SOURCE가 비어 즉시 실패한다. bash -c로 강제할 것.
# (~/2026_IFAC 언더레이는 lap_referee 빌드에 필수는 아니라 있을 때만 소싱)
ssh "miru@$JET" "bash -c '\
  source /opt/ros/jazzy/setup.bash \
  && { [ -f ~/2026_IFAC/install/setup.bash ] && source ~/2026_IFAC/install/setup.bash || true; } \
  && cd ~/lut_ws \
  && colcon build --packages-select lap_referee \
  && echo BUILD_OK'"

echo "[3/3] 완료. 다음부터 랩마다 젯슨 터미널에서:"
echo "      bash ~/lut_lap_runner.sh <prefix>"
echo "      (runner가 ~/lut_ws 오버레이를 자동 source — 추가 설정 불필요)"
echo "      랩 끝나면 랩탑에서: scp \"miru@$JET:~/lut_traces/*\" ~/lut_traces/"
