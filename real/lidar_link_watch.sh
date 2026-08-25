#!/usr/bin/env bash
# =================================================================================================
# 라이다 이더넷 링크 — 기동 시 1회 점검 + USB-C 허브 복구, 이후에는 감시만
#   (젯슨에서 실행 — real/run_real.sh ping 이 띄운다)
#
#   ./lidar_link_watch.sh [호스트]        기본 192.168.0.10 (UST-10LX)
#
# 동작은 딱 두 단계다.
#   ① 기동 점검 (1회)  : ping 을 몇 번 던져 본다. "Destination Host Unreachable" 이면
#                        usbc_power.sh cycle --force 로 허브를 재부착하고 다시 확인한다.
#   ② 이후            : 그냥 ping 을 계속 흘린다. **자동 복구는 더 이상 하지 않는다.**
#
# 🔴 ②에서 자동 복구를 안 하는 이유:
#    허브에는 라이다 이더넷과 **조이스틱이 같이** 물려 있다. 주행 중에 허브를 끊으면
#    /joy 가 죽어 사람이 조이스틱으로 E-stop 을 걸 수 없다. 기동 시점(차가 서 있고
#    아직 자율 체결 전)에만 복구하는 게 안전하다.
#    주행 중에 링크가 죽으면 이 창이 unreachable 을 찍어 주고, 복구는 사람이 판단해
#    수동으로 돌린다:  src/f1tenth_control/tools/usbc_power.sh cycle --force
#
# 왜 필요한가 (2026-08-25 실차 백):
#   충돌 1.3 초 뒤 라이다가 죽고 urg_node 가 89 초 내내 "Not Connected" 였는데, 현장에서
#   "이더넷 링크가 끊긴 것"과 "라이다 본체가 죽은 것"을 가를 계기가 없었다. 전자는 허브
#   재부착으로 살아나고 후자는 안 살아난다.
#   (docs: src/kinematic_localization/docs/kinematic_localization.md §10-7)
#
# 환경변수:
#   LIDAR_PROBE_COUNT=5     기동 점검에 던질 ping 개수
#   LIDAR_PROBE_DEADLINE=6  기동 점검 제한시간 [s] — ⚠️ 3.5초 밑으로 줄이지 말 것
#   LIDAR_MAX_CYCLES=2      기동 점검에서 허브를 재부착해 볼 최대 횟수
#   LIDAR_SETTLE=12         재부착 뒤 링크가 올라오길 기다리는 시간 [s]
#   LIDAR_AUTOCYCLE=1       0 이면 점검만 하고 복구는 안 한다(명령어만 찍어 줌)
#   USBC_POWER_SH=$HOME/2026_IFAC/src/f1tenth_control/tools/usbc_power.sh
# =================================================================================================
set -uo pipefail

HOST="${1:-192.168.0.10}"
COUNT="${LIDAR_PROBE_COUNT:-5}"
DEADLINE="${LIDAR_PROBE_DEADLINE:-6}"
MAX_CYCLES="${LIDAR_MAX_CYCLES:-2}"
SETTLE="${LIDAR_SETTLE:-12}"
AUTO="${LIDAR_AUTOCYCLE:-1}"
USBC="${USBC_POWER_SH:-$HOME/2026_IFAC/src/f1tenth_control/tools/usbc_power.sh}"

inf() { printf '\033[1;36m[link] %s\033[0m\n' "$*"; }
say() { printf '\033[1;33m[link] %s\033[0m\n' "$*"; }
err() { printf '\033[1;31m[link] %s\033[0m\n' "$*"; }
ok()  { printf '\033[1;32m[link] %s\033[0m\n' "$*"; }

# probe — 0: 응답 있음 / 1: unreachable(허브 복구 대상) / 2: 무응답(복구 대상 아님)
#
# 🔴 짧게 재면 안 된다. ARP 해석이 실패해 커널이 "Destination Host Unreachable" 을
#    내기까지 **약 3.1 초**가 걸린다(2026-08-25 젯슨 실측). ARP 캐시가 비어 있는
#    기동 시점이 정확히 그 경우라, `-i 0.3` 같은 빠른 점검은 2.3 초에 끝나 버려
#    진짜 unreachable 을 "무응답"으로 오판하고 복구를 건너뛴다. 실측 결과:
#      -c 5 -W 1 -i 0.3 -> ❌ 놓침(2.3 s) / -c 5 -w 6 -> ✅ 잡힘(3.1 s)
probe() {
  local out
  out=$(ping -c "$COUNT" -w "$DEADLINE" "$HOST" 2>&1)
  case "$out" in
    *" bytes from "*) PROBE_MSG=$(printf '%s\n' "$out" | tail -2 | head -1); return 0 ;;
  esac
  case "$out" in
    *"Destination Host Unreachable"*) PROBE_MSG="Destination Host Unreachable";        return 1 ;;
    *"Network is unreachable"*|*"Network is down"*)
                                      PROBE_MSG="Network is unreachable (어댑터 자체가 없다)"; return 1 ;;
    *)                                PROBE_MSG="무응답(타임아웃) — unreachable 은 아니다";     return 2 ;;
  esac
}

# ── ① 기동 시 1회 점검 ────────────────────────────────────────────────────────
inf "기동 점검: $HOST 에 ping ${COUNT}회"
probe; rc=$?
if [ "$rc" -eq 0 ]; then
  ok "링크 정상 — $PROBE_MSG"
elif [ "$rc" -eq 2 ]; then
  say "$PROBE_MSG"
  say "허브 문제로 보이지 않아 재부착은 하지 않는다. 라이다 전원/부팅을 확인할 것."
elif [ "$AUTO" != "1" ]; then
  err "$PROBE_MSG"
  say "자동 복구 꺼짐(LIDAR_AUTOCYCLE=0) — 수동: $USBC cycle --force"
elif [ ! -x "$USBC" ]; then
  err "$PROBE_MSG"
  err "usbc_power.sh 를 찾을 수 없다: $USBC — 복구 못 함"
else
  err "$PROBE_MSG"
  n=0
  while [ "$n" -lt "$MAX_CYCLES" ]; do
    n=$((n + 1))
    err "USB-C 허브 강제 재부착 (${n}/${MAX_CYCLES}) — 이 동안 /joy 도 끊긴다"
    "$USBC" cycle --force || err "usbc_power.sh 종료코드 $?"
    inf "링크가 올라오길 ${SETTLE}초 기다린다"
    sleep "$SETTLE"
    if probe; then
      ok "복구 성공 — $PROBE_MSG"
      say "⚠️ urg_node 는 그 사이 죽었을 수 있다. 1번 창(bringup)을 다시 띄울 것."
      break
    fi
    err "아직 안 된다 — $PROBE_MSG"
  done
  if [ "$n" -ge "$MAX_CYCLES" ] && ! probe; then
    err "${MAX_CYCLES}회 재부착해도 안 살아난다 — 라이다 본체/커넥터 물리 손상을 의심할 것."
  fi
fi

# ── ② 이후에는 감시만 (자동 복구 없음) ────────────────────────────────────────
echo
inf "이제부터는 감시만 한다 — 주행 중 unreachable 이 보이면 사람이 판단해서:"
inf "    $USBC cycle --force"
echo
exec stdbuf -oL ping -O "$HOST"
