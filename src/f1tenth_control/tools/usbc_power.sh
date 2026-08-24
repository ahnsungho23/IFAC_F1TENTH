#!/usr/bin/env bash
# 젯슨 USB-C 포트를 소프트웨어로 껐다 켠다 (Orin Nano Dev Kit Super 전용).
#
# 배경 (2026-08-24~25 실측):
#   USB-C 포트 = xHCI 루트포트 usb1-port1 (pad usb2-0). 롤과 VBUS는 fusb301
#   Type-C 컨트롤러(I2C 1-0025)가 쥔다. 제어 수단이 두 개인데 효과가 서로 다르다.
#
#   ① 루트포트 disable  → 지속 차단 O / 먹통 복구 X
#      usb1-port1/disable=1 이면 장치가 전부 사라지고 그 상태가 유지된다(VBUS는 유지).
#      0으로 되돌리면 2초 안에 복귀한다. 그래서 off/on 스위치로 쓴다.
#      단, 이미 먹통이 된 포트에는 아무 반응이 없다 (실측: dmesg 무반응).
#
#   ② fusb301 fmode SNK(4)→SRC(1)  → 지속 차단 X / 먹통 복구 O
#      쓰는 순간 detach/attach가 한 번 일어나고 드라이버는 곧바로 SRC로 되돌아온다
#      (MODES 레지스터가 항상 0x01로 복귀하므로 "계속 꺼두기"에는 쓸 수 없다).
#      대신 이것만이 먹통 상태를 실제로 깬다. 그래서 cycle(복구)로 쓴다.
#
#   먹통 상태란: fusb301은 ATTACH=1·VBUSOK=1·롤 host 로 정상인데 xHCI 열거만 0건인
#   상태. 부팅 시 fusb301이 DRP+ACC로 토글하다 detach로 흘려버리고, 뒤늦게 SRC로
#   붙어도 padctl 롤 전환 핸드오프가 어긋나면 이렇게 된다 (dmesg에 padctl usb2-0
#   <-> fusb301 connector@0 "Fixed dependency cycle" 경고 동반). dmesg에 disconnect도
#   error도 안 남고 over_current_count도 0이라 전원·케이블 문제로 보이지 않는다.
#   fusb301 freset(칩 리셋)은 듣지 않는다 — 실측 확인.
#
# 사용법 (젯슨에서 직접 실행):
#   usbc_power.sh status    현재 상태 + 허브에 물린 장치 목록
#   usbc_power.sh off       포트 차단 (on 할 때까지 유지)
#   usbc_power.sh on        차단 해제 + 복귀 대기. 안 오면 자동으로 cycle까지 간다
#   usbc_power.sh cycle     강제 재부착. 허브가 먹통일 때 쓰는 복구 명령
#
#   옵션:
#     --force              주행 스택이 떠 있어도 강행
#     --off-seconds N      cycle에서 내려 두는 시간 (기본 3)
#     --wait-seconds N     복귀 대기 한도 (기본 15)
#
# ⚠️ 이 허브에는 라이다 이더넷(RTL8152 → 192.168.0.x)과 조이스틱(F710)이 물려 있다.
#    끊으면 /scan·/joy가 통째로 죽는다. 주행 스택이 감지되면 기본적으로 거부하고,
#    정말 필요할 때만 --force 로 강행한다.

set -uo pipefail

# --- 설정값 (환경변수로 덮어쓸 수 있음) ---------------------------------------
FUSB_DIR="${FUSB301_DIR:-/sys/bus/i2c/devices/1-0025/fusb301}"
USBC_PORT="${USBC_PORT:-/sys/bus/usb/devices/usb1/1-0:1.0/usb1-port1}"
ROLE_FILE="${USBC_ROLE_FILE:-/sys/class/usb_role/usb2-0-role-switch/role}"
# USB-C 포트 밑으로 열거되는 장치의 sysfs 접두어. 1-1=USB2 계통, 2-2=USB3 계통.
USBC_DEV_PREFIXES="${USBC_DEV_PREFIXES:-1-1 2-2}"
STACK_PATTERN="${USBC_STACK_PATTERN:-urg_node|vesc_driver|joy_node|ackermann_to_vesc|controller_node|local_planner|state_machine|global_planner}"
# 마지막으로 정상이던 장치 개수. on 이 몇 개를 기다려야 하는지 여기서 읽는다.
STATE_FILE="${USBC_STATE_FILE:-/tmp/usbc_power.baseline}"

# fusb301 fmode 값 = FUSB301 MODES 레지스터 비트 (1=SRC, 4=SNK, 16=DRP)
MODE_SRC=1
MODE_SNK=4

OFF_SECONDS=3
WAIT_SECONDS=15
FORCE=0

die() { echo "❌ $*" >&2; exit 1; }
usage() { sed -n '2,45p' "$0"; }

# --- 인자 파싱 ----------------------------------------------------------------
CMD=""
while [ $# -gt 0 ]; do
  case "$1" in
    status|off|on|cycle) CMD="$1"; shift ;;
    --force)             FORCE=1; shift ;;
    --off-seconds)       OFF_SECONDS="$2"; shift 2 ;;
    --wait-seconds)      WAIT_SECONDS="$2"; shift 2 ;;
    -h|--help)           usage; exit 0 ;;
    *)                   die "알 수 없는 인자: $1  (status|off|on|cycle)" ;;
  esac
done
[ -n "$CMD" ] || { usage; exit 1; }

[ -d "$FUSB_DIR" ]   || die "fusb301이 없다: $FUSB_DIR (젯슨에서 실행하는 게 맞는지 확인)"
[ -w "$USBC_PORT/disable" ] || [ -e "$USBC_PORT/disable" ] \
  || die "루트포트를 찾을 수 없다: $USBC_PORT"

SUDO=""
[ "$(id -u)" -ne 0 ] && SUDO="sudo"

# --- 상태 조회 헬퍼 -----------------------------------------------------------
status_reg() { sed -n 's/^from 0x11 read 0x//p' "$FUSB_DIR/fregdump"; }
# CC에 무언가 물려 있나. 0이면 케이블 자체가 안 꽂힌 것이라 소프트웨어로 할 게 없다.
is_attached() { [ $(( 0x$(status_reg) & 0x01 )) -eq 1 ]; }
require_attached() {
  is_attached && return 0
  echo "ℹ️  USB-C 포트에 아무것도 부착돼 있지 않다 (ATTACH=0, CC=0)." >&2
  die "허브 케이블이 빠졌거나 허브 전원이 없다. 물리적으로 꽂은 뒤 다시 실행할 것."
}

# USB-C 포트 아래에 열거된 USB 장치들 (':' 붙은 건 인터페이스라 제외)
usbc_devices() {
  local p
  for p in $USBC_DEV_PREFIXES; do
    ls -d "/sys/bus/usb/devices/$p" "/sys/bus/usb/devices/$p."* 2>/dev/null
  done | grep -v ':'
}
usbc_count() { usbc_devices | wc -l; }

print_devices() {
  local d n=0
  while read -r d; do
    [ -n "$d" ] || continue
    n=$((n + 1))
    printf "    %-8s %s:%s %s\n" "$(basename "$d")" \
      "$(cat "$d/idVendor" 2>/dev/null)" "$(cat "$d/idProduct" 2>/dev/null)" \
      "$(cat "$d/product" 2>/dev/null)"
  done < <(usbc_devices)
  [ "$n" -eq 0 ] && echo "    (없음)"
  return 0
}

save_baseline() {   # 0개를 저장하면 다음 on 이 아무것도 안 기다리게 되므로 막는다
  local n; n="$(usbc_count)"
  [ "$n" -gt 0 ] && { echo "$n" > "$STATE_FILE" 2>/dev/null || true; }
  return 0
}
load_baseline() {
  local b; b="$(cat "$STATE_FILE" 2>/dev/null)"
  if [[ "$b" =~ ^[0-9]+$ ]] && [ "$b" -gt 0 ]; then echo "$b"; else echo 1; fi
}

print_status() {
  local st attach vbus dis
  st="$(status_reg)"
  attach=$(( 0x$st & 0x01 ))
  vbus=$(( (0x$st >> 3) & 0x01 ))
  dis="$(cat "$USBC_PORT/disable" 2>/dev/null)"
  echo "USB-C 포트 상태"
  echo "  fusb301   mode=$(cat "$FUSB_DIR/fmode") type=$(cat "$FUSB_DIR/ftype")" \
       "CC=$(cat "$FUSB_DIR/ftypec_cc_orientation") STATUS=0x$st (ATTACH=$attach VBUSOK=$vbus)"
  echo "  롤        $(cat "$ROLE_FILE" 2>/dev/null || echo '?')"
  echo "  루트포트  disable=$dis state=$(cat "$USBC_PORT/state" 2>/dev/null || echo '?')" \
       "over_current_count=$(cat "$USBC_PORT/over_current_count" 2>/dev/null || echo '?')"
  echo "  열거 장치 $(usbc_count)개"
  print_devices
  if [ "$dis" = "1" ]; then
    echo "  → off 상태다. 'usbc_power.sh on' 으로 켤 것."
  elif [ "$attach" = "1" ] && [ "$vbus" = "1" ] && [ "$(usbc_count)" -eq 0 ]; then
    echo "  → 먹통 상태다 (부착·급전 중인데 열거 0건). 'usbc_power.sh cycle' 로 복구할 것."
  fi
}

# --- 주행 스택 가드 -----------------------------------------------------------
guard_running_stack() {
  local hits
  hits="$(pgrep -af "$STACK_PATTERN" 2>/dev/null)"
  [ -n "$hits" ] || return 0
  echo "⚠️  주행 스택으로 보이는 프로세스가 떠 있다:" >&2
  echo "$hits" | sed 's/^/      /' >&2
  if [ "$FORCE" -eq 1 ]; then
    echo "    --force 지정됨 → 강행한다. /scan·/joy가 끊긴다." >&2
    return 0
  fi
  die "주행 중 USB-C를 끊으면 라이다·조이스틱이 통째로 죽는다. 정말 필요하면 --force."
}

# --- 저수준 동작 --------------------------------------------------------------
set_disable() {
  echo "$1" | $SUDO tee "$USBC_PORT/disable" >/dev/null \
    || die "루트포트 disable 쓰기 실패 (sudo 권한 확인)"
}
set_mode() {
  echo "$1" | $SUDO tee "$FUSB_DIR/fmode" >/dev/null \
    || die "fusb301 fmode 쓰기 실패 (sudo 권한 확인)"
}

# wait_devices <목표 개수> — 복귀를 초 단위로 기다리며 진행 상황을 찍는다
wait_devices() {
  local want="$1" i n
  for ((i = 1; i <= WAIT_SECONDS; i++)); do
    sleep 1
    n="$(usbc_count)"
    printf "\r  +%02ds 장치 %s/%s개  " "$i" "$n" "$want"
    [ "$n" -ge "$want" ] && break
  done
  sleep 2   # 조이스틱처럼 늦게 올라오는 장치까지 세고 나서 보고한다
  echo
  [ "$(usbc_count)" -ge "$want" ]
}

# 강제 재부착: 먹통을 깨는 유일한 수단
force_reattach() {
  echo "▶ 강제 재부착 (fmode SNK→SRC)"
  set_mode "$MODE_SNK"
  sleep "$OFF_SECONDS"
  set_mode "$MODE_SRC"
}

# --- 명령 실행 ----------------------------------------------------------------
case "$CMD" in
  status)
    print_status
    ;;

  off)
    guard_running_stack
    echo "▶ 포트 차단 (disable=1) — 현재 $(usbc_count)개"
    save_baseline          # on 이 몇 개를 기다릴지 기억해 둔다
    set_disable 1
    sleep 2
    N="$(usbc_count)"
    if [ "$N" -eq 0 ]; then
      echo "✅ 차단 완료. 'usbc_power.sh on' 으로 켤 것."
    else
      echo "⚠️  아직 ${N}개가 남아 있다 (예상 밖). status 로 확인할 것." >&2
      exit 1
    fi
    ;;

  on)
    WANT="$(load_baseline)"
    echo "▶ 차단 해제 (disable=0) — ${WANT}개 복귀를 기다린다"
    set_disable 0
    sleep 1
    require_attached
    if wait_devices "$WANT"; then
      echo "✅ 복귀 완료 ($(usbc_count)개)"
      save_baseline
      print_devices
    else
      echo "  다 안 돌아왔다 → 강제 재부착으로 넘어간다"
      force_reattach
      if wait_devices "$WANT"; then
        echo "✅ 복귀 완료 ($(usbc_count)개)"
        save_baseline
        print_devices
      else
        print_status
        die "복구 실패. 케이블을 물리적으로 뽑았다 꽂거나 재부팅할 것."
      fi
    fi
    ;;

  cycle)
    guard_running_stack
    set_disable 0                 # off 상태로 남아 있었다면 먼저 풀어 준다
    sleep 1
    require_attached
    BASE="$(usbc_count)"
    [ "$BASE" -eq 0 ] && BASE="$(load_baseline)"
    echo "사이클 시작 — 현재 $(usbc_count)개, 목표 ${BASE}개"
    force_reattach
    if ! wait_devices "$BASE"; then
      echo "▶ 1회 재시도"
      force_reattach
      wait_devices "$BASE" || { print_status; die "복구 실패. 케이블을 뽑았다 꽂거나 재부팅할 것."; }
    fi
    echo "✅ 복귀 완료 ($(usbc_count)개)"
    save_baseline
    print_devices
    ;;
esac
