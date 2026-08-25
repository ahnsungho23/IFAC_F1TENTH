#!/usr/bin/env bash
# =================================================================================================
# Open the real-car FULL stack (local_planning + obstacle_detector included) in ONE Terminator
# window. Global-only driving (no local_planning)는 ./real/open_real.sh 를 쓸 것.
#
#   ./real/local.sh
#
# Panes (실차 실행 순서, 전부 젯슨 ssh):
#   1 bringup(시각동기화 → sudo jetson_clocks && f110) · 2 ping 192.168.0.10 · 3 kinematic_localization
#   4 global_planning · 5 local_planning(+obstacle_detector) · 6 state_machine · 7 control
#   RViz+bag 은 이 레이아웃에 없다 — 별도 창에서 ./real/run_real.sh rviz
#
# Substitutes this directory's absolute path into the layout and launches Terminator with a
# DEDICATED config (-g) + --no-dbus (a second `terminator` call would otherwise be handed to the
# running DBus server, which ignores -g/-l).
#
# Env overrides (read by real/run_real.sh in each pane):
#   F1_HOST=miru@10.1.1.1
#   F1_MAP_NAME=map
# =================================================================================================
set -euo pipefail

REALDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAYOUT="f1real_local"
SRC="$REALDIR/local.terminator"

if ! command -v terminator >/dev/null 2>&1; then
  echo "terminator is not installed. Install it (sudo apt install terminator) or open the panes" >&2
  echo "manually — see src/f1tenth_control/LAUNCH_FULLSTACK.md §3." >&2
  exit 1
fi

GEN="${TMPDIR:-/tmp}/ifac_real_${LAYOUT}.terminator"
sed "s#@REALDIR@#${REALDIR}#g" "$SRC" > "$GEN"

echo "launching Terminator layout '${LAYOUT}' (config: $GEN)"
nohup terminator --no-dbus -g "$GEN" -l "$LAYOUT" >/dev/null 2>&1 &
disown
echo "done — the '${LAYOUT}' window should be opening (pid $!)."
