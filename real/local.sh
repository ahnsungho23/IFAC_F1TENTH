#!/usr/bin/env bash
# =================================================================================================
# Open the real-car FULL stack (local_planning + obstacle_detector included) in ONE Terminator
# window. Global-only driving (no local_planning)는 ./real/open_real.sh 를 쓸 것.
#
#   ./real/local.sh
#
# Panes (LAUNCH_FULLSTACK.md §3 full order):
#   1 bringup · 2 mcl · 3 global_planning · 4 local_planning(+obstacle_detector)
#   5 state_machine · 6 control · 7 rviz+bag (전부 젯슨 ssh, 7번만 -X)
#
# Substitutes this directory's absolute path into the layout and launches Terminator with a
# DEDICATED config (-g) + --no-dbus (a second `terminator` call would otherwise be handed to the
# running DBus server, which ignores -g/-l).
#
# Env overrides (read by real/run_real.sh in each pane):
#   F1_HOST=miru@10.1.1.3
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
