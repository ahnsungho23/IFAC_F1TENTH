#!/usr/bin/env bash
# =================================================================================================
# Open the real-car stack (global-only driving, NO local_planning) in ONE Terminator window.
#
#   ./real/open_real.sh
#
# Panes 1-6 all SSH into the Jetson ($F1_HOST, default miru@10.1.1.3); pane 6 additionally uses
# X-forwarding for RViz and starts `ros2 bag record -a` in PAUSED state.
# Order inside the window follows LAUNCH_FULLSTACK.md §3 minus T4 (local_planning):
#   1 bringup · 2 mcl · 3 global_planning · 4 state_machine · 5 control · 6 rviz+bag
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
LAYOUT="f1real"
SRC="$REALDIR/global.terminator"

if ! command -v terminator >/dev/null 2>&1; then
  echo "terminator is not installed. Install it (sudo apt install terminator) or open the panes" >&2
  echo "manually — see src/f1tenth_control/LAUNCH_FULLSTACK.md §3 (skip T4 local_planning)." >&2
  exit 1
fi

GEN="${TMPDIR:-/tmp}/ifac_real_${LAYOUT}.terminator"
sed "s#@REALDIR@#${REALDIR}#g" "$SRC" > "$GEN"

echo "launching Terminator layout '${LAYOUT}' (config: $GEN)"
nohup terminator --no-dbus -g "$GEN" -l "$LAYOUT" >/dev/null 2>&1 &
disown
echo "done — the '${LAYOUT}' window should be opening (pid $!)."
