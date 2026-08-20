#!/usr/bin/env bash
# =================================================================================================
# Open the full simulation stack in ONE Terminator window.
#
#   ./sim/open_sim.sh              # 6-pane base loop (CLAUDE.md terminals 1-6)
#   ./sim/open_sim.sh --opp        # 8-pane full loop with opponent sim + detector (terminals 1-8)
#
# It substitutes the absolute path of this directory into the layout (so the repo can live
# anywhere) and launches Terminator with a DEDICATED config (-g), leaving your personal
# ~/.config/terminator/config untouched.
#
# --no-dbus is REQUIRED: without it a second `terminator` invocation is handed to the running
# Terminator DBus server, which IGNORES -g/-l and just opens a default empty window. --no-dbus
# forces a standalone process that actually applies our layout.
#
# Useful env overrides (read by sim/run.sh in each pane):
#   SIM_MAP_NAME=<map> ./sim/open_sim.sh       # localization map (default: ifac_track)
#   F1SIM_WS=~/f1sim_C                         # f1tenth_gym_ros workspace path
# =================================================================================================
set -euo pipefail

SIMDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAYOUT="f1sim"
SRC="$SIMDIR/f1sim.terminator"

if [[ "${1:-}" == "--opp" || "${1:-}" == "-o" ]]; then
  LAYOUT="f1sim_opp"
  SRC="$SIMDIR/f1sim_opp.terminator"
fi

if ! command -v terminator >/dev/null 2>&1; then
  echo "terminator is not installed. Install it (sudo apt install terminator) or open the panes" >&2
  echo "manually — see $SIMDIR/README.md for the per-terminal commands." >&2
  exit 1
fi

GEN="${TMPDIR:-/tmp}/ifac_sim_${LAYOUT}.terminator"
sed "s#@SIMDIR@#${SIMDIR}#g" "$SRC" > "$GEN"

echo "launching Terminator layout '${LAYOUT}' (config: $GEN)"
nohup terminator --no-dbus -g "$GEN" -l "$LAYOUT" >/dev/null 2>&1 &
disown
echo "done — the '${LAYOUT}' window should be opening (pid $!)."
