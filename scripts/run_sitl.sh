#!/usr/bin/env bash
# Start coptersim + PX4 SITL for interactive use (e.g. with QGroundControl).
# PX4's interactive shell (pxh>) runs in this terminal; type `shutdown` or Ctrl-C to quit.
#
# Usage: scripts/run_sitl.sh [path/to/PX4-Autopilot]
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/scripts/platform.sh"
PX4_DIR="${1:-${PX4_DIR:-$HOME/tools/PX4-Autopilot}}"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
SIM="$(rfly_bin coptersim)"
LOG_DIR="$(rfly_logs)"; rfly_set_dbarg

rfly_need_bins coptersim || exit 1
[ -x "$PX4_BUILD/bin/px4" ] || { echo "PX4 SITL not built at $PX4_BUILD (make px4_sitl_default)"; exit 1; }

"$SIM" ${DBARG[@]+"${DBARG[@]}"} >"$LOG_DIR/coptersim.log" 2>&1 &
SIM_PID=$!
trap 'kill $SIM_PID 2>/dev/null' EXIT
echo "coptersim running (log: $LOG_DIR/coptersim.log). Starting PX4 ..."
echo "Open QGroundControl — it connects automatically on UDP 14550."

cd "$PX4_BUILD/rootfs" && PX4_SIM_MODEL=none_iris ../bin/px4 ../etc
