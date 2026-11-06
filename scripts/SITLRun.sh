#!/usr/bin/env bash
# SITLRun for macOS and Linux: the RflyTools "SITLRun" shortcut.
# Starts N × (coptersim + PX4 SITL), the RflySim3D viewer and optionally QGroundControl.
#
# Ports per vehicle i (1-based), the same as RflySim on Windows:
#   PX4 ⇄ coptersim     TCP 4560+i-1
#   Python → PX4        UDP 20100+2(i-1)   (PX4MavCtrler sends here)
#   PX4 → Python        UDP 20101+2(i-1)   (PX4MavCtrler listens here)
#   true state → Python UDP 30101+2(i-1)
#   vehicles → RflySim3D UDP 20010..20012
#   QGC                 UDP 14550 (PX4 default, all instances)
#
# Usage: scripts/SITLRun.sh [N] [--map NAME] [--no-3d] [--qgc] [--fg] [--spacing M] [--px4 DIR]
#   --fg: also show vehicle #1 in FlightGear (F450 model), see scripts/FlightGearF450.sh
# Ctrl-C (or closing the RflySim3D window) stops everything.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/scripts/platform.sh"
N=1; MAP="Grasslands"; NO3D=0; QGC=0; FG=0; SPACING=2; PX4_DIR="${PX4_DIR:-$HOME/tools/PX4-Autopilot}"
while [ $# -gt 0 ]; do
  case "$1" in
    --map) MAP="$2"; shift ;;
    --no-3d) NO3D=1 ;;
    --qgc) QGC=1 ;;
    --fg) FG=1 ;;
    --spacing) SPACING="$2"; shift ;;
    --px4) PX4_DIR="$2"; shift ;;
    -h|--help) sed -n 2,16p "$0"; exit 0 ;;
    *) N="$1" ;;
  esac
  shift
done
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
SIM="$(rfly_bin coptersim)"; VIEW="$(rfly_bin rflysim3d)"
LOG="$(rfly_logs sitl)"; rfly_set_dbarg

rfly_need_bins coptersim rflysim3d || exit 1
rfly_update_notice
[ -x "$PX4_BUILD/bin/px4" ] || { echo "PX4 SITL not found at $PX4_BUILD. Build PX4 once (see the README: \"PX4 SITL\"), or pass --px4 DIR."; exit 1; }

if pgrep -f "bin/px4( |$)" >/dev/null || pgrep -x coptersim >/dev/null; then
  echo "SITLRun: PX4 or coptersim is already running (pgrep -fl 'px4|coptersim'). Stop it first."
  exit 1
fi

PIDS=()
cleanup() {
  echo "SITLRun: stopping ..."
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  pkill -f "$PX4_BUILD/bin/px4 -i" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

if [ "$NO3D" = 0 ]; then
  "$VIEW" --map "$MAP" >"$LOG/rflysim3d.log" 2>&1 &
  VIEW_PID=$!; PIDS+=("$VIEW_PID")
fi

COLS=$(python3 -c "import math;print(max(1,math.ceil(math.sqrt($N))))")
for i in $(seq 1 "$N"); do
  k=$((i - 1))
  X=$(python3 -c "print(($k // $COLS) * $SPACING)"); Y=$(python3 -c "print(($k % $COLS) * $SPACING)")
  EXTRA=(); [ "$FG" = 1 ] && [ "$i" = 1 ] && EXTRA=(--fg)
  "$SIM" --id "$i" ${DBARG[@]+"${DBARG[@]}"} --init-x "$X" --init-y "$Y" ${EXTRA[@]+"${EXTRA[@]}"} ${SIM_ARGS:-} >"$LOG/coptersim_$i.log" 2>&1 &
  PIDS+=("$!")
  mkdir -p "$PX4_BUILD/instance_$k"
  PX4_SIM_MODEL=none_iris "$PX4_BUILD/bin/px4" -i "$k" -d -w "$PX4_BUILD/instance_$k" "$PX4_BUILD/etc" >"$LOG/px4_$i.log" 2>&1 &
  PIDS+=("$!")
done

# Once each PX4 is up, open the RflySim Python port pair on it.
for i in $(seq 1 "$N"); do
  k=$((i - 1)); t=0
  until grep -q "Startup script returned successfully" "$LOG/px4_$i.log" 2>/dev/null; do
    t=$((t + 1)); [ $t -gt 60 ] && { echo "PX4 #$i did not start, see $LOG/px4_$i.log"; exit 1; }; sleep 1
  done
  (cd "$PX4_BUILD" && ./bin/px4-mavlink --instance "$k" start -x -u $((20100 + 2 * k)) -o $((20101 + 2 * k)) \
      -t 127.0.0.1 -r 4000000 -m onboard) >>"$LOG/px4_$i.log" 2>&1
  echo "SITLRun: vehicle #$i ready (PX4 instance $k, Python port $((20100 + 2 * k))/$((20101 + 2 * k)))"
done

[ "$QGC" = 1 ] && launch_qgc
if [ "$FG" = 1 ]; then
  "$ROOT/scripts/FlightGearF450.sh" ${FG_ARGS:-} >"$LOG/flightgear.log" 2>&1 &
  PIDS+=("$!")
fi
echo "SITLRun: $N vehicle(s) running. Logs: $LOG. Ctrl-C to stop."
if [ "$NO3D" = 0 ]; then
  wait "$VIEW_PID"
else
  while true; do sleep 3600; done
fi
