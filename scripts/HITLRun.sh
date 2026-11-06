#!/usr/bin/env bash
# HITLRun for macOS and Linux: the RflyTools "HITLRun" shortcut.
# One coptersim per Pixhawk on USB (in HIL airframe, e.g. SYS_AUTOSTART 1001 "HIL Quadcopter X"),
# plus the RflySim3D viewer. Coptersim relays MAVLink to QGroundControl (UDP 14550) and
# to the RflySim Python ports (20100/20101 + 2(i-1)), as Windows CopterSim does. Close QGC's
# own serial auto-connect so the port is free (QGC: Application Settings → AutoConnect: untick USB/Pixhawk).
#
# Usage: scripts/HITLRun.sh [--map NAME] [--baud 921600] [PORT ...]
#   PORT defaults to every /dev/cu.usbmodem* (macOS) or /dev/ttyACM* (Linux).
#   Linux: your user must be in the serial group (sudo usermod -aG dialout $USER, then log in again)
#   and ModemManager must not grab the port (sudo systemctl stop ModemManager).
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/scripts/platform.sh"
MAP="Grasslands"; BAUD=921600; PORTS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --map) MAP="$2"; shift ;;
    --baud) BAUD="$2"; shift ;;
    -h|--help) sed -n 2,12p "$0"; exit 0 ;;
    *) PORTS+=("$1") ;;
  esac
  shift
done
# shellcheck disable=SC2206 # the glob is meant to expand
[ ${#PORTS[@]} -eq 0 ] && PORTS=($(default_serial_glob))
[ -e "${PORTS[0]}" ] || { echo "No Pixhawk found ($(default_serial_glob)). Plug it in or pass the port."; exit 1; }
if [ ! -r "${PORTS[0]}" ] || [ ! -w "${PORTS[0]}" ]; then
  echo "No permission for ${PORTS[0]}. Linux: sudo usermod -aG dialout $USER, then log out and in."; exit 1
fi
SIM="$(rfly_bin coptersim)"; VIEW="$(rfly_bin rflysim3d)"
LOG="$(rfly_logs hitl)"; rfly_set_dbarg
rfly_need_bins coptersim rflysim3d || exit 1
rfly_update_notice

PIDS=()
trap 'for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; wait 2>/dev/null' EXIT INT TERM
"$VIEW" --map "$MAP" >"$LOG/rflysim3d.log" 2>&1 &
VIEW_PID=$!; PIDS+=("$VIEW_PID")
i=1
for p in "${PORTS[@]}"; do
  "$SIM" --id "$i" --link "serial:$p:$BAUD" ${DBARG[@]+"${DBARG[@]}"} --init-y $(( (i - 1) * 2 )) \
      >"$LOG/coptersim_$i.log" 2>&1 &
  PIDS+=("$!")
  echo "HITLRun: vehicle #$i on $p (log $LOG/coptersim_$i.log)"
  i=$((i + 1))
done
wait "$VIEW_PID"
