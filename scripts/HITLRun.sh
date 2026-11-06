#!/usr/bin/env bash
# HITLRun for macOS and Linux: the RflyTools "HITLRun" shortcut (Windows HITLRun.bat and its variants).
# One coptersim per Pixhawk on USB (in a HIL airframe, e.g. SYS_AUTOSTART 1001 "HIL Quadcopter X"), plus the
# RflySim3D viewer. coptersim relays MAVLink to QGroundControl (UDP 14550) and to the RflySim Python ports
# (20100/20101 + 2(i-1)), as Windows CopterSim does. Turn off QGC's own USB auto-connect so the port stays free
# (QGC: Application Settings → AutoConnect: untick Pixhawk/USB).
#
# Usage: scripts/HITLRun.sh [options] [PORT ...]
#   No PORT in a terminal: shows the serial ports (scripts/ComList.sh) and asks
#   "My port list for HITL simulation is:" (list numbers like 1,2 or paths; Enter = recommended ports).
#   --auto                use all recommended ports without asking (Windows HITLRunAuto)
#   --baud 921600         serial speed (BaudRate)
#   --start-index I       first CopterID (START_INDEX); vehicles follow the port order
#   --sysid               CopterID = each Pixhawk's MAV_SYS_ID (HITLRunSysID)
#   --origin X Y, --yaw DEG, --interval M, --pos-x/--pos-y/--yaw-list LISTS   start positions (as SITLRun)
#   --fixed-wing          fixed-wing aircraft (HITLRunFw). The Pixhawk needs a fixed-wing airframe with
#                         SYS_HITL 1 and the output mapping printed by: coptersim --print-px4-params --fixed-wing
#   --map NAME, --no-3d, --low-gpu, --qgc, --params-file FILE, --status-port P, --udp-mode M   (as SITLRun)
#   Linux: your user must be in the serial group (sudo usermod -aG dialout $USER, then log in again)
#   and ModemManager must not grab the port (sudo systemctl stop ModemManager).
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/scripts/platform.sh"
MAP="Grasslands"; BAUD=921600; PORTS=(); AUTO=0; START=1; SYSID=0; NO3D=0; LOWGPU=0; QGC=0
SPACING=2; OX=0; OY=0; YAW=0; PX=""; PY=""; PYAW=""; PARAMS=""; STATUS=""; UDPMODE=2; FW=0
while [ $# -gt 0 ]; do
  case "$1" in
    --map) MAP="$2"; shift ;;
    --baud) BAUD="$2"; shift ;;
    --auto) AUTO=1 ;;
    --start-index) START="$2"; shift ;;
    --sysid) SYSID=1 ;;
    --no-3d) NO3D=1 ;;
    --low-gpu) LOWGPU=1 ;;
    --qgc) QGC=1 ;;
    --fixed-wing) FW=1 ;;
    --spacing|--interval) SPACING="$2"; shift ;;
    --origin) OX="$2"; OY="$3"; shift 2 ;;
    --yaw) YAW="$2"; shift ;;
    --pos-x) PX="$2"; shift ;;
    --pos-y) PY="$2"; shift ;;
    --yaw-list) PYAW="$2"; shift ;;
    --params-file) PARAMS="$2"; shift ;;
    --status-port) STATUS="$2"; shift ;;
    --udp-mode) UDPMODE="$2"; shift ;;
    -h|--help) sed -n 2,21p "$0"; exit 0 ;;
    -*) echo "HITLRun: unknown option $1 (see --help)"; exit 1 ;;
    *) PORTS+=("$1") ;;
  esac
  shift
done

if [ ${#PORTS[@]} -eq 0 ]; then
  ALL=(); while IFS= read -r l; do [ -n "$l" ] && ALL+=("$l"); done < <("$ROOT/scripts/ComList.sh" | awk '/^  [0-9]+  /{print $2}')
  REC="$("$ROOT/scripts/ComList.sh" --recommended)"
  [ ${#ALL[@]} -gt 0 ] || { echo "Warning: there is no available serial port. Plug in the Pixhawk (USB) and try again."; exit 1; }
  if [ "$AUTO" = 1 ] || [ ! -t 0 ]; then
    [ -n "$REC" ] || { echo "No flight controller recognised; pass the port(s), e.g. scripts/HITLRun.sh ${ALL[0]}"; exit 1; }
    IFS=, read -ra PORTS <<<"$REC"
  else
    "$ROOT/scripts/ComList.sh"
    while [ ${#PORTS[@]} -eq 0 ]; do
      read -rp "My port list for HITL simulation is: " IN
      IN="${IN:-$REC}"
      IFS=, read -ra ITEMS <<<"$IN"
      for it in "${ITEMS[@]}"; do
        it="$(echo "$it" | tr -d ' ')"
        case "$it" in
          '') ;;
          *[!0-9]*) [ -e "$it" ] && PORTS+=("$it") || { echo "Error: $it is not a serial port"; PORTS=(); break; } ;;
          *) [ "$it" -ge 1 ] && [ "$it" -le ${#ALL[@]} ] && PORTS+=("${ALL[$((it - 1))]}") || { echo "Error: Input '$it' is not in the list"; PORTS=(); break; } ;;
        esac
      done
    done
  fi
fi
for p in "${PORTS[@]}"; do
  [ -e "$p" ] || { echo "No such port: $p"; exit 1; }
  if [ ! -r "$p" ] || [ ! -w "$p" ]; then
    echo "No permission for $p. Linux: sudo usermod -aG dialout $USER, then log out and in."; exit 1
  fi
done
[ -z "$PARAMS" ] || [ -f "$PARAMS" ] || { echo "HITLRun: parameter file not found: $PARAMS"; exit 1; }
SIM="$(rfly_bin coptersim)"; VIEW="$(rfly_bin rflysim3d)"
LOG="$(rfly_logs hitl)"; rfly_set_dbarg
rfly_need_bins coptersim rflysim3d || exit 1
rfly_update_notice

PIDS=()
cleanup() {
  trap - EXIT INT TERM; echo "HITLRun: stopping ..."
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  sleep 1; for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null; done
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM
if [ "$NO3D" = 0 ]; then
  VARGS=(--map "$MAP"); [ "$LOWGPU" = 1 ] && VARGS+=(--low-gpu)
  "$VIEW" "${VARGS[@]}" >"$LOG/rflysim3d.log" 2>&1 &
  VIEW_PID=$!; PIDS+=("$VIEW_PID")
fi
nth() { echo "$1" | cut -d, -f"$2" -s 2>/dev/null; }
N=${#PORTS[@]}
COLS=$(awk -v n="$N" 'BEGIN { c = int(sqrt(n)); if (c * c < n) c++; print (c < 1 ? 1 : c) }')
k=0
for p in "${PORTS[@]}"; do
  i=$((START + k))
  if [ "$SYSID" = 1 ]; then
    id=$("$ROOT/scripts/Python38Env.sh" "$ROOT/python/tools/mav_sysid.py" "$p" "$BAUD" 2>/dev/null | tail -1)
    case "$id" in ''|*[!0-9]*) echo "HITLRun: no heartbeat from $p; using CopterID $i" ;; *) i=$id ;; esac
  fi
  X=$(awk -v k="$k" -v c="$COLS" -v s="$SPACING" -v o="$OX" 'BEGIN { print o + int(k / c) * s }')
  Y=$(awk -v k="$k" -v c="$COLS" -v s="$SPACING" -v o="$OY" 'BEGIN { print o + (k % c) * s }')
  VYAW="$YAW"
  [ -n "$PX" ] && [ -n "$(nth "$PX" $((k + 1)))" ] && X="$(nth "$PX" $((k + 1)))"
  [ -n "$PY" ] && [ -n "$(nth "$PY" $((k + 1)))" ] && Y="$(nth "$PY" $((k + 1)))"
  [ -n "$PYAW" ] && [ -n "$(nth "$PYAW" $((k + 1)))" ] && VYAW="$(nth "$PYAW" $((k + 1)))"
  EXTRA=(--udp-mode "$UDPMODE"); [ -n "$PARAMS" ] && EXTRA+=(--params-file "$PARAMS"); [ -n "$STATUS" ] && EXTRA+=(--status-port $((STATUS + k)))
  [ "$FW" = 1 ] && EXTRA+=(--fixed-wing --alt=0)   # home at 0 m: see SITLRun (PX4 TECS boot reference)
  "$SIM" --id "$i" --link "serial:$p:$BAUD" ${DBARG[@]+"${DBARG[@]}"} --init-x="$X" --init-y="$Y" --init-yaw="$VYAW" \
      ${EXTRA[@]+"${EXTRA[@]}"} >"$LOG/coptersim_$i.log" 2>&1 &
  PIDS+=("$!")
  echo "HITLRun: vehicle #$i on $p at x=$X y=$Y (log $LOG/coptersim_$i.log)"
  k=$((k + 1))
done
# Like Windows: QGroundControl starts after CopterSim has the serial port, so QGC cannot take it.
[ "$QGC" = 1 ] && { sleep 2; launch_qgc; }
echo "HITLRun: $N vehicle(s) running."
if [ -t 0 ]; then
  echo "Press Enter to stop the simulation."
  while true; do
    if read -r -t 1 _; then break; fi
    if [ "$NO3D" = 0 ] && ! kill -0 "$VIEW_PID" 2>/dev/null; then break; fi
  done
elif [ "$NO3D" = 0 ]; then
  wait "$VIEW_PID"
else
  while true; do sleep 3600 & wait $!; done  # wait (not sleep) lets the stop signal through at once
fi
