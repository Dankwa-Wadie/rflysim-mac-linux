#!/usr/bin/env bash
# SITLRun for macOS and Linux: the RflyTools "SITLRun" shortcut (Windows SITLRun.bat and its variants).
# Starts N × (coptersim + PX4 SITL), the RflySim3D viewer and optionally QGroundControl.
#
# Usage: scripts/SITLRun.sh [N] [options]        (no N in a terminal: asks "Please input UAV swarm number:")
#   Windows variable      option here
#   VehicleNum            N
#   START_INDEX=1         --start-index I          first CopterID (ports follow the ID)
#   MAX_VEHICLE=50        --max M
#   VEHICLE_INTERVAL=2    --interval M             grid spacing (m); alias --spacing
#   ORIGIN_POS_X/Y=0      --origin X Y             grid origin (m, NED)
#   ORIGIN_YAW=0          --yaw DEG                heading of every vehicle
#   PosXStr/PosYStr/YawStr  --pos-x 1,2,3 --pos-y 0,0,0 --yaw-list 0,90,0   explicit positions (SITLPosStr)
#   UE4_MAP=Grasslands    --map NAME               RflySim3D scene
#   (SITLRunNoUI)         --no-3d                  no viewer (headless)
#   (SITLRunLowGPU)       --low-gpu                lighter viewer (smaller window, no shadows/trails)
#                         --qgc                    also start QGroundControl
#                         --fg                     also show vehicle #1 in FlightGear (F450 model)
#   UDPSIMMODE=2          --udp-mode M             Python link: 0 UDP_Full, 1 UDP_Simple, 2 Mavlink_Full (default),
#                                                  3 Mavlink_Simple, 4 Mavlink_NoSend, 5 Mavlink_NoGPS, 6 Mavlink_Vision
#   IS_BROADCAST=0        --broadcast X            RflySim3D on other computers: 1 = whole LAN (SITLRunLANSwarm),
#                                                  or IP / IP1,IP2 (vehicles still show on this computer)
#   CLASS_3D_ID           --vehicle-type T         3D model style in RflySim3D (SITLRunChange3D): 3 quad, 5 hexa,
#                                                  8 octo, 100 plane, 30 car, 150 box
#   (SITLPos)             --ask-pos                ask for the PosX / PosY / yaw lists when starting
#   (frame type)          --frame F                quad_x (default), quad_plus, hexa_x, octo_x, y6 (or set in --params-file)
#                         --params-file FILE       vehicle parameters (from the CopterSim window)
#                         --status-port P          10 Hz status of each vehicle to UDP P+i-1 (CopterSim window)
#                         --px4 DIR                PX4-Autopilot checkout (default ~/tools/PX4-Autopilot)
#
# Ports per vehicle (CopterID i), the same as RflySim on Windows:
#   PX4 ⇄ coptersim TCP 4560+i-1 · Python → PX4 UDP 20100+2(i-1) · PX4 → Python UDP 20101+2(i-1)
#   true state → Python UDP 30101+2(i-1) · vehicles → RflySim3D UDP 20010.. · QGC UDP 14550
# Stop: press Enter (in a terminal), close the RflySim3D window, or Ctrl-C. scripts/CloseALL.sh stops everything.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/scripts/platform.sh"
N=""; MAP="Grasslands"; NO3D=0; LOWGPU=0; QGC=0; FG=0; SPACING=2; START=1; MAX=50
OX=0; OY=0; YAW=0; PX=""; PY=""; PYAW=""; PARAMS=""; STATUS=""; UDPMODE=2; BCAST=""; VTYPE=""; ASKPOS=0; FRAME=""
PX4_DIR="${PX4_DIR:-$HOME/tools/PX4-Autopilot}"
while [ $# -gt 0 ]; do
  case "$1" in
    --map) MAP="$2"; shift ;;
    --no-3d) NO3D=1 ;;
    --low-gpu) LOWGPU=1 ;;
    --qgc) QGC=1 ;;
    --fg) FG=1 ;;
    --spacing|--interval) SPACING="$2"; shift ;;
    --start-index) START="$2"; shift ;;
    --max) MAX="$2"; shift ;;
    --origin) OX="$2"; OY="$3"; shift 2 ;;
    --yaw) YAW="$2"; shift ;;
    --pos-x) PX="$2"; shift ;;
    --pos-y) PY="$2"; shift ;;
    --yaw-list) PYAW="$2"; shift ;;
    --params-file) PARAMS="$2"; shift ;;
    --status-port) STATUS="$2"; shift ;;
    --udp-mode) UDPMODE="$2"; shift ;;
    --broadcast) BCAST="$2"; shift ;;
    --vehicle-type) VTYPE="$2"; shift ;;
    --ask-pos) ASKPOS=1 ;;
    --frame) FRAME="$2"; shift ;;
    --px4) PX4_DIR="$2"; shift ;;
    -h|--help) sed -n 2,36p "$0"; exit 0 ;;
    -*) echo "SITLRun: unknown option $1 (see --help)"; exit 1 ;;
    *) N="$1" ;;
  esac
  shift
done
is_uint() { case "$1" in ''|*[!0-9]*) return 1 ;; esac; return 0; }
# Like SITLRun.bat: ask for the number of vehicles and re-ask until it is a positive integer.
while ! is_uint "$N" || [ "$N" -lt 1 ]; do
  [ -t 0 ] || { echo "SITLRun: give the number of vehicles (e.g. scripts/SITLRun.sh 2)"; exit 1; }
  [ -n "$N" ] && echo "Not a positive integer"
  read -rp "Please input UAV swarm number: " N
done
# SITLPos: type the start positions (comma-separated, one value per vehicle; empty = grid).
if [ "$ASKPOS" = 1 ] && [ -t 0 ]; then
  read -rp "PosX list (m, e.g. 0,2,4): " PX; read -rp "PosY list (m): " PY; read -rp "Yaw list (deg): " PYAW
fi
if [ "$N" -gt "$MAX" ]; then echo "The vehicle number is too large, which may cause a crash (max $MAX; --max to change)"; exit 1; fi
is_uint "$START" && [ "$START" -ge 1 ] || { echo "SITLRun: --start-index must be a positive integer"; exit 1; }
[ -z "$PARAMS" ] || [ -f "$PARAMS" ] || { echo "SITLRun: parameter file not found: $PARAMS"; exit 1; }
case "$UDPMODE" in
  0|[Uu][Dd][Pp]_[Ff]ull) UDPMODE=0 ;; 1|[Uu][Dd][Pp]_[Ss]imple) UDPMODE=1 ;; 2|[Mm]avlink_[Ff]ull) UDPMODE=2 ;;
  3|[Mm]avlink_[Ss]imple) UDPMODE=3 ;; 4|[Mm]avlink_[Nn]o[Ss]end) UDPMODE=4 ;; 5|[Mm]avlink_[Nn]o[Gg][Pp][Ss]) UDPMODE=5 ;;
  6|[Mm]avlink_[Vv]ision) UDPMODE=6 ;; *) echo "SITLRun: unknown --udp-mode $UDPMODE"; exit 1 ;;
esac

PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
SIM="$(rfly_bin coptersim)"; VIEW="$(rfly_bin rflysim3d)"
LOG="$(rfly_logs sitl)"; rfly_set_dbarg
rfly_need_bins coptersim rflysim3d || exit 1
rfly_update_notice
[ -x "$PX4_BUILD/bin/px4" ] || { echo "PX4 SITL not found at $PX4_BUILD. Build PX4 once (see the README: \"PX4 SITL\"), or pass --px4 DIR."; exit 1; }
if pgrep -f "bin/px4( |$)" >/dev/null || pgrep -x coptersim >/dev/null; then
  echo "SITLRun: PX4 or coptersim is already running. Stop it first (scripts/CloseALL.sh)."
  exit 1
fi

PIDS=()
cleanup() {
  trap - EXIT INT TERM
  echo "SITLRun: stopping ..."
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  pkill -f "$PX4_BUILD/bin/px4 -i" 2>/dev/null
  # PX4 can miss SIGTERM while it is still starting; force anything left after 3 s.
  for _ in 1 2 3 4 5 6; do
    left=0; for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && left=1; done
    [ $left = 0 ] && break; sleep 0.5
  done
  for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null; done
  pkill -9 -f "$PX4_BUILD/bin/px4 -i" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

if [ "$NO3D" = 0 ]; then
  VARGS=(--map "$MAP"); [ "$LOWGPU" = 1 ] && VARGS+=(--low-gpu)
  "$VIEW" "${VARGS[@]}" >"$LOG/rflysim3d.log" 2>&1 &
  VIEW_PID=$!; PIDS+=("$VIEW_PID")
fi

# nth LIST k → k-th (1-based) comma-separated item, or empty
nth() { echo "$1" | cut -d, -f"$2" -s 2>/dev/null; }
COLS=$(awk -v n="$N" 'BEGIN { c = int(sqrt(n)); if (c * c < n) c++; print (c < 1 ? 1 : c) }')
for k in $(seq 0 $((N - 1))); do
  i=$((START + k)); inst=$((i - 1))
  X=$(awk -v k="$k" -v c="$COLS" -v s="$SPACING" -v o="$OX" 'BEGIN { print o + int(k / c) * s }')
  Y=$(awk -v k="$k" -v c="$COLS" -v s="$SPACING" -v o="$OY" 'BEGIN { print o + (k % c) * s }')
  VYAW="$YAW"
  [ -n "$PX" ] && [ -n "$(nth "$PX" $((k + 1)))" ] && X="$(nth "$PX" $((k + 1)))"
  [ -n "$PY" ] && [ -n "$(nth "$PY" $((k + 1)))" ] && Y="$(nth "$PY" $((k + 1)))"
  [ -n "$PYAW" ] && [ -n "$(nth "$PYAW" $((k + 1)))" ] && VYAW="$(nth "$PYAW" $((k + 1)))"
  EXTRA=(); [ "$FG" = 1 ] && [ "$k" = 0 ] && EXTRA+=(--fg)
  [ -n "$PARAMS" ] && EXTRA+=(--params-file "$PARAMS")
  [ -n "$STATUS" ] && EXTRA+=(--status-port $((STATUS + k)))
  EXTRA+=(--udp-mode "$UDPMODE")
  [ -n "$VTYPE" ] && EXTRA+=(--vehicle-type "$VTYPE")
  [ -n "$FRAME" ] && EXTRA+=(--frame "$FRAME")
  [ -n "$BCAST" ] && [ "$BCAST" != 0 ] && EXTRA+=(--ue4-ip "127.0.0.1,$BCAST")
  # UDP_Full/UDP_Simple: coptersim serves Python's ports and talks to PX4 over its own MAVLink link.
  [ "$UDPMODE" -le 1 ] && EXTRA+=(--px4-mav "udpin:0.0.0.0:$((25100 + 2 * inst))")
  "$SIM" --id "$i" ${DBARG[@]+"${DBARG[@]}"} --init-x="$X" --init-y="$Y" --init-yaw="$VYAW" ${EXTRA[@]+"${EXTRA[@]}"} ${SIM_ARGS:-} >"$LOG/coptersim_$i.log" 2>&1 &
  PIDS+=("$!")
  mkdir -p "$PX4_BUILD/instance_$inst"
  PX4_SIM_MODEL=none_iris "$PX4_BUILD/bin/px4" -i "$inst" -d -w "$PX4_BUILD/instance_$inst" "$PX4_BUILD/etc" >"$LOG/px4_$i.log" 2>&1 &
  PIDS+=("$!")
  echo "SITLRun: vehicle #$i at x=$X y=$Y yaw=$VYAW"
done

# Once each PX4 is up, open the RflySim Python port pair on it.
for k in $(seq 0 $((N - 1))); do
  i=$((START + k)); inst=$((i - 1)); t=0
  until grep -q "Startup script returned successfully" "$LOG/px4_$i.log" 2>/dev/null; do
    t=$((t + 1)); [ $t -gt 60 ] && { echo "PX4 #$i did not start, see $LOG/px4_$i.log"; exit 1; }; sleep 1
  done
  # Python's MAVLink link, per UDPSIMMODE (PX4 listens on -u, sends to -o).
  case "$UDPMODE" in
    0|1) MAV=(-u $((25101 + 2 * inst)) -o $((25100 + 2 * inst)) -r 4000000 -m onboard) ;;  # to coptersim's bridge
    3) MAV=(-u $((20100 + 2 * inst)) -o $((20101 + 2 * inst)) -r 200000 -m onboard) ;;     # Mavlink_Simple: lower rate
    4) MAV=() ;;                                                                           # Mavlink_NoSend
    *) MAV=(-u $((20100 + 2 * inst)) -o $((20101 + 2 * inst)) -r 4000000 -m onboard) ;;
  esac
  [ ${#MAV[@]} -gt 0 ] && (cd "$PX4_BUILD" && ./bin/px4-mavlink --instance "$inst" start -x -t 127.0.0.1 "${MAV[@]}") >>"$LOG/px4_$i.log" 2>&1
  # Mavlink_NoGPS / Mavlink_Vision: no GPS; Vision fuses coptersim's external-vision odometry instead.
  setp() { (cd "$PX4_BUILD" && ./bin/px4-param --instance "$inst" set "$1" "$2") >>"$LOG/px4_$i.log" 2>&1; }
  # Frame geometry for PX4's control allocation, from coptersim (every run: PX4 keeps parameters).
  FA=(); [ -n "$PARAMS" ] && FA+=(--params-file "$PARAMS"); [ -n "$FRAME" ] && FA+=(--frame "$FRAME")
  "$SIM" --print-px4-params ${FA[@]+"${FA[@]}"} 2>/dev/null | grep -E "^[A-Z][A-Z0-9_]+ " | while read -r pn pv; do setp "$pn" "$pv"; done
  # PX4 keeps parameters per instance, so every mode sets (or resets) all of these.
  (cd "$PX4_BUILD" && ./bin/px4-param --instance "$inst" reset COM_ARM_WO_GPS EKF2_GPS_CTRL EKF2_EV_CTRL EKF2_HGT_REF) >>"$LOG/px4_$i.log" 2>&1
  if [ "$UDPMODE" = 5 ] || [ "$UDPMODE" = 6 ]; then setp COM_ARM_WO_GPS 1; setp EKF2_GPS_CTRL 0; fi
  if [ "$UDPMODE" = 6 ]; then setp EKF2_EV_CTRL 15; setp EKF2_HGT_REF 3; fi
  echo "SITLRun: vehicle #$i ready (PX4 instance $inst, Python port $((20100 + 2 * inst))/$((20101 + 2 * inst)), mode $UDPMODE)"
done

[ "$QGC" = 1 ] && launch_qgc
if [ "$FG" = 1 ]; then
  "$ROOT/scripts/FlightGearF450.sh" ${FG_ARGS:-} >"$LOG/flightgear.log" 2>&1 &
  PIDS+=("$!")
fi
echo "SITLRun: $N vehicle(s) running. Logs: $LOG."
if [ -t 0 ]; then
  echo "Press Enter to stop the simulation."
  # Stop on Enter, or when the viewer window is closed.
  while true; do
    if read -r -t 1 _; then break; fi
    if [ "$NO3D" = 0 ] && ! kill -0 "$VIEW_PID" 2>/dev/null; then break; fi
  done
elif [ "$NO3D" = 0 ]; then
  wait "$VIEW_PID"
else
  while true; do sleep 3600 & wait $!; done  # wait (not sleep) lets the stop signal through at once
fi
