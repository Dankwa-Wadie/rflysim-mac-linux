#!/usr/bin/env bash
# APMSITLRun for macOS and Linux: ArduPilot software-in-the-loop with RflyTools (Windows RflySim's
# APMSITLRun / APMSITLRun_Plane, a Full-edition feature there). One coptersim per vehicle serves
# ArduPilot's JSON physics interface (UDP 9002 + 10 × instance); ArduPilot SITL (arducopter / arduplane,
# built from source) flies it; RflySim3D shows it. QGroundControl gets MAVLink on UDP 14550.
#
# Usage: scripts/APMSITLRun.sh [N] [options]       (no N in a terminal: asks, as SITLRun does)
#   --plane               ArduPlane with the fixed-wing model (APMSITLRun_Plane)
#   --frame F             copter frame: quad_x (default), quad_plus, hexa_x
#   --ardupilot DIR       ArduPilot checkout (default $ARDUPILOT_DIR or ~/tools/ardupilot), built with
#                         ./waf configure --board sitl && ./waf copter plane
#   --start-index I, --interval M, --origin X Y, --yaw DEG, --map NAME, --no-3d, --low-gpu, --qgc
#   --home LAT,LON,ALT    home location (default RflySim's 47.397742,8.545594,488)
#   MAVLink: instance k: QGC on UDP 14550 (all vehicles, SYSID_THISMAV = CopterID), and TCP 5760 + 10k.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/scripts/platform.sh"
APM_DIR="${ARDUPILOT_DIR:-$HOME/tools/ardupilot}"
N=""; START=1; SPACING=2; OX=0; OY=0; YAW=0; MAP="Grasslands"; NO3D=0; LOWGPU=0; QGC=0; PLANE=0; FRAME=""
HOME_LOC="47.397742,8.545594,488"
while [ $# -gt 0 ]; do
  case "$1" in
    --plane) PLANE=1 ;;
    --frame) FRAME="$2"; shift ;;
    --ardupilot) APM_DIR="$2"; shift ;;
    --start-index) START="$2"; shift ;;
    --interval|--spacing) SPACING="$2"; shift ;;
    --origin) OX="$2"; OY="$3"; shift 2 ;;
    --yaw) YAW="$2"; shift ;;
    --map) MAP="$2"; shift ;;
    --home) HOME_LOC="$2"; shift ;;
    --no-3d) NO3D=1 ;;
    --low-gpu) LOWGPU=1; MAP="LowGPU" ;;
    --qgc) QGC=1 ;;
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
    -*) echo "APMSITLRun: unknown option $1"; exit 1 ;;
    *) N="$1" ;;
  esac
  shift
done
if [ -z "$N" ]; then
  if [ -t 0 ]; then read -rp "Please input UAV swarm number:" N; N="${N:-1}"; else N=1; fi
fi
case "$N" in ''|*[!0-9]*) echo "APMSITLRun: the vehicle count must be a number"; exit 1 ;; esac
[ "$N" -ge 1 ] || N=1

if [ "$PLANE" = 1 ]; then VEH=arduplane; PARM=plane.parm; else VEH=arducopter; PARM=copter.parm; fi
APM_BIN="$APM_DIR/build/sitl/bin/$VEH"
SIM="$(rfly_bin coptersim)"; VIEW="$(rfly_bin rflysim3d)"
LOG="$(rfly_logs apm)"; rfly_set_dbarg
rfly_need_bins coptersim rflysim3d || exit 1
[ -x "$APM_BIN" ] || {
  echo "ArduPilot SITL not found at $APM_BIN."
  echo "Build it once: git clone --recurse-submodules https://github.com/ArduPilot/ardupilot $APM_DIR"
  echo "  cd $APM_DIR && Tools/environment_install/install-prereqs-*.sh (or pip install empy==3.3.4 pexpect future)"
  echo "  ./waf configure --board sitl && ./waf copter plane"
  exit 1
}
if pgrep -f "build/sitl/bin/ardu" >/dev/null || pgrep -x coptersim >/dev/null; then
  echo "APMSITLRun: ArduPilot or coptersim is already running. Stop it first (scripts/CloseALL.sh)."
  exit 1
fi
SIMF=(); [ "$PLANE" = 1 ] && SIMF+=(--fixed-wing); [ -n "$FRAME" ] && SIMF+=(--frame "$FRAME")
FRAME_PARAMS="$("$SIM" --print-ardupilot-params ${SIMF[@]+"${SIMF[@]}"} ${DBARG[@]+"${DBARG[@]}"})" || exit 1

PIDS=()
cleanup() {
  trap - EXIT INT TERM
  echo "APMSITLRun: stopping ..."
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  sleep 0.5
  for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null; done
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

if [ "$NO3D" = 0 ]; then
  VARGS=(--map "$MAP"); [ "$LOWGPU" = 1 ] && VARGS+=(--low-gpu)
  "$VIEW" "${VARGS[@]}" >"$LOG/rflysim3d.log" 2>&1 &
  VIEW_PID=$!; PIDS+=("$VIEW_PID")
fi

COLS=$(awk -v n="$N" 'BEGIN { c = int(sqrt(n)); if (c * c < n) c++; print (c < 1 ? 1 : c) }')
IFS=, read -r HLAT HLON HALT <<<"$HOME_LOC"
for k in $(seq 0 $((N - 1))); do
  i=$((START + k)); inst=$((i - 1))
  X=$(awk -v k="$k" -v c="$COLS" -v s="$SPACING" -v o="$OX" 'BEGIN { print o + int(k / c) * s }')
  Y=$(awk -v k="$k" -v c="$COLS" -v s="$SPACING" -v o="$OY" 'BEGIN { print o + (k % c) * s }')
  # ArduPilot puts each vehicle at its own home, so the offset goes into the home location (and the
  # physics starts at the origin of that home).
  VHOME=$(awk -v la="$HLAT" -v lo="$HLON" -v x="$X" -v y="$Y" 'BEGIN { pi = atan2(0, -1);
    printf "%.7f,%.7f", la + x / 6378137 * 180 / pi, lo + y / (6378137 * cos(la * pi / 180)) * 180 / pi }')
  "$SIM" --id "$i" --ardupilot $((9002 + 10 * inst)) ${SIMF[@]+"${SIMF[@]}"} ${DBARG[@]+"${DBARG[@]}"} \
      --lat "${VHOME%,*}" --lon "${VHOME#*,}" --alt "$HALT" --init-yaw="$YAW" \
      ${SIM_ARGS:-} >"$LOG/coptersim_$i.log" 2>&1 &
  PIDS+=("$!")
  WD="$LOG/instance_$inst"; mkdir -p "$WD"
  { echo "SYSID_THISMAV $i"; echo "$FRAME_PARAMS"; } >"$WD/rflytools.parm"
  # -w: wipe the stored parameters each run (defaults + ours), as SITLRun resets PX4's.
  (cd "$WD" && exec "$APM_BIN" -w --model "JSON:127.0.0.1" --speedup 1 -I "$inst" \
      --home "$VHOME,$HALT,$YAW" --defaults "$APM_DIR/Tools/autotest/default_params/$PARM,$WD/rflytools.parm" \
      --serial0 "tcp:$((0))" --serial1 "udpclient:127.0.0.1:14550" ${APM_ARGS:-}) >"$LOG/ardupilot_$i.log" 2>&1 &
  PIDS+=("$!")
  echo "APMSITLRun: vehicle #$i ($VEH, instance $inst): JSON UDP $((9002 + 10 * inst)), MAVLink TCP $((5760 + 10 * inst)) and UDP 14550"
done

[ "$QGC" = 1 ] && launch_qgc
echo "APMSITLRun: $N vehicle(s) running. Logs: $LOG."
if [ -t 0 ]; then
  echo "Press Enter to stop the simulation."
  while true; do
    if read -r -t 1 _; then break; fi
    if [ "$NO3D" = 0 ] && ! kill -0 "$VIEW_PID" 2>/dev/null; then break; fi
  done
elif [ "$NO3D" = 0 ]; then
  wait "$VIEW_PID"
else
  while true; do sleep 3600 & wait $!; done
fi
