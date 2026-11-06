#!/usr/bin/env bash
# One-command FlightGear-F450 demo: FlightGear at Honolulu (PHNL; FGData 2024 ships Hawaii and Iceland scenery only),
# coptersim feeding it, and a Python controller flying a climb, circles and a landing.
# Usage: scripts/FlightGearDemo.sh [seconds]     (default 120; FlightGear stays open afterwards)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/scripts/platform.sh"
LOGS="$(rfly_logs)"
PY="${RFLY_VENV:-$HOME/tools/rflysim-py}/bin/python"
SIM="$(rfly_bin coptersim)"
rfly_need_bins coptersim || exit 1
if ! udp_listening 5502; then
  echo "Starting FlightGear (first start can take a minute)..."
  "$ROOT/scripts/FlightGearF450.sh" --airport=PHNL >"$LOGS/flightgear.log" 2>&1 &
  for _ in $(seq 1 90); do udp_listening 5502 && break; sleep 2; done
  sleep 8 # let the scenery finish loading
else
  echo "FlightGear is already listening on UDP 5502."
fi
pkill -f "coptersim --ext 20100" 2>/dev/null
"$SIM" --ext 20100 --no-rfly --fg 127.0.0.1:5502 --lat=21.3187 --lon=-157.9224 --alt=4 >"$LOGS/fg_demo_sim.log" 2>&1 &
SIM_PID=$!
trap 'kill $SIM_PID 2>/dev/null' EXIT
sleep 1
echo "Flying (watch the FlightGear window; press v there to change view)..."
"$PY" "$ROOT/python/examples/fg_demo_flight.py" "${1:-120}"
