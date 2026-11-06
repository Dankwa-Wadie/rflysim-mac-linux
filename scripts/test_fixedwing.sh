#!/usr/bin/env bash
# Fixed-wing SITL check: SITLRun --fixed-wing, arm, take off, then verify the plane climbs and
# keeps flying (PX4 loiters around the takeoff point at ~30 m).  Prints PASS/FAIL per check.
#   scripts/test_fixedwing.sh [flight seconds, default 60]
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; ROOT="$(cd "$HERE/.." && pwd)"
. "$HERE/platform.sh"
FLY="${1:-60}"
PX4_DIR="${PX4_DIR:-$HOME/tools/PX4-Autopilot}"; PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
LOG="$(rfly_logs sitl)"
CL="$LOG/coptersim_1.log"
cmd() { (cd "$PX4_BUILD" && ./bin/px4-commander --instance 0 "$@") >/dev/null 2>&1; }

bash "$HERE/SITLRun.sh" 1 --no-3d --fixed-wing </dev/null >"$LOG/test_fixedwing.log" 2>&1 &
RUN=$!; disown "$RUN"
cleanup() { pkill -P "$RUN" 2>/dev/null; kill "$RUN" 2>/dev/null; bash "$HERE/CloseALL.sh" >/dev/null 2>&1; }
trap cleanup EXIT

fail=0
check() { if [ "$2" = 1 ]; then echo "PASS $1"; else echo "FAIL $1"; fail=1; fi; }

# Wait for PX4's "Ready for takeoff", give the EKF ~15 s to settle its GPS origin (a takeoff sent
# while it is still resetting gets a takeoff waypoint at lat/lon 0,0 and never ends), arm until the
# log says so (px4-commander's exit code does not), then one takeoff.
PL="$LOG/px4_1.log"
for _ in $(seq 60); do sleep 1; grep -q 'Ready for takeoff' "$PL" 2>/dev/null && break; done
sleep 15
for _ in $(seq 20); do cmd arm; sleep 1; grep -q 'Armed by' "$PL" && break; done
cmd takeoff
up=0
for _ in $(seq 20); do
  sleep 1
  z=$(grep -E '^t=' "$CL" | tail -1 | sed -E 's/.*NED=\( *[-0-9.]+, *[-0-9.]+, *([-0-9.]+)\).*/\1/')
  awk -v z="${z:-0}" 'BEGIN{exit !(z < -2)}' && { up=1; break; }
done
check "armed and took off" "$up"
[ "$up" = 1 ] || exit 1
sleep "$FLY"

last=$(grep -E '^t=' "$CL" | tail -1)
z=$(sed -E 's/.*NED=\( *[-0-9.]+, *[-0-9.]+, *([-0-9.]+)\).*/\1/' <<<"$last")
as=$(grep -E 'fw: airspeed' "$CL" | tail -1 | awk '{print $3}')
minz=$(grep -E '^t=' "$CL" | tail -n "$((FLY / 2))" | sed -E 's/.*NED=\( *[-0-9.]+, *[-0-9.]+, *([-0-9.]+)\).*/\1/' | sort -g | tail -1)
echo "altitude $(awk -v z="$z" 'BEGIN{printf "%.1f", -z}') m, airspeed $as m/s, lowest in last $((FLY / 2)) s: $(awk -v z="$minz" 'BEGIN{printf "%.1f", -z}') m"
check "climbed above 20 m" "$(awk -v z="$z" 'BEGIN{print (z < -20) ? 1 : 0}')"
check "held altitude (never below 15 m late in the flight)" "$(awk -v z="$minz" 'BEGIN{print (z < -15) ? 1 : 0}')"
check "flying speed (airspeed 10-25 m/s)" "$(awk -v a="${as:-0}" 'BEGIN{print (a > 10 && a < 25) ? 1 : 0}')"
# PX4 should leave takeoff mode for a loiter once at the takeoff altitude.
st=$( (cd "$PX4_BUILD" && ./bin/px4-listener --instance 0 vehicle_status -n 1) 2>/dev/null | awk '/ nav_state:/{print $2; exit}')
check "takeoff finished, loitering (nav_state $st)" "$([ "$st" = 4 ] && echo 1 || echo 0)"
exit "$fail"
