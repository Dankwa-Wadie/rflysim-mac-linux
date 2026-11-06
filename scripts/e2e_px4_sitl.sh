#!/usr/bin/env bash
# End-to-end test: coptersim + PX4 SITL (external MAVLink simulator, airframe none_iris).
# Takeoff → hover check → land → disarm check. Exit code 0 on success.
#
# Usage: scripts/e2e_px4_sitl.sh [path/to/PX4-Autopilot]
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PX4_DIR="${1:-${PX4_DIR:-$HOME/tools/PX4-Autopilot}}"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
SIM="$ROOT/target/release/coptersim"
OUT="${E2E_OUT:-$(mktemp -d)}"
SIM_LOG="$OUT/coptersim.log"
PX4_LOG="$OUT/px4.log"

[ -x "$SIM" ] || { echo "build first: cargo build --release"; exit 2; }
[ -x "$PX4_BUILD/bin/px4" ] || { echo "PX4 SITL not built at $PX4_BUILD"; exit 2; }

SIM_PID=""; PX4_PID=""
cleanup() {
  [ -n "$PX4_PID" ] && kill "$PX4_PID" 2>/dev/null
  [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT

fail() { echo "FAIL: $*"; echo "logs: $OUT"; exit 1; }
pass() { echo "PASS: $*"; }

# Last simulated NED z (m, negative = up) from coptersim's 1 Hz status line.
last_z() { grep -o 'NED=([^)]*)' "$SIM_LOG" | tail -1 | tr -d 'NED=() ' | cut -d, -f3; }
last_rp() { grep -o 'rpy=([^)]*)' "$SIM_LOG" | tail -1 | tr -d 'rpy=() ' | cut -d, -f1,2; }
cmd() { (cd "$PX4_BUILD" && ./bin/px4-commander "$@") >>"$OUT/commands.log" 2>&1; }
wait_for() { # pattern file timeout_s
  local i=0
  until grep -q "$1" "$2" 2>/dev/null; do
    i=$((i + 1)); [ "$i" -gt "$3" ] && return 1; sleep 1
  done
}

echo "logs: $OUT"
"$SIM" --db "$ROOT/assets/ModelData.db" >"$SIM_LOG" 2>&1 & SIM_PID=$!
wait_for "awaiting PX4" "$SIM_LOG" 5 || fail "coptersim did not start"

# Fresh parameters each run so results are repeatable.
rm -rf "$PX4_BUILD/rootfs/eeprom" "$PX4_BUILD/rootfs/parameters"*.bson 2>/dev/null
(cd "$PX4_BUILD/rootfs" && PX4_SIM_MODEL=none_iris HEADLESS=1 exec ../bin/px4 -d ../etc >"$PX4_LOG" 2>&1) & PX4_PID=$!

wait_for "link open" "$SIM_LOG" 30 || fail "PX4 never connected to coptersim"
pass "PX4 connected to coptersim"

wait_for "Ready for takeoff" "$PX4_LOG" 90 || fail "PX4 never became ready for takeoff (EKF/GPS)"
pass "PX4 EKF + GPS ready"

cmd takeoff
wait_for "PX4 armed" "$SIM_LOG" 15 || fail "vehicle did not arm"
pass "armed"

sleep 15
z=$(last_z); rp=$(last_rp)
echo "  hover: z=$z m  roll,pitch=$rp deg"
awk -v z="$z" 'BEGIN{exit !(z < -1.5 && z > -4.0)}' || fail "not hovering near takeoff altitude (z=$z)"
awk -v rp="$rp" 'BEGIN{split(rp,a,","); exit !(a[1]<5 && a[1]>-5 && a[2]<5 && a[2]>-5)}' \
  || fail "attitude not level in hover ($rp)"
pass "takeoff + stable hover"

cmd land
wait_for "PX4 disarmed" "$SIM_LOG" 40 || fail "did not land and disarm"
z=$(last_z)
awk -v z="$z" 'BEGIN{exit !(z > -0.05)}' || fail "disarmed but not on ground (z=$z)"
pass "landed + auto-disarmed"

echo "ALL E2E CHECKS PASSED"
