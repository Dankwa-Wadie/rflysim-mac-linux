#!/usr/bin/env bash
# Full RflyTools test for macOS and Linux (no FlightGear / Pixhawk needed). Prints PASS/FAIL per stage.
# Linux without a desktop: run it under a virtual display, e.g. xvfb-run -a -s '-screen 0 1280x800x24' scripts/test_all.sh
#   A unit tests          B rflysim3d protocol + images     C FlightGear packets (stand-in listener)
#   D SITLRun + Python offboard/camera + MAVLink relay (HIL path, relayed from SITL)
#   E bdsim block diagram flying live in rflysim3d            F RflyTools launchers (.app / .desktop)
#   G RflyGame: race, missions, flight school, collisions, 2-player multiplayer (autopilot)
#   H PX4 fixed wing (SITLRunFw) + ArduPilot copter/plane (APMSITLRun; skipped without an ArduPilot build)
# Usage: scripts/test_all.sh [stages, default ABCDEFGH]
set -u
[ -f "$HOME/.cargo/env" ] && . "$HOME/.cargo/env"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
STAGES="${1:-ABCDEFGH}"
PY="$HOME/tools/rflysim-py/bin/python"
OUT="$ROOT/logs/test_all"; rm -rf "$OUT"; mkdir -p "$OUT"
SIM=target/release/coptersim; VIEW=target/release/rflysim3d
RESULTS=()
res() { RESULTS+=("$1 $2"); echo "== $1: $2"; }
stopall() { pkill -f "build/px4_sitl_default/bin/px4" 2>/dev/null; pkill -x coptersim 2>/dev/null; pkill -x rflysim3d 2>/dev/null; pkill -x rflygame 2>/dev/null; pkill -f SITLRun.sh 2>/dev/null; sleep 1; }
trap stopall EXIT
if pgrep -f "bin/px4( |$)" >/dev/null || pgrep -x coptersim >/dev/null || pgrep -x rflysim3d >/dev/null; then
  echo "Stop running px4/coptersim/rflysim3d first."; exit 1
fi
cargo build --release -q 2>&1 | tail -3
case "$STAGES" in *[CDEGH]*) [ -x "$PY" ] || scripts/Python38Env.sh -c "print('Python env ready')" ;; esac

has() { case "$STAGES" in *"$1"*) return 0 ;; esac; return 1; }
# Waits up to 10 s for the viewer process (a freshly rebuilt .app can take a few seconds to start).
launched() { local i; for i in $(seq 1 20); do pgrep -x rflysim3d >/dev/null && return 0; sleep 0.5; done; return 1; }

if has A; then
  cargo test --workspace -q >"$OUT/A_cargo.log" 2>&1 && res A "PASS unit tests ($(grep -o '[0-9]* passed' "$OUT/A_cargo.log" | awk '{s+=$1} END {print s}') passed)" || res A FAIL
fi

if has B; then
  $VIEW --screenshot "$OUT/B_window.png" --after 8 >"$OUT/B_viewer.log" 2>&1 &
  sleep 1.5
  python3 scripts/test_rflysim3d.py --secs 7 --out "$OUT/B" >"$OUT/B_test.log" 2>&1 && res B "PASS rflysim3d protocol ($(grep -c PASS "$OUT/B_test.log") checks)" || res B "FAIL ($(grep FAIL "$OUT/B_test.log" | head -1))"
  wait
fi

if has C; then
  $SIM --ext 20100 --no-rfly --fg 127.0.0.1:5512 >"$OUT/C_sim.log" 2>&1 &
  sleep 0.5
  "$PY" - >"$OUT/C_fg.log" 2>&1 <<'EOF' && res C "PASS FlightGear native-fdm ($(tail -1 "$OUT/C_fg.log"))" || res C "FAIL ($(tail -1 "$OUT/C_fg.log"))"
import socket, struct, sys, threading
sys.path.insert(0, "python")
from rflyctl.coptersim_ext import CopterSimExt
fg = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); fg.bind(("127.0.0.1", 5512))  # not 5502, so a running FlightGear does not clash; fg.settimeout(2)
pk = []
def rx():
    while len(pk) < 400:
        try: pk.append(fg.recv(1024))
        except socket.timeout: return
threading.Thread(target=rx, daemon=True).start()
s = CopterSimExt(); s.reset()
for _ in range(1000): s.step([0.68] * 4, 4)  # 4 s climb at above-hover throttle
import time; time.sleep(0.3)
assert pk, "no FlightGear packets"
assert all(len(p) == 408 for p in pk), "bad packet size"
ver = struct.unpack(">I", pk[-1][:4])[0]
lon, lat, alt = struct.unpack(">3d", pk[-1][8:32])
agl = struct.unpack(">f", pk[-1][32:36])[0]
rpm = struct.unpack(">f", pk[-1][140:144])[0]
assert ver == 24 and agl > 1.0 and rpm > 1000, (ver, agl, rpm)
print(f"{len(pk)} pkts v{ver}, agl {agl:.1f} m, alt {alt:.1f} m, rpm {rpm:.0f}")
EOF
  pkill -f "coptersim --ext"; sleep 0.5
fi

if has D; then
  SIM_ARGS="--forward udpout:127.0.0.1:24550" scripts/SITLRun.sh 1 >"$OUT/D_sitlrun.log" 2>&1 &
  t=0; until grep -qE "running\. Logs|did not start|already running" "$OUT/D_sitlrun.log" || [ $t -gt 90 ]; do sleep 1; t=$((t + 1)); done
  if grep -q "running\. Logs" "$OUT/D_sitlrun.log"; then
    "$PY" - >"$OUT/D_relay.log" 2>&1 <<'EOF' && res D1 "PASS MAVLink relay ($(tail -1 "$OUT/D_relay.log"))" || res D1 "FAIL relay ($(tail -1 "$OUT/D_relay.log"))"
# In SITL the simulator link carries HIL_ACTUATOR_CONTROLS (no HEARTBEAT); on a real
# Pixhawk (HITLRun) the same relay carries the full MAVLink stream.
from collections import Counter
from pymavlink import mavutil
c = mavutil.mavlink_connection("udpin:0.0.0.0:24550")
seen = Counter()
for _ in range(200):
    m = c.recv_match(blocking=True, timeout=5)
    if m is None: break
    seen[m.get_type()] += 1
assert seen, "nothing relayed"
assert seen["HIL_ACTUATOR_CONTROLS"] > 0, dict(seen)
print("relayed " + ", ".join(f"{k}x{v}" for k, v in seen.most_common(3)))
EOF
    scripts/Python38Env.sh python/examples/offboard_camera.py >"$OUT/D_offboard.log" 2>&1 && res D2 "PASS SITL offboard+camera ($(grep -c PASS "$OUT/D_offboard.log") checks)" || res D2 "FAIL ($(grep FAIL "$OUT/D_offboard.log" | tr '\n' ' '))"
    cp logs/offboard_camera/fpv.png "$OUT/D_fpv.png" 2>/dev/null
  else
    res D "FAIL SITLRun did not start ($(tail -1 "$OUT/D_sitlrun.log"))"
  fi
  stopall
fi

if has E; then
  $SIM --ext 20100 >"$OUT/E_sim.log" 2>&1 &
  $VIEW --screenshot "$OUT/E_window.png" --after 15 >"$OUT/E_viewer.log" 2>&1 &
  sleep 1
  ( sleep 13; "$PY" -c "
import socket,struct; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.sendto(struct.pack('i52s',1234567890,b'RflyShowTextTime bdsim-diagram-live 5'),('127.0.0.1',20010))" ) &
  scripts/Python38Env.sh python/examples/bd_position_control.py --realtime --noplot --save "$OUT/E_scope.png" >"$OUT/E_bd.log" 2>&1 \
    && res E "PASS bdsim diagram live ($(grep 'final position' "$OUT/E_bd.log"))" || res E "FAIL ($(tail -2 "$OUT/E_bd.log" | tr '\n' ' '))"
  grep -q "new vehicle #1" "$OUT/E_viewer.log" && res E2 "PASS rflysim3d showed the bdsim vehicle" || res E2 "FAIL rflysim3d got no vehicle"
  stopall
fi

if has F && [ "$(uname -s)" != Darwin ]; then
  scripts/build_apps.sh >"$OUT/F_build.log" 2>&1 || res F "FAIL build_apps"
  ok=1
  for a in RflySim3D 3DDisplay; do
    RflyTools/bin/$a &
    launched || { ok=0; echo "$a did not launch" >>"$OUT/F_apps.log"; }
    pkill -x rflysim3d; sleep 1
  done
  for c in RflyTools/bin/*; do bash -n "$c" 2>>"$OUT/F_apps.log" || ok=0; done
  for d in RflyTools/*.desktop; do
    grep -q '^\[Desktop Entry\]' "$d" || { ok=0; echo "$d: no [Desktop Entry]" >>"$OUT/F_apps.log"; }
    exe=$(sed -n 's/^Exec="\(.*\)"$/\1/p' "$d"); [ -x "$exe" ] || { ok=0; echo "$d: Exec $exe not executable" >>"$OUT/F_apps.log"; }
    ico=$(sed -n 's/^Icon=//p' "$d"); [ -f "$ico" ] || { ok=0; echo "$d: icon $ico missing" >>"$OUT/F_apps.log"; }
    if command -v desktop-file-validate >/dev/null 2>&1; then desktop-file-validate "$d" >>"$OUT/F_apps.log" 2>&1 || ok=0; fi
  done
  n=$(ls RflyTools/*.desktop | wc -l | tr -d ' ')
  [ $ok = 1 ] && res F "PASS $n .desktop launchers (GUI tools launch; launchers, icons, entries valid)" || res F "FAIL ($(head -2 "$OUT/F_apps.log"))"
elif has F; then
  scripts/build_apps.sh >"$OUT/F_build.log" 2>&1 || res F "FAIL build_apps"
  ok=1
  for a in RflySim3D 3DDisplay; do
    open "RflyTools/$a.app"
    launched || { ok=0; echo "$a did not launch" >>"$OUT/F_apps.log"; }
    pkill -x rflysim3d; sleep 1
  done
  for c in RflyTools/*.app/Contents/Resources/run.command RflyTools/*.app/Contents/MacOS/launch; do
    bash -n "$c" 2>>"$OUT/F_apps.log" || ok=0
  done
  for p in RflyTools/*.app/Contents/Info.plist; do plutil -lint -s "$p" >>"$OUT/F_apps.log" 2>&1 || ok=0; done
  n=$(ls -d RflyTools/*.app | wc -l | tr -d ' ')
  [ $ok = 1 ] && res F "PASS $n app bundles (GUI apps launch; scripts + plists valid)" || res F "FAIL ($(cat "$OUT/F_apps.log" | head -2))"
fi

if has G; then
  G=target/release/rflygame
  gok=1; glines=""
  for a in race=2 mission=0 mission=1 mission=2 school=0; do
    out=$($G --auto $a --quit-after 300 2>&1 | grep "rflygame auto:" | head -1)
    echo "$a: $out" >>"$OUT/G_game.log"
    case "$out" in *finished*|*complete*|*passed*) glines="$glines ok:$a" ;; *) gok=0; glines="$glines FAIL:$a" ;; esac
  done
  c=$($G --auto collide --screenshot "$OUT/G_collide.png" --after 14 2>&1 | grep "auto=")
  echo "collide: $c" >>"$OUT/G_game.log"
  # Realistic crashes: impact disarms, the drone falls and comes to rest (upright here), no crash screen.
  case "$c" in *grounded=true*impact=Some*crashed=None*) glines="$glines ok:collide" ;; *) gok=0; glines="$glines FAIL:collide" ;; esac
  f=$($G --auto flip --screenshot "$OUT/G_flip.png" --after 6 2>&1 | grep "auto=")
  echo "flip: $f" >>"$OUT/G_game.log"
  case "$f" in *grounded=true*inverted=false*) glines="$glines ok:turtle" ;; *) gok=0; glines="$glines FAIL:turtle" ;; esac
  $G --auto host --name Alice --screenshot "$OUT/G_mp_host.png" --after 16 >"$OUT/G_mp_host.log" 2>&1 &
  sleep 2
  $G --auto join=127.0.0.1:27150 --name Bob --screenshot "$OUT/G_mp_bob.png" --after 12 >"$OUT/G_mp_bob.log" 2>&1
  wait
  grep -q "players=2 others=1" "$OUT/G_mp_bob.log" && glines="$glines ok:multiplayer" || { gok=0; glines="$glines FAIL:multiplayer"; }
  # Split screen: 3 pilots on one Mac, each sees the other two (plus the race ghost).
  sp=$(RFLY_SPLIT_ACT=7 $G --auto split=3 --screenshot "$OUT/G_split.png" --after 12 2>&1 | grep "auto=split")
  echo "split: $sp" >>"$OUT/G_game.log"
  case "$sp" in *players=3*P1:*armed=true*P2:*armed=true*P3:*armed=true*) glines="$glines ok:splitscreen" ;; *) gok=0; glines="$glines FAIL:splitscreen" ;; esac
  # Virtual controllers: 4 pads join split screen, pick a race, start it and fly (no keyboard, no autopilot).
  $G --vpad --auto split=0 --screenshot "$OUT/G_vpad.png" --after 24 >"$OUT/G_vpad.log" 2>&1 &
  sleep 2
  scripts/Python38Env.sh python/examples/vpad_split_test.py >>"$OUT/G_vpad.log" 2>&1
  wait
  vp=$(grep "auto=split" "$OUT/G_vpad.log")
  echo "vpad: $vp" >>"$OUT/G_game.log"
  [ "$(echo "$vp" | grep -o 'Virtual_controller_[1-4]:[A-Za-z]*:armed=true,alt=[1-9]' | wc -l | tr -d ' ')" = 4 ] && echo "$vp" | grep -q "Virtual_controller_4:Beginner" \
    && glines="$glines ok:vpad" || { gok=0; glines="$glines FAIL:vpad"; }
  [ $gok = 1 ] && res G "PASS RflyGame ($glines )" || res G "FAIL RflyGame ($glines )"
fi

if has H; then
  echo "== H: fixed wing + ArduPilot"
  hl=""; hok=1
  scripts/test_fixedwing.sh 60 >"$OUT/H_fixedwing.log" 2>&1 && hl="$hl ok:px4-plane" || { hok=0; hl="$hl FAIL:px4-plane"; }
  if [ -x "${ARDUPILOT_DIR:-$HOME/tools/ardupilot}/build/sitl/bin/arducopter" ]; then
    "$PY" scripts/test_ardupilot.py both >"$OUT/H_ardupilot.log" 2>&1 && hl="$hl ok:apm-copter ok:apm-plane" \
      || { hok=0; hl="$hl FAIL:ardupilot($(grep -c '^FAIL' "$OUT/H_ardupilot.log"))"; }
  else
    hl="$hl skip:ardupilot(not built)"
  fi
  [ $hok = 1 ] && res H "PASS fixed wing / ArduPilot ($hl )" || res H "FAIL fixed wing / ArduPilot ($hl )"
fi

echo; echo "========== SUMMARY =========="
printf '%s\n' "${RESULTS[@]}"
echo "artefacts: $OUT"
! printf '%s\n' "${RESULTS[@]}" | grep -q FAIL
