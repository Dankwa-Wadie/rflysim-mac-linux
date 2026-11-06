"""Demo flight for FlightGear-F450: flies coptersim (external-controller mode) in real time.

Climbs to 8 m, hovers, then flies slow 6 m circles, so FlightGear shows RflySim's F450 moving.
Started by scripts/FlightGearDemo.sh; it needs `coptersim --ext 20100 --fg` running.

Usage: python fg_demo_flight.py [seconds]   (default 120)
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from rflyctl.coptersim_ext import CopterSimExt as C  # noqa: E402
from rflyctl.controllers import AttitudeAltCtrl, PositionCtrl  # noqa: E402

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
sim = C()
y = sim.reset()
att, pc = AttitudeAltCtrl(), PositionCtrl()
dt, k, t0 = 0.02, 0, time.time()
while time.time() - t0 < dur:
    t = time.time() - t0
    a = (t - 15) * 0.3
    target = [0, 0, -8] if t < 15 else [6 * math.cos(a) - 6, 6 * math.sin(a), -8]
    if t > dur - 12:
        target = [target[0], target[1], 0.5]  # land at the end
    pos, vel, eu, rate = (C.field(y, f) for f in ("pos", "vel", "euler", "rate"))
    motors = att(pc(target, 0.0, pos, vel, eu[2], dt), eu, rate, pos, vel, dt)
    y = sim.step(motors, 20)
    k += 1
    if k % 250 == 0:
        print(f"t={t:5.1f}s  pos N {pos[0]:5.1f}  E {pos[1]:5.1f}  alt {-pos[2]:4.1f} m", flush=True)
    time.sleep(max(0.0, t0 + k * dt - time.time()))
