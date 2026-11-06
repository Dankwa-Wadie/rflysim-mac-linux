"""Simulink-style position control of the RflySim quad, as a bdsim block diagram (no MATLAB).

    WAYPOINTS ─▶ POSCTRL ─▶ ATTCTRL ─▶ COPTERSIM ─┬─▶ SCOPEs
                  ▲  ▲        ▲  ▲                 │
                  └──┴────────┴──┴── pos/vel/att ──┘

Run:  target/release/coptersim --ext            (lockstep physics; also feeds rflysim3d)
      target/release/rflysim3d                  (optional: watch it fly)
      scripts/Python38Env.sh python/examples/bd_position_control.py [--realtime] [--noplot]
"""
import os, sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import bdsim
import numpy as np
from rflyctl.bdblocks import ATTCTRL, COPTERSIM, POSCTRL, WAYPOINTS

realtime = "--realtime" in sys.argv
noplot = "--noplot" in sys.argv

sim = bdsim.BDSim(graphics=not noplot, progress=False)
bd = sim.blockdiagram()
clock = bd.clock(0.004, unit="s")  # 250 Hz, like PX4's rate loop

wp = WAYPOINTS([(0, [0, 0, -5, 0]), (8, [10, 5, -8, 1.0]), (20, [0, 0, -3, 1.0])], name="waypoints")
pos_ctrl = POSCTRL(clock, name="position ctrl")
att_ctrl = ATTCTRL(clock, name="attitude ctrl")
copter = COPTERSIM(clock, realtime=realtime, name="CopterSim")
for b in (wp, pos_ctrl, att_ctrl, copter):
    bd.add_block(b)

bd.connect(wp, pos_ctrl[0])
bd.connect(copter[0], pos_ctrl[1], att_ctrl[3])
bd.connect(copter[1], pos_ctrl[2], att_ctrl[4])
bd.connect(copter[2], pos_ctrl[3], att_ctrl[1])
bd.connect(copter[3], att_ctrl[2])
bd.connect(pos_ctrl, att_ctrl[0])
bd.connect(att_ctrl, copter[0])

if not noplot:
    pos_scope = bd.SCOPE(nin=1, vector=3, labels=["N", "E", "D"], name="position NED (m)")
    att_scope = bd.SCOPE(nin=1, vector=3, labels=["roll", "pitch", "yaw"], name="attitude (rad)")
    bd.connect(copter[0], pos_scope)
    bd.connect(copter[2], att_scope)

bd.compile()
out = sim.run(bd, T=30, watch=[copter[0], copter[2]])

final = out.y[-1, 0:3]  # watched signal 0 = position (3 columns)
print("final position NED:", np.round(final, 2))
ok = np.allclose(final, [0, 0, -3], atol=0.3)
print("PASS" if ok else "FAIL", "reached last waypoint (0, 0, -3)")
if "--save" in sys.argv:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    path = sys.argv[sys.argv.index("--save") + 1]
    fig, (a, b) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    tgt = np.array([wp.output(t, [], None)[0] for t in out.t])
    for k, (n, c) in enumerate(zip("NED", "tab:blue tab:orange tab:green".split())):
        a.plot(out.t, out.y[:, k], c, label=n)
        a.plot(out.t, tgt[:, k], c, ls="--", lw=0.8)
    a.set_ylabel("position (m)"); a.legend(ncol=3); a.grid(alpha=0.3)
    a.set_title("bdsim position control of CopterSim (dashed = waypoint)")
    for k, n in enumerate(["roll", "pitch", "yaw"]):
        b.plot(out.t, np.degrees(out.y[:, 3 + k]), label=n)
    b.set_ylabel("attitude (deg)"); b.set_xlabel("time (s)"); b.legend(ncol=3); b.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path, dpi=110)
    print("saved", path)
if not noplot:
    sim.done(bd, block=True)
sys.exit(0 if ok else 1)
