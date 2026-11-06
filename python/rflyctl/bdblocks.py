"""bdsim blocks for RflySim-style controller design without MATLAB/Simulink.

    COPTERSIM  the real coptersim physics (lockstep over `coptersim --ext`); also streams to rflysim3d
    POSCTRL    NED position → velocity → tilt setpoints
    ATTCTRL    altitude + attitude (angle P → rate PI) + PX4 quad-x mixer
    WAYPOINTS  piecewise-constant [x, y, z, yaw] targets over time

All are clocked (discrete) blocks on one bdsim clock, like the 250 Hz PX4 controllers that
RflySim's Simulink demos generate. Each clocked block adds a one-sample delay, as on hardware.
"""
import time

import numpy as np
from bdsim.block import SampledBlock, SourceBlock

from .controllers import AttitudeAltCtrl, PositionCtrl
from .coptersim_ext import CopterSimExt


class COPTERSIM(SampledBlock):
    """Outputs: 0 pos NED[3], 1 vel NED[3], 2 euler[3], 3 body rate[3], 4 full 39-value state.
    Input: 0 motor throttles[8] (0..1)."""

    nin = 1
    nout = 5
    inlabels = ("u",)
    outlabels = ("pos", "vel", "euler", "rate", "y")

    def __init__(self, clock, host="127.0.0.1", port=20100, realtime=False, **blockargs):
        self.sim = CopterSimExt(host, port)
        super().__init__(clock=clock, nin=1, nout=5, x0=np.array(self.sim.reset()), **blockargs)
        self.nsteps = max(1, int(round(clock.T * 1000)))
        self.realtime = realtime
        self._wall0 = None

    def start(self, simstate):
        super().start(simstate)
        self.sim.reset()
        self._wall0 = time.time()

    def output(self, t, inports, x):
        return [x[1:4], x[4:7], x[11:14], x[14:17], x]

    def next(self, t, inports, x):
        u = np.clip(np.ravel(inports[0])[:8], 0.0, 1.0)
        y = np.array(self.sim.step(u.tolist(), self.nsteps))
        if self.realtime and self._wall0 is not None:
            lag = y[0] - (time.time() - self._wall0)
            if lag > 0:
                time.sleep(lag)
        return y


class POSCTRL(SampledBlock):
    """Inputs: 0 target [x,y,z,yaw], 1 pos, 2 vel, 3 euler. Output: sp [roll,pitch,yaw,z]."""

    nin = 4
    nout = 1
    inlabels = ("target", "pos", "vel", "euler")
    outlabels = ("sp",)

    def __init__(self, clock, **blockargs):
        super().__init__(clock=clock, nin=4, nout=1, x0=np.zeros(4), **blockargs)
        self.ctrl = PositionCtrl()

    def start(self, simstate):
        super().start(simstate)
        self.ctrl.reset()

    def output(self, t, inports, x):
        return [x]

    def next(self, t, inports, x):
        tgt, pos, vel, eul = (np.ravel(p) for p in inports)
        return np.array(self.ctrl(tgt[:3], tgt[3], pos, vel, eul[2], self.T))


class ATTCTRL(SampledBlock):
    """Inputs: 0 sp [roll,pitch,yaw,z], 1 euler, 2 rate, 3 pos, 4 vel. Output: motors[8]."""

    nin = 5
    nout = 1
    inlabels = ("sp", "euler", "rate", "pos", "vel")
    outlabels = ("u",)

    def __init__(self, clock, **blockargs):
        super().__init__(clock=clock, nin=5, nout=1, x0=np.zeros(8), **blockargs)
        self.ctrl = AttitudeAltCtrl()

    def start(self, simstate):
        super().start(simstate)
        self.ctrl.reset()

    def output(self, t, inports, x):
        return [x]

    def next(self, t, inports, x):
        sp, eul, rate, pos, vel = (np.ravel(p) for p in inports)
        return np.array(self.ctrl(sp, eul, rate, pos, vel, self.T))


class WAYPOINTS(SourceBlock):
    """Piecewise-constant targets: points = [(t0, [x,y,z,yaw]), (t1, [...]), ...]."""

    nin = 0
    nout = 1
    outlabels = ("target",)

    def __init__(self, points, **blockargs):
        super().__init__(nout=1, **blockargs)
        self.points = sorted((float(t), np.array(p, float)) for t, p in points)

    def output(self, t, inports, x):
        cur = self.points[0][1]
        for t0, p in self.points:
            if t >= t0:
                cur = p
        return [cur]
