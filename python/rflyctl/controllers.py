"""Reference multicopter controllers (pure Python), structured like RflySim's Simulink demos.

AttitudeAltCtrl: port of matlab/RflyMac/rflymac_attitude_ctrl.m (and the C reference in
  matlab/RflyMac/tests/test_client.c): altitude pos→vel→throttle, angle P → rate PI, PX4 quad-x mixer.
PositionCtrl: outer loop, NED position → velocity → tilt setpoints for AttitudeAltCtrl.
"""
import math


def clamp(x, lo, hi):
    return min(max(x, lo), hi)


class AttitudeAltCtrl:
    HOVER = 0.608  # hover throttle of the default RflySim quad (ModelData.db)
    KP_Z, VZ_MAX = 1.0, 2.0
    KP_VZ, KI_VZ = 0.05, 0.02
    KP_ANG = (5.0, 5.0, 3.0)
    KP_RATE = (0.03, 0.03, 0.2)
    KI_RATE = (0.02, 0.02, 0.05)

    def __init__(self):
        self.reset()

    def reset(self):
        self.iz = 0.0
        self.ir = [0.0, 0.0, 0.0]

    def __call__(self, sp, euler, rate, pos, vel, dt):
        """sp = [roll, pitch, yaw, z] (rad, NED m). Returns 8 throttles 0..1 (1 FR, 2 RL, 3 FL, 4 RR)."""
        ez = sp[3] - pos[2]
        vz_sp = clamp(self.KP_Z * ez, -self.VZ_MAX, self.VZ_MAX)
        self.iz = clamp(self.iz + self.KI_VZ * (vel[2] - vz_sp) * dt, -0.1, 0.1)
        thr = self.HOVER + self.KP_VZ * (vel[2] - vz_sp) + self.iz
        thr = clamp(thr / max(math.cos(euler[0]) * math.cos(euler[1]), 0.5), 0.05, 0.95)
        tau = [0.0, 0.0, 0.0]
        for i in range(3):
            e = sp[i] - euler[i]
            if i == 2:
                e = math.atan2(math.sin(e), math.cos(e))
            er = self.KP_ANG[i] * e - rate[i]
            self.ir[i] = clamp(self.ir[i] + self.KI_RATE[i] * er * dt, -0.1, 0.1)
            tau[i] = self.KP_RATE[i] * er + self.ir[i]
        m = [
            thr - tau[0] + tau[1] + tau[2],
            thr + tau[0] - tau[1] + tau[2],
            thr + tau[0] + tau[1] - tau[2],
            thr - tau[0] - tau[1] - tau[2],
        ]
        return [clamp(x, 0.0, 1.0) for x in m] + [0.0] * 4


class PositionCtrl:
    """NED position → velocity (P) → horizontal acceleration (PI) → roll/pitch setpoints."""
    KP_POS, V_MAX = 0.8, 3.0
    KP_VEL, KI_VEL = 2.0, 0.3
    TILT_MAX = math.radians(25)
    G = 9.8

    def __init__(self):
        self.reset()

    def reset(self):
        self.ivel = [0.0, 0.0]

    def __call__(self, target, yaw_sp, pos, vel, yaw, dt):
        """target = [x, y, z] NED. Returns sp = [roll, pitch, yaw, z] for AttitudeAltCtrl."""
        acc = [0.0, 0.0]
        for i in range(2):
            v_sp = clamp(self.KP_POS * (target[i] - pos[i]), -self.V_MAX, self.V_MAX)
            ev = v_sp - vel[i]
            self.ivel[i] = clamp(self.ivel[i] + self.KI_VEL * ev * dt, -2.0, 2.0)
            acc[i] = self.KP_VEL * ev + self.ivel[i]
        # Rotate the NED acceleration demand into the yaw frame, then into tilt angles.
        c, s = math.cos(yaw), math.sin(yaw)
        ax_b = c * acc[0] + s * acc[1]
        ay_b = -s * acc[0] + c * acc[1]
        pitch = clamp(-math.atan2(ax_b, self.G), -self.TILT_MAX, self.TILT_MAX)
        roll = clamp(math.atan2(ay_b, self.G), -self.TILT_MAX, self.TILT_MAX)
        return [roll, pitch, yaw_sp, target[2]]
