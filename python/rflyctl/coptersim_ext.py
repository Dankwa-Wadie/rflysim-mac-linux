"""Lockstep client for `coptersim --ext` (RflyMac protocol v1), the controller-in-the-loop
interface that Simulink uses in RflySim. The controller owns time: each step() sends motor
throttles plus a number of 1 ms physics steps, and returns the resulting state.

Request (80 B):  u32 'RFC1', u32 seq, u32 flags (1 = reset), u32 nsteps, f64 motors[8]
Reply  (320 B):  u32 'RFS1', u32 seq, f64 data[39]   (field map: FIELDS below)
"""
import socket, struct

REQ_MAGIC = struct.unpack("<I", b"RFC1")[0]
REP_MAGIC = struct.unpack("<I", b"RFS1")[0]

# name → slice into the 39-value reply
FIELDS = {
    "t": slice(0, 1), "pos": slice(1, 4), "vel": slice(4, 7), "quat": slice(7, 11),
    "euler": slice(11, 14), "rate": slice(14, 17), "acc": slice(17, 20), "gyro": slice(20, 23),
    "mag": slice(23, 26), "baro_alt": slice(26, 27), "gps": slice(27, 30),
    "motor_w": slice(30, 38), "on_ground": slice(38, 39),
}


class CopterSimExt:
    def __init__(self, host="127.0.0.1", port=20100, timeout=2.0):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)
        self.seq = 0

    def step(self, motors, nsteps=4, reset=False):
        """Advance nsteps × 1 ms with these throttles (0..1, up to 8). Returns the 39-value list."""
        m = (list(motors) + [0.0] * 8)[:8]
        self.seq += 1
        self.sock.sendto(struct.pack("<4I8d", REQ_MAGIC, self.seq, 1 if reset else 0, nsteps, *m), self.addr)
        while True:
            buf, _ = self.sock.recvfrom(1024)
            magic, seq = struct.unpack("<2I", buf[:8])
            if magic == REP_MAGIC and seq == self.seq:
                return list(struct.unpack("<39d", buf[8:320]))

    def reset(self):
        return self.step([0] * 8, 0, reset=True)

    @staticmethod
    def field(y, name):
        v = y[FIELDS[name]]
        return v[0] if len(v) == 1 else v
