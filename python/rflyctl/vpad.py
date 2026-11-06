"""Virtual game controllers for RflyGame (start the game with ``--vpad``).

Each ``VirtualPad`` behaves like a Mode 2 gamepad: it can join split screen, drive menus and
fly. The game drops a pad 2 s after its last packet, so call ``send()`` (or ``hold()``)
regularly; ``press()`` taps a button.

    from rflyctl.vpad import VirtualPad
    p = VirtualPad(0)
    p.press("A")                         # join / select / arm
    p.sticks(throttle=0.4, pitch=0.3)    # climb and fly forward
    p.hold(2.0)                          # keep sending for 2 s

Packet: b"RVP1", u8 pad, u16 buttons, f32 lx, ly, rx, ry (little endian).
"""
import socket
import struct
import time

PORT = 27170
BUTTONS = {"A": 0, "B": 1, "Y": 2, "X": 3, "START": 4, "SELECT": 5, "UP": 6, "DOWN": 7, "LEFT": 8, "RIGHT": 9}


class VirtualPad:
    def __init__(self, index, host="127.0.0.1", port=PORT, rate=60.0):
        assert 0 <= index <= 3
        self.index, self.addr, self.dt = index, (host, port), 1.0 / rate
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.buttons = 0
        self.axes = [0.0, 0.0, 0.0, 0.0]  # lx (yaw), ly (throttle), rx (roll), ry (pitch)

    def sticks(self, throttle=None, yaw=None, pitch=None, roll=None):
        """Sets Mode 2 stick positions (-1..1; throttle up, yaw/roll right, pitch forward = +)."""
        for i, v in ((1, throttle), (0, yaw), (3, pitch), (2, roll)):
            if v is not None:
                self.axes[i] = max(-1.0, min(1.0, float(v)))
        self.send()

    def down(self, *names):
        for n in names:
            self.buttons |= 1 << BUTTONS[n.upper()]
        self.send()

    def up(self, *names):
        for n in names:
            self.buttons &= ~(1 << BUTTONS[n.upper()])
        self.send()

    def send(self):
        self.sock.sendto(struct.pack("<4sBH4f", b"RVP1", self.index, self.buttons, *self.axes), self.addr)

    def hold(self, secs):
        """Keeps the current state alive for `secs`."""
        t = time.time()
        while time.time() - t < secs:
            self.send()
            time.sleep(self.dt)

    def press(self, name, secs=0.12):
        """Taps a button: held long enough for the game to see at least a few frames."""
        self.down(name)
        self.hold(secs)
        self.up(name)
        self.hold(secs)


def hold_all(pads, secs, dt=1 / 60):
    """Keeps several pads alive at once."""
    t = time.time()
    while time.time() - t < secs:
        for p in pads:
            p.send()
        time.sleep(dt)


def press_all(pads, name, secs=0.12):
    for p in pads:
        p.down(name)
    hold_all(pads, secs)
    for p in pads:
        p.up(name)
    hold_all(pads, secs)
