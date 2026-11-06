#!/usr/bin/env python3
"""Protocol test for the macOS rflysim3d viewer, using only the stdlib.

It sends the same packets RflySimSDK's UE4CtrlAPI / VisionCaptureApi send:
vehicle states (sendUE4PosNew/sendUE4Pos2Ground), console commands, labels and
vision-sensor requests. Then it reassembles the RGB and depth images that come back over UDP.

Usage: python3 scripts/test_rflysim3d.py [--secs 6] [--out DIR]
Exit code 0 means every check passed.
"""
import argparse, math, os, socket, struct, sys, threading, time

UE = ("127.0.0.1", 20010)


def cmd(s, sock):
    b = s.encode()
    sock.sendto(struct.pack("i52s" if len(b) <= 51 else "i252s", 1234567890, b), UE)


def pos_new(sock, cid, vtype, pos, ang, rpm, t):
    # SOut2SimulatorSimpleTime (6i14f4d), the format CopterSim streams.
    sock.sendto(struct.pack("6i14f4d", 1234567891, cid, vtype, 0, 0, 0, *([rpm] * 8), 0, 0, 0, *ang, *pos, t), UE)


def pos_ground(sock, cid, vtype, pos, yaw):
    sock.sendto(struct.pack("3i7f", 1234567891, cid, vtype, 0, *pos, 0, 0, yaw), UE)


def label(sock, cid, txt, rgb=(255, 255, 0)):
    sock.sendto(struct.pack("6i2f120s", 1234567899, cid, -1, *rgb, 0.0, 20.0, txt.encode()), UE)


def vision_req(sock, seq, typ, w, h, freq, proto, port, pos, ang, fov=90):
    ints = [12345, seq, typ, 1, 0, w, h, freq, proto, 127, 0, 0, 1, port, 0, 0]
    flo = [fov, *pos, 0, *ang, 0, 0, 0, 0] + [0] * 16
    sock.sendto(struct.pack("16H28f", *ints, *flo), UE)


class ImgRx(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 60000 * 100)
        self.s.bind(("0.0.0.0", port))
        self.s.settimeout(0.5)
        self.frames, self.last, self.stamp = 0, None, None
        self.parts, self.total = {}, 0

    def run(self):
        while True:
            try:
                buf, _ = self.s.recvfrom(62000)
            except socket.timeout:
                continue
            ck, ln, seq, tot, ts = struct.unpack("4i1d", buf[:24])
            if ck != 1234567890 or ln != len(buf):
                continue
            if seq == 0 or ts != self.stamp:
                self.parts, self.total, self.stamp = {}, tot, ts
            self.parts[seq] = buf[24:]
            if len(self.parts) == self.total:
                self.last = b"".join(self.parts[i] for i in range(self.total))
                self.frames += 1
                self.parts = {}


def png_info(b):
    if b[:8] != b"\x89PNG\r\n\x1a\n":
        return None
    w, h, depth, ctype = struct.unpack(">IIBB", b[16:26])
    return w, h, depth, ctype


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=6.0)
    ap.add_argument("--out", default="logs/rflysim3d_test")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Listen for replies (RflyReqObjData → CoptReqData on 20006).
    rep = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rep.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rep.bind(("0.0.0.0", 20006))
    rep.setblocking(False)

    rgb, dep = ImgRx(9999), ImgRx(10000)
    rgb.start(); dep.start()

    cmd("RflyChangeMapbyName Grasslands", s)
    cmd("RflyShowTextTime macOS-RflySim3D-protocol-test 5", s)
    vision_req(s, 0, 1, 640, 480, 15, 1, 9999, [0.3, 0, 0], [0, -0.3, 0])  # RGB, JPEG
    vision_req(s, 1, 2, 320, 240, 10, 3, 10000, [0.3, 0, 0], [0, -0.3, 0])  # depth, PNG16
    time.sleep(0.3)
    cmd("RflyReqObjData 1 1 0", s)

    t0 = time.time()
    copt_replies = 0
    labelled = False
    while time.time() - t0 < a.secs:
        t = time.time() - t0
        yaw = 0.6 * t
        p1 = [8 * math.cos(yaw), 8 * math.sin(yaw), -4.0]
        pos_new(s, 1, 3, p1, [0.15, -0.05, yaw + math.pi / 2], 9000, t)
        pos_new(s, 2, 6, [12, -6, -6 - math.sin(t)], [0, 0, 0.3 * t], 8000, t)
        pos_new(s, 3, 100, [30 * math.cos(-0.3 * t), 30 * math.sin(-0.3 * t), -15], [-0.35, 0, -0.3 * t - math.pi / 2], 6000, t)
        pos_ground(s, 4, 30, [10, 6, 0], 1.0)
        if not labelled and t > 0.5:
            label(s, 1, "PX4 SITL #1")
            labelled = True
        try:
            while True:
                buf, _ = rep.recvfrom(2000)
                if len(buf) == 80 and struct.unpack("i", buf[:4])[0] == 1234567891:
                    copt_replies += 1
        except BlockingIOError:
            pass
        time.sleep(1 / 60)

    ok = True
    def check(name, cond, extra=""):
        nonlocal ok
        print(("PASS " if cond else "FAIL ") + name + (" " + extra if extra else ""))
        ok &= bool(cond)

    check("RGB frames received", rgb.frames > 0, f"({rgb.frames})")
    if rgb.last:
        open(os.path.join(a.out, "cam_rgb.jpg"), "wb").write(rgb.last)
        check("RGB is JPEG", rgb.last[:2] == b"\xff\xd8", f"{len(rgb.last)} B")
    check("depth frames received", dep.frames > 0, f"({dep.frames})")
    if dep.last:
        open(os.path.join(a.out, "cam_depth.png"), "wb").write(dep.last)
        info = png_info(dep.last)
        check("depth is 16-bit grey PNG 320x240", info == (320, 240, 16, 0), str(info))
    check("CoptReqData replies on 20006", copt_replies > 0, f"({copt_replies})")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
