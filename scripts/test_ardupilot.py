#!/usr/bin/env python3
"""ArduPilot SITL flight check for APMSITLRun (copter and/or plane).

Starts scripts/APMSITLRun.sh 1 --no-3d [--plane], talks MAVLink to ArduPilot on TCP 5760 and flies:
  copter: GUIDED, arm, take off to 10 m, hold, LAND, disarm on the ground
  plane:  TAKEOFF mode, arm, climb out, then check it holds height and flying speed
Prints PASS/FAIL lines; exit code 0 when everything passed.

    scripts/test_ardupilot.py [copter|hexa|quad_plus|plane|both]     (needs pymavlink: the Python38Env venv has it)
"""
import os, subprocess, sys, time

from pymavlink import mavutil

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FAILS = []


def check(name, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  ({detail})" if detail else ""), flush=True)
    if not ok:
        FAILS.append(name)


def connect(timeout=90):
    end = time.time() + timeout
    while time.time() < end:
        try:
            m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255)
            if m.wait_heartbeat(timeout=5):
                return m
        except OSError:
            pass
        time.sleep(1)
    return None


def set_mode(m, name):
    mapping = m.mode_mapping()
    m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping[name])


def state(m, timeout=1.0):
    """Latest (relative altitude m, ground speed m/s, armed) after draining messages."""
    end = time.time() + timeout
    alt = gs = None
    while time.time() < end:
        msg = m.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "GLOBAL_POSITION_INT":
            alt = msg.relative_alt / 1000.0
        elif t == "VFR_HUD":
            gs = msg.airspeed
        elif t == "STATUSTEXT":
            print("   ArduPilot:", msg.text, flush=True)
    return alt, gs, m.motors_armed()


def arm_when_ready(m, mode, timeout=120):
    """Retry mode + arm until ArduPilot accepts (pre-arm checks pass once the EKF has GPS)."""
    end = time.time() + timeout
    while time.time() < end:
        set_mode(m, mode)
        m.arducopter_arm()
        t0 = time.time()
        while time.time() - t0 < 3:
            state(m, 0.5)
            if m.motors_armed():
                return True
    return False


def fly_copter(m):
    m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    check("copter armed in GUIDED", arm_when_ready(m, "GUIDED"))
    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
    alts = []
    end = time.time() + 25
    while time.time() < end:
        a, _, _ = state(m, 1.0)
        if a is not None:
            alts.append(a)
    top = max(alts) if alts else 0
    last = alts[-5:] if alts else [0]
    check("copter took off to 10 m", 8.5 < top < 12.5, f"max {top:.1f} m")
    check("copter holds 10 m", all(abs(a - 10) < 1.0 for a in last), " ".join(f"{a:.1f}" for a in last))
    set_mode(m, "LAND")
    end = time.time() + 40
    a = top
    while time.time() < end:
        a2, _, armed = state(m, 1.0)
        a = a2 if a2 is not None else a
        if not armed:
            break
    check("copter landed and disarmed", not m.motors_armed() and a < 0.5, f"alt {a:.2f} m")


def fly_plane(m):
    m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    check("plane armed in TAKEOFF", arm_when_ready(m, "TAKEOFF"))
    samples = []
    end = time.time() + 70
    while time.time() < end:
        a, gs, _ = state(m, 1.0)
        if a is not None and gs is not None:
            samples.append((a, gs))
    late = samples[-20:] or [(0, 0)]
    top = max(s[0] for s in samples) if samples else 0
    check("plane climbed out", top > 30, f"max {top:.0f} m")
    check("plane holds height", min(s[0] for s in late) > 25, f"last 20 s {min(s[0] for s in late):.0f}-{max(s[0] for s in late):.0f} m")
    check("plane flying speed", all(9 < s[1] < 30 for s in late), f"airspeed {late[-1][1]:.1f} m/s")


def run(kind):
    print(f"== ArduPilot {kind}", flush=True)
    extra = {"plane": ["--plane"], "hexa": ["--frame", "hexa_x"], "quad_plus": ["--frame", "quad_plus"]}.get(kind, [])
    # With RFLY_RECORD set (frame recorder), RflySim3D runs too, so the flight can be recorded.
    view = [] if os.environ.get("RFLY_RECORD") else ["--no-3d"]
    args = [os.path.join(ROOT, "scripts", "APMSITLRun.sh"), "1"] + view + extra
    proc = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        m = connect()
        check(f"{kind}: MAVLink heartbeat on TCP 5760", m is not None)
        if m:
            (fly_plane if kind == "plane" else fly_copter)(m)
    finally:
        proc.terminate()
        try:
            proc.wait(10)
        except subprocess.TimeoutExpired:
            proc.kill()
        subprocess.run([os.path.join(ROOT, "scripts", "CloseALL.sh"), "--keep-qgc"], stdout=subprocess.DEVNULL)


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "both"
    for k in (["copter", "plane"] if what == "both" else [what]):
        run(k)
    print("ALL PASSED" if not FAILS else f"FAILED: {', '.join(FAILS)}")
    sys.exit(1 if FAILS else 0)
