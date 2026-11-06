#!/usr/bin/env python3
"""Fly a square pattern with PX4 SITL + coptersim, then land.

Example of MATLAB-free experiment scripting. Talks MAVLink to PX4's API port (UDP 14540).
Run with:  ~/tools/px4-venv/bin/python scripts/fly_square.py [--side 80] [--alt 30]
"""
import argparse
import math
import time

from pymavlink import mavutil

p = argparse.ArgumentParser()
p.add_argument("--url", default="udpin:0.0.0.0:14540")
p.add_argument("--side", type=float, default=80.0, help="square side (m)")
p.add_argument("--alt", type=float, default=30.0, help="height above home (m)")
p.add_argument("--land", action=argparse.BooleanOptionalAction, default=True)
a = p.parse_args()

m = mavutil.mavlink_connection(a.url)
m.wait_heartbeat(timeout=10)
print(f"connected to system {m.target_system}")


def msg(kind):
    return m.recv_match(type=kind, blocking=True, timeout=5)


home = msg("HOME_POSITION")
if home is None:  # ask for it if not streamed
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 0, 0, 0, 0, 0, 0)
    home = msg("HOME_POSITION")
lat0, lon0, alt0 = home.latitude / 1e7, home.longitude / 1e7, home.altitude / 1e3
print(f"home {lat0:.6f}, {lon0:.6f}, {alt0:.1f} m AMSL")

pos = msg("LOCAL_POSITION_NED")
if pos is None or -pos.z < 1.0:
    print("taking off ...")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(1)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, math.nan,
                            math.nan, math.nan, alt0 + a.alt)


def goto(n, e):
    lat = lat0 + math.degrees(n / 6378137.0)
    lon = lon0 + math.degrees(e / (6378137.0 * math.cos(math.radians(lat0))))
    m.mav.command_int_send(m.target_system, m.target_component,
                           mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                           0, 0, -1, 1, 0, math.nan,
                           int(lat * 1e7), int(lon * 1e7), alt0 + a.alt)
    t0 = time.time()
    while time.time() - t0 < 90:
        lp = msg("LOCAL_POSITION_NED")
        if lp and math.hypot(lp.x - n, lp.y - e) < 2.0 and abs(-lp.z - a.alt) < 2.0:
            print(f"  reached N={n:.0f} E={e:.0f}  (alt {-lp.z:.1f} m)")
            return True
    print(f"  timeout going to N={n:.0f} E={e:.0f}")
    return False


s = a.side
for n, e in [(0, 0), (s, 0), (s, s), (0, s), (0, 0)]:
    goto(n, e)

if a.land:
    print("landing ...")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, math.nan, 0, 0, 0)
    while True:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=60)
        if hb and not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("landed and disarmed")
            break
