"""Prints the MAVLink system ID of the autopilot on a serial port (HITLRun --sysid).
Usage: python mav_sysid.py PORT [BAUD]"""
import sys
from pymavlink import mavutil

port, baud = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 921600
m = mavutil.mavlink_connection(port, baud=baud)
hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
m.close()
if hb is None:
    sys.exit("no heartbeat")
print(hb.get_srcSystem())
