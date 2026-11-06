"""Flies with RflySim's UDP communication modes (UDPSIMMODE 0 UDP_Full / 1 UDP_Simple), like Windows scripts
that call InitMavLoop(0 or 1). Start the simulation first: scripts/SITLRun.sh 1 --udp-mode 0 (or 1).
Usage: python udp_mode_test.py MODE"""
import sys, time
import PX4MavCtrlV4 as PX4MavCtrl

mode = int(sys.argv[1]) if len(sys.argv) > 1 else 0
mav = PX4MavCtrl.PX4MavCtrler(1)
mav.InitMavLoop(mode)
mav.InitTrueDataLoop()
t0 = time.time()
while not mav.isPX4Ekf3DFixed and time.time() - t0 < 60:
    time.sleep(0.5)
print("mode %d: EKF ready %s after %.0f s, home %s" % (mode, mav.isPX4Ekf3DFixed, time.time() - t0, [round(v, 4) for v in mav.uavPosGPSHome]))
mav.initOffboard()
mav.SendPosNED(0, 0, -5, 0)
time.sleep(1)
# Arm, retrying until PX4's preflight checks pass (as a careful script would). UDP modes report no armed state.
t1 = time.time()
while time.time() - t1 < 60:
    if mode < 2:
        mav.SendMavArm(1)
        break
    mav.initOffboard()  # MAVLink modes: arm + OFFBOARD, again until PX4 accepts
    mav.SendPosNED(0, 0, -5, 0)
    time.sleep(1.5)
    if mav.isArmed:
        break
print("armed after %.0f s" % (time.time() - t1))
time.sleep(13)
ok_h = abs(mav.truePosNED[2] + 5) < 0.6 and abs(mav.uavPosNED[2] + 5) < 0.6
print("hover: bridge NED", [round(v, 2) for v in mav.uavPosNED], " true NED", [round(v, 2) for v in mav.truePosNED])
mav.SendPosNED(5, 3, -6, 0)
time.sleep(12)
ok_m = abs(mav.truePosNED[0] - 5) < 0.8 and abs(mav.truePosNED[1] - 3) < 0.8
print("moved: bridge NED", [round(v, 2) for v in mav.uavPosNED], " true NED", [round(v, 2) for v in mav.truePosNED])
# Without GPS (Mavlink_NoGPS/Vision) PX4 publishes no home position, so there is no EKF-ready flag to expect.
checks = [("hover 5 m", ok_h), ("move to (5,3,-6)", ok_m)]
if mode not in (5, 6):
    checks.insert(0, ("EKF ready", mav.isPX4Ekf3DFixed))
for name, ok in checks:
    print(("PASS " if ok else "FAIL ") + name)
mav.stopRun()
sys.exit(0 if all(ok for _, ok in checks) else 1)
