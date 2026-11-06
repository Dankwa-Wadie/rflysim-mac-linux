"""RflySim-style experiment on macOS: offboard flight + RflySim3D camera.

Start the platform first:   scripts/SITLRun.sh 1
Then:                       scripts/Python38Env.sh python/examples/offboard_camera.py

It is written against the RflySimSDK API (PX4MavCtrlV4 / VisionCaptureApi), so the same
code runs on Windows RflySim. Exit code 0 means the flight and image checks passed.
"""
import os, sys, time

import PX4MavCtrlV4 as PX4MavCtrl
import VisionCaptureApi

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "logs", "offboard_camera")
os.makedirs(OUT, exist_ok=True)

vis = VisionCaptureApi.VisionCaptureApi()
vis.jsonLoad(jsonPath=os.path.join(os.path.dirname(os.path.abspath(__file__)), "Config.json"))
vis.sendReqToUE4()
vis.startImgCap()
vis.sendUE4Cmd("RflyChangeMapbyName Grasslands")
vis.sendUE4Cmd("RflyShowTextTime Python-offboard-demo 5")

mav = PX4MavCtrl.PX4MavCtrler(1)
mav.InitMavLoop()
mav.InitTrueDataLoop()
t0 = time.time()
while not (mav.hasTrueDataRec and mav.uavPosGPSHome[0] != 0) and time.time() - t0 < 30:
    time.sleep(0.5)
print("connected: home", mav.uavPosGPSHome, " true pos", mav.truePosNED)

mav.initOffboard()
mav.SendPosNED(0, 0, -5, 0)
time.sleep(10)
ok_alt = abs(mav.truePosNED[2] + 5) < 0.6
print("hover: PX4 NED", [round(v, 2) for v in mav.uavPosNED], " true NED", [round(v, 2) for v in mav.truePosNED], "armed", mav.isArmed)
mav.sendUE4LabelMsg(1, "offboard hold 5 m", 20, [0, 255, 0], 0, -1)
mav.SendPosNED(10, 5, -8, 1.0)
time.sleep(10)
ok_move = abs(mav.truePosNED[0] - 10) < 1.0 and abs(mav.truePosNED[1] - 5) < 1.0
print("moved: true NED", [round(v, 2) for v in mav.truePosNED], " yaw", round(mav.trueAngEular[2], 2))

ok_img = False
try:
    import cv2
    if vis.hasData[0]:
        cv2.imwrite(os.path.join(OUT, "fpv.png"), vis.Img[0])
        ok_img = vis.Img[0].shape == (480, 640, 3)
    if len(vis.hasData) > 1 and vis.hasData[1]:
        d = vis.Img[1]
        print("depth image", d.shape, d.dtype, "centre", d[d.shape[0] // 2, d.shape[1] // 2], "mm")
        cv2.imwrite(os.path.join(OUT, "depth.png"), d)
except ImportError:
    pass

mav.SendMavCmdLong(PX4MavCtrl.mavlink2.MAV_CMD_NAV_LAND, 0, 0, 0, 0, float("nan"), float("nan"), 0)
mav.endOffboard()
time.sleep(12)
ok_land = mav.truePosNED[2] > -0.3
print("landed: true z", round(mav.truePosNED[2], 2))
mav.stopRun()
vis.stopRun()

for name, ok in [("hover 5 m", ok_alt), ("move to (10,5,-8)", ok_move), ("camera image 640x480", ok_img), ("landed", ok_land)]:
    print(("PASS " if ok else "FAIL ") + name)
sys.exit(0 if ok_alt and ok_move and ok_img and ok_land else 1)
