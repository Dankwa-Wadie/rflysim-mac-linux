"""macOS RflySimSDK: VisionCaptureApi, camera images from RflySim3D.

API-compatible with RflySimSDK's VisionCaptureApi for the UDP transports (macOS has no
RflySim shared memory: SendProtocol[0]=0 is switched to 1 = UDP JPEG automatically).

    vis = VisionCaptureApi()
    vis.jsonLoad()          # Config.json next to your script (RflySim format)
    vis.sendReqToUE4()
    vis.startImgCap()
    if vis.hasData[0]: img = vis.Img[0]   # numpy array (BGR / uint16 depth / grey)
"""
import json, os, re, socket, struct, sys, threading, time

try:
    import numpy as np
    import cv2
except ImportError:  # image decoding needs numpy + opencv (installed by Python38Env)
    np = cv2 = None

from UE4CtrlAPI import UE4CtrlAPI


class VisionSensorReq:
    def __init__(self):
        self.checksum, self.SeqID, self.TypeID, self.TargetCopter, self.TargetMountType = 12345, 0, 1, 1, 0
        self.DataWidth, self.DataHeight, self.DataCheckFreq = 640, 480, 30
        self.SendProtocol = [1, 127, 0, 0, 1, 0, 0, 0]
        self.CameraFOV, self.EularOrQuat = 90, 0
        self.SensorPosXYZ, self.SensorAngEular, self.SensorAngQuat = [0, 0, 0], [0, 0, 0], [0, 0, 0, 0]
        self.otherParams = [0] * 16


class VisionCaptureApi(UE4CtrlAPI):
    def __init__(self, ip="127.0.0.1"):
        super().__init__(ip)
        self.VisSensor, self.Img, self.hasData, self.timeStmp, self.ImgData = [], [], [], [], []
        self.stopFlag = False
        self.RemotSendIP = ""
        self.isNewJson = True

    def addVisSensor(self, vsr=None):
        self.VisSensor.append(vsr or VisionSensorReq())

    def jsonLoad(self, ChangeMode=-1, jsonPath=""):
        if not os.path.isabs(jsonPath):
            base = os.path.dirname(os.path.realpath(sys.argv[0]))
            jsonPath = os.path.join(base, jsonPath or "Config.json")
        if not os.path.exists(jsonPath):
            print("The json file does not exist!", jsonPath)
            return False
        text = "\n".join(re.sub(r"//.*|#.*", "", l) for l in open(jsonPath, encoding="utf-8"))
        cur = -1
        for d in json.loads(text)["VisionSensors"]:
            v = VisionSensorReq()
            v.SeqID = d.get("SeqID", 0)
            if v.SeqID == 0 and cur >= 0:
                v.SeqID = cur + 1
            cur = v.SeqID
            for k in ("TypeID", "TargetCopter", "TargetMountType", "DataWidth", "DataHeight", "DataCheckFreq", "CameraFOV", "EularOrQuat"):
                if k in d:
                    setattr(v, k, d[k])
            v.SendProtocol = list(d.get("SendProtocol", [0] * 8))
            v.SensorPosXYZ = list(d.get("SensorPosXYZ", [0, 0, 0]))
            v.SensorAngEular = list(d.get("SensorAngEular", [0, 0, 0]))
            v.SensorAngQuat = list(d.get("SensorAngQuat", [0, 0, 0, 0]))
            v.otherParams = (list(d.get("otherParams", [])) + [0] * 16)[:16]
            if ChangeMode >= 0:
                v.SendProtocol[0] = ChangeMode
            if v.SendProtocol[0] == 0:
                v.SendProtocol[0] = 1  # no shared memory on macOS → UDP JPEG
            if v.SendProtocol[5] == 0:
                v.SendProtocol[5] = 9999 + v.SeqID
            self.VisSensor.append(v)
        print("Got", len(self.VisSensor), "vision sensors from json")
        return True

    def sendReqToUE4(self, windID=0, IP=""):
        IP = IP or self.ip
        if not self.VisSensor:
            print("Error: No sensor is obtained.")
            return False
        if self.RemotSendIP:
            parts = [int(x) for x in self.RemotSendIP.split(".")]
            for v in self.VisSensor:
                v.SendProtocol[1:5] = parts
        if any(v.SeqID == 0 for v in self.VisSensor):
            self.sendUE4Cmd("RflyClearCapture", windID)
        for v in self.VisSensor:
            ints = [v.checksum, v.SeqID, v.TypeID, v.TargetCopter, v.TargetMountType, v.DataWidth, v.DataHeight, v.DataCheckFreq] + v.SendProtocol
            flo = [v.CameraFOV] + v.SensorPosXYZ + [v.EularOrQuat] + v.SensorAngEular + v.SensorAngQuat + v.otherParams
            self.udp_socket.sendto(struct.pack("16H28f", *ints, *flo), (IP, 20010 + windID))
        time.sleep(0.5)
        return True

    def startImgCap(self, isRemoteSend=False):
        for i, v in enumerate(self.VisSensor):
            self.Img.append(None); self.hasData.append(False); self.timeStmp.append(0); self.ImgData.append(0)
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 60000 * 100)
            s.bind(("0.0.0.0", v.SendProtocol[5]))
            s.settimeout(0.5)
            threading.Thread(target=self._img_loop, args=(s, i), daemon=True).start()

    def _img_loop(self, sock, idx):
        parts, total, stamp = {}, 0, None
        v = self.VisSensor[idx]
        while not self.stopFlag:
            try:
                buf, _ = sock.recvfrom(62000)
            except socket.timeout:
                continue
            if len(buf) < 24:
                continue
            ck, ln, seq, tot, ts = struct.unpack("4i1d", buf[:24])
            if ck != 1234567890 or ln != len(buf):
                continue
            if ts != stamp or tot != total:
                parts, total, stamp = {}, tot, ts
            parts[seq] = buf[24:]
            if len(parts) < total:
                continue
            data = b"".join(parts[k] for k in range(total))
            parts = {}
            if np is None:
                self.Img[idx] = data
            elif v.SendProtocol[0] == 2:
                if v.TypeID == 2:
                    self.Img[idx] = np.frombuffer(data, np.uint16).reshape(v.DataHeight, v.DataWidth)
                elif v.TypeID == 3:
                    self.Img[idx] = np.frombuffer(data, np.uint8).reshape(v.DataHeight, v.DataWidth)
                else:
                    self.Img[idx] = np.frombuffer(data, np.uint8).reshape(v.DataHeight, v.DataWidth, 3)
            else:
                flag = cv2.IMREAD_ANYDEPTH if v.TypeID == 2 else cv2.IMREAD_GRAYSCALE if v.TypeID == 3 else cv2.IMREAD_COLOR
                self.Img[idx] = cv2.imdecode(np.frombuffer(data, np.uint8), flag)
            self.hasData[idx] = self.Img[idx] is not None
            self.timeStmp[idx] = ts

    def stopRun(self):
        self.stopFlag = True
