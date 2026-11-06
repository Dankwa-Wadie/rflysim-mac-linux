"""macOS RflySimSDK: UE4CtrlAPI, the RflySim3D control interface.

API-compatible re-implementation (same class, method names and packet formats as
RflySimSDK's UE4CtrlAPI.py) that talks to the macOS `rflysim3d` viewer, or to a Windows
RflySim3D on the LAN. Pure stdlib.
"""
import socket, struct, threading, time


class CoptReqData:
    def __init__(self, iv=None):
        self.checksum, self.CopterID = 0, 0
        self.PosUE, self.angEuler, self.angQuat = [0] * 3, [0] * 3, [0] * 4
        self.boxOrigin, self.BoxExtent, self.timestmp = [0] * 3, [0] * 3, 0
        if iv:
            self.CopyData(iv)

    def CopyData(self, iv):
        self.checksum, self.CopterID = iv[0], iv[1]
        self.PosUE, self.angEuler, self.angQuat = list(iv[2:5]), list(iv[5:8]), list(iv[8:12])
        self.boxOrigin, self.BoxExtent, self.timestmp = list(iv[12:15]), list(iv[15:18]), iv[18]


class ObjReqData(CoptReqData):
    def CopyData(self, iv):
        super().CopyData(iv)
        self.seqID = iv[1]
        self.ObjName = iv[19].split(b"\x00")[0].decode(errors="ignore")


class CameraData:
    def __init__(self, iv=None):
        self.checksum = self.SeqID = self.TypeID = self.DataHeight = self.DataWidth = 0
        self.CameraFOV, self.PosUE, self.angEuler, self.angQuat, self.timestmp = 0, [0] * 3, [0] * 3, [0] * 4, 0
        if iv:
            self.CopyData(iv)

    def CopyData(self, iv):
        (self.checksum, self.SeqID, self.TypeID, self.DataHeight, self.DataWidth, self.CameraFOV) = iv[:6]
        self.PosUE, self.angEuler, self.angQuat, self.timestmp = list(iv[6:9]), list(iv[9:12]), list(iv[12:16]), iv[16]


class UE4CtrlAPI:
    def __init__(self, ip="127.0.0.1"):
        self.ip = ip
        self.startTime = time.time()
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.stopFlagUE4 = True
        self.CoptDataVect, self.ObjDataVect, self.CamDataVect = [], [], []
        self.isVehicleCrash, self.isVehicleCrashID, self.CopterID = False, -10, 1

    # ---------------------------------------------------------------- sending
    def _send(self, buf, windowID=-1):
        if windowID < 0:
            for i in range(3):
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))
        else:
            self.udp_socket.sendto(buf, (self.ip, 20010 + windowID))

    def fillList(self, data, inLen, fill=0):
        data = list(data) if isinstance(data, (list, tuple)) else [data]
        return (data + [fill] * inLen)[:inLen]

    def sendUE4Cmd(self, cmd, windowID=-1):
        """RflySim3D console command, e.g. 'RflyChangeMapbyName Grasslands'."""
        if isinstance(cmd, str):
            cmd = cmd.encode()
        if len(cmd) <= 51:
            buf = struct.pack("i52s", 1234567890, cmd)
        elif len(cmd) <= 249:
            buf = struct.pack("i252s", 1234567890, cmd)
        else:
            print("Error: Cmd is too long")
            return
        self._send(buf, windowID)

    def sendUE4CmdNet(self, cmd):
        if isinstance(cmd, str):
            cmd = cmd.encode()
        self.udp_socket.sendto(struct.pack("i92s", 1234567897, cmd), ("224.0.0.10", 20009))

    def sendUE4LabelID(self, CopterID=0, Txt="", fontSize=30, RGB=[255, 0, 0], windowID=-1):
        self.sendUE4LabelMsg(CopterID, Txt, fontSize, RGB, 1, 0, windowID)

    def sendUE4LabelMsg(self, CopterID=0, Txt="", fontSize=30, RGB=[255, 0, 0], dispTime=0, dispFlag=-1, windowID=-1):
        buf = struct.pack("6i2f120s", 1234567899, CopterID, dispFlag, *RGB, dispTime, fontSize, Txt.encode("UTF-8"))
        self._send(buf, windowID)

    def sendUE4Attatch(self, CopterIDs, AttatchIDs, AttatchTypes, windowID=-1):
        f = lambda v: self.fillList(v, 25)
        self._send(struct.pack("i25i25i25i", 1234567892, *f(CopterIDs), *f(AttatchIDs), *f(AttatchTypes)), windowID)

    def sendUE4Pos(self, copterID=1, vehicleType=3, MotorRPMSMean=0, PosE=[0, 0, 0], AngEuler=[0, 0, 0], windowID=-1):
        self.sendUE4PosNew(copterID, vehicleType, PosE, AngEuler, [0, 0, 0], [MotorRPMSMean] * 8, -1, windowID)

    def sendUE4Pos2Ground(self, copterID=1, vehicleType=3, MotorRPMSMean=0, PosE=[0, 0, 0], AngEuler=[0, 0, 0], windowID=-1):
        self._send(struct.pack("3i7f", 1234567891, copterID, vehicleType, MotorRPMSMean, *self.fillList(PosE, 3), *self.fillList(AngEuler, 3)), windowID)

    def sendUE4PosScale(self, copterID=1, vehicleType=3, MotorRPMSMean=0, PosE=[0, 0, 0], AngEuler=[0, 0, 0], Scale=[1, 1, 1], windowID=-1):
        self._send(struct.pack("3i10f", 1234567890, copterID, vehicleType, MotorRPMSMean, *self.fillList(PosE, 3), *self.fillList(AngEuler, 3), *self.fillList(Scale, 3)), windowID)

    def sendUE4PosScale2Ground(self, copterID=1, vehicleType=3, MotorRPMSMean=0, PosE=[0, 0, 0], AngEuler=[0, 0, 0], Scale=[1, 1, 1], windowID=-1):
        self._send(struct.pack("3i10f", 1234567891, copterID, vehicleType, MotorRPMSMean, *self.fillList(PosE, 3), *self.fillList(AngEuler, 3), *self.fillList(Scale, 3)), windowID)

    def sendUE4PosFull(self, copterID, vehicleType, MotorRPMS=[0] * 8, VelE=[0, 0, 0], PosE=[0, 0, 0], RateB=[0, 0, 0], AngEuler=[0, 0, 0], windowID=-1):
        buf = struct.pack("4i24f7d", 123456789, copterID, vehicleType, 0, *self.fillList(VelE, 3), *self.fillList(AngEuler, 3), 0, 0, 0, 0,
                          *self.fillList(MotorRPMS, 8), 0, 0, 0, *self.fillList(RateB, 3), -1, *self.fillList(PosE, 3), 0, 0, 0)
        self._send(buf, windowID)

    def sendUE4ExtAct(self, copterID=1, ActExt=[0] * 16, windowID=-1):
        self._send(struct.pack("2i1d16d", 1234567894, copterID, time.time() - self.startTime, *self.fillList(ActExt, 16)), windowID)

    def sendUE4PosSimple(self, copterID, vehicleType, PWMs, VelE, PosE, AngEuler, runnedTime=-1, windowID=-1):
        f = self.fillList
        self._send(struct.pack("3i17f1d", 1234567890, copterID, vehicleType, *f(PWMs, 8), *f(PosE, 3), *f(VelE, 3), *f(AngEuler, 3), runnedTime), windowID)

    def sendUE4PosNew(self, copterID=1, vehicleType=3, PosE=[0, 0, 0], AngEuler=[0, 0, 0], VelE=[0, 0, 0], PWMs=[0] * 8, runnedTime=-1, windowID=-1):
        f = self.fillList
        self._send(struct.pack("6i14f4d", 1234567891, copterID, vehicleType, 0, 0, 0, *f(PWMs, 8), *f(VelE, 3), *f(AngEuler, 3), *f(PosE, 3), runnedTime), windowID)

    def sendUE4PosScale100(self, copterID, vehicleType, PosE, AngEuler, MotorRPMSMean, Scale, isFitGround=False, windowID=-1):
        f = self.fillList
        sc = [int(s * 100) for s in f(Scale, 300)]
        self._send(struct.pack("i200H600f300H100f", 1234567891 if isFitGround else 1234567890, *f(copterID, 100), *f(vehicleType, 100), *f(PosE, 300), *f(AngEuler, 300), *sc, *f(MotorRPMSMean, 100)), windowID)

    # ---------------------------------------------------------------- receiving
    def initUE4MsgRec(self):
        """Listen on 20006 (+ multicast 224.0.0.10) for crash / request replies."""
        self.stopFlagUE4 = False
        self.udp_socketUE4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socketUE4.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            self.udp_socketUE4.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.udp_socketUE4.bind(("0.0.0.0", 20006))
        try:
            self.udp_socketUE4.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton("224.0.0.10") + socket.inet_aton("0.0.0.0"))
        except OSError:
            print("Failed to Init multicast!")
        self.udp_socketUE4.settimeout(0.5)
        self.t4 = threading.Thread(target=self.UE4MsgRecLoop, daemon=True)
        self.t4.start()

    def endUE4MsgRec(self):
        self.stopFlagUE4 = True
        self.t4.join()
        self.udp_socketUE4.close()

    def _upsert(self, vect, key, iv, cls):
        for o in vect:
            if getattr(o, key) == iv[1]:
                o.CopyData(iv)
                return
        vect.append(cls(iv))

    def UE4MsgRecLoop(self):
        while not self.stopFlagUE4:
            try:
                buf, _ = self.udp_socketUE4.recvfrom(65500)
            except socket.timeout:
                continue
            except OSError:
                break
            n = len(buf)
            if n == 12:
                ck, cid, tid = struct.unpack("iii", buf)
                if ck == 1234567890:
                    if tid > -0.5 and cid == self.CopterID:
                        self.isVehicleCrash, self.isVehicleCrashID = True, tid
                    print("Vehicle #", cid, " Crashed with vehicle #", tid)
            elif n == 80:
                iv = struct.unpack("2i16f1d", buf)
                if iv[0] == 1234567891:
                    self._upsert(self.CoptDataVect, "CopterID", iv, CoptReqData)
            elif n == 72:
                iv = struct.unpack("5i11f1d", buf)
                if iv[0] == 1234567891:
                    self._upsert(self.CamDataVect, "SeqID", iv, CameraData)
            elif n == 112:
                iv = struct.unpack("2i16f1d32s", buf)
                if iv[0] == 1234567891:
                    for o in self.ObjDataVect:
                        if o.seqID == iv[1]:
                            o.CopyData(iv)
                            break
                    else:
                        self.ObjDataVect.append(ObjReqData(iv))

    def reqCamCoptObj(self, type_id=1, objName=1, windowID=0):
        """type 0 camera (objName=SeqID), 1 vehicle (CopterID), 2 scene object (name)."""
        names = objName if isinstance(objName, list) else [objName]
        for v in names:
            self.sendUE4Cmd("RflyReqObjData %d %s %d" % (type_id, str(v), 0), windowID)
            time.sleep(0.05)

    def reqCam(self, SeqIDList=[0], windowID=0):
        self.reqCamCoptObj(0, SeqIDList, windowID)

    def reqCopt(self, CopterIDList=[0], windowID=0):
        self.reqCamCoptObj(1, CopterIDList, windowID)

    def reqObj(self, ObjNameList=[""], windowID=0):
        self.reqCamCoptObj(2, ObjNameList, windowID)

    def getCamCoptObj(self, type_id=1, objName=1):
        vect, key = {0: (self.CamDataVect, "SeqID"), 1: (self.CoptDataVect, "CopterID"), 2: (self.ObjDataVect, "ObjName")}[type_id]
        for o in vect:
            if getattr(o, key) == objName:
                return o
        return 0
