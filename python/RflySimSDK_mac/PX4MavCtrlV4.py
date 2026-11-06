"""macOS RflySimSDK: PX4MavCtrlV4, MAVLink control of PX4 through the RflySim port layout.

API-compatible subset of RflySimSDK's PX4MavCtrler (MAVLink_Full mode):
  * receives PX4 MAVLink on UDP 20101+2(id-1) and sends to 20100+2(id-1)
    (scripts/SITLRun.sh opens these on PX4; coptersim relays them in HITL),
  * true simulator state from coptersim on 30101+2(id-1) (InitTrueDataLoop),
  * offboard helpers (initOffboard, SendPosNED, SendVelNED, SendVelFRD, ...).

    import PX4MavCtrlV4 as PX4MavCtrl
    mav = PX4MavCtrl.PX4MavCtrler(1)
    mav.InitMavLoop(); mav.InitTrueDataLoop()
    mav.initOffboard(); mav.SendPosNED(0, 0, -5)
"""
import math, socket, struct, threading, time

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

from UE4CtrlAPI import UE4CtrlAPI


class PX4_CUSTOM_MAIN_MODE:
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1
    PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
    PX4_CUSTOM_MAIN_MODE_POSCTL = 3
    PX4_CUSTOM_MAIN_MODE_AUTO = 4
    PX4_CUSTOM_MAIN_MODE_ACRO = 5
    PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
    PX4_CUSTOM_MAIN_MODE_STABILIZED = 7


class PX4_CUSTOM_SUB_MODE_AUTO:
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
    PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6


class PX4MavCtrler(UE4CtrlAPI):
    def __init__(self, ID=1, ip="127.0.0.1", Com="udp", port=0):
        super().__init__(ip)
        if ID > 10000:  # legacy: port number given
            port, ID = ID, int((ID - 20100) / 2) + 1
        self.CopterID = ID
        self.port = port if port > 0 else 20100 + 2 * (ID - 1)
        self.isCom = Com.lower() != "udp"
        self.ComName = Com
        self.startTime = time.time()
        self.mav0 = mavlink2.MAVLink(None, srcSystem=255, srcComponent=190)
        self.stopFlag = False
        self.isInOffboard = False
        self.isArmed = 0
        self.uavTimeStmp = 0
        self.uavPosNED, self.uavVelNED, self.uavAngEular, self.uavAngRate = [0] * 3, [0] * 3, [0] * 3, [0] * 3
        self.uavAngQuatern, self.uavPosGPS, self.uavPosGPSHome, self.uavGlobalPos = [1, 0, 0, 0], [0] * 9, [0] * 3, [0] * 3
        self.uavAccB, self.uavGyro, self.uavMag, self.uavMotorRPMS = [0] * 3, [0] * 3, [0] * 3, [0] * 8
        self.batInfo, self.uavMsg, self.system_status = [0, 0], "", 0
        self.truePosNED, self.trueVelNED, self.trueAngEular, self.trueAngRate = [0] * 3, [0] * 3, [0] * 3, [0] * 3
        self.trueAngQuatern, self.trueMotorRPMS, self.trueAccB, self.truePosGPS = [1, 0, 0, 0], [0] * 8, [0] * 3, [0] * 3
        self.trueTimeStmp, self.hasTrueDataRec = 0, False
        self.pos, self.vel, self.acc, self.yaw, self.yawrate = [0] * 3, [0] * 3, [0] * 3, 0, 0
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.type_mask = self.TypeMask([0, 1, 0, 0, 0, 1])
        self.target_system, self.target_component = ID, 1

    # ---------------------------------------------------------------- links
    def InitMavLoop(self, UDPMode=2):
        self.UDPMode = UDPMode
        if self.isCom:
            self.the_connection = mavutil.mavlink_connection(self.ComName, 921600)
        else:
            self.the_connection = mavutil.mavlink_connection("udpin:0.0.0.0:%d" % (self.port + 1))
        self.t1 = threading.Thread(target=self.getMavMsg, daemon=True)
        self.t1.start()
        self.t2 = threading.Thread(target=self.OffboardSendMode, daemon=True)

    def SendBuf(self, buf):
        if self.isCom:
            self.the_connection.write(buf)
        else:
            self.udp_socket.sendto(buf, (self.ip, self.port))

    def getMavMsg(self):
        while not self.stopFlag:
            msg = self.the_connection.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                self.target_system = msg.get_srcSystem()
                self.isArmed = 1 if msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED else 0
                self.system_status = msg.system_status
            elif t == "LOCAL_POSITION_NED":
                self.uavTimeStmp = msg.time_boot_ms / 1000
                self.uavPosNED = [msg.x, msg.y, msg.z]
                self.uavVelNED = [msg.vx, msg.vy, msg.vz]
            elif t == "ATTITUDE":
                self.uavAngEular = [msg.roll, msg.pitch, msg.yaw]
                self.uavAngRate = [msg.rollspeed, msg.pitchspeed, msg.yawspeed]
            elif t == "ATTITUDE_QUATERNION":
                self.uavAngQuatern = [msg.q1, msg.q2, msg.q3, msg.q4]
            elif t == "GLOBAL_POSITION_INT":
                self.uavPosGPS = [msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1e3, msg.time_boot_ms, msg.relative_alt / 1e3, msg.vx / 100, msg.vy / 100, msg.vz / 100, msg.hdg / 100]
            elif t == "HOME_POSITION":
                self.uavPosGPSHome = [msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1e3]
            elif t == "HIGHRES_IMU":
                self.uavAccB, self.uavGyro, self.uavMag = [msg.xacc, msg.yacc, msg.zacc], [msg.xgyro, msg.ygyro, msg.zgyro], [msg.xmag, msg.ymag, msg.zmag]
            elif t == "BATTERY_STATUS":
                self.batInfo = [msg.voltages[0] / 1000, msg.battery_remaining]
            elif t == "STATUSTEXT":
                self.uavMsg = msg.text

    def InitTrueDataLoop(self):
        self.udp_socketTrue = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socketTrue.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socketTrue.bind(("0.0.0.0", self.port + 1 + 10000))
        self.udp_socketTrue.settimeout(0.5)
        self.stopFlagTrueData = False
        self.tTrue = threading.Thread(target=self.getTrueDataMsg, daemon=True)
        self.tTrue.start()

    def EndTrueDataLoop(self):
        self.stopFlagTrueData = True

    def getTrueDataMsg(self):
        while not self.stopFlagTrueData:
            try:
                buf, _ = self.udp_socketTrue.recvfrom(65500)
            except socket.timeout:
                continue
            if len(buf) == 168:
                u = struct.unpack("4i24f7d", buf)
                if u[0] == 123456789:
                    self.trueVelNED, self.trueAngEular, self.trueAngQuatern = list(u[4:7]), list(u[7:10]), list(u[10:14])
                    self.trueMotorRPMS, self.trueAccB, self.trueAngRate = list(u[14:22]), list(u[22:25]), list(u[25:28])
                    self.trueTimeStmp, self.truePosNED, self.truePosGPS = u[28], list(u[29:32]), list(u[32:35])
                    self.hasTrueDataRec = True
            elif len(buf) == 12:
                ck, cid, tid = struct.unpack("iii", buf)
                if ck == 1234567890 and tid > -0.5:
                    self.isVehicleCrash, self.isVehicleCrashID = True, tid

    def stopRun(self):
        self.endOffboard()
        self.stopFlag = True
        self.stopFlagTrueData = True
        time.sleep(0.3)

    endMavLoop = stopRun

    # ---------------------------------------------------------------- commands
    def SendMavCmdLong(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        buf = self.mav0.command_long_encode(self.target_system, self.target_component, command, 0, param1, param2, param3, param4, param5, param6, param7).pack(self.mav0)
        self.SendBuf(buf)

    def SendMavArm(self, isArm=1):
        if isArm:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        else:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196.0)

    def SendSetMode(self, mainmode, cusmode=0):
        self.SendMavCmdLong(mavlink2.MAV_CMD_DO_SET_MODE, mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mainmode, cusmode)

    def sendMavSetParam(self, param_id, param_value, param_type):
        if isinstance(param_id, str):
            param_id = param_id.encode()
        buf = self.mav0.param_set_encode(self.target_system, self.target_component, param_id, param_value, param_type).pack(self.mav0)
        self.SendBuf(buf)

    def sendMavTakeOff(self, xM=0, yM=0, zM=0, YawRad=0, PitchRad=0):
        """Takeoff to local NED point (zM < 0 is up), via PX4's takeoff mode."""
        lat, lon, alt = self.uavPosGPSHome
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_TAKEOFF, PitchRad, 0, 0, YawRad, lat, lon, alt - zM)

    def sendMavLand(self, xM=0, yM=0, zM=0):
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_LAND, 0, 0, 0, 0, math.nan, math.nan, 0)

    # ---------------------------------------------------------------- offboard
    def TypeMask(self, EnList):
        # EnList: [pos, vel, acc, force, yaw, yawrate]; returns the IGNORE bitmask.
        m = 0
        if not EnList[0]: m |= 0b111
        if not EnList[1]: m |= 0b111000
        if not EnList[2]: m |= 0b111000000
        if EnList[3]: m |= 0b1000000000
        if not EnList[4]: m |= 0b10000000000
        if not EnList[5]: m |= 0b100000000000
        return m

    def sendMavOffboardAPI(self, type_mask=0, coordinate_frame=1, pos=[0, 0, 0], vel=[0, 0, 0], acc=[0, 0, 0], yaw=0, yawrate=0):
        t = int((time.time() - self.startTime) * 1000)
        buf = self.mav0.set_position_target_local_ned_encode(t, self.target_system, self.target_component, coordinate_frame, type_mask, *pos, *vel, *acc, yaw, yawrate).pack(self.mav0)
        self.SendBuf(buf)

    def OffboardSendMode(self):
        while self.isInOffboard and not self.stopFlag:
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw, self.yawrate)
            time.sleep(1 / 30)

    def initOffboard(self):
        self.type_mask = self.TypeMask([0, 1, 0, 0, 0, 1])
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos, self.vel, self.acc, self.yaw, self.yawrate = [0] * 3, [0] * 3, [0] * 3, 0, 0
        self.isInOffboard = True
        if not self.t2.is_alive():
            self.t2 = threading.Thread(target=self.OffboardSendMode, daemon=True)
            self.t2.start()
        self.sendMavSetParam("NAV_RCL_ACT", 0, mavlink2.MAV_PARAM_TYPE_INT32)
        self.sendMavSetParam("NAV_DLL_ACT", 0, mavlink2.MAV_PARAM_TYPE_INT32)
        time.sleep(0.3)
        if not self.isArmed:
            self.SendMavArm(1)
        time.sleep(0.1)
        self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        time.sleep(0.1)
        self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)

    def endOffboard(self):
        self.isInOffboard = False

    def _set(self, en, frame, pos=None, vel=None, yaw=0, yawrate=0):
        self.type_mask = self.TypeMask(en)
        self.coordinate_frame = frame
        self.pos = pos or [0, 0, 0]
        self.vel = vel or [0, 0, 0]
        self.acc = [0, 0, 0]
        self.yaw, self.yawrate = yaw, yawrate

    def SendPosNED(self, x=math.nan, y=math.nan, z=math.nan, yaw=math.nan):
        self._set([1, 0, 0, 0, 1, 0], mavlink2.MAV_FRAME_LOCAL_NED, pos=[x, y, z], yaw=yaw)

    def SendPosNEDNoYaw(self, x=math.nan, y=math.nan, z=math.nan):
        self._set([1, 0, 0, 0, 0, 0], mavlink2.MAV_FRAME_LOCAL_NED, pos=[x, y, z])

    def SendVelNED(self, vx=math.nan, vy=math.nan, vz=math.nan, yawrate=math.nan):
        self._set([0, 1, 0, 0, 0, 1], mavlink2.MAV_FRAME_LOCAL_NED, vel=[vx, vy, vz], yawrate=yawrate)

    def SendVelNEDNoYaw(self, vx, vy, vz):
        self._set([0, 1, 0, 0, 0, 0], mavlink2.MAV_FRAME_LOCAL_NED, vel=[vx, vy, vz])

    def SendVelFRD(self, vx=math.nan, vy=math.nan, vz=math.nan, yawrate=math.nan):
        self._set([0, 1, 0, 0, 0, 1], mavlink2.MAV_FRAME_BODY_NED, vel=[vx, vy, vz], yawrate=yawrate)

    def SendPosFRD(self, x=math.nan, y=math.nan, z=math.nan, yaw=math.nan):
        self._set([1, 0, 0, 0, 1, 0], mavlink2.MAV_FRAME_BODY_NED, pos=[x, y, z], yaw=yaw)
