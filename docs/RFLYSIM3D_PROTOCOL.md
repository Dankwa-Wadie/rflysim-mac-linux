# RflySim3D network protocol (as implemented on macOS)

The Windows RflySim3D is a closed Unreal Engine 4 build, and it is not on the
`RFLYExpPspSoftwarePack` ISO. Everything it does on the network is visible in the public
RflySimSDK Python (`UE4CtrlAPI.py`, `PX4MavCtrlV4.py`, `VisionCaptureApi.py`). The macOS
`rflysim3d` viewer (crate `crates/rflysim3d`) and the codecs in `crates/rfly-proto` implement
the formats below.

Every packet is a packed C struct with native alignment (Python `struct.pack` without a
prefix), little-endian. Receivers tell packets apart by **length**, then **checksum**.

## Ports

| Port | Direction | Content |
|---|---|---|
| UDP 20010 + windowID | → RflySim3D | vehicle states, commands, labels, sensor requests. Each extra window takes the next port |
| 224.0.0.10:20009 | → all RflySim3D on LAN | `i92s` commands (checksum 1234567897) and the same packets |
| UDP 20006 (+ multicast 224.0.0.10:20006) | RflySim3D → clients | crash notices, `RflyReqObjData` replies |
| 224.0.0.10:20007 | client ↔ client | start signal `3i {1234567890, isStart, copterID}` |
| UDP 20100+2(id−1) / +1 | Python → PX4 / PX4 → Python | MAVLink (`PX4MavCtrler`) |
| UDP 30100+2(id−1) / +1 | → CopterSim / CopterSim → Python | true state `SOut2Simulator` |
| UDP 9999 + SeqID (default) | RflySim3D → client | camera images |

## Packets into RflySim3D

| Size | Format | Checksum | Meaning |
|---|---|---|---|
| 56 / 256 | `i52s` / `i252s` | 1234567890 | console command |
| 40 | `3i7f` | 1234567890 (…91 = stick to ground) | `SOut2SimulatorSimple`: id, type, rpmMean, PosE[3], Euler[3] |
| 52 | `3i10f` | same | + Scale[3] |
| 88 | `3i17f1d` | 1234567890 | `SOut2SimulatorSimpleTime` (old): PWMs[8], PosE, VelE, Euler, time |
| 112 | `6i14f4d` | 1234567891 | `SOut2SimulatorSimpleTime`: GPS int[3], RPM[8], VelE, Euler, PosE (double), time. **CopterSim streams this one** |
| 168 | `4i24f7d` | 123456789 | `SOut2Simulator` (full state) |
| 152 | `6i2f120s` | 1234567899 | label: id, dispFlag, RGB, dispTime, fontSize, text |
| 304 | `i25i25i25i` | 1234567892 | attach vehicles (type 0 detach, 1 pos, 2 pos+yaw, 3 pos+att) |
| 144 | `2i1d16d` | 1234567894 | `Ue4ExtMsg` (custom actuators, accepted and ignored) |
| 144 / 92 | `16H28f` / `16H15f` | u16 12345 | `VisionSensorReq` (new / old) |
| 148 | `2H1I14H28f` | u16 12345 | `VisionSensorReqNew` (bitmask form) |
| 3804 | `i200H600f300H100f` | 1234567890/1 | 100 vehicles at once (scale ×100 in u16) |
| 1324 | `i40H120f60H160f` | 1234567890/1 | 20 vehicles with 8 PWMs each |

Positions are NED metres relative to the map origin; angles are radians (roll, pitch, yaw).

## Commands supported by the macOS viewer

`RflyShowTextTime txt t`, `RflyShowText txt`, `RflyChangeMapbyName name`, `RflyChangeMapbyID n`,
`RflyChangeViewKeyCmd key n` (V view, B vehicle, M map, P crash, T trails, H HUD),
`RflyCameraPosAng x y z r p y` / `RflyCameraPosAngAdd` (m, degrees), `RflyCameraFovDegrees`,
`RflyChange3DModel id type`, `RflyChangeVehicleSize id s`, `RflySetVehiclePosAng` /
`RflyMoveVehiclePosAng id fit x y z r p y` (degrees), `RflyClearCapture`,
`RflyReqObjData op name flag` (op 0 camera, 1 vehicle, 2 scene object; replies at 10 Hz),
`RflyCesiumOriPos lat lon alt`, `RflyDelVehicles [ids]`.
`t.MaxFPS`, `r.setres` and `RflyScanTerrainH` are accepted and have no effect.

Maps: `Grasslands` (default), `City`, `Blank`, `VisionRingBlank`, `Desert`. These are
procedural stand-ins: the UE4 map assets are not redistributable and are not on the ISO.

## Vision sensors

`VisionSensorReq` fields: SeqID, TypeID (1 RGB, 2 depth, 3 grey, 4 segmentation),
TargetCopter, TargetMountType (0 body centre, 1 body bottom, 2 world-fixed, 3 position-only,
4 treated as 0), DataWidth/Height, DataCheckFreq (Hz), SendProtocol[8]
(`[0]` 1 JPEG, 2 raw, 3 PNG; 0 = Windows shared memory → served as JPEG; `[1..4]` IP;
`[5]` port, 0 → 9999+SeqID), CameraFOV (horizontal, degrees), SensorPosXYZ (m, body FRD),
EularOrQuat, SensorAngEular (rad), SensorAngQuat (w,x,y,z).

Frames are split into ≤ 60000-byte chunks, each with header `4i1d`
`{1234567890, packet length incl. header, packet index, packet count, timestamp}`.
Depth is uint16 millimetres (16-bit PNG, or raw little-endian in mode 2); 65535 = no return.
Raw RGB is BGR (OpenCV order).

**macOS note:** a UDP socket on macOS cannot send a datagram larger than its send buffer
(9216 B by default). Senders must raise `SO_SNDBUF` (the viewer does), and Python senders of
large datagrams must do the same.

## RflySim3D → client replies

| Size | Format | Meaning |
|---|---|---|
| 12 | `3i` {1234567890, CopterID, TargetID} | crash (TargetID −1 scene object, other id = vehicle). Only sent while P (crash detection) is on |
| 80 | `2i16f1d` | `CoptReqData`: PosUE, Euler, quat, box origin, half extents, time |
| 72 | `5i11f1d` | `CameraData` |
| 112 | `2i16f1d32s` | `ObjReqData` |

Assumption: `PosUE` is reported in NED metres; UE4 RflySim3D may report UE centimetres.
