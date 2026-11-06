# RflySim software pack: compatibility notes

Source: RflySim's installer ISO `RFLYExpPspSoftwarePack.iso` (ISO 9660, 3.89 GB).
Method: a static inventory of the files (ISO/zip listings, the SQLite schema, UI strings and DLL
dependencies), to find what a compatible macOS/Linux version has to provide. No RflySim code or data is
included in this project.

Legend: **[O]** observed directly · **[I]** inference · **[?]** unknown

## 1. What is in the ISO

| Path | Size | What it is | Mac plan |
|---|---|---|---|
| `PX4InstallScript.p` | 30 KB | MATLAB P-code installer (Windows-only) [O] | Replace with a shell/cargo installer |
| `1.ToolChainZip/PX4Msys2Toolchain.zip` | 990 MB | MSYS2 GCC-ARM toolchain [O] | Not needed: use `brew` arm-none-eabi / PX4's macOS setup |
| `2.FirmwareZip/PX4Firmware1.7.3/1.8.2.zip` | ~360 MB each | Stock PX4 source [O] | Use upstream PX4 (builds natively on macOS) |
| `2.FirmwareZip/Python27.zip` | 33 MB | Python 2.7 for the build [O] | Not needed |
| `3.PX4PSP/PX4 PSP.zip` | 16 MB | MathWorks add-on `PX4PSP_v3_0_4_351_R2017b.mltbx` (Simulink→PX4 codegen) [O] | Out of scope (MATLAB-specific); keep it optional |
| `4.HILApps/CopterSim.zip` | 40 MB | **Core simulator**: Qt5 C++ x64 GUI [O] | **Rewrite in Rust** |
| `4.HILApps/3DDisplay.zip` | 179 MB | **3D viewer**: Ogre3D + Qt5 + MFC [O] | **Rewrite in Rust** (lightweight renderer) |
| `4.HILApps/QGroundControl.zip` | 144 MB | Stock QGC [O] | Use QGC's official macOS build |
| `4.HILApps/FlightGear 2016.1.2.zip` | 1.3 GB | Stock FlightGear [O] | Optional; FlightGear has a macOS build |
| `4.HILApps/eclipse.zip` | 412 MB | IDE [O] | Not needed |
| `4.HILApps/px4driver.msi`, `MSVCP_*.exe`, `getGUID.exe`, `GetMD5.mexw64` | small | Windows drivers/runtime/licensing helpers [O] | Not needed |

So the RflySim-specific work is **CopterSim** plus **3DDisplay**. Everything else either
already runs on macOS or depends on MATLAB.

## 2. CopterSim (`CopterSim.exe`, PE32+ GUI, 1.1 MB)

Imports: Qt5 Core/Gui/Widgets/Network/SerialPort/Sql, sqlite3, MSVC 2015+ CRT [O].

### Roles (from Qt slot names and log strings) [O]
- `Mavlink2px4Connector`, `SerialLink`, `onMavLinkMessage(mavlink_message_t)`: speaks MAVLink to PX4.
- **HIL mode**: connects to a Pixhawk over a COM port ("Close QGC or other programs to release the COM port"),
  sends reboot, waits for heartbeat.
- **SITL mode**: "Awaiting PX4 SITL to connect to TCP port", "Use UDP ports rec send". This is the standard
  PX4 simulator interface (MAVLink `HIL_SENSOR`/`HIL_GPS`/`HIL_ACTUATOR_CONTROLS`) [I, matches upstream PX4].
- **QGC bridge**: "UDP connected for QGC", "HIL QGC Port": forwards MAVLink to QGroundControl.
- **3D output**: UDP to 3DDisplay / RflySim3D (UE4); `UE4Req Port`, `..\RflySim3D\...\Binaries\Win64` [O].
- Timers: `modelSimTimer1ms` (1 kHz physics), `qTimer10msLoop`, `qTimerSilUdpmsLoop`, `mainSimLoop` [O].
- Simulation modes: `Mavlink_Full`, `Mavlink_Simple`, `UDP_Full`, `UDP_Simple`, `Simulink&DLL_SIL` [O].
- Pluggable dynamics: "Use DLL Model", `HLLDLL`, `GetProcAddress` [O]. A Simulink-generated
  model DLL can replace the built-in model. Its exported symbol names and struct layout are **[?]**.
- Multi-vehicle: `CopterID`, "Collide with vehicle #", "CopterSim instances simultaneously" [O].
- Status echoes from PX4: armed/disarmed, "GPS 3D fixed & EKF initialized", loss of manual/offboard signals [O].
- Licensing: the Windows build has its own licensing. **This project does not reproduce or bypass it.**

### Vehicle configurator (GUI)
Frame types: QUAD, HEXA, OCTO, OCTO_QUAD, TRI, Y6A [O]. The user picks motor, propeller, ESC and battery
from the catalog. The app then computes hover time, throttle and efficiency (an eCalc-style estimate) [O/I].

### `ModelData.db` (SQLite) [O]
- Catalogs: `motor` (1676 rows), `propeller` (1399), `battery` (1560), `esc`. Columns cover brand, KV,
  resistance, I0/U0, weight, diameter/pitch, capacity, C-rating, etc.
- `planeItems(name,item)`: named airframes such as "DJI F450" and "T-MOTOR 670". The item is
  `FRAME,..;...;MotorBrand,Type;PropBrand,Type;EscBrand,Type;BatteryBrand,Type,...`.
- `uavparams(name, modelInfo, hoverInfo, frameInfo)`. Default `modelInfo`:
  `1.4, 9.8, 0.0211, 0.0219, 0.0366, 0.225, 1.105e-05, 1.779e-07, 1148, -141.4, 1.287e-04, 0.05, 0.073, 0.0055`.
  Mapping from the UI label order [I]: m, g, Jxx, Jyy, Jzz, arm length d, Ct (T=Ct·ω²), Cm (M=Cm·ω²),
  Cr and ωb (motor throttle→speed: ω=Cr·σ+ωb), Jm, Tm (motor time constant),
  Cd (drag D=Cd·V²), Cmd (air-torque coefficient). This is the textbook multicopter model from Quan Quan,
  *Introduction to Multicopter Design and Control*.
- `uidata`: last UI state (CopterID=1, SILPort=20100, sim mode, broadcast flag, initial x/y/yaw) [O].

## 3. 3DDisplay (Ogre3D 1.x + Qt5)
- Renderer: `RenderSystem_GL` (OpenGL) via `OgreMain`/`OgreOverlay` [O].
- Assets: `media/f550/*.mesh` (F550 hexa; `f550broken` is the crash model), `media/dji/dji.mesh`,
  a tiled terrain `media/terr/img11..44.jpg` with `terrain.cfg`, and a 5121² 16-bit heightmap `neweleb` [O]
  (the same heightmap ships as `CopterSim/neweleb.png`, so CopterSim does ground collision on it [I]).
- Receives vehicle state over a `QUdpSocket` [O]. The port and packet layout are **[?]**.

## 4. Open questions
1. ~~UDP packet layout CopterSim→RflySim3D and the ports~~: resolved from RflySimSDK's public Python, see
   [RFLYSIM3D_PROTOCOL.md](RFLYSIM3D_PROTOCOL.md).
2. Signature of the model-DLL ABI (`HLLDLL`).
3. Exact force/torque equations, ground contact and the battery model in the built-in physics.
4. The `UDP_Simple/Full` payloads used by external (Python/Simulink) controllers.

None of these block a Mac version that works with **upstream PX4 SITL + QGC**. That path relies only
on the public PX4 simulator MAVLink interface. They do matter for compatibility with existing RflySim
Python/Simulink scripts.
