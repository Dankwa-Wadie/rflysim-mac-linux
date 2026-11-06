# Windows RflySim vs. RflyTools for macOS/Linux: parity audit

Goal: every RflyTools item a Windows RflySim user has, working the same way on macOS (and Linux).

Sources: the RflySim installer ISO (`RFLYExpPspSoftwarePack`, readme and file list), the original
CopterSim.exe running under Wine, and RflySim's public documentation: the
[BAT script reference](https://rflysim.com/doc/en/4.DevRef/bat.html),
[quick start](https://rflysim.com/doc/en/1.Start/quick.html) and [FAQ](https://rflysim.com/doc/en/6.Adv/faq.html).
The docs describe the current RflySim (31 main + 20 advanced BAT scripts, some marked *Full version only*, i.e.
RflySim's paid edition). The ISO is an older pack (CopterSim, 3DDisplay, FlightGear, QGC, eclipse, MATLAB PSP).
No public page lists the exact contents of the desktop RflyTools folder.

Legend: ✅ done · 🟡 partly · ❌ missing · ➖ not needed on macOS/Linux (Windows-only plumbing) · 💲 Full (paid) version only on Windows

## 1. Programs

| Windows | Here | Status | Gap |
|---|---|---|---|
| CopterSim.exe (GUI: vehicle configurator, mode/port/map/start position, Start/Stop/Restart, live state) | **CopterSim window** (`coptersim-ui`) + `coptersim` | ✅ | Same layout: frame, weight, size, altitude, temperature, motor/propeller/ESC/battery from RflySim's catalog or custom design, design estimates (matched to RflySim's F450 numbers), Model Parameter, Assembled UAV Database (Activate!, Add, Delete), vehicle ID, vehicles, simulation mode, UDP mode, 3D scene, Pixhawk port, initial position, Start/Stop/Restart, live state |
| CopterSimNoUI.exe | `coptersim` | ✅ | |
| GetComList.exe (find Pixhawk COM ports) | `rfly-comlist` (`scripts/ComList.sh`) | ✅ | Lists serial ports with USB names, marks flight controllers, recommended list |
| RflySim3D.exe (UE4) | `rflysim3d` | 🟡 | Same UDP protocol, `-key=` / `-cmd=` start options, `LowGPU`, model styles, models stand on their landing gear. Missing: `RflySim3D.txt` config file, UE4 scenes such as `OldFactory` (procedural scenes instead) |
| RflySimUE5\RflySim3D.exe | `rflysim3d` | ➖ | One viewer covers both |
| 3DDisplay.exe | `rflysim3d --map Blank` | ✅ | |
| QGroundControl.exe | official QGC | ✅ | |
| FlightGear-F450 | `rfly-flightgear` | ✅ | |
| Python38Env / Python 3.8 + RflySimSDK | `rfly-python` + RflySimSDK port | 🟡 | SDK is a subset (UE4CtrlAPI, VisionCaptureApi UDP, PX4MavCtrlV4) |
| eclipse | VS Code / any editor | ➖ | |
| WinWSL / WslGUI / WslSwitch2 / VcXsrv / gpedit | not needed | ➖ | PX4 builds natively on macOS/Linux |
| MATLAB PSP (Simulink → Pixhawk code generation, PX4CMD) | RflyMac (optional) + bdsim; `rfly-px4build BOARD [--upload]` | 🟡 | MATLAB-free by choice. Firmware build/upload for any PX4 board (old RflySim board names accepted); needs the Arm compiler (not tested on a board yet) |
| One-key installer, GenerateShortcutCMD.bat | `.pkg` / `.deb` / tarball, launchers | ✅ | |
| RflySimAPIs folder (examples) | `python/examples` | 🟡 | Few examples |
| DistSim (multi-machine) | — | ❌ | |
| cfclient (Crazyflie) | — | ❌ | Wrapper only (pip package) |

## 2. Launch scripts

| Windows script | Here | Status | Gap |
|---|---|---|---|
| SITLRun (asks "Please input UAV swarm number:") | `rfly-sitlrun [N]` | ✅ | Same prompt and checks; options for `START_INDEX`, `MAX_VEHICLE`, `VEHICLE_INTERVAL`, `ORIGIN_POS_X/Y`, `ORIGIN_YAW`, position lists, `UE4_MAP`, `UDPSIMMODE`, `IS_BROADCAST`, `CLASS_3D_ID`, frame, parameter file; Enter stops. (`DLLModel`: see below) |
| HITLRun (asks for a COM list "3,4,5") | `rfly-hitlrun [ports]` | ✅ | Port list + "My port list for HITL simulation is:" prompt (numbers or paths, Enter = recommended), `BaudRate`, positions, UDP mode; QGC starts after the ports are taken |
| HITLRunAuto | HITLRun default (all ports) | ✅ | |
| SITLRunLowGPU / HITLRunLowGPU | launchers + `--low-gpu` | ✅ | |
| SITLRunNoUI / HITLRunNoUI | `--no-3d` | ✅ | |
| SITLRunDebug | `scripts/run_sitl.sh` | ✅ | Name differs |
| SITLRunChange3D / HITLRunChange3D | `--vehicle-type T` | ✅ | |
| SITLRunFw / HITLRunFw (fixed wing) | — | ❌ | **No fixed-wing physics** |
| SITLRunLANSwarm / HITLRunLANSwarm (`IS_BROADCAST=1`) | `--broadcast 1` or IP list | ✅ | |
| SITLRunROS / HITLRunROS (MAVROS) | — | ❌ | ROS is rare on macOS; doable on Linux |
| SITLRunMAVLinkNoGPS / HITLRunMAVLinkNoGPS | `--udp-mode Mavlink_NoGPS` | ✅ | |
| SITLPos / HITLPos (type X/Y/yaw per vehicle) | `--ask-pos` | ✅ | |
| SITLPosStr / HITLPosStr (lists `PosXStr=1,2,3`) | `--pos-x/--pos-y/--yaw-list` | ✅ | |
| SITLPosAlt(Str) / HITLPosAlt(Str) | — | ❌ | |
| SITLPosStrSel | — | ❌ | |
| HITLRunSysID / HITLPosSysID(Str) | `--sysid` | ✅ | |
| CloseALL | `rfly-closeall`, CloseALL launcher | ✅ | |
| UdpPortFree | `rfly-udpportfree [--kill]` | ✅ | |
| Python38Run | `rfly-python-run` | ✅ | |
| StopBuild | — | ➖ | |
| SITLRunUE5 / HITLRunUE5 | — | ➖ | |
| 💲 SITLPosStrGPS / HITLPosStrGPS, HITLRunPX4Net, HITLRunExtNet, APMSITLRun(_Plane), APMSwitchSITL, Redis modes, SimMode 4–11 | — | ❌ | Paid-edition features on Windows; ArduPilot SITL is the most useful of them |

## 3. Simulation options (CopterSim)

| Option | Here | Status |
|---|---|---|
| SimMode 0 PX4_HITL, 1 PX4_SITL | yes | ✅ |
| SimMode 2 PX4_SITL_RFLY | PX4_SITL path | 🟡 (RflySim's modified PX4 build is not used; upstream PX4) |
| SimMode 3 Simulink&DLL_SIL | `--ext` lockstep port | 🟡 (different mechanism, same purpose) |
| UDPSIMMODE 0 UDP_Full | yes (RflySim3D + true-data outputs) | ✅ |
| UDPSIMMODE 1 UDP_Simple, 2 Mavlink_Full, 3 Mavlink_Simple, 4 Mavlink_NoSend, 5 Mavlink_NoGPS, 6 Mavlink_Vision | `--udp-mode` (all) | ✅ (UDP_Full/Simple bridge wire-compatible with RflySimSDK; all flown in PX4 SITL) |
| Frames: quad, hexa, octo, octo-quad, tri, Y6 | quad X/+, hexa X, octo X, Y6 | 🟡 (tricopter needs a tail servo: not modelled; octo-quad (coaxial X8) missing) |
| Model-DLL plug-in | — | ❌ |
| Broadcast (`IS_BROADCAST`) | `--broadcast` / coptersim `--ue4-ip` | ✅ |
| Up to 50 vehicles | tested with 2–4 | 🟡 |

## 4. Plan (proposed order)

1. **CopterSim window**: configurator (frame, motor, propeller, ESC, battery, estimates, save to database) +
   simulation panel (SITL/HITL, vehicles, ports, map, position, UDP mode) + Start/Stop/Restart + live state.
   coptersim gains `--params-file` so the configured drone is what flies.
2. **SITLRun/HITLRun parity**: all variables above as options, Windows prompts, LowGPU/NoUI/Change3D variants,
   position and SysID variants, CloseALL, UdpPortFree, Python38Run, a COM/serial port lister.
3. **UDPSIMMODE** 1–6 and broadcast (LAN swarm).
4. **RflySim3D**: `-key=`/`-cmd=` options, `RflySim3D.txt`, an `OldFactory`-style map, LowGPU, model styles.
5. **Pixhawk firmware build/upload** (PX4CMD equivalent).
6. **More frames** (hexa, octo, tri, Y6), then **fixed wing**.
7. Optional: ArduPilot SITL, MAVROS (Linux), DistSim, cfclient wrapper.
