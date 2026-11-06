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
| CopterSim.exe (GUI: vehicle configurator, mode/port/map/start position, Start/Stop/Restart, live state) | `coptersim` (headless) | 🟡 | **No GUI window.** Catalog picking exists only in RflyGame's Garage, without ESC, and can't feed the simulator |
| CopterSimNoUI.exe | `coptersim` | ✅ | |
| GetComList.exe (find Pixhawk COM ports) | HITLRun's port glob | 🟡 | No port lister with Pixhawk detection / recommended list |
| RflySim3D.exe (UE4) | `rflysim3d` | 🟡 | Same UDP protocol. Missing: `-key=` and `-cmd=` start options, `RflySim3D.txt` config, UE4 maps (`OldFactory`, `LowGPU`, …), model-style switching |
| RflySimUE5\RflySim3D.exe | `rflysim3d` | ➖ | One viewer covers both |
| 3DDisplay.exe | `rflysim3d --map Blank` | ✅ | |
| QGroundControl.exe | official QGC | ✅ | |
| FlightGear-F450 | `rfly-flightgear` | ✅ | |
| Python38Env / Python 3.8 + RflySimSDK | `rfly-python` + RflySimSDK port | 🟡 | SDK is a subset (UE4CtrlAPI, VisionCaptureApi UDP, PX4MavCtrlV4) |
| eclipse | VS Code / any editor | ➖ | |
| WinWSL / WslGUI / WslSwitch2 / VcXsrv / gpedit | not needed | ➖ | PX4 builds natively on macOS/Linux |
| MATLAB PSP (Simulink → Pixhawk code generation, PX4CMD) | RflyMac (optional) + bdsim | 🟡 | MATLAB-free by choice. **No firmware build/upload tool for Pixhawk boards** (`PX4CMD px4-fmu-v5_default`) |
| One-key installer, GenerateShortcutCMD.bat | `.pkg` / `.deb` / tarball, launchers | ✅ | |
| RflySimAPIs folder (examples) | `python/examples` | 🟡 | Few examples |
| DistSim (multi-machine) | — | ❌ | |
| cfclient (Crazyflie) | — | ❌ | Wrapper only (pip package) |

## 2. Launch scripts

| Windows script | Here | Status | Gap |
|---|---|---|---|
| SITLRun (asks "Please input UAV swarm number:") | `rfly-sitlrun N` | 🟡 | Same idea. Missing variables: `START_INDEX`, `MAX_VEHICLE` (50), `UDP_START_PORT`, `VEHICLE_INTERVAL`, `ORIGIN_POS_X/Y`, `ORIGIN_YAW`, `SimMode`, `UDPSIMMODE`, `IS_BROADCAST`, `DLLModel`, `CLASS_3D_ID`, `PX4SitlFrame`; prompt text; "press any key to stop" |
| HITLRun (asks for a COM list "3,4,5") | `rfly-hitlrun [ports]` | 🟡 | No interactive port list prompt with recommended list; no `BaudRate`/`IsSysID` options |
| HITLRunAuto | HITLRun default (all ports) | ✅ | |
| SITLRunLowGPU / HITLRunLowGPU | — | ❌ | Easy: lighter render settings (`UE4_MAP=LowGPU`) |
| SITLRunNoUI / HITLRunNoUI | `--no-3d` (SITL only) | 🟡 | HITL variant missing |
| SITLRunDebug | `scripts/run_sitl.sh` | ✅ | Name differs |
| SITLRunChange3D / HITLRunChange3D | — | ❌ | Choose the 3D vehicle model style |
| SITLRunFw / HITLRunFw (fixed wing) | — | ❌ | **No fixed-wing physics** |
| SITLRunLANSwarm / HITLRunLANSwarm (`IS_BROADCAST=1`) | — | ❌ | Broadcast to other machines' RflySim3D |
| SITLRunROS / HITLRunROS (MAVROS) | — | ❌ | ROS is rare on macOS; doable on Linux |
| SITLRunMAVLinkNoGPS / HITLRunMAVLinkNoGPS | — | ❌ | `UDPSIMMODE` choice |
| SITLPos / HITLPos (type X/Y/yaw per vehicle) | — | ❌ | |
| SITLPosStr / HITLPosStr (lists `PosXStr=1,2,3`) | — | ❌ | |
| SITLPosAlt(Str) / HITLPosAlt(Str) | — | ❌ | |
| SITLPosStrSel | — | ❌ | |
| HITLRunSysID / HITLPosSysID(Str) | — | ❌ | CopterID from the Pixhawk's SysID |
| CloseALL | Ctrl-C in SITLRun | 🟡 | No standalone "stop everything" |
| UdpPortFree | — | ❌ | Find and free busy simulation ports |
| Python38Run | — | ❌ | Run the example in the current folder |
| StopBuild | — | ➖ | |
| SITLRunUE5 / HITLRunUE5 | — | ➖ | |
| 💲 SITLPosStrGPS / HITLPosStrGPS, HITLRunPX4Net, HITLRunExtNet, APMSITLRun(_Plane), APMSwitchSITL, Redis modes, SimMode 4–11 | — | ❌ | Paid-edition features on Windows; ArduPilot SITL is the most useful of them |

## 3. Simulation options (CopterSim)

| Option | Here | Status |
|---|---|---|
| SimMode 0 PX4_HITL, 1 PX4_SITL | yes | ✅ |
| SimMode 2 PX4_SITL_RFLY | PX4_SITL path | 🟡 |
| SimMode 3 Simulink&DLL_SIL | `--ext` lockstep port | 🟡 (different mechanism, same purpose) |
| UDPSIMMODE 0 UDP_Full | yes (RflySim3D + true-data outputs) | ✅ |
| UDPSIMMODE 1 UDP_Simple, 2 Mavlink_Full, 3 Mavlink_Simple, 4 Mavlink_NoSend, 5 Mavlink_NoGPS, 6 Mavlink_Vision | — | ❌ |
| Frames: quad, hexa, octo, octo-quad, tri, Y6 | quad | 🟡 |
| Model-DLL plug-in | — | ❌ |
| Broadcast (`IS_BROADCAST`) | — | ❌ |
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
