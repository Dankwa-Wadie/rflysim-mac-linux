# Progress log: RflySim for macOS (Rust)

Goal: a lightweight native macOS replacement for RflySim's **CopterSim** (simulator) and
**3DDisplay** (viewer), written in Rust. It works with upstream PX4 SITL/HIL and QGroundControl.

## Status

| Area | State |
|---|---|
| ISO recon (static) | ✅ Done, see [docs/RE_FINDINGS.md](docs/RE_FINDINGS.md) |
| Rust workspace + physics core (`rfly-core`) | ✅ Quad-X 6-DOF, motors, drag, ground contact, IMU/mag/baro/GPS. 4 tests |
| `ModelData.db` reader (`rfly-db`) | ✅ Param sets + airframe list. 2 tests |
| `coptersim` CLI + PX4 MAVLink link | ✅ Builds (2.8 MB). Smoke-tested message rates without PX4 |
| Fly with real PX4 SITL | ✅ PX4 v1.17.0. Automated takeoff/hover/land passes (`scripts/e2e_px4_sitl.sh`) |
| QGroundControl link | ✅ QGC installed. It connects automatically (PX4 sees partner 127.0.0.1 on UDP 14550). Manual flight by user pending |
| MATLAB-free toolchain (plugins, logs, scripting) | 🟡 Python SDK port done (`python/RflySimSDK_mac`); plugins/logs pending |
| **RflySim3D replacement** (`rflysim3d`) | ✅ Same UDP protocol. 5 procedural maps, vehicles, labels, commands, RGB/depth/grey/seg cameras. Protocol test 5/5 |
| RflySim UDP compatibility (`rfly-proto`) | ✅ CopterSim → RflySim3D (6i14f4d), true data 30101+, MAVLink ports 20100+. 7 tests |
| SITLRun / Python38Env | ✅ `scripts/SITLRun.sh N`, `scripts/Python38Env.sh`. E2E Python offboard + camera passes |
| HITLRun | 🟡 Script + coptersim MAVLink relay (QGC 14550, Python 20100/20101). Untested, no Pixhawk |
| FlightGear-F450 | 🟡 `coptersim --fg` (FGNetFDM v24, tested by unit test) + F450 model from ISO. FlightGear not installed (needs ~2 GB) |
| Serial HIL (Pixhawk) | 🟡 Wired (`--link serial:...`), untested |
| Hexa/Octo/Tri/Y6 frames | ⏳ Physics is quad only; the viewer draws hexa/octo/plane/car |
| **Linux support** | ✅ Debian 12 arm64 container: test_all **A–G all PASS incl. PX4 v1.17 SITL**. `.desktop` launchers, tarball package |
| **Installers** | ✅ macOS `.pkg` (universal); Linux `.deb` + tarball (amd64, arm64). Linux packages install-tested on Ubuntu 24.04 / Debian 12 |
| **GitHub** | ✅ Public: github.com/Dankwa-Wadie/rflysim-mac-linux |
| GUI (vehicle configurator) | ⏳ |
| Model-DLL ABI | ⏳ |

## Decisions
- **2026-11-04**: Rust, lightweight. First milestone: headless simulator + PX4 SITL, tested with QGC. Viewer and GUI come after.
- **2026-11-04**: Leave out the original's serial-number licensing. Leave out the MATLAB PSP; it may come back later as optional.
- **2026-11-04**: `rfly-core` has zero dependencies (own vec/quat math). The `coptersim` deps are only `mavlink`, `clap` and `rusqlite`.
- **2026-11-04**: **Trimmed-down, MATLAB-free version** (user's choice). MATLAB's roles get these replacements:
  - Simulink controllers → Rust/Python controller plugins.
  - Model DLLs → a Rust plugin trait, or a C-ABI `.dylib` that accepts Simulink-generated code later.
  - Experiment control → Python scripting.
  - Analysis → CSV logs, PlotJuggler and pyulog.
  - Build order: 3D viewer → CSV logs → plugins → Python examples → more frames.
- **2026-11-04**: `ModelData.db` is copied to `assets/` as a data source for parameter presets.

## Log

### 2026-11-04: session 1
- Inventoried the ISO (3.89 GB, complete). Only CopterSim and 3DDisplay are RflySim-specific.
- Read the `ModelData.db` schema, default vehicle params and CopterSim strings (modes, ports, timers, DLL model hook).
- Built the workspace:
  - `rfly-core`: physics and sensors.
  - `rfly-db`: SQLite reader.
  - `coptersim`: listens on TCP 4560 for PX4. Sends HIL_SENSOR at 250 Hz, HIL_GPS at 10 Hz and a 1 Hz heartbeat.
    Applies HIL_ACTUATOR_CONTROLS to the motors.
- Checks:
  - `cargo test`: 6/6 pass.
  - Fake-client smoke test: 551 HIL_SENSOR, 22 HIL_GPS and 2 HEARTBEAT in 2.2 s.
- Sanity check on the inferred param order: the default set gives hover throttle 0.608 (realistic).

### 2026-11-04: session 1 (cont.): first real flight
- PX4 v1.17.0 (latest stable): shallow clone in `~/tools/PX4-Autopilot`.
  Python deps in the venv `~/tools/px4-venv`. `brew install ninja ccache`.
  `make px4_sitl_default` built OK (1109 targets).
- v1.17 external-simulator airframe: `10016_none_iris`. Run it with
  `cd build/px4_sitl_default/rootfs && PX4_SIM_MODEL=none_iris ../bin/px4 -d ../etc`.
  PX4 connects to `coptersim` on TCP 4560 with lockstep.
- Confirmed from the PX4 source (`SimulatorMavlink.cpp`): controls are sent only while armed,
  and the armed state comes from the `MAV_MODE_FLAG_SAFETY_ARMED` bit.
- Added `scripts/e2e_px4_sitl.sh`. It starts both programs, waits for "Ready for takeoff",
  sends `commander takeoff`, checks the hover, then sends `commander land` and checks the disarm.
- **All tests pass**:
  - `cargo test --workspace`: 6/6.
  - E2E: connect ✅, EKF+GPS ready ✅, arm ✅, hover at z = −2.51 m with roll/pitch −0.1° ✅,
    land + auto-disarm ✅.
  - The PX4 log has **zero WARN/ERROR lines**.
- Cross-check: PX4's controller hovers at throttle 0.61, against a model prediction of 0.608.
  This supports the inferred `modelInfo` field order.

### 2026-11-04: QGroundControl
- The user installed QGroundControl. Added `scripts/run_sitl.sh`, which starts coptersim + PX4 with the pxh shell in the terminal.
- Fixed `e2e_px4_sitl.sh`: it left PX4 running after exit. It now `exec`s px4 so the trap kills the right process.
  Re-ran: all E2E checks pass, with no leftover processes.
- Live session launched: QGC connected (PX4 log: `partner IP: 127.0.0.1`).
- QGC 5.1 has a built-in 3D viewer (Settings → enable the 3D viewer, plus an OSM file for buildings).
  Downloaded `assets/osm/zurich_home.osm`: 282 buildings around the PX4 default home 47.3977, 8.5456.
  Data © OpenStreetMap contributors, ODbL.
- Computer use (direct clicking in QGC) is not available in this Claude session. Instead, flights are commanded from scripts while the user watches in QGC.
- Added `scripts/fly_square.py`, the first MATLAB-free experiment script. It uses pymavlink to UDP 14540:
  takeoff → 80 m square at 30 m via `DO_REPOSITION` → land.
  Flew it live: all 5 waypoints reached at 30.0 m, landed and auto-disarmed ✅.
- Set `MIS_TAKEOFF_ALT=30` in the live PX4 session. The E2E test wipes params, so it's unaffected.

### 2026-11-04: How RflySim uses Simulink
- The user expected QGC's 3D view to be FPV. It isn't (map/3rd-person only). FPV = sim-rendered camera,
  optionally streamed to QGC on UDP 5600. Planned for our viewer.
- MAVLinkMCP (ion-g-ion, commit 776b05e) selected and reviewed (no exec/subprocess). Install was blocked by
  auto mode; the user must install it or allow it.
- Analysed the PSP from the ISO without MATLAB (read the `.slx` XML). Write-up:
  [docs/RFLYSIM_SIMULINK_WORKFLOW.md](docs/RFLYSIM_SIMULINK_WORKFLOW.md). Key points:
  - "One controller, two harnesses": desktop plant model vs PX4 hardware blocks.
  - Embedded Coder generates `px4_simulink_app`, which writes PWM directly.
  - Testing is done with CopterSim HIL/SITL.
  - `Simulink&DLL_SIL` is non-functional in the free CopterSim (it falls back to HIL mode).

### 2026-11-04: RflyMac MATLAB/Simulink package
- New `coptersim --ext [PORT]` (default 20100, RflySim's SIL port). It is a **lockstep** external-controller mode:
  each UDP request carries motors and N×1 ms steps, and the reply is a 39-value state/sensor vector.
  Protocol spec and code in `crates/coptersim/src/ext.rs`, with 2 new unit tests.
- New package `matlab/RflyMac/` (see its README). It needs only MATLAB + Simulink + Xcode, no other toolboxes:
  - **CopterSim (Mac)** block: C-MEX S-function, 1 input (motors), 13 outputs, not direct-feedthrough.
  - `rflymac_step` MEX for scripts.
  - Example controller `rflymac_attitude_ctrl.m`: altitude cascade + angle-P/rate-PI, PX4 quad-x mixer.
  - Generators for the library + example model (`rflymac_example_attitude.slx`, built when you install).
  - `rflymac_quicktest.m`.
- Tuning: the first altitude PID overshot to 6.7 m (integrator windup), caught by the test. It was replaced with a
  position→velocity cascade (|vz| ≤ 2 m/s). Now: 5.04 m at t = 4.9 s, roll 9.72°/9.7°, yaw 28.6°/28.6°.
- Verified without MATLAB:
  - `scripts/test_rflymac_client.sh`: 8/8 PASS. 15 s of flight simulates in ~0.6 s.
  - The S-function + MEX compile against stub headers.
  - `cargo test`: 8/8.
- **Not yet verified inside MATLAB.** First run after the user's MATLAB install: `install_rflymac('save')`, then
  `rflymac_quicktest`, then open the example model.

### 2026-11-04: RflySim3D and RflyTools on macOS
- The user asked for RflySim3D plus all of RflyTools (3DDisplay, CopterSim, eclipse, FlightGear-F450, HITLRun,
  Python38Env, QGC, RflySim3D, SITLRun, Win10WSL), without MATLAB.
- RflySim3D is a closed UE4 binary and is **not on the ISO**, so it can't be ported. Instead, its protocol came from
  the public RflySimSDK Python (read from GitHub `maris205/airsim_agent`, not vendored).
  Write-up: [docs/RFLYSIM3D_PROTOCOL.md](docs/RFLYSIM3D_PROTOCOL.md).
- New crate `rfly-proto`: codecs for every RflySim3D packet (sizes checked against Python `struct.calcsize`). 7 tests.
- New crate `rflysim3d` (macroquad, 1.3 MB):
  - Listens on UDP 20010+N (auto-increments per window) and multicast 224.0.0.10:20009.
  - Maps: Grasslands, City, Blank, VisionRingBlank, Desert.
  - Procedural quad/hexa/octo/plane/car models with spinning props, lit shader with fog, blob shadows, trails and labels.
  - Commands: RflyChangeMap*, RflyCameraPosAng*, RflySetVehiclePosAng, RflyReqObjData, ...
  - Vision sensors: RGB/JPEG/PNG/raw, 16-bit mm depth, grey and segmentation, streamed in RflySim's 4i1d UDP image format.
  - Crash detection (P key).
  - **macOS gotcha:** `sendto` fails with EMSGSIZE above SO_SNDBUF (9216 B by default). The fix is to raise SO_SNDBUF.
- coptersim:
  - New flags: `--id` (PX4 TCP 4560+id−1), `--init-x/y/yaw`, `--vehicle-type`.
  - Streams to RflySim3D at 60 Hz and true state to 30101+2(id−1) at 100 Hz, in sim time, so the ext mode also streams.
  - `--fg` FlightGear native-fdm output.
  - MAVLink relay for HIL (`--forward`, defaults to QGC + the Python ports on serial links).
- Scripts:
  - `SITLRun.sh N`: N × coptersim + PX4 (`-i k`) + an extra PX4 mavlink instance on 20100+2k/20101+2k + viewer [+ QGC].
  - `HITLRun.sh`, `Python38Env.sh` (venv `~/tools/rflysim-py`), `FlightGearF450.sh`.
  - F450 aircraft extracted from the ISO's FlightGear zip into `assets/flightgear/Aircraft/F450`.
- `python/RflySimSDK_mac`: API-compatible `UE4CtrlAPI`, `VisionCaptureApi` (UDP only; shared-memory mode 0 becomes JPEG) and `PX4MavCtrlV4` (subset).
- Verified:
  - `cargo test --workspace`: 16/16.
  - `scripts/test_rflysim3d.py`: 5/5 (82 RGB frames, 55 depth frames, CoptReqData replies).
  - SITLRun with 2 vehicles: both PX4 instances ready.
  - `python/examples/offboard_camera.py`: hover 5 m ✅, move to (10,5,−8) ✅ (true 9.72/4.93/−7.95), camera 640×480 ✅, landed ✅.
- Killed a stale coptersim + PX4 pair left over from the 07:22 QGC demo. It was blocking instance 0. SITLRun now refuses to start if one is running.

### 2026-11-04: FlightGear, icons, controller design
- FlightGear 2024.1.7 installed (`brew install --cask flightgear`, 238 MB). The macOS app ships **without FGData**,
  so FGData (1.9 GB) is downloading to `~/tools/fgdata` with resume. The network is slow (~50 KB/s), so it runs in the background.
  `FlightGearF450.sh` now uses `FlightGear.app/Contents/MacOS/FlightGear --fg-root`.
  Bundled scenery covers Hawaii and Iceland only (use PHNL); `--terrasync` downloads more. SITLRun has a new `--fg` flag.
- **Icons + app bundles** (user request): `scripts/make_icons.py` draws original icons with numpy/OpenCV.
  `scripts/build_apps.sh` builds `RflyTools/*.app`, linked to `~/Applications/RflyTools`:
  RflySim3D, 3DDisplay, FlightGear-F450 (GUI); CopterSim, SITLRun (asks for N), HITLRun, Python38Env (open in Terminal); QGroundControl symlink.
  RflySim3D.app was launch-tested via `open`.
- Controller design (user chose **bdsim**, Python Simulink-like):
  - `python/rflyctl/coptersim_ext.py`: lockstep client.
  - `python/rflyctl/controllers.py`: port of the MATLAB attitude/altitude controller plus a position outer loop.
  - Closed loop against `coptersim --ext`: 30 s of flight in 0.33 s, targets held within ~5 cm.
  - bdsim 1.4.0 installed (needed several retries on the slow network).
  - `python/rflyctl/bdblocks.py`: COPTERSIM, POSCTRL, ATTCTRL, WAYPOINTS. All are `SampledBlock`s on one 250 Hz clock.
  - Example diagram `python/examples/bd_position_control.py` (`--realtime`, `--noplot`, `--save PNG`):
    **PASS**, final position (−0.04, −0.07, −2.99) vs target (0, 0, −3). 30 s simulated in 0.69 s.
    Scope plot: `logs/bd_position_control.png`.
  - bdsim 1.4 API notes: subclass `bdsim.block.SampledBlock` (ClockedBlock is deprecated) and pass nin/nout/x0 to super;
    `bd.add_block()` returns None; the sample period is `self.T`; watched signals are in `out.y`.
  - Graphical editor `bdedit` is installed but needs PyQt5 (not installed yet).
  - Tuning note: pitch peaks at −32° on the first 11 m move (tilt command limit 25°).

### 2026-11-04: full test without FlightGear
- New `scripts/test_all.sh [ABCDEF]`: one command for the whole system. It needs no FlightGear and no Pixhawk. **All 8 checks PASS** in one clean run:
  - A: unit tests 16/16.
  - B: rflysim3d protocol + images 5/5.
  - C: FlightGear native-fdm packets checked by a stand-in listener. 408 B, v24, AGL and rpm sane.
  - D1: MAVLink relay (the HITL code path), relayed from SITL PX4.
  - D2: SITLRun + Python offboard/camera 4/4.
  - E/E2: bdsim diagram flying live in rflysim3d.
  - F: 8 app bundles (GUI apps launch; scripts and plists valid).
- Finding: PX4 SITL's simulator link carries only HIL_ACTUATOR_CONTROLS (no HEARTBEAT). A real Pixhawk link carries everything, so the relay test checks for actuator messages.
- FGData is still downloading in the background (~154 MB of 1797 MB at last check).

### 2026-11-04: RflyGame (playable game), phase 1
- The user wants a playable game with all modes, controller plug-in, LAN + online multiplayer, menus, leaderboards/replays,
  custom drones and a packaged app. Plan: phase 1 core → 2 race/missions/school → 3 leaderboards/replays/custom drones
  → 4 LAN + online (relay server) → 5 packaging.
- Refactor: `rflysim3d` is now lib + bin (`render.rs` shared renderer with CamPose/DrawVehicle/DrawExtra). The viewer was rebuilt on it.
- New crate `crates/rflygame` (phase 1 **done**):
  - `flight.rs`: presets Trainer F450 (ModelData default), Racer 5-inch (TWR 5.5) and Heavy Lifter (4 kg).
    Flight modes Beginner (velocity + altitude hold), Angle and Acro (rates + expo).
    Uses model-based allocation (torques → rotor thrust → throttle); gains scale with each drone's motor time constant. 3 tests.
  - `input.rs`: keyboard (eased sticks; Shift = full), right-mouse virtual stick, gilrs gamepads (Mode 2),
    and raw-axis joysticks (RC transmitters) with an AxisMap.
  - `ui.rs`: menus navigable by keyboard, gamepad and mouse; stick display. `settings.rs`: persisted to
    ~/Library/Application Support/RflyGame/settings.txt.
  - `game.rs`: 1 kHz physics substeps; crashes (ground impact > 4 m/s, upside down, map objects, out of bounds);
    cameras Chase/FPV (uptilt)/Orbit/Line-of-sight; HUD.
  - `main.rs`: Main menu → Free Flight setup (map/drone/mode) → play/pause; Settings; Controls
    with a 4-step "push the stick" axis wizard for RC transmitters. `--demo --screenshot` for automated checks.
  - RflyGame.app with its own icon added to RflyTools (launch-tested).
- Not yet tested: a physical gamepad / RC transmitter (none connected here). The wizard and mapping are code-complete.

### 2026-11-04: RflyGame phases 2-5 complete (user: "do not stop until done")
- **Phase 2 (modes):**
  - `race.rs`: 4 tracks (Raceway GP on the new Raceway map, City Slalom, Ring Run, Meadow Mini); countdown, laps,
    splits, respawn at the last gate, next-gate pointer.
  - `mission.rs`: 6 missions (Precision Landing, Rooftop Drop, Package Delivery with +30 % mass,
    Follow the Car, Windy Hover with gusts, City Run).
  - `school.rs`: 7 lessons with live checks.
  - `objects.rs`: square/ring/sphere gates with exact crossing tests, pads, beacons, off-screen indicator.
- **Phase 3:**
  - `records.rs`: leaderboards (top 10), replays, best-run ghosts, school progress.
  - 50 Hz CSV flight logs.
  - `garage.rs`: drones from ModelData.db parts (`rfly-db` now reads the motor/propeller/battery catalogs and
    airframes). `flight::from_components` reproduces RflySim's F450 Ct within 10 %.
  - Battery with voltage sag.
- **Phase 4:**
  - `net.rs`: UDP protocol, embedded or dedicated `rflygame-server`, LAN beacon discovery, lobby with host
    controls, synced race start, standings.
  - Verified with two game windows: free flight and race, both see each other.
- **Phase 5:** `scripts/package_rflygame.sh` builds `dist/RflyGame.app` (5 MB, ad-hoc signed, bundles the server
  and ModelData.db), the zip (2.2 MB) and a standalone server. arm64 only.
- **Bugs found and fixed by the autopilot runs:**
  - Saturated yaw used to raise total thrust (yaw is now lowest priority).
  - Leaderboard rank lookup lost precision.
  - App Nap throttles hidden windows (test mode allows physics catch-up).
- **User feedback round:**
  - **FPV** is now the default (Auto: chase only in Beginner) with a Betaflight-style OSD.
  - **Collisions:** penalty contact model with 11 points plus a body sphere vs boxes/gate bars. Bounces, friction,
    tumbling; damage thresholds (hard landing, prop strike, wall > 2.5 m/s, bump below that, flipped).
    `rfly-core` gained `ext_force`/`ext_torque`/`builtin_ground`.
  - **Sticks:**
    - Acro pitch sign bug fixed (forward = nose down).
    - Angle/Beginner use quaternion attitude error with yaw about world vertical.
    - New test covers every stick direction × mode × drone, including coordinated turns.
    - Keyboard default is now transmitter Mode 2 (W/S throttle, A/D yaw, arrows pitch/roll).
  - Chase camera no longer enters buildings; the ghost is hidden in FPV when it overlaps you.
- **Crash style (user feedback):** the default is now *Realistic*. Impacts disarm the motors and the drone falls and
  settles upright (re-arm) or on its back (turtle mode flips it back). No crash screen, and races/missions/lessons continue.
  *Arcade* is selectable in Settings. Hard-landing threshold raised to 5 m/s.
- `scripts/test_all.sh` gained stage G (RflyGame autopilot runs, collisions, 2-player multiplayer).

### 2026-11-05 — final regression
- FGData 2024.1.7 installed at `~/tools/fgdata` (2.7 GB).
- The disk filled up mid-run, from 8.5 GB free down to 0. The cause was other apps' caches, not this project.
  - Effect: PX4 SITL wrote a damaged `parameters.bson` and hung on "importing from parameter backup".
  - Fix: delete `build/px4_sitl_default/{rootfs,instance_*}/parameters*.bson`. PX4 recreates them.
  - Freed about 7 GB by clearing the Chrome, Homebrew, old IDE and Spotify caches. 8.7 GB is now free.
- `scripts/test_all.sh`: **all stages A–G pass**. That covers unit tests, the protocol, FlightGear FDM, MAVLink relay,
  SITL offboard with camera, bdsim, the 9 apps, and RflyGame (race, missions, lesson, collide, turtle, multiplayer).
- Note: `--quit-after` counts wall-clock seconds. With 3-lap races and several windows open, use 600 s or more.
- The test autopilot now brakes on measured velocity in Angle mode, where the stick sets tilt rather than speed.
  Before this it swung back and forth past the pad in lesson 6. Lesson 6 now passes (17 s). The game itself was not
  changed.
- The test autopilot can now fly Acro: throttle down to arm, then attitude error → rate sticks through the inverse of
  the expo curve. Lesson 7 passes (24 s).
- **All game content passes on autopilot:**
  - Races: Meadow Mini 133 s, Raceway Grand Prix 276 s, City Slalom 185 s, Ring Run (race 2) passes in stage G.
  - All 6 missions complete.
  - All 7 lessons pass.
- Packaging redone: `dist/RflyGame.app` (5 MB, arm64) and the zip. RflyTools apps rebuilt. Unit tests 30/30.
- **FlightGear-F450 visual check: blocked.**
  - `FlightGear` (even `--version`) hangs at 0 % CPU with no window and no log when started from the agent shell.
  - Tried: clearing a stale `fgfs_lock.pid` (from a dead run on 11-04), `open -n`, and `--disable-sentry`. None
    helped.
  - The FDM link itself is verified by test stage C (408-byte v24 packets).
  - To check by hand, from a normal Terminal: run `scripts/FlightGearF450.sh --airport=PHNL`, plus coptersim
    `--fg --lat=21.3187 --lon=-157.9224 --alt=4`.

### 2026-11-05 — universal build, sound, FlightGear demo
- **Universal app:** `scripts/package_rflygame.sh` builds arm64 + x86_64 when the Intel target is installed
  (`rustup target add x86_64-apple-darwin`, done), then joins them with `lipo`.
  - `dist/RflyGame.app` is 9.4 MB, universal, and ad-hoc signed.
  - The Intel build passed lesson 1 under Rosetta.
- **Sound (`crates/rflygame/src/sound.rs`):** all synthesised in code, with no assets.
  - Motor hum (4 layers cross-faded by motor speed), wind noise, arm/disarm beeps, impact/bump thuds,
    countdown/GO, gate/lap chimes, and win/lose tunes.
  - New Settings → Sound volume. Muted in `--auto` unless `RFLY_SOUND=1`.
  - `Session.sfx` is the event queue; race.rs and the contact code push to it.
  - 4 new unit tests (34 in total).
- **FlightGear-F450 works when started from a normal Terminal.** The user's run loaded RflySim's F450 from
  `assets/flightgear/Aircraft` at KSFO in about 3.5 s (later moved to Honolulu, PHNL: FGData 2024 ships only Hawaii and Iceland scenery, so KSFO showed an empty world). Only the agent's sandboxed shell hangs it.
  - Flew coptersim (`--ext`, Python position controller) into it for 90 s: climb to 8 m, then 6 m circles.
  - New `scripts/FlightGearDemo.sh [secs]` starts FlightGear (unless it is already listening), coptersim at PHNL (`--lat=21.3187 --lon=-157.9224 --alt=4`),
    and `python/examples/fg_demo_flight.py`.
  - Gotcha: negative values need `=` (`--lon=-122.3750`). Fixed in the docs and scripts.
  - test_all stage C now uses UDP 5512, so an open FlightGear no longer makes it fail.

## 2026-11-05 — multiplayer under internet conditions
- **Bug found:** the client sent nothing in the lobby or while paused, so the server dropped any player idle >6 s.
  The automated test missed it (the host started after 3 s). Also, the host's start command was sent once
  (lost packet = race never starts), and remote drones jumped between raw snapshots.
- **Netcode (`net.rs`, protocol bumped to `RFG2`):**
  - 0.5 s ping/pong: keepalive, RTT shown in the lobby and HUD, and automatic re-join after an outage.
  - Host commands carry a sequence number. They are resent until acknowledged and de-duplicated on the server.
  - States are stamped with the sender's clock. The server ignores reordered states.
  - Clients interpolate other drones with a per-player delay sized to measured jitter (70–300 ms) and
    extrapolate for ≤150 ms when packets are late.
  - The server drains all queued packets per loop.
- **New `netem.rs` + `rflygame-netem` tool:** a UDP proxy adding latency, jitter (reorders), loss and duplication.
- **New unit test `internet_conditions`** (35 tests in total, all pass):
  - Both players are behind proxies, one at 70+0..40 ms with 5 % loss, the other at 90+0..60 ms with 15 % loss.
  - They idle 8 s in the lobby, and a single host command starts the race.
  - Measured: RTT 243 ms, countdown skew 71 ms, largest view step 0.17 m/frame (raw snapshots jumped 15×).
- **End to end, real game:** Alice hosted, and Bob joined via `rflygame-netem` (80+0..40 ms, 5 % loss).
  - They waited 10 s in the lobby, then the race started. Bob saw Alice racing, with ping 199 ms in the HUD.
  - dist/RflyGame.app was repackaged (universal).
- **FPV in multiplayer:** sessions were hard-coded to Beginner mode, which uses the chase camera. The lobby now has
  "Your flight mode" (default Angle → FPV with OSD; Beginner keeps chase). Each player picks their own.
  Verified in a run through netem: Bob raced in FPV at 191 ms ping.
- **Hosting note:** this Mac is on an iPhone hotspot (172.20.10.x). Mobile carriers use CGNAT, so router port
  forwarding cannot work from here. Use a home router with a public IP, or a VPS.
- **Not done:** a test across a real internet path. It needs a reachable host: router port forward, a VPS,
  or a UDP tunnel/VPN, plus a second machine or a friend. See docs/RFLYGAME.md → Multiplayer details.

## 2026-11-05 — Play together: split screen, local network, online
- The user asked for separate modes for same-screen, same-network and internet play, citing Blur's split screen.
- **Menus:** "Multiplayer" became "Play together", which opens Split screen, Local network (host or discovered games)
  and Online (join by address; host with a port forward; server notes).
- **Split screen (`split.rs`):** up to 4 local pilots, each with their own Session; all see each other.
  - Join screen: four player cards. Join with Q, U or a controller's A/Start. Each pilot picks a flight mode (Angle = FPV default).
  - `input.rs` gained `Slot` (KbLeft, KbRight, Pad) and `poll_slot`, with per-player smoothing and button edges.
  - Each view and its HUD render into its own target at 1.3×/1.6× and are scaled down.
    - New `ui::set_view`/`vw`/`vh`/`hud_camera`: every HUD widget now lays out for the current view, not the window.
  - Layout: 2 pilots top/bottom, 3–4 in quarters. With 3, the spare quarter has an overhead view and the standings.
  - Pause, restart, and results with Race again.
- **Fix:** very wide views keep ≥55° vertical FOV. Otherwise the 20° FPV uptilt in a 3.4:1 half showed only sky.
- **Verified with screenshots:** 2 and 3 pilots, and the join screen.
  - A 2-pilot Meadow Mini ran to the results screen (127.83 s / 129.46 s).
  - LAN multiplayer still works, and all 35 unit tests pass.
  - test_all stage G gained a split-screen check. The app was repackaged.
- **Not yet:** split-screen pilots joining an online/LAN game together (a Client per local pilot), and real controllers.

### 2026-11-05: MATLAB
- The RflyMac package waits for a licensed MATLAB (trial/student/home/campus). The MATLAB-free path continues meanwhile.

## 2026-11-05 — virtual controllers
- **Why in-game:** macOS cannot create a system-wide virtual HID gamepad without an Apple-restricted entitlement
  (foohid-style kexts no longer work). So the game accepts virtual controllers itself.
- **`input.rs`:**
  - `VirtualPads` listens on 127.0.0.1:27170, enabled with `--vpad [port]`.
  - Packet: `RVP1`, pad, buttons, 4 axes. A pad expires after 2 s of silence.
  - New `Slot::Virtual(0..3)`. Real and virtual pads share `pad_raw` and `merge_edges`.
  - Virtual pad 1 also drives single player.
- **Python:** `python/rflyctl/vpad.py` (`VirtualPad.press/sticks/hold`, `hold_all`).
- **Bugs fixed along the way:**
  - D-pad right was missing in split-screen controller input.
  - The join and menu edge state was shared, which swallowed presses.
  - Controllers were read twice per frame (the pilot loop stole menu presses).
  - An A press from an already-joined pilot blocked the menu that frame.
- **E2E (`python/examples/vpad_split_test.py`, test_all stage G "vpad"):** passes.
  - Using controller input only, 4 pads joined split screen, P4 switched to Beginner, and P1 picked and started a race.
  - All 4 were armed at GO and flew to 9.4 m.
- Single-player check: a virtual pad drove the main menu → Free Flight → Start → arm → fly.
- **Tests:** 19 rflygame unit tests (new: `virtual_pad_packets_are_parsed_and_expire`).
- **Still uncovered:** the real-controller OS/gilrs layer. Pairing any Bluetooth Xbox/PlayStation pad would confirm it.

## 2026-11-05 — Linux compatibility
- **Rust:** only one real portability bug. RflyGame saved to `~/Library/Application Support` everywhere.
  `settings::data_dir()` now uses the platform folder: Linux `$XDG_DATA_HOME/rflygame` (default
  `~/.local/share/rflygame`), Windows `%APPDATA%`, with an `RFLY_DATA_DIR` override. New unit test (37 total).
  The sockets need no change: SO_REUSEPORT on Linux still fans multicast/broadcast out to every socket.
- **Linux build deps:** `libudev-dev` (gilrs, serialport), `libasound2-dev` (macroquad audio), `pkg-config`.
  miniquad dlopens X11/GL at run time. Note: quad-snd panics on its audio thread if ALSA has no device;
  the game keeps running silently.
- **New `scripts/platform.sh`** (sourced): `udp_listening` (lsof → ss → /proc), `default_serial_glob`
  (`/dev/ttyACM*` on Linux), `launch_qgc` (finds a QGroundControl AppImage or `QGC_BIN`).
- **Scripts:**
  - SITLRun `--qgc` and HITLRun (port glob, dialout/ModemManager hints, permission check) work on Linux.
  - FlightGearDemo uses `udp_listening`; FlightGearF450 finds apt's `fgfs` + `/usr/share/games/flightgear`.
  - Python38Env hints at `python3-venv`.
  - `build_apps.sh` on Linux writes `RflyTools/*.desktop` + `bin/` launchers + PNG icons and links them into
    `~/.local/share/applications` (terminal tools use `Terminal=true`; SITLRun asks N with `read`).
  - `package_rflygame.sh` on Linux makes `dist/RflyGame-linux-<arch>/` + `.tar.gz` (binaries, ModelData.db,
    icon, `install.sh` for a menu entry, README with runtime packages).
  - `test_all.sh` stage F validates the `.desktop` launchers on Linux.
- **Linux test harness:** `docker/linux-test/Dockerfile` (Debian 12 slim, rustup minimal, Xvfb, Mesa llvmpipe,
  null ALSA) + `scripts/linux_docker_test.sh [stages]`. It copies the repo into the container and runs
  `test_all.sh` (stages ABCEFG; PX4 SITL skipped) under a virtual display. Results go to `logs/linux_test/`.
- **Disk:** deleted the leftover Docker Desktop VM disk (11 GB, Docker Desktop itself was already uninstalled;
  user approved). Free space went from 7 GB to 18 GB. Installing colima as the Docker engine.
- macOS regression: `test_all.sh AF` PASS (37 unit tests, 9 app bundles).
- Docs: README Linux section; RFLYGAME.md Linux launch + save paths.
- **Linux verification (2026-11-06, colima 0.10.3, Debian 12 aarch64, Xvfb + Mesa llvmpipe):**
  - First run: unit tests 37/37 passed; every GUI stage failed with `X11 backend failed: libxkbcommon.so…`.
    miniquad 0.4.11 dlopens **libxkbcommon** on X11, so `libxkbcommon0` is now a documented runtime dependency.
  - Real bugs found: `find_qgc_linux` returned non-zero when QGC was missing, so `set -e` killed `build_apps.sh`
    (fixed); `.desktop` launchers had two main categories (now `Science;Simulation;` / `Game;Simulation;`).
  - test_all now creates the Python env up front when stages C/D/E/G need it (stage C used to run before it existed).
  - The split-screen check in stage G had a stale pattern (`P1:armed=true`). The output gained the input slot
    name in the virtual-controller work, so it failed on macOS too. Pattern fixed.
  - Final: **A B C E F G all PASS on Linux**. Split-screen screenshot renders correctly (3 FPV views + standings).
  - `package_rflygame.sh` on Linux: `RflyGame-linux-aarch64.tar.gz` (2.3 MB, needs glibc ≥ 2.34). Unpacked,
    `install.sh` writes a valid menu entry, a mission completes, saves land in `~/.local/share/rflygame`.
  - macOS regression after all changes: test_all A F G PASS.
  - colima stopped after the run (`colima start` to reuse; the Docker volumes keep the build cache).
- **PX4 SITL on Linux (2026-11-06):** `linux_docker_test.sh` now copies `~/tools/PX4-Autopilot` (without build/)
  into the `rfly-linux-px4` volume, makes a minimal PX4 venv (empy 3.3.4, jinja2, pyyaml, jsonschema, kconfiglib,
  future, pyros-genmsg, packaging, toml, numpy, lark) and builds `px4_sitl_default` once (1109 targets, OK).
  - **Real bug found:** PX4 refused to arm on Linux ("High Accelerometer Bias", "vertical velocity unstable",
    height estimate drifting 2 m on the ground). PX4 used 6.7 % CPU, so not load. `ss` showed coptersim's socket
    with `unacked:1 notsent:74`: **Nagle's algorithm** held each small HIL_SENSOR until the previous ACK, and
    Linux delays loopback ACKs (ato 40 ms). Sensors arrived in clumps, PX4 replied only ~160×/sim-s instead of 250
    and lost IMU samples. macOS loopback ACKs differently, so it never showed.
  - **Fix:** coptersim opens tcpin/tcpout links itself with `TCP_NODELAY` (`open_link` in coptersim main.rs,
    via `mavlink::TcpConfig::from_stream`). Linux E2E: all checks pass, 0 PX4 WARN/ERROR, 250 replies/s,
    hover z = −2.51 m (identical to macOS).
  - **Final Linux run: A B C D1 D2 E E2 F G all PASS.** Offboard: hover true z −4.84, move to (9.85, 4.91, −7.88), landed.
  - macOS after the fix: E2E all pass (z −2.51); test_all A–G all PASS. Stage F's launch check now polls up to
    10 s (a freshly rebuilt .app sometimes took >3 s to start after a long run).
- **Not verified on Linux:** real gamepads/serial Pixhawk, a real desktop session (Wayland/XWayland), x86_64
  (only arm64 was tested; nothing is arch-specific).

## 2026-11-06 — Installable packages for all tools
- User: packages for macOS and Linux that install all the tools; then commit to GitHub (**public** repo).
- **RflySim's data is not redistributed** (user's choice): `ModelData.db` and the F450 FlightGear model stay out of
  git and packages. New `scripts/import_rflysim_data.sh` (app "Import RflySim Data", command `rfly-import-data`)
  streams them out of the user's own ISO with bsdtar/7z in ~2 s (also takes CopterSim.zip, the FlightGear zip
  or an unpacked folder; asks for a path, macOS drag-and-drop escapes handled). Result is byte-identical to the
  old `assets/` copies. Data goes to `~/Library/Application Support/RflyTools` / `~/.local/share/rflytools`.
  - `rfly_db::find_model_db` / `tools_data_dir`: coptersim (no `--db` needed now), RflyGame's Garage and the
    scripts all find the imported copy. Without it: built-in F450 parameters and built-in drones.
  - The 3 rfly-db tests skip when the file is absent (fresh clone/CI).
- **Relocatable tools:** `platform.sh` gained `rfly_bin`, `rfly_need_bins`, `rfly_logs` (per-user logs when installed),
  `rfly_data_dir`, `rfly_db`/`rfly_set_dbarg` (Bash 3.2-safe), `rfly_fg_aircraft`. SITLRun, HITLRun, run_sitl,
  FlightGearF450/Demo use them. `build_apps.sh` gained `--root` / `--package` and run-time-resolving launchers,
  plus the Import app (new icon).
- **`scripts/package.sh`**:
  - macOS `dist/RflyTools-0.1.0-macOS.pkg` (11 MB, universal, productbuild with welcome + MIT license):
    `/Library/RflyTools`, `/Applications/RflyTools/*.app` (10), commands in `/usr/local/bin`, `uninstall.sh`.
    Binaries ad-hoc signed; the pkg is unsigned/not notarized (needs an Apple Developer ID).
  - Linux (`--linux` on a Mac, in Docker): `rflytools_0.1.0_{amd64,arm64}.deb` (~3 MB, `/opt/rflytools`, menu entries,
    `/usr/bin` commands) and `rflytools-0.1.0-linux-{x86_64,aarch64}.tar.gz` with a per-user, relocatable `install.sh`.
  - amd64 built under Rosetta (`colima --vz-rosetta`). The classic Docker builder ignores `--platform`, so the
    Dockerfile takes `BASE` and package.sh passes the arch-specific digest of debian:bookworm-slim.
- **Bugs found by install tests:**
  - Depends `libasound2 | libasound2t64` let apt on Ubuntu 24.04 pick `liboss4-salsa-asound2` (OSS stand-in) →
    `undefined symbol: snd_pcm_recover`. Now `libasound2t64 | libasound2 (>= 1.1)`.
  - `package_rflygame.sh` bundled ModelData.db; removed.
- **`scripts/test_linux_packages.sh [--iso ISO]`**: clean ubuntu:24.04/amd64 + debian:12/arm64; apt install, commands,
  menu, ISO import, coptersim DB, rflysim3d + RflyGame under Xvfb as a normal user, server, tarball install/move/
  uninstall, apt remove. **37/37 PASS.**
- macOS: payload tested in installed mode (import, DB pickup, F450 lookup, game); `test_all.sh` A–G all PASS.
  The real `.pkg` install still needs the user's admin password.
- Repo prep for public GitHub: MIT `LICENSE`, `.gitignore` excludes RflySim data + build output, README Install /
  RflySim data / PX4 sections, "not affiliated with RflySim" note.
- Disk: Docker's 15 GB data disk filled up (apt "invalid signature" = no space for gpgv). Removed reproducible caches
  (amd64 build image + cargo volume), trimmed the VM. Host free ~12 GB.
- Note: `~/.docker/config.json` still has Docker Desktop's `credsStore: desktop` (breaks pulls); commands here used a
  session-local `DOCKER_CONFIG` + `DOCKER_HOST=unix://~/.colima/default/docker.sock` instead of editing it.

## 2026-11-06 — Public GitHub repo, media, comparison with Windows RflySim
- Repo: **https://github.com/Dankwa-Wadie/rflysim-mac-linux** (public, MIT, author Dankwa-Wadie with the GitHub
  noreply address). Initial commit `1bda10e`, 115 files, 26 MB.
- Public-repo cleanup (user's choices): RE notes trimmed to compatibility facts (no tooling, method or function
  addresses), the MATLAB-installer note removed, RflySim's licensing not described. OSM data has an ODbL attribution file.
- **Original Windows CopterSim captured under Wine**: Homebrew's Wine casks are all disabled (Gatekeeper), so
  Debian's Wine 8 runs in an amd64 container (colima + Rosetta) on Xvfb. CopterSim.exe (64-bit Qt5) starts and
  shows its configurator with RflySim's catalog. **3DDisplay.exe is 32-bit**: Rosetta's Linux mode runs only
  x86_64 and Debian's Wine 10 still needs wine32:i386 (no new-WoW64), so it can't run here. RflySim3D is not on
  the ISO. Both are described and linked instead. Originals are unpacked in `~/tools/rflysim-wine` (not in the repo).
- **Frame recorder** (`rflysim3d::record`): `RFLY_RECORD=DIR` (+ `_FPS`, `_SECS`, `_AFTER`) writes the app's own
  frames as JPEGs from a background thread; hooked into rflysim3d and every RflyGame render loop. Used instead of
  screen capture, which would include the user's desktop.
- Media (`docs/media`, 20 MB): PX4 SITL offboard flight in rflysim3d, RflyGame FPV City Slalom, chase Raceway GP,
  3-pilot split screen (MP4 + README GIFs ≤ 4 MB), stills, Python camera image, bdsim scope, original CopterSim.
- `docs/COMPARISON.md`: Windows RflySim vs this project (systems, size, activation, PX4, MATLAB, each tool).
- README: showcase GIFs at the top, installed command names, Releases link.

## Next steps
0. When a licensed MATLAB is installed: run the RflyMac install + quicktest + example model. Fix anything MATLAB-specific.
1. User test-flies in QGC.
2. User tries `scripts/SITLRun.sh 1 --qgc` and existing RflySim Python scripts.
2b. FlightGear: done. Run `scripts/FlightGearDemo.sh` from Terminal.
2c. HITL with a real Pixhawk.
2f. RflyGame: real gamepad/RC transmitter test; a real internet session (netcode now tested under simulated internet conditions).
2e. Optional: `pip install PyQt5` for the bdedit graphical diagram editor, and a bdedit file for the example.
3. Compare CopterSim's force/ground model against runtime captures of the Windows build. Update RE_FINDINGS.
3. More frames; then the wgpu viewer.

## Open questions
See §4 of [docs/RE_FINDINGS.md](docs/RE_FINDINGS.md).
