# RflyGame

RflyGame is a drone flight game for macOS and Linux. It runs on the RflySim-for-macOS physics
(`rfly-core`, the same 6-DOF model as CopterSim) and draws the `rflysim3d` world.

**Launch:** the packaged app is universal (Apple Silicon and Intel). Double-click `RflyTools/RflyGame.app` (or `dist/RflyGame.app` once packaged), or run
`target/release/rflygame`.

**Linux:** run `scripts/package_rflygame.sh` to get `dist/RflyGame-linux-<arch>.tar.gz`. Unpack it and run `./rflygame`, or run `./install.sh` once to add an app-menu entry. The README's Linux section lists the packages it needs.

## Modes

| Mode | What you do |
|---|---|
| **Free Flight** | Pick a map (Grasslands, Raceway, City, Desert, VisionRingBlank, Blank), a drone and a flight mode, then explore. Building roofs are landable. |
| **Racing** | Choose a track: Raceway Grand Prix (8 gates, 3 laps), City Slalom (street circuit, 2 laps), Ring Run (RflySim's vision rings) or Meadow Mini (beginner course). You get a countdown, laps, splits and a pointer to the next gate. A crash respawns you at your last gate with progress kept. Your best run comes back as a **ghost**. |
| **Missions** | Precision Landing, Rooftop Drop, Package Delivery (the parcel adds 30 % mass), Follow the Car (stay 5–15 m away for 60 s), Windy Hover (gusts) and City Run (checkpoints). Each one is scored and time-limited. |
| **Flight School** | Seven lessons from first take-off to Acro. Each step is checked live (altitude band, hold time, yaw circle, waypoints, soft landing, mode switch). Finished lessons get a tick. |
| **Play together** | **Split screen** (up to 4 pilots on one Mac), **Local network** (host; friends on the same Wi-Fi find you automatically) or **Online** (join a server by address, or host with a port forward). The host picks free flight or a race; everyone starts on the same countdown and sees live standings. |
| **Garage** | Build drones from the RflySim **ModelData.db** catalog: 1676 motors, 1399 propellers and 1560 batteries, or start from one of the 15 named airframes. Stats update live: mass, thrust-to-weight, hover throttle, top speed and estimated flight time. Saved drones appear in every drone list. |
| **Leaderboards / Replays** | Top 10 per track and mission. Every finished race or mission saves a replay you can watch back (pause, seek, camera). |

## Drones and flight modes

- **Presets:** Trainer F450 (RflySim's default quad), Racer 5-inch (650 g, 5.5:1 thrust),
  Heavy Lifter (4 kg). Custom builds come from the Garage.
- **Beginner:** the sticks command velocity, and releasing them holds position and altitude.
- **Angle:** self-levelling. Mid throttle hovers; the drone drifts unless you correct it.
- **Acro:** rate mode for FPV racing. Rates and expo are set in Settings. Throttle must be down
  to arm.
- **Battery:** optional (Settings). Voltage sags as it drains, using hover power from momentum
  theory.

## Controls

| Action | Keyboard (Mode 2, default) | Gamepad (Mode 2) |
|---|---|---|
| Throttle / yaw | W/S, A/D | left stick |
| Pitch / roll | Up/Down, Left/Right (hold Shift for full stick) | right stick |
| Arm / disarm | Space | A |
| Flight mode | M | X |
| Camera (Chase, FPV, Orbit, Line of sight) | C | Y |
| Reset | R | Select |
| Pause / menu | Esc | Start |

- **Keyboard layout:** the default matches a Mode 2 transmitter: left hand on WASD for
  throttle and yaw, right hand on the arrows for pitch and roll. Settings → Keyboard layout
  switches to the "arcade" layout (W/S/A/D move, arrows throttle/yaw).
- **Mouse:** hold the right button for a virtual stick. Left-drag orbits the camera and the
  wheel zooms.
- **RC transmitter:** set it to USB-joystick mode and plug it in. Then go to Controls → *Map RC
  transmitter / joystick axes*, push each stick when asked, and press Enter.

## FPV first

The default camera is FPV (Settings → Default camera → Auto). Chase is used only in Beginner
mode, where seeing the drone helps you learn its orientation. C cycles FPV → Chase → Orbit →
Line of sight at any time.

The FPV view has a goggles-style OSD:
- artificial horizon and centre mark
- heading tape
- speed (km/h), height and climb rate
- pack voltage (cells × V/cell, sagging as the battery drains)
- throttle %, flight timer, mode, and armed/disarmed status

The camera is tilted up by the "FPV camera uptilt" setting (20° by default; racers use 25–40°).

## Physics and collisions

- **Sticks behave like a real flight controller:**
  - Stick forward = nose down and fly forward; roll right = bank and slide right; yaw right =
    nose right.
  - Angle and Beginner modes use quaternion attitude control, with yaw about the world
    vertical, so banked turns hold altitude.
  - Yaw has the lowest mixer priority, so a full yaw input never costs altitude.
- **Contact model:**
  - 11 contact points (4 feet, 4 prop hubs, body top/nose/tail) with spring-damper normal
    forces and friction on ground and roofs.
  - The body collides as a sphere with buildings, trees and gate bars.
  - Drones bounce, slide, tip over and come to rest. Nothing snaps or freezes.
- **Crash behaviour (Settings, default Realistic):**
  - **Realistic:** there is no crash screen. A hard impact makes the flight controller stop the
    motors, as real crash detection does, and the drone falls and settles. If it lands
    **upright**, press arm and fly on. If it lands **on its back**, press arm for **turtle mode**:
    reversed props flip it back over. Races carry on (R returns you to the last gate you passed),
    and missions and lessons only end on their clock.
  - **Arcade:** the old style, with a crash screen and reset (races respawn you at the last gate).
- **Damage rules (what counts as a hard impact):**
  - Feet touching down at more than 5 m/s is a hard landing.
  - Any prop hitting something while armed is a prop strike.
  - The body hitting the ground at more than 1.5 m/s is a crash.
  - Hitting a wall or gate at more than 2.5 m/s is a crash; slower is just a "Bump!".
  - Arcade only: coming to rest upside down is "Flipped over".

## Sound

All sounds are generated in code when the game starts, so there are no audio files.

- Motor hum: four looping layers, cross-faded by motor speed, so you hear the motors spool up and down.
- Wind: noise that gets louder with airspeed.
- Events: arm and disarm beeps, impact and bump thuds, race countdown and GO, gate chime, lap jingle, and
  a win or lose tune on the results screen.
- Volume: Settings → Sound volume (0–10, 0 = off).
- Sound is muted in `--auto` test runs; set `RFLY_SOUND=1` to hear it there.

## Split screen

Play together → Split screen. Up to four pilots on one Mac; each flies their own drone and
sees the others.

- **Join screen:** four player cards. Join with **Q** (left half of the keyboard), **U**
  (right half) or **A / Start** on a controller. Each pilot changes their own flight mode
  (Z, M or X); Angle and Acro fly FPV, Beginner uses the chase camera. Pick the activity
  (free flight on a map, or a race) in the menu above the cards, then Start.
- **Shared keyboard:**
  - Left pilot: W/S throttle, A/D yaw, T/G pitch, F/H roll; Q arm, E camera, Z mode, R reset,
    left Shift full stick.
  - Right pilot: I/K throttle, J/L yaw, arrows pitch/roll; U arm, O camera, M mode, P reset,
    right Shift full stick.
  - Controllers use the normal Mode 2 layout.
- **Layout:** 2 pilots split top/bottom; 3–4 in quarters. With 3, the spare quarter shows an
  overhead view of everyone and the standings.
  - Each view is drawn off-screen at 1.3× (2 pilots) or 1.6× (3–4) and scaled down, so
    the HUD shrinks to fit.
  - Wide views keep at least ~55° vertical field of view, so the uptilted FPV camera still
    shows the ground.
  - Stick widgets are hidden in quarter views.
- **Pause:** Esc or Start on any controller (Resume, Restart, Back to the join screen).
- **Results:** when everyone finishes a race, a combined results screen offers Race again.

Ideas taken from racing games with good split screen (e.g. Blur): every pilot gets a full,
slimmed-down HUD; join by pressing a button on your own controller; the spare quarter is
used for an overview rather than left black.

## Virtual controllers (scripts and tests)

Start the game with `--vpad` and up to four virtual controllers can be driven from a script
over UDP on `127.0.0.1:27170` (localhost only; off unless the flag is given). They behave like
real controllers everywhere: joining split screen, menus and flying. Virtual controller 1 also
drives single player.

```python
from rflyctl.vpad import VirtualPad      # python/rflyctl/vpad.py
p = VirtualPad(0)
p.press("A")                             # join / select / arm
p.sticks(throttle=0.5, pitch=0.3)        # Mode 2 sticks, -1..1
p.hold(3.0)                              # keep sending; a pad drops 2 s after its last packet
```

Buttons: A, B, X, Y, START, SELECT, UP, DOWN, LEFT, RIGHT. Packet format: `b"RVP1"`, u8 pad,
u16 buttons, then four f32 axes (lx, ly, rx, ry), little endian.
`python/examples/vpad_split_test.py` has four pads join split screen, pick a race and fly it.
macOS cannot create a system-wide virtual gamepad without an Apple-restricted entitlement,
so this lives inside the game. Only the OS → gilrs step of a real controller is not covered.

## Multiplayer details

- **Local network:** Play together → Local network. One player chooses *Host a game on this network*. Everyone else sees it in the Multiplayer list
  (UDP beacon on 27151), or joins by address.
- **Internet:** someone runs a server that the others can reach on UDP 27150, then players use
  *Join by address* (`host:27150`, or a host name). Any one of these works:
  - **Port forwarding:** host a game (or run `rflygame-server`) at home and forward UDP 27150
    on your router to that Mac. Friends join your public IP (search "what is my IP"). This does not work on a phone
    hotspot or most mobile connections (carrier-grade NAT): use a VPS there.
    Allow incoming connections if the macOS firewall asks.
  - **A small cloud server (VPS):** copy `dist/rflygame-server` (built by `scripts/package_rflygame.sh`
    on the same OS as the server, e.g. Linux), or build it there with
    `cargo build --release -p rflygame --bin rflygame-server`. Run
    `rflygame-server --port 27150 --name "My server" --no-lan` and open UDP 27150 in its firewall.
  - **A UDP tunnel service or a VPN** (e.g. a mesh VPN): join the address it gives you. A
    TCP-only tunnel (e.g. ngrok's free tier) will not work: the game uses UDP.
  - Each player picks their own flight mode in the lobby (Angle/Acro fly FPV; Beginner uses chase).
  - Everyone needs the same RflyGame version. Different protocol versions ignore each other.
- **Model:**
  - Each client simulates its own drone and sends its time-stamped state at 20 Hz.
  - The server relays snapshots and runs the lobby. The first player in is the host.
  - Clients ping every 0.5 s. That keeps idle (lobby, paused) players connected, shows the ping
    in the lobby and HUD, and re-joins automatically after an outage.
  - The host's start command is retransmitted until the server acknowledges it.
  - Other drones are drawn slightly in the past (70–300 ms, sized to that player's jitter),
    interpolated between states, so they move smoothly despite lost and reordered packets.
  - Race start times are synchronised from the server.
  - Standings are sorted by finish time, then by gates passed.
- **Trying internet conditions on one Mac:** `rflygame-netem` adds latency, jitter, loss and
  duplication. Host a game, run `rflygame-netem --delay 80 --jitter 40 --loss 0.05`, then join
  `127.0.0.1:27160` from a second RflyGame.

## Files

Everything is saved in the per-user data folder (set `RFLY_DATA_DIR` to use another one):

- macOS: `~/Library/Application Support/RflyGame/`
- Linux: `$XDG_DATA_HOME/rflygame/`, normally `~/.local/share/rflygame/`


| Path | Contents |
|---|---|
| `settings.txt` | Settings |
| `leaderboard.txt` | Leaderboard entries |
| `drones.txt` | Garage builds |
| `school.txt` | Flight School progress |
| `ghosts/` | Best-run ghosts |
| `replays/` | Replays |
| `logs/*.csv` | 50 Hz flight logs (Pause → *Save flight log*): position, velocity, attitude, rates, sticks, motors, battery |

## For developers

- `rflygame --auto race=N | mission=N | school=N | free | collide | bump | drop | host | join=ADDR`
  flies with a built-in autopilot.
- `--quit-after S` stops the run after S seconds; `--screenshot out.png --after S` saves a
  screenshot.
- Results are printed to stdout. The test suite uses this.
