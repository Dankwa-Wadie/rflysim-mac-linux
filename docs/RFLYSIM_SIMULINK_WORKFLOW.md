# How RflySim uses Simulink, and our MATLAB-free equivalent

Sources:
- The PX4 PSP from the ISO: `3.PX4PSP/PX4 PSP.zip` and `PSPUpdate.zip`, MathWorks add-on `PX4PSP_v3_0_4`, R2017b.
  Models were read from the `.slx` XML.
- CopterSim's command-line behaviour and UI text.

Legend: **[O]** observed · **[I]** inference

## 1. The core pattern: one controller, two harnesses

The controller is a **separate Simulink model** (`px4demo_attitude_control.slx`). Two different
top-level models include it as a `ModelReference` block named `ControlSubsystem` [O]:

```
                      ┌──────────────────────────────┐
                      │  ControlSubsystem (one .slx) │
                      │  ARMINGControl               │
                      │  InputConditioning           │
                      │  AttitudeControl (5 PIDs:    │
                      │   roll/pitch angle + rate,   │
                      │   yaw rate) → motor mixing   │
                      └──────────────┬───────────────┘
              ┌──────────────────────┴──────────────────────┐
   (A) DESKTOP TEST: px4demo_attitude_plant      (B) DEPLOY: px4demo_attitude_system
   ControlSubsystem + dynamic_model:             ControlSubsystem + hardware blocks:
   - Aerospace Blockset "6DoF" rigid body        - input_rc         (RC sticks in)
   - ISA atmosphere                              - sensor_combined  (IMU in)
   - "Invensense MPU-6000" IMU model             - vehicle_attitude (PX4 attitude estimate in)
   - Delay blocks = sensor latency               - PWM_output       (motors out)
   - Step blocks = test stick inputs             - RGB_LED, Speaker_Tune (status)
   - Scopes = plots
   Runs entirely inside Simulink.                Embedded Coder → C → PX4 firmware module.
```

You design and tune in (A). Then you build (B) with the **same** controller file, so the code that
flies is exactly what you tested.

## 2. From Simulink to firmware (the PSP)

1. **Block library** (`blocks/pixhawk_slib_*.slx`) [O]: sensors/actuators (input_rc, sensor_combined,
   vehicle_attitude, GPS, battery, PWM, AUX, RGB LED, tune), generic uORB read/write, and parameter update.
2. **Code generation**: every block has a TLC file that turns it into PX4 C API calls [O].
   - `sfun_px4_pwm.tlc` opens `PWM_OUTPUT0_DEVICE_PATH`, calls `px4_ioctl(PWM_SERVO_SET_ARM_OK…)`,
     and publishes `actuator_outputs`/`actuator_armed` via uORB.
   - The uORB blocks call `orb_subscribe`/`orb_publish` for any topic. `uORB_msgToBus.m` turns PX4 `.msg`
     files into Simulink bus types.
3. **PX4 module**: the generated code is wrapped as the PX4 module `px4_simulink_app` (`MODULE modules__px4_simulink_app`)
   and added to the firmware's CMake board config [O].
4. **Build + upload**: the MSYS2/Cygwin/WSL toolchain from the ISO builds PX4 and uploads it to the Pixhawk [O].
   On the board, the Simulink controller drives the PWM outputs directly in place of PX4's own controllers [I].

## 3. Testing the generated firmware (CopterSim)

- **HIL**: the Pixhawk running the Simulink firmware connects over USB. CopterSim simulates the drone and sends
  `HIL_SENSOR`/`HIL_GPS`, then reads back motor outputs. 3DDisplay shows the flight [O: CopterSim modes `PX4_HITL`].
- **SITL**: the same loop runs against PX4 on the PC [O: `PX4_SITL`, `PX4_SITL_RFLY`].
- **"Simulink&DLL_SIL"** (vehicle model compiled from Simulink into a DLL that CopterSim loads):
  in this free build it is **not functional**: it falls back to mode 0, the same as `PX4_HITL` [O].
  The UI points to RflySim's advanced (paid) platform for it [O].
  It belongs to the paid version.
- CopterSim CLI [O]: `argv[5]` = sim mode (`PX4_HITL`|`PX4_SITL`|`PX4_SITL_RFLY`|
  `Simulink&DLL_SIL` or a number), `argv[6]` = display (`3DDisplay` or a map name under `external/map`).

## 4. Our MATLAB-free equivalent (proposal)

Keep the pattern ("one controller, two harnesses") and replace Simulink with code:

| RflySim + Simulink | rflysim_macV1 |
|---|---|
| `ControlSubsystem` .slx | A **controller** written against a small trait with the same I/O as the PSP blocks (RC, IMU, attitude in → motor commands out) |
| (A) plant harness: Simulink 6DoF + IMU + scopes | **Model-in-the-loop**: the controller runs inside `coptersim` against `rfly-core` physics, faster than real time, logs to CSV for plotting |
| (B) Embedded Coder → `px4_simulink_app` | Run the same controller against **PX4 SITL** over MAVLink offboard. Later, optionally, as a C++ PX4 module |
| Scopes | CSV logs + PlotJuggler / Python (matplotlib) |
| Step blocks for test inputs | Scripted test scenarios (step inputs, missions) |
| uORB read/write blocks | Full access to PX4 telemetry and parameters via MAVLink |
| Simulink-built vehicle DLL (paid) | `rfly-core` model plugins: Rust, or a C-ABI `.dylib`, so a Simulink-generated model still fits later |
