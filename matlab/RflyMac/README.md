# RflyMac: MATLAB/Simulink package for rflysim_macV1 (macOS)

Simulink becomes the **controller** and `coptersim` becomes the **drone**. They run in lockstep:
each Simulink step sends motor commands, and the simulator advances by exactly one step and returns the sensors and state.
This is RflySim's "controller + plant" desktop test (`px4demo_attitude_plant`), with our physics in place of
the Simulink 6-DOF block. It needs no toolboxes beyond Simulink.

## Requirements
- MATLAB + Simulink for macOS (Apple Silicon or Intel).
- Xcode, as the C compiler for MEX. Run `mex -setup C` once in MATLAB.
- `coptersim` built from this repo (`cargo build --release`).

## Install (once MATLAB is ready)
```matlab
cd ~/projects/rflysim_macV1/matlab/RflyMac
install_rflymac('save')      % compiles 2 MEX files, adds paths, builds the library + example model
```

## Use
1. Start the simulator in a terminal:
   ```sh
   ~/projects/rflysim_macV1/target/release/coptersim --ext     # UDP 20100, lockstep
   ```
2. In MATLAB:
   ```matlab
   rflymac_quicktest                          % script-only test: climb 5 m, roll + yaw steps, plots
   open_system('rflymac_example_attitude')    % Simulink example, then press Run
   ```
3. Write your own controller. Edit the **Controller** MATLAB Function block, or copy
   `rflymac_attitude_ctrl.m`. Inputs: setpoints + euler/rates/pos/vel. Output: 8 motor throttles in the range 0..1.

## Contents
| File | Purpose |
|---|---|
| `install_rflymac.m` | Build + install |
| `src/sfun_rflymac.c` | **CopterSim (Mac)** Simulink block (C-MEX S-function) |
| `src/rflymac_step_mex.c` | `rflymac_step(...)` for plain MATLAB scripts |
| `src/rflymac_client.[ch]` | UDP client shared by both (protocol: `crates/coptersim/src/ext.rs`) |
| `rflymac_attitude_ctrl.m` | Example controller: altitude cascade + angle-P/rate-PI per axis, PX4 quad-x mixer |
| `rflymac_build_library.m`, `rflymac_build_example.m` | Generate `models/rflymac_lib.slx` and `models/rflymac_example_attitude.slx` |
| `rflymac_quicktest.m` | Scripted 15 s test with pass/fail checks and plots |
| `rflymac_fields.m` | Index map of the 39-value state vector |
| `tests/test_client.c` | Same test in C (runs without MATLAB: `scripts/test_rflymac_client.sh`) |

## CopterSim (Mac) block
- **Input:** motor throttles in 0..1, 1 to 8 values, PX4 order (quad-x: 1 FR, 2 RL, 3 FL, 4 RR).
- **Outputs:** time · pos NED · vel NED · quat · euler · rates · accel · gyro · mag · baro alt · gps · motor speeds · on ground.
- **Parameters:**
  - UDP port, default 20100. This was RflySim CopterSim's SIL port.
  - Sample time Ts, default 4 ms. It must be a multiple of 1 ms.
  - Host.
- **Feedthrough:** the block is not direct-feedthrough, so feeding outputs back into a controller doesn't create an algebraic loop.
- **Reset:** each simulation run resets the drone to the ground at t = 0.

## Status
- Verified without MATLAB (`scripts/test_rflymac_client.sh`):
  - The C client runs against `coptersim --ext`.
  - The reference controller passes all checks: lockstep, 5 m climb, level hover, roll and yaw step tracking.
  - 15 s of flight runs in ~0.6 s.
- The S-function and MEX sources compile against stub headers.
- **Not yet run inside MATLAB.** The `.m` scripts, model generation and the MEX build need a real MATLAB session.
