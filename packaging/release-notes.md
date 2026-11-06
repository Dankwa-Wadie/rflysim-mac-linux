RflyTools VERSION: RflySim's tools for macOS and Linux, plus RflyGame.

| System | Download |
|---|---|
| macOS 11+ (Apple Silicon and Intel) | `RflyTools-VERSION-macOS.pkg` |
| Debian / Ubuntu x86_64 | `rflytools_VERSION_amd64.deb` (`sudo apt install ./rflytools_VERSION_amd64.deb`) |
| Debian / Ubuntu arm64 | `rflytools_VERSION_arm64.deb` |
| Other Linux | `rflytools-VERSION-linux-x86_64.tar.gz` / `-aarch64.tar.gz`: unpack, run `./install.sh` |

**Already installed?** RflyTools 0.2.0 and later update themselves: run `rfly-update` (or *Update RflyTools*), choose *Update* in RflyGame, or on Debian/Ubuntu `sudo apt upgrade`. Older versions: install this package once.

**After installing,** open *Import RflySim Data* (or run `rfly-import-data YOUR.iso`) once to add RflySim's
vehicle database and F450 FlightGear model from your own RflySim ISO. They are RflySim's files and not included.
SITLRun also needs PX4 SITL in `~/tools/PX4-Autopilot` (see the README).

**Included:** `coptersim`, `rflysim3d` (RflySim3D/3DDisplay), RflyGame + `rflygame-server`, SITLRun, HITLRun,
Python38Env with the RflySimSDK port, FlightGear-F450, the RflyMac Simulink package, app/menu launchers.

**Notes**
- The macOS package is not notarized. If macOS blocks it: right-click → Open, or System Settings → Privacy & Security → Open Anyway.
- Linux packages need glibc 2.34+ (Ubuntu 22.04+, Debian 12+) and were install-tested on Ubuntu 24.04 (x86_64) and Debian 12 (arm64).
- Checksums: `SHA256SUMS.txt`.

See [COMPARISON.md](https://github.com/Dankwa-Wadie/rflysim-mac-linux/blob/main/docs/COMPARISON.md) for how this compares with the Windows original.

Release files are signed: `SHA256SUMS.txt.sig` (Ed25519) covers `SHA256SUMS.txt`; `rfly-update` checks both before installing anything.
