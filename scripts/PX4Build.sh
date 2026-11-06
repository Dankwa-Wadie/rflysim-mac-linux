#!/usr/bin/env bash
# PX4Build (Windows: PX4CMD in MATLAB / the PX4 firmware build of RflySim's installer): builds PX4 firmware
# for a Pixhawk board and optionally uploads it over USB.
#
# Usage: scripts/PX4Build.sh [BOARD] [--upload] [--px4 DIR] [--list]
#   BOARD     PX4 build target, default px4_fmu-v5_default (Pixhawk 4). RflySim's older names work too
#             (px4-fmu-v5_default → px4_fmu-v5_default). Common: px4_fmu-v2_default (Pixhawk 1, 1 MB flash),
#             px4_fmu-v3_default, px4_fmu-v5_default, px4_fmu-v6c_default, px4_fmu-v6x_default, cuav_nora_default
#   --upload  flash it to the Pixhawk on USB afterwards (close QGroundControl first)
#   --list    list the boards this PX4 checkout can build
#   --px4     PX4-Autopilot checkout (default ~/tools/PX4-Autopilot or $PX4_DIR)
# Needs the GNU Arm Embedded compiler (arm-none-eabi-gcc):
#   macOS: brew install --cask gcc-arm-embedded
#   Linux: sudo apt install gcc-arm-none-eabi   (or PX4's Tools/setup/ubuntu.sh)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/scripts/platform.sh"
BOARD="px4_fmu-v5_default"; UPLOAD=0; LIST=0; PX4_DIR="${PX4_DIR:-$HOME/tools/PX4-Autopilot}"
while [ $# -gt 0 ]; do
  case "$1" in
    --upload) UPLOAD=1 ;;
    --list) LIST=1 ;;
    --px4) PX4_DIR="$2"; shift ;;
    -h|--help) sed -n 2,15p "$0"; exit 0 ;;
    -*) echo "PX4Build: unknown option $1"; exit 1 ;;
    *) BOARD="$1" ;;
  esac
  shift
done
BOARD="${BOARD/px4-fmu/px4_fmu}"   # RflySim's PX4 1.7/1.8-era names
[ -f "$PX4_DIR/Makefile" ] || { echo "PX4-Autopilot not found at $PX4_DIR (clone it, see the README, or pass --px4 DIR)"; exit 1; }
# PX4's Python build dependencies (the same venv SITL uses), if there is one.
for v in "${PX4_VENV:-}" "$HOME/tools/px4-venv"; do
  [ -n "$v" ] && [ -x "$v/bin/python" ] && { export PATH="$v/bin:$PATH"; break; }
done
# PX4 lists default variants as "board[_default]" (the suffix is optional).
targets() { (cd "$PX4_DIR" && make list_config_targets 2>/dev/null) | sed 's/\[_default\]$/_default/' | grep -vE 'bootloader|canbootloader'; }
if [ "$LIST" = 1 ]; then
  targets | grep -E '_default$' | sort
  exit 0
fi
if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  echo "The Arm compiler (arm-none-eabi-gcc) is not installed. Install it once:"
  [ "$RFLY_OS" = macos ] && echo "  brew install --cask gcc-arm-embedded" || echo "  sudo apt install gcc-arm-none-eabi"
  exit 1
fi
case "$BOARD" in *_*_*|*_default) ;; *) BOARD="${BOARD}_default" ;; esac
if ! targets | grep -qx "$BOARD"; then
  echo "PX4Build: '$BOARD' is not a board in this PX4 checkout. scripts/PX4Build.sh --list shows them."; exit 1
fi
echo "PX4Build: building $BOARD in $PX4_DIR ($(arm-none-eabi-gcc -dumpversion)) ..."
(cd "$PX4_DIR" && make "$BOARD") || { echo "PX4Build: build failed"; exit 1; }
FW="$(ls "$PX4_DIR/build/$BOARD/"*.px4 2>/dev/null | head -1)"
echo "PX4Build: firmware: ${FW:-$PX4_DIR/build/$BOARD}"
if [ "$UPLOAD" = 1 ]; then
  echo "PX4Build: uploading (plug in the Pixhawk now if it is not connected; close QGroundControl) ..."
  (cd "$PX4_DIR" && make "$BOARD" upload) || { echo "PX4Build: upload failed"; exit 1; }
  echo "PX4Build: done. For HIL, set SYS_AUTOSTART 1001 (HIL Quadcopter X) in QGroundControl, then use HITLRun."
else
  echo "PX4Build: flash it with --upload, or with QGroundControl (Vehicle Setup → Firmware → Custom firmware file)."
fi
