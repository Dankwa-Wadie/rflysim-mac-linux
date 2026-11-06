#!/usr/bin/env bash
# Python38Env for macOS and Linux: the RflyTools "Python38Env" shortcut.
# Creates (first run) a venv with pymavlink, numpy and opencv, and puts the Mac/Linux RflySimSDK
# (python/RflySimSDK_mac: PX4MavCtrlV4, UE4CtrlAPI, VisionCaptureApi) on its path.
#
#   scripts/Python38Env.sh                 # interactive shell inside the env
#   scripts/Python38Env.sh my_script.py    # run a script with the env
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VENV="${RFLY_VENV:-$HOME/tools/rflysim-py}"
if [ ! -x "$VENV/bin/python" ]; then
  echo "Python38Env: creating $VENV ..."
  python3 -m venv "$VENV" || { echo "Python38Env: venv failed. Debian/Ubuntu: sudo apt install python3-venv"; exit 1; }
  "$VENV/bin/pip" install -q --upgrade pip
  "$VENV/bin/pip" install -q pymavlink numpy opencv-python bdsim
fi
SITE="$("$VENV/bin/python" -c 'import sysconfig;print(sysconfig.get_paths()["purelib"])')"
echo "$ROOT/python/RflySimSDK_mac" >"$SITE/rflysim_sdk_mac.pth"
if [ $# -gt 0 ]; then
  exec "$VENV/bin/python" "$@"
fi
echo "Python38Env: $("$VENV/bin/python" --version), RflySimSDK_mac on path. 'exit' to leave."
exec env VIRTUAL_ENV="$VENV" PATH="$VENV/bin:$PATH" PS1="(rflysim) \w \$ " bash --norc -i
