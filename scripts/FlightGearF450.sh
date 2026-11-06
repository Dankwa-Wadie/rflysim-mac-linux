#!/usr/bin/env bash
# FlightGear-F450 for macOS and Linux: the RflyTools "FlightGear-F450" shortcut.
# Shows coptersim's vehicle in FlightGear with RflySim's F450 model (extracted from the ISO into
# assets/flightgear/Aircraft/F450, or imported with scripts/import_rflysim_data.sh). Start coptersim with
# `--fg` (sends to 127.0.0.1:5502).
#
# One-time setup:
#   macOS: brew install --cask flightgear, plus FGData (1.9 GB) unpacked into ~/tools/fgdata (or set FG_ROOT)
#   Linux: sudo apt install flightgear   (Debian/Ubuntu; brings fgfs and FGData in /usr/share/games/flightgear)
#          or any fgfs on PATH / $FGFS with FGData in ~/tools/fgdata or $FG_ROOT
#
# Usage: scripts/FlightGearF450.sh [--terrasync] [extra fgfs options]
#   FGData 2024 ships scenery for Hawaii and Iceland only. Fly to Honolulu (PHNL)
#   (coptersim --lat=21.3187 --lon=-157.9224 --alt=4), or pass --terrasync to download
#   scenery around PX4's default home (Zurich) on the fly.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/scripts/platform.sh"
FGFS="${FGFS:-$(command -v fgfs || echo /Applications/FlightGear.app/Contents/MacOS/FlightGear)}"
if [ -z "${FG_ROOT:-}" ]; then
  FG_ROOT="$HOME/tools/fgdata"
  for d in /usr/share/games/flightgear /usr/share/flightgear /usr/local/share/flightgear; do
    [ -f "$FG_ROOT/version" ] || { [ -f "$d/version" ] && FG_ROOT="$d"; }
  done
fi
if [ ! -x "$FGFS" ]; then
  case "$(uname -s)" in
    Darwin) echo "FlightGear not found. Install: brew install --cask flightgear" ;;
    *) echo "FlightGear (fgfs) not found. Install: sudo apt install flightgear (or set FGFS)" ;;
  esac
  exit 1
fi
[ -f "$FG_ROOT/version" ] || { echo "FGData not found at $FG_ROOT (set FG_ROOT)"; exit 1; }
AIRCRAFT="$(rfly_fg_aircraft)"
[ -n "$AIRCRAFT" ] || { echo "RflySim F450 model not found. Import it from your RflySim ISO: scripts/import_rflysim_data.sh ISO"; exit 1; }
TS="--disable-terrasync"
if [ "${1:-}" = "--terrasync" ]; then TS="--enable-terrasync"; shift; fi
# Same core flags as RflySim's FlightGear-F450.bat: external FDM over native-fdm UDP 5502.
exec "$FGFS" --fg-root="$FG_ROOT" --fg-aircraft="$AIRCRAFT" --aircraft=F450 \
  --fdm=null --native-fdm=socket,in,30,,5502,udp \
  $TS --timeofday=noon --disable-ai-traffic --disable-real-weather-fetch --disable-sound \
  --geometry=1280x720 "$@"
