#!/usr/bin/env bash
# Builds the "RflyTools" folder of launchers with icons, like RflySim's Windows RflyTools shortcuts folder.
#   macOS: .app bundles, linked to ~/Applications/RflyTools
#   Linux: .desktop launchers + bin/ launcher scripts + PNG icons, listed in the app menu (~/.local/share/applications)
#
# Usage: scripts/build_apps.sh [OUT_DIR] [--root DIR] [--package]
#   OUT_DIR    where the launchers are written (default ./RflyTools)
#   --root     where RflyTools lives when the launchers run (default: this checkout).
#   --package  only write files (for scripts/package.sh): no menu entries, no ~/Applications link, no QGC link.
#              Linux: OUT_DIR is staged for $ROOT/RflyTools, so icon paths point there.
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/scripts/platform.sh"
OUT=""; RROOT="$ROOT"; PACKAGE=0
while [ $# -gt 0 ]; do
  case "$1" in
    --root) RROOT="$2"; shift ;;
    --package) PACKAGE=1 ;;
    -h|--help) sed -n 2,10p "$0"; exit 0 ;;
    *) OUT="$1" ;;
  esac
  shift
done
OUT="${OUT:-$ROOT/RflyTools}"
LOUT="$OUT"; [ "$PACKAGE" = 1 ] && LOUT="$RROOT/RflyTools"   # where OUT ends up at run time
PY="${RFLY_VENV:-$HOME/tools/rflysim-py}/bin/python"
ICONS="$ROOT/build/icons"
VER="$(grep -m1 '^version' "$ROOT/Cargo.toml" | cut -d'"' -f2)"
[ "$PACKAGE" = 1 ] || rfly_need_bins coptersim rflysim3d rflygame
if [ ! -f "$ICONS/ImportData.png" ] || [ "$ROOT/scripts/make_icons.py" -nt "$ICONS/ImportData.png" ]; then
  [ -x "$PY" ] || "$ROOT/scripts/Python38Env.sh" -c "print('env ok')"
  "$PY" "$ROOT/scripts/make_icons.py" "$ICONS" >/dev/null
fi

# Launcher body: resolves binaries and logs at run time through platform.sh, so the same launcher works
# from a checkout (target/release, ./logs) and an installed package (bin/, per-user logs).
# body NAME MODE COMMAND
body() {
  printf '#!/usr/bin/env bash\n# RflyTools launcher: %s\nROOT="%s"\n. "$ROOT/scripts/platform.sh"\n' "$1" "$RROOT"
  printf 'if rfly_is_repo; then cd "$ROOT"; else cd "$HOME"; fi\n'
  if [ "$2" = gui ]; then
    printf 'exec %s >>"$(rfly_logs)/%s.app.log" 2>&1\n' "$3" "$1"
  else
    printf '%s\nstatus=$?\necho; read -rp "[%s exited ($status). Press Enter to close this window] " _\n' "$3" "$1"
  fi
}

mkicns() { # name → $ICONS/name.icns
  local set="$ICONS/$1.iconset"; rm -rf "$set"; mkdir -p "$set"
  for s in 16 32 128 256 512; do
    sips -z $s $s "$ICONS/$1.png" --out "$set/icon_${s}x${s}.png" >/dev/null
    sips -z $((s * 2)) $((s * 2)) "$ICONS/$1.png" --out "$set/icon_${s}x${s}@2x.png" >/dev/null
  done
  iconutil -c icns "$set" -o "$ICONS/$1.icns"; rm -rf "$set"
}

# app NAME ICON MODE COMMAND   (MODE: gui = run directly, term = run in a terminal window)
app_macos() {
  local name="$1" icon="$2" mode="$3" cmd="$4" dir="$OUT/$1.app/Contents"
  rm -rf "$OUT/$name.app"; mkdir -p "$dir/MacOS" "$dir/Resources"
  mkicns "$icon"; cp "$ICONS/$icon.icns" "$dir/Resources/AppIcon.icns"
  local id; id="org.rflysim-mac.$(echo "$name" | tr -cd '[:alnum:]-')"
  cat >"$dir/Info.plist" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleName</key><string>$name</string>
  <key>CFBundleDisplayName</key><string>$name</string>
  <key>CFBundleIdentifier</key><string>$id</string>
  <key>CFBundleExecutable</key><string>launch</string>
  <key>CFBundleIconFile</key><string>AppIcon</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>$VER</string>
  <key>CFBundleVersion</key><string>$VER</string>
  <key>LSMinimumSystemVersion</key><string>11.0</string>
  <key>NSHighResolutionCapable</key><true/>
</dict></plist>
EOF
  if [ "$mode" = term ]; then
    body "$name" term "$cmd" >"$dir/Resources/run.command"
    chmod +x "$dir/Resources/run.command"
    printf '#!/usr/bin/env bash\nopen -a Terminal "$(dirname "$0")/../Resources/run.command"\n' >"$dir/MacOS/launch"
  else
    body "$name" gui "$cmd" >"$dir/MacOS/launch"
  fi
  chmod +x "$dir/MacOS/launch"
  echo "app: $OUT/$name.app"
}

APPS_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
app_linux() {
  local name="$1" icon="$2" mode="$3" cmd="$4" term=false file="${1// /}"   # file names without spaces
  mkdir -p "$OUT/bin" "$OUT/icons"
  cp "$ICONS/$icon.png" "$OUT/icons/$icon.png"
  [ "$mode" = term ] && term=true
  body "$name" "$mode" "$cmd" >"$OUT/bin/$file"
  chmod +x "$OUT/bin/$file"
  cat >"$OUT/$file.desktop" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=$name
Comment=RflyTools: $name
Exec="$LOUT/bin/$file"
Icon=$LOUT/icons/$icon.png
Terminal=$term
Categories=$([ "$name" = RflyGame ] && echo "Game;Simulation;" || echo "Science;Simulation;")
StartupWMClass=$name
EOF
  chmod +x "$OUT/$file.desktop"
  if [ "$PACKAGE" = 0 ]; then mkdir -p "$APPS_DIR" && ln -sfn "$OUT/$file.desktop" "$APPS_DIR/rflytools-$file.desktop"; fi
  echo "app: $OUT/$file.desktop"
}

app() { if [ "$RFLY_OS" = macos ]; then app_macos "$@"; else app_linux "$@"; fi; }

mkdir -p "$OUT"
if [ "$RFLY_OS" = macos ]; then
  ASK_N='N=$(osascript -e '"'"'text returned of (display dialog "Number of vehicles:" default answer "1" with title "SITLRun")'"'"' 2>/dev/null) || exit 0'
else
  ASK_N='read -rp "SITLRun: number of vehicles [1]: " N; N="${N:-1}"'
fi
app "RflyGame"            RflyGame        gui  '"$(rfly_bin rflygame)"'
app "RflySim3D"           RflySim3D       gui  '"$(rfly_bin rflysim3d)"'
app "3DDisplay"           3DDisplay       gui  '"$(rfly_bin rflysim3d)" --map Blank'
app "FlightGear-F450"     FlightGear-F450 term '"$ROOT/scripts/FlightGearF450.sh"'
app "CopterSim"           CopterSim       gui  '"$(rfly_bin coptersim-ui)"'
app "SITLRun"             SITLRun         term "$ASK_N
\"\$ROOT/scripts/SITLRun.sh\" \"\$N\" --qgc"
app "SITLRunLowGPU"       SITLRun         term "$ASK_N
\"\$ROOT/scripts/SITLRun.sh\" \"\$N\" --qgc --low-gpu"
app "SITLRunFw"           SITLRun         term "$ASK_N
\"\$ROOT/scripts/SITLRun.sh\" \"\$N\" --qgc --fixed-wing"
app "APMSITLRun"          SITLRun         term "$ASK_N
\"\$ROOT/scripts/APMSITLRun.sh\" \"\$N\" --qgc"
app "APMSITLRun_Plane"    SITLRun         term "$ASK_N
\"\$ROOT/scripts/APMSITLRun.sh\" \"\$N\" --qgc --plane"
app "HITLRun"             HITLRun         term '"$ROOT/scripts/HITLRun.sh" --qgc'
app "HITLRunFw"           HITLRun         term '"$ROOT/scripts/HITLRun.sh" --qgc --fixed-wing'
app "HITLRunLowGPU"       HITLRun         term '"$ROOT/scripts/HITLRun.sh" --qgc --low-gpu'
app "CloseALL"            CloseALL        term '"$ROOT/scripts/CloseALL.sh"'
app "Python38Env"         Python38Env     term '"$ROOT/scripts/Python38Env.sh"'
app "Import RflySim Data" ImportData      term '"$ROOT/scripts/import_rflysim_data.sh"'
app "Update RflyTools"    Update          term '"$(rfly_bin rfly-update)"'
[ "$PACKAGE" = 1 ] && { echo "RflyTools launchers staged in $OUT (for $LOUT)"; exit 0; }
if [ "$RFLY_OS" = macos ]; then
  [ -d /Applications/QGroundControl.app ] && ln -sfn /Applications/QGroundControl.app "$OUT/QGroundControl.app"
  mkdir -p "$HOME/Applications" && ln -sfn "$OUT" "$HOME/Applications/RflyTools"
  for a in "$OUT"/*.app; do [ -L "$a" ] || touch "$a"; done
  echo "RflyTools: $OUT (also ~/Applications/RflyTools)"
else
  QGC_EXE="$(find_qgc_linux)"
  if [ -n "$QGC_EXE" ]; then app "QGroundControl" SITLRun gui "\"$QGC_EXE\""; fi
  if command -v update-desktop-database >/dev/null 2>&1; then update-desktop-database "$APPS_DIR" >/dev/null 2>&1 || true; fi
  echo "RflyTools: $OUT (also in the app menu via $APPS_DIR/rflytools-*.desktop)"
fi
