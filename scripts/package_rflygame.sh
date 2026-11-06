#!/usr/bin/env bash
# Packages RflyGame so it runs without the repo.
# macOS:
#   dist/RflyGame.app          game + rflygame-server + icon (ad-hoc signed)
#   dist/RflyGame-macOS.zip    the same, zipped for sharing
#   dist/rflygame-server       standalone server for a LAN box or internet host
# Linux:
#   dist/RflyGame-linux-ARCH/          rflygame, rflygame-server, icon, .desktop, install.sh
# RflySim's ModelData.db is not bundled (it is RflySim's file); the Garage uses the copy imported
# with scripts/import_rflysim_data.sh. For all tools in one installer, use scripts/package.sh.
#   dist/RflyGame-linux-ARCH.tar.gz    the same, for sharing (unpack anywhere; ./install.sh adds a menu entry)
#   dist/rflygame-server               standalone server
# Usage: scripts/package_rflygame.sh
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
[ -f "$HOME/.cargo/env" ] && . "$HOME/.cargo/env"
cargo build --release -p rflygame
VER="$(grep -m1 '^version' Cargo.toml | cut -d'"' -f2)"
PY="${RFLY_VENV:-$HOME/tools/rflysim-py}/bin/python"
DIST="$ROOT/dist"; mkdir -p "$DIST"

if [ "$(uname -s)" != Darwin ]; then
  ARCH="$(uname -m)"; NAME="RflyGame-linux-$ARCH"; PKG="$DIST/$NAME"
  rm -rf "$PKG"; mkdir -p "$PKG"
  cp target/release/rflygame target/release/rflygame-server "$PKG/"
  if [ -x "$PY" ] && "$PY" scripts/make_icons.py build/icons >/dev/null 2>&1; then
    cp build/icons/RflyGame.png "$PKG/rflygame.png"
  else
    echo "note: no icon (needs the Python38Env venv with opencv: scripts/Python38Env.sh -c pass)"
  fi
  cat >"$PKG/install.sh" <<'EOF'
#!/usr/bin/env sh
# Adds RflyGame to your app menu (per user, no root). Run again after moving this folder.
set -eu
DIR="$(cd "$(dirname "$0")" && pwd)"
APPS="${XDG_DATA_HOME:-$HOME/.local/share}/applications"; mkdir -p "$APPS"
cat >"$APPS/rflygame.desktop" <<DESKTOP
[Desktop Entry]
Type=Application
Name=RflyGame
Comment=FPV drone racing, missions and flight school
Exec="$DIR/rflygame"
Path=$DIR
Icon=$DIR/rflygame.png
Terminal=false
Categories=Game;Simulation;
StartupWMClass=RflyGame
DESKTOP
command -v update-desktop-database >/dev/null 2>&1 && update-desktop-database "$APPS" >/dev/null 2>&1 || true
echo "RflyGame added to the app menu ($APPS/rflygame.desktop)."
EOF
  chmod +x "$PKG/install.sh"
  cat >"$PKG/README.txt" <<EOF
RflyGame $VER for Linux ($ARCH)

Run:      ./rflygame            (or ./install.sh once for an app-menu entry)
Server:   ./rflygame-server --help
Needs:    OpenGL 2.1+, X11 or XWayland, libxkbcommon, ALSA (libasound2), libudev1. Debian/Ubuntu:
          sudo apt install libasound2 libudev1 libgl1 libx11-6 libxi6 libxkbcommon0
          (libasound2t64 on Ubuntu 24.04+)
Gamepads: your user needs read access to /dev/input/event* (the "input" group on some distros).
Saves:    ~/.local/share/rflygame (or \$XDG_DATA_HOME/rflygame)
EOF
  cp target/release/rflygame-server "$DIST/"
  tar -C "$DIST" -czf "$DIST/$NAME.tar.gz" "$NAME"
  echo "Packaged: $PKG ($(du -sh "$PKG" | cut -f1)), $DIST/$NAME.tar.gz ($(du -h "$DIST/$NAME.tar.gz" | cut -f1)), $DIST/rflygame-server"
  echo "glibc needed: $(objdump -T "$PKG/rflygame" 2>/dev/null | grep -o 'GLIBC_[0-9.]*' | sort -uV | tail -1)"
  exit 0
fi

# Universal (Apple Silicon + Intel) when the Intel target is installed: rustup target add x86_64-apple-darwin
BIN="$ROOT/build/universal"; mkdir -p "$BIN"
if rustup target list --installed 2>/dev/null | grep -q x86_64-apple-darwin; then
  cargo build --release -p rflygame --target x86_64-apple-darwin
  for b in rflygame rflygame-server; do
    lipo -create "target/release/$b" "target/x86_64-apple-darwin/release/$b" -output "$BIN/$b"
  done
else
  cp target/release/rflygame target/release/rflygame-server "$BIN/"
fi
"$PY" scripts/make_icons.py build/icons >/dev/null

APP="$DIST/RflyGame.app"
rm -rf "$APP"; mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources"
cp "$BIN/rflygame" "$BIN/rflygame-server" "$APP/Contents/MacOS/"
set_dir="build/icons/RflyGame.iconset"; rm -rf "$set_dir"; mkdir -p "$set_dir"
for s in 16 32 128 256 512; do
  sips -z $s $s build/icons/RflyGame.png --out "$set_dir/icon_${s}x${s}.png" >/dev/null
  sips -z $((s * 2)) $((s * 2)) build/icons/RflyGame.png --out "$set_dir/icon_${s}x${s}@2x.png" >/dev/null
done
iconutil -c icns "$set_dir" -o "$APP/Contents/Resources/AppIcon.icns"; rm -rf "$set_dir"
cat >"$APP/Contents/Info.plist" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleName</key><string>RflyGame</string>
  <key>CFBundleDisplayName</key><string>RflyGame</string>
  <key>CFBundleIdentifier</key><string>org.rflysim-mac.RflyGame</string>
  <key>CFBundleExecutable</key><string>rflygame</string>
  <key>CFBundleIconFile</key><string>AppIcon</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>$VER</string>
  <key>CFBundleVersion</key><string>$VER</string>
  <key>LSMinimumSystemVersion</key><string>11.0</string>
  <key>LSApplicationCategoryType</key><string>public.app-category.simulation-games</string>
  <key>NSHighResolutionCapable</key><true/>
</dict></plist>
EOF
codesign --force --deep -s - "$APP" 2>/dev/null || echo "note: codesign unavailable, app is unsigned"
cp "$BIN/rflygame-server" "$DIST/"
rm -f "$DIST/RflyGame-macOS.zip"
ditto -c -k --keepParent "$APP" "$DIST/RflyGame-macOS.zip"
echo "Packaged: $APP ($(du -sh "$APP" | cut -f1)), $DIST/RflyGame-macOS.zip ($(du -h "$DIST/RflyGame-macOS.zip" | cut -f1)), $DIST/rflygame-server"
echo "Architecture: $(lipo -archs "$APP/Contents/MacOS/rflygame")"
