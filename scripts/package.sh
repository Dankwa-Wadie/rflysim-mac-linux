#!/usr/bin/env bash
# Builds installable RflyTools packages (all tools: CopterSim, RflySim3D/3DDisplay, RflyGame + server,
# SITLRun, HITLRun, Python38Env + RflySimSDK_mac, FlightGear-F450, Import RflySim Data, RflyMac for MATLAB).
#
#   macOS:  dist/RflyTools-VER-macOS.pkg                 (universal arm64 + x86_64 when the Intel target is installed)
#           installs /Library/RflyTools, /Applications/RflyTools/*.app, commands in /usr/local/bin
#   Linux:  dist/rflytools_VER_ARCH.deb                  installs /opt/rflytools, app-menu entries, commands in /usr/bin
#           dist/rflytools-VER-linux-ARCH.tar.gz         unpack anywhere, ./install.sh (per user, no root)
#
# Usage: scripts/package.sh                 package for this machine's OS
#        scripts/package.sh --linux [amd64|arm64 ...]   (on a Mac) build the Linux packages in Docker
#                                           (amd64 needs colima with Rosetta: colima start --vz-rosetta)
#
# RflySim's own data (ModelData.db, the F450 FlightGear model) is NOT packaged. Users import it from
# their RflySim ISO with "Import RflySim Data" (scripts/import_rflysim_data.sh).
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
. scripts/platform.sh
[ -f "$HOME/.cargo/env" ] && . "$HOME/.cargo/env"
VER="$(grep -m1 '^version' Cargo.toml | cut -d'"' -f2)"
DIST="$ROOT/dist"; mkdir -p "$DIST"
BINS="coptersim coptersim-ui rflysim3d rflygame rflygame-server rflygame-netem rfly-update"
# Commands on PATH that run a script (symlinks would break the scripts' own ROOT lookup).
WRAPPERS="rfly-sitlrun:SITLRun.sh rfly-hitlrun:HITLRun.sh rfly-python:Python38Env.sh rfly-python-run:Python38Run.sh rfly-flightgear:FlightGearF450.sh rfly-import-data:import_rflysim_data.sh rfly-closeall:CloseALL.sh rfly-udpportfree:UdpPortFree.sh rfly-comlist:ComList.sh"

# ---------- on a Mac: build the Linux packages in containers ----------
if [ "${1:-}" = --linux ]; then
  shift; ARCHES="${*:-amd64 arm64}"
  rfly_docker_env
  for a in $ARCHES; do
    echo "== Linux $a"
    # The classic builder ignores --platform, so build FROM the base image's digest for this architecture.
    base=$(docker manifest inspect debian:bookworm-slim | python3 -c "import json,sys; print(next(m['digest'] for m in json.load(sys.stdin)['manifests'] if m['platform']['architecture']=='$a' and m['platform'].get('os')=='linux'))")
    docker build -q --build-arg "BASE=debian:bookworm-slim@$base" -t "rflysim-linux-test:$a" docker/linux-test >/dev/null
    [ "$(docker image inspect -f '{{.Architecture}}' "rflysim-linux-test:$a")" = "$a" ] || { echo "image is not $a"; exit 1; }
    docker run --rm --platform "linux/$a" -v "$ROOT:/src:ro" -v "$DIST:/dist" \
      -v "$HOME/.cargo/registry:/opt/cargo/registry" -v "rfly-linux-target$([ "$a" = arm64 ] || echo "-$a"):/work/target" \
      -v "$ROOT/build/icons:/icons:ro" "rflysim-linux-test:$a" bash -c '
        rsync -a --exclude /target --exclude /logs --exclude /dist --exclude /build --exclude /RflyTools --exclude .DS_Store \
          --exclude /assets/ModelData.db --exclude /assets/flightgear /src/ /work/ && cd /work
        mkdir -p build/icons && cp /icons/*.png build/icons/
        scripts/package.sh && cp dist/rflytools_* dist/rflytools-*linux* /dist/'
  done
  ls -la "$DIST" | grep -E 'rflytools[-_]'
  exit 0
fi

# ---------- build ----------
cargo build --release
BIN_SRC="$ROOT/target/release"
if [ "$RFLY_OS" = macos ] && rustup target list --installed 2>/dev/null | grep -q x86_64-apple-darwin; then
  cargo build --release --target x86_64-apple-darwin
  BIN_SRC="$ROOT/build/universal"; mkdir -p "$BIN_SRC"
  for b in $BINS; do lipo -create "target/release/$b" "target/x86_64-apple-darwin/release/$b" -output "$BIN_SRC/$b"; done
fi

# ---------- stage: the tree that becomes $PREFIX ----------
# stage DIR PREFIX → copies everything into DIR, launchers pointing at PREFIX
stage() {
  local d="$1" prefix="$2"
  rm -rf "$d"; mkdir -p "$d/bin" "$d/scripts" "$d/assets" "$d/docs"
  for b in $BINS; do cp "$BIN_SRC/$b" "$d/bin/"; done
  for s in platform.sh SITLRun.sh HITLRun.sh Python38Env.sh Python38Run.sh FlightGearF450.sh FlightGearDemo.sh run_sitl.sh \
           import_rflysim_data.sh CloseALL.sh UdpPortFree.sh ComList.sh; do
    cp "scripts/$s" "$d/scripts/"
  done
  rsync -a --exclude __pycache__ --exclude '*.pyc' python "$d/"
  rsync -a --exclude bin --exclude models matlab "$d/"
  cp -R assets/osm "$d/assets/"
  cp README.md "$d/"; cp docs/RFLYGAME.md docs/RFLYSIM3D_PROTOCOL.md docs/RFLYSIM_SIMULINK_WORKFLOW.md "$d/docs/"
  [ -f LICENSE ] && cp LICENSE "$d/"
  echo "$VER" >"$d/VERSION"
  scripts/build_apps.sh "$d/RflyTools" --root "$prefix" --package >/dev/null
}
# wrappers DIR PREFIX → DIR/rfly-* scripts
wrappers() {
  local w; mkdir -p "$1"
  for w in $WRAPPERS; do
    printf '#!/usr/bin/env bash\nexec "%s/scripts/%s" "$@"\n' "$2" "${w#*:}" >"$1/${w%%:*}"; chmod 755 "$1/${w%%:*}"
  done
}

# ---------- macOS .pkg ----------
if [ "$RFLY_OS" = macos ]; then
  P="$ROOT/build/pkg"; rm -rf "$P"; PAY="$P/root"
  stage "$PAY/Library/RflyTools" /Library/RflyTools
  mkdir -p "$PAY/Applications"; mv "$PAY/Library/RflyTools/RflyTools" "$PAY/Applications/RflyTools"
  wrappers "$PAY/Library/RflyTools/commands" /Library/RflyTools
  cat >"$PAY/Library/RflyTools/uninstall.sh" <<'EOF'
#!/bin/sh
# Removes RflyTools (run: sudo /Library/RflyTools/uninstall.sh). Your data in ~/Library stays.
set -e
[ "$(id -u)" = 0 ] || { echo "Run with sudo."; exit 1; }
for f in /Library/RflyTools/bin/* /Library/RflyTools/commands/*; do rm -f "/usr/local/bin/$(basename "$f")"; done
rm -rf /Applications/RflyTools /Library/RflyTools
pkgutil --forget org.rflysim-mac.rflytools >/dev/null 2>&1 || true
echo "RflyTools removed."
EOF
  chmod 755 "$PAY/Library/RflyTools/uninstall.sh"
  for b in $BINS; do codesign --force -s - "$PAY/Library/RflyTools/bin/$b" 2>/dev/null || true; done
  mkdir -p "$P/scripts"
  cat >"$P/scripts/postinstall" <<'EOF'
#!/bin/sh
# Commands on PATH; register the apps with Launch Services.
mkdir -p /usr/local/bin
for f in /Library/RflyTools/bin/* /Library/RflyTools/commands/*; do ln -sf "$f" "/usr/local/bin/$(basename "$f")"; done
for a in /Applications/RflyTools/*.app; do touch "$a"; done
exit 0
EOF
  chmod 755 "$P/scripts/postinstall"
  # No extended attributes in the payload (they become ._* AppleDouble files on the target).
  xattr -cr "$PAY"
  # Apps must land in /Applications/RflyTools. By default the Installer "relocates" a bundle to wherever an
  # app with the same bundle ID already is (e.g. a development copy), so mark every bundle non-relocatable.
  pkgbuild --analyze --root "$PAY" "$P/components.plist" >/dev/null
  /usr/bin/python3 - "$P/components.plist" <<'PY'
import plistlib, sys
path = sys.argv[1]
items = plistlib.load(open(path, "rb"))
def fix(b):
    b["BundleIsRelocatable"] = False
    for c in b.get("ChildBundles", []):
        fix(c)
for b in items:
    fix(b)
plistlib.dump(items, open(path, "wb"))
print(f"{len(items)} app bundles marked non-relocatable")
PY
  COPYFILE_DISABLE=1 pkgbuild --root "$PAY" --component-plist "$P/components.plist" --scripts "$P/scripts" \
    --identifier org.rflysim-mac.rflytools --version "$VER" --install-location / "$P/rflytools.pkg" >/dev/null
  mkdir -p "$P/res"
  cat >"$P/res/welcome.html" <<EOF
<html><body style="font-family:-apple-system,Helvetica;font-size:13px">
<h2>RflyTools $VER</h2>
<p>RflySim's tools for macOS, MATLAB-free: CopterSim, RflySim3D / 3DDisplay, RflyGame, SITLRun, HITLRun,
Python38Env (RflySimSDK), FlightGear-F450 and the RflyMac Simulink package.</p>
<p>Installs the apps into <b>/Applications/RflyTools</b>, the tools into <b>/Library/RflyTools</b> and the commands
(<code>coptersim</code>, <code>rflysim3d</code>, <code>rflygame</code>, <code>rfly-sitlrun</code>, ...) into /usr/local/bin.</p>
<p><b>After installing:</b> open <i>Import RflySim Data</i> once and give it your RflySim ISO. It adds RflySim's
vehicle database and F450 FlightGear model, which are RflySim's files and not included here.</p>
<p>SITLRun also needs PX4 SITL built in ~/tools/PX4-Autopilot (see README). Uninstall: <code>sudo /Library/RflyTools/uninstall.sh</code></p>
</body></html>
EOF
  cat >"$P/distribution.xml" <<EOF
<?xml version="1.0" encoding="utf-8"?>
<installer-gui-script minSpecVersion="2">
  <title>RflyTools $VER</title>
  <welcome file="welcome.html" mime-type="text/html"/>
  $( [ -f LICENSE ] && echo '<license file="LICENSE"/>' )
  <options customize="never" require-scripts="false" hostArchitectures="arm64,x86_64"/>
  <domains enable_localSystem="true"/>
  <choices-outline><line choice="default"><line choice="rflytools"/></line></choices-outline>
  <choice id="default"/>
  <choice id="rflytools" visible="false"><pkg-ref id="org.rflysim-mac.rflytools"/></choice>
  <pkg-ref id="org.rflysim-mac.rflytools" version="$VER" onConclusion="none">rflytools.pkg</pkg-ref>
</installer-gui-script>
EOF
  [ -f LICENSE ] && cp LICENSE "$P/res/LICENSE"
  OUTPKG="$DIST/RflyTools-$VER-macOS.pkg"
  productbuild --distribution "$P/distribution.xml" --resources "$P/res" --package-path "$P" "$OUTPKG" >/dev/null
  echo "Packaged: $OUTPKG ($(du -h "$OUTPKG" | cut -f1)), $(lipo -archs "$PAY/Library/RflyTools/bin/coptersim")"
  echo "Unsigned: users right-click > Open (or System Settings > Privacy & Security > Open Anyway)."
  exit 0
fi

# ---------- Linux .deb + .tar.gz ----------
DEB_ARCH="$(dpkg --print-architecture 2>/dev/null || { case "$(uname -m)" in x86_64) echo amd64 ;; aarch64) echo arm64 ;; *) uname -m ;; esac; })"
P="$ROOT/build/deb"; rm -rf "$P"
stage "$P/opt/rflytools" /opt/rflytools
mkdir -p "$P/usr/bin" "$P/usr/share/applications" "$P/DEBIAN"
for b in $BINS; do ln -s "/opt/rflytools/bin/$b" "$P/usr/bin/$b"; done
wrappers "$P/usr/bin" /opt/rflytools
for f in "$P/opt/rflytools/RflyTools/"*.desktop; do cp "$f" "$P/usr/share/applications/rflytools-$(basename "$f")"; done
GLIBC="$(objdump -T "$P/opt/rflytools/bin/rflygame" | grep -o 'GLIBC_[0-9.]*' | sed 's/GLIBC_//' | sort -uV | tail -1)"
cat >"$P/DEBIAN/control" <<EOF
Package: rflytools
Version: $VER
Section: science
Priority: optional
Architecture: $DEB_ARCH
Depends: libc6 (>= $GLIBC), libasound2t64 | libasound2 (>= 1.1), libudev1, libgl1, libx11-6, libxi6, libxkbcommon0, python3, python3-venv, libarchive-tools, curl
Recommends: x-terminal-emulator
Suggests: flightgear
Installed-Size: $(du -sk "$P/opt" | cut -f1)
Maintainer: Dankwa-Wadie <187449218+Dankwa-Wadie@users.noreply.github.com>
Homepage: https://github.com/Dankwa-Wadie/rflysim-mac-linux
Description: RflySim's tools for Linux and macOS (MATLAB-free)
 CopterSim-compatible simulator for PX4 SITL/HIL, RflySim3D-compatible 3D viewer,
 RflyGame (FPV drone game), SITLRun/HITLRun, Python38Env with a RflySimSDK port,
 FlightGear-F450 output and the RflyMac Simulink package. RflySim's own data files
 are imported from the user's RflySim ISO with "Import RflySim Data".
EOF
# The RflyTools APT repository (GitHub Pages), so updates come with `apt upgrade`. Its signing key ships here.
mkdir -p "$P/usr/share/keyrings"
cp packaging/rflytools-archive-keyring.gpg "$P/usr/share/keyrings/rflytools-archive-keyring.gpg"
cat >"$P/DEBIAN/postinst" <<'EOF'
#!/bin/sh
# Adds the RflyTools APT repository (unless it is already configured, or an admin set REPO=no in
# /etc/default/rflytools), so `apt upgrade` and `rfly-update` deliver new versions.
set -e
LIST=/etc/apt/sources.list.d/rflytools.list
URL="https://dankwa-wadie.github.io/rflysim-mac-linux/apt"
if [ "$1" = configure ]; then
  REPO=yes; [ -f /etc/default/rflytools ] && . /etc/default/rflytools
  if [ "$REPO" != no ] && [ ! -e "$LIST" ] && ! grep -rqs "dankwa-wadie.github.io/rflysim-mac-linux" /etc/apt/sources.list /etc/apt/sources.list.d/; then
    printf '%s\n' "# RflyTools updates (added by the rflytools package; set REPO=no in /etc/default/rflytools to stop this)" \
      "deb [signed-by=/usr/share/keyrings/rflytools-archive-keyring.gpg] $URL stable main" >"$LIST"
  fi
fi
command -v update-desktop-database >/dev/null 2>&1 && update-desktop-database -q /usr/share/applications || true
exit 0
EOF
cat >"$P/DEBIAN/postrm" <<'EOF'
#!/bin/sh
set -e
# The keyring goes away with the package, so the source must too (apt would warn about a missing key).
case "$1" in remove|purge) rm -f /etc/apt/sources.list.d/rflytools.list ;; esac
command -v update-desktop-database >/dev/null 2>&1 && update-desktop-database -q /usr/share/applications || true
exit 0
EOF
chmod 755 "$P/DEBIAN/postinst" "$P/DEBIAN/postrm"
DEB="$DIST/rflytools_${VER}_${DEB_ARCH}.deb"
dpkg-deb --root-owner-group --build "$P" "$DEB" >/dev/null
echo "Packaged: $DEB ($(du -h "$DEB" | cut -f1), glibc >= $GLIBC)"

# Portable tarball: launchers carry a placeholder root that install.sh replaces with the unpack location.
NAME="rflytools-$VER-linux-$(uname -m)"; T="$ROOT/build/tgz/$NAME"; rm -rf "$ROOT/build/tgz"
stage "$T" @RFLY_ROOT@
cat >"$T/install.sh" <<'EOF'
#!/usr/bin/env sh
# Per-user install of this RflyTools folder (no root): app-menu entries + commands in ~/.local/bin.
# Run it again after moving the folder. Remove: ./install.sh --uninstall
set -eu
DIR="$(cd "$(dirname "$0")" && pwd)"
APPS="${XDG_DATA_HOME:-$HOME/.local/share}/applications"; BIN="$HOME/.local/bin"
CMDS="coptersim coptersim-ui rflysim3d rflygame rflygame-server rflygame-netem rfly-update"
WRAP="rfly-sitlrun:SITLRun.sh rfly-hitlrun:HITLRun.sh rfly-python:Python38Env.sh rfly-python-run:Python38Run.sh rfly-flightgear:FlightGearF450.sh rfly-import-data:import_rflysim_data.sh rfly-closeall:CloseALL.sh rfly-udpportfree:UdpPortFree.sh rfly-comlist:ComList.sh"
if [ "${1:-}" = --uninstall ]; then
  rm -f "$APPS"/rflytools-*.desktop
  for c in $CMDS; do rm -f "$BIN/$c"; done; for w in $WRAP; do rm -f "$BIN/${w%%:*}"; done
  echo "RflyTools menu entries and commands removed (this folder is untouched)."; exit 0
fi
# Point the launchers at this folder (they hold the previous location, or a placeholder in a fresh unpack).
OLD="$(cat "$DIR/.installed-root" 2>/dev/null || echo @RFLY_ROOT@)"
if [ "$OLD" != "$DIR" ]; then
  for f in "$DIR"/RflyTools/bin/* "$DIR"/RflyTools/*.desktop; do sed -i "s|$OLD|$DIR|g" "$f"; done
  echo "$DIR" >"$DIR/.installed-root"
fi
mkdir -p "$APPS" "$BIN"
for f in "$DIR"/RflyTools/*.desktop; do cp "$f" "$APPS/rflytools-$(basename "$f")"; done
for c in $CMDS; do ln -sf "$DIR/bin/$c" "$BIN/$c"; done
for w in $WRAP; do printf '#!/usr/bin/env sh\nexec "%s/scripts/%s" "$@"\n' "$DIR" "${w#*:}" >"$BIN/${w%%:*}"; chmod 755 "$BIN/${w%%:*}"; done
command -v update-desktop-database >/dev/null 2>&1 && update-desktop-database -q "$APPS" || true
echo "RflyTools installed from $DIR: app menu + commands in $BIN (add it to PATH if needed)."
echo "Next: open 'Import RflySim Data' (or run rfly-import-data YOUR.iso)."
echo "Runtime packages (Debian/Ubuntu): sudo apt install libasound2 libudev1 libgl1 libx11-6 libxi6 libxkbcommon0 python3-venv libarchive-tools curl"
echo "Updates: rfly-update (or the Update RflyTools menu entry) checks for and installs new versions."
EOF
chmod 755 "$T/install.sh"
TGZ="$DIST/$NAME.tar.gz"
tar -C "$ROOT/build/tgz" -czf "$TGZ" "$NAME"
echo "Packaged: $TGZ ($(du -h "$TGZ" | cut -f1))"
