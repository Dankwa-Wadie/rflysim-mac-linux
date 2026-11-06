#!/usr/bin/env bash
# Install-tests the Linux packages in dist/ on clean distro containers (needs Docker; amd64 on a Mac needs
# colima --vz-rosetta). For each IMAGE:ARCH: apt-installs the .deb (dependency resolution included), checks
# commands and menu entries, imports RflySim data from ISO (if given), runs coptersim, rflysim3d and RflyGame
# as a normal user under Xvfb, removes the package; then installs the tarball per user with install.sh.
#
# Usage: scripts/test_linux_packages.sh [--iso PATH] [IMAGE:ARCH ...]
#        default targets: ubuntu:24.04:amd64 debian:12:arm64
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ISO=""; TARGETS=()
while [ $# -gt 0 ]; do
  case "$1" in --iso) ISO="$2"; shift ;; *) TARGETS+=("$1") ;; esac; shift
done
[ ${#TARGETS[@]} -eq 0 ] && TARGETS=(ubuntu:24.04:amd64 debian:12:arm64)
VER="$(grep -m1 '^version' "$ROOT/Cargo.toml" | cut -d'"' -f2)"
ISOMOUNT=(); [ -n "$ISO" ] && ISOMOUNT=(-v "$ISO:/iso/RFLY.iso:ro")
FAIL=0

for t in "${TARGETS[@]}"; do
  arch="${t##*:}"; img="${t%:*}"; uarch=$([ "$arch" = amd64 ] && echo x86_64 || echo aarch64)
  echo "===== $img ($arch)"
  docker run --rm --platform "linux/$arch" -v "$ROOT/dist:/dist:ro" ${ISOMOUNT[@]+"${ISOMOUNT[@]}"} \
    -e VER="$VER" -e ARCH="$arch" -e UARCH="$uarch" "$img" bash -c '
set -u; ok=1; chk() { if eval "$2"; then echo "PASS $1"; else echo "FAIL $1"; ok=0; fi; }
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq >/dev/null
chk "apt install .deb" "apt-get install -y -qq /dist/rflytools_${VER}_${ARCH}.deb >/tmp/apt.log 2>&1"
apt-get install -y -qq xvfb xauth libgl1-mesa-dri >/dev/null 2>&1
chk "real ALSA (not an OSS stand-in)" "dpkg -l | grep -qE \"^ii  libasound2(t64)?[: ]\""
for c in coptersim rflysim3d rflygame rflygame-server rflygame-netem rfly-sitlrun rfly-hitlrun rfly-python rfly-flightgear rfly-import-data; do
  command -v $c >/dev/null || { echo "missing command $c"; ok=0; }
done
chk "10 commands on PATH" "true"
chk "9 menu entries" "[ \$(ls /usr/share/applications | grep -c ^rflytools-) = 9 ]"
useradd -m pilot
cat >/tmp/user.sh <<"EOF"
# Runs as a normal user. Each check is a function, so no nested quoting.
set -u; ok=1
check() { local name="$1"; shift; if "$@" >/dev/null 2>&1; then echo "PASS $name"; else echo "FAIL $name"; ok=0; fi; }
x() { xvfb-run -a -s "-screen 0 1280x800x24" "$@"; }
cd ~
D=~/rflytools-$VER-linux-$UARCH
c_import() { rfly-import-data /iso/RFLY.iso; }
c_db() { coptersim --ext 20155 --no-rfly >/tmp/cs.log 2>&1 & local p=$!; sleep 1; kill $p
         grep -q "loaded .default. from $HOME/.local/share/rflytools/ModelData.db" /tmp/cs.log; }  # no single quotes: this runs inside bash -c '...'
c_view() { x rflysim3d --screenshot /tmp/v.png --after 4 && test -s /tmp/v.png; }
c_game() { x rflygame --auto mission=0 --quit-after 120 2>&1 | grep -q complete; }
c_saves() { test -f ~/.local/share/rflygame/leaderboard.txt; }
c_server() { rflygame-server --port 27199 >/tmp/srv.log 2>&1 & local p=$!; sleep 1; kill $p; grep -q 27199 /tmp/srv.log; }
c_untar() { tar -xzf /dist/rflytools-$VER-linux-$UARCH.tar.gz -C ~ && "$D/install.sh"; }
c_paths() { grep -q "ROOT=\"$D\"" "$D/RflyTools/bin/RflySim3D" && ! grep -rq @RFLY_ROOT@ "$D/RflyTools"; }
c_menu() { [ "$(ls ~/.local/share/applications | grep -c "^rflytools-")" = 9 ]; }
c_cmds() { [ -x ~/.local/bin/rfly-sitlrun ] && ~/.local/bin/coptersim --help; }
c_move() { mv "$D" ~/moved && ~/moved/install.sh && grep -q "ROOT=\"$HOME/moved\"" ~/moved/RflyTools/bin/RflySim3D; }
c_tarview() { x ~/.local/bin/rflysim3d --screenshot /tmp/v2.png --after 3 && test -s /tmp/v2.png; }
c_uninst() { ~/moved/install.sh --uninstall && [ ! -e ~/.local/bin/coptersim ] && [ "$(ls ~/.local/share/applications | grep -c "^rflytools-")" = 0 ]; }
if [ -f /iso/RFLY.iso ]; then
  check "import RflySim data from ISO" c_import
  check "coptersim loads imported ModelData.db" c_db
fi
check "rflysim3d renders (.deb)" c_view
check "RflyGame mission on autopilot (.deb)" c_game
check "RflyGame saves in ~/.local/share/rflygame" c_saves
check "rflygame-server starts" c_server
check "tarball: unpack + install.sh" c_untar
check "tarball: launchers point at the unpack folder" c_paths
check "tarball: 9 menu entries" c_menu
check "tarball: commands in ~/.local/bin" c_cmds
check "tarball: install.sh again after moving the folder" c_move
check "tarball: rflysim3d renders" c_tarview
check "tarball: uninstall" c_uninst
exit $((1 - ok))
EOF
su - pilot -c "VER=$VER UARCH=$UARCH bash /tmp/user.sh" || ok=0
chk "apt remove" "apt-get remove -y -qq rflytools >/dev/null 2>&1 && [ ! -e /opt/rflytools/bin ] && ! command -v coptersim >/dev/null"
exit $((1 - ok))' || FAIL=1
done
[ $FAIL = 0 ] && echo "ALL PACKAGE TESTS PASSED" || { echo "SOME PACKAGE TESTS FAILED"; exit 1; }
