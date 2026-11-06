#!/usr/bin/env bash
# Builds and tests the Linux port inside a Debian container (needs a Docker engine, e.g. colima).
# The repo is copied into the container, so Linux build output never touches this checkout.
# Stage D (PX4 SITL) copies $PX4_DIR (default ~/tools/PX4-Autopilot, without its build/) into a volume and
# builds px4_sitl_default for Linux there once (~10-20 min, ~2.5 GB).
#
# Usage: scripts/linux_docker_test.sh [stages, default ABCDEFG] [--shell]
#   Results and screenshots: logs/linux_test/
#   Caches live in Docker volumes: rfly-linux-target (cargo), rfly-linux-py (Python venvs), rfly-linux-px4 (PX4).
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STAGES="ABCDEFG"; SHELL_ONLY=0
PX4_SRC="${PX4_DIR:-$HOME/tools/PX4-Autopilot}"
for a in "$@"; do case "$a" in --shell) SHELL_ONLY=1 ;; *) STAGES="$a" ;; esac; done
IMG=rflysim-linux-test
docker build -q -t "$IMG" "$ROOT/docker/linux-test" >/dev/null
OUT="$ROOT/logs/linux_test"; mkdir -p "$OUT"
REG="${CARGO_HOME:-$HOME/.cargo}/registry"; mkdir -p "$REG"

INNER='set -e
rsync -a --delete --exclude /target --exclude /logs --exclude /dist --exclude /build --exclude /RflyTools \
  --exclude .DS_Store /src/ /work/
mkdir -p /work/RflyTools
export RFLY_DATA_DIR=/tmp/rflygame-data
case "$STAGES" in *D*)
  [ -d /px4src/src ] || { echo "PX4 source not found (set PX4_DIR)"; exit 1; }
  P=/root/tools/PX4-Autopilot; V=/root/tools/px4-venv
  rsync -a --exclude /build /px4src/ "$P/"
  if [ ! -x "$V/bin/python" ]; then
    python3 -m venv "$V"
    "$V/bin/pip" install -q empy==3.3.4 jinja2 pyyaml jsonschema kconfiglib future pyros-genmsg packaging toml numpy lark
  fi
  if [ ! -x "$P/build/px4_sitl_default/bin/px4" ]; then
    echo "Building PX4 SITL for Linux (first run only) ..."
    (cd "$P" && PATH="$V/bin:$PATH" make px4_sitl_default) >/out/px4_build.log 2>&1 || { tail -30 /out/px4_build.log; exit 1; }
  fi
  echo "PX4 SITL for Linux ready: $P/build/px4_sitl_default/bin/px4"
;; esac
if [ "$SHELL_ONLY" = 1 ]; then exec bash; fi
xvfb-run -a -s "-screen 0 1280x800x24" scripts/test_all.sh "$STAGES" || status=$?
cp -r logs/test_all/. /out/ 2>/dev/null || true
[ -d logs/sitl ] && rm -rf /out/sitl && cp -r logs/sitl /out/sitl
exit ${status:-0}'

TTY=(); [ -t 0 ] && TTY=(-it)
docker run --rm ${TTY[@]+"${TTY[@]}"} \
  -v "$ROOT:/src:ro" -v "$OUT:/out" \
  -v "$REG:/opt/cargo/registry" \
  -v rfly-linux-target:/work/target -v rfly-linux-py:/root/tools \
  -v rfly-linux-px4:/root/tools/PX4-Autopilot -v "$PX4_SRC:/px4src:ro" \
  -e STAGES="$STAGES" -e SHELL_ONLY="$SHELL_ONLY" \
  "$IMG" bash -c "$INNER"
