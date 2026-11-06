# Shared OS helpers for the RflyTools scripts (macOS and Linux). Source it: . "$ROOT/scripts/platform.sh"
# shellcheck shell=bash

case "$(uname -s)" in
  Darwin) RFLY_OS=macos ;;
  Linux) RFLY_OS=linux ;;
  *) RFLY_OS=other ;;
esac

# True when something is bound to UDP port $1 on this machine.
udp_listening() {
  if command -v lsof >/dev/null 2>&1; then
    lsof -nP -iUDP:"$1" >/dev/null 2>&1
  elif command -v ss >/dev/null 2>&1; then
    ss -Hlun "sport = :$1" 2>/dev/null | grep -q .
  else
    grep -qi ":$(printf '%04X' "$1") " /proc/net/udp /proc/net/udp6 2>/dev/null
  fi
}

# Default Pixhawk / flight-controller serial ports (USB CDC-ACM).
default_serial_glob() {
  if [ "$RFLY_OS" = macos ]; then echo "/dev/cu.usbmodem*"; else echo "/dev/ttyACM*"; fi
}

# Path of the QGroundControl executable on Linux ($QGC_BIN, PATH, or a downloaded AppImage), or nothing.
find_qgc_linux() {
  if [ -n "${QGC_BIN:-}" ]; then echo "$QGC_BIN"; return; fi
  local c
  for c in QGroundControl qgroundcontrol QGroundControl.AppImage; do
    command -v "$c" 2>/dev/null && return
  done
  for c in "$HOME"/QGroundControl*.AppImage "$HOME"/Applications/QGroundControl*.AppImage \
           "$HOME"/tools/QGroundControl*.AppImage "$HOME"/Downloads/QGroundControl*.AppImage; do
    [ -x "$c" ] && { echo "$c"; return 0; }
  done
  return 0
}

# Starts QGroundControl in the background (it connects to PX4 on UDP 14550 by itself).
launch_qgc() {
  if [ "$RFLY_OS" = macos ]; then
    open -a QGroundControl 2>/dev/null || echo "QGroundControl not found (install it into /Applications)."
    return 0
  fi
  local q; q="$(find_qgc_linux)"
  if [ -z "$q" ]; then
    echo "QGroundControl not found: download QGroundControl-x86_64.AppImage (or -aarch64) to ~/Applications, chmod +x it, or set QGC_BIN."
    return 0
  fi
  nohup "$q" >/dev/null 2>&1 &
}

# ---- Paths. Scripts run from a git checkout (binaries in target/release) or from an installed package
# ---- (binaries in $ROOT/bin, read-only $ROOT). Callers set ROOT to the folder above scripts/.

# True in a source checkout.
rfly_is_repo() { [ -f "$ROOT/Cargo.toml" ]; }

# Path of a RflyTools binary (coptersim, rflysim3d, rflygame, ...).
rfly_bin() {
  if [ -x "$ROOT/bin/$1" ]; then echo "$ROOT/bin/$1"; else echo "$ROOT/target/release/$1"; fi
}

# Makes sure the given binaries exist: builds them in a checkout, explains otherwise.
rfly_need_bins() {
  local b missing=0
  for b in "$@"; do [ -x "$(rfly_bin "$b")" ] || missing=1; done
  [ $missing = 0 ] && return 0
  if rfly_is_repo; then (cd "$ROOT" && cargo build --release) && return 0; fi
  echo "RflyTools binaries missing in $ROOT/bin. Reinstall the package." >&2
  return 1
}

# Per-user folder for data imported from the RflySim ISO (ModelData.db, FlightGear F450).
rfly_data_dir() {
  if [ -n "${RFLY_TOOLS_DATA:-}" ]; then echo "$RFLY_TOOLS_DATA"
  elif [ "$RFLY_OS" = macos ]; then echo "$HOME/Library/Application Support/RflyTools"
  else echo "${XDG_DATA_HOME:-$HOME/.local/share}/rflytools"; fi
}

# Log folder: logs/ in a checkout, else a per-user folder (an installed $ROOT is read-only).
rfly_logs() {
  local d
  if rfly_is_repo; then d="$ROOT/logs"
  elif [ "$RFLY_OS" = macos ]; then d="$HOME/Library/Logs/RflyTools"
  else d="${XDG_STATE_HOME:-$HOME/.local/state}/rflytools/logs"; fi
  mkdir -p "$d${1:+/$1}" && echo "$d${1:+/$1}"
}

# RflySim's ModelData.db ($RFLY_DB, the checkout's assets/, or imported), or nothing.
rfly_db() {
  local f
  for f in "${RFLY_DB:-}" "$ROOT/assets/ModelData.db" "$(rfly_data_dir)/ModelData.db"; do
    [ -n "$f" ] && [ -f "$f" ] && { echo "$f"; return 0; }
  done
  return 0
}

# Sets the array DBARG to (--db PATH) when ModelData.db is available, else (). coptersim has F450 defaults.
rfly_set_dbarg() { local f; f="$(rfly_db)"; DBARG=(); if [ -n "$f" ]; then DBARG=(--db "$f"); fi; }

# FlightGear aircraft folder that holds RflySim's F450, or nothing.
rfly_fg_aircraft() {
  local d
  for d in "$ROOT/assets/flightgear/Aircraft" "$(rfly_data_dir)/flightgear/Aircraft"; do
    [ -f "$d/F450/F450-set.xml" ] && { echo "$d"; return 0; }
  done
  return 0
}

# Prints "RflyTools X is available" when the daily update check found a newer release (instant: cached;
# the check itself runs in the background). Silent in a source checkout and when checks are disabled.
rfly_update_notice() {
  rfly_is_repo && return 0
  local u; u="$(rfly_bin rfly-update)"
  [ -x "$u" ] && "$u" --notice
  return 0
}

# Makes `docker` usable for the container-based scripts without changing the user's Docker config:
# starts colima if it is installed and stopped, talks to its socket, and works around a leftover
# Docker Desktop credential helper ("credsStore": "desktop") by using an empty, private client config.
rfly_docker_env() {
  if ! docker info >/dev/null 2>&1 && command -v colima >/dev/null 2>&1; then
    colima status >/dev/null 2>&1 || colima start --vm-type vz --vz-rosetta >/dev/null 2>&1 || true
    [ -S "$HOME/.colima/default/docker.sock" ] && export DOCKER_HOST="unix://$HOME/.colima/default/docker.sock"
  fi
  if grep -qs '"credsStore": *"desktop"' "${DOCKER_CONFIG:-$HOME/.docker}/config.json" && ! command -v docker-credential-desktop >/dev/null 2>&1; then
    # Keep the endpoint of the current context (it lives in the config we are about to bypass).
    if [ -z "${DOCKER_HOST:-}" ]; then
      DOCKER_HOST="$(docker context inspect --format '{{.Endpoints.docker.Host}}' 2>/dev/null)"; export DOCKER_HOST
    fi
    export DOCKER_CONFIG="${TMPDIR:-/tmp}/rflytools-docker-config"; mkdir -p "$DOCKER_CONFIG"
  fi
  docker info >/dev/null 2>&1 || { echo "Docker is not available (install colima: brew install colima)"; return 1; }
}
