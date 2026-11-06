#!/usr/bin/env bash
# Imports the RflySim data files RflyTools can use but does not ship (they belong to RflySim):
#   ModelData.db   vehicle presets + parts catalog (coptersim --db, RflyGame Garage)
#   F450 model     FlightGear aircraft for FlightGear-F450
# from your own copy of the RflySim installer ISO (RFLYExpPspSoftwarePack.iso), its
# 4.HILApps/CopterSim.zip and "FlightGear 2016.1.2.zip", or an unpacked RflySim folder.
#
# Usage: scripts/import_rflysim_data.sh SOURCE [SOURCE ...]
#        scripts/import_rflysim_data.sh --status
# Data goes to ~/Library/Application Support/RflyTools (macOS) or ~/.local/share/rflytools (Linux).
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/scripts/platform.sh"
DEST="$(rfly_data_dir)"
DB_IN_ZIP="CopterSim/ModelData.db"
FG_ZIP="4.HILApps/FlightGear 2016.1.2.zip"
CS_ZIP="4.HILApps/CopterSim.zip"
F450_IN_ZIP="FlightGear 2016.1.2/data/Aircraft/F450"

status() {
  echo "RflySim data folder: $DEST"
  if [ -f "$DEST/ModelData.db" ]; then echo "  ModelData.db  ✓"; else echo "  ModelData.db  missing"; fi
  if [ -f "$DEST/flightgear/Aircraft/F450/F450-set.xml" ]; then echo "  F450 model    ✓"; else echo "  F450 model    missing"; fi
}
if [ "${1:-}" = --status ]; then status; exit 0; fi
if [ $# -eq 0 ] || [ "${1:-}" = -h ] || [ "${1:-}" = --help ]; then
  sed -n 2,11p "$0" | sed 's/^# \{0,1\}//'
  if [ -t 0 ]; then
    echo; read -rp "Path to your RflySim ISO, zip or folder (drag it here): " src
    # Terminals paste dragged files as 'path' or with backslash escapes (macOS: My\ File.iso).
    src="${src% }"; src="${src%\'}"; src="${src#\'}"; src="$(printf '%s' "$src" | sed 's/\\\(.\)/\1/g')"
    [ -n "$src" ] && exec "$0" "$src"
  fi
  exit 0
fi

# bsdtar reads ISO 9660/UDF and zip archives, and streams members out without unpacking everything.
BSDTAR=""
for t in bsdtar tar; do
  if command -v $t >/dev/null 2>&1 && $t --version 2>/dev/null | grep -q libarchive; then BSDTAR=$t; break; fi
done
if [ -z "$BSDTAR" ] && ! command -v 7z >/dev/null 2>&1; then
  echo "Need bsdtar or 7z to read the ISO. Debian/Ubuntu: sudo apt install libarchive-tools"; exit 1
fi
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT

# member ARCHIVE PATH → stdout (one archive member, by exact path)
member() {
  if [ -n "$BSDTAR" ]; then $BSDTAR -xOf "$1" "$2"
  else 7z x -so "$1" "$2" 2>/dev/null; fi
}
# extract_tree ARCHIVE_OR_STDIN PREFIX OUTDIR   (- = stdin)
extract_tree() {
  local strip; strip=$(( $(printf '%s' "$2" | tr -cd / | wc -c) + 1 ))  # PREFIX's own components
  mkdir -p "$3"
  if [ -n "$BSDTAR" ]; then
    $BSDTAR -xf "$1" -C "$3" --strip-components "$strip" --exclude '*.bak' "$2"
  else
    local f="$1"; [ "$f" = - ] && { cat >"$TMP/in.zip"; f="$TMP/in.zip"; }
    7z x -y -o"$TMP/7z" "$f" "$2/*" >/dev/null && cp -R "$TMP/7z/$2/." "$3/" && find "$3" -name '*.bak' -delete
  fi
}

got_db=0; got_fg=0
install_db() { # file
  if ! head -c 16 "$1" | grep -q "SQLite format 3"; then echo "  not an SQLite file: $1"; return 1; fi
  mkdir -p "$DEST"; cp "$1" "$DEST/ModelData.db"; got_db=1; echo "  ModelData.db imported"
}
install_f450() { # dir containing F450-set.xml
  [ -f "$1/F450-set.xml" ] || return 1
  rm -rf "$DEST/flightgear/Aircraft/F450"; mkdir -p "$DEST/flightgear/Aircraft"
  cp -R "$1" "$DEST/flightgear/Aircraft/F450"; find "$DEST/flightgear/Aircraft/F450" -name '*.bak' -delete
  got_fg=1; echo "  F450 model imported"
}

for src in "$@"; do
  echo "Reading $src ..."
  if [ -d "$src" ]; then
    f=$(find "$src" -name ModelData.db -type f 2>/dev/null | head -1)
    [ -n "$f" ] && install_db "$f"
    d=$(find "$src" -path '*Aircraft/F450/F450-set.xml' -type f 2>/dev/null | head -1)
    [ -n "$d" ] && install_f450 "$(dirname "$d")"
    for z in "$src/$CS_ZIP" "$src/$FG_ZIP"; do [ -f "$z" ] && set -- "$@" "$z"; done
    continue
  fi
  [ -f "$src" ] || { echo "  not found"; continue; }
  case "$(basename "$src")" in
    CopterSim.zip)
      member "$src" "$DB_IN_ZIP" >"$TMP/db" 2>/dev/null && install_db "$TMP/db" ;;
    FlightGear*.zip)
      extract_tree "$src" "$F450_IN_ZIP" "$TMP/F450" 2>/dev/null && install_f450 "$TMP/F450" ;;
    *) # the ISO
      if member "$src" "$CS_ZIP" >"$TMP/cs.zip" 2>/dev/null && [ -s "$TMP/cs.zip" ]; then
        member "$TMP/cs.zip" "$DB_IN_ZIP" >"$TMP/db" 2>/dev/null && install_db "$TMP/db"
      fi
      if [ -n "$BSDTAR" ]; then
        member "$src" "$FG_ZIP" 2>/dev/null | extract_tree - "$F450_IN_ZIP" "$TMP/F450" 2>/dev/null || true
      else
        member "$src" "$FG_ZIP" >"$TMP/fg.zip" 2>/dev/null && extract_tree "$TMP/fg.zip" "$F450_IN_ZIP" "$TMP/F450" || true
      fi
      install_f450 "$TMP/F450" || true ;;
  esac
done

echo; status
[ $got_db = 1 ] || [ $got_fg = 1 ] || { echo "Nothing imported. Pass the RflySim ISO, CopterSim.zip, the FlightGear zip or an unpacked RflySim folder."; exit 1; }
