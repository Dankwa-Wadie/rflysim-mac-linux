#!/usr/bin/env bash
# Tests rfly-update against a mock release server (scripts/mock_release_server.py): update check, notices,
# opt-out, signature and checksum enforcement, and (with --real-tarball) a real in-place tarball upgrade.
#
# Usage: scripts/test_update.sh [--updater PATH] [--key KEY] [--real-tarball OLD.tar.gz NEW.tar.gz]
#   --updater  rfly-update binary to test (default: target/release/rfly-update)
#   --key      release signing key (default: ~/.config/rflytools-release/release.key; must match the built-in key)
#   --real-tarball  install OLD per user, publish NEW on the mock server, run rfly-update --install --yes in OLD,
#                   and check the folder now runs NEW (Linux).
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
UPD="$ROOT/target/release/rfly-update"; SIGNER="$ROOT/target/release/rfly-sign"
KEY="${RFLY_RELEASE_KEY:-$HOME/.config/rflytools-release/release.key}"; OLD_TGZ=""; NEW_TGZ=""
while [ $# -gt 0 ]; do
  case "$1" in
    --updater) UPD="$2"; shift ;;
    --signer) SIGNER="$2"; shift ;;
    --key) KEY="$2"; shift ;;
    --real-tarball) OLD_TGZ="$2"; NEW_TGZ="$3"; shift 2 ;;
  esac
  shift
done
T="$(mktemp -d)"; PORT=$((28000 + RANDOM % 1000)); FAIL=0
trap 'kill $SRV 2>/dev/null; rm -rf "$T"' EXIT
pass() { echo "PASS $1"; }
bad() { echo "FAIL $1" | head -1; echo "   last: $(tail -2 "$T/o" | tr "\n" " ")"; FAIL=1; }
CUR="$("$UPD" --version | awk '{print $2}')"
ARCH="$(uname -m)"; [ "$ARCH" = arm64 ] && ARCH=aarch64

mkdir -p "$T/srv/files"
python3 "$ROOT/scripts/mock_release_server.py" "$T/srv" "$PORT" & SRV=$!
export RFLY_UPDATE_URL="http://127.0.0.1:$PORT" RFLY_TOOLS_DATA="$T/data" RFLY_UPDATE_INTERVAL=0
for _ in $(seq 1 50); do curl -s "$RFLY_UPDATE_URL/files/x" >/dev/null 2>&1 && break; sleep 0.1; done

# release VER [tag] → files for VER, SHA256SUMS.txt signed with $SIGN_KEY, release.json with tag
release() {
  local v="$1" tag="${2:-v$1}" d="$T/srv/files"
  rm -rf "$d"; mkdir -p "$d" "$T/build/rflytools-$v-linux-$ARCH/bin"
  printf '#!/bin/sh\necho "rfly-update %s"\n' "$v" >"$T/build/rflytools-$v-linux-$ARCH/bin/rfly-update"
  chmod +x "$T/build/rflytools-$v-linux-$ARCH/bin/rfly-update"
  printf '#!/bin/sh\nexit 0\n' >"$T/build/rflytools-$v-linux-$ARCH/install.sh"; chmod +x "$T/build/rflytools-$v-linux-$ARCH/install.sh"
  tar -C "$T/build" -czf "$d/rflytools-$v-linux-$ARCH.tar.gz" "rflytools-$v-linux-$ARCH"
  echo "pkg $v" >"$d/RflyTools-$v-macOS.pkg"
  (cd "$d" && shasum -a 256 ./* 2>/dev/null | sed 's#\./##' >SHA256SUMS.txt || sha256sum ./* | sed 's#\./##' >SHA256SUMS.txt)
  "$SIGNER" sign "${SIGN_KEY:-$KEY}" "$d/SHA256SUMS.txt" >/dev/null 2>&1
  write_json "$tag"
}
write_json() {
  python3 - "$T/srv" "$1" <<'PY'
import json, os, sys
d, tag = sys.argv[1], sys.argv[2]
names = sorted(os.listdir(os.path.join(d, "files")))
json.dump({"tag_name": tag, "html_url": "https://example.invalid/" + tag, "body": "Test release " + tag,
           "draft": False, "prerelease": False, "assets": [{"name": n, "browser_download_url": n} for n in names]},
          open(os.path.join(d, "release.json"), "w"))
PY
}

# A fake per-user install (tarball layout) that runs the updater under test.
INST="$T/inst/rflytools"; mkdir -p "$INST/bin"; cp "$UPD" "$INST/bin/rfly-update"
printf '#!/bin/sh\nexit 0\n' >"$INST/install.sh"; chmod +x "$INST/install.sh"
U="$INST/bin/rfly-update"

# 1. Same version published → up to date
release "$CUR"; "$U" --check >"$T/o" 2>&1; rc=$?
[ $rc = 0 ] && grep -q "up to date" "$T/o" && pass "current release: up to date" || bad "current release ($rc): $(cat "$T/o")"

# 2. Newer signed release → --check reports it (exit 10), notice and status show it
release 9.9.9; "$U" --check >"$T/o" 2>&1; rc=$?
[ $rc = 10 ] && grep -q "9.9.9 is available" "$T/o" && pass "newer release detected (exit 10)" || bad "newer release ($rc): $(cat "$T/o")"
"$U" --notice 2>"$T/o"; grep -q "RflyTools 9.9.9 is available" "$T/o" && pass "notice line" || bad "notice: $(cat "$T/o")"

# 3. Verified download (dry run)
"$U" --install --yes --dry-run >"$T/o" 2>&1; rc=$?
[ $rc = 0 ] && grep -q "Verified: rflytools-9.9.9-linux-$ARCH.tar.gz" "$T/o" && pass "signed download verified (dry run)" || bad "dry run ($rc): $(cat "$T/o")"

# 4. Tampered file after signing → refused
release 9.9.9; echo tampered >>"$T/srv/files/rflytools-9.9.9-linux-$ARCH.tar.gz"
"$U" --install --yes --dry-run >"$T/o" 2>&1; rc=$?
[ $rc != 0 ] && grep -q "checksum mismatch" "$T/o" && pass "tampered download refused" || bad "tampered ($rc): $(cat "$T/o")"

# 5. Signed with a key that is not built in → refused
"$SIGNER" keygen "$T/evil" >/dev/null 2>&1
SIGN_KEY="$T/evil/release.key"; release 9.9.9; unset SIGN_KEY
"$U" --install --yes --dry-run >"$T/o" 2>&1; rc=$?
[ $rc != 0 ] && grep -q "does not match any RflyTools release key" "$T/o" && pass "foreign signature refused" || bad "foreign key ($rc): $(cat "$T/o")"

# 6. Unsigned release → refused
release 9.9.9; rm "$T/srv/files/SHA256SUMS.txt.sig"; write_json v9.9.9
"$U" --install --yes --dry-run >"$T/o" 2>&1; rc=$?
[ $rc != 0 ] && grep -q "not signed" "$T/o" && pass "unsigned release refused" || bad "unsigned ($rc): $(cat "$T/o")"

# 7. Release metadata claims 10.0.0 but the signed files are 9.9.9 → refused
release 9.9.9 v10.0.0
"$U" --install --yes --dry-run >"$T/o" 2>&1; rc=$?
[ $rc != 0 ] && grep -qE "not in the signed checksum list|signed files are for 9.9.9" "$T/o" && pass "version mismatch refused" || bad "mismatch ($rc): $(cat "$T/o")"

# 8. Old signed release republished as latest (downgrade) → not offered
release 0.0.1; "$U" --check >"$T/o" 2>&1; rc=$?
[ $rc = 0 ] && pass "older release not offered" || bad "downgrade ($rc): $(cat "$T/o")"

# 9. Opt-out
release 9.9.9; "$U" --check >/dev/null 2>&1; "$U" --disable >/dev/null
"$U" --notice 2>"$T/o"; [ ! -s "$T/o" ] && pass "--disable silences notices" || bad "disable: $(cat "$T/o")"
"$U" --enable >/dev/null; RFLY_NO_UPDATE_CHECK=1 "$U" --notice 2>"$T/o"; [ ! -s "$T/o" ] && pass "RFLY_NO_UPDATE_CHECK" || bad "env opt-out"

# 10. Offline → clear error, no crash
kill $SRV 2>/dev/null; wait $SRV 2>/dev/null
"$U" --check >"$T/o" 2>&1; rc=$?
[ $rc = 1 ] && grep -q "could not check" "$T/o" && pass "offline handled" || bad "offline ($rc): $(cat "$T/o")"

# 11. Real tarball upgrade (Linux): OLD installed per user, NEW published, rfly-update swaps it in place.
if [ -n "$OLD_TGZ" ]; then
  python3 "$ROOT/scripts/mock_release_server.py" "$T/srv" "$PORT" & SRV=$!; sleep 0.5
  OLDV=$(basename "$OLD_TGZ" | cut -d- -f2); NEWV=$(basename "$NEW_TGZ" | cut -d- -f2)
  mkdir -p "$T/home"; tar -xzf "$OLD_TGZ" -C "$T/home"; D="$T/home/$(basename "$OLD_TGZ" .tar.gz)"
  HOME="$T/home" "$D/install.sh" >/dev/null
  rm -rf "$T/srv/files"; mkdir -p "$T/srv/files"; cp "$NEW_TGZ" "$T/srv/files/"
  (cd "$T/srv/files" && sha256sum ./* | sed 's#\./##' >SHA256SUMS.txt); "$SIGNER" sign "$KEY" "$T/srv/files/SHA256SUMS.txt" >/dev/null
  write_json "v$NEWV"
  echo "y" | HOME="$T/home" "$D/bin/rfly-update" --install --yes >"$T/o" 2>&1; rc=$?
  got=$("$D/bin/rfly-update" --version | awk '{print $2}')
  [ $rc = 0 ] && [ "$got" = "$NEWV" ] && pass "tarball upgrade $OLDV → $NEWV in place" || bad "tarball upgrade ($rc, now $got): $(tail -5 "$T/o")"
  grep -q "ROOT=\"$D\"" "$D/RflyTools/bin/RflySim3D" && [ -x "$T/home/.local/bin/rfly-update" ] && \
    [ "$(ls "$T/home/.local/share/applications" | grep -c '^rflytools-')" -ge 9 ] && pass "launchers and menu entries refreshed" || bad "launchers after upgrade"
  ls -a "$T/home" | grep -q '\.rflytools-' && bad "leftover temp folders" || pass "no leftover temp folders"
fi

[ $FAIL = 0 ] && echo "ALL UPDATE TESTS PASSED" || { echo "SOME UPDATE TESTS FAILED"; exit 1; }
