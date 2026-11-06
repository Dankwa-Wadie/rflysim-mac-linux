#!/usr/bin/env bash
# Real Linux update tests in a container (needs Docker, e.g. colima): builds RflyTools at the current version
# and again as 9.9.9, then checks
#   - rfly-update against a mock release server (signatures, checksums, opt-out, offline: scripts/test_update.sh)
#   - a per-user tarball install upgrading itself in place to 9.9.9
#   - the .deb: installs the RflyTools APT source, `apt update` verifies the repository signature, rfly-update
#     upgrades through apt to 9.9.9; a repository signed with another key is rejected; removal drops the source
# The release and APT keys come from ~/.config/rflytools-release (the real ones; nothing is published).
# Usage: scripts/test_update_linux.sh
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
KEYS="${RFLY_RELEASE_KEYS:-$HOME/.config/rflytools-release}"
[ -f "$KEYS/release.key" ] && [ -f "$KEYS/apt-signing.asc" ] || { echo "release keys not found in $KEYS"; exit 1; }
. "$ROOT/scripts/platform.sh"; rfly_docker_env
docker build -q -t rflysim-linux-test "$ROOT/docker/linux-test" >/dev/null
OUT="$ROOT/logs/update_test"; mkdir -p "$OUT"
docker run --rm -v "$ROOT:/src:ro" -v "$KEYS:/keys:ro" -v "$OUT:/out" \
  -v "${CARGO_HOME:-$HOME/.cargo}/registry:/opt/cargo/registry" -v rfly-linux-target:/work/target \
  rflysim-linux-test bash -c '
set -u; FAIL=0
pass() { echo "PASS $1"; }; bad() { echo "FAIL $1"; FAIL=1; }
rsync -a --exclude /target --exclude /logs --exclude /dist --exclude /build --exclude /RflyTools --exclude .DS_Store \
  --exclude /assets/ModelData.db --exclude /assets/flightgear /src/ /work/ && cd /work
CUR=$(grep -m1 "^version" Cargo.toml | cut -d\" -f2); ARCH=$(uname -m); DA=$(dpkg --print-architecture)
python3 -m venv /tmp/venv >/dev/null && /tmp/venv/bin/pip -q install numpy opencv-python-headless >/dev/null 2>&1
export RFLY_VENV=/tmp/venv RFLY_RELEASE_KEY=/keys/release.key
mkdir -p /pk/old /pk/new
echo "== building $CUR"; scripts/package.sh >/out/build_old.log 2>&1 || { tail -20 /out/build_old.log; exit 1; }
cp dist/rflytools_${CUR}_$DA.deb dist/rflytools-$CUR-linux-$ARCH.tar.gz /pk/old/
cp target/release/rfly-update /pk/old/rfly-update; cp target/release/rfly-sign /pk/rfly-sign
sed -i "s/^version = \"$CUR\"/version = \"9.9.9\"/" Cargo.toml
echo "== building 9.9.9"; scripts/package.sh >/out/build_new.log 2>&1 || { tail -20 /out/build_new.log; exit 1; }
cp dist/rflytools_9.9.9_$DA.deb dist/rflytools-9.9.9-linux-$ARCH.tar.gz /pk/new/
sed -i "s/^version = \"9.9.9\"/version = \"$CUR\"/" Cargo.toml

echo "== rfly-update against the mock server, and a real tarball upgrade"
useradd -m pilot; chmod -R a+rX /pk /work/scripts; cp /keys/release.key /tmp/release.key; chown pilot /tmp/release.key
su pilot -c "/work/scripts/test_update.sh --updater /pk/old/rfly-update --signer /pk/rfly-sign --key /tmp/release.key \
  --real-tarball /pk/old/rflytools-$CUR-linux-$ARCH.tar.gz /pk/new/rflytools-9.9.9-linux-$ARCH.tar.gz" | grep -E "PASS|FAIL|ALL|SOME" || FAIL=1

echo "== APT repository"
mkdir -p /site && scripts/build_apt_repo.sh /pk/new /site /keys/apt-signing.asc
(cd /site && python3 -m http.server 8088 >/dev/null 2>&1 &) ; sleep 1
apt-get update -qq >/dev/null 2>&1; apt-get install -y -qq /pk/old/rflytools_${CUR}_$DA.deb >/tmp/apt1.log 2>&1 && pass "install $CUR .deb" || { bad "install .deb"; tail -5 /tmp/apt1.log; }
grep -q "^deb \[signed-by=/usr/share/keyrings/rflytools-archive-keyring.gpg\] https://dankwa-wadie.github.io/rflysim-mac-linux/apt stable main" /etc/apt/sources.list.d/rflytools.list \
  && pass "postinst added the RflyTools APT source" || bad "APT source: $(cat /etc/apt/sources.list.d/rflytools.list 2>&1)"
sed -i "s#https://dankwa-wadie.github.io/rflysim-mac-linux/apt#http://127.0.0.1:8088/apt#" /etc/apt/sources.list.d/rflytools.list
apt-get update -o Dir::Etc::sourcelist=sources.list.d/rflytools.list -o Dir::Etc::sourceparts=- -o APT::Get::List-Cleanup=0 >/tmp/upd.log 2>&1 \
  && ! grep -qiE "NO_PUBKEY|not signed|BADSIG|Err:" /tmp/upd.log && pass "apt update verifies the repository signature" || { bad "apt update"; cat /tmp/upd.log; }
apt list --upgradable 2>/dev/null | grep -q "rflytools/stable .*9.9.9" && pass "apt sees rflytools 9.9.9" || bad "upgradable: $(apt list --upgradable 2>/dev/null)"
# rfly-update (Debian install + repository configured) upgrades through apt. The mock server announces 9.9.9.
mkdir -p /srv/files; cp /pk/new/rflytools_9.9.9_$DA.deb /srv/files/; (cd /srv/files && sha256sum ./* | sed "s#\./##" >SHA256SUMS.txt) && /pk/rfly-sign sign /keys/release.key /srv/files/SHA256SUMS.txt >/dev/null
python3 -c "import json,os; json.dump({\"tag_name\":\"v9.9.9\",\"html_url\":\"https://example.invalid\",\"body\":\"\",\"draft\":False,\"prerelease\":False,\"assets\":[{\"name\":n,\"browser_download_url\":n} for n in os.listdir(\"/srv/files\")]},open(\"/srv/release.json\",\"w\"))"
python3 scripts/mock_release_server.py /srv 28999 & sleep 1
RFLY_UPDATE_URL=http://127.0.0.1:28999 RFLY_TOOLS_DATA=/tmp/upd rfly-update --install --yes >/tmp/ru.log 2>&1 \
  && [ "$(dpkg-query -W -f="\${Version}" rflytools)" = 9.9.9 ] && [ "$(rfly-update --version)" = "rfly-update 9.9.9" ] \
  && pass "rfly-update upgraded the .deb to 9.9.9 through apt" || { bad "rfly-update via apt"; tail -8 /tmp/ru.log; }
# A repository signed with another key must be rejected.
mkdir -p /evil && export GNUPGHOME=$(mktemp -d) && gpg --batch --pinentry-mode loopback --passphrase "" --quick-gen-key "Evil <e@x>" rsa2048 sign never 2>/dev/null \
  && gpg --armor --export-secret-keys >/tmp/evil.asc && unset GNUPGHOME
scripts/build_apt_repo.sh /pk/new /evil /tmp/evil.asc >/dev/null 2>&1; (cd /evil && python3 -m http.server 8089 >/dev/null 2>&1 &); sleep 1
sed -i "s#http://127.0.0.1:8088/apt#http://127.0.0.1:8089/apt#" /etc/apt/sources.list.d/rflytools.list
apt-get update -o Dir::Etc::sourcelist=sources.list.d/rflytools.list -o Dir::Etc::sourceparts=- -o APT::Get::List-Cleanup=0 >/tmp/evil.log 2>&1
grep -qiE "NO_PUBKEY|not signed|signatures couldn|is not signed|BADSIG" /tmp/evil.log && pass "repository signed with another key is rejected" || { bad "evil repo accepted"; cat /tmp/evil.log; }
apt-get remove -y -qq rflytools >/dev/null 2>&1; [ ! -e /etc/apt/sources.list.d/rflytools.list ] && pass "removal drops the APT source" || bad "source left after removal"
cp -r /site /out/site-preview 2>/dev/null || true
[ $FAIL = 0 ] && echo "ALL LINUX UPDATE TESTS PASSED" || { echo "SOME LINUX UPDATE TESTS FAILED"; exit 1; }'
