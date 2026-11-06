#!/usr/bin/env bash
# Adds .deb packages to the RflyTools APT repository and re-signs its indexes.
# Runs on Debian/Ubuntu (needs apt-utils + gnupg), e.g. inside a container from scripts/publish_release.sh.
#
# Usage: scripts/build_apt_repo.sh DEB_DIR SITE_DIR SIGNING_KEY.asc
#   SITE_DIR is the GitHub Pages tree; the repository goes to SITE_DIR/apt:
#     apt/pool/main/r/rflytools/*.deb
#     apt/dists/stable/{InRelease,Release,Release.gpg}
#     apt/dists/stable/main/binary-{amd64,arm64}/Packages{,.gz}
#     apt/rflytools-archive-keyring.gpg   (public key, also shipped in the .deb)
# Users get it automatically from the .deb, or by hand:
#   deb [signed-by=/usr/share/keyrings/rflytools-archive-keyring.gpg] https://dankwa-wadie.github.io/rflysim-mac-linux/apt stable main
set -eu
DEBS="$1"; SITE="$2"; KEY="$3"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
A="$SITE/apt"; POOL="$A/pool/main/r/rflytools"
mkdir -p "$POOL" "$A/dists/stable/main/binary-amd64" "$A/dists/stable/main/binary-arm64"
cp "$DEBS"/rflytools_*.deb "$POOL/"
# Keep the newest 5 versions per architecture (old ones stay installable for a while, the repo stays small).
for arch in amd64 arm64; do
  ls "$POOL"/rflytools_*_"$arch".deb 2>/dev/null | sort -V | head -n -5 | xargs -r rm -f
done
cd "$A"
for arch in amd64 arm64; do
  apt-ftparchive --arch "$arch" packages pool >"dists/stable/main/binary-$arch/Packages"
  gzip -9nkf "dists/stable/main/binary-$arch/Packages"
done
apt-ftparchive \
  -o APT::FTPArchive::Release::Origin="RflyTools" \
  -o APT::FTPArchive::Release::Label="RflyTools" \
  -o APT::FTPArchive::Release::Suite="stable" \
  -o APT::FTPArchive::Release::Codename="stable" \
  -o APT::FTPArchive::Release::Architectures="amd64 arm64" \
  -o APT::FTPArchive::Release::Components="main" \
  -o APT::FTPArchive::Release::Description="RflySim tools for Linux (github.com/Dankwa-Wadie/rflysim-mac-linux)" \
  release dists/stable >dists/stable/Release.new
mv dists/stable/Release.new dists/stable/Release
export GNUPGHOME="$(mktemp -d)"; trap 'rm -rf "$GNUPGHOME"' EXIT
gpg --batch --quiet --import "$KEY"
gpg --batch --yes --pinentry-mode loopback --passphrase "" --digest-algo SHA512 --clearsign -o dists/stable/InRelease dists/stable/Release
gpg --batch --yes --pinentry-mode loopback --passphrase "" --digest-algo SHA512 -abs -o dists/stable/Release.gpg dists/stable/Release
gpg --export >rflytools-archive-keyring.gpg
cmp -s rflytools-archive-keyring.gpg "$ROOT/packaging/rflytools-archive-keyring.gpg" || {
  echo "warning: the signing key does not match packaging/rflytools-archive-keyring.gpg (shipped in the .deb)" >&2; }
cat >"$SITE/index.html" <<'EOF'
<!doctype html><meta charset="utf-8"><title>RflyTools packages</title>
<style>body{font:16px/1.5 system-ui,sans-serif;max-width:46em;margin:3em auto;padding:0 1em}code,pre{background:#f3f3f3;padding:.1em .3em}</style>
<h1>RflyTools APT repository</h1>
<p>RflySim's tools for Linux: <a href="https://github.com/Dankwa-Wadie/rflysim-mac-linux">github.com/Dankwa-Wadie/rflysim-mac-linux</a>.
Installing a <code>.deb</code> from the Releases page sets this repository up automatically, so updates arrive with
<code>sudo apt upgrade</code>. To add it by hand (Debian 12+, Ubuntu 22.04+):</p>
<pre>sudo curl -fsSLo /usr/share/keyrings/rflytools-archive-keyring.gpg \
  https://dankwa-wadie.github.io/rflysim-mac-linux/apt/rflytools-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/rflytools-archive-keyring.gpg] https://dankwa-wadie.github.io/rflysim-mac-linux/apt stable main" \
  | sudo tee /etc/apt/sources.list.d/rflytools.list
sudo apt update && sudo apt install rflytools</pre>
EOF
touch "$SITE/.nojekyll"
echo "APT repository: $(ls "$POOL" | wc -l) packages, signed by $(gpg --with-colons --list-keys | awk -F: '/^fpr/{print $10; exit}')"
