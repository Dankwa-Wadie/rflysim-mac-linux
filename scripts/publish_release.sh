#!/usr/bin/env bash
# Maintainer: publishes a RflyTools release so installed tools pick it up.
#
#   1. Push a tag (git tag v0.2.0 && git push origin v0.2.0). GitHub Actions builds and tests all packages and
#      creates a DRAFT release with them and SHA256SUMS.txt (.github/workflows/release.yml).
#   2. Run: scripts/publish_release.sh v0.2.0
#        - downloads the draft's files and checks them against SHA256SUMS.txt
#        - signs SHA256SUMS.txt with your release key (~/.config/rflytools-release/release.key) and uploads the .sig
#        - adds the .deb files to the APT repository on the gh-pages branch (signed with apt-signing.asc) and pushes
#        - publishes the draft as the latest release
#      Installed tools see the update within a day (or at once with `rfly-update`); apt users with `apt upgrade`.
#
# Options: --from-dist   create the draft from dist/ (packages built locally with scripts/package.sh)
#          --yes         do not ask before publishing
#          --no-apt      skip the APT repository step
# The keys never leave this machine. Back up ~/.config/rflytools-release: without release.key no update can be
# published to existing installs (a new key needs a release that adds it to crates/rfly-update/release-keys.pub).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
REPO="Dankwa-Wadie/rflysim-mac-linux"
KEYS="${RFLY_RELEASE_KEYS:-$HOME/.config/rflytools-release}"
TAG=""; FROM_DIST=0; YES=0; APT=1
for a in "$@"; do
  case "$a" in
    --from-dist) FROM_DIST=1 ;; --yes) YES=1 ;; --no-apt) APT=0 ;;
    -h|--help) sed -n 2,20p "$0"; exit 0 ;;
    v*) TAG="$a" ;;
    *) echo "unknown option $a"; exit 1 ;;
  esac
done
[ -n "$TAG" ] || { echo "usage: scripts/publish_release.sh vX.Y.Z [--from-dist] [--yes] [--no-apt]"; exit 1; }
VER="${TAG#v}"
[ -f "$KEYS/release.key" ] || { echo "release key not found: $KEYS/release.key"; exit 1; }
[ "$APT" = 0 ] || [ -f "$KEYS/apt-signing.asc" ] || { echo "APT signing key not found: $KEYS/apt-signing.asc"; exit 1; }
CARGO_VER="$(grep -m1 '^version' Cargo.toml | cut -d'"' -f2)"
[ "$CARGO_VER" = "$VER" ] || echo "note: Cargo.toml says $CARGO_VER, publishing $VER"
[ -f "$HOME/.cargo/env" ] && . "$HOME/.cargo/env"
SIGNER="$ROOT/target/release/rfly-sign"
cargo build --release -q -p rfly-update
FILES="RflyTools-$VER-macOS.pkg rflytools_${VER}_amd64.deb rflytools_${VER}_arm64.deb rflytools-$VER-linux-x86_64.tar.gz rflytools-$VER-linux-aarch64.tar.gz"
W="$(mktemp -d)"; trap 'git worktree remove --force "$W/site" >/dev/null 2>&1 || true; rm -rf "$W"' EXIT
mkdir -p "$W/files"

# 1. The draft and its files
if [ "$FROM_DIST" = 1 ]; then
  for f in $FILES; do [ -f "dist/$f" ] || { echo "missing dist/$f (scripts/package.sh and package.sh --linux)"; exit 1; }; cp "dist/$f" "$W/files/"; done
  (cd "$W/files" && shasum -a 256 $FILES >SHA256SUMS.txt)
  NOTES="dist/RELEASE_NOTES_$VER.md"; [ -f "$NOTES" ] || { NOTES="$W/notes.md"; sed "s/VERSION/$VER/g" packaging/release-notes.md >"$NOTES"; }
  gh release view "$TAG" --repo "$REPO" >/dev/null 2>&1 && { echo "$TAG already exists on GitHub"; exit 1; }
  gh release create "$TAG" --repo "$REPO" --draft --title "RflyTools $VER" --notes-file "$NOTES" "$W/files"/*
else
  [ "$(gh release view "$TAG" --repo "$REPO" --json isDraft --jq .isDraft)" = true ] || { echo "$TAG is not a draft release (did the release workflow finish?)"; exit 1; }
  gh release download "$TAG" --repo "$REPO" -D "$W/files" --clobber
fi
for f in $FILES SHA256SUMS.txt; do [ -f "$W/files/$f" ] || { echo "draft is missing $f"; exit 1; }; done
(cd "$W/files" && shasum -a 256 -c SHA256SUMS.txt) || { echo "checksums do not match; not publishing"; exit 1; }

# 2. Sign the checksum list (rfly-sign also checks it against the key built into the tools)
"$SIGNER" sign "$KEYS/release.key" "$W/files/SHA256SUMS.txt"
gh release upload "$TAG" --repo "$REPO" --clobber "$W/files/SHA256SUMS.txt.sig"

# 3. APT repository on gh-pages
if [ "$APT" = 1 ]; then
  git fetch -q origin gh-pages 2>/dev/null || true
  if git rev-parse -q --verify origin/gh-pages >/dev/null; then
    git worktree add -q -B gh-pages "$W/site" origin/gh-pages
  else
    git worktree add -q --detach "$W/site" && (cd "$W/site" && git checkout -q --orphan gh-pages && git rm -rqf . 2>/dev/null || true)
  fi
  mkdir -p "$W/debs" && cp "$W/files"/rflytools_*.deb "$W/debs/"
  if command -v apt-ftparchive >/dev/null && command -v gpg >/dev/null; then
    scripts/build_apt_repo.sh "$W/debs" "$W/site" "$KEYS/apt-signing.asc"
  else
    # macOS: build it in a Debian container (colima/Docker). Paths must be under $HOME for colima's file sharing.
    . "$ROOT/scripts/platform.sh"; rfly_docker_env
    D="$HOME/.cache/rflytools-publish"; rm -rf "$D"; mkdir -p "$D"; cp -R "$W/debs" "$W/site" "$D/"; cp "$KEYS/apt-signing.asc" "$D/"
    docker run --rm -v "$ROOT:/src:ro" -v "$D:/w" debian:bookworm-slim bash -c '
      apt-get update -qq >/dev/null && apt-get install -y -qq apt-utils gnupg >/dev/null 2>&1
      /src/scripts/build_apt_repo.sh /w/debs /w/site /w/apt-signing.asc'
    rsync -a --delete --exclude .git "$D/site/" "$W/site/"; rm -rf "$D"
  fi
  (cd "$W/site" && git add -A && git -c user.name="$(git -C "$ROOT" config user.name)" -c user.email="$(git -C "$ROOT" config user.email)" \
     commit -qm "APT repository: rflytools $VER" && git push -q origin HEAD:gh-pages)
  if ! gh api "repos/$REPO/pages" >/dev/null 2>&1; then
    echo "Turning on GitHub Pages for the gh-pages branch (serves the APT repository)..."
    gh api -X POST "repos/$REPO/pages" -f "source[branch]=gh-pages" -f "source[path]=/" >/dev/null
  fi
fi

# 4. Publish
if [ "$YES" = 0 ]; then
  read -rp "Publish $TAG as the latest release now? Installed RflyTools will offer it as an update. [y/N] " ok
  [ "$ok" = y ] || [ "$ok" = Y ] || { echo "Not published. The draft stays as it is (signature uploaded)."; exit 0; }
fi
gh release edit "$TAG" --repo "$REPO" --draft=false --latest
echo "Published: https://github.com/$REPO/releases/tag/$TAG"
