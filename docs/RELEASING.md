# Releasing RflyTools (maintainer)

A published GitHub Release is the update channel. Installed tools check for the latest release once a day,
show a notice, and install it when the user agrees (`rfly-update`, *Update RflyTools*, RflyGame's menu, or
`sudo apt upgrade` on Debian/Ubuntu).

## Steps

1. Bump `version` in `Cargo.toml`, commit, and push to `main` (CI runs the unit tests on Linux and macOS).
2. Tag and push the tag:
   ```sh
   git tag v0.2.0 && git push origin v0.2.0
   ```
   The **Release** workflow builds the macOS `.pkg` (universal) and the Linux `.deb` and tarballs (amd64, arm64),
   installs each one on clean systems to test it, and creates a **draft** release with the files and `SHA256SUMS.txt`.
3. When the draft is there, sign and publish it from your Mac:
   ```sh
   scripts/publish_release.sh v0.2.0
   ```
   It checks the files against `SHA256SUMS.txt`, signs that list with your release key, uploads
   `SHA256SUMS.txt.sig`, adds the `.deb` files to the APT repository (`gh-pages` branch, served by GitHub Pages
   at https://dankwa-wadie.github.io/rflysim-mac-linux/apt) and publishes the release as *latest*.
   It needs Docker (colima) for the APT step on macOS.

Without CI (packages built locally with `scripts/package.sh` and `scripts/package.sh --linux`):
`scripts/publish_release.sh v0.2.0 --from-dist`.

## Keys

Both live only in `~/.config/rflytools-release/` on the maintainer's machine. **Back this folder up** (an
encrypted backup or a password manager), and never commit or upload it.

| File | Purpose | Public half |
|---|---|---|
| `release.key` | Ed25519 key that signs `SHA256SUMS.txt`. `rfly-update` installs nothing without a valid signature. | Built into the tools: `crates/rfly-update/release-keys.pub` |
| `apt-signing.asc` | GPG key (RSA 4096) that signs the APT repository indexes. | Shipped in the `.deb`: `packaging/rflytools-archive-keyring.gpg` |

Rotating the release key: generate a new one (`target/release/rfly-sign keygen DIR`), add its public line to
`release-keys.pub`, and publish one release signed with the **old** key. After users have that version, releases can
be signed with the new key and the old line removed. If `release.key` is lost, existing installs cannot receive
signed updates any more; users would have to install a new package by hand.

## What the tools trust

- `rfly-update` downloads `SHA256SUMS.txt` and `SHA256SUMS.txt.sig`, verifies the signature against the built-in
  keys, checks that the signed file names carry a version newer than the installed one, downloads the package
  and checks its SHA-256. Any mismatch stops the update and nothing is installed.
- apt verifies the repository's `InRelease` signature against the shipped keyring.
- The update check sends one HTTPS request a day to `api.github.com` (no data about the user or machine).
  Users can turn it off: `rfly-update --disable`, RflyGame Settings, or `RFLY_NO_UPDATE_CHECK=1`.

## Tests

```sh
scripts/test_update.sh            # rfly-update against a mock release server (signatures, tampering, opt-out)
scripts/test_update_linux.sh      # in Docker: real tarball upgrade and APT repository upgrade to a 9.9.9 build
scripts/test_linux_packages.sh    # install tests of dist/*.deb and tarballs on clean Ubuntu/Debian
```
