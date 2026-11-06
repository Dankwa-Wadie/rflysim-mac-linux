//! Updates for installed RflyTools: a GitHub Release is the update channel.
//!
//! * **Check**: at most once a day (and only if enabled), ask GitHub for the latest published release and
//!   remember the answer in the per-user data folder. Apps show a notice; nothing installs on its own.
//! * **Trust**: every release carries `SHA256SUMS.txt` and `SHA256SUMS.txt.sig`, an Ed25519 signature made with
//!   the maintainer's offline key. The public half is built into the tools (`release-keys.pub`). A download is
//!   used only if the signature verifies, its SHA-256 matches, and the signed file names a newer version.
//! * **Install**, by how RflyTools was installed: macOS `.pkg` (Installer with the admin prompt), Debian package
//!   (`apt`, through the RflyTools APT repository when configured), or the per-user tarball (swapped in place).
//!
//! Downloads use `curl`, which macOS and Linux ship. `RFLY_UPDATE_URL` points the client at another API base
//! (tests use a local mock server); the signature check makes that safe.

use base64::Engine as _;
use ed25519_dalek::{Signature, VerifyingKey};
use sha2::{Digest, Sha256};
use std::fmt;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{SystemTime, UNIX_EPOCH};

pub const REPO: &str = "Dankwa-Wadie/rflysim-mac-linux";
/// Version of the running tools (all RflyTools binaries share the workspace version).
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
const DAY: u64 = 24 * 3600;
pub const SUMS: &str = "SHA256SUMS.txt";
pub const SUMS_SIG: &str = "SHA256SUMS.txt.sig";

pub type Result<T> = std::result::Result<T, String>;

// ---------------------------------------------------------------------------------------------------------
// Versions

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Version(pub u64, pub u64, pub u64);

impl Version {
    /// Parses `1.2.3`, `v1.2.3`, `1.2.3-rc1` (suffix ignored). Missing parts are 0.
    pub fn parse(s: &str) -> Option<Version> {
        let s = s.trim().trim_start_matches(['v', 'V']);
        let core = s.split(['-', '+']).next()?;
        let mut it = core.split('.').map(|p| p.parse::<u64>());
        let a = it.next()?.ok()?;
        let b = it.next().unwrap_or(Ok(0)).ok()?;
        let c = it.next().unwrap_or(Ok(0)).ok()?;
        if it.next().is_some() {
            return None;
        }
        Some(Version(a, b, c))
    }
    pub fn current() -> Version {
        Version::parse(VERSION).expect("crate version")
    }
}

impl fmt::Display for Version {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}.{}.{}", self.0, self.1, self.2)
    }
}

// ---------------------------------------------------------------------------------------------------------
// Signatures

/// Release-signing public keys built into the tools (base64 Ed25519, one per line, `#` comments).
pub fn release_keys() -> Vec<VerifyingKey> {
    include_str!("../release-keys.pub")
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .filter_map(|l| base64::engine::general_purpose::STANDARD.decode(l).ok())
        .filter_map(|b| <[u8; 32]>::try_from(b.as_slice()).ok())
        .filter_map(|b| VerifyingKey::from_bytes(&b).ok())
        .collect()
}

/// Checks `sig_text` (base64 Ed25519 signature) over `data` against the built-in release keys.
pub fn verify_signature(data: &[u8], sig_text: &str) -> Result<()> {
    verify_with(&release_keys(), data, sig_text)
}

pub fn verify_with(keys: &[VerifyingKey], data: &[u8], sig_text: &str) -> Result<()> {
    let line = sig_text.lines().map(str::trim).find(|l| !l.is_empty() && !l.starts_with('#')).unwrap_or("");
    let raw = base64::engine::general_purpose::STANDARD.decode(line).map_err(|_| "signature is not valid base64".to_string())?;
    let sig = Signature::from_slice(&raw).map_err(|_| "signature has the wrong length".to_string())?;
    if keys.iter().any(|k| k.verify_strict(data, &sig).is_ok()) {
        Ok(())
    } else {
        Err("signature does not match any RflyTools release key".into())
    }
}

pub fn sha256_hex(data: &[u8]) -> String {
    Sha256::digest(data).iter().map(|b| format!("{b:02x}")).collect()
}

pub fn sha256_file(path: &Path) -> Result<String> {
    let mut f = std::fs::File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut h = Sha256::new();
    let mut buf = vec![0u8; 1 << 16];
    loop {
        let n = f.read(&mut buf).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        h.update(&buf[..n]);
    }
    Ok(h.finalize().iter().map(|b| format!("{b:02x}")).collect())
}

/// `shasum -a 256` / `sha256sum` output: `<hex>  <name>` per line.
pub fn parse_sums(text: &str) -> Vec<(String, String)> {
    text.lines()
        .filter_map(|l| {
            let (h, n) = l.trim().split_once(char::is_whitespace)?;
            let n = n.trim().trim_start_matches('*');
            (h.len() == 64 && h.chars().all(|c| c.is_ascii_hexdigit()) && !n.is_empty()).then(|| (h.to_lowercase(), n.to_string()))
        })
        .collect()
}

// ---------------------------------------------------------------------------------------------------------
// Network (curl)

pub fn api_base() -> String {
    std::env::var("RFLY_UPDATE_URL").ok().filter(|s| !s.is_empty()).unwrap_or_else(|| format!("https://api.github.com/repos/{REPO}"))
}

fn curl(url: &str, out: Option<&Path>, timeout_s: u32) -> Result<Vec<u8>> {
    let mut c = Command::new("curl");
    c.args(["-fsSL", "--proto", "=https,http", "--retry", "2", "--connect-timeout", "10", "--max-time"])
        .arg(timeout_s.to_string())
        .args(["-H", &format!("User-Agent: rflytools-updater/{VERSION}"), "-H", "Accept: application/vnd.github+json"]);
    if let Some(o) = out {
        c.arg("-o").arg(o);
    }
    let r = c.arg(url).stdin(Stdio::null()).output().map_err(|e| format!("cannot run curl: {e}"))?;
    if !r.status.success() {
        let msg = String::from_utf8_lossy(&r.stderr).trim().to_string();
        return Err(format!("download failed ({url}): {}", if msg.is_empty() { r.status.to_string() } else { msg }));
    }
    Ok(r.stdout)
}

#[derive(Clone, Debug)]
pub struct Asset {
    pub name: String,
    pub url: String,
}

#[derive(Clone, Debug)]
pub struct Release {
    pub version: Version,
    pub tag: String,
    pub page: String,
    pub notes: String,
    pub assets: Vec<Asset>,
}

impl Release {
    pub fn asset(&self, name: &str) -> Option<&Asset> {
        self.assets.iter().find(|a| a.name == name)
    }
}

pub fn parse_release(json: &[u8]) -> Result<Release> {
    let v: serde_json::Value = serde_json::from_slice(json).map_err(|e| format!("bad release JSON: {e}"))?;
    let tag = v["tag_name"].as_str().ok_or("release has no tag_name")?.to_string();
    let version = Version::parse(&tag).ok_or_else(|| format!("release tag {tag} is not a version"))?;
    if v["draft"].as_bool() == Some(true) || v["prerelease"].as_bool() == Some(true) {
        return Err("latest release is a draft or pre-release".into());
    }
    let assets = v["assets"]
        .as_array()
        .map(|a| {
            a.iter()
                .filter_map(|x| Some(Asset { name: x["name"].as_str()?.to_string(), url: x["browser_download_url"].as_str()?.to_string() }))
                .collect()
        })
        .unwrap_or_default();
    Ok(Release {
        version,
        tag,
        page: v["html_url"].as_str().unwrap_or_default().to_string(),
        notes: v["body"].as_str().unwrap_or_default().to_string(),
        assets,
    })
}

/// The latest published release (GitHub excludes drafts and pre-releases from `releases/latest`).
pub fn latest() -> Result<Release> {
    parse_release(&curl(&format!("{}/releases/latest", api_base()), None, 30)?)
}

// ---------------------------------------------------------------------------------------------------------
// Per-user state: last check, latest version seen, opt-out

/// Per-user RflyTools data folder (same as `rfly_db::tools_data_dir` and the scripts' `rfly_data_dir`).
pub fn data_dir() -> PathBuf {
    let var = |k: &str| std::env::var_os(k).filter(|v| !v.is_empty()).map(PathBuf::from);
    if let Some(d) = var("RFLY_TOOLS_DATA") {
        return d;
    }
    let home = var("HOME").unwrap_or_else(|| ".".into());
    if cfg!(target_os = "macos") {
        home.join("Library/Application Support/RflyTools")
    } else if cfg!(windows) {
        var("APPDATA").unwrap_or(home).join("RflyTools")
    } else {
        var("XDG_DATA_HOME").filter(|p| p.is_absolute()).unwrap_or_else(|| home.join(".local/share")).join("rflytools")
    }
}

fn kv_read(path: &Path) -> Vec<(String, String)> {
    std::fs::read_to_string(path)
        .unwrap_or_default()
        .lines()
        .filter_map(|l| l.split_once('=').map(|(k, v)| (k.trim().to_string(), v.trim().to_string())))
        .collect()
}
fn kv_get(kv: &[(String, String)], k: &str) -> Option<String> {
    kv.iter().rev().find(|(a, _)| a == k).map(|(_, v)| v.clone())
}

fn state_path() -> PathBuf {
    data_dir().join("update-state.txt")
}
fn config_path() -> PathBuf {
    data_dir().join("update.conf")
}

/// Update checks are on unless `RFLY_NO_UPDATE_CHECK` is set or `rfly-update --disable` was run.
pub fn enabled() -> bool {
    if std::env::var_os("RFLY_NO_UPDATE_CHECK").is_some_and(|v| !v.is_empty() && v != "0") {
        return false;
    }
    kv_get(&kv_read(&config_path()), "check").as_deref() != Some("off")
}

pub fn set_enabled(on: bool) -> Result<()> {
    std::fs::create_dir_all(data_dir()).map_err(|e| e.to_string())?;
    std::fs::write(config_path(), format!("# RflyTools update settings (rfly-update --enable / --disable)\ncheck={}\n", if on { "on" } else { "off" }))
        .map_err(|e| e.to_string())
}

fn now() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
}

fn interval() -> u64 {
    std::env::var("RFLY_UPDATE_INTERVAL").ok().and_then(|v| v.parse().ok()).unwrap_or(DAY)
}

/// True when the last check is older than the interval (a day).
pub fn due() -> bool {
    let last = kv_get(&kv_read(&state_path()), "last_check").and_then(|v| v.parse::<u64>().ok()).unwrap_or(0);
    now().saturating_sub(last) >= interval()
}

fn save_state(latest: Option<&Release>) {
    let mut s = format!("# RflyTools update check (written by rfly-update)\nlast_check={}\n", now());
    if let Some(r) = latest {
        s += &format!("latest={}\npage={}\n", r.version, r.page);
    }
    let _ = std::fs::create_dir_all(data_dir());
    let _ = std::fs::write(state_path(), s);
}

/// Asks GitHub now and records the result. Returns the release if it is newer than the running tools.
pub fn refresh() -> Result<Option<Release>> {
    match latest() {
        Ok(r) => {
            save_state(Some(&r));
            Ok((r.version > Version::current()).then_some(r))
        }
        Err(e) => {
            // Remember the attempt so an offline machine does not retry on every start.
            save_state(None);
            Err(e)
        }
    }
}

/// Newer version recorded by the last check, if any: (version, release page).
pub fn cached_update() -> Option<(Version, String)> {
    let kv = kv_read(&state_path());
    let v = Version::parse(&kv_get(&kv, "latest")?)?;
    (v > Version::current()).then(|| (v, kv_get(&kv, "page").unwrap_or_default()))
}

/// One-line notice for programs and scripts.
pub fn notice() -> Option<String> {
    if !enabled() {
        return None;
    }
    cached_update().map(|(v, _)| format!("RflyTools {v} is available (you have {VERSION}). Update with: rfly-update"))
}

/// For GUI apps: runs the daily check on a background thread (if enabled and due) and delivers the notice
/// version, if an update is available. Never blocks the caller.
pub fn spawn_check() -> std::sync::mpsc::Receiver<Version> {
    let (tx, rx) = std::sync::mpsc::channel();
    if enabled() {
        std::thread::spawn(move || {
            if due() {
                let _ = refresh();
            }
            if let Some((v, _)) = cached_update() {
                let _ = tx.send(v);
            }
        });
    }
    rx
}

// ---------------------------------------------------------------------------------------------------------
// How RflyTools was installed

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Install {
    /// macOS `.pkg` in /Library/RflyTools
    MacPkg(PathBuf),
    /// Debian package in /opt/rflytools
    Deb(PathBuf),
    /// Per-user tarball (folder with install.sh)
    Tarball(PathBuf),
    /// Source checkout (update with git)
    Source(PathBuf),
    Unknown(PathBuf),
}

impl Install {
    pub fn root(&self) -> &Path {
        match self {
            Install::MacPkg(p) | Install::Deb(p) | Install::Tarball(p) | Install::Source(p) | Install::Unknown(p) => p,
        }
    }
}

/// Classifies an install from the folder above `bin/` (or a checkout's `target/<profile>/`).
pub fn classify(exe: &Path) -> Install {
    let bin = exe.parent().unwrap_or(Path::new("/"));
    let root = bin.parent().unwrap_or(Path::new("/")).to_path_buf();
    // target/release/x or target/<triple>/release/x in a checkout
    if let Some(repo) = bin.ancestors().find(|a| a.join("Cargo.toml").is_file() && a.join("crates").is_dir()) {
        return Install::Source(repo.to_path_buf());
    }
    if root == Path::new("/Library/RflyTools") {
        Install::MacPkg(root)
    } else if root == Path::new("/opt/rflytools") {
        Install::Deb(root)
    } else if root.join("install.sh").is_file() && root.join("bin").is_dir() {
        Install::Tarball(root)
    } else {
        Install::Unknown(root)
    }
}

pub fn detect() -> Install {
    let exe = std::env::current_exe().ok().and_then(|p| p.canonicalize().ok()).unwrap_or_default();
    classify(&exe)
}

/// Release asset that updates this install.
pub fn asset_name(install: &Install, v: Version) -> Option<String> {
    let deb_arch = match std::env::consts::ARCH {
        "x86_64" => "amd64",
        "aarch64" => "arm64",
        a => a,
    };
    match install {
        Install::MacPkg(_) => Some(format!("RflyTools-{v}-macOS.pkg")),
        Install::Deb(_) => Some(format!("rflytools_{v}_{deb_arch}.deb")),
        Install::Tarball(_) => Some(format!("rflytools-{v}-linux-{}.tar.gz", std::env::consts::ARCH)),
        _ => None,
    }
}

// ---------------------------------------------------------------------------------------------------------
// Verified download

/// Downloads `name` from the release into `dir`, after checking the signed checksum list. Fails closed.
pub fn download_verified(r: &Release, name: &str, dir: &Path) -> Result<PathBuf> {
    let sums_a = r.asset(SUMS).ok_or("release has no SHA256SUMS.txt")?;
    let sig_a = r.asset(SUMS_SIG).ok_or("release is not signed (no SHA256SUMS.txt.sig); not installing it")?;
    let sums = curl(&sums_a.url, None, 60)?;
    let sig = curl(&sig_a.url, None, 60)?;
    verify_signature(&sums, &String::from_utf8_lossy(&sig))?;
    let list = parse_sums(&String::from_utf8_lossy(&sums));
    let (want, _) = list.iter().find(|(_, n)| n == name).ok_or_else(|| format!("{name} is not in the signed checksum list"))?;
    // The version comes from the signed file name, not from the (unsigned) release metadata.
    let signed_version = list.iter().filter_map(|(_, n)| version_in_name(n)).max().ok_or("signed checksum list names no version")?;
    if signed_version != r.version || signed_version <= Version::current() {
        return Err(format!("signed files are for {signed_version}, not a newer release than {VERSION}"));
    }
    let a = r.asset(name).ok_or_else(|| format!("release has no {name}"))?;
    std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    let out = dir.join(name);
    curl(&a.url, Some(&out), 1800)?;
    let got = sha256_file(&out)?;
    if &got != want {
        let _ = std::fs::remove_file(&out);
        return Err(format!("{name}: checksum mismatch (download corrupted or tampered with)"));
    }
    Ok(out)
}

/// Version embedded in a release file name (`RflyTools-1.2.3-macOS.pkg`, `rflytools_1.2.3_amd64.deb`, ...).
pub fn version_in_name(name: &str) -> Option<Version> {
    name.split(['-', '_']).find_map(|p| {
        let p = p.trim_end_matches(".pkg").trim_end_matches(".deb");
        (p.matches('.').count() == 2).then(|| Version::parse(p)).flatten()
    })
}

// ---------------------------------------------------------------------------------------------------------
// Installing

/// RflyTools programs that should not be running while files are replaced.
pub fn running_tools() -> Vec<String> {
    ["coptersim", "rflysim3d", "rflygame", "rflygame-server", "rflygame-netem"]
        .iter()
        .filter(|n| Command::new("pgrep").arg("-x").arg(n).stdout(Stdio::null()).status().is_ok_and(|s| s.success()))
        .map(|n| n.to_string())
        .collect()
}

fn run(cmd: &mut Command) -> Result<()> {
    let s = cmd.status().map_err(|e| format!("cannot run {cmd:?}: {e}"))?;
    if s.success() {
        Ok(())
    } else {
        Err(format!("{cmd:?} failed ({s})"))
    }
}

/// macOS: the Installer, with the standard administrator prompt (or sudo over SSH).
pub fn install_pkg(pkg: &Path) -> Result<()> {
    let gui = std::env::var_os("SSH_TTY").is_none();
    if gui {
        let p = pkg.display().to_string().replace('\\', "\\\\").replace('"', "\\\"");
        let script = format!("do shell script \"/usr/sbin/installer -pkg \" & quoted form of \"{p}\" & \" -target /\" with administrator privileges");
        run(Command::new("osascript").arg("-e").arg(script))
    } else {
        run(Command::new("sudo").arg("/usr/sbin/installer").arg("-pkg").arg(pkg).arg("-target").arg("/"))
    }
}

/// True when the RflyTools APT repository is configured (the .deb adds it).
pub fn apt_repo_configured() -> bool {
    Path::new("/etc/apt/sources.list.d/rflytools.list").is_file()
}

/// Debian/Ubuntu: apt upgrade from the RflyTools repository, or the verified .deb when there is none.
pub fn install_deb(deb: Option<&Path>) -> Result<()> {
    match deb {
        None => {
            run(Command::new("sudo").args(["apt-get", "update"]))?;
            run(Command::new("sudo").args(["apt-get", "install", "--only-upgrade", "-y", "rflytools"]))
        }
        Some(d) => run(Command::new("sudo").args(["apt-get", "install", "-y"]).arg(d)),
    }
}

/// Per-user tarball: unpack next to the install, swap folders, re-run install.sh. Rolls back on failure.
pub fn install_tarball(root: &Path, tgz: &Path) -> Result<()> {
    let parent = root.parent().ok_or("install folder has no parent")?;
    let pid = std::process::id();
    let stage = parent.join(format!(".rflytools-update-{pid}"));
    let old = parent.join(format!(".rflytools-old-{pid}"));
    let _ = std::fs::remove_dir_all(&stage);
    std::fs::create_dir_all(&stage).map_err(|e| e.to_string())?;
    let result = (|| {
        run(Command::new("tar").arg("-xzf").arg(tgz).arg("-C").arg(&stage))?;
        let new = std::fs::read_dir(&stage)
            .map_err(|e| e.to_string())?
            .filter_map(|e| e.ok().map(|e| e.path()))
            .find(|p| p.join("install.sh").is_file() && p.join("bin").is_dir())
            .ok_or("the update archive has no RflyTools folder")?;
        std::fs::rename(root, &old).map_err(|e| format!("cannot move the old version aside: {e}"))?;
        if let Err(e) = std::fs::rename(&new, root) {
            let _ = std::fs::rename(&old, root);
            return Err(format!("cannot move the new version into place: {e}"));
        }
        if let Err(e) = run(Command::new(root.join("install.sh")).stdout(Stdio::null())) {
            let _ = std::fs::rename(root, &stage.join("failed"));
            let _ = std::fs::rename(&old, root);
            let _ = run(Command::new(root.join("install.sh")).stdout(Stdio::null()));
            return Err(format!("install.sh failed, previous version restored: {e}"));
        }
        Ok(())
    })();
    let _ = std::fs::remove_dir_all(&stage);
    let _ = std::fs::remove_dir_all(&old);
    result
}

/// Opens a terminal window running `rfly-update` (for "Update" buttons in the GUI apps).
pub fn open_updater_terminal() -> Result<()> {
    let exe = std::env::current_exe().map_err(|e| e.to_string())?;
    let updater = exe.with_file_name("rfly-update");
    let updater = if updater.is_file() { updater } else { PathBuf::from("rfly-update") };
    if cfg!(target_os = "macos") {
        let script = std::env::temp_dir().join(format!("rflytools-update-{}.command", std::process::id()));
        std::fs::write(&script, format!("#!/bin/sh\n\"{}\"\necho\nread -p 'Press Enter to close this window ' _\n", updater.display()))
            .map_err(|e| e.to_string())?;
        run(Command::new("chmod").arg("755").arg(&script))?;
        run(Command::new("open").arg("-a").arg("Terminal").arg(&script))
    } else {
        let cmd = format!("\"{}\"; echo; read -p 'Press Enter to close this window ' _", updater.display());
        for (t, args) in [("x-terminal-emulator", vec!["-e"]), ("gnome-terminal", vec!["--"]), ("konsole", vec!["-e"]), ("xterm", vec!["-e"])] {
            if Command::new(t).args(&args).arg("sh").arg("-c").arg(&cmd).spawn().is_ok() {
                return Ok(());
            }
        }
        Err("no terminal emulator found; run rfly-update in a terminal".into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};

    #[test]
    fn versions_compare() {
        assert_eq!(Version::parse("v0.1.1"), Some(Version(0, 1, 1)));
        assert_eq!(Version::parse("1.2"), Some(Version(1, 2, 0)));
        assert_eq!(Version::parse("1.2.3-rc1"), Some(Version(1, 2, 3)));
        assert!(Version::parse("x").is_none() && Version::parse("1.2.3.4").is_none());
        assert!(Version(0, 2, 0) > Version(0, 1, 9) && Version(1, 0, 0) > Version(0, 99, 99));
    }

    #[test]
    fn file_names_carry_versions() {
        assert_eq!(version_in_name("RflyTools-0.2.0-macOS.pkg"), Some(Version(0, 2, 0)));
        assert_eq!(version_in_name("rflytools_0.2.0_amd64.deb"), Some(Version(0, 2, 0)));
        assert_eq!(version_in_name("rflytools-0.2.0-linux-x86_64.tar.gz"), Some(Version(0, 2, 0)));
        assert_eq!(version_in_name("SHA256SUMS.txt"), None);
    }

    #[test]
    fn signatures_and_sums() {
        let sk = SigningKey::from_bytes(&[7u8; 32]);
        let data = b"abc  file.txt\n";
        let sig = base64::engine::general_purpose::STANDARD.encode(sk.sign(data).to_bytes());
        assert!(verify_with(&[sk.verifying_key()], data, &sig).is_ok());
        assert!(verify_with(&[sk.verifying_key()], b"abd  file.txt\n", &sig).is_err(), "tampered data must fail");
        let other = SigningKey::from_bytes(&[8u8; 32]).verifying_key();
        assert!(verify_with(&[other], data, &sig).is_err(), "wrong key must fail");
        assert!(verify_with(&[sk.verifying_key()], data, "not base64!").is_err());
        let h = sha256_hex(b"hello");
        let sums = format!("{h}  hello.txt\n{h} *star.txt\ngarbage\n");
        assert_eq!(parse_sums(&sums), vec![(h.clone(), "hello.txt".into()), (h, "star.txt".into())]);
        assert!(!release_keys().is_empty(), "a release key must be built in");
    }

    #[test]
    fn install_kinds() {
        assert_eq!(classify(Path::new("/Library/RflyTools/bin/rfly-update")), Install::MacPkg("/Library/RflyTools".into()));
        assert_eq!(classify(Path::new("/opt/rflytools/bin/rfly-update")), Install::Deb("/opt/rflytools".into()));
        let repo = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().unwrap();
        assert!(matches!(classify(&repo.join("target/release/rfly-update")), Install::Source(_)));
        assert_eq!(asset_name(&Install::MacPkg("/".into()), Version(1, 2, 3)).unwrap(), "RflyTools-1.2.3-macOS.pkg");
    }

    #[test]
    fn release_json() {
        let j = br#"{"tag_name":"v0.2.0","html_url":"https://x/r","body":"notes","draft":false,"prerelease":false,
            "assets":[{"name":"SHA256SUMS.txt","browser_download_url":"https://x/s"}]}"#;
        let r = parse_release(j).unwrap();
        assert_eq!((r.version, r.asset("SHA256SUMS.txt").unwrap().url.as_str()), (Version(0, 2, 0), "https://x/s"));
        assert!(parse_release(br#"{"tag_name":"v0.2.0","prerelease":true}"#).is_err());
    }
}
