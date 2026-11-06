//! rfly-update: checks for and installs RflyTools updates (a GitHub Release is the update channel).
//!
//!   rfly-update               check now; if a newer release exists, show it and offer to install
//!   rfly-update --check       check now and print the result (exit 0 = up to date, 10 = update available)
//!   rfly-update --install [--yes] [--dry-run]
//!   rfly-update --notice      print the cached notice (scripts use this; refreshes in the background once a day)
//!   rfly-update --refresh     check quietly and record the result
//!   rfly-update --enable | --disable | --status | --version
use rfly_update::*;
use std::io::{BufRead, IsTerminal, Write};
use std::process::{Command, Stdio};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let has = |f: &str| args.iter().any(|a| a == f);
    let code = if has("--help") || has("-h") {
        println!("{}", include_str!("rfly-update.rs").lines().skip(2).take(8).map(|l| l.trim_start_matches("//!")).collect::<Vec<_>>().join("\n"));
        0
    } else if has("--version") {
        println!("rfly-update {VERSION}");
        0
    } else if has("--notice") {
        notice_mode()
    } else if has("--refresh") {
        let _ = refresh();
        0
    } else if has("--enable") || has("--disable") {
        let on = has("--enable");
        match set_enabled(on) {
            Ok(()) => {
                println!("RflyTools update checks {}.", if on { "enabled" } else { "disabled" });
                0
            }
            Err(e) => fail(&e),
        }
    } else if has("--status") {
        status();
        0
    } else if has("--check") {
        check_mode()
    } else {
        update_mode(has("--install"), has("--yes") || has("-y"), has("--dry-run"))
    };
    std::process::exit(code);
}

fn fail(m: &str) -> i32 {
    eprintln!("rfly-update: {m}");
    1
}

fn status() {
    println!("RflyTools {VERSION}, installed as: {}", describe(&detect()));
    println!("Update checks: {}", if enabled() { "on (daily)" } else { "off" });
    match cached_update() {
        Some((v, page)) => println!("Last check found {v}: {page}"),
        None => println!("Last check found no newer release."),
    }
    println!("Release signing keys built in: {}", release_keys().len());
}

fn describe(i: &Install) -> String {
    match i {
        Install::MacPkg(p) => format!("macOS package ({})", p.display()),
        Install::Deb(p) => format!("Debian package ({})", p.display()),
        Install::Tarball(p) => format!("per-user folder ({})", p.display()),
        Install::Source(p) => format!("source checkout ({})", p.display()),
        Install::Unknown(p) => format!("unknown layout ({})", p.display()),
    }
}

/// Cached, instant. If the cache is stale, starts a detached `--refresh` so the next run knows.
fn notice_mode() -> i32 {
    if !enabled() {
        return 0;
    }
    if due() {
        if let Ok(exe) = std::env::current_exe() {
            let _ = Command::new(exe).arg("--refresh").stdin(Stdio::null()).stdout(Stdio::null()).stderr(Stdio::null()).spawn();
        }
    }
    if let Some(n) = notice() {
        eprintln!("{n}");
    }
    0
}

fn check_mode() -> i32 {
    match refresh() {
        Ok(Some(r)) => {
            println!("RflyTools {} is available (you have {VERSION}): {}", r.version, r.page);
            10
        }
        Ok(None) => {
            println!("RflyTools {VERSION} is up to date.");
            0
        }
        Err(e) => fail(&format!("could not check for updates: {e}")),
    }
}

fn ask(q: &str) -> bool {
    print!("{q} [Y/n] ");
    let _ = std::io::stdout().flush();
    let mut s = String::new();
    let _ = std::io::stdin().lock().read_line(&mut s);
    !s.trim().to_lowercase().starts_with('n')
}

fn update_mode(install_flag: bool, yes: bool, dry: bool) -> i32 {
    let install = detect();
    println!("RflyTools {VERSION} ({}). Checking for updates...", describe(&install));
    let r = match refresh() {
        Ok(Some(r)) => r,
        Ok(None) => {
            println!("You have the latest version.");
            return 0;
        }
        Err(e) => return fail(&format!("could not check for updates: {e}")),
    };
    println!("\nRflyTools {} is available: {}\n", r.version, r.page);
    let notes: Vec<&str> = r.notes.lines().filter(|l| !l.trim().is_empty()).take(12).collect();
    if !notes.is_empty() {
        println!("{}\n", notes.join("\n"));
    }
    match &install {
        Install::Source(p) => {
            println!("This is a source checkout ({}). Update it with: git pull && cargo build --release", p.display());
            return 0;
        }
        Install::Unknown(p) => {
            println!("RflyTools at {} was not installed from a package; download the new one from the page above.", p.display());
            return 0;
        }
        _ => {}
    }
    let interactive = std::io::stdin().is_terminal();
    if !install_flag && !(interactive && ask(&format!("Install RflyTools {} now?", r.version))) {
        if !interactive {
            println!("Run `rfly-update --install` to install it.");
        }
        return 0;
    }
    if install_flag && !yes && interactive && !ask(&format!("Install RflyTools {} now?", r.version)) {
        return 0;
    }
    let running = running_tools();
    if !running.is_empty() && !dry {
        return fail(&format!("close these RflyTools programs first, then run rfly-update again: {}", running.join(", ")));
    }

    // Debian package with the RflyTools APT repository: apt does the download and signature checks.
    if matches!(install, Install::Deb(_)) && apt_repo_configured() {
        println!("Updating through the RflyTools APT repository (sudo may ask for your password)...");
        if dry {
            println!("[dry run] sudo apt-get update && sudo apt-get install --only-upgrade rflytools");
            return 0;
        }
        return match install_deb(None) {
            Ok(()) => done(&r),
            Err(e) => fail(&e),
        };
    }

    let name = match asset_name(&install, r.version) {
        Some(n) => n,
        None => return fail("no package for this kind of install"),
    };
    let dir = std::env::temp_dir().join(format!("rflytools-update-{}", r.version));
    println!("Downloading {name} and checking its signature...");
    let file = match download_verified(&r, &name, &dir) {
        Ok(f) => f,
        Err(e) => return fail(&format!("{e}. Nothing was installed.")),
    };
    println!("Verified: {name} is signed by the RflyTools release key.");
    if dry {
        println!("[dry run] would install {} into {}", file.display(), install.root().display());
        return 0;
    }
    let res = match &install {
        Install::MacPkg(_) => {
            println!("Installing (macOS asks for an administrator password)...");
            install_pkg(&file)
        }
        Install::Deb(_) => {
            println!("Installing the package (sudo may ask for your password)...");
            install_deb(Some(&file))
        }
        Install::Tarball(root) => {
            println!("Replacing {} ...", root.display());
            install_tarball(root, &file)
        }
        _ => unreachable!(),
    };
    let _ = std::fs::remove_dir_all(&dir);
    match res {
        Ok(()) => done(&r),
        Err(e) => fail(&e),
    }
}

fn done(r: &Release) -> i32 {
    println!("\nRflyTools {} is installed. Restart any RflyTools programs to use it.", r.version);
    0
}
