//! Maintainer tool: Ed25519 keys and signatures for RflyTools releases (not shipped to users).
//!   rfly-sign keygen DIR          → DIR/release.key (secret, 0600) + DIR/release.pub
//!   rfly-sign sign KEY FILE       → FILE.sig
//!   rfly-sign verify FILE SIG     (against the keys built into rfly-update)
use base64::Engine as _;
use ed25519_dalek::{Signer, SigningKey};
use std::io::Read;
use std::path::Path;

fn b64() -> base64::engine::GeneralPurpose {
    base64::engine::general_purpose::STANDARD
}

fn load_key(path: &str) -> SigningKey {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| die(&format!("{path}: {e}")));
    let line = text.lines().map(str::trim).find(|l| !l.is_empty() && !l.starts_with('#')).unwrap_or("");
    let raw = b64().decode(line).unwrap_or_else(|_| die("key file is not base64"));
    let seed: [u8; 32] = raw.as_slice().try_into().unwrap_or_else(|_| die("key must be 32 bytes"));
    SigningKey::from_bytes(&seed)
}

fn die(m: &str) -> ! {
    eprintln!("rfly-sign: {m}");
    std::process::exit(1)
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    match a.iter().map(String::as_str).collect::<Vec<_>>().as_slice() {
        ["keygen", dir] => {
            let d = Path::new(dir);
            let key = d.join("release.key");
            if key.exists() {
                die(&format!("{} exists; refusing to overwrite a release key", key.display()));
            }
            std::fs::create_dir_all(d).unwrap_or_else(|e| die(&e.to_string()));
            let mut seed = [0u8; 32];
            std::fs::File::open("/dev/urandom").and_then(|mut f| f.read_exact(&mut seed)).unwrap_or_else(|e| die(&e.to_string()));
            let sk = SigningKey::from_bytes(&seed);
            #[cfg(unix)]
            {
                use std::os::unix::fs::OpenOptionsExt;
                use std::io::Write;
                let mut f = std::fs::OpenOptions::new().write(true).create_new(true).mode(0o600).open(&key).unwrap_or_else(|e| die(&e.to_string()));
                writeln!(f, "# RflyTools release-signing SECRET key. Keep it private and backed up.\n{}", b64().encode(seed)).unwrap();
            }
            let pubk = b64().encode(sk.verifying_key().to_bytes());
            std::fs::write(d.join("release.pub"), format!("{pubk}\n")).unwrap();
            println!("{pubk}");
        }
        ["sign", key, file] => {
            let sk = load_key(key);
            let data = std::fs::read(file).unwrap_or_else(|e| die(&format!("{file}: {e}")));
            let sig = b64().encode(sk.sign(&data).to_bytes());
            std::fs::write(format!("{file}.sig"), format!("{sig}\n")).unwrap();
            // Self-check against the built-in keys so a wrong key is caught before publishing.
            match rfly_update::verify_signature(&data, &sig) {
                Ok(()) => println!("signed {file} → {file}.sig (verifies with the built-in release key)"),
                Err(e) => die(&format!("signed, but {e}: is this key in crates/rfly-update/release-keys.pub?")),
            }
        }
        ["verify", file, sig] => {
            let data = std::fs::read(file).unwrap_or_else(|e| die(&e.to_string()));
            let s = std::fs::read_to_string(sig).unwrap_or_else(|e| die(&e.to_string()));
            match rfly_update::verify_signature(&data, &s) {
                Ok(()) => println!("OK: {file} is signed by an RflyTools release key"),
                Err(e) => die(&e),
            }
        }
        ["pubkey", key] => println!("{}", b64().encode(load_key(key).verifying_key().to_bytes())),
        _ => die("usage: rfly-sign keygen DIR | sign KEY FILE | verify FILE SIG | pubkey KEY"),
    }
}
