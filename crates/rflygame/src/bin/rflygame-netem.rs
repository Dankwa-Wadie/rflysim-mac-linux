//! Makes a local link behave like the internet, to try multiplayer under real conditions.
//!
//!   rflygame-netem [--listen 27160] [--to 127.0.0.1:27150] [--delay 70] [--jitter 40] [--loss 0.05] [--dup 0.01]
//!
//! Start rflygame-server (or host a game), run this, then join 127.0.0.1:27160 from a
//! second RflyGame. Delay and jitter are one-way, ms; loss and dup apply in each direction.
use rflygame::netem::{spawn_proxy, Impair};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.iter().any(|x| x == "-h" || x == "--help") {
        println!("rflygame-netem [--listen 27160] [--to 127.0.0.1:27150] [--delay 70] [--jitter 40] [--loss 0.05] [--dup 0.01]");
        return;
    }
    let get = |k: &str| a.iter().position(|x| x == k).and_then(|i| a.get(i + 1)).cloned();
    let num = |k: &str, d: f64| get(k).and_then(|v| v.parse().ok()).unwrap_or(d);
    let d = Impair::INTERNET;
    let imp = Impair { delay_ms: num("--delay", d.delay_ms), jitter_ms: num("--jitter", d.jitter_ms), loss: num("--loss", d.loss), dup: num("--dup", d.dup) };
    let to = get("--to").unwrap_or_else(|| "127.0.0.1:27150".into());
    let Ok(up) = to.parse() else {
        eprintln!("rflygame-netem: bad --to address {to}");
        std::process::exit(1);
    };
    let listen = num("--listen", 27160.0) as u16;
    let stop = Arc::new(AtomicBool::new(false));
    match spawn_proxy(listen, up, imp, 0x9e3779b97f4a7c15, stop.clone()) {
        Ok((port, st)) => {
            println!("rflygame-netem: UDP {port} → {up}  delay {} ms + 0..{} ms jitter, loss {:.0} %, dup {:.0} % (each way)", imp.delay_ms, imp.jitter_ms, imp.loss * 100.0, imp.dup * 100.0);
            loop {
                std::thread::sleep(Duration::from_secs(5));
                println!("rflygame-netem: forwarded {}, dropped {}", st.forwarded.load(Ordering::Relaxed), st.dropped.load(Ordering::Relaxed));
            }
        }
        Err(e) => {
            eprintln!("rflygame-netem: {e}");
            stop.store(true, Ordering::Relaxed);
            std::process::exit(1);
        }
    }
}
