//! Dedicated RflyGame server for LAN or internet play.
//!
//!   rflygame-server [--port 27150] [--name "My server"] [--no-lan]
//!
//! For internet play run it on a host with a public IP (open UDP <port>), then players use
//! Multiplayer → "Join by address" with host:port. The first player to join is the host.
use rflygame::net::{spawn_server, ServerConfig, DEFAULT_PORT};
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let get = |k: &str| a.iter().position(|x| x == k).and_then(|i| a.get(i + 1)).cloned();
    if a.iter().any(|x| x == "-h" || x == "--help") {
        println!("rflygame-server [--port {DEFAULT_PORT}] [--name NAME] [--no-lan]");
        return;
    }
    let cfg = ServerConfig {
        port: get("--port").and_then(|p| p.parse().ok()).unwrap_or(DEFAULT_PORT),
        name: get("--name").unwrap_or_else(|| "RflyGame server".into()),
        lan_beacon: !a.iter().any(|x| x == "--no-lan"),
    };
    let stop = Arc::new(AtomicBool::new(false));
    if let Err(e) = spawn_server(cfg, stop) {
        eprintln!("rflygame-server: {e}");
        std::process::exit(1);
    }
    loop {
        std::thread::park();
    }
}
