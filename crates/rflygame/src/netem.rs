//! A UDP proxy that makes a local link behave like the internet: added latency, jitter
//! (which also reorders packets), loss and duplication, applied in both directions.
//! Each client behind the proxy gets its own upstream socket, so the server sees one
//! address per player, exactly as it would through NAT.
//!
//!   client ──> 127.0.0.1:<listen> ──(delay, jitter, loss)──> server
//!
//! Used by the multiplayer tests and by the `rflygame-netem` tool.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug)]
pub struct Impair {
    /// One-way base latency, ms.
    pub delay_ms: f64,
    /// Extra one-way latency, uniform 0..jitter, ms.
    pub jitter_ms: f64,
    /// Probability that a packet is dropped (each direction).
    pub loss: f64,
    /// Probability that a packet is delivered twice.
    pub dup: f64,
}

impl Impair {
    /// A long-distance home connection: ~180 ms round trip, 5 % loss.
    pub const INTERNET: Impair = Impair { delay_ms: 70.0, jitter_ms: 40.0, loss: 0.05, dup: 0.01 };
}

#[derive(Default)]
pub struct Stats {
    pub forwarded: AtomicU64,
    pub dropped: AtomicU64,
}

struct Rng(u64);
impl Rng {
    fn f(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Starts the proxy on `listen` (0 = any free port) forwarding to `upstream`.
/// Returns the bound port and live counters; runs until `stop` is set.
pub fn spawn_proxy(listen: u16, upstream: SocketAddr, imp: Impair, seed: u64, stop: Arc<AtomicBool>) -> std::io::Result<(u16, Arc<Stats>)> {
    let front = UdpSocket::bind(("0.0.0.0", listen))?;
    front.set_nonblocking(true)?;
    let port = front.local_addr()?.port();
    let stats = Arc::new(Stats::default());
    let st = stats.clone();
    std::thread::spawn(move || {
        let mut rng = Rng(seed.max(1));
        // client address → its upstream socket
        let mut back: HashMap<SocketAddr, UdpSocket> = HashMap::new();
        // (deliver at, seq) → (to upstream?, client, bytes)
        let mut queue: BinaryHeap<Reverse<(Instant, u64)>> = BinaryHeap::new();
        let mut pkts: HashMap<u64, (bool, SocketAddr, Vec<u8>)> = HashMap::new();
        let mut seq = 0u64;
        let mut buf = [0u8; 4096];
        let mut schedule = |rng: &mut Rng, up: bool, client: SocketAddr, data: &[u8], queue: &mut BinaryHeap<Reverse<(Instant, u64)>>, pkts: &mut HashMap<u64, (bool, SocketAddr, Vec<u8>)>| {
            if rng.f() < imp.loss {
                st.dropped.fetch_add(1, Ordering::Relaxed);
                return;
            }
            let copies = if rng.f() < imp.dup { 2 } else { 1 };
            for _ in 0..copies {
                let d = imp.delay_ms + rng.f() * imp.jitter_ms;
                seq += 1;
                queue.push(Reverse((Instant::now() + Duration::from_secs_f64(d / 1000.0), seq)));
                pkts.insert(seq, (up, client, data.to_vec()));
            }
        };
        while !stop.load(Ordering::Relaxed) {
            while let Ok((n, from)) = front.recv_from(&mut buf) {
                if !back.contains_key(&from) {
                    let Ok(s) = UdpSocket::bind("0.0.0.0:0") else { continue };
                    let _ = s.set_nonblocking(true);
                    back.insert(from, s);
                }
                schedule(&mut rng, true, from, &buf[..n], &mut queue, &mut pkts);
            }
            let clients: Vec<SocketAddr> = back.keys().copied().collect();
            for c in clients {
                while let Ok((n, from)) = back[&c].recv_from(&mut buf) {
                    if from == upstream {
                        schedule(&mut rng, false, c, &buf[..n], &mut queue, &mut pkts);
                    }
                }
            }
            while queue.peek().is_some_and(|Reverse((t, _))| *t <= Instant::now()) {
                let Reverse((_, k)) = queue.pop().unwrap();
                if let Some((up, client, data)) = pkts.remove(&k) {
                    let _ = if up { back[&client].send_to(&data, upstream) } else { front.send_to(&data, client) };
                    st.forwarded.fetch_add(1, Ordering::Relaxed);
                }
            }
            std::thread::sleep(Duration::from_micros(500));
        }
    });
    Ok((port, stats))
}
