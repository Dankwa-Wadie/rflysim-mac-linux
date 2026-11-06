//! Multiplayer over UDP: one protocol for LAN (with broadcast discovery) and internet play
//! (dedicated `rflygame-server`). Clients are authoritative for their own drone; the server
//! keeps the roster, relays states at 20 Hz and runs the lobby/race start.
//!
//! Every packet: b"RFG2", u8 type, payload. Strings are u8 length + UTF-8.

use std::collections::{HashMap, VecDeque};
use std::net::{Ipv4Addr, SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

pub const DEFAULT_PORT: u16 = 27150;
pub const BEACON_PORT: u16 = 27151;
/// Bumped whenever the wire format changes, so mismatched versions ignore each other.
const MAGIC: &[u8; 4] = b"RFG2";

const T_JOIN: u8 = 1;
const T_WELCOME: u8 = 2;
const T_STATE: u8 = 3;
const T_SNAPSHOT: u8 = 4;
const T_ROSTER: u8 = 5;
const T_ACTIVITY: u8 = 6;
const T_HOSTCMD: u8 = 7;
const T_LEAVE: u8 = 8;
const T_BEACON: u8 = 9;
const T_PING: u8 = 10;
const T_PONG: u8 = 11;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DroneState {
    pub pos: [f32; 3],
    pub quat: [f32; 4],
    pub spin: f32,
    /// Gates passed in the current race.
    pub progress: u16,
    /// Finish time (s) or −1.
    pub finish: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PlayerInfo {
    pub id: u8,
    pub name: String,
    pub color: u32,
    pub drone: String,
    pub host: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Activity {
    Lobby,
    /// Free flight on a map.
    Free(String),
    /// Race on a track (name).
    Race(String),
}

// ---------------------------------------------------------------- codec

#[derive(Default)]
struct W(Vec<u8>);
impl W {
    fn new(t: u8) -> Self {
        let mut v = MAGIC.to_vec();
        v.push(t);
        W(v)
    }
    fn u8(&mut self, v: u8) -> &mut Self {
        self.0.push(v);
        self
    }
    fn u16(&mut self, v: u16) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn u32(&mut self, v: u32) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn f32(&mut self, v: f32) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn s(&mut self, v: &str) -> &mut Self {
        let b = &v.as_bytes()[..v.len().min(60)];
        self.u8(b.len() as u8);
        self.0.extend_from_slice(b);
        self
    }
    fn state(&mut self, s: &DroneState) -> &mut Self {
        for v in s.pos.iter().chain(s.quat.iter()) {
            self.f32(*v);
        }
        self.f32(s.spin).u16(s.progress).f32(s.finish)
    }
    fn activity(&mut self, a: &Activity) -> &mut Self {
        match a {
            Activity::Lobby => self.u8(0).s(""),
            Activity::Free(m) => self.u8(1).s(m),
            Activity::Race(t) => self.u8(2).s(t),
        }
    }
}

struct R<'a>(&'a [u8], usize);
impl R<'_> {
    fn u8(&mut self) -> Option<u8> {
        let v = *self.0.get(self.1)?;
        self.1 += 1;
        Some(v)
    }
    fn take<const N: usize>(&mut self) -> Option<[u8; N]> {
        let v = self.0.get(self.1..self.1 + N)?.try_into().ok()?;
        self.1 += N;
        Some(v)
    }
    fn u16(&mut self) -> Option<u16> {
        self.take::<2>().map(u16::from_le_bytes)
    }
    fn u32(&mut self) -> Option<u32> {
        self.take::<4>().map(u32::from_le_bytes)
    }
    fn f32(&mut self) -> Option<f32> {
        self.take::<4>().map(f32::from_le_bytes)
    }
    fn s(&mut self) -> Option<String> {
        let n = self.u8()? as usize;
        let b = self.0.get(self.1..self.1 + n)?;
        self.1 += n;
        Some(String::from_utf8_lossy(b).into_owned())
    }
    fn state(&mut self) -> Option<DroneState> {
        let mut s = DroneState::default();
        for v in s.pos.iter_mut() {
            *v = self.f32()?;
        }
        for v in s.quat.iter_mut() {
            *v = self.f32()?;
        }
        s.spin = self.f32()?;
        s.progress = self.u16()?;
        s.finish = self.f32()?;
        Some(s)
    }
    fn activity(&mut self) -> Option<Activity> {
        let k = self.u8()?;
        let s = self.s()?;
        Some(match k {
            1 => Activity::Free(s),
            2 => Activity::Race(s),
            _ => Activity::Lobby,
        })
    }
}

fn header(b: &[u8]) -> Option<(u8, R<'_>)> {
    (b.len() >= 5 && &b[..4] == MAGIC).then(|| (b[4], R(b, 5)))
}

// ---------------------------------------------------------------- server

struct Peer {
    info: PlayerInfo,
    state: DroneState,
    /// Sender's clock (ms) when `state` was produced.
    t: u32,
    last: Instant,
    /// Last host-command sequence number applied from this peer.
    cmd_seq: u16,
}

pub struct ServerConfig {
    pub port: u16,
    pub name: String,
    pub lan_beacon: bool,
}

/// A peer is dropped after this long without any packet (pings arrive every 0.5 s).
const PEER_TIMEOUT: Duration = Duration::from_secs(6);

/// Runs the server until `stop` is set. Returns the bound port.
pub fn spawn_server(cfg: ServerConfig, stop: Arc<AtomicBool>) -> std::io::Result<u16> {
    let sock = UdpSocket::bind(("0.0.0.0", cfg.port))?;
    let port = sock.local_addr()?.port();
    sock.set_read_timeout(Some(Duration::from_millis(5)))?;
    let beacon = UdpSocket::bind("0.0.0.0:0")?;
    let _ = beacon.set_broadcast(true);
    std::thread::spawn(move || server_loop(sock, beacon, cfg, port, stop));
    Ok(port)
}

fn server_loop(sock: UdpSocket, beacon: UdpSocket, cfg: ServerConfig, port: u16, stop: Arc<AtomicBool>) {
    let epoch = Instant::now();
    let mut peers: HashMap<SocketAddr, Peer> = HashMap::new();
    let mut next_id: u8 = 1;
    let mut activity = Activity::Lobby;
    let mut start_at: Option<Instant> = None;
    let (mut t_snap, mut t_roster, mut t_beacon) = (Instant::now(), Instant::now(), Instant::now() - Duration::from_secs(5));
    let mut buf = [0u8; 2048];
    println!("rflygame-server: '{}' on UDP {port}{}", cfg.name, if cfg.lan_beacon { " (LAN beacon on)" } else { "" });
    while !stop.load(Ordering::Relaxed) {
        let mut roster_dirty = false;
        // Drain everything queued (bounded so the send timers still run).
        for _ in 0..256 {
            let Ok((n, from)) = sock.recv_from(&mut buf) else { break };
            let Some((t, mut r)) = header(&buf[..n]) else { continue };
            if let Some(p) = peers.get_mut(&from) {
                p.last = Instant::now();
            }
            match t {
                T_JOIN => {
                    if let (Some(name), Some(color), Some(drone)) = (r.s(), r.u32(), r.s()) {
                        let known = peers.get(&from).map(|p| (p.info.id, p.info.host, p.cmd_seq));
                        let (id, host, cmd_seq) = known.unwrap_or_else(|| {
                            let id = next_id;
                            next_id = next_id.wrapping_add(1).max(1);
                            (id, peers.values().all(|p| !p.info.host), 0)
                        });
                        if known.is_none() {
                            println!("rflygame-server: {name} joined from {from} as #{id}{}", if host { " (host)" } else { "" });
                        }
                        let state = peers.get(&from).map_or(DroneState { finish: -1.0, ..Default::default() }, |p| p.state);
                        let t = peers.get(&from).map_or(0, |p| p.t);
                        peers.insert(from, Peer { info: PlayerInfo { id, name, color, drone, host }, state, t, last: Instant::now(), cmd_seq });
                        let mut w = W::new(T_WELCOME);
                        w.u8(id).s(&cfg.name);
                        let _ = sock.send_to(&w.0, from);
                        roster_dirty = true;
                    }
                }
                T_STATE => {
                    if let (Some(p), Some(t), Some(s)) = (peers.get_mut(&from), r.u32(), r.state()) {
                        if t > p.t || p.t.wrapping_sub(t) > 60_000 {
                            (p.state, p.t) = (s, t); // ignore reordered (older) states
                        }
                    }
                }
                T_PING => {
                    if let (true, Some(ms)) = (peers.contains_key(&from), r.u32()) {
                        let mut w = W::new(T_PONG);
                        w.u32(ms);
                        let _ = sock.send_to(&w.0, from);
                    }
                }
                T_HOSTCMD => {
                    let (Some(seq), Some(a)) = (r.u16(), r.activity()) else { continue };
                    let Some(p) = peers.get_mut(&from).filter(|p| p.info.host) else { continue };
                    if p.cmd_seq == seq {
                        continue; // a retransmit we already applied
                    }
                    p.cmd_seq = seq;
                    start_at = matches!(a, Activity::Race(_)).then(|| Instant::now() + Duration::from_secs(5));
                    println!("rflygame-server: activity → {a:?}");
                    activity = a;
                    for p in peers.values_mut() {
                        p.state.progress = 0;
                        p.state.finish = -1.0;
                    }
                    roster_dirty = true;
                }
                T_LEAVE => {
                    if let Some(p) = peers.remove(&from) {
                        println!("rflygame-server: {} left", p.info.name);
                        roster_dirty = true;
                    }
                }
                _ => {}
            }
        }
        // Timeouts; pass host on if needed.
        let before = peers.len();
        peers.retain(|a, p| {
            let keep = p.last.elapsed() < PEER_TIMEOUT;
            if !keep {
                println!("rflygame-server: {} ({a}) timed out", p.info.name);
            }
            keep
        });
        if peers.len() != before {
            roster_dirty = true;
        }
        if !peers.is_empty() && peers.values().all(|p| !p.info.host) {
            if let Some(p) = peers.values_mut().min_by_key(|p| p.info.id) {
                p.info.host = true;
            }
        }
        if t_snap.elapsed() >= Duration::from_millis(50) {
            t_snap = Instant::now();
            let mut w = W::new(T_SNAPSHOT);
            w.u32(epoch.elapsed().as_millis() as u32).u8(peers.len() as u8);
            for p in peers.values() {
                w.u8(p.info.id).u32(p.t).state(&p.state);
            }
            for a in peers.keys() {
                let _ = sock.send_to(&w.0, a);
            }
        }
        if roster_dirty || t_roster.elapsed() >= Duration::from_secs(1) {
            t_roster = Instant::now();
            let mut w = W::new(T_ROSTER);
            w.u8(peers.len() as u8);
            for p in peers.values() {
                w.u8(p.info.id).s(&p.info.name).u32(p.info.color).s(&p.info.drone).u8(p.info.host as u8);
            }
            let ms = start_at.map_or(0, |s| s.saturating_duration_since(Instant::now()).as_millis() as u32);
            for (addr, p) in &peers {
                let mut a = W::new(T_ACTIVITY);
                a.activity(&activity).u32(ms).u16(p.cmd_seq);
                let _ = sock.send_to(&w.0, addr);
                let _ = sock.send_to(&a.0, addr);
            }
        }
        if cfg.lan_beacon && t_beacon.elapsed() >= Duration::from_secs(1) {
            t_beacon = Instant::now();
            let mut w = W::new(T_BEACON);
            let act = match &activity {
                Activity::Lobby => "lobby".to_string(),
                Activity::Free(m) => format!("free flight: {m}"),
                Activity::Race(t) => format!("race: {t}"),
            };
            w.s(&cfg.name).u16(port).u8(peers.len() as u8).s(&act);
            let _ = beacon.send_to(&w.0, (Ipv4Addr::BROADCAST, BEACON_PORT));
            let _ = beacon.send_to(&w.0, (Ipv4Addr::LOCALHOST, BEACON_PORT));
        }
    }
    println!("rflygame-server: stopped");
}

// ---------------------------------------------------------------- client

/// Remote drones are drawn a little in the past so there are always two states to
/// interpolate between. The delay adapts to each player's measured jitter, within these bounds.
pub const INTERP_DELAY_MS: (f64, f64) = (70.0, 300.0);

pub struct Remote {
    pub info: PlayerInfo,
    /// Latest received state (standings use this).
    pub state: DroneState,
    /// Recent states as (sender ms, state), oldest first.
    hist: VecDeque<(f64, DroneState)>,
    /// Estimate of (our ms − sender ms), tracking the fastest packets, and the jitter above it.
    offset: Option<f64>,
    jitter: f64,
}

impl Remote {
    fn new(info: PlayerInfo) -> Remote {
        Remote { info, state: DroneState { finish: -1.0, ..Default::default() }, hist: VecDeque::new(), offset: None, jitter: 0.0 }
    }

    /// Interpolation delay for this player, ms.
    pub fn delay_ms(&self) -> f64 {
        (40.0 + 2.5 * self.jitter).clamp(INTERP_DELAY_MS.0, INTERP_DELAY_MS.1)
    }
}

pub struct Client {
    sock: UdpSocket,
    pub server: SocketAddr,
    pub id: Option<u8>,
    pub server_name: String,
    pub players: Vec<Remote>,
    pub activity: Activity,
    /// Local time when the current race starts.
    pub start_at: Option<Instant>,
    /// Smoothed round-trip time to the server, ms.
    pub rtt_ms: Option<f64>,
    epoch: Instant,
    last_rx: Instant,
    last_ping: Instant,
    join: (String, u32, String),
    last_join: Instant,
    /// Host command waiting for the server's acknowledgement: (seq, activity, last sent).
    pending: Option<(u16, Activity, Instant)>,
    cmd_seq: u16,
    last_snap: f64,
}

impl Client {
    pub fn connect(server: &str, name: &str, color: u32, drone: &str) -> std::io::Result<Client> {
        use std::net::ToSocketAddrs;
        let server = if server.contains(':') { server.to_string() } else { format!("{server}:{DEFAULT_PORT}") };
        let addr = server.to_socket_addrs()?.find(|a| a.is_ipv4()).ok_or_else(|| std::io::Error::other("no IPv4 address"))?;
        let sock = UdpSocket::bind("0.0.0.0:0")?;
        sock.set_nonblocking(true)?;
        let now = Instant::now();
        let mut c = Client {
            sock,
            server: addr,
            id: None,
            server_name: String::new(),
            players: vec![],
            activity: Activity::Lobby,
            start_at: None,
            rtt_ms: None,
            epoch: now,
            last_rx: now,
            last_ping: now,
            join: (name.into(), color, drone.into()),
            last_join: now - Duration::from_secs(5),
            pending: None,
            cmd_seq: 0,
            last_snap: f64::MIN,
        };
        c.send_join();
        Ok(c)
    }

    fn send_join(&mut self) {
        let mut w = W::new(T_JOIN);
        w.s(&self.join.0).u32(self.join.1).s(&self.join.2);
        let _ = self.sock.send_to(&w.0, self.server);
        self.last_join = Instant::now();
    }

    fn now_ms(&self) -> f64 {
        self.epoch.elapsed().as_secs_f64() * 1000.0
    }

    pub fn connected(&self) -> bool {
        self.id.is_some() && self.last_rx.elapsed() < Duration::from_secs(5)
    }

    pub fn is_host(&self) -> bool {
        self.me().is_some_and(|m| m.info.host)
    }

    pub fn me(&self) -> Option<&Remote> {
        self.players.iter().find(|p| Some(p.info.id) == self.id)
    }

    /// Processes everything received; keeps the link alive, re-joins after an outage and
    /// retransmits an unacknowledged host command. Call every frame.
    pub fn poll(&mut self) {
        let mut buf = [0u8; 4096];
        while let Ok((n, from)) = self.sock.recv_from(&mut buf) {
            if from != self.server {
                continue;
            }
            let Some((t, mut r)) = header(&buf[..n]) else { continue };
            self.last_rx = Instant::now();
            match t {
                T_WELCOME => {
                    self.id = r.u8();
                    self.server_name = r.s().unwrap_or_default();
                }
                T_PONG => {
                    if let Some(ms) = r.u32() {
                        let rtt = (self.now_ms() - ms as f64).max(0.0);
                        self.rtt_ms = Some(self.rtt_ms.map_or(rtt, |x| x + (rtt - x) * 0.2));
                    }
                }
                T_ROSTER => {
                    let n = r.u8().unwrap_or(0);
                    let mut infos = vec![];
                    for _ in 0..n {
                        if let (Some(id), Some(name), Some(color), Some(drone), Some(host)) = (r.u8(), r.s(), r.u32(), r.s(), r.u8()) {
                            infos.push(PlayerInfo { id, name, color, drone, host: host != 0 });
                        }
                    }
                    let mut old = std::mem::take(&mut self.players);
                    self.players = infos
                        .into_iter()
                        .map(|info| match old.iter().position(|o| o.info.id == info.id) {
                            Some(i) => Remote { info, ..old.swap_remove(i) },
                            None => Remote::new(info),
                        })
                        .collect();
                }
                T_SNAPSHOT => {
                    let Some(st) = r.u32().map(|v| v as f64) else { continue };
                    if st <= self.last_snap {
                        continue; // late or duplicated: a newer snapshot already arrived
                    }
                    self.last_snap = st;
                    let now = self.now_ms();
                    let n = r.u8().unwrap_or(0);
                    for _ in 0..n {
                        let (Some(id), Some(t), Some(s)) = (r.u8(), r.u32(), r.state()) else { break };
                        let Some(p) = self.players.iter_mut().find(|p| p.info.id == id) else { continue };
                        p.state = s;
                        let t = t as f64;
                        if p.hist.back().is_some_and(|h| t <= h.0) {
                            continue; // the server repeated a state we already have
                        }
                        p.hist.push_back((t, s));
                        while p.hist.len() > 40 {
                            p.hist.pop_front();
                        }
                        // Follow the fastest packet at once; drift up slowly (clock skew).
                        let sample = now - t;
                        let o = match p.offset {
                            Some(o) if sample >= o => o + (sample - o) * 0.002,
                            _ => sample,
                        };
                        p.offset = Some(o);
                        p.jitter += ((sample - o) - p.jitter) * 0.05;
                    }
                }
                T_ACTIVITY => {
                    if let (Some(a), Some(ms), Some(seq)) = (r.activity(), r.u32(), r.u16()) {
                        if a != self.activity || (ms > 0 && self.start_at.is_none()) {
                            self.start_at = (ms > 0 || matches!(a, Activity::Race(_))).then(|| Instant::now() + Duration::from_millis(ms as u64));
                        }
                        self.activity = a;
                        if self.pending.as_ref().is_some_and(|p| p.0 == seq) {
                            self.pending = None;
                        }
                    }
                }
                _ => {}
            }
        }
        // Join, or re-join if the server has gone quiet (it may have restarted or dropped us).
        let quiet = self.last_rx.elapsed() > Duration::from_secs(2);
        if (self.id.is_none() || quiet) && self.last_join.elapsed() > Duration::from_millis(500) {
            self.send_join();
        }
        if self.id.is_some() && self.last_ping.elapsed() >= Duration::from_millis(500) {
            self.last_ping = Instant::now();
            let mut w = W::new(T_PING);
            w.u32(self.now_ms() as u32);
            let _ = self.sock.send_to(&w.0, self.server);
        }
        let resend = self.pending.as_ref().is_some_and(|p| p.2.elapsed() > Duration::from_millis(250));
        if resend {
            if let Some(p) = self.pending.as_mut() {
                p.2 = Instant::now();
            }
            self.send_cmd();
        }
    }

    pub fn send_state(&self, s: &DroneState) {
        let mut w = W::new(T_STATE);
        w.u32(self.now_ms() as u32).state(s);
        let _ = self.sock.send_to(&w.0, self.server);
    }

    /// Asks the server to switch everyone to `a` (host only). Retransmitted by `poll` until
    /// the server acknowledges it; asking again for the same thing while it is in flight is a
    /// no-op.
    pub fn host_cmd(&mut self, a: &Activity) {
        if self.pending.as_ref().is_some_and(|p| &p.1 == a) {
            return;
        }
        self.cmd_seq = self.cmd_seq.wrapping_add(1).max(1);
        self.pending = Some((self.cmd_seq, a.clone(), Instant::now()));
        self.send_cmd();
    }

    fn send_cmd(&self) {
        if let Some((seq, a, _)) = &self.pending {
            let mut w = W::new(T_HOSTCMD);
            w.u16(*seq).activity(a);
            let _ = self.sock.send_to(&w.0, self.server);
        }
    }

    /// Where to draw a remote drone now: interpolated between its states a jitter-sized
    /// delay in the past (see `Remote::delay_ms`), extrapolated briefly when packets are late.
    pub fn view(&self, p: &Remote) -> DroneState {
        let (Some(off), Some(last)) = (p.offset, p.hist.back()) else { return p.state };
        let t = self.now_ms() - off - p.delay_ms();
        let i = p.hist.iter().position(|h| h.0 > t);
        let (a, b) = match i {
            Some(0) => return p.hist[0].1,
            Some(i) => (p.hist[i - 1], p.hist[i]),
            None if p.hist.len() >= 2 => {
                // Late: extrapolate along the last segment for at most 150 ms, then hold.
                let a = p.hist[p.hist.len() - 2];
                let ext = (t - last.0).min(150.0);
                let k = 1.0 + ext / (last.0 - a.0).max(1.0);
                return blend(&a.1, &last.1, k as f32);
            }
            None => return last.1,
        };
        blend(&a.1, &b.1, ((t - a.0) / (b.0 - a.0).max(1.0)) as f32)
    }

    pub fn leave(&self) {
        let _ = self.sock.send_to(&W::new(T_LEAVE).0, self.server);
    }
}

/// Interpolates (k in 0..1) or extrapolates (k > 1) position and attitude; race data from `b`.
fn blend(a: &DroneState, b: &DroneState, k: f32) -> DroneState {
    let mut s = *b;
    for i in 0..3 {
        s.pos[i] = a.pos[i] + (b.pos[i] - a.pos[i]) * k;
    }
    let dot: f32 = (0..4).map(|i| a.quat[i] * b.quat[i]).sum();
    let sg = if dot < 0.0 { -1.0 } else { 1.0 };
    let kq = k.min(1.0);
    let mut q = [0.0f32; 4];
    for i in 0..4 {
        q[i] = a.quat[i] * (1.0 - kq) + b.quat[i] * sg * kq;
    }
    let n = q.iter().map(|v| v * v).sum::<f32>().sqrt();
    if n > 1e-6 {
        s.quat = q.map(|v| v / n);
    }
    s.spin = a.spin + (b.spin - a.spin) * kq;
    s
}

impl Drop for Client {
    fn drop(&mut self) {
        self.leave();
    }
}

// ---------------------------------------------------------------- LAN discovery

#[derive(Clone, Debug)]
pub struct Found {
    pub addr: SocketAddr,
    pub name: String,
    pub players: u8,
    pub activity: String,
    pub seen: Instant,
}

pub struct Browser {
    sock: Option<UdpSocket>,
    pub found: Vec<Found>,
}

impl Browser {
    pub fn new() -> Self {
        let sock = socket2::Socket::new(socket2::Domain::IPV4, socket2::Type::DGRAM, Some(socket2::Protocol::UDP)).ok().and_then(|s| {
            s.set_reuse_address(true).ok()?;
            s.set_reuse_port(true).ok()?;
            s.bind(&std::net::SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, BEACON_PORT).into()).ok()?;
            let u: UdpSocket = s.into();
            u.set_nonblocking(true).ok()?;
            Some(u)
        });
        Browser { sock, found: vec![] }
    }

    pub fn poll(&mut self) {
        let Some(s) = &self.sock else { return };
        let mut buf = [0u8; 512];
        while let Ok((n, from)) = s.recv_from(&mut buf) {
            let Some((T_BEACON, mut r)) = header(&buf[..n]) else { continue };
            if let (Some(name), Some(port), Some(players), Some(activity)) = (r.s(), r.u16(), r.u8(), r.s()) {
                let addr = SocketAddr::new(from.ip(), port);
                // Prefer the LAN address over loopback for the same server.
                self.found.retain(|f| f.addr != addr && !(f.addr.port() == port && f.name == name && (f.addr.ip().is_loopback() || addr.ip().is_loopback()) && !addr.ip().is_loopback()));
                if self.found.iter().any(|f| f.addr.port() == port && f.name == name) {
                    continue;
                }
                self.found.push(Found { addr, name, players, activity, seen: Instant::now() });
            }
        }
        self.found.retain(|f| f.seen.elapsed() < Duration::from_secs(4));
    }
}

impl Default for Browser {
    fn default() -> Self {
        Self::new()
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::netem::{spawn_proxy, Impair};

    fn wait_for(secs: f64, mut f: impl FnMut() -> bool) -> bool {
        let t = Instant::now();
        while t.elapsed().as_secs_f64() < secs {
            if f() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        false
    }

    #[test]
    fn two_players_see_each_other_and_race_starts() {
        let stop = Arc::new(AtomicBool::new(false));
        let port = spawn_server(ServerConfig { port: 0, name: "test".into(), lan_beacon: false }, stop.clone()).unwrap();
        let addr = format!("127.0.0.1:{port}");
        let mut a = Client::connect(&addr, "Alice", 0xff0000ff, "Racer").unwrap();
        assert!(wait_for(3.0, || {
            a.poll();
            a.connected()
        }));
        let mut b = Client::connect(&addr, "Bob", 0x00ff00ff, "Trainer").unwrap();
        assert!(wait_for(3.0, || {
            a.poll();
            b.poll();
            a.players.len() == 2 && b.players.len() == 2
        }));
        assert!(a.is_host() && !b.is_host(), "first player hosts");
        let s = DroneState { pos: [1.0, 2.0, -3.0], quat: [0.0, 0.0, 0.0, 1.0], spin: 0.5, progress: 4, finish: -1.0 };
        assert!(wait_for(3.0, || {
            b.send_state(&s);
            a.poll();
            b.poll();
            a.players.iter().any(|p| p.info.name == "Bob" && p.state.pos == [1.0, 2.0, -3.0] && p.state.progress == 4)
        }));
        // Only the host can start a race.
        b.host_cmd(&Activity::Race("Meadow Mini".into()));
        assert!(!wait_for(0.4, || {
            a.poll();
            b.poll();
            a.activity != Activity::Lobby
        }));
        a.host_cmd(&Activity::Race("Meadow Mini".into()));
        assert!(wait_for(3.0, || {
            a.poll();
            b.poll();
            b.activity == Activity::Race("Meadow Mini".into()) && b.start_at.is_some()
        }));
        stop.store(true, Ordering::Relaxed);
    }

    /// Both players behind an internet-like link (≈180 ms round trip, jitter that reorders
    /// packets, 5–15 % loss): they join, idle in the lobby past the server timeout, start a
    /// race on a synchronised countdown, and see each other's drone move smoothly.
    #[test]
    fn internet_conditions() {
        let stop = Arc::new(AtomicBool::new(false));
        let port = spawn_server(ServerConfig { port: 0, name: "net".into(), lan_beacon: false }, stop.clone()).unwrap();
        let up: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
        let (pa, _) = spawn_proxy(0, up, Impair::INTERNET, 7, stop.clone()).unwrap();
        let (pb, sb) = spawn_proxy(0, up, Impair { delay_ms: 90.0, jitter_ms: 60.0, loss: 0.15, dup: 0.02 }, 11, stop.clone()).unwrap();
        let mut a = Client::connect(&format!("127.0.0.1:{pa}"), "Alice", 1, "Racer").unwrap();
        let mut b = Client::connect(&format!("127.0.0.1:{pb}"), "Bob", 2, "Racer").unwrap();
        let both = |a: &mut Client, b: &mut Client| {
            a.poll();
            b.poll();
        };
        assert!(wait_for(6.0, || {
            both(&mut a, &mut b);
            a.players.len() == 2 && b.players.len() == 2 && a.connected() && b.connected()
        }), "both join");
        let host_is_a = a.is_host();
        // Sit in the lobby longer than the server's peer timeout.
        let t = Instant::now();
        while t.elapsed() < Duration::from_secs(8) {
            both(&mut a, &mut b);
            assert_eq!(a.players.len(), 2, "Alice still sees two players after {:.1}s", t.elapsed().as_secs_f64());
            assert_eq!(b.players.len(), 2, "Bob still sees two players");
            std::thread::sleep(Duration::from_millis(10));
        }
        let rtt = b.rtt_ms.expect("ping measured");
        assert!((150.0..450.0).contains(&rtt), "Bob's RTT {rtt:.0} ms");
        // Start a race with one call; the command must survive loss.
        let race = Activity::Race("Meadow Mini".into());
        let (h, o) = if host_is_a { (&mut a, &mut b) } else { (&mut b, &mut a) };
        h.host_cmd(&race);
        assert!(wait_for(4.0, || {
            h.poll();
            o.poll();
            h.activity == race && o.activity == race && h.start_at.is_some() && o.start_at.is_some()
        }), "race starts for both");
        let skew = h.start_at.unwrap().max(o.start_at.unwrap()) - h.start_at.unwrap().min(o.start_at.unwrap());
        assert!(skew < Duration::from_millis(250), "countdowns agree within {skew:?}");
        // Bob flies a straight line at 10 m/s; Alice's view must be smooth.
        let t0 = Instant::now();
        let (mut last_send, mut prev, mut max_step, mut raw_jumps) = (0.0, None::<f32>, 0.0f32, 0);
        let mut raw_prev = None::<f32>;
        while t0.elapsed() < Duration::from_secs(4) {
            let t = t0.elapsed().as_secs_f64();
            if t - last_send >= 0.05 {
                last_send = t;
                b.send_state(&DroneState { pos: [(t * 10.0) as f32, 0.0, -5.0], quat: [0.0, 0.0, 0.0, 1.0], spin: 0.5, progress: 0, finish: -1.0 });
            }
            both(&mut a, &mut b);
            if let Some(p) = a.players.iter().find(|p| p.info.name == "Bob") {
                let raw = p.state.pos[0];
                if raw_prev.is_some_and(|r| (raw - r).abs() > 0.9) {
                    raw_jumps += 1;
                }
                raw_prev = Some(raw);
                let x = a.view(p).pos[0];
                if t > 1.0 {
                    if let Some(px) = prev {
                        max_step = max_step.max((x - px).abs());
                        assert!(x >= px - 0.05, "view never goes backwards ({px} → {x})");
                    }
                    prev = Some(x);
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        // 10 m/s × 10 ms = 0.1 m per frame; a lost snapshot without smoothing would jump ≥1 m.
        assert!(max_step < 0.45, "largest per-frame step {max_step:.2} m (raw snapshots jumped {raw_jumps}×)");
        let lag = (t0.elapsed().as_secs_f64() * 10.0) as f32 - a.view(a.players.iter().find(|p| p.info.name == "Bob").unwrap()).pos[0];
        assert!((1.0..5.0).contains(&lag), "view lags {lag:.1} m (latency + interpolation delay)");
        assert!(sb.dropped.load(Ordering::Relaxed) > 10, "the lossy link really dropped packets");
        println!("internet test: Bob RTT {rtt:.0} ms, countdown skew {skew:?}, max view step {max_step:.2} m, raw jumps {raw_jumps}, lag {lag:.1} m, Bob's link dropped {}", sb.dropped.load(Ordering::Relaxed));
        stop.store(true, Ordering::Relaxed);
    }
}
