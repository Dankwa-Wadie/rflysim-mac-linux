//! rflysim3d: an RflySim3D-compatible 3D viewer for macOS.
//!
//! It listens where the UE4 RflySim3D listens (UDP 20010+window, multicast 224.0.0.10:20009)
//! and understands the same packets: vehicle states from CopterSim/Python, console commands
//! (`RflyChangeMapbyName`, `RflyCameraPosAng`, …), labels, attachments and vision-sensor
//! requests. Camera images go back in RflySim's UDP image format, so
//! `VisionCaptureApi`-style clients keep working.
//!
//! Keys (as close to RflySim3D as practical):
//!   V cycle view (chase / FPV / top / orbit / free)      B / 0-9 select vehicle
//!   mouse drag orbit · wheel zoom · WASD QE move (free)   M next map
//!   T trails · L labels · P crash detection · H help      F1 hide HUD · Esc quit

mod net;
mod sensors;

use macroquad::prelude::*;
use rfly_proto::*;
use rflysim3d::render::*;
use rflysim3d::scene::*;
use sensors::*;
use std::collections::{HashMap, VecDeque};
use std::net::{SocketAddr, UdpSocket};
use std::sync::mpsc::{channel, Receiver, Sender};

struct Args {
    window: Option<u16>,
    map: String,
    width: i32,
    height: i32,
    screenshot: Option<String>,
    after: f64,
    /// Lighter rendering for integrated graphics / remote desktop (Windows SITLRunLowGPU).
    low_gpu: bool,
}

fn args() -> Args {
    let mut a = Args { window: None, map: "Grasslands".into(), width: 1280, height: 720, screenshot: None, after: 3.0, low_gpu: false };
    let v: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i < v.len() {
        let next = v.get(i + 1).cloned().unwrap_or_default();
        match v[i].as_str() {
            "--window" | "-w" => a.window = next.parse().ok(),
            "--map" | "-m" => a.map = next,
            "--width" => a.width = next.parse().unwrap_or(1280),
            "--height" => a.height = next.parse().unwrap_or(720),
            "--screenshot" => a.screenshot = Some(next),
            "--after" => a.after = next.parse().unwrap_or(3.0),
            "--low-gpu" => {
                a.low_gpu = true;
                i += 1;
                continue;
            }
            "--help" | "-h" => {
                println!(
                    "rflysim3d [--window N] [--map NAME] [--width W] [--height H] [--low-gpu] [--screenshot out.png --after SECS]\n\
                     Maps: {}",
                    MAP_NAMES.join(", ")
                );
                std::process::exit(0);
            }
            _ => {
                i += 1;
                continue;
            }
        }
        i += 2;
    }
    // Windows scripts select the low-GPU mode with UE4_MAP=LowGPU.
    if a.map.eq_ignore_ascii_case("LowGPU") {
        a.map = "Grasslands".into();
        a.low_gpu = true;
    }
    if a.low_gpu {
        a.width = a.width.min(960);
        a.height = a.height.min(540);
    }
    a
}

fn window_conf() -> macroquad::conf::Conf {
    let a = args();
    let mq = Conf {
        window_title: format!("RflySim3D ({OS})"),
        window_width: a.width,
        window_height: a.height,
        high_dpi: false,
        sample_count: if a.low_gpu { 1 } else { 4 },
        window_resizable: true,
        ..Default::default()
    };
    macroquad::conf::Conf {
        miniquad_conf: mq,
        draw_call_vertex_capacity: 65536,
        draw_call_index_capacity: 262144,
        ..Default::default()
    }
}

// ---------------------------------------------------------------- world state

struct Label {
    text: String,
    color: Color,
    size: f32,
    expires: Option<f64>,
}

struct Veh {
    id: i32,
    vtype: i32,
    model: VehicleModel,
    pos: Vec3,
    euler: [f32; 3],
    vel: Vec3,
    motors: [f32; 8],
    scale: Vec3,
    fit_ground: bool,
    time: f64,
    last_rx: f64,
    prop_angle: f32,
    trail: VecDeque<Vec3>,
    labels: Vec<Label>,
    /// (parent id, type, offset in parent frame, relative attitude)
    attach: Option<(i32, i32, Vec3, Quat)>,
}

impl Veh {
    fn new(id: i32, vtype: i32) -> Self {
        Veh {
            id,
            vtype,
            model: build_model(model_for_type(vtype), tint(id)),
            pos: Vec3::ZERO,
            euler: [0.0; 3],
            vel: Vec3::ZERO,
            motors: [0.0; 8],
            scale: Vec3::ONE,
            fit_ground: false,
            time: -1.0,
            last_rx: 0.0,
            prop_angle: 0.0,
            trail: VecDeque::new(),
            labels: Vec::new(),
            attach: None,
        }
    }
    fn set_type(&mut self, t: i32) {
        if t != self.vtype {
            self.vtype = t;
            self.model = build_model(model_for_type(t), tint(self.id));
        }
    }
    fn q(&self) -> Quat {
        rot(self.euler)
    }
    fn radius(&self) -> f32 {
        (self.model.half * self.scale).max_element()
    }
}

fn tint(id: i32) -> Color {
    const P: [Color; 6] = [
        Color::new(0.95, 0.95, 0.97, 1.),
        Color::new(0.95, 0.55, 0.15, 1.),
        Color::new(0.25, 0.55, 0.95, 1.),
        Color::new(0.30, 0.80, 0.35, 1.),
        Color::new(0.90, 0.30, 0.55, 1.),
        Color::new(0.95, 0.85, 0.25, 1.),
    ];
    P[(id.max(1) as usize - 1) % P.len()]
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum View {
    Chase,
    Fpv,
    Top,
    Orbit,
    Free,
}
const VIEWS: [View; 5] = [View::Chase, View::Fpv, View::Top, View::Orbit, View::Free];

struct Cam {
    view: View,
    focus: i32,
    az: f32,
    el: f32,
    dist: f32,
    free_pos: Vec3,
    free_yaw: f32,
    free_pitch: f32,
    fov_h_deg: f32,
    smooth_pos: Option<Vec3>,
}


struct ObjSub {
    op: i32,
    name: String,
    to: SocketAddr,
}

struct App {
    window: u16,
    map: Map,
    map_idx: usize,
    vehicles: HashMap<i32, Veh>,
    cam: Cam,
    sensors: Vec<Sensor>,
    messages: Vec<(String, f64)>,
    show_trails: bool,
    show_labels: bool,
    show_help: bool,
    show_hud: bool,
    crash_detect: bool,
    crash_sent: HashMap<(i32, i32), f64>,
    subs: Vec<ObjSub>,
    next_sub: f64,
    tx: UdpSocket,
    enc: Sender<Job>,
    inbox: Receiver<net::Inbox>,
    r: Renderer,
    t0: f64,
    gps_origin: [f64; 3],
    rx_count: u64,
    /// Newer RflyTools version from the background update check.
    update: Option<String>,
}

const OS: &str = if cfg!(target_os = "macos") { "macOS" } else if cfg!(windows) { "Windows" } else { "Linux" };

impl App {
    fn now(&self) -> f64 {
        get_time() - self.t0
    }
    fn msg(&mut self, s: impl Into<String>, secs: f64) {
        let s = s.into();
        println!("rflysim3d: {s}");
        let until = self.now() + secs;
        self.messages.push((s, until));
        if self.messages.len() > 8 {
            self.messages.remove(0);
        }
    }
    fn set_map(&mut self, name: &str) {
        let idx = MAP_NAMES.iter().position(|m| m.eq_ignore_ascii_case(name));
        self.map = build_map(name);
        self.map_idx = idx.unwrap_or(0);
        self.msg(format!("map: {}", self.map.name), 3.0);
    }

    // ------------------------------------------------------------ inbound

    fn drain(&mut self) {
        while let Ok((m, from)) = self.inbox.try_recv() {
            self.rx_count += 1;
            match m {
                Ue4In::Vehicles(list) => {
                    let now = self.now();
                    for u in list {
                        let v = self.vehicles.entry(u.copter_id).or_insert_with(|| {
                            println!("rflysim3d: new vehicle #{} (type {})", u.copter_id, u.vehicle_type);
                            Veh::new(u.copter_id, u.vehicle_type)
                        });
                        v.set_type(u.vehicle_type);
                        v.pos = vec3(u.pos[0] as f32, u.pos[1] as f32, u.pos[2] as f32);
                        v.euler = u.euler;
                        v.vel = Vec3::from(u.vel);
                        v.motors = u.motors;
                        v.fit_ground = u.fit_ground;
                        v.time = u.time;
                        v.last_rx = now;
                        if let Some(s) = u.scale {
                            v.scale = Vec3::from(s).max(Vec3::splat(0.01));
                        }
                    }
                    if self.cam.focus == 0 {
                        self.cam.focus = self.vehicles.keys().min().copied().unwrap_or(0);
                    }
                }
                Ue4In::Cmd(c) => self.command(&c, from),
                Ue4In::Vision(req) => {
                    let (seq, w, h, t, port) = (req.seq_id, req.width, req.height, req.type_id, req.dest_port());
                    self.sensors.retain(|s| s.req.seq_id != seq);
                    self.sensors.push(Sensor::new(req));
                    self.msg(format!("vision sensor #{seq}: type {t}, {w}x{h} → UDP {port}"), 3.0);
                }
                Ue4In::Label(l) => self.label(l),
                Ue4In::Attach { copter_ids, attach_ids, types } => {
                    for ((c, a), t) in copter_ids.iter().zip(&attach_ids).zip(&types) {
                        if *c <= 0 {
                            continue;
                        }
                        let parent = self.vehicles.get(a).map(|p| (p.pos, p.q()));
                        if let (Some(v), Some((pp, pq))) = (self.vehicles.get_mut(c), parent) {
                            if *t == 0 {
                                v.attach = None;
                            } else {
                                v.attach = Some((*a, *t, pq.inverse() * (v.pos - pp), pq.inverse() * v.q()));
                            }
                        }
                    }
                }
                Ue4In::ExtAct { .. } | Ue4In::SilIntFloat { .. } => {}
            }
        }
    }

    fn label(&mut self, l: LabelMsg) {
        let color = Color::from_rgba(l.rgb[0].clamp(0, 255) as u8, l.rgb[1].clamp(0, 255) as u8, l.rgb[2].clamp(0, 255) as u8, 255);
        let expires = (l.disp_time > 0.0).then(|| self.now() + l.disp_time as f64);
        let size = if l.font_size > 1.0 { l.font_size } else { 20.0 };
        let ids: Vec<i32> = if l.copter_id <= 0 { self.vehicles.keys().copied().collect() } else { vec![l.copter_id] };
        for id in ids {
            let v = self.vehicles.entry(id).or_insert_with(|| Veh::new(id, 3));
            let lab = Label { text: l.text.clone(), color, size, expires };
            if l.disp_flag < 0 {
                if l.disp_time < 0.0 {
                    v.labels.clear();
                } else {
                    v.labels.push(lab);
                    if v.labels.len() > 5 {
                        v.labels.remove(0);
                    }
                }
            } else {
                let k = l.disp_flag as usize;
                if l.disp_time < 0.0 {
                    if k < v.labels.len() {
                        v.labels.remove(k);
                    }
                } else if k < v.labels.len() {
                    v.labels[k] = lab;
                } else {
                    v.labels.push(lab);
                }
            }
        }
    }

    fn command(&mut self, line: &str, from: SocketAddr) {
        let (name, a) = split_cmd(line);
        let f = |i: usize| a.get(i).and_then(|s| s.parse::<f32>().ok()).unwrap_or(0.0);
        let n = |i: usize| a.get(i).and_then(|s| s.parse::<f32>().ok()).map(|x| x as i32).unwrap_or(0);
        match name.to_ascii_lowercase().as_str() {
            "rflyshowtexttime" => {
                let t = a.last().and_then(|s| s.parse::<f64>().ok());
                let txt = if t.is_some() { a[..a.len() - 1].join(" ") } else { a.join(" ") };
                self.msg(txt, t.unwrap_or(5.0));
            }
            "rflyshowtext" => self.msg(a.join(" "), 5.0),
            "rflychangemapbyname" => {
                let m = a.join(" ");
                self.set_map(if m.is_empty() { "Grasslands" } else { &m });
            }
            "rflychangemapbyid" => {
                let i = n(0).max(0) as usize % MAP_NAMES.len();
                self.set_map(MAP_NAMES[i]);
            }
            "rflychangeviewkeycmd" => {
                let key = a.first().map(|s| s.to_ascii_uppercase()).unwrap_or_default();
                self.key_cmd(&key, n(1));
            }
            "rflycameraposang" | "rflycameraposangadd" => {
                let add = name.to_ascii_lowercase().ends_with("add");
                let p = vec3(f(0), f(1), f(2));
                let (r, pi, y) = (f(3).to_radians(), f(4).to_radians(), f(5).to_radians());
                let _ = r;
                self.cam.view = View::Free;
                if add {
                    self.cam.free_pos += p;
                    self.cam.free_pitch += pi;
                    self.cam.free_yaw += y;
                } else {
                    self.cam.free_pos = p;
                    self.cam.free_pitch = pi;
                    self.cam.free_yaw = y;
                }
            }
            "rflycamerafovdegrees" => self.cam.fov_h_deg = f(0).clamp(5.0, 170.0),
            "rflychange3dmodel" => {
                let id = n(0);
                self.vehicles.entry(id).or_insert_with(|| Veh::new(id, n(1))).set_type(n(1));
            }
            "rflychangevehiclesize" => {
                if let Some(v) = self.vehicles.get_mut(&n(0)) {
                    let s = if f(1) > 0.0 { f(1) } else { 1.0 };
                    v.scale = Vec3::splat(s);
                }
            }
            "rflysetvehicleposang" | "rflymovevehicleposang" => {
                let id = n(0);
                let add = name.to_ascii_lowercase().starts_with("rflymove");
                let now = self.now();
                let v = self.vehicles.entry(id).or_insert_with(|| Veh::new(id, 3));
                let p = vec3(f(2), f(3), f(4));
                let e = [f(5).to_radians(), f(6).to_radians(), f(7).to_radians()];
                v.fit_ground = n(1) != 0;
                v.last_rx = now;
                if add {
                    v.pos += p;
                    for k in 0..3 {
                        v.euler[k] += e[k];
                    }
                } else {
                    v.pos = p;
                    v.euler = e;
                }
            }
            "rflyclearcapture" => {
                self.sensors.clear();
                self.msg("capture cleared", 2.0);
            }
            "rflyreqobjdata" => {
                let op = n(0);
                let obj = a.get(1).cloned().unwrap_or_default();
                let to = SocketAddr::new(from.ip(), UE4_REPLY_PORT);
                self.subs.retain(|s| !(s.op == op && s.name == obj && s.to == to));
                self.subs.push(ObjSub { op, name: obj, to });
            }
            "rflycesiumoripos" => {
                self.gps_origin = [f(0) as f64, f(1) as f64, f(2) as f64];
                let _ = self.tx.send_to(&encode_map_gps(0, self.gps_origin[0], self.gps_origin[1], self.gps_origin[2]), SocketAddr::new(from.ip(), UE4_REPLY_PORT));
            }
            "rflydelvehicles" | "rflyclearvehicles" | "rflydelvehicle" => {
                if a.is_empty() {
                    self.vehicles.clear();
                } else {
                    for s in &a {
                        if let Ok(id) = s.parse::<i32>() {
                            self.vehicles.remove(&id);
                        }
                    }
                }
            }
            "t.maxfps" | "r.setres" | "rflyscanterrainh" => {
                self.msg(format!("'{name}' accepted (no effect on macOS viewer)"), 2.0);
            }
            _ => self.msg(format!("unknown command: {line}"), 3.0),
        }
    }

    /// Emulates RflySim3D's "press key + number" shortcuts.
    fn key_cmd(&mut self, key: &str, num: i32) {
        match key {
            "V" => self.cam.view = VIEWS[num.max(0) as usize % VIEWS.len()],
            "B" => {
                if self.vehicles.contains_key(&num) {
                    self.cam.focus = num;
                }
            }
            "M" => self.set_map(MAP_NAMES[num.max(0) as usize % MAP_NAMES.len()]),
            "P" => self.crash_detect = num != 0 || !self.crash_detect,
            "T" => self.show_trails = !self.show_trails,
            "H" => self.show_hud = !self.show_hud,
            _ => self.msg(format!("view key '{key}' not supported"), 2.0),
        }
    }

    // ------------------------------------------------------------ update

    fn update(&mut self, dt: f32) {
        // Attachments follow their parents.
        let poses: HashMap<i32, (Vec3, Quat)> = self.vehicles.iter().map(|(k, v)| (*k, (v.pos, v.q()))).collect();
        for v in self.vehicles.values_mut() {
            if let Some((parent, ty, off, rq)) = v.attach {
                if let Some((pp, pq)) = poses.get(&parent) {
                    let yaw_only = Quat::from_rotation_z(pq.to_euler(EulerRot::ZYX).0);
                    let (p, q) = match ty {
                        1 => (*pp + off, v.q()),
                        2 => (*pp + yaw_only * off, yaw_only * rq),
                        _ => (*pp + *pq * off, *pq * rq),
                    };
                    v.pos = p;
                    let (y, pi, r) = q.to_euler(EulerRot::ZYX);
                    v.euler = [r, pi, y];
                }
            }
            if v.fit_ground {
                v.pos.z = 0.0;
            }
            let m = v.motors.iter().cloned().fold(0.0f32, f32::max);
            // Senders use RPM (thousands), PWM µs (1000-2000) or 0..1; normalise roughly.
            let spin = if m > 2500.0 { m / 60.0 } else if m > 900.0 { (m - 1000.0).max(0.0) / 10.0 } else if m > 1.5 { m / 60.0 } else { m * 100.0 };
            v.prop_angle = (v.prop_angle + spin.min(60.0) * dt * std::f32::consts::TAU / 4.0) % std::f32::consts::TAU;
            if v.trail.back().map_or(true, |p| p.distance(v.pos) > 0.3) {
                v.trail.push_back(v.pos);
                if v.trail.len() > 2000 {
                    v.trail.pop_front();
                }
            }
            let now = get_time() - self.t0;
            v.labels.retain(|l| l.expires.map_or(true, |e| e > now));
        }
        let now = self.now();
        self.messages.retain(|(_, until)| *until > now);
        if self.crash_detect {
            self.detect_crashes();
        }
        if now >= self.next_sub && !self.subs.is_empty() {
            self.next_sub = now + 0.1;
            self.send_subs();
        }
    }

    fn detect_crashes(&mut self) {
        let now = self.now();
        let list: Vec<(i32, Vec3, f32)> = self.vehicles.values().map(|v| (v.id, v.pos, v.radius())).collect();
        let mut hits = Vec::new();
        for (i, a) in list.iter().enumerate() {
            for b in &list[i + 1..] {
                if a.1.distance(b.1) < (a.2 + b.2) * 0.7 {
                    hits.push((a.0, b.0));
                    hits.push((b.0, a.0));
                }
            }
            for o in &self.map.objects {
                let d = (a.1 - o.centre).abs() - o.half;
                if d.max_element() < 0.0 {
                    hits.push((a.0, -1));
                }
            }
        }
        for (a, b) in hits {
            let last = self.crash_sent.get(&(a, b)).copied().unwrap_or(-10.0);
            if now - last > 1.0 {
                self.crash_sent.insert((a, b), now);
                let pkt = encode_crash(a, b);
                let _ = self.tx.send_to(&pkt, ("127.0.0.1", UE4_REPLY_PORT));
                let _ = self.tx.send_to(&pkt, (std::net::Ipv4Addr::from(UE4_MCAST_GROUP), UE4_REPLY_PORT));
                self.msg(format!("crash: #{a} with {}", if b < 0 { "scene object".to_string() } else { format!("#{b}") }), 2.0);
            }
        }
    }

    fn send_subs(&mut self) {
        let t = self.now();
        for s in &self.subs {
            match s.op {
                1 => {
                    let ids: Vec<i32> = match s.name.parse::<i32>() {
                        Ok(0) | Err(_) => self.vehicles.keys().copied().collect(),
                        Ok(id) => vec![id],
                    };
                    for id in ids {
                        if let Some(v) = self.vehicles.get(&id) {
                            let q = v.q();
                            let h = v.model.half * v.scale;
                            let pkt = encode_copt_req(id, v.pos.into(), v.euler, [q.w, q.x, q.y, q.z], v.pos.into(), h.into(), t);
                            let _ = self.tx.send_to(&pkt, s.to);
                        }
                    }
                }
                0 => {
                    for sen in &self.sensors {
                        if s.name.parse::<u16>().map_or(true, |id| id == sen.req.seq_id) {
                            if let Some(p) = self.sensor_pose(sen) {
                                let q = Quat::from_mat3(&Mat3::from_cols(p.fwd, p.up.cross(p.fwd).normalize() * -1.0, -p.up));
                                let (y, pi, r) = q.to_euler(EulerRot::ZYX);
                                let pkt = encode_camera_data(sen.req.seq_id as i32, sen.req.type_id as i32, sen.req.height as i32, sen.req.width as i32, sen.req.fov_deg, p.pos.into(), [r, pi, y], [q.w, q.x, q.y, q.z], t);
                                let _ = self.tx.send_to(&pkt, s.to);
                            }
                        }
                    }
                }
                2 => {
                    for (k, o) in self.map.objects.iter().enumerate() {
                        if o.name == s.name {
                            let mut w = Wr::default();
                            w.i32(CK_1).i32(k as i32);
                            w.f32s(&o.centre.to_array()).f32s(&[0.0; 3]).f32s(&[1.0, 0.0, 0.0, 0.0]).f32s(&o.centre.to_array()).f32s(&o.half.to_array()).f64(t);
                            w.bytes(o.name.as_bytes(), 32);
                            let _ = self.tx.send_to(&w.0, s.to);
                        }
                    }
                }
                _ => {}
            }
        }
    }

    // ------------------------------------------------------------ camera

    fn input(&mut self, dt: f32) {
        if is_key_pressed(KeyCode::V) {
            let i = VIEWS.iter().position(|v| *v == self.cam.view).unwrap_or(0);
            self.cam.view = VIEWS[(i + 1) % VIEWS.len()];
            if self.cam.view == View::Free {
                let p = self.main_pose();
                self.cam.free_pos = p.pos;
                self.cam.free_yaw = p.fwd.y.atan2(p.fwd.x);
                self.cam.free_pitch = (-p.fwd.z).asin();
            }
            self.msg(format!("view: {:?}", self.cam.view), 1.5);
        }
        let mut ids: Vec<i32> = self.vehicles.keys().copied().collect();
        ids.sort();
        if is_key_pressed(KeyCode::B) && !ids.is_empty() {
            let i = ids.iter().position(|v| *v == self.cam.focus).map_or(0, |i| (i + 1) % ids.len());
            self.cam.focus = ids[i];
        }
        let digits = [KeyCode::Key0, KeyCode::Key1, KeyCode::Key2, KeyCode::Key3, KeyCode::Key4, KeyCode::Key5, KeyCode::Key6, KeyCode::Key7, KeyCode::Key8, KeyCode::Key9];
        for (d, k) in digits.iter().enumerate() {
            if is_key_pressed(*k) && self.vehicles.contains_key(&(d as i32)) {
                self.cam.focus = d as i32;
            }
        }
        if is_key_pressed(KeyCode::M) {
            let next = MAP_NAMES[(self.map_idx + 1) % MAP_NAMES.len()];
            self.set_map(next);
        }
        if is_key_pressed(KeyCode::T) {
            self.show_trails = !self.show_trails;
        }
        if is_key_pressed(KeyCode::L) {
            self.show_labels = !self.show_labels;
        }
        if is_key_pressed(KeyCode::H) {
            self.show_help = !self.show_help;
        }
        if is_key_pressed(KeyCode::F1) {
            self.show_hud = !self.show_hud;
        }
        if is_key_pressed(KeyCode::P) {
            self.crash_detect = !self.crash_detect;
            let on = self.crash_detect;
            self.msg(format!("crash detection {}", if on { "on" } else { "off" }), 2.0);
        }
        let md = mouse_delta_position();
        let dragging = is_mouse_button_down(MouseButton::Left) || is_mouse_button_down(MouseButton::Right);
        let (_, wheel) = mouse_wheel();
        let c = &mut self.cam;
        if c.view == View::Free {
            if dragging {
                c.free_yaw += md.x * 2.0;
                c.free_pitch = (c.free_pitch - md.y * 2.0).clamp(-1.5, 1.5);
            }
            let fwd = vec3(c.free_yaw.cos() * c.free_pitch.cos(), c.free_yaw.sin() * c.free_pitch.cos(), -c.free_pitch.sin());
            let right = vec3(-c.free_yaw.sin(), c.free_yaw.cos(), 0.0);
            let sp = if is_key_down(KeyCode::LeftShift) { 40.0 } else { 8.0 } * dt;
            let mut mv = Vec3::ZERO;
            if is_key_down(KeyCode::W) || is_key_down(KeyCode::Up) { mv += fwd; }
            if is_key_down(KeyCode::S) || is_key_down(KeyCode::Down) { mv -= fwd; }
            if is_key_down(KeyCode::D) || is_key_down(KeyCode::Right) { mv += right; }
            if is_key_down(KeyCode::A) || is_key_down(KeyCode::Left) { mv -= right; }
            if is_key_down(KeyCode::E) { mv.z -= 1.0; }
            if is_key_down(KeyCode::Q) { mv.z += 1.0; }
            c.free_pos += mv * sp + fwd * wheel.signum() * if wheel != 0.0 { 2.0 } else { 0.0 };
        } else {
            if dragging {
                c.az -= md.x * 2.5;
                c.el = (c.el - md.y * 2.0).clamp(-0.2, 1.55);
            }
            if wheel != 0.0 {
                c.dist = (c.dist * if wheel > 0.0 { 0.9 } else { 1.1 }).clamp(0.5, 500.0);
            }
        }
    }

    fn focus(&self) -> Option<&Veh> {
        self.vehicles.get(&self.cam.focus).or_else(|| self.vehicles.values().min_by_key(|v| v.id))
    }

    fn main_pose(&mut self) -> CamPose {
        let aspect = screen_width() / screen_height().max(1.0);
        let fov_h = self.cam.fov_h_deg.to_radians();
        let focus = self.focus().map(|v| (v.pos, v.q(), v.euler[2], v.radius()));
        let (fp, fq, fyaw, frad) = focus.unwrap_or((Vec3::ZERO, Quat::IDENTITY, 0.0, 0.3));
        let c = &self.cam;
        let (pos, target) = match c.view {
            View::Free => {
                let fwd = vec3(c.free_yaw.cos() * c.free_pitch.cos(), c.free_yaw.sin() * c.free_pitch.cos(), -c.free_pitch.sin());
                (c.free_pos, c.free_pos + fwd)
            }
            View::Fpv => {
                let p = fp + fq * vec3(frad * 0.6, 0.0, -0.05);
                (p, p + fq * Vec3::X)
            }
            View::Top => (fp + vec3(-0.01, 0.0, -c.dist * 4.0 - 10.0), fp),
            View::Chase | View::Orbit => {
                let az = if c.view == View::Chase { fyaw + c.az } else { c.az };
                let d = c.dist * (frad / 0.35).max(1.0);
                let off = vec3(-az.cos() * c.el.cos(), -az.sin() * c.el.cos(), -c.el.sin()) * d;
                (fp + off, fp)
            }
        };
        // Smooth chase-cam position a little; snap for other modes.
        let pos = if c.view == View::Chase {
            let sp = self.cam.smooth_pos.map_or(pos, |s| s.lerp(pos, 0.25));
            self.cam.smooth_pos = Some(sp);
            sp
        } else {
            self.cam.smooth_pos = None;
            pos
        };
        let fwd = (target - pos).normalize_or(Vec3::X);
        let up = if fwd.z.abs() > 0.99 { Vec3::X } else { vec3(0., 0., -1.) };
        let up = (up - fwd * up.dot(fwd)).normalize();
        CamPose { pos, fwd, up, fov_h, aspect }
    }

    fn sensor_pose(&self, s: &Sensor) -> Option<CamPose> {
        let r = &s.req;
        let aspect = r.width.max(1) as f32 / r.height.max(1) as f32;
        let mount = if r.use_quat && r.quat.iter().any(|x| *x != 0.0) {
            Quat::from_xyzw(r.quat[1], r.quat[2], r.quat[3], r.quat[0]).normalize()
        } else {
            rot(r.euler)
        };
        let off = Vec3::from(r.pos);
        let (pos, q) = match r.mount_type {
            2 => (off, mount),
            _ => {
                let v = self.vehicles.get(&(r.target_copter as i32))?;
                let bottom = if r.mount_type == 1 { vec3(0., 0., v.model.half.z * v.scale.z) } else { Vec3::ZERO };
                if r.mount_type == 3 {
                    let yaw = Quat::from_rotation_z(v.euler[2]);
                    (v.pos + yaw * off, yaw * mount)
                } else {
                    (v.pos + v.q() * (off + bottom), v.q() * mount)
                }
            }
        };
        Some(CamPose { pos, fwd: q * Vec3::X, up: q * vec3(0., 0., -1.), fov_h: r.fov_deg.clamp(1.0, 170.0).to_radians(), aspect })
    }

    // ------------------------------------------------------------ render

    fn render(&self, p: &CamPose, rt: Option<RenderTarget>, mode: f32) {
        let mut vs: Vec<&Veh> = self.vehicles.values().collect();
        vs.sort_by_key(|v| v.id);
        let draws: Vec<DrawVehicle> = vs
            .iter()
            .map(|v| DrawVehicle {
                model: &v.model,
                kind: model_for_type(v.vtype),
                pos: v.pos,
                q: v.q(),
                scale: v.scale,
                prop_angle: v.prop_angle,
                spinning: v.motors.iter().any(|x| *x > 0.05),
                seg_id: v.id as u32,
            })
            .collect();
        self.r.render(&self.map, &draws, &[], p, rt, mode);
        if mode < 0.5 && self.show_trails {
            for v in &vs {
                let pts: Vec<Vec3> = v.trail.iter().map(|p| v_ned_to_gl(*p)).collect();
                for w in pts.windows(2) {
                    draw_line_3d(w[0], w[1], Color::new(1.0, 0.85, 0.1, 0.9));
                }
            }
        }
    }

    fn capture_sensors(&mut self) {
        let now = self.now();
        // 1) Read back frames rendered last frame.
        for s in &mut self.sensors {
            if let Some(stamp) = s.pending.take() {
                let img = s.rt.texture.get_texture_data();
                let job = Job { req: s.req.clone(), rgba: img.bytes, w: img.width as u32, h: img.height as u32, stamp };
                let _ = self.enc.send(job);
                s.frames += 1;
            }
        }
        // 2) Render the sensors that are due.
        let due: Vec<usize> = (0..self.sensors.len()).filter(|i| now >= self.sensors[*i].next_due).collect();
        for i in due {
            let Some(pose) = self.sensor_pose(&self.sensors[i]) else { continue };
            let mode = match self.sensors[i].req.type_id {
                2 => 1.0,
                4 => 2.0,
                _ => 0.0,
            };
            let rt = self.sensors[i].rt.clone();
            self.render(&pose, Some(rt), mode);
            let stamp = self.vehicles.get(&(self.sensors[i].req.target_copter as i32)).map(|v| v.time).filter(|t| *t > 0.0).unwrap_or(now);
            let s = &mut self.sensors[i];
            s.pending = Some(stamp);
            s.next_due = now + s.period();
        }
    }

    fn hud(&self, p: &CamPose) {
        set_default_camera();
        let (w, h) = (screen_width(), screen_height());
        if self.show_labels {
            let fovy = 2.0 * ((p.fov_h / 2.0).tan() / p.aspect).atan();
            let view = Mat4::look_at_rh(v_ned_to_gl(p.pos), v_ned_to_gl(p.pos + p.fwd), v_ned_to_gl(p.up));
            let vp = Mat4::perspective_rh_gl(fovy, p.aspect, 0.05, 4000.0) * view;
            for v in self.vehicles.values() {
                let top = v.pos - vec3(0., 0., v.radius() + 0.25);
                let c = vp * v_ned_to_gl(top).extend(1.0);
                if c.w <= 0.0 {
                    continue;
                }
                let (sx, sy) = ((c.x / c.w + 1.0) * 0.5 * w, (1.0 - c.y / c.w) * 0.5 * h);
                let mut y = sy;
                let id = format!("#{}", v.id);
                draw_text(&id, sx - 10.0, y, 22.0, if v.id == self.cam.focus { YELLOW } else { WHITE });
                for l in v.labels.iter().rev() {
                    y -= l.size;
                    draw_text(&l.text, sx - 10.0, y, l.size, l.color);
                }
            }
        }
        if !self.show_hud {
            return;
        }
        draw_rectangle(0.0, 0.0, w, 26.0, Color::new(0.0, 0.0, 0.0, 0.45));
        let head = format!(
            "RflySim3D ({OS})  window #{}  UDP {}  |  map {}  |  vehicles {}  sensors {}  |  view {:?}  |  {} fps{}{}",
            self.window,
            UE4_BASE_PORT + self.window,
            self.map.name,
            self.vehicles.len(),
            self.sensors.len(),
            self.cam.view,
            get_fps(),
            if self.crash_detect { "  |  CRASH DETECT" } else { "" },
            self.update.as_ref().map(|v| format!("  |  RflyTools {v} available: rfly-update")).unwrap_or_default()
        );
        draw_text(&head, 8.0, 18.0, 20.0, WHITE);
        if let Some(v) = self.focus() {
            let spd = v.vel.length();
            let lines = [
                format!("Vehicle #{} (type {})", v.id, v.vtype),
                format!("NED  {:8.2} {:8.2} {:8.2} m", v.pos.x, v.pos.y, v.pos.z),
                format!("RPY  {:7.1} {:7.1} {:7.1} deg", v.euler[0].to_degrees(), v.euler[1].to_degrees(), v.euler[2].to_degrees()),
                format!("alt {:6.2} m   speed {:5.2} m/s   t {:.1}s", -v.pos.z, spd, v.time.max(0.0)),
            ];
            draw_rectangle(6.0, 32.0, 330.0, 92.0, Color::new(0.0, 0.0, 0.0, 0.4));
            for (i, l) in lines.iter().enumerate() {
                draw_text(l, 12.0, 52.0 + i as f32 * 20.0, 19.0, WHITE);
            }
        } else {
            draw_text("Waiting for vehicles on UDP 20010+ (CopterSim / Python sendUE4Pos)...", 12.0, 54.0, 22.0, WHITE);
        }
        for (i, (m, _)) in self.messages.iter().enumerate() {
            let tw = measure_text(m, None, 24, 1.0).width;
            draw_text(m, (w - tw) / 2.0, 70.0 + i as f32 * 26.0, 24.0, YELLOW);
        }
        let help = if self.show_help {
            "V view · B/0-9 vehicle · drag orbit · wheel zoom · WASD/QE free-cam · M map · T trails · L labels · P crash · F1 HUD · H hide help"
        } else {
            "H: help"
        };
        draw_text(help, 8.0, h - 10.0, 18.0, Color::new(1.0, 1.0, 1.0, 0.8));
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let a = args();
    let (window, sock) = match net::bind_window(a.window) {
        Ok(x) => x,
        Err(e) => {
            eprintln!("rflysim3d: cannot bind UDP 20010+: {e}");
            std::process::exit(1);
        }
    };
    println!("rflysim3d: window #{window}, listening on UDP {} and multicast 224.0.0.10:{}", UE4_BASE_PORT + window, UE4_MCAST_PORT);
    let (txc, rx) = channel();
    net::spawn(sock, txc);
    let tx = net::sender();
    let enc = spawn_encoder(tx.try_clone().unwrap());
    let mut app = App {
        window,
        map: build_map(&a.map),
        map_idx: MAP_NAMES.iter().position(|m| m.eq_ignore_ascii_case(&a.map)).unwrap_or(0),
        vehicles: HashMap::new(),
        cam: Cam { view: View::Chase, focus: 0, az: 0.0, el: 0.35, dist: 3.5, free_pos: vec3(-10.0, 0.0, -5.0), free_yaw: 0.0, free_pitch: -0.2, fov_h_deg: 90.0, smooth_pos: None },
        sensors: Vec::new(),
        messages: Vec::new(),
        show_trails: !a.low_gpu,
        show_labels: true,
        show_help: true,
        show_hud: true,
        crash_detect: false,
        crash_sent: HashMap::new(),
        subs: Vec::new(),
        next_sub: 0.0,
        tx,
        enc,
        inbox: rx,
        r: Renderer::new(),
        t0: get_time(),
        gps_origin: [0.0; 3],
        rx_count: 0,
        update: None,
    };
    let update_rx = (a.screenshot.is_none() || std::env::var_os("RFLY_UPDATE_URL").is_some()).then(rfly_update::spawn_check);
    // A ground-fixed demo camera pose until a vehicle shows up.
    app.cam.free_pos = vec3(-12.0, -6.0, -6.0);
    loop {
        if is_key_pressed(KeyCode::Escape) {
            break;
        }
        let dt = get_frame_time().min(0.1);
        if let Some(v) = update_rx.as_ref().and_then(|r| r.try_recv().ok()) {
            println!("rflysim3d: RflyTools {v} is available. Update with: rfly-update");
            app.update = Some(v.to_string());
        }
        app.drain();
        app.input(dt);
        app.update(dt);
        app.capture_sensors();
        let pose = app.main_pose();
        app.render(&pose, None, 0.0);
        app.hud(&pose);
        if let Some(path) = &a.screenshot {
            if app.now() >= a.after {
                get_screen_data().export_png(path);
                println!("rflysim3d: screenshot → {path} (rx {} packets)", app.rx_count);
                break;
            }
        }
        rflysim3d::record::tick();
        next_frame().await;
    }
}
