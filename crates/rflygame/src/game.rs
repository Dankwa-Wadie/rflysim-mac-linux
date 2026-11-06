//! One flight session: physics (1 kHz substeps), terrain + roofs, crashes, wind, payload,
//! battery, cameras, recording, HUD. Game modes (race, missions, school, multiplayer)
//! drive a Session through `Activity`.

use crate::flight::{DronePreset, FlightMode, Pilot, Sticks};
use crate::input::Actions;
use crate::objects::Gate;
use crate::settings::Settings;
use crate::ui;
use macroquad::prelude::*;
use rfly_core::{Quat as DQuat, Vec3 as DVec3, Vehicle};
use rflysim3d::render::*;
use rflysim3d::scene::*;

const DT: f64 = 0.001;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CamMode {
    Chase,
    Fpv,
    Orbit,
    LineOfSight,
}
const CAMS: [CamMode; 4] = [CamMode::Chase, CamMode::Fpv, CamMode::Orbit, CamMode::LineOfSight];

impl CamMode {
    pub fn name(self) -> &'static str {
        match self {
            CamMode::Chase => "Chase",
            CamMode::Fpv => "FPV",
            CamMode::Orbit => "Orbit",
            CamMode::LineOfSight => "Line of sight",
        }
    }
}

/// One recorded sample (replays, ghosts, multiplayer).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub t: f32,
    pub pos: Vec3,
    pub q: Quat,
    pub spin: f32,
}

/// Another drone to draw (ghost, remote player, mission prop).
pub struct OtherDrone {
    pub name: String,
    pub pos: Vec3,
    pub q: Quat,
    pub spin: f32,
    pub color: Color,
    pub kind: ModelKind,
    pub scale: f32,
}

pub struct Session {
    pub map: Map,
    pub drone: DronePreset,
    model: VehicleModel,
    pub v: Vehicle,
    pub pilot: Pilot,
    pub armed: bool,
    /// Motors forced off (race countdown, finished).
    pub locked: bool,
    pub crashed: Option<String>,
    pub cam: CamMode,
    pub t: f64,
    acc: f64,
    prop_angle: f32,
    pub spawn: (DVec3, f64),
    orbit: (f32, f32, f32),
    smooth_cam: Option<Vec3>,
    messages: Vec<(String, f64)>,
    pub flight_time: f64,
    pub max_speed: f64,
    pub last_sticks: Sticks,
    /// Position at the start of the last frame (for gate crossing).
    pub prev_pos: Vec3,
    /// Gates whose frames are solid.
    pub solid_gates: Vec<Gate>,
    /// Mean wind (NED, m/s) and gust amplitude.
    pub wind: Vec3,
    pub gust: f32,
    /// Payload (kg) currently carried.
    pub payload: f64,
    /// Battery state of charge 0..1 (None = unlimited).
    pub soc: Option<f64>,
    pub recording: Vec<Sample>,
    rec_next: f64,
    pub log: Vec<[f32; 20]>,
    log_next: f64,
    pub others: Vec<OtherDrone>,
    pub extras: Vec<(std::rc::Rc<Mesh>, Mat4)>,
    pub parcel: Option<Mesh>,
    others_models: Vec<(ModelKind, [u8; 4], VehicleModel)>,
    /// Physics catch-up limit per frame (1 ms steps).
    pub max_steps: usize,
    prev_contact: Vec<bool>,
    pub touching: bool,
    pub grounded: bool,
    /// Realistic crashes: impacts disarm and the drone falls; no crash screen.
    pub realistic: bool,
    /// Resting upside down (realistic mode).
    pub inverted: bool,
    /// Turtle mode: reversed props flip the drone back over.
    pub turtle: bool,
    /// Last impact message and until when to show it.
    pub impact: Option<(String, f64)>,
    /// Sound events raised this frame (played and cleared by `sound::Audio`).
    pub sfx: Vec<Sfx>,
}

pub fn to_v(v: DVec3) -> Vec3 {
    vec3(v.x as f32, v.y as f32, v.z as f32)
}
pub fn to_q(q: DQuat) -> Quat {
    Quat::from_xyzw(q.x as f32, q.y as f32, q.z as f32, q.w as f32)
}

enum Contact {
    Foot(f64),
    Prop(f64),
    Body(f64),
    Wall(f64, String),
}

/// Ray (origin, unit dir) vs axis-aligned box: distance to entry, if hit ahead.
fn ray_box(o: Vec3, d: Vec3, lo: Vec3, hi: Vec3) -> Option<f32> {
    let inv = Vec3::ONE / d;
    let t1 = (lo - o) * inv;
    let t2 = (hi - o) * inv;
    let tmin = t1.min(t2).max_element();
    let tmax = t1.max(t2).min_element();
    (tmax >= tmin.max(0.0) && tmin > 0.0).then_some(tmin)
}

/// Roofs you can land on: buildings/houses/stands.
fn is_platform(name: &str) -> bool {
    name.starts_with("Building") || name.starts_with("House")
}

impl Session {
    pub fn new(map_name: &str, drone: DronePreset, mode: FlightMode, s: &Settings) -> Self {
        let model = build_model(ModelKind::Multirotor(4), Color::new(0.95, 0.55, 0.15, 1.0));
        let mut pilot = Pilot::new(mode);
        pilot.rates.acro_rate = s.acro_rate;
        pilot.rates.expo = s.expo;
        pilot.rates.max_angle_deg = s.max_angle_deg;
        let mut sess = Session {
            map: build_map(map_name),
            v: Vehicle::new(drone.params.clone()),
            drone,
            model,
            pilot,
            armed: false,
            locked: false,
            crashed: None,
            cam: match s.default_cam {
                1 => CamMode::Fpv,
                2 => CamMode::Chase,
                _ => if mode == FlightMode::Beginner { CamMode::Chase } else { CamMode::Fpv },
            },
            t: 0.0,
            acc: 0.0,
            prop_angle: 0.0,
            spawn: (DVec3::ZERO, 0.0),
            orbit: (0.0, 0.35, 4.0),
            smooth_cam: None,
            messages: Vec::new(),
            flight_time: 0.0,
            max_speed: 0.0,
            last_sticks: Sticks::default(),
            prev_pos: Vec3::ZERO,
            solid_gates: Vec::new(),
            wind: Vec3::ZERO,
            gust: 0.0,
            payload: 0.0,
            soc: if s.battery { Some(1.0) } else { None },
            recording: Vec::new(),
            rec_next: 0.0,
            log: Vec::new(),
            log_next: 0.0,
            others: Vec::new(),
            extras: Vec::new(),
            parcel: None,
            others_models: Vec::new(),
            max_steps: 100,
            prev_contact: Vec::new(),
            touching: true,
            grounded: true,
            realistic: s.crash_style == 0,
            inverted: false,
            turtle: false,
            impact: None,
            sfx: Vec::new(),
        };
        sess.reset();
        sess.msg("Press SPACE / (A) to arm", 4.0);
        sess
    }

    pub fn msg(&mut self, m: impl Into<String>, secs: f64) {
        self.messages.push((m.into(), self.t + secs));
        if self.messages.len() > 4 {
            self.messages.remove(0);
        }
    }

    pub fn set_spawn(&mut self, x: f64, y: f64, yaw_deg: f64) {
        self.spawn = (DVec3::new(x, y, 0.0), yaw_deg.to_radians());
        self.reset();
    }

    pub fn reset(&mut self) {
        self.v = Vehicle::new(self.drone.params.clone());
        self.v.builtin_ground = false;
        self.v.s.pos = self.spawn.0;
        let g = self.surface_at(to_v(self.spawn.0) - vec3(0.0, 0.0, 0.5)) as f64;
        self.v.s.pos.z = g - 0.11 * self.drone.scale as f64 + 0.001;
        self.prev_contact.clear();
        self.grounded = true;
        self.inverted = false;
        self.turtle = false;
        self.impact = None;
        self.v.s.att = crate::flight::yaw_quat(self.spawn.1);
        self.pilot.reset();
        self.armed = false;
        self.crashed = None;
        self.smooth_cam = None;
        self.payload = 0.0;
        self.prev_pos = self.pos();
        if self.soc.is_some() {
            self.soc = Some(1.0);
        }
    }

    pub fn pos(&self) -> Vec3 {
        to_v(self.v.s.pos)
    }
    pub fn quat(&self) -> Quat {
        to_q(self.v.s.att)
    }
    pub fn speed(&self) -> f64 {
        self.v.s.vel.norm()
    }
    pub fn radius(&self) -> f32 {
        (self.model.half * self.drone.scale).max_element() * 0.75
    }
    pub fn landed(&self) -> bool {
        self.grounded
    }
    /// Height of the feet above the surface below (m).
    pub fn agl(&self) -> f32 {
        let p = self.pos();
        self.ground_at(p) - p.z - 0.11 * self.drone.scale
    }
    pub fn spin(&self) -> f32 {
        self.prop_angle
    }

    /// Height (NED z, ≤ 0) of the surface under a point: ground or a roof below it.
    pub fn ground_at(&self, p: Vec3) -> f32 {
        let mut g = 0.0f32;
        for o in &self.map.objects {
            if !is_platform(&o.name) {
                continue;
            }
            let top = o.centre.z - o.half.z;
            if (p.x - o.centre.x).abs() < o.half.x && (p.y - o.centre.y).abs() < o.half.y && p.z <= top + 0.05 && top < g {
                g = top;
            }
        }
        g
    }

    /// Advances the session by a frame.
    pub fn update(&mut self, sticks: Sticks, a: &Actions, dt: f32, input_spring: &mut bool) {
        self.last_sticks = sticks;
        self.prev_pos = self.pos();
        if a.reset {
            self.reset();
            self.msg("Reset", 1.5);
        }
        if a.arm && self.crashed.is_none() && !self.locked && self.inverted && self.realistic {
            self.turtle = !self.turtle;
            self.msg(if self.turtle { "Turtle mode: flipping over..." } else { "Turtle mode off" }, 2.0);
        } else if a.arm && self.crashed.is_none() && !self.locked {
            if self.armed {
                self.armed = false;
                self.msg("Disarmed", 1.5);
            } else if self.pilot.mode == FlightMode::Acro && sticks.throttle > -0.8 {
                self.msg("Throttle down to arm (Acro)", 2.0);
            } else {
                self.armed = true;
                self.msg("ARMED", 1.5);
            }
        }
        if a.mode {
            let i = FlightMode::ALL.iter().position(|m| *m == self.pilot.mode).unwrap_or(0);
            self.pilot.mode = FlightMode::ALL[(i + 1) % FlightMode::ALL.len()];
            self.pilot.reset();
            let m = self.pilot.mode.name();
            self.msg(format!("Flight mode: {m}"), 2.0);
        }
        if a.camera {
            let i = CAMS.iter().position(|c| *c == self.cam).unwrap_or(0);
            self.cam = CAMS[(i + 1) % CAMS.len()];
            self.smooth_cam = None;
            let c = self.cam.name();
            self.msg(format!("Camera: {c}"), 1.5);
        }
        *input_spring = self.pilot.mode != FlightMode::Acro;

        if is_mouse_button_down(MouseButton::Left) {
            let d = mouse_delta_position();
            self.orbit.0 -= d.x * 2.5;
            self.orbit.1 = (self.orbit.1 - d.y * 2.0).clamp(-0.2, 1.5);
        }
        let (_, wheel) = mouse_wheel();
        if wheel != 0.0 {
            self.orbit.2 = (self.orbit.2 * if wheel > 0.0 { 0.9 } else { 1.1 }).clamp(1.0, 60.0);
        }

        self.acc += dt.min(self.max_steps as f32 * 0.001) as f64;
        let mut steps = 0;
        while self.acc >= DT && steps < self.max_steps {
            self.acc -= DT;
            steps += 1;
            self.step_physics(sticks);
        }
        if self.armed && !self.grounded {
            self.flight_time += dt as f64;
        }
        self.max_speed = self.max_speed.max(self.speed());
        let m = self.v.s.motor_w.iter().take(4).cloned().fold(0.0, f64::max);
        self.prop_angle = (self.prop_angle + (m as f32 / 30.0).min(40.0) * dt) % std::f32::consts::TAU;
        let now = self.t;
        self.messages.retain(|(_, u)| *u > now);
    }

    fn step_physics(&mut self, sticks: Sticks) {
        let armed = self.armed && self.crashed.is_none() && !self.locked;
        // Battery sag scales motor speed; payload adds mass.
        let full = &self.drone.params;
        let volt_scale = self.soc.map_or(1.0, |s| (0.83 + 0.17 * s.clamp(0.0, 1.0)) * if s <= 0.0 { 0.6 } else { 1.0 });
        self.v.p.cr = full.cr * volt_scale;
        self.v.p.wb = full.wb * volt_scale;
        self.v.p.mass = full.mass + self.payload;
        let u = self.pilot.update(&self.v, sticks, armed, DT);
        let events = self.apply_contacts();
        self.v.step(&u, DT);
        // Wind: drag on air-relative velocity minus the model's still-air drag.
        if !self.grounded && (self.wind != Vec3::ZERO || self.gust > 0.0) {
            let t = self.t as f32;
            let gust = vec3((t * 0.7).sin() + (t * 1.9).sin() * 0.5, (t * 0.5).cos() + (t * 1.3).sin() * 0.4, (t * 1.1).sin() * 0.2) * self.gust;
            let w = self.wind + gust;
            let v = to_v(self.v.s.vel);
            let rel = w - v;
            let k = (self.v.p.cd / self.v.p.mass) as f32;
            let a = (rel * rel.length() + v * v.length()) * k;
            self.v.s.vel += DVec3::new(a.x as f64, a.y as f64, a.z as f64) * DT;
        }
        // Battery drain from hover-power estimate scaled by thrust.
        if let Some(s) = self.soc.as_mut() {
            if armed {
                let w2: f64 = self.v.s.motor_w.iter().take(4).map(|w| w * w).sum();
                let thrust = self.v.p.ct * w2;
                let hover = self.v.p.mass * self.v.p.g;
                let p_w = self.drone.hover_power() * (thrust / hover).max(0.05).powf(1.5);
                let wh = self.drone.cells * 3.7 * self.drone.mah / 1000.0;
                *s -= p_w * DT / 3600.0 / wh;
            }
        }
        self.t += DT;
        self.record(u);
        if self.crashed.is_some() {
            return;
        }
        for e in events {
            match e {
                Contact::Foot(v) if v > 5.0 => self.crash(format!("Hard landing at {v:.1} m/s")),
                Contact::Prop(v) if armed || v > 1.0 => self.crash("Prop strike".into()),
                Contact::Body(v) if v > 1.5 => self.crash(format!("Hit the ground at {v:.1} m/s")),
                Contact::Wall(v, name) if v > 2.5 => self.crash(format!("Crashed into a {name} at {v:.1} m/s")),
                Contact::Wall(v, _) if v > 0.6 => {
                    self.msg("Bump!", 0.8);
                    self.sfx.push(Sfx::Bump);
                }
                _ => {}
            }
        }
        let up = self.v.s.att.rotate(DVec3::new(0.0, 0.0, -1.0));
        let on_back = self.touching && up.z > -0.3 && self.speed() < 0.8;
        if self.realistic {
            if on_back && !self.inverted && !self.turtle {
                self.armed = false;
            }
            self.inverted = on_back;
            if self.turtle && up.z < -0.6 {
                self.turtle = false;
                self.msg("Back on its feet - arm to fly", 2.5);
            }
        } else if self.touching && up.z > 0.2 && self.speed() < 0.5 && self.crashed.is_none() {
            self.crash("Flipped over".into());
        }
        let p = self.pos();
        if p.x.abs() > 1000.0 || p.y.abs() > 1000.0 || p.z < -600.0 {
            self.crash("Flew out of the map".into());
        }
    }

    /// Contact points in body FRD (m): 4 feet, 4 prop hubs, body top / nose / tail.
    fn contact_points(&self) -> Vec<(DVec3, u8)> {
        let k = self.drone.scale as f64;
        let mut v = Vec::new();
        for a in [45f64, -135.0, -45.0, 135.0] {
            let (s, c) = a.to_radians().sin_cos();
            v.push((DVec3::new(c * 0.225 * k, s * 0.225 * k, 0.11 * k), 0)); // foot
            v.push((DVec3::new(c * 0.225 * k, s * 0.225 * k, -0.045 * k), 1)); // prop hub
        }
        v.push((DVec3::new(0.0, 0.0, -0.06 * k), 2));
        v.push((DVec3::new(0.12 * k, 0.0, 0.0), 2));
        v.push((DVec3::new(-0.12 * k, 0.0, 0.0), 2));
        v
    }

    /// Surface height (NED z) under a point, allowing it to have sunk a little into a roof.
    fn surface_at(&self, p: Vec3) -> f32 {
        let mut g = 0.0f32;
        for o in &self.map.objects {
            if is_platform(&o.name) {
                let top = o.centre.z - o.half.z;
                if (p.x - o.centre.x).abs() < o.half.x && (p.y - o.centre.y).abs() < o.half.y && p.z <= top + 0.35 && top < g {
                    g = top;
                }
            }
        }
        g
    }

    /// Penalty-based contact: spring-damper normal force + Coulomb friction per point;
    /// the body is a sphere against boxes (buildings, trees) and gate bars.
    fn apply_contacts(&mut self) -> Vec<Contact> {
        let s = &self.v.s;
        let m = self.v.p.mass;
        let (kn, cn, mu) = (m * 2500.0, m * 60.0, 0.7);
        let mut f = DVec3::ZERO;
        let mut tq_w = DVec3::ZERO;
        let mut events = Vec::new();
        let pts = self.contact_points();
        if self.prev_contact.len() != pts.len() + 1 {
            self.prev_contact = vec![false; pts.len() + 1];
        }
        let mut feet = 0;
        let mut any = false;
        let fric = |vt: DVec3, fn_: f64| -> DVec3 {
            let n = vt.norm();
            if n < 1e-6 {
                DVec3::ZERO
            } else {
                vt * (-(mu * fn_) * (n / 0.05).min(1.0) / n)
            }
        };
        for (i, (r, kind)) in pts.iter().enumerate() {
            let rw = s.att.rotate(*r);
            let pw = s.pos + rw;
            let vw = s.vel + s.att.rotate(s.rate.cross(*r));
            let g = self.surface_at(to_v(pw)) as f64;
            let pen = pw.z - g;
            let touching = pen > 0.0;
            if touching {
                any = true;
                let fz = (kn * pen + cn * vw.z).max(0.0);
                let fp = DVec3::new(0.0, 0.0, -fz) + fric(DVec3::new(vw.x, vw.y, 0.0), fz);
                f += fp;
                tq_w += rw.cross(fp);
                if *kind == 0 {
                    feet += 1;
                }
                if !self.prev_contact[i] {
                    let v = vw.z.max(0.0);
                    events.push(match kind {
                        0 => Contact::Foot(v),
                        1 => Contact::Prop(v),
                        _ => Contact::Body(v),
                    });
                }
            }
            self.prev_contact[i] = touching;
        }
        // Body sphere vs boxes and gate bars.
        let rb = self.radius() as f64 * 0.85;
        let pos = to_v(s.pos);
        let mut wall: Option<(DVec3, f64, String)> = None;
        for o in &self.map.objects {
            if o.name.starts_with("Ring_") {
                continue;
            }
            let top = o.centre.z - o.half.z;
            if is_platform(&o.name) && pos.z < top - 0.02 {
                continue; // above a roof: the feet handle it
            }
            let q = pos.clamp(o.centre - o.half, o.centre + o.half);
            let d = pos - q;
            let dist = d.length() as f64;
            if dist < rb {
                let n = if dist > 1e-4 { d / dist as f32 } else { vec3(0.0, 0.0, -1.0) };
                wall = Some((DVec3::new(n.x as f64, n.y as f64, n.z as f64), rb - dist, o.name.split('_').next().unwrap_or("object").to_lowercase()));
                break;
            }
        }
        if wall.is_none() {
            for g in &self.solid_gates {
                if let Some((q, br)) = g.closest_on_frame(pos) {
                    let d = pos - q;
                    let dist = d.length() as f64;
                    if dist < rb + br as f64 {
                        let n = d / (dist as f32).max(1e-4);
                        wall = Some((DVec3::new(n.x as f64, n.y as f64, n.z as f64), rb + br as f64 - dist, "gate".into()));
                        break;
                    }
                }
            }
        }
        let wi = pts.len();
        if let Some((n, pen, name)) = wall {
            any = true;
            let vn = s.vel.dot(n);
            let fn_ = (kn * 2.0 * pen - cn * 2.0 * vn).max(0.0);
            let fp = n * fn_ + fric(s.vel - n * vn, fn_);
            f += fp;
            tq_w += (n * -rb).cross(fp);
            if !self.prev_contact[wi] {
                events.push(Contact::Wall((-vn).max(0.0), name));
            }
            self.prev_contact[wi] = true;
        } else {
            self.prev_contact[wi] = false;
        }
        self.v.ext_force = f;
        self.v.ext_torque = s.att.rotate_inv(tq_w);
        if self.turtle {
            // Reversed props on one side: roll torque about the body x axis.
            let k = self.drone.scale as f64;
            self.v.ext_torque.x += 0.9 * m * self.v.p.g * 0.225 * k;
        }
        self.touching = any;
        self.grounded = feet >= 3 && s.vel.norm() < 0.6;
        events
    }

    fn record(&mut self, u: [f64; 4]) {
        if self.t >= self.rec_next {
            self.rec_next = self.t + 1.0 / 30.0;
            self.recording.push(Sample { t: self.t as f32, pos: self.pos(), q: self.quat(), spin: self.prop_angle });
        }
        if self.t >= self.log_next {
            self.log_next = self.t + 0.02;
            let s = &self.v.s;
            let (r, p, y) = s.att.to_euler();
            let st = self.last_sticks;
            self.log.push([
                self.t as f32, s.pos.x as f32, s.pos.y as f32, s.pos.z as f32, s.vel.x as f32, s.vel.y as f32, s.vel.z as f32,
                r as f32, p as f32, y as f32, s.rate.x as f32, s.rate.y as f32, s.rate.z as f32,
                st.roll, st.pitch, st.yaw, st.throttle, u[0] as f32, u[1] as f32, self.soc.unwrap_or(1.0) as f32,
            ]);
        }
    }

    pub fn crash(&mut self, why: String) {
        if !why.starts_with("Flew out") && self.impact.is_none() && self.crashed.is_none() {
            self.sfx.push(Sfx::Impact);
        }
        if self.realistic {
            // Like a flight controller's crash detection: stop the motors, let physics play out.
            if why.starts_with("Flew out") {
                self.reset();
                self.msg("Back to the start", 2.0);
                return;
            }
            self.armed = false;
            self.turtle = false;
            self.impact = Some((why, self.t + 4.0));
            return;
        }
        if self.crashed.is_none() {
            self.armed = false;
            self.crashed = Some(why);
        }
    }

    pub fn camera(&mut self, s: &Settings) -> CamPose {
        let aspect = crate::ui::vw() / crate::ui::vh().max(1.0);
        // Very wide views (split screen) keep at least ~55° vertically, or the uptilted FPV
        // camera would show nothing but sky.
        let fov = s.fov_deg.to_radians().max((2.0 * (0.48f32.tan() * aspect).atan()).min(2.2));
        let p = self.pos();
        let q = self.quat();
        let yaw = self.v.s.att.to_euler().2 as f32;
        match self.cam {
            CamMode::Fpv => {
                let up = Quat::from_rotation_y(s.fpv_uptilt_deg.to_radians());
                CamPose::from_quat(p + q * vec3(0.08 * self.drone.scale, 0.0, -0.03), q * up, fov, aspect)
            }
            CamMode::Chase | CamMode::Orbit => {
                let az = if self.cam == CamMode::Chase { yaw + self.orbit.0 } else { self.orbit.0 };
                let d = self.orbit.2 * self.drone.scale.max(0.6);
                let el = self.orbit.1;
                let target = vec3(-az.cos() * el.cos(), -az.sin() * el.cos(), -el.sin()) * d + p;
                let cp = if self.cam == CamMode::Chase {
                    let sp = self.smooth_cam.map_or(target, |s| s.lerp(target, 0.12));
                    self.smooth_cam = Some(sp);
                    sp
                } else {
                    target
                };
                // Keep the camera out of buildings: pull it in along the drone→camera ray.
                let dir = cp - p;
                let len = dir.length();
                let mut t_hit = len;
                if len > 1e-3 {
                    let dn = dir / len;
                    for o in &self.map.objects {
                        if let Some(t) = ray_box(p, dn, o.centre - o.half, o.centre + o.half) {
                            t_hit = t_hit.min(t);
                        }
                    }
                    // Not below the ground either.
                    if dn.z > 1e-3 {
                        t_hit = t_hit.min((-0.3 - p.z) / dn.z).max(0.4);
                    }
                }
                let cp = if t_hit < len { p + dir / len * (t_hit - 0.3).max(0.4) } else { cp };
                CamPose::look_at(cp, p, fov.min(1.6), aspect)
            }
            CamMode::LineOfSight => {
                let eye = to_v(self.spawn.0) + vec3(-6.0, 3.0, -1.7);
                CamPose::look_at(eye, p, 1.1, aspect)
            }
        }
    }

    fn other_model(&mut self, kind: ModelKind, c: Color) -> usize {
        let key = [(c.r * 255.0) as u8, (c.g * 255.0) as u8, (c.b * 255.0) as u8, (c.a * 255.0) as u8];
        if let Some(i) = self.others_models.iter().position(|(k, cc, _)| *k == kind && *cc == key) {
            return i;
        }
        self.others_models.push((kind, key, build_model(kind, c)));
        self.others_models.len() - 1
    }

    pub fn draw(&mut self, r: &Renderer, s: &Settings) -> CamPose {
        let cam = self.camera(s);
        let idx: Vec<usize> = (0..self.others.len()).map(|i| {
            let (k, c) = (self.others[i].kind, self.others[i].color);
            self.other_model(k, c)
        }).collect();
        let mut list = Vec::new();
        if self.cam != CamMode::Fpv {
            list.push(DrawVehicle {
                model: &self.model,
                kind: ModelKind::Multirotor(4),
                pos: self.pos(),
                q: self.quat(),
                scale: Vec3::splat(self.drone.scale),
                prop_angle: self.prop_angle,
                spinning: self.armed,
                seg_id: 1,
            });
        }
        for (o, i) in self.others.iter().zip(&idx) {
            if self.cam == CamMode::Fpv && o.pos.distance(cam.pos) < 2.0 {
                continue; // a ghost right on top of you would fill the goggles
            }
            list.push(DrawVehicle {
                model: &self.others_models[*i].2,
                kind: o.kind,
                pos: o.pos,
                q: o.q,
                scale: Vec3::splat(o.scale),
                prop_angle: o.spin,
                spinning: true,
                seg_id: 2,
            });
        }
        let mut extras: Vec<DrawExtra> = self.extras.iter().map(|(m, x)| DrawExtra { mesh: &**m, xf: *x }).collect();
        if let Some(pm) = &self.parcel {
            if self.payload > 0.0 {
                let xf = Mat4::from_rotation_translation(self.quat(), self.pos() + vec3(0.0, 0.0, 0.25 * self.drone.scale));
                extras.push(DrawExtra { mesh: pm, xf });
            }
        }
        r.render(&self.map, &list, &extras, &cam, crate::ui::view_rt(), 0.0);
        crate::ui::hud_camera();
        for o in &self.others {
            if o.name.is_empty() {
                continue;
            }
            if let Some(p) = cam.project(o.pos - vec3(0.0, 0.0, 0.6), crate::ui::vw(), crate::ui::vh()) {
                let tw = measure_text(&o.name, None, 20, 1.0).width;
                draw_text(&o.name, p.x - tw / 2.0, p.y, 20.0, o.color);
            }
        }
        cam
    }

    fn recovery_hint(&self) {
        let h = crate::ui::vh();
        if let Some((m, until)) = &self.impact {
            if self.t < *until {
                ui::centered(&format!("Impact: {m} - motors stopped"), h / 2.0 + 120.0, 24.0, ORANGE);
            }
        }
        if self.inverted && !self.turtle {
            ui::centered("Upside down - SPACE / (A) for turtle mode to flip back", h / 2.0 + 150.0, 22.0, YELLOW);
        } else if !self.armed && self.grounded && self.impact.is_some() && self.crashed.is_none() {
            ui::centered("Landed - SPACE / (A) to arm and fly on, R to reset", h / 2.0 + 150.0, 22.0, ui::ACCENT);
        }
    }

    /// Betaflight-style FPV on-screen display.
    fn osd(&self, cam: &CamPose) {
        let (w, h) = (crate::ui::vw(), crate::ui::vh());
        let txt = |t: &str, x: f32, y: f32, size: f32, c: Color| {
            draw_text(t, x + 1.5, y + 1.5, size, Color::new(0.0, 0.0, 0.0, 0.8));
            draw_text(t, x, y, size, c);
        };
        let ctr = |t: &str, y: f32, size: f32, c: Color| {
            let tw = measure_text(t, None, size as u16, 1.0).width;
            txt(t, (w - tw) / 2.0, y, size, c);
        };
        let white = Color::new(1.0, 1.0, 1.0, 0.92);
        // Artificial horizon: project two far points on the true horizon at camera height.
        let yaw = cam.fwd.y.atan2(cam.fwd.x);
        let far = |a: f32| cam.pos + vec3((yaw + a).cos(), (yaw + a).sin(), 0.0) * 2000.0;
        if let (Some(a), Some(b)) = (cam.project(far(-0.5), w, h), cam.project(far(0.5), w, h)) {
            let d = (b - a).normalize_or_zero();
            let c = (a + b) / 2.0;
            let gap = 50.0;
            for side in [-1.0f32, 1.0] {
                let p0 = c + d * gap * side;
                let p1 = c + d * (gap + 140.0) * side;
                draw_line(p0.x + 1.0, p0.y + 1.0, p1.x + 1.0, p1.y + 1.0, 3.0, Color::new(0.0, 0.0, 0.0, 0.6));
                draw_line(p0.x, p0.y, p1.x, p1.y, 2.0, white);
            }
        }
        // Centre mark.
        draw_line(w / 2.0 - 18.0, h / 2.0, w / 2.0 - 6.0, h / 2.0, 2.0, white);
        draw_line(w / 2.0 + 6.0, h / 2.0, w / 2.0 + 18.0, h / 2.0, 2.0, white);
        draw_line(w / 2.0, h / 2.0 - 6.0, w / 2.0, h / 2.0 - 12.0, 2.0, white);
        // Heading tape.
        let hdg = (self.v.s.att.to_euler().2.to_degrees() as f32).rem_euclid(360.0);
        for k in -4..=4 {
            let deg = ((hdg / 15.0).round() as i32 + k) * 15;
            let x = w / 2.0 + (deg as f32 - hdg) * 6.0;
            let lab = match deg.rem_euclid(360) {
                0 => "N".to_string(),
                90 => "E".into(),
                180 => "S".into(),
                270 => "W".into(),
                d if d % 45 == 0 => format!("{d}"),
                _ => "|".into(),
            };
            txt(&lab, x - 6.0, 34.0, 20.0, white);
        }
        txt(&format!("{:03.0}", hdg), w / 2.0 - 16.0, 58.0, 22.0, YELLOW);
        // Left: speed, right: altitude.
        txt(&format!("{:3.0} KM/H", self.speed() * 3.6), 40.0, h / 2.0 + 8.0, 26.0, white);
        txt(&format!("{:5.1} M", self.agl()), w - 150.0, h / 2.0 + 8.0, 26.0, white);
        let climb = -self.v.s.vel.z;
        txt(&format!("{:+4.1} M/S", climb), w - 150.0, h / 2.0 + 34.0, 18.0, white);
        // Bottom: battery voltage, throttle, timer, mode.
        let cells = self.drone.cells as f32;
        let volts = cells * (3.5 + 0.7 * self.soc.unwrap_or(1.0).clamp(0.0, 1.0) as f32);
        let bc = if self.soc.unwrap_or(1.0) < 0.2 { RED } else { white };
        txt(&format!("{:.0}S {:.1}V", cells, volts), 40.0, h - 70.0, 24.0, bc);
        let thr = ((self.last_sticks.throttle + 1.0) * 50.0).clamp(0.0, 100.0);
        txt(&format!("THR {:3.0}%", thr), 40.0, h - 44.0, 22.0, white);
        let m = self.flight_time as i32;
        txt(&format!("{:02}:{:02}", m / 60, m % 60), w - 150.0, h - 190.0, 26.0, white);
        txt(self.pilot.mode.name().to_uppercase().as_str(), w - 150.0, h - 164.0, 22.0, white);
        if self.crashed.is_none() {
            if self.locked {
                ctr("READY", h / 2.0 + 80.0, 30.0, YELLOW);
            } else if !self.armed {
                ctr("DISARMED", h / 2.0 + 80.0, 30.0, ORANGE);
            }
        }
    }

    pub fn hud(&self, s: &Settings, device: &str, cam: &CamPose) {
        crate::ui::hud_camera();
        let (w, h) = (crate::ui::vw(), crate::ui::vh());
        self.recovery_hint();
        if self.cam == CamMode::Fpv {
            self.osd(cam);
            for (i, (m, _)) in self.messages.iter().enumerate() {
                ui::centered(m, h / 2.0 - 90.0 + i as f32 * 30.0, 28.0, YELLOW);
            }
            if s.show_sticks {
                ui::draw_sticks(&self.last_sticks);
            }
            if let Some(why) = &self.crashed {
                draw_rectangle(0.0, h / 2.0 - 60.0, w, 110.0, Color::new(0.5, 0.0, 0.0, 0.55));
                ui::centered("CRASHED", h / 2.0 - 10.0, 56.0, WHITE);
                ui::centered(&format!("{why}  -  press R / (Select) to reset"), h / 2.0 + 30.0, 24.0, WHITE);
            }
            return;
        }
        let st = &self.v.s;
        draw_rectangle(10.0, 10.0, 330.0, 140.0, ui::PANEL);
        let status = if self.crashed.is_some() {
            ("CRASHED", RED)
        } else if self.locked {
            ("READY", YELLOW)
        } else if self.armed {
            ("ARMED", ui::ACCENT)
        } else {
            ("DISARMED", ORANGE)
        };
        draw_text(status.0, 22.0, 40.0, 30.0, status.1);
        draw_text(&format!("{} | {}", self.pilot.mode.name(), self.drone.name), 22.0, 64.0, 22.0, WHITE);
        draw_text(&format!("ALT {:5.1} m   SPD {:5.1} m/s", self.agl(), st.vel.norm()), 22.0, 88.0, 22.0, WHITE);
        draw_text(&format!("Air time {:5.1}s   cam {}", self.flight_time, self.cam.name()), 22.0, 112.0, 20.0, ui::DIM);
        match self.soc {
            Some(soc) => {
                let c = if soc > 0.3 { ui::ACCENT } else if soc > 0.1 { ORANGE } else { RED };
                draw_rectangle(22.0, 124.0, 200.0, 12.0, Color::new(1.0, 1.0, 1.0, 0.15));
                draw_rectangle(22.0, 124.0, 200.0 * soc.clamp(0.0, 1.0) as f32, 12.0, c);
                draw_text(&format!("battery {:.0}%", soc.max(0.0) * 100.0), 230.0, 135.0, 18.0, c);
            }
            None => {
                draw_text("battery: unlimited", 22.0, 136.0, 18.0, ui::DIM);
            }
        }
        if self.wind.length() > 0.1 || self.gust > 0.0 {
            draw_text(&format!("wind {:.0} m/s gusts {:.0}", self.wind.length(), self.gust), 22.0, 166.0, 20.0, YELLOW);
        }
        draw_text(device, w - measure_text(device, None, 18, 1.0).width - 14.0, 24.0, 18.0, ui::DIM);
        for (i, (m, _)) in self.messages.iter().enumerate() {
            ui::centered(m, 60.0 + i as f32 * 30.0, 28.0, YELLOW);
        }
        if s.show_sticks {
            ui::draw_sticks(&self.last_sticks);
        }
        if let Some(why) = &self.crashed {
            draw_rectangle(0.0, h / 2.0 - 60.0, w, 110.0, Color::new(0.5, 0.0, 0.0, 0.55));
            ui::centered("CRASHED", h / 2.0 - 10.0, 56.0, WHITE);
            ui::centered(&format!("{why}  -  press R / (Select) to reset"), h / 2.0 + 30.0, 24.0, WHITE);
        }
        draw_text(
            if s.kb_mode2 {
                "SPACE arm  W/S throttle  A/D yaw  Arrows pitch/roll (Shift = full)  M mode  C camera  R reset  Esc menu"
            } else {
                "SPACE arm  W/S A/D pitch/roll  Arrows throttle/yaw (Shift = full)  M mode  C camera  R reset  Esc menu"
            },
            12.0,
            h - 12.0,
            17.0,
            ui::DIM,
        );
    }

    /// Writes the 50 Hz flight log as CSV; returns the path.
    pub fn save_log(&self, tag: &str) -> Option<std::path::PathBuf> {
        let dir = crate::settings::data_dir().join("logs");
        std::fs::create_dir_all(&dir).ok()?;
        let path = dir.join(format!("{}_{}.csv", crate::records::stamp(), tag.replace(' ', "_")));
        let mut out = String::from("t,x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r,stick_roll,stick_pitch,stick_yaw,stick_thr,motor1,motor2,battery\n");
        for row in &self.log {
            out += &row.iter().map(|v| format!("{v:.4}")).collect::<Vec<_>>().join(",");
            out.push('\n');
        }
        std::fs::write(&path, out).ok()?;
        Some(path)
    }
}

/// Autopilot for demos and automated checks (Beginner-mode sticks command velocity):
/// fly straight at the target (forward + sideways at once), face it, climb first if far
/// below it, and slow down on arrival.
/// Sound events (see `sound.rs`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sfx {
    Impact,
    Bump,
    Beep,
    Go,
    Gate,
    Lap,
    Win,
    Lose,
}

pub fn autopilot(sess: &Session, target: Vec3) -> Sticks {
    let p = sess.pos();
    let d = target - p;
    let yaw = sess.v.s.att.to_euler().2 as f32;
    let (c, s) = (yaw.cos(), yaw.sin());
    let horiz = vec2(d.x, d.y);
    let dist = horiz.length();
    let max_v = sess.pilot.rates.max_speed as f32;
    let mut speed = (dist * 0.45).min(max_v * 0.75);
    if d.z.abs() > 3.0 {
        speed *= 0.15; // get to altitude before cruising (don't hit what's below the target)
    }
    let v = if dist > 1e-3 { horiz / dist * speed } else { Vec2::ZERO };
    let fwd = c * v.x + s * v.y;
    let right = -s * v.x + c * v.y;
    let mut e = d.y.atan2(d.x) - yaw;
    e = e.sin().atan2(e.cos());
    if sess.pilot.mode == FlightMode::Acro {
        // Acro: throttle down to arm, then fly it like a pilot does in rate mode: pick a tilt from the
        // velocity error, and give the stick rate that closes the attitude error.
        if !sess.armed {
            return Sticks { throttle: -1.0, ..Default::default() };
        }
        let st = &sess.v.s;
        let (vx, vy) = (st.vel.x as f32, st.vel.y as f32);
        let (vf, vr) = (c * vx + s * vy, -s * vx + c * vy);
        let lim = 25f32.to_radians();
        let pitch_sp = (-((fwd - vf) * 1.1 / 9.81).atan()).clamp(-lim, lim);
        let roll_sp = (((right - vr) * 1.1 / 9.81).atan()).clamp(-lim, lim);
        let q_sp = DQuat::from_euler(roll_sp as f64, pitch_sp as f64, yaw as f64);
        let qe = st.att.conj().mul(q_sp);
        let sgn = if qe.w < 0.0 { -1.0 } else { 1.0 };
        let mut rate = DVec3::new(qe.x, qe.y, qe.z) * (2.0 * sgn * 4.0);
        if dist > 6.0 {
            rate.z += (e as f64 * 0.8).clamp(-1.0, 1.0);
        }
        let r = &sess.pilot.rates;
        let max = r.acro_rate.to_radians();
        // Invert the expo curve: stick position that gives the wanted rate.
        let inv = |w: f64| {
            let goal = (w / max).clamp(-1.0, 1.0);
            let (mut lo, mut hi) = (-1.0f64, 1.0f64);
            for _ in 0..30 {
                let m = (lo + hi) / 2.0;
                if crate::flight::expo(m, r.expo) < goal { lo = m } else { hi = m }
            }
            ((lo + hi) / 2.0) as f32
        };
        let (rl, pt) = (st.att.to_euler().0 as f32, st.att.to_euler().1 as f32);
        let tiltc = (rl.cos() * pt.cos()).max(0.5);
        let thr = (-d.z * 0.12 + st.vel.z as f32 * 0.3 + (1.0 / tiltc - 1.0) * 0.6).clamp(-0.6, 0.6);
        return Sticks { roll: inv(rate.x), pitch: -inv(rate.y), yaw: inv(rate.z / 0.8), throttle: thr };
    }
    if sess.pilot.mode == FlightMode::Angle {
        // Angle mode: sticks set the tilt, not the speed, so brake on the measured velocity like a pilot would.
        let vel = sess.v.s.vel;
        let (vx, vy) = (vel.x as f32, vel.y as f32);
        let (vf, vr) = (c * vx + s * vy, -s * vx + c * vy);
        let max_tilt = (sess.pilot.rates.max_angle_deg as f32).to_radians();
        let tilt = |a: f32| ((a / 9.81).atan() / max_tilt).clamp(-0.8, 0.8);
        let (af, ar) = ((fwd - vf) * 1.1, (right - vr) * 1.1);
        let yaw_cmd = if dist > 6.0 { (e * 1.2).clamp(-1.0, 1.0) } else { 0.0 };
        return Sticks { pitch: tilt(af), roll: tilt(ar), yaw: yaw_cmd, throttle: (-(d.z) * 0.6).clamp(-1.0, 1.0) };
    }
    let yaw_cmd = if dist > 2.0 { (e * 1.2).clamp(-1.0, 1.0) } else { 0.0 };
    Sticks { pitch: (fwd / max_v).clamp(-1.0, 1.0), roll: (right / max_v).clamp(-1.0, 1.0), yaw: yaw_cmd, throttle: (-(d.z) * 0.6).clamp(-1.0, 1.0) }
}
