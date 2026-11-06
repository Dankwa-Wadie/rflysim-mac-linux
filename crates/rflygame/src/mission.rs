//! Missions: scored challenges with a time limit.

use crate::game::{OtherDrone, Session};
use crate::objects::{self, Gate, Shape};
use crate::race::Outcome;
use crate::records;
use macroquad::prelude::*;
use rflysim3d::render::CamPose;
use rflysim3d::scene::ModelKind;

pub struct MissionDef {
    pub name: &'static str,
    pub map: &'static str,
    pub desc: &'static str,
    pub limit: f64,
}

pub fn missions() -> Vec<MissionDef> {
    vec![
        MissionDef { name: "Precision Landing", map: "Grasslands", desc: "Fly to the orange pad 35 m away and land dead centre.", limit: 90.0 },
        MissionDef { name: "Rooftop Drop", map: "City", desc: "Land on the pad on top of the tower north-east of you.", limit: 120.0 },
        MissionDef { name: "Package Delivery", map: "Raceway", desc: "Land on the green pad to load a parcel, deliver it to the blue pad.", limit: 150.0 },
        MissionDef { name: "Follow the Car", map: "Raceway", desc: "Stay 5-15 m from the car for 60 s (at least 60% of the time).", limit: 75.0 },
        MissionDef { name: "Windy Hover", map: "Grasslands", desc: "Hold inside the bubble for 20 s while gusts push you around.", limit: 90.0 },
        MissionDef { name: "City Run", map: "City", desc: "Hit all 8 street checkpoints in order before time runs out.", limit: 120.0 },
    ]
}

enum State {
    Land { pad: Vec3, r: f32 },
    Deliver { a: Vec3, b: Vec3, loaded: bool },
    Follow { zone: f64, car_t: f64 },
    Hover { target: Vec3, r: f32, inside: f64 },
    Run { cps: Vec<Gate>, next: usize },
}

pub struct Mission {
    pub def: MissionDef,
    state: State,
    start: Option<f64>,
    airborne: bool,
    shown: i32,
    pub key: String,
}

fn car_path(t: f64) -> (Vec3, f32) {
    let (a, b) = (45.0f64, 28.0f64);
    let per = std::f64::consts::PI * (3.0 * (a + b) - ((3.0 * a + b) * (a + 3.0 * b)).sqrt());
    let th = -std::f64::consts::FRAC_PI_2 + (t * 6.0 / per) * std::f64::consts::TAU;
    let p = vec3((50.0 + a * th.cos()) as f32, (b * th.sin()) as f32, 0.0);
    let yaw = (b * th.cos()).atan2(-a * th.sin()) as f32;
    (p, yaw)
}

impl Mission {
    pub fn new(def: MissionDef, sess: &mut Session) -> Self {
        let state = match def.name {
            "Precision Landing" => State::Land { pad: vec3(30.0, 18.0, 0.0), r: 1.5 },
            "Rooftop Drop" => {
                let top = sess.map.objects.iter().find(|o| o.name == "Building_1_1").map_or(-20.0, |o| o.centre.z - o.half.z);
                State::Land { pad: vec3(40.0, 40.0, top), r: 2.0 }
            }
            "Package Delivery" => State::Deliver { a: vec3(20.0, 20.0, 0.0), b: vec3(85.0, -15.0, 0.0), loaded: false },
            "Follow the Car" => {
                sess.set_spawn(40.0, -36.0, 0.0);
                State::Follow { zone: 0.0, car_t: 0.0 }
            }
            "Windy Hover" => {
                sess.wind = vec3(3.0, 1.5, 0.0);
                sess.gust = 2.5;
                State::Hover { target: vec3(10.0, 0.0, -5.0), r: 1.5, inside: 0.0 }
            }
            _ => {
                let pts = [(20.0, 20.0), (20.0, 100.0), (60.0, 100.0), (100.0, 100.0), (100.0, 60.0), (60.0, 60.0), (60.0, 20.0), (20.0, 20.0)];
                State::Run { cps: pts.iter().map(|(x, y)| Gate::new(*x, *y, 6.0, 0.0, Shape::Sphere(3.5))).collect(), next: 0 }
            }
        };
        let key = format!("Mission: {}", def.name);
        sess.parcel = Some(objects::parcel_mesh());
        let mut m = Mission { def, state, start: None, airborne: false, shown: -1, key };
        m.refresh(sess);
        sess.msg(m.def.desc, 5.0);
        m
    }

    fn refresh(&mut self, sess: &mut Session) {
        let stage = match &self.state {
            State::Deliver { loaded, .. } => *loaded as i32,
            State::Run { next, .. } => *next as i32,
            _ => 0,
        };
        if stage == self.shown {
            return;
        }
        self.shown = stage;
        sess.extras = match &self.state {
            State::Land { pad, r } => vec![(std::rc::Rc::new(objects::pad_mesh(*pad, *r, Color::new(1.0, 0.45, 0.1, 1.0))), Mat4::IDENTITY)],
            State::Deliver { a, b, loaded } => vec![
                (std::rc::Rc::new(objects::pad_mesh(*a, 1.6, if *loaded { objects::DONE } else { Color::new(0.2, 0.8, 0.3, 1.0) })), Mat4::IDENTITY),
                (std::rc::Rc::new(objects::pad_mesh(*b, 1.6, Color::new(0.2, 0.45, 0.95, 1.0))), Mat4::IDENTITY),
            ],
            State::Hover { target, r, .. } => vec![(std::rc::Rc::new(objects::beacon_mesh(*target, *r, objects::NEXT)), Mat4::IDENTITY)],
            State::Run { cps, next } => cps
                .iter()
                .enumerate()
                .skip(*next)
                .take(3)
                .map(|(i, g)| (std::rc::Rc::new(objects::beacon_mesh(g.c, 3.5, if i == *next { objects::NEXT } else { objects::LATER })), Mat4::IDENTITY))
                .collect(),
            State::Follow { .. } => vec![],
        };
    }

    /// Where to go now (for the HUD indicator and the autopilot).
    pub fn target(&self, sess: &Session) -> Option<Vec3> {
        match &self.state {
            State::Land { pad, .. } => Some(*pad),
            State::Deliver { a, b, loaded } => Some(if *loaded { *b } else { *a }),
            State::Follow { .. } => sess.others.first().map(|o| o.pos),
            State::Hover { target, .. } => Some(*target),
            State::Run { cps, next } => cps.get(*next).map(|g| g.c),
        }
    }

    pub fn elapsed(&self, sess: &Session) -> f64 {
        self.start.map_or(0.0, |s| sess.t - s)
    }

    fn done(&self, sess: &Session, player: &str, score: f64, lines: Vec<String>) -> Outcome {
        let rank = records::submit(
            records::Entry { key: self.key.clone(), player: player.into(), value: score.round(), drone: sess.drone.name.clone(), mode: sess.pilot.mode.name().into(), date: records::stamp() },
            false,
        );
        records::save_replay(&self.key, self.def.map, &sess.recording);
        let mut lines = lines;
        lines.push(format!("Score {:.0}   leaderboard rank #{rank}", score));
        Outcome { success: true, title: format!("{}: complete!", self.def.name), lines, board_key: self.key.clone(), lower_is_better: false }
    }

    fn fail(&self, why: &str) -> Outcome {
        Outcome { success: false, title: format!("{}: failed", self.def.name), lines: vec![why.to_string()], board_key: self.key.clone(), lower_is_better: false }
    }

    pub fn update(&mut self, sess: &mut Session, dt: f32, player: &str) -> Option<Outcome> {
        if self.start.is_none() && sess.armed {
            self.start = Some(sess.t);
        }
        if !sess.landed() {
            self.airborne = true;
        }
        if let Some(why) = &sess.crashed {
            return Some(self.fail(&why.clone()));
        }
        let t = self.elapsed(sess);
        if t > self.def.limit {
            return Some(self.fail("Out of time"));
        }
        let p = sess.pos();
        let on_pad = |c: Vec3, r: f32, s: &Session| s.landed() && vec2(p.x - c.x, p.y - c.y).length() < r && (s.ground_at(p) - c.z).abs() < 0.5;
        let out = match &mut self.state {
            State::Land { pad, r } => {
                if self.airborne && on_pad(*pad, *r, sess) {
                    let d = vec2(p.x - pad.x, p.y - pad.y).length() as f64;
                    let score = (1000.0 - d * 400.0 - t * 4.0).max(50.0);
                    Some((score, vec![format!("Landed {:.2} m from centre in {:.1}s", d, t)]))
                } else {
                    None
                }
            }
            State::Deliver { a, b, loaded } => {
                if !*loaded && self.airborne && on_pad(*a, 1.6, sess) {
                    *loaded = true;
                    sess.payload = sess.drone.params.mass * 0.3;
                    self.airborne = false;
                    sess.msg(format!("Parcel loaded (+{:.1} kg). Deliver to the blue pad!", sess.payload), 3.0);
                    None
                } else if *loaded && self.airborne && on_pad(*b, 1.6, sess) {
                    sess.payload = 0.0;
                    Some(((1500.0 - t * 6.0).max(100.0), vec![format!("Delivered in {:.1}s", t)]))
                } else {
                    None
                }
            }
            State::Follow { zone, car_t } => {
                if self.start.is_some() {
                    *car_t += dt as f64;
                }
                let (cp, yaw) = car_path((*car_t - 3.0).max(0.0));
                let d = vec2(p.x - cp.x, p.y - cp.y).length();
                if *car_t > 3.0 && (5.0..=15.0).contains(&d) && -p.z > 1.5 {
                    *zone += dt as f64;
                }
                sess.others.clear();
                sess.others.push(OtherDrone { name: format!("{:.0} m", d), pos: cp, q: Quat::from_rotation_z(yaw), spin: 0.0, color: Color::new(0.9, 0.2, 0.2, 1.0), kind: ModelKind::Car, scale: 1.0 });
                if *car_t >= 63.0 {
                    let pct = *zone / 60.0 * 100.0;
                    if pct >= 60.0 {
                        Some((pct * 10.0, vec![format!("In the zone {:.0}% of the time", pct)]))
                    } else {
                        return Some(self.fail(&format!("Only {:.0}% in the zone (need 60%)", pct)));
                    }
                } else {
                    None
                }
            }
            State::Hover { target, r, inside } => {
                if p.distance(*target) < *r {
                    *inside += dt as f64;
                }
                (*inside >= 20.0).then(|| ((1500.0 - t * 10.0).max(100.0), vec![format!("Held position for 20 s in {:.1}s", t)]))
            }
            State::Run { cps, next } => {
                if cps[*next].crossed(sess.prev_pos, p) {
                    *next += 1;
                    sess.msg(format!("Checkpoint {}/{}", *next, cps.len()), 1.2);
                }
                (*next >= cps.len()).then(|| ((2500.0 - t * 15.0).max(100.0), vec![format!("All checkpoints in {:.1}s", t)]))
            }
        };
        self.refresh(sess);
        out.map(|(score, lines)| self.done(sess, player, score, lines))
    }

    pub fn hud(&self, sess: &Session, cam: &CamPose) {
        let w = crate::ui::vw();
        draw_rectangle(w - 330.0, 40.0, 320.0, 104.0, crate::ui::PANEL);
        let t = self.elapsed(sess);
        draw_text(&format!("{:5.1}s / {:.0}s", t, self.def.limit), w - 316.0, 78.0, 34.0, if self.def.limit - t < 15.0 { ORANGE } else { WHITE });
        let (status, target): (String, Option<Vec3>) = match &self.state {
            State::Land { pad, .. } => ("Land on the pad".into(), Some(*pad)),
            State::Deliver { a, b, loaded } => (if *loaded { "Deliver the parcel".into() } else { "Pick up the parcel".into() }, Some(if *loaded { *b } else { *a })),
            State::Follow { zone, car_t } => (format!("In zone {:.0}s   ({:.0}s left)", zone, (63.0 - car_t).max(0.0)), sess.others.first().map(|o| o.pos)),
            State::Hover { target, inside, .. } => (format!("Hold: {:.1}/20 s", inside), Some(*target)),
            State::Run { cps, next } => (format!("Checkpoint {}/{}", next + 1, cps.len()), cps.get(*next).map(|g| g.c)),
        };
        draw_text(&status, w - 316.0, 108.0, 22.0, crate::ui::ACCENT);
        draw_text(self.def.name, w - 316.0, 132.0, 20.0, crate::ui::DIM);
        if let Some(tg) = target {
            objects::indicator(cam, tg, "target", objects::NEXT);
        }
        if self.start.is_none() {
            crate::ui::centered("Arm to start the clock", crate::ui::vh() / 2.0 + 120.0, 26.0, YELLOW);
        }
    }
}
