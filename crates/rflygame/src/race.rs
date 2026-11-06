//! Gate racing: tracks, countdown, laps, splits, respawn, ghosts, results.

use crate::game::{OtherDrone, Sample, Session};
use crate::objects::{self, Gate, Shape};
use crate::records;
use macroquad::prelude::*;
use std::rc::Rc;
use rflysim3d::render::CamPose;
use rflysim3d::scene::ModelKind;

pub struct Track {
    pub name: &'static str,
    pub map: &'static str,
    pub desc: &'static str,
    pub gates: Vec<Gate>,
    pub laps: usize,
    pub spawn: (f32, f32, f32),
}

pub fn tracks() -> Vec<Track> {
    // Raceway: ellipse around (50, 0), radii 45 × 28, 8 gates counter-clockwise.
    let raceway = (0..8)
        .map(|k| {
            let th = (-60.0f32 + 45.0 * k as f32).to_radians();
            let (x, y) = (50.0 + 45.0 * th.cos(), 28.0 * th.sin());
            let yaw = (28.0 * th.cos()).atan2(-45.0 * th.sin()).to_degrees();
            let alt = [3.0, 4.5, 6.0, 4.0, 3.0, 5.5, 7.0, 4.0][k];
            Gate::new(x, y, alt, yaw, Shape::Square(3.0))
        })
        .collect();
    let s = Shape::Square(3.0);
    let city = vec![
        Gate::new(40.0, 20.0, 5.0, 0.0, s),
        Gate::new(80.0, 20.0, 6.0, 0.0, s),
        Gate::new(100.0, 40.0, 5.0, 90.0, s),
        Gate::new(80.0, 60.0, 8.0, 180.0, s),
        Gate::new(40.0, 60.0, 5.0, 180.0, s),
        Gate::new(20.0, 40.0, 4.0, -90.0, s),
    ];
    let rings = (0..6).map(|k| Gate::new(10.0 + k as f32 * 12.0, (k as f32 * 1.3).sin() * 4.0, 2.5, 0.0, Shape::Ring(1.2, false))).collect();
    let small = Shape::Square(2.5);
    let meadow = vec![
        Gate::new(10.0, 0.0, 2.0, 0.0, small),
        Gate::new(16.0, 10.0, 3.0, 90.0, small),
        Gate::new(6.0, 16.0, 2.5, 180.0, small),
        Gate::new(-8.0, 10.0, 3.5, -90.0, small),
        Gate::new(-6.0, -6.0, 2.0, -45.0, small),
    ];
    vec![
        Track { name: "Raceway Grand Prix", map: "Raceway", desc: "8 gates, 3 laps, height changes. Classic FPV loop.", gates: raceway, laps: 3, spawn: (38.0, -26.0, 0.0) },
        Track { name: "City Slalom", map: "City", desc: "Street circuit between the towers. 2 laps.", gates: city, laps: 2, spawn: (0.0, 0.0, 45.0) },
        Track { name: "Ring Run", map: "VisionRingBlank", desc: "Thread RflySim's vision rings. One run, no mistakes.", gates: rings, laps: 1, spawn: (0.0, 0.0, 0.0) },
        Track { name: "Meadow Mini", map: "Grasslands", desc: "Tight beginner course around the pad. 3 laps.", gates: meadow, laps: 3, spawn: (0.0, 0.0, 0.0) },
    ]
}

enum Phase {
    Countdown(f64),
    Running,
    Finished,
}

pub struct Race {
    pub track: Track,
    phase: Phase,
    next: usize,
    start_t: f64,
    lap_start: f64,
    pub laps: Vec<f64>,
    shown_next: usize,
    meshes: Vec<[Rc<Mesh>; 3]>,
    ghost: Vec<Sample>,
    rec_start: usize,
    respawn_at: Option<f64>,
    pub key: String,
}

pub struct Outcome {
    pub success: bool,
    pub title: String,
    pub lines: Vec<String>,
    pub board_key: String,
    pub lower_is_better: bool,
}

impl Race {
    pub fn new(track: Track, sess: &mut Session) -> Self {
        let meshes = track.gates.iter().map(|g| [Rc::new(g.mesh(objects::NEXT)), Rc::new(g.mesh(objects::LATER)), Rc::new(g.mesh(objects::DONE))]).collect();
        sess.set_spawn(track.spawn.0 as f64, track.spawn.1 as f64, track.spawn.2 as f64);
        sess.solid_gates = track.gates.iter().filter(|g| !matches!(g.shape, Shape::Ring(_, false))).cloned().collect();
        // Rings already exist in the VisionRing map; still make them solid for collisions.
        sess.solid_gates.extend(track.gates.iter().filter(|g| matches!(g.shape, Shape::Ring(_, false))).cloned());
        sess.locked = true;
        let key = format!("Race: {}", track.name);
        let ghost = records::load_samples(&records::ghost_path(&key));
        let mut r = Race { track, phase: Phase::Countdown(3.0), next: 0, start_t: 0.0, lap_start: 0.0, laps: vec![], shown_next: usize::MAX, meshes, ghost, rec_start: 0, respawn_at: None, key };
        r.refresh(sess);
        sess.msg("Get ready!", 2.0);
        r
    }

    fn total_gates(&self) -> usize {
        self.track.gates.len() * self.track.laps
    }

    fn refresh(&mut self, sess: &mut Session) {
        if self.shown_next == self.next {
            return;
        }
        self.shown_next = self.next;
        let n = self.track.gates.len();
        let cur = self.next % n;
        sess.extras = self
            .track
            .gates
            .iter()
            .enumerate()
            .map(|(i, _)| {
                let state = if self.next >= self.total_gates() {
                    2
                } else if i == cur {
                    0
                } else if self.next / n == self.track.laps - 1 && i < cur {
                    2
                } else {
                    1
                };
                (self.meshes[i][state].clone(), Mat4::IDENTITY)
            })
            .collect();
    }

    /// Gates passed so far (all laps).
    pub fn progress(&self) -> u16 {
        self.next as u16
    }
    pub fn finished(&self) -> Option<f64> {
        matches!(self.phase, Phase::Finished).then(|| self.laps.iter().sum())
    }
    /// Syncs the countdown (multiplayer start).
    pub fn set_countdown(&mut self, secs: f64) {
        if let Phase::Countdown(l) = &mut self.phase {
            *l = secs.max(0.0);
        }
    }
    /// Steering target for the autopilot: a point on the next gate's axis ahead of the drone.
    pub fn auto_target(&self, p: Vec3) -> Option<Vec3> {
        let g = self.track.gates.get(self.next % self.track.gates.len())?;
        if self.next >= self.total_gates() {
            return None;
        }
        let dir = Quat::from_rotation_z(g.yaw) * Vec3::X;
        let l = g.to_local(p);
        let lateral = vec2(l.y, l.z).length();
        // Past the gate without credit (e.g. flew over it): go back to the approach point.
        let ahead = if l.x > 0.3 {
            -6.0
        } else if l.x < -1.0 && lateral > 0.6 {
            (l.x + 2.5).clamp(-8.0, -3.0)
        } else {
            (l.x + 4.0).min(3.0)
        };
        Some(g.c + dir * ahead)
    }

    pub fn elapsed(&self, sess: &Session) -> f64 {
        match self.phase {
            Phase::Running => sess.t - self.start_t,
            Phase::Finished => self.laps.iter().sum(),
            Phase::Countdown(_) => 0.0,
        }
    }

    pub fn update(&mut self, sess: &mut Session, dt: f32, player: &str) -> Option<Outcome> {
        match &mut self.phase {
            Phase::Countdown(left) => {
                let before = left.ceil();
                *left -= dt as f64;
                if left.ceil() != before && *left > 0.0 {
                    sess.msg(format!("{}", left.ceil()), 0.9);
                    sess.sfx.push(crate::game::Sfx::Beep);
                }
                if *left <= 0.0 {
                    self.phase = Phase::Running;
                    self.start_t = sess.t;
                    self.lap_start = sess.t;
                    sess.locked = false;
                    if sess.pilot.mode != crate::flight::FlightMode::Acro {
                        sess.armed = true;
                    }
                    self.rec_start = sess.recording.len();
                    sess.msg("GO!", 1.2);
                    sess.sfx.push(crate::game::Sfx::Go);
                }
            }
            Phase::Running => {
                // Respawn after a crash at the last passed gate (progress kept).
                if sess.crashed.is_some() {
                    let at = *self.respawn_at.get_or_insert(sess.t + 1.5);
                    if sess.t >= at {
                        self.respawn_at = None;
                        let n = self.track.gates.len();
                        let last = if self.next == 0 { None } else { Some(&self.track.gates[(self.next - 1) % n]) };
                        let (x, y, yaw) = last.map_or((self.track.spawn.0, self.track.spawn.1, self.track.spawn.2.to_radians()), |g| {
                            let back = g.c - Quat::from_rotation_z(g.yaw) * vec3(-2.0, 0.0, 0.0);
                            (back.x, back.y, g.yaw)
                        });
                        let keep = sess.spawn;
                        sess.set_spawn(x as f64, y as f64, yaw.to_degrees() as f64);
                        sess.spawn = keep;
                        if sess.pilot.mode != crate::flight::FlightMode::Acro {
                            sess.armed = true;
                        }
                        sess.msg("Respawned at last gate", 1.5);
                    }
                    return None;
                }
                let n = self.track.gates.len();
                let g = &self.track.gates[self.next % n];
                if g.crossed(sess.prev_pos, sess.pos()) {
                    // R (reset) now brings you back here instead of the start.
                    let back = g.c + Quat::from_rotation_z(g.yaw) * vec3(2.0, 0.0, 0.0);
                    sess.spawn = (rfly_core::Vec3::new(back.x as f64, back.y as f64, 0.0), g.yaw as f64);
                    self.next += 1;
                    sess.sfx.push(if self.next % n == 0 { crate::game::Sfx::Lap } else { crate::game::Sfx::Gate });
                    if self.next % n == 0 {
                        let lap = sess.t - self.lap_start;
                        self.lap_start = sess.t;
                        self.laps.push(lap);
                        let best = self.laps.iter().cloned().fold(f64::MAX, f64::min);
                        sess.msg(format!("Lap {}: {:.2}s{}", self.laps.len(), lap, if lap <= best { "  (best)" } else { "" }), 2.5);
                    }
                    if self.next >= self.total_gates() {
                        self.phase = Phase::Finished;
                        sess.locked = true;
                        return Some(self.finish(sess, player));
                    }
                }
                self.refresh(sess);
            }
            Phase::Finished => {}
        }
        // Ghost of the best run.
        sess.others.retain(|o| o.name != "Best run");
        if let Phase::Running = self.phase {
            if let Some(s) = records::sample_at(&self.ghost, (sess.t - self.start_t) as f32) {
                sess.others.push(OtherDrone { name: "Best run".into(), pos: s.pos, q: s.q, spin: s.spin, color: Color::new(0.6, 0.85, 1.0, 0.55), kind: ModelKind::Multirotor(4), scale: sess.drone.scale });
            }
        }
        None
    }

    fn finish(&mut self, sess: &mut Session, player: &str) -> Outcome {
        let total: f64 = self.laps.iter().sum();
        let best = self.laps.iter().cloned().fold(f64::MAX, f64::min);
        let prev_best = records::top(&self.key, true, 1).first().map(|e| e.value);
        let rank = records::submit(
            records::Entry { key: self.key.clone(), player: player.into(), value: total, drone: sess.drone.name.clone(), mode: sess.pilot.mode.name().into(), date: records::stamp() },
            true,
        );
        // Recording relative to race start → ghost and replay.
        let t0 = self.start_t as f32;
        let run: Vec<Sample> = sess.recording[self.rec_start..].iter().map(|s| Sample { t: s.t - t0, ..*s }).collect();
        let new_best = prev_best.is_none_or(|b| total < b);
        if new_best {
            records::save_samples(&records::ghost_path(&self.key), &run);
        }
        records::save_replay(&self.key, self.track.map, &run);
        let mut lines = vec![format!("Total {:.2}s   best lap {:.2}s", total, best)];
        for (i, l) in self.laps.iter().enumerate() {
            lines.push(format!("Lap {}: {:.2}s", i + 1, l));
        }
        lines.push(if new_best { "NEW TRACK RECORD! Ghost saved.".into() } else { format!("Leaderboard rank #{rank}") });
        Outcome { success: true, title: format!("{} finished", self.track.name), lines, board_key: self.key.clone(), lower_is_better: true }
    }

    pub fn hud(&self, sess: &Session, cam: &CamPose) {
        let w = crate::ui::vw();
        let n = self.track.gates.len();
        draw_rectangle(w - 300.0, 40.0, 290.0, 104.0, crate::ui::PANEL);
        let t = self.elapsed(sess);
        draw_text(&format!("{:6.2}s", t), w - 286.0, 80.0, 40.0, WHITE);
        let lap = (self.next / n + 1).min(self.track.laps);
        draw_text(&format!("Lap {}/{}   Gate {}/{}", lap, self.track.laps, self.next % n + 1, n), w - 286.0, 108.0, 22.0, crate::ui::ACCENT);
        if let Some(b) = self.laps.iter().cloned().reduce(f64::min) {
            draw_text(&format!("best lap {:.2}s", b), w - 286.0, 132.0, 20.0, crate::ui::DIM);
        }
        if let Phase::Countdown(l) = self.phase {
            crate::ui::centered(&format!("{}", l.ceil().max(1.0)), crate::ui::vh() / 2.0, 120.0, YELLOW);
        }
        if self.next < self.total_gates() {
            objects::indicator(cam, self.track.gates[self.next % n].c, "gate", objects::NEXT);
        }
    }
}
