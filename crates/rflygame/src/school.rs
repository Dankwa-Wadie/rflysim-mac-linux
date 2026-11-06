//! Flight School: step-by-step lessons with live checks and feedback.

use crate::flight::FlightMode;
use crate::game::Session;
use crate::objects;
use crate::race::Outcome;
use crate::records;
use macroquad::prelude::*;
use rflysim3d::render::CamPose;

#[derive(Clone, Copy, Debug)]
pub enum Check {
    Armed,
    /// Altitude band (m above ground), held for `hold` seconds.
    Alt(f32, f32),
    /// Stay within r of a point (NED) for `hold` seconds.
    Near(Vec3, f32),
    /// Accumulate this much yaw rotation (rad) while airborne.
    Yaw(f32),
    /// Reach a point (NED) within r.
    Visit(Vec3, f32),
    /// Touch down gently (< 1.5 m/s) after flying.
    Land,
    /// Switch to this flight mode (M / X button).
    Mode(FlightMode),
}

pub struct Step {
    pub text: &'static str,
    pub check: Check,
    pub hold: f32,
}

pub struct Lesson {
    pub name: &'static str,
    pub desc: &'static str,
    pub mode: FlightMode,
    pub steps: Vec<Step>,
}

fn st(text: &'static str, check: Check, hold: f32) -> Step {
    Step { text, check, hold }
}

pub fn lessons() -> Vec<Lesson> {
    use Check::*;
    let up = |x: f32, y: f32, alt: f32| vec3(x, y, -alt);
    vec![
        Lesson {
            name: "1. Take off and land",
            desc: "Arm, lift off, hold a low hover, land softly.",
            mode: FlightMode::Beginner,
            steps: vec![
                st("Arm the motors: SPACE (or A on a gamepad)", Armed, 0.0),
                st("Push throttle up (Up arrow / left stick up) to climb to 2-4 m, then let go", Alt(2.0, 4.0), 3.0),
                st("Lower the throttle and land gently", Land, 0.0),
            ],
        },
        Lesson {
            name: "2. Hover on the spot",
            desc: "Keep the drone inside the green bubble.",
            mode: FlightMode::Beginner,
            steps: vec![st("Arm and climb to the bubble at 3 m", Visit(up(0.0, 0.0, 3.0), 1.5), 0.0), st("Stay inside the bubble for 10 seconds", Near(up(0.0, 0.0, 3.0), 1.5), 10.0), st("Land", Land, 0.0)],
        },
        Lesson {
            name: "3. Yaw around",
            desc: "Turn on the spot with the yaw control.",
            mode: FlightMode::Beginner,
            steps: vec![st("Arm and climb to 3 m", Alt(2.0, 6.0), 1.0), st("Turn a full circle with yaw (Left/Right arrows, left stick sideways)", Yaw(6.28), 0.0), st("Land", Land, 0.0)],
        },
        Lesson {
            name: "4. Forward and back",
            desc: "Fly to the marker 20 m ahead and return.",
            mode: FlightMode::Beginner,
            steps: vec![
                st("Climb to 3 m", Alt(2.0, 6.0), 1.0),
                st("Fly forward (W / right stick up) to the marker", Visit(up(20.0, 0.0, 3.0), 2.5), 0.0),
                st("Fly back (S / right stick down) to the pad", Visit(up(0.0, 0.0, 3.0), 2.5), 0.0),
                st("Land on the pad", Land, 0.0),
            ],
        },
        Lesson {
            name: "5. Square pattern",
            desc: "Visit four corners of a 15 m square.",
            mode: FlightMode::Beginner,
            steps: vec![
                st("Corner 1 (15 m north)", Visit(up(15.0, 0.0, 3.0), 2.5), 0.0),
                st("Corner 2 (north-east)", Visit(up(15.0, 15.0, 3.0), 2.5), 0.0),
                st("Corner 3 (east)", Visit(up(0.0, 15.0, 3.0), 2.5), 0.0),
                st("Back to the start", Visit(up(0.0, 0.0, 3.0), 2.5), 0.0),
                st("Land", Land, 0.0),
            ],
        },
        Lesson {
            name: "6. Angle mode",
            desc: "No position hold: the drone drifts unless you correct it.",
            mode: FlightMode::Beginner,
            steps: vec![
                st("Switch to Angle mode (M key / X button)", Mode(FlightMode::Angle), 0.0),
                st("Arm and climb to 3-6 m, hold it there", Alt(3.0, 6.0), 3.0),
                st("Fly to the marker 20 m north", Visit(up(20.0, 0.0, 4.0), 3.0), 0.0),
                st("Come back to the pad", Visit(up(0.0, 0.0, 4.0), 3.0), 0.0),
                st("Land gently", Land, 0.0),
            ],
        },
        Lesson {
            name: "7. Acro basics",
            desc: "Rate mode: the drone keeps its angle until you level it.",
            mode: FlightMode::Angle,
            steps: vec![
                st("Switch to Acro mode (M / X). Throttle fully down before arming", Mode(FlightMode::Acro), 0.0),
                st("Arm, climb and hold 3-10 m for 5 s (throttle mid = hover)", Alt(3.0, 10.0), 5.0),
                st("Fly to the marker 25 m north", Visit(up(25.0, 0.0, 5.0), 4.0), 0.0),
                st("Return and land on the pad", Land, 0.0),
            ],
        },
    ]
}

pub struct School {
    pub idx: usize,
    pub lesson: Lesson,
    step: usize,
    held: f32,
    yaw_acc: f32,
    last_yaw: Option<f32>,
    airborne: bool,
    shown: usize,
    start: f64,
}

impl School {
    pub fn new(idx: usize, sess: &mut Session) -> Self {
        let lesson = lessons().remove(idx);
        sess.pilot.mode = lesson.mode;
        let mut s = School { idx, lesson, step: 0, held: 0.0, yaw_acc: 0.0, last_yaw: None, airborne: false, shown: usize::MAX, start: sess.t };
        s.refresh(sess);
        s
    }

    /// The check the pilot is working on now.
    pub fn current(&self) -> Option<Check> {
        self.lesson.steps.get(self.step).map(|s| s.check)
    }

    fn target(&self) -> Option<(Vec3, f32)> {
        match self.lesson.steps.get(self.step)?.check {
            Check::Near(p, r) | Check::Visit(p, r) => Some((p, r)),
            _ => None,
        }
    }

    fn refresh(&mut self, sess: &mut Session) {
        if self.shown == self.step {
            return;
        }
        self.shown = self.step;
        sess.extras = self.target().map(|(p, r)| vec![(std::rc::Rc::new(objects::beacon_mesh(p, r, objects::NEXT)), Mat4::IDENTITY)]).unwrap_or_default();
        if let Some(s) = self.lesson.steps.get(self.step) {
            sess.msg(s.text, 4.0);
        }
    }

    pub fn update(&mut self, sess: &mut Session, dt: f32) -> Option<Outcome> {
        if sess.crashed.is_some() {
            return Some(Outcome { success: false, title: format!("{}: crashed", self.lesson.name), lines: vec![sess.crashed.clone().unwrap_or_default(), "Try again - slower is smoother.".into()], board_key: String::new(), lower_is_better: false });
        }
        let Some(step) = self.lesson.steps.get(self.step) else { return None };
        let p = sess.pos();
        let agl = sess.ground_at(p) - p.z;
        if !sess.landed() {
            self.airborne = true;
        }
        let yaw = sess.v.s.att.to_euler().2 as f32;
        if let Some(ly) = self.last_yaw {
            let mut d = yaw - ly;
            d = d.sin().atan2(d.cos());
            if !sess.landed() {
                self.yaw_acc += d.abs();
            }
        }
        self.last_yaw = Some(yaw);
        let ok_now = match step.check {
            Check::Armed => sess.armed,
            Check::Alt(lo, hi) => agl >= lo && agl <= hi,
            Check::Near(c, r) | Check::Visit(c, r) => p.distance(c) < r,
            Check::Yaw(total) => self.yaw_acc >= total,
            Check::Land => self.airborne && sess.landed(),
            Check::Mode(m) => sess.pilot.mode == m,
        };
        if ok_now {
            self.held += dt;
        } else {
            self.held = 0.0;
        }
        if ok_now && self.held >= step.hold {
            self.step += 1;
            self.held = 0.0;
            self.yaw_acc = 0.0;
            if matches!(step.check, Check::Land) {
                self.airborne = false;
            }
            if self.step >= self.lesson.steps.len() {
                records::mark_school_done(self.idx);
                let t = sess.t - self.start;
                return Some(Outcome {
                    success: true,
                    title: format!("{}: passed!", self.lesson.name),
                    lines: vec![format!("Completed in {:.0} s", t), if self.idx + 1 < lessons().len() { "Next lesson unlocked.".into() } else { "You graduated from Flight School!".into() }],
                    board_key: String::new(),
                    lower_is_better: false,
                });
            }
            sess.msg("Good!", 1.0);
        }
        self.refresh(sess);
        None
    }

    pub fn hud(&self, sess: &Session, cam: &CamPose) {
        let w = crate::ui::vw();
        let n = self.lesson.steps.len();
        let pw = 620.0f32.min(w - 40.0);
        let x = (w - pw) / 2.0;
        draw_rectangle(x, crate::ui::vh() - 150.0, pw, 96.0, crate::ui::PANEL);
        draw_text(&format!("{}  -  step {}/{}", self.lesson.name, (self.step + 1).min(n), n), x + 16.0, crate::ui::vh() - 120.0, 22.0, crate::ui::ACCENT);
        if let Some(s) = self.lesson.steps.get(self.step) {
            draw_text(s.text, x + 16.0, crate::ui::vh() - 92.0, 22.0, WHITE);
            let prog = match s.check {
                Check::Yaw(t) => self.yaw_acc / t,
                _ if s.hold > 0.0 => self.held / s.hold,
                _ => 0.0,
            };
            if prog > 0.0 {
                draw_rectangle(x + 16.0, crate::ui::vh() - 76.0, (pw - 32.0) * prog.min(1.0), 8.0, crate::ui::ACCENT);
            }
        }
        if let Some((p, _)) = self.target() {
            objects::indicator(cam, p, "target", objects::NEXT);
        }
        let _ = sess;
    }
}
