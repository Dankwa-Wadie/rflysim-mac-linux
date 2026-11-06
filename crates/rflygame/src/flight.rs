//! Drone presets and the in-game flight controller (stick → motors).
//!
//! The vehicle is the real rfly-core 6-DOF model (motor lag, drag, ground contact).
//! The controller works through the model's own allocation: desired body torques and
//! thrust → per-rotor thrust → motor speed → throttle. Gains therefore scale with each
//! drone's inertia, arm and motor time constant.

use rfly_core::{Params, Quat, Vec3, Vehicle};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlightMode {
    /// Sticks command velocity (heading frame) and climb rate; releases = hover in place.
    Beginner,
    /// Self-levelling: sticks command tilt angle, throttle centre = hover.
    Angle,
    /// Rate mode (FPV racing): sticks command body rates, throttle is collective.
    Acro,
}

impl FlightMode {
    pub const ALL: [FlightMode; 3] = [FlightMode::Beginner, FlightMode::Angle, FlightMode::Acro];
    pub fn name(self) -> &'static str {
        match self {
            FlightMode::Beginner => "Beginner",
            FlightMode::Angle => "Angle",
            FlightMode::Acro => "Acro",
        }
    }
}

#[derive(Clone, Debug)]
pub struct DronePreset {
    pub name: String,
    pub desc: String,
    pub params: Params,
    /// Visual scale relative to the 450 mm model.
    pub scale: f32,
    /// rflysim3d vehicle type (selects the 3D model).
    pub vtype: i32,
    /// Battery: cells (S), capacity (mAh).
    pub cells: f64,
    pub mah: f64,
    /// Propeller diameter (inches), for hover-power estimates.
    pub prop_in: f64,
}

pub const RHO: f64 = 1.225;

impl DronePreset {
    /// Hover power (W) from momentum theory with a 55 % figure of merit × efficiency.
    pub fn hover_power(&self) -> f64 {
        let r = self.prop_in * 0.0254 / 2.0;
        let area = 4.0 * std::f64::consts::PI * r * r;
        let t = self.params.mass * self.params.g;
        t.powf(1.5) / (2.0 * RHO * area).sqrt() / 0.55
    }
    /// Estimated hover flight time (min) using 80 % of the pack.
    pub fn hover_minutes(&self) -> f64 {
        let wh = self.cells * 3.7 * self.mah / 1000.0 * 0.8;
        wh / self.hover_power() * 60.0
    }
    pub fn thrust_to_weight(&self) -> f64 {
        let p = &self.params;
        let w = p.cr + p.wb;
        4.0 * p.ct * w * w / (p.mass * p.g)
    }
    /// Rough top speed (m/s) where horizontal thrust at 45° tilt meets drag.
    pub fn top_speed(&self) -> f64 {
        let p = &self.params;
        let w = p.cr + p.wb;
        let t = 4.0 * p.ct * w * w;
        let horiz = (t * t - (p.mass * p.g).powi(2)).max(0.0).sqrt().min(t * 0.8);
        (horiz / p.cd).sqrt()
    }
}

/// Params from physical quantities (all SI). Inertia and drag scale with size.
pub fn params_from(mass: f64, arm: f64, ct: f64, cm: f64, w_max: f64, tau: f64) -> Params {
    let mut p = Params::default();
    let k = arm / 0.225;
    p.mass = mass;
    p.arm = arm;
    p.ct = ct;
    p.cm = cm;
    p.cr = w_max * 1.1;
    p.wb = -0.1 * w_max;
    p.tm = tau;
    p.jxx = 0.30 * mass * arm * arm;
    p.jyy = 0.31 * mass * arm * arm;
    p.jzz = 0.52 * mass * arm * arm;
    p.jm = 1.287e-4 * k.powi(4);
    p.cd = 0.073 * (mass / 1.4).powf(0.67);
    p.cmd = 0.0055 * k.powi(3);
    p
}

/// Builds a drone from catalog parts (ModelData.db): total mass (kg), wheelbase (mm),
/// motor KV, prop diameter/pitch (in) with RflySim's CT/CM corrections, battery cells/mAh.
#[allow(clippy::too_many_arguments)]
pub fn from_components(
    name: &str,
    mass: f64,
    wheelbase_mm: f64,
    kv: f64,
    prop_d_in: f64,
    prop_p_in: f64,
    ct_corr: f64,
    cm_corr: f64,
    cells: f64,
    mah: f64,
) -> DronePreset {
    let d = prop_d_in * 0.0254;
    let pd = (prop_p_in / prop_d_in.max(0.1)).clamp(0.2, 1.2);
    let c_t = 0.09 * (0.7 + 0.6 * pd) * ct_corr;
    let ct = c_t * RHO * d.powi(4) / (4.0 * std::f64::consts::PI.powi(2));
    let cm = ct * d * 0.063 * (pd / 0.45).sqrt() * cm_corr;
    let w_max = kv * cells * 3.7 * 0.8 * std::f64::consts::TAU / 60.0;
    let tau = 0.012 + 0.038 * (d / 0.254).powi(2);
    let arm = wheelbase_mm / 2000.0;
    DronePreset {
        name: name.to_string(),
        desc: format!("{:.0} mm, {:.2} kg, KV{:.0}, {:.0}x{:.1} in, {:.0}S {:.0} mAh", wheelbase_mm, mass, kv, prop_d_in, prop_p_in, cells, mah),
        params: params_from(mass, arm, ct, cm, w_max, tau),
        scale: (arm / 0.225) as f32,
        vtype: 3,
        cells,
        mah,
        prop_in: prop_d_in,
    }
}

/// Builds parameters from a few physical numbers (inertia and drag scale with size).
fn derived(mass: f64, arm: f64, twr: f64, tau: f64, w_max: f64) -> Params {
    let mut p = Params::default();
    let k = arm / 0.225;
    p.mass = mass;
    p.arm = arm;
    p.ct = twr * mass * p.g / (4.0 * w_max * w_max);
    p.cm = p.ct * 0.0161;
    p.cr = w_max * 1.1;
    p.wb = -0.1 * w_max;
    p.tm = tau;
    p.jxx = 0.30 * mass * arm * arm;
    p.jyy = 0.31 * mass * arm * arm;
    p.jzz = 0.52 * mass * arm * arm;
    p.jm = 1.287e-4 * k.powi(4);
    p.cd = 0.073 * k * k * 0.8;
    p.cmd = 0.0055 * k.powi(3);
    p
}

pub fn presets() -> Vec<DronePreset> {
    vec![
        DronePreset {
            name: "Trainer F450".into(),
            desc: "RflySim's default quad (ModelData.db). Stable, forgiving, 3.3:1 thrust.".into(),
            params: Params::default(),
            scale: 1.0,
            vtype: 3,
            cells: 3.0,
            mah: 5500.0,
            prop_in: 10.0,
        },
        DronePreset {
            name: "Racer 5-inch".into(),
            desc: "FPV racing quad: 650 g, 5.5:1 thrust, very fast motors. Use Acro.".into(),
            params: derived(0.65, 0.11, 5.5, 0.02, 3800.0),
            scale: 0.55,
            vtype: 3,
            cells: 6.0,
            mah: 1300.0,
            prop_in: 5.1,
        },
        DronePreset {
            name: "Heavy Lifter".into(),
            desc: "4 kg camera/delivery drone: 2.2:1 thrust, slow and steady.".into(),
            params: derived(4.0, 0.45, 2.2, 0.08, 700.0),
            scale: 1.9,
            vtype: 3,
            cells: 6.0,
            mah: 16000.0,
            prop_in: 15.0,
        },
    ]
}

/// Stick inputs, all in −1..1 (throttle: −1 bottom, 0 centre, +1 top).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Sticks {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
}

#[derive(Clone, Debug)]
pub struct Rates {
    /// Acro max rate (deg/s) and expo (0..1).
    pub acro_rate: f64,
    pub expo: f64,
    pub max_angle_deg: f64,
    pub max_speed: f64,
    pub max_climb: f64,
}

impl Default for Rates {
    fn default() -> Self {
        Rates { acro_rate: 600.0, expo: 0.4, max_angle_deg: 35.0, max_speed: 10.0, max_climb: 3.0 }
    }
}

pub fn expo(x: f64, e: f64) -> f64 {
    x * x * x * e + x * (1.0 - e)
}

fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    x.max(lo).min(hi)
}

pub struct Pilot {
    pub mode: FlightMode,
    pub rates: Rates,
    i_rate: Vec3,
    i_vz: f64,
    hold_alt: Option<f64>,
}

impl Pilot {
    pub fn new(mode: FlightMode) -> Self {
        Pilot { mode, rates: Rates::default(), i_rate: Vec3::ZERO, i_vz: 0.0, hold_alt: None }
    }

    pub fn reset(&mut self) {
        self.i_rate = Vec3::ZERO;
        self.i_vz = 0.0;
        self.hold_alt = None;
    }

    /// Returns motor throttles (PX4 order) for this physics step.
    pub fn update(&mut self, v: &Vehicle, s: Sticks, armed: bool, dt: f64) -> [f64; 4] {
        if !armed {
            self.reset();
            return [0.0; 4];
        }
        let p = &v.p;
        let st = &v.s;
        let (roll, pitch, yaw) = st.att.to_euler();
        let r = &self.rates;
        let tilt_max = r.max_angle_deg.to_radians();
        let (sr, sp, sy, stt) = (s.roll as f64, s.pitch as f64, s.yaw as f64, s.throttle as f64);

        // ---- outer loops → body rate setpoint + collective thrust (N)
        let hover_t = p.mass * p.g;
        let tiltc = (roll.cos() * pitch.cos()).max(0.5);
        let (rate_sp, thrust) = match self.mode {
            FlightMode::Acro => {
                let max = r.acro_rate.to_radians();
                // Stick forward = nose down (negative pitch rate in FRD), right = roll right, yaw right = nose right.
                let rs = Vec3::new(expo(sr, r.expo) * max, -expo(sp, r.expo) * max, expo(sy, r.expo) * max * 0.8);
                // Collective: throttle stick bottom..top → 0..max thrust (centre ≈ hover).
                let t01 = (stt + 1.0) / 2.0;
                let tmax = self.max_thrust(p);
                let hover01 = hover_t / tmax;
                // Piecewise so that mid-stick is hover for any thrust-to-weight ratio.
                let t = if t01 < 0.5 { t01 / 0.5 * hover01 } else { hover01 + (t01 - 0.5) / 0.5 * (1.0 - hover01) };
                (rs, t * tmax)
            }
            FlightMode::Angle | FlightMode::Beginner => {
                let (roll_sp, pitch_sp, vz_sp);
                if self.mode == FlightMode::Angle {
                    roll_sp = sr * tilt_max;
                    pitch_sp = -sp * tilt_max;
                    vz_sp = -stt * r.max_climb;
                } else {
                    // Velocity in heading frame: pitch stick forward = fly forward.
                    let (c, sn) = (yaw.cos(), yaw.sin());
                    let vf = sp * r.max_speed;
                    let vr = sr * r.max_speed;
                    let v_sp = Vec3::new(c * vf - sn * vr, sn * vf + c * vr, 0.0);
                    let ax = 1.8 * (v_sp.x - st.vel.x);
                    let ay = 1.8 * (v_sp.y - st.vel.y);
                    let axb = c * ax + sn * ay;
                    let ayb = -sn * ax + c * ay;
                    pitch_sp = clamp(-(axb / p.g).atan(), -tilt_max, tilt_max);
                    roll_sp = clamp((ayb / p.g).atan(), -tilt_max, tilt_max);
                    // Altitude hold when the throttle stick is centred.
                    if stt.abs() < 0.05 {
                        let h = *self.hold_alt.get_or_insert(st.pos.z);
                        vz_sp = clamp(1.2 * (h - st.pos.z), -r.max_climb, r.max_climb);
                    } else {
                        self.hold_alt = None;
                        vz_sp = -stt * r.max_climb;
                    }
                }
                // Climb-rate loop (NED: vz < 0 climbs).
                let evz = vz_sp - st.vel.z;
                self.i_vz = clamp(self.i_vz + evz * dt * 1.5, -3.0, 3.0);
                let az = 3.5 * evz + self.i_vz; // desired NED accel correction
                let thrust = p.mass * (p.g - az) / tiltc;
                // Attitude: quaternion error to the desired tilt at the current heading → body
                // rates; the yaw stick turns about the world vertical (coordinated with tilt).
                let k_ang = self.k_rate(p) * 0.35;
                let q_sp = Quat::from_euler(roll_sp, pitch_sp, yaw);
                let qe = st.att.conj().mul(q_sp);
                let sgn = if qe.w < 0.0 { -1.0 } else { 1.0 };
                let err = Vec3::new(qe.x, qe.y, qe.z) * (2.0 * sgn);
                let yaw_body = st.att.rotate_inv(Vec3::new(0.0, 0.0, sy * 150f64.to_radians()));
                let rate_body = err * k_ang + yaw_body;
                (rate_body, thrust)
            }
        };

        // ---- rate loop → body torques
        let j = Vec3::new(p.jxx, p.jyy, p.jzz);
        let kr = self.k_rate(p);
        let e = rate_sp - st.rate;
        self.i_rate = Vec3::new(
            clamp(self.i_rate.x + e.x * dt, -0.5, 0.5),
            clamp(self.i_rate.y + e.y * dt, -0.5, 0.5),
            clamp(self.i_rate.z + e.z * dt, -0.5, 0.5),
        );
        let alpha = e * kr + self.i_rate * (kr * 0.6);
        // Gyroscopic term ω×Jω and rotational drag compensation.
        let w = st.rate;
        let tau = alpha.hadamard(j) + w.cross(w.hadamard(j)) + w * (p.cmd * w.norm());
        self.allocate(p, thrust, tau)
    }

    /// Acro throttle stick position (−1..1) that gives hover thrust (mid-stick by design).
    pub fn acro_hover_stick(&self, _p: &Params) -> f32 {
        0.0
    }

    /// Rate-loop bandwidth (1/s) from the motor time constant.
    fn k_rate(&self, p: &Params) -> f64 {
        (0.28 / p.tm).min(18.0)
    }

    fn max_thrust(&self, p: &Params) -> f64 {
        let w = p.cr + p.wb;
        4.0 * p.ct * w * w
    }

    /// Thrust (N) + body torques → 4 motor throttles via the model's geometry.
    fn allocate(&self, p: &Params, thrust: f64, tau: Vec3) -> [f64; 4] {
        let rotors = p.frame.rotors();
        let k = p.cm / p.ct;
        let mut out = [0.0; 4];
        let tmax = {
            let w = p.cr + p.wb;
            p.ct * w * w
        };
        // Thrust + roll/pitch first; yaw is scaled down to fit (lowest priority, like PX4),
        // so a saturated yaw command never changes total thrust.
        let mut base = [0.0f64; 4];
        let mut yaw = [0.0f64; 4];
        for (i, r) in rotors.iter().enumerate().take(4) {
            let (sa, ca) = r.angle.sin_cos();
            base[i] = thrust / 4.0 - tau.x * sa / (2.0 * p.arm) + tau.y * ca / (2.0 * p.arm);
            yaw[i] = r.dir * tau.z / (4.0 * k);
        }
        let hi = base.iter().cloned().fold(f64::MIN, f64::max);
        let lo = base.iter().cloned().fold(f64::MAX, f64::min);
        let shift = if hi > tmax { tmax - hi } else if lo < 0.0 { -lo } else { 0.0 };
        for b in base.iter_mut() {
            *b += shift;
        }
        let mut sy = 1.0f64;
        for i in 0..4 {
            let y = yaw[i];
            if y > 0.0 {
                sy = sy.min(((tmax - base[i]) / y).max(0.0));
            } else if y < 0.0 {
                sy = sy.min((base[i] / -y).max(0.0));
            }
        }
        for i in 0..4 {
            let t = clamp(base[i] + yaw[i] * sy, 0.0, tmax);
            let w = (t / p.ct).sqrt();
            out[i] = if t <= 1e-6 { 0.0 } else { clamp((w - p.wb) / p.cr, 0.0, 1.0) };
        }
        out
    }
}

/// Euler → quaternion helper for spawning.
pub fn yaw_quat(yaw: f64) -> Quat {
    Quat::from_euler(0.0, 0.0, yaw)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fly(p: Params, mode: FlightMode, s: Sticks, secs: f64) -> Vehicle {
        let mut v = Vehicle::new(p);
        let mut pilot = Pilot::new(mode);
        let dt = 0.001;
        for _ in 0..(secs / dt) as usize {
            let u = pilot.update(&v, s, true, dt);
            v.step(&u, dt);
        }
        v
    }

    #[test]
    fn every_preset_hovers_in_every_assisted_mode() {
        for d in presets() {
            for mode in [FlightMode::Beginner, FlightMode::Angle] {
                // Climb for 2 s, then centre sticks → hold.
                let mut v = Vehicle::new(d.params.clone());
                let mut pilot = Pilot::new(mode);
                for k in 0..6000 {
                    let thr = if k < 2000 { 1.0 } else { 0.0 };
                    let u = pilot.update(&v, Sticks { throttle: thr, ..Default::default() }, true, 0.001);
                    v.step(&u, 0.001);
                }
                let (r, p, _) = v.s.att.to_euler();
                assert!(v.s.pos.z < -2.0, "{} {:?} should climb: z {}", d.name, mode, v.s.pos.z);
                assert!(v.s.vel.z.abs() < 0.3, "{} {:?} should hold: vz {}", d.name, mode, v.s.vel.z);
                assert!(r.abs() < 0.05 && p.abs() < 0.05, "{} {:?} level", d.name, mode);
            }
        }
    }

    #[test]
    fn components_reproduce_rflysim_f450() {
        // DJI F450: 1.5 kg, 450 mm, DJI 2312 KV960, 10x4.5 props, 3S 5500 mAh.
        let d = from_components("F450", 1.5, 450.0, 960.0, 10.0, 4.5, 1.0, 1.0, 3.0, 5500.0);
        let ct_ref = Params::default().ct;
        assert!((d.params.ct / ct_ref - 1.0).abs() < 0.1, "ct {} vs {}", d.params.ct, ct_ref);
        let twr = d.thrust_to_weight();
        assert!((2.0..3.5).contains(&twr), "twr {twr}");
        let mins = d.hover_minutes();
        assert!((8.0..30.0).contains(&mins), "flight time {mins} min");
        // And it flies under the assisted controller.
        let mut v = Vehicle::new(d.params.clone());
        let mut pilot = Pilot::new(FlightMode::Beginner);
        for k in 0..5000 {
            let thr = if k < 2000 { 1.0 } else { 0.0 };
            let u = pilot.update(&v, Sticks { throttle: thr, ..Default::default() }, true, 0.001);
            v.step(&u, 0.001);
        }
        assert!(v.s.pos.z < -2.0 && v.s.vel.z.abs() < 0.3, "z {} vz {}", v.s.pos.z, v.s.vel.z);
    }

    #[test]
    fn beginner_pitch_stick_flies_forward() {
        let mut v = Vehicle::new(Params::default());
        let mut pilot = Pilot::new(FlightMode::Beginner);
        for k in 0..8000 {
            let s = if k < 2000 { Sticks { throttle: 1.0, ..Default::default() } } else { Sticks { pitch: 0.5, ..Default::default() } };
            let u = pilot.update(&v, s, true, 0.001);
            v.step(&u, 0.001);
        }
        assert!(v.s.vel.x > 3.5, "forward speed {}", v.s.vel.x);
        assert!(v.s.vel.y.abs() < 0.5);
    }

    #[test]
    fn full_yaw_does_not_change_altitude() {
        for d in presets() {
            let mut v2 = Vehicle::new(d.params.clone());
            let mut p2 = Pilot::new(FlightMode::Beginner);
            let mut z_hold = 0.0;
            let mut worst: f64 = 0.0;
            for k in 0..9000 {
                let s = if k < 2000 { Sticks { throttle: 1.0, ..Default::default() } } else if k < 4000 { Sticks::default() } else { Sticks { yaw: 1.0, ..Default::default() } };
                let u = p2.update(&v2, s, true, 0.001);
                v2.step(&u, 0.001);
                if k == 4000 {
                    z_hold = v2.s.pos.z;
                }
                if k > 4000 {
                    worst = worst.max((v2.s.pos.z - z_hold).abs());
                }
            }
            assert!(worst < 0.5, "{}: altitude moved {worst:.2} m while yawing", d.name);
            assert!(v2.s.rate.z > 1.0, "{}: should be yawing, r = {}", d.name, v2.s.rate.z);
        }
    }

    /// Flies `secs` from a 5 m hover with the given sticks; returns the vehicle.
    fn from_hover(mode: FlightMode, d: &DronePreset, s: Sticks, secs: f64) -> (Vehicle, f64) {
        let mut v = Vehicle::new(d.params.clone());
        let mut pilot = Pilot::new(FlightMode::Beginner);
        for k in 0..5000 {
            let thr = if k < 2500 { 1.0 } else { 0.0 };
            let u = pilot.update(&v, Sticks { throttle: thr, ..Default::default() }, true, 0.001);
            v.step(&u, 0.001);
        }
        let z0 = v.s.pos.z;
        pilot.mode = mode;
        pilot.reset();
        let base = if mode == FlightMode::Acro { pilot.acro_hover_stick(&v.p) } else { 0.0 };
        let s = Sticks { throttle: s.throttle + base, ..s };
        for _ in 0..(secs * 1000.0) as usize {
            let u = pilot.update(&v, s, true, 0.001);
            v.step(&u, 0.001);
        }
        (v, z0)
    }

    #[test]
    fn sticks_move_the_drone_the_normal_way_in_every_mode() {
        for d in presets() {
            for mode in [FlightMode::Beginner, FlightMode::Angle] {
                let n = &d.name;
                let (v, _) = from_hover(mode, &d, Sticks { pitch: 0.6, ..Default::default() }, 1.5);
                assert!(v.s.vel.x > 1.0, "{n} {mode:?}: pitch forward flies forward (vx {})", v.s.vel.x);
                if mode == FlightMode::Angle {
                    assert!(v.s.att.to_euler().1 < -0.1, "{n}: pitch forward = nose down");
                }
                let (v, _) = from_hover(mode, &d, Sticks { roll: 0.6, ..Default::default() }, 1.5);
                assert!(v.s.vel.y > 1.0, "{n} {mode:?}: roll right flies right (vy {})", v.s.vel.y);
                let (v, z0) = from_hover(mode, &d, Sticks { yaw: 0.6, ..Default::default() }, 1.0);
                let yaw = v.s.att.to_euler().2;
                assert!(yaw > 0.5, "{n} {mode:?}: yaw right turns the nose right (yaw {yaw})");
                assert!((v.s.pos.z - z0).abs() < 0.4, "{n} {mode:?}: yaw keeps altitude");
                // Banked turn: roll + yaw together, altitude must hold.
                let (v, z0) = from_hover(mode, &d, Sticks { roll: 0.5, yaw: 0.6, ..Default::default() }, 2.0);
                assert!((v.s.pos.z - z0).abs() < 0.8, "{n} {mode:?}: coordinated turn keeps altitude ({:.2})", v.s.pos.z - z0);
            }
            // Acro: sticks command body rates.
            let n = &d.name;
            let (v, _) = from_hover(FlightMode::Acro, &d, Sticks { pitch: 0.4, ..Default::default() }, 0.15);
            assert!(v.s.rate.y < -0.5, "{n} acro: pitch forward = nose down (q {})", v.s.rate.y);
            let (v, _) = from_hover(FlightMode::Acro, &d, Sticks { roll: 0.4, ..Default::default() }, 0.15);
            assert!(v.s.rate.x > 0.5, "{n} acro: roll right (p {})", v.s.rate.x);
            let (v, _) = from_hover(FlightMode::Acro, &d, Sticks { yaw: 0.4, ..Default::default() }, 0.3);
            assert!(v.s.rate.z > 0.3, "{n} acro: yaw right (r {})", v.s.rate.z);
        }
    }

    #[test]
    fn acro_roll_stick_rolls_right() {
        let mut v = fly(presets()[1].params.clone(), FlightMode::Acro, Sticks { throttle: 0.3, ..Default::default() }, 1.0);
        let mut pilot = Pilot::new(FlightMode::Acro);
        for _ in 0..100 {
            let u = pilot.update(&v, Sticks { roll: 0.5, throttle: 0.3, ..Default::default() }, true, 0.001);
            v.step(&u, 0.001);
        }
        assert!(v.s.rate.x > 1.0, "roll rate {}", v.s.rate.x);
    }
}
