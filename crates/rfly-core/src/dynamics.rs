//! 6-DOF rigid-body multicopter dynamics. World frame NED (origin on the ground), body frame FRD.

use crate::math::{Quat, Vec3};
use crate::params::Params;

pub const MAX_ROTORS: usize = 8;

#[derive(Clone, Debug, Default)]
pub struct State {
    /// Position NED (m). z < 0 is above ground.
    pub pos: Vec3,
    /// Velocity NED (m/s).
    pub vel: Vec3,
    /// Body→NED attitude.
    pub att: Quat,
    /// Body angular rate FRD (rad/s).
    pub rate: Vec3,
    /// Rotor speeds (rad/s).
    pub motor_w: [f64; MAX_ROTORS],
    /// Last true acceleration NED (m/s²) — used to synthesise the accelerometer.
    pub acc: Vec3,
    pub on_ground: bool,
}

pub struct Vehicle {
    pub p: Params,
    pub s: State,
    /// Extra force (NED, N) and body torque (FRD, N·m) applied during the next step(s),
    /// e.g. contact forces computed by a game or a terrain model.
    pub ext_force: Vec3,
    pub ext_torque: Vec3,
    /// Built-in flat-ground contact (rest on z = 0). Disable when the caller models contacts.
    pub builtin_ground: bool,
    /// Fixed-wing aircraft: when set, `step` uses its aerodynamics instead of the multicopter model and
    /// takes raw actuator commands (servos −1..1, motor 0..1, PX4 output order).
    pub fw: Option<crate::fixedwing::FixedWing>,
}

impl Vehicle {
    pub fn new(p: Params) -> Self {
        let s = State { on_ground: true, ..Default::default() };
        Self { p, s, ext_force: Vec3::ZERO, ext_torque: Vec3::ZERO, builtin_ground: true, fw: None }
    }

    /// A fixed-wing aircraft resting on the ground at the origin.
    pub fn new_fixed_wing(fw: crate::fixedwing::FixedWing) -> Self {
        let mut p = Params::default();
        p.mass = fw.mass;
        let mut v = Self::new(p);
        v.s.pos.z = -fw.rest_height();
        v.fw = Some(fw);
        v
    }

    /// Advance by `dt` seconds with throttles `u` (0..1, PX4 motor order).
    pub fn step(&mut self, u: &[f64], dt: f64) {
        if let Some(fw) = &self.fw {
            let s = &mut self.s;
            let mut w = s.motor_w[0];
            s.on_ground = crate::fixedwing::step(fw, &mut s.pos, &mut s.vel, &mut s.att, &mut s.rate, &mut w, &mut s.acc, u, self.p.g, dt);
            s.motor_w[0] = w;
            return;
        }
        let p = &self.p;
        let s = &mut self.s;
        let rotors = p.frame.rotors();

        // Motors: first-order lag towards ω_ss = cr·σ + wb.
        let mut thrust = 0.0;
        let mut torque = Vec3::ZERO;
        for (i, r) in rotors.iter().enumerate() {
            let sigma = u.get(i).copied().unwrap_or(0.0).clamp(0.0, 1.0);
            let w_ss = if sigma > 0.0 { (p.cr * sigma + p.wb).max(0.0) } else { 0.0 };
            s.motor_w[i] += (w_ss - s.motor_w[i]) * (dt / p.tm).min(1.0);
            let w2 = s.motor_w[i] * s.motor_w[i];
            let t = p.ct * w2;
            thrust += t;
            let (sa, ca) = r.angle.sin_cos();
            let (rx, ry) = (p.arm * ca, p.arm * sa);
            // r × (0,0,-t)
            torque += Vec3::new(-ry * t, rx * t, r.dir * p.cm * w2);
        }

        // Rotational dynamics: J·ω̇ = τ − ω×(J·ω) − air drag.
        let j = Vec3::new(p.jxx, p.jyy, p.jzz);
        let w = s.rate;
        let tau = torque + self.ext_torque - w.cross(w.hadamard(j)) - w * (p.cmd * w.norm());
        let wdot = Vec3::new(tau.x / j.x, tau.y / j.y, tau.z / j.z);

        // Translational dynamics (NED).
        let f_thrust = s.att.rotate(Vec3::new(0.0, 0.0, -thrust));
        let f_drag = s.vel * (-p.cd * s.vel.norm());
        let mut acc = (f_thrust + f_drag + self.ext_force) * (1.0 / p.mass) + Vec3::new(0.0, 0.0, p.g);
        if !self.builtin_ground {
            s.on_ground = false;
            s.rate += wdot * dt;
            s.att = s.att.integrate(s.rate, dt);
            s.vel += acc * dt;
            s.pos += s.vel * dt;
            s.acc = acc;
            return;
        }

        // Ground contact: rest on z = 0 until thrust lifts the vehicle.
        let touching = s.pos.z >= 0.0;
        if touching && acc.z >= 0.0 {
            s.on_ground = true;
            acc = Vec3::ZERO;
            s.vel = Vec3::ZERO;
            s.pos.z = 0.0;
            s.rate = Vec3::ZERO;
            let (_, _, yaw) = s.att.to_euler();
            s.att = Quat::from_euler(0.0, 0.0, yaw);
        } else {
            s.on_ground = false;
            s.rate += wdot * dt;
            s.att = s.att.integrate(s.rate, dt);
            s.vel += acc * dt;
            s.pos += s.vel * dt;
            if s.pos.z > 0.0 {
                // Hit the ground: stop.
                s.pos.z = 0.0;
                s.vel = Vec3::ZERO;
            }
        }
        s.acc = acc;
    }

    /// Specific force in body frame (what an ideal accelerometer measures).
    pub fn specific_force(&self) -> Vec3 {
        self.s.att.rotate_inv(self.s.acc - Vec3::new(0.0, 0.0, self.p.g))
    }
}
