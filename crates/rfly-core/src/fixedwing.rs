//! Fixed-wing dynamics (RflySim's SITLRunFw / HITLRunFw): lifting surfaces with control deflections, a puller
//! propeller and runway contact. The default aircraft is PX4's Gazebo-Classic "plane" (PX4 airframe 1030,
//! `gazebo-classic_plane`), with its mass, inertia and lift-drag parameters, so PX4's tuning for that airframe
//! applies. Each surface works like Gazebo's LiftDragPlugin: linear lift/drag up to stall, a post-stall slope,
//! plus `deflection × rad_to_cl` from its control surface, applied at its centre of pressure. Body frame FRD.

use crate::math::{Quat, Vec3};

/// One lifting surface (wing half, tailplane, fin).
#[derive(Clone, Debug)]
pub struct Surface {
    /// Centre of pressure (m, FRD from the centre of gravity).
    pub cp: Vec3,
    /// Chord direction and lift-side normal (unit, FRD).
    pub forward: Vec3,
    pub upward: Vec3,
    pub area: f64,
    /// Zero-lift angle offset (rad), lift and drag slopes (per rad), stall angle and post-stall slopes.
    pub a0: f64,
    pub cla: f64,
    pub cda: f64,
    pub alpha_stall: f64,
    pub cla_stall: f64,
    pub cda_stall: f64,
    /// Control input: (actuator channel, lift coefficient per radian of deflection).
    pub control: Option<(usize, f64)>,
}

#[derive(Clone, Debug)]
pub struct FixedWing {
    pub mass: f64,
    /// Principal inertia (kg·m²).
    pub inertia: Vec3,
    pub surfaces: Vec<Surface>,
    /// Control-surface travel limit (rad). As in Gazebo's plane, a servo command is the joint angle in radians
    /// (input scaling 1), clamped to this limit.
    pub max_deflection: f64,
    /// Motor: throttle channel, thrust = kt·ω², ω_max, time constants up/down (s), airspeed at which thrust
    /// has dropped to zero (m/s).
    pub motor_channel: usize,
    pub kt: f64,
    pub w_max: f64,
    pub tau_up: f64,
    pub tau_down: f64,
    pub v_zero_thrust: f64,
    /// Ground contact points (FRD), e.g. the belly box corners.
    pub contacts: Vec<Vec3>,
    pub air_density: f64,
}

/// Gazebo FLU (x fwd, y left, z up) → FRD.
fn flu(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, -y, -z)
}

impl FixedWing {
    /// PX4's Gazebo-Classic plane (airframe 1030). Actuator channels follow its PWM_MAIN_FUNC mapping:
    /// 2 rudder, 4 motor, 5 left aileron, 6 right aileron, 7 elevator (3 and 8 are flaps, not modelled).
    /// PX4 parameters that make a fixed-wing PX4 drive this model: the control allocation and output
    /// functions of PX4's `gazebo-classic_plane` airframe (1030), for SITL (`PWM_MAIN_*`) and for a Pixhawk
    /// in HIL (`HIL_ACT_*`). Output n is HIL_ACTUATOR_CONTROLS index n-1: 3 rudder, 4 left flap, 5 motor,
    /// 6/7 left/right aileron, 8 elevator, 9 right flap (reversed).
    pub fn px4_params() -> Vec<(String, f64)> {
        let mut v: Vec<(String, f64)> = [
            ("CA_AIRFRAME", 1.0), ("CA_ROTOR_COUNT", 1.0), ("CA_ROTOR0_PX", 0.3), ("CA_SV_CS_COUNT", 6.0),
            ("CA_SV_CS0_TYPE", 1.0), ("CA_SV_CS0_TRQ_R", -0.5), ("CA_SV_CS1_TYPE", 2.0), ("CA_SV_CS1_TRQ_R", 0.5),
            ("CA_SV_CS2_TYPE", 3.0), ("CA_SV_CS2_TRQ_P", 1.0), ("CA_SV_CS3_TYPE", 4.0), ("CA_SV_CS3_TRQ_Y", 1.0),
            ("CA_SV_CS4_TYPE", 9.0), ("CA_SV_CS5_TYPE", 10.0),
        ]
        .iter()
        .map(|(k, x)| (k.to_string(), *x))
        .collect();
        let funcs = [(3, 204.0), (4, 205.0), (5, 101.0), (6, 201.0), (7, 202.0), (8, 203.0), (9, 206.0)];
        for prefix in ["PWM_MAIN", "HIL_ACT"] {
            for (n, f) in funcs {
                v.push((format!("{prefix}_FUNC{n}"), f));
            }
            v.push((format!("{prefix}_REV"), 256.0));
        }
        v
    }

    pub fn px4_plane() -> FixedWing {
        let wing = |cp: Vec3, ch: usize| Surface {
            cp,
            forward: Vec3::new(1.0, 0.0, 0.0),
            upward: flu(0.0, 0.0, 1.0),
            area: 0.6,
            a0: 0.059_842_811_13,
            cla: 4.752_798_721,
            cda: 0.641_711_229_9,
            alpha_stall: 0.339_142_811_1,
            cla_stall: -3.85,
            cda_stall: -0.923_398_405_5,
            control: Some((ch, -0.3)),
        };
        FixedWing {
            mass: 1.5,
            inertia: Vec3::new(0.197_563, 0.145_892_9, 0.1477),
            surfaces: vec![
                wing(flu(-0.05, 0.45, 0.05), 5), // left wing + left aileron
                wing(flu(-0.05, -0.45, 0.05), 6), // right wing + right aileron
                Surface { a0: -0.2, area: 0.01, cp: flu(-0.5, 0.0, 0.0), control: Some((7, -4.0)), ..wing(Vec3::ZERO, 7) }, // tailplane + elevator
                Surface { a0: 0.0, area: 0.02, cp: flu(-0.5, 0.0, 0.05), upward: flu(0.0, 1.0, 0.0), control: Some((2, 0.8)), ..wing(Vec3::ZERO, 2) }, // fin + rudder
            ],
            max_deflection: 0.53,
            motor_channel: 4,
            kt: 8.548_58e-6,
            w_max: 3500.0,
            tau_up: 0.0125,
            tau_down: 0.025,
            v_zero_thrust: 30.0,
            contacts: [(0.2, 0.2), (0.2, -0.2), (-0.2, 0.2), (-0.2, -0.2)].iter().map(|(x, y)| Vec3::new(*x, *y, 0.125)).collect(),
            air_density: 1.2041,
        }
    }

    /// Height of the centre of gravity above the ground when resting on the contacts (m).
    pub fn rest_height(&self) -> f64 {
        self.contacts.iter().map(|c| c.z).fold(0.0, f64::max)
    }

    /// Aerodynamic + propulsion force and moment in the body frame, given body velocity (air-relative), body
    /// rate, raw actuator commands (servos −1..1, motor 0..1) and the motor speed (rad/s).
    pub fn aero(&self, v: Vec3, w: Vec3, u: &[f64], motor_w: f64) -> (Vec3, Vec3) {
        let mut f = Vec3::ZERO;
        let mut m = Vec3::ZERO;
        for s in &self.surfaces {
            let vel = v + w.cross(s.cp);
            let span = s.forward.cross(s.upward);
            let v_ld = vel - span * vel.dot(span);
            let speed = v_ld.norm();
            if speed < 0.1 {
                continue;
            }
            // Angle of attack: airflow below the chord line (towards -upward) is positive.
            let alpha = (-v_ld.dot(s.upward)).atan2(v_ld.dot(s.forward)) + s.a0;
            let (mut cl, mut cd) = if alpha > s.alpha_stall {
                (s.cla * s.alpha_stall + s.cla_stall * (alpha - s.alpha_stall), s.cda * s.alpha_stall + s.cda_stall * (alpha - s.alpha_stall))
            } else if alpha < -s.alpha_stall {
                (-s.cla * s.alpha_stall + s.cla_stall * (alpha + s.alpha_stall), -s.cda * s.alpha_stall + s.cda_stall * (alpha + s.alpha_stall))
            } else {
                (s.cla * alpha, s.cda * alpha)
            };
            if alpha.abs() > s.alpha_stall {
                // Post-stall lift keeps the sign of the angle and fades towards zero.
                cl = if alpha > 0.0 { cl.max(0.0) } else { cl.min(0.0) };
            }
            if let Some((ch, k)) = s.control {
                cl += u.get(ch).copied().unwrap_or(0.0).clamp(-self.max_deflection, self.max_deflection) * k;
            }
            cd = cd.abs();
            let q = 0.5 * self.air_density * speed * speed * s.area;
            let lift_dir = span.cross(v_ld) * (1.0 / speed);
            let drag_dir = v_ld * (-1.0 / speed);
            let force = lift_dir * (cl * q) + drag_dir * (cd * q);
            f += force;
            m += s.cp.cross(force);
        }
        // Puller propeller along +x; thrust fades with airspeed (inflow).
        let fade = (1.0 - v.x.max(0.0) / self.v_zero_thrust).clamp(0.0, 1.0);
        f += Vec3::new(self.kt * motor_w * motor_w * fade, 0.0, 0.0);
        (f, m)
    }
}

/// Integrates one step of the fixed-wing aircraft into the shared vehicle state.
/// `u` = raw actuator commands as PX4 sends them (HIL_ACTUATOR_CONTROLS order). Returns true if touching ground.
#[allow(clippy::too_many_arguments)]
pub fn step(fw: &FixedWing, pos: &mut Vec3, vel: &mut Vec3, att: &mut Quat, rate: &mut Vec3, motor_w: &mut f64, acc_out: &mut Vec3, u: &[f64], g: f64, dt: f64) -> bool {
    // Motor lag.
    let sigma = u.get(fw.motor_channel).copied().unwrap_or(0.0).clamp(0.0, 1.0);
    let w_ref = sigma * fw.w_max;
    let tau = if w_ref > *motor_w { fw.tau_up } else { fw.tau_down };
    *motor_w += (w_ref - *motor_w) * (dt / tau).min(1.0);

    let v_body = att.rotate_inv(*vel);
    let (mut f_b, mut m_b) = fw.aero(v_body, *rate, u, *motor_w);

    // Ground contact: spring-damper normal force, low rolling friction along x, high sideways.
    let mut touching = false;
    for c in &fw.contacts {
        let p = *pos + att.rotate(*c);
        if p.z <= 0.0 {
            continue;
        }
        touching = true;
        let vp_b = v_body + rate.cross(*c);
        let vp_n = att.rotate(vp_b);
        let fn_z = (3000.0 * p.z + 80.0 * vp_n.z.max(0.0)).max(0.0); // N, upwards (−z)
        let normal_b = att.rotate_inv(Vec3::new(0.0, 0.0, -fn_z));
        let mu = |v: f64, k: f64| -k * fn_z * (v / 0.2).tanh();
        let fric_b = Vec3::new(mu(vp_b.x, 0.04), mu(vp_b.y, 0.6), 0.0);
        let fc = normal_b + fric_b;
        f_b += fc;
        m_b += c.cross(fc);
    }

    let j = fw.inertia;
    let wv = *rate;
    let tau_b = m_b - wv.cross(wv.hadamard(j)) - wv * 0.02;
    let wdot = Vec3::new(tau_b.x / j.x, tau_b.y / j.y, tau_b.z / j.z);
    let acc = att.rotate(f_b) * (1.0 / fw.mass) + Vec3::new(0.0, 0.0, g);
    *rate += wdot * dt;
    *att = att.integrate(*rate, dt);
    *vel += acc * dt;
    *pos += *vel * dt;
    *acc_out = acc;
    touching
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn controls_move_the_plane_the_way_px4_expects() {
        let fw = FixedWing::px4_plane();
        let v = Vec3::new(15.0, 0.0, 0.0);
        let mut u = [0.0; 16];
        u[fw.motor_channel] = 0.3;
        let (_, m0) = fw.aero(v, Vec3::ZERO, &u, 0.0);
        let delta = |ch: usize, val: f64| {
            let mut uu = u;
            uu[ch] = val;
            fw.aero(v, Vec3::ZERO, &uu, 0.0).1 - m0
        };
        // PX4 airframe 1030: CA_SV_CS0_TRQ_R -0.5 (left aileron), CS1 +0.5 (right), CS2 TRQ_P +1, CS3 TRQ_Y +1.
        assert!(delta(5, 1.0).x < 0.0, "left aileron + → roll left");
        assert!(delta(6, 1.0).x > 0.0, "right aileron + → roll right");
        assert!(delta(7, 1.0).y > 0.0, "elevator + → nose up");
        assert!(delta(2, 1.0).z > 0.0, "rudder + → nose right");
    }

    #[test]
    fn lift_and_drag_behave() {
        let fw = FixedWing::px4_plane();
        let u = [0.0; 16];
        // Lift rises with speed and angle of attack; airflow from the front gives drag (−x).
        let (f10, _) = fw.aero(Vec3::new(10.0, 0.0, 0.0), Vec3::ZERO, &u, 0.0);
        let (f15, _) = fw.aero(Vec3::new(15.0, 0.0, 0.0), Vec3::ZERO, &u, 0.0);
        assert!(f15.z < f10.z && f10.z < 0.0, "lift is up (−z) and grows with speed");
        let (fa, _) = fw.aero(Vec3::new(15.0, 0.0, 1.5), Vec3::ZERO, &u, 0.0);
        assert!(fa.z < f15.z, "more angle of attack, more lift");
        assert!(f15.x < 0.0, "drag");
        // Pitch damping and weathercock stability.
        let (_, mq) = fw.aero(Vec3::new(15.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), &u, 0.0);
        let (_, m0) = fw.aero(Vec3::new(15.0, 0.0, 0.0), Vec3::ZERO, &u, 0.0);
        assert!(mq.y < m0.y, "pitch rate is damped");
        let (_, mb) = fw.aero(Vec3::new(15.0, 2.0, 0.0), Vec3::ZERO, &u, 0.0);
        assert!(mb.z > m0.z, "sideslip to the right yaws the nose right (into the wind)");
    }

    #[test]
    fn rests_on_the_ground_and_rolls_when_pushed() {
        let fw = FixedWing::px4_plane();
        let (mut pos, mut vel, mut att, mut rate, mut w, mut acc) = (Vec3::new(0.0, 0.0, -fw.rest_height()), Vec3::ZERO, Quat::IDENTITY, Vec3::ZERO, 0.0, Vec3::ZERO);
        let idle = [0.0; 16];
        for _ in 0..3000 {
            step(&fw, &mut pos, &mut vel, &mut att, &mut rate, &mut w, &mut acc, &idle, 9.8, 0.001);
        }
        assert!(pos.z.abs() < 0.2 && vel.norm() < 0.05, "settles on its contacts: {pos:?} {vel:?}");
        let mut full = [0.0; 16];
        full[fw.motor_channel] = 1.0;
        // The first 0.7 s of the take-off roll (with no pilot it lifts off at ~8 m/s and pitches up).
        for _ in 0..700 {
            step(&fw, &mut pos, &mut vel, &mut att, &mut rate, &mut w, &mut acc, &full, 9.8, 0.001);
        }
        assert!(vel.x > 3.0 && pos.z.abs() < 0.3, "full throttle accelerates down the runway: {vel:?} {pos:?}");
        assert!(vel.y.abs() < 0.5, "tracks straight: {vel:?}");
    }
}

#[cfg(test)]
mod trim_probe {
    use super::*;

    /// Prints level-flight behaviour with a simple attitude hold (run with --nocapture).
    #[test]
    #[ignore]
    fn probe_level_flight() {
        let fw = FixedWing::px4_plane();
        for thr in [0.15, 0.25, 0.4, 0.6] {
            let (mut pos, mut vel, mut att, mut rate, mut w, mut acc) = (Vec3::new(0.0, 0.0, -50.0), Vec3::new(15.0, 0.0, 0.0), Quat::IDENTITY, Vec3::ZERO, 2000.0, Vec3::ZERO);
            let mut out = String::new();
            for k in 0..20000 {
                let (r, p, _) = att.to_euler();
                let mut u = [0.0; 16];
                u[fw.motor_channel] = thr;
                let elev = (2.0 * (0.05 - p) - 0.3 * rate.y).clamp(-1.0, 1.0);
                let ail = (-1.5 * r - 0.2 * rate.x).clamp(-1.0, 1.0);
                u[7] = elev;
                u[5] = -ail; // left aileron: + rolls left
                u[6] = ail;
                step(&fw, &mut pos, &mut vel, &mut att, &mut rate, &mut w, &mut acc, &u, 9.8, 0.001);
                if k % 4000 == 3999 {
                    let vb = att.rotate_inv(vel);
                    out += &format!(" | t{:.0}s V={:.1} alt={:.1} pitch={:.1} elev={:.2} α={:.1}°", (k + 1) as f64 / 1000.0, vel.norm(), -pos.z, p.to_degrees(), elev, vb.z.atan2(vb.x).to_degrees());
                }
            }
            println!("throttle {thr}:{out}");
        }
    }
}
