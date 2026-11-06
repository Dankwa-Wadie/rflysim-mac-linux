//! ArduPilot SITL backend (Windows RflySim's APMSITLRun / APMSITLRun_Plane, a Full-edition feature there).
//!
//! Speaks ArduPilot's "JSON" external-physics interface (libraries/SITL/SIM_JSON.cpp): ArduPilot runs with
//! `--model JSON` and sends a binary servo packet to UDP 9002 (+10 per instance) every physics frame; the
//! simulator advances by one frame (1/frame_rate s, lockstep) and replies with one JSON line holding the
//! IMU (gyro, specific force in body FRD), position/velocity NED relative to home, and attitude. ArduPilot
//! makes its own GPS, baro and compass from that, so only the vehicle state goes back.
//!
//! Servo packet (little-endian): u16 magic (18458 = 16 channels, 29569 = 32), u16 frame_rate (Hz),
//! u32 frame_count, u16 pwm[16|32] (µs).

use rfly_core::{Frame, Params, Vehicle};
use std::net::UdpSocket;

pub const MAGIC_16: u16 = 18458;
pub const MAGIC_32: u16 = 29569;

#[derive(Debug, Clone, PartialEq)]
pub struct ServoPacket {
    pub frame_rate: u16,
    pub frame_count: u32,
    pub pwm: Vec<u16>,
}

impl ServoPacket {
    pub fn decode(b: &[u8]) -> Option<ServoPacket> {
        if b.len() < 8 {
            return None;
        }
        let n = match u16::from_le_bytes([b[0], b[1]]) {
            MAGIC_16 => 16,
            MAGIC_32 => 32,
            _ => return None,
        };
        if b.len() < 8 + 2 * n {
            return None;
        }
        let pwm = (0..n).map(|i| u16::from_le_bytes([b[8 + 2 * i], b[9 + 2 * i]])).collect();
        Some(ServoPacket {
            frame_rate: u16::from_le_bytes([b[2], b[3]]),
            frame_count: u32::from_le_bytes(b[4..8].try_into().unwrap()),
            pwm,
        })
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn encode(&self) -> Vec<u8> {
        let mut b = Vec::with_capacity(8 + 2 * self.pwm.len());
        b.extend_from_slice(&(if self.pwm.len() > 16 { MAGIC_32 } else { MAGIC_16 }).to_le_bytes());
        b.extend_from_slice(&self.frame_rate.to_le_bytes());
        b.extend_from_slice(&self.frame_count.to_le_bytes());
        for p in &self.pwm {
            b.extend_from_slice(&p.to_le_bytes());
        }
        b
    }
}

/// ArduCopter's motor layout (AP_MotorsMatrix): motor k at (angle deg from front towards right, CCW?).
fn ardupilot_motors(frame: Frame) -> Option<&'static [(f64, bool)]> {
    const QUAD_X: [(f64, bool); 4] = [(45.0, true), (-135.0, true), (-45.0, false), (135.0, false)];
    const QUAD_PLUS: [(f64, bool); 4] = [(90.0, true), (-90.0, true), (0.0, false), (180.0, false)];
    const HEXA_X: [(f64, bool); 6] =
        [(90.0, false), (-90.0, true), (-30.0, false), (150.0, true), (30.0, true), (-150.0, false)];
    match frame {
        Frame::QuadX => Some(&QUAD_X),
        Frame::QuadPlus => Some(&QUAD_PLUS),
        Frame::HexaX => Some(&HEXA_X),
        // ArduPilot's octo X spins every motor the other way round from PX4's, and its Y6 numbering
        // differs too: not offered for ArduPilot yet.
        Frame::OctoX | Frame::Y6 => None,
    }
}

/// ArduCopter FRAME_CLASS / FRAME_TYPE for a frame.
pub fn ardupilot_frame_params(frame: Frame) -> Option<(u8, u8)> {
    match frame {
        Frame::QuadX => Some((1, 1)),
        Frame::QuadPlus => Some((1, 0)),
        Frame::HexaX => Some((2, 1)),
        _ => None,
    }
}

/// For each ArduPilot output (servo channel k), the model rotor it drives.
pub fn motor_map(frame: Frame) -> Result<Vec<usize>, String> {
    let ap = ardupilot_motors(frame).ok_or_else(|| format!("frame {} is not supported with ArduPilot yet", frame.name()))?;
    let ours = frame.rotors();
    ap.iter()
        .map(|&(deg, ccw)| {
            ours.iter()
                .position(|r| {
                    let d = (r.angle.to_degrees() - deg + 540.0).rem_euclid(360.0) - 180.0;
                    d.abs() < 1.0 && (r.dir > 0.0) == ccw
                })
                .ok_or_else(|| format!("no rotor at {deg}° for ArduPilot"))
        })
        .collect()
}

/// Turns ArduPilot PWM outputs into the model's control vector.
pub enum Mixer {
    /// Servo k → rotor map[k], throttle (pwm-1000)/1000.
    Copter(Vec<usize>),
    /// ArduPlane defaults: 1 aileron, 2 elevator, 3 throttle, 4 rudder (± 500 µs about 1500).
    Plane,
}

/// Full surface deflection (rad) of the plane model: ±500 µs maps onto it.
const PLANE_MAX_DEFLECTION: f64 = 0.53;

impl Mixer {
    pub fn controls(&self, pwm: &[u16]) -> [f64; 16] {
        let p = |i: usize| pwm.get(i).copied().unwrap_or(0) as f64;
        let thr = |i: usize| ((p(i) - 1000.0) / 1000.0).clamp(0.0, 1.0);
        let sym = |i: usize| if p(i) < 900.0 { 0.0 } else { ((p(i) - 1500.0) / 500.0).clamp(-1.0, 1.0) };
        let mut u = [0.0; 16];
        match self {
            Mixer::Copter(map) => {
                for (k, &rotor) in map.iter().enumerate() {
                    u[rotor] = thr(k);
                }
            }
            Mixer::Plane => {
                // Model channels (PX4 gazebo-classic_plane): 2 rudder, 4 motor, 5/6 left/right aileron, 7 elevator.
                let (a, e, r) = (sym(0), sym(1), sym(3));
                u[5] = -a * PLANE_MAX_DEFLECTION;
                u[6] = a * PLANE_MAX_DEFLECTION;
                u[7] = e * PLANE_MAX_DEFLECTION;
                u[2] = r * PLANE_MAX_DEFLECTION;
                u[4] = thr(2);
            }
        }
        u
    }
}

/// The JSON state line ArduPilot expects back.
pub fn state_json(t: f64, v: &Vehicle, airspeed: Option<f64>) -> String {
    let s = &v.s;
    let f = v.specific_force();
    let mut j = format!(
        "\n{{\"timestamp\":{t:.6},\"imu\":{{\"gyro\":[{:.6},{:.6},{:.6}],\"accel_body\":[{:.6},{:.6},{:.6}]}},\
\"position\":[{:.6},{:.6},{:.6}],\"quaternion\":[{:.8},{:.8},{:.8},{:.8}],\"velocity\":[{:.6},{:.6},{:.6}]",
        s.rate.x, s.rate.y, s.rate.z, f.x, f.y, f.z, s.pos.x, s.pos.y, s.pos.z, s.att.w, s.att.x, s.att.y, s.att.z,
        s.vel.x, s.vel.y, s.vel.z
    );
    if let Some(a) = airspeed {
        j.push_str(&format!(",\"airspeed\":{a:.4}"));
    }
    j.push_str("}\n");
    j
}

/// Lockstep with ArduPilot on UDP `port` until killed.
pub fn run(
    port: u16,
    mut vehicle: Vehicle,
    mixer: Mixer,
    mut rfly: Option<crate::rfly_out::RflyOut>,
    mut status: Option<crate::status::StatusOut>,
    init: &dyn Fn(&mut Vehicle),
) -> std::io::Result<()> {
    let sock = UdpSocket::bind(("0.0.0.0", port))?;
    println!("coptersim: ArduPilot SITL (JSON interface) on UDP {port}; start ArduPilot with --model JSON");
    let fresh = vehicle.clone();
    init(&mut vehicle);
    let fixed_wing = vehicle.fw.is_some();
    let mut buf = [0u8; 2048];
    let (mut t, mut last_frame, mut next_report, mut connected) = (0.0f64, None::<u32>, 1.0f64, false);
    let mut reply = String::new();
    loop {
        let (n, from) = sock.recv_from(&mut buf)?;
        let Some(pkt) = ServoPacket::decode(&buf[..n]) else { continue };
        if !connected {
            println!("coptersim: ArduPilot connected from {from} ({} Hz frames)", pkt.frame_rate);
            connected = true;
        }
        let frames = match last_frame {
            Some(l) if pkt.frame_count == l => 0, // repeat: resend the last state
            Some(l) if pkt.frame_count < l => {
                println!("coptersim: ArduPilot restarted, resetting the vehicle");
                vehicle = fresh.clone();
                init(&mut vehicle);
                t = 0.0;
                next_report = 1.0;
                1
            }
            Some(l) => (pkt.frame_count - l).min(1000),
            None => 1,
        };
        last_frame = Some(pkt.frame_count);
        if frames > 0 {
            let dt = 1.0 / f64::from(pkt.frame_rate.max(1));
            let u = mixer.controls(&pkt.pwm);
            for _ in 0..frames {
                // Sub-step at <= 1 ms so the physics matches the PX4 path.
                let sub = (dt / crate::PHYSICS_DT).ceil().max(1.0);
                for _ in 0..sub as usize {
                    vehicle.step(&u, dt / sub);
                }
                t += dt;
            }
            let airspeed = fixed_wing.then(|| vehicle.s.att.rotate_inv(vehicle.s.vel).x.max(0.0));
            reply = state_json(t, &vehicle, airspeed);
            if let Some(o) = rfly.as_mut() {
                o.publish(t, &vehicle);
            }
            if let Some(st) = status.as_mut() {
                let armed = pkt.pwm.iter().take(8).any(|&p| p > 1050);
                st.publish(t, &vehicle, armed);
            }
            if t >= next_report {
                let s = &vehicle.s;
                let (r, p, y) = s.att.to_euler();
                println!(
                    "t={:6.1}s  NED=({:7.2},{:7.2},{:7.2})  rpy=({:6.1},{:6.1},{:6.1})°  pwm={:?}",
                    t, s.pos.x, s.pos.y, s.pos.z, r.to_degrees(), p.to_degrees(), y.to_degrees(),
                    &pkt.pwm[..8.min(pkt.pwm.len())]
                );
                next_report = t.floor() + 1.0;
            }
        }
        sock.send_to(reply.as_bytes(), from)?;
    }
}

/// Copter by default; the plane with `--fixed-wing`.
pub fn mixer_for(params: &Params, fixed_wing: bool) -> Result<Mixer, String> {
    if fixed_wing {
        Ok(Mixer::Plane)
    } else {
        motor_map(params.frame).map(Mixer::Copter)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn servo_packet_roundtrip() {
        let p = ServoPacket { frame_rate: 1200, frame_count: 42, pwm: (0..16).map(|i| 1000 + i * 10).collect() };
        assert_eq!(ServoPacket::decode(&p.encode()), Some(p));
        assert_eq!(ServoPacket::decode(&[0u8; 40]), None);
    }

    #[test]
    fn copter_motor_maps_match_px4_geometry() {
        assert_eq!(motor_map(Frame::QuadX).unwrap(), vec![0, 1, 2, 3]); // same numbering as PX4
        assert_eq!(motor_map(Frame::QuadPlus).unwrap(), vec![0, 1, 2, 3]);
        let h = motor_map(Frame::HexaX).unwrap();
        let mut sorted = h.clone();
        sorted.sort();
        assert_eq!(sorted, vec![0, 1, 2, 3, 4, 5]);
        assert!(motor_map(Frame::OctoX).is_err());
    }

    #[test]
    fn copter_hovers_on_ardupilot_pwm() {
        let params = Params::default();
        let mixer = mixer_for(&params, false).unwrap();
        let mut v = Vehicle::new(params.clone());
        let pwm: Vec<u16> = vec![(1000.0 + 1000.0 * (params.hover_throttle() + 0.05)) as u16; 16];
        let u = mixer.controls(&pwm);
        for _ in 0..2000 {
            v.step(&u, 0.001);
        }
        assert!(v.s.pos.z < -0.2, "climbs slightly above hover throttle: z = {}", v.s.pos.z);
        let (r, p, _) = v.s.att.to_euler();
        assert!(r.abs() < 0.01 && p.abs() < 0.01, "level: {r} {p}");
    }

    #[test]
    fn json_has_the_required_fields() {
        let v = Vehicle::new(Params::default());
        let j = state_json(1.5, &v, Some(3.0));
        assert!(j.starts_with('\n') && j.ends_with("}\n"));
        for k in ["timestamp", "gyro", "accel_body", "position", "quaternion", "velocity", "airspeed"] {
            assert!(j.contains(&format!("\"{k}\"")), "{k} missing in {j}");
        }
        // At rest the accelerometer reads -g on body z.
        assert!(j.contains("\"accel_body\":[0.000000,0.000000,-9.8"), "{j}");
    }
}
