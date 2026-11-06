//! What Windows CopterSim broadcasts besides MAVLink:
//! * vehicle state to RflySim3D windows (`SOut2SimulatorSimpleTime`, UDP 20010+w), 60 Hz;
//! * true state to Python (`SOut2Simulator`, UDP 30101 + 2(id-1)), 100 Hz. This is what
//!   `PX4MavCtrler.InitTrueDataLoop()` listens for.
//!
//! Rates are in **simulation time**, so lockstep (ext) mode streams correctly too.

use rfly_core::{Home, Vehicle};
use rfly_proto::*;
use std::net::{SocketAddr, UdpSocket};

pub struct RflyOut {
    sock: UdpSocket,
    id: i32,
    vtype: i32,
    ue4: Vec<SocketAddr>,
    true_to: SocketAddr,
    home: Home,
    next_ue4: f64,
    next_true: f64,
    pub fg: Option<crate::fg_out::FgOut>,
}

impl RflyOut {
    /// `ue4_ip` = where RflySim3D runs; `windows` = how many windows (20010..) to feed.
    pub fn new(id: u32, vtype: i32, ue4_ip: &str, windows: u16, home: Home) -> std::io::Result<Self> {
        let sock = UdpSocket::bind("0.0.0.0:0")?;
        let ue4 = (0..windows).map(|w| format!("{ue4_ip}:{}", UE4_BASE_PORT + w).parse()).collect::<Result<_, _>>().map_err(std::io::Error::other)?;
        let true_to = format!("127.0.0.1:{}", true_port(id) + 1).parse().unwrap();
        println!(
            "coptersim: RflySim outputs: vehicle #{id} → RflySim3D {ue4_ip}:{}..{}, true data → {true_to}",
            UE4_BASE_PORT,
            UE4_BASE_PORT + windows.saturating_sub(1)
        );
        Ok(Self { sock, id: id as i32, vtype, ue4, true_to, home, next_ue4: 0.0, next_true: 0.0, fg: None })
    }

    fn lla(&self, v: &Vehicle) -> [f64; 3] {
        const R: f64 = 6_378_137.0;
        let p = v.s.pos;
        let lat = self.home.lat_deg + (p.x / R).to_degrees();
        let lon = self.home.lon_deg + (p.y / (R * self.home.lat_deg.to_radians().cos())).to_degrees();
        [lat, lon, self.home.alt_m - p.z]
    }

    /// Call after every physics step with the simulation time (s).
    pub fn publish(&mut self, t: f64, v: &Vehicle) {
        if let Some(fg) = self.fg.as_mut() {
            fg.publish(t, v);
        }
        if t < self.next_ue4 && t < self.next_true {
            return;
        }
        let s = &v.s;
        let (r, p, y) = s.att.to_euler();
        let rpm: [f32; 8] = std::array::from_fn(|i| (s.motor_w[i] * 60.0 / std::f64::consts::TAU) as f32);
        let gps = self.lla(v);
        if t >= self.next_ue4 {
            self.next_ue4 = t + 1.0 / 60.0;
            let u = VehicleUpdate {
                copter_id: self.id,
                vehicle_type: self.vtype,
                pos: [s.pos.x, s.pos.y, s.pos.z],
                euler: [r as f32, p as f32, y as f32],
                vel: [s.vel.x as f32, s.vel.y as f32, s.vel.z as f32],
                motors: rpm,
                scale: None,
                fit_ground: false,
                time: t,
            };
            let pkt = encode_vehicle_simple_time(&u, gps);
            for to in &self.ue4 {
                let _ = self.sock.send_to(&pkt, to);
            }
        }
        if t >= self.next_true {
            self.next_true = t + 0.01;
            let acc = v.specific_force();
            let fs = FullState {
                copter_id: self.id,
                vehicle_type: self.vtype,
                vel: [s.vel.x as f32, s.vel.y as f32, s.vel.z as f32],
                euler: [r as f32, p as f32, y as f32],
                quat: [s.att.w as f32, s.att.x as f32, s.att.y as f32, s.att.z as f32],
                motor_rpm: rpm,
                acc_b: [acc.x as f32, acc.y as f32, acc.z as f32],
                rate_b: [s.rate.x as f32, s.rate.y as f32, s.rate.z as f32],
                time: t,
                pos: [s.pos.x, s.pos.y, s.pos.z],
                gps,
            };
            let _ = self.sock.send_to(&encode_full_state(&fs), self.true_to);
        }
    }
}
