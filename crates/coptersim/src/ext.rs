//! External-controller lockstep mode ("RflyMac protocol v1"), used by the MATLAB/Simulink package
//! and by Python/C clients. The controller (Simulink etc.) owns time: each request carries
//! motor commands and a number of 1 ms physics steps; the simulator advances exactly that much
//! and replies with the resulting state and sensor readings.
//!
//! All fields little-endian.
//!
//! Request  (80 bytes):  u32 magic "RFC1", u32 seq, u32 flags, u32 nsteps, f64 motors[8]
//!   flags bit0 = RESET (restore initial state and t = 0 before stepping)
//! Reply   (320 bytes):  u32 magic "RFS1", u32 seq, f64 data[39]:
//!   [0] time_s
//!   [1..4]   pos NED (m)          [4..7]   vel NED (m/s)
//!   [7..11]  quat w,x,y,z         [11..14] euler roll,pitch,yaw (rad)
//!   [14..17] body rate p,q,r      [17..20] accel (specific force, body, m/s²)
//!   [20..23] gyro (rad/s)         [23..26] mag (gauss, body)
//!   [26]     baro altitude AMSL   [27..30] gps lat, lon (deg), alt (m)
//!   [30..38] motor speeds (rad/s) [38]     on_ground (0/1)

use rfly_core::{Home, Params, SensorModel, Vehicle};
use std::net::UdpSocket;

pub const REQ_MAGIC: u32 = u32::from_le_bytes(*b"RFC1");
pub const REP_MAGIC: u32 = u32::from_le_bytes(*b"RFS1");
pub const REQ_LEN: usize = 80;
pub const REP_FIELDS: usize = 39;
pub const REP_LEN: usize = 8 + REP_FIELDS * 8;
pub const FLAG_RESET: u32 = 1;
const MAX_STEPS_PER_REQUEST: u32 = 100_000;

#[derive(Debug, Clone, PartialEq)]
pub struct Request {
    pub seq: u32,
    pub flags: u32,
    pub nsteps: u32,
    pub motors: [f64; 8],
}

impl Request {
    pub fn decode(b: &[u8]) -> Option<Request> {
        if b.len() != REQ_LEN || u32_at(b, 0) != REQ_MAGIC {
            return None;
        }
        let mut motors = [0.0; 8];
        for (i, m) in motors.iter_mut().enumerate() {
            *m = f64_at(b, 16 + 8 * i);
        }
        Some(Request { seq: u32_at(b, 4), flags: u32_at(b, 8), nsteps: u32_at(b, 12), motors })
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn encode(&self) -> [u8; REQ_LEN] {
        let mut b = [0u8; REQ_LEN];
        b[0..4].copy_from_slice(&REQ_MAGIC.to_le_bytes());
        b[4..8].copy_from_slice(&self.seq.to_le_bytes());
        b[8..12].copy_from_slice(&self.flags.to_le_bytes());
        b[12..16].copy_from_slice(&self.nsteps.to_le_bytes());
        for (i, m) in self.motors.iter().enumerate() {
            b[16 + 8 * i..24 + 8 * i].copy_from_slice(&m.to_le_bytes());
        }
        b
    }
}

pub fn encode_reply(seq: u32, data: &[f64; REP_FIELDS]) -> [u8; REP_LEN] {
    let mut b = [0u8; REP_LEN];
    b[0..4].copy_from_slice(&REP_MAGIC.to_le_bytes());
    b[4..8].copy_from_slice(&seq.to_le_bytes());
    for (i, v) in data.iter().enumerate() {
        b[8 + 8 * i..16 + 8 * i].copy_from_slice(&v.to_le_bytes());
    }
    b
}

fn u32_at(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(b[o..o + 4].try_into().unwrap())
}
fn f64_at(b: &[u8], o: usize) -> f64 {
    f64::from_le_bytes(b[o..o + 8].try_into().unwrap())
}

pub struct ExtSim {
    params: Params,
    home: Home,
    pub vehicle: Vehicle,
    sensors: SensorModel,
    steps: u64,
}

impl ExtSim {
    pub fn new(params: Params, home: Home) -> Self {
        Self {
            vehicle: Vehicle::new(params.clone()),
            sensors: SensorModel::new(home),
            params,
            home,
            steps: 0,
        }
    }

    pub fn handle(&mut self, req: &Request) -> [f64; REP_FIELDS] {
        if req.flags & FLAG_RESET != 0 {
            self.vehicle = Vehicle::new(self.params.clone());
            self.sensors = SensorModel::new(self.home);
            self.steps = 0;
        }
        for _ in 0..req.nsteps.min(MAX_STEPS_PER_REQUEST) {
            self.vehicle.step(&req.motors, crate::PHYSICS_DT);
            self.steps += 1;
        }
        self.snapshot()
    }

    fn snapshot(&mut self) -> [f64; REP_FIELDS] {
        let imu = self.sensors.imu(&self.vehicle);
        let gps = self.sensors.gps(&self.vehicle);
        let s = &self.vehicle.s;
        let (r, p, y) = s.att.to_euler();
        let mut d = [0.0; REP_FIELDS];
        d[0] = self.steps as f64 * crate::PHYSICS_DT;
        d[1..4].copy_from_slice(&[s.pos.x, s.pos.y, s.pos.z]);
        d[4..7].copy_from_slice(&[s.vel.x, s.vel.y, s.vel.z]);
        d[7..11].copy_from_slice(&[s.att.w, s.att.x, s.att.y, s.att.z]);
        d[11..14].copy_from_slice(&[r, p, y]);
        d[14..17].copy_from_slice(&[s.rate.x, s.rate.y, s.rate.z]);
        d[17..20].copy_from_slice(&[imu.acc.x, imu.acc.y, imu.acc.z]);
        d[20..23].copy_from_slice(&[imu.gyro.x, imu.gyro.y, imu.gyro.z]);
        d[23..26].copy_from_slice(&[imu.mag.x, imu.mag.y, imu.mag.z]);
        d[26] = imu.pressure_alt;
        d[27..30].copy_from_slice(&[gps.lat_deg, gps.lon_deg, gps.alt_m]);
        d[30..38].copy_from_slice(&s.motor_w);
        d[38] = if s.on_ground { 1.0 } else { 0.0 };
        d
    }
}

/// Serve the protocol on UDP `port` until killed.
pub fn run(
    port: u16,
    params: Params,
    home: Home,
    mut rfly: Option<crate::rfly_out::RflyOut>,
    mut status: Option<crate::status::StatusOut>,
    init: &dyn Fn(&mut Vehicle),
) -> std::io::Result<()> {
    let sock = UdpSocket::bind(("0.0.0.0", port))?;
    println!("coptersim: external-controller lockstep mode on UDP {port} (RflyMac protocol v1)");
    let mut sim = ExtSim::new(params, home);
    init(&mut sim.vehicle);
    let mut buf = [0u8; 2048];
    let mut next_report = 1.0;
    loop {
        let (n, from) = sock.recv_from(&mut buf)?;
        let Some(req) = Request::decode(&buf[..n]) else { continue };
        if req.flags & FLAG_RESET != 0 {
            println!("coptersim: reset by {from}");
            next_report = 1.0;
        }
        let reset = req.flags & FLAG_RESET != 0;
        let data = sim.handle(&req);
        if reset && req.nsteps == 0 {
            init(&mut sim.vehicle);
        }
        if let Some(o) = rfly.as_mut() {
            o.publish(data[0], &sim.vehicle);
        }
        if let Some(st) = status.as_mut() {
            st.publish(data[0], &sim.vehicle, true);
        }
        sock.send_to(&encode_reply(req.seq, &data), from)?;
        if data[0] >= next_report {
            println!(
                "t={:6.1}s  NED=({:7.2},{:7.2},{:7.2})  rpy=({:6.1},{:6.1},{:6.1})°",
                data[0], data[1], data[2], data[3],
                data[11].to_degrees(), data[12].to_degrees(), data[13].to_degrees()
            );
            next_report = data[0].floor() + 1.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_roundtrip() {
        let r = Request { seq: 7, flags: FLAG_RESET, nsteps: 4, motors: [0.6; 8] };
        assert_eq!(Request::decode(&r.encode()), Some(r));
        assert_eq!(Request::decode(&[0u8; REQ_LEN]), None);
    }

    #[test]
    fn lockstep_advances_exact_time_and_resets() {
        let mut sim = ExtSim::new(Params::default(), Home::default());
        let h = Params::default().hover_throttle() + 0.1;
        let mut req = Request { seq: 1, flags: 0, nsteps: 4, motors: [h; 8] };
        let mut d = [0.0; REP_FIELDS];
        for _ in 0..250 {
            d = sim.handle(&req);
        }
        assert!((d[0] - 1.0).abs() < 1e-9, "t = {}", d[0]);
        assert!(d[3] < -0.5, "climbed, z = {}", d[3]);
        assert_eq!(d[38], 0.0);
        req.flags = FLAG_RESET;
        req.nsteps = 0;
        let d = sim.handle(&req);
        assert_eq!(d[0], 0.0);
        assert_eq!(d[3], 0.0);
        assert_eq!(d[38], 1.0);
    }
}
