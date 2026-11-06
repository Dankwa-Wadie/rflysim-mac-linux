//! FlightGear visual output: the "FlightGear-F450" RflyTools shortcut.
//! Sends FGNetFDM (version 24, 408 bytes, network byte order) to
//! `fgfs --fdm=null --native-fdm=socket,in,30,,5502,udp --aircraft=F450`.

use rfly_core::{Home, Vehicle};
use std::net::UdpSocket;

pub const FG_NET_FDM_VERSION: u32 = 24;
pub const FG_NET_FDM_LEN: usize = 408;

pub struct FgOut {
    sock: UdpSocket,
    to: String,
    home: Home,
    next: f64,
}

impl FgOut {
    pub fn new(to: &str, home: Home) -> std::io::Result<Self> {
        println!("coptersim: FlightGear native-fdm → udp {to}");
        Ok(Self { sock: UdpSocket::bind("0.0.0.0:0")?, to: to.to_string(), home, next: 0.0 })
    }

    pub fn publish(&mut self, t: f64, v: &Vehicle) {
        if t < self.next {
            return;
        }
        self.next = t + 1.0 / 30.0;
        let _ = self.sock.send_to(&encode(&self.home, v, t), &self.to);
    }
}

pub fn encode(home: &Home, v: &Vehicle, t: f64) -> Vec<u8> {
    const R: f64 = 6_378_137.0;
    let s = &v.s;
    let lat = home.lat_deg.to_radians() + s.pos.x / R;
    let lon = home.lon_deg.to_radians() + s.pos.y / (R * home.lat_deg.to_radians().cos());
    let (roll, pitch, yaw) = s.att.to_euler();
    let vb = s.att.rotate_inv(s.vel);
    let acc = v.specific_force();
    let rpm: Vec<f32> = s.motor_w.iter().take(4).map(|w| (w * 60.0 / std::f64::consts::TAU) as f32).collect();

    let mut b = Vec::with_capacity(FG_NET_FDM_LEN);
    let u = |b: &mut Vec<u8>, x: u32| b.extend_from_slice(&x.to_be_bytes());
    let f = |b: &mut Vec<u8>, x: f32| b.extend_from_slice(&x.to_be_bytes());
    let d = |b: &mut Vec<u8>, x: f64| b.extend_from_slice(&x.to_be_bytes());
    u(&mut b, FG_NET_FDM_VERSION);
    u(&mut b, 0); // padding
    d(&mut b, lon);
    d(&mut b, lat);
    d(&mut b, home.alt_m - s.pos.z);
    for x in [
        (-s.pos.z) as f32, // agl
        roll as f32,
        pitch as f32,
        yaw as f32,
        0.0, // alpha
        0.0, // beta
        s.rate.x as f32,
        s.rate.y as f32,
        s.rate.z as f32,
        (s.vel.x.hypot(s.vel.y) * 1.943_844) as f32, // vcas (kt)
        (-s.vel.z * 3.28084) as f32,                // climb rate (ft/s)
        (s.vel.x * 3.28084) as f32,
        (s.vel.y * 3.28084) as f32,
        (s.vel.z * 3.28084) as f32,
        (vb.x * 3.28084) as f32,
        (vb.y * 3.28084) as f32,
        (vb.z * 3.28084) as f32,
        (acc.x * 3.28084) as f32,
        (acc.y * 3.28084) as f32,
        (acc.z * 3.28084) as f32,
        0.0, // stall warning
        0.0, // slip
    ] {
        f(&mut b, x);
    }
    u(&mut b, 4); // num_engines
    for _ in 0..4 {
        u(&mut b, if v.s.motor_w[0] > 1.0 { 2 } else { 0 }); // eng_state: running
    }
    for k in 0..4 {
        f(&mut b, rpm.get(k).copied().unwrap_or(0.0));
    }
    b.resize(b.len() + 8 * 4 * 4, 0); // fuel_flow, fuel_px, egt, cht, mp_osi, tit, oil_temp, oil_px
    u(&mut b, 0); // num_tanks
    b.resize(b.len() + 16, 0); // fuel_quantity
    u(&mut b, 3); // num_wheels
    for _ in 0..3 {
        u(&mut b, s.on_ground as u32);
    }
    b.resize(b.len() + 36, 0); // gear pos / steer / compression
    u(&mut b, t as u32); // cur_time
    u(&mut b, 0); // warp
    f(&mut b, 5000.0); // visibility (m)
    b.resize(b.len() + 40, 0); // control surfaces
    debug_assert_eq!(b.len(), FG_NET_FDM_LEN);
    b
}

#[cfg(test)]
mod tests {
    use super::*;
    use rfly_core::Params;

    #[test]
    fn net_fdm_layout() {
        let mut v = Vehicle::new(Params::default());
        v.s.pos.z = -10.0;
        let home = Home::default();
        let b = encode(&home, &v, 1.0);
        assert_eq!(b.len(), FG_NET_FDM_LEN);
        assert_eq!(u32::from_be_bytes(b[0..4].try_into().unwrap()), 24);
        let alt = f64::from_be_bytes(b[24..32].try_into().unwrap());
        assert!((alt - (home.alt_m + 10.0)).abs() < 1e-9);
        let agl = f32::from_be_bytes(b[32..36].try_into().unwrap());
        assert_eq!(agl, 10.0);
    }
}
