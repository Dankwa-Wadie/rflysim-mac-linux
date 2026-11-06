//! 10 Hz status lines for the CopterSim window's live panel (`--status-port`).
//! Format (UDP, text): `RFLYSTAT id t x y z vx vy vz roll pitch yaw u1 u2 u3 u4 armed`
//! (NED m and m/s, Euler degrees, motor throttle 0..1 recovered from rotor speed, armed 0/1).

use rfly_core::{Params, Vehicle};
use std::net::UdpSocket;

pub struct StatusOut {
    sock: UdpSocket,
    port: u16,
    id: u32,
    cr: f64,
    wb: f64,
    next_t: f64,
}

impl StatusOut {
    pub fn new(port: u16, id: u32, p: &Params) -> std::io::Result<StatusOut> {
        Ok(StatusOut { sock: UdpSocket::bind("127.0.0.1:0")?, port, id, cr: p.cr, wb: p.wb, next_t: 0.0 })
    }

    /// Sends at most every 0.1 s of simulated time.
    pub fn publish(&mut self, t: f64, v: &Vehicle, armed: bool) {
        if t < self.next_t {
            return;
        }
        self.next_t = t + 0.1;
        let s = &v.s;
        let (r, p, y) = s.att.to_euler();
        let u: Vec<String> = s.motor_w.iter().take(4).map(|w| format!("{:.3}", ((w - self.wb) / self.cr).clamp(0.0, 1.0))).collect();
        let line = format!(
            "RFLYSTAT {} {:.2} {:.3} {:.3} {:.3} {:.3} {:.3} {:.3} {:.2} {:.2} {:.2} {} {}",
            self.id, t, s.pos.x, s.pos.y, s.pos.z, s.vel.x, s.vel.y, s.vel.z,
            r.to_degrees(), p.to_degrees(), y.to_degrees(), u.join(" "), armed as u8
        );
        let _ = self.sock.send_to(line.as_bytes(), ("127.0.0.1", self.port));
    }
}
