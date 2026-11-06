//! RflySim's UDP_Full / UDP_Simple communication modes (UDPSIMMODE 0 and 1).
//!
//! In these modes the RflySimSDK Python class (PX4MavCtrler with `InitMavLoop(0 or 1)`) does not speak
//! MAVLink. It sends RflySim's packed control structs to CopterSim on UDP 20100+2(id-1) and reads packed state
//! structs on 20101+2(id-1). CopterSim turns the controls into MAVLink for PX4 and PX4's telemetry into the
//! state structs. Formats follow the public RflySimSDK `PX4MavCtrlV4.py`:
//!
//! * Python → CopterSim, `inOffboardShortData` (24 B, any mode): `<2i4f` checksum 1234567890, ctrlMode, ctrls[4].
//!   ctrlMode 0 vel NED + yaw rate, 1 vel body + yaw rate, 2 pos NED + yaw, 3 pos body + yaw, 6 acc,
//!   7 acc + yaw rate, 8 acc + yaw, 9 arm (ctrls[0] 1/0), 10 cruise speed / loiter radius, 11 takeoff to x,y,z.
//! * Python → CopterSim, UDP_Full `inHILCMDData` (120 B): `<2i` tg=3, len=80, then `<4I16f` time_boot_ms, copterID,
//!   type_mask, coordinate_frame, ctrls = pos[3] vel[3] acc[3] yaw yawrate + 5 spare, zero-padded to 112 bytes.
//! * CopterSim → Python, UDP_Full (120 B): `<2i` tg (crashed-with vehicle, -1 none), len=112, then `<2I14i12f`
//!   time_boot_ms, copterID, GpsPos[3] (deg·1e7, deg·1e7, mm), GpsVel[3] (cm/s), gpsHome[3], relative_alt (mm),
//!   hdg (deg·1000), satellites, fix_type, reserved, AngEular[3], localPos[3], localVel[3], pos accuracy h/v,
//!   reserved float (> 0.5 once PX4's EKF has a 3D fix).
//! * CopterSim → Python, UDP_Simple `outHILStateShort` (52 B): `<4i9f` checksum 1234567890, gpsHome[3],
//!   AngEular[3], localPos[3], localVel[3]; one packet with checksum 1234567891 when the EKF has a 3D fix.

#![allow(deprecated)] // MAV_FRAME_BODY_NED (8): what RflySimSDK sends; PX4 still accepts it.
use mavlink::dialects::common::{
    MavCmd, MavFrame, MavMessage, PositionTargetTypemask, COMMAND_LONG_DATA, SET_POSITION_TARGET_LOCAL_NED_DATA,
};
use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub const UDP_FULL: u8 = 0;
pub const UDP_SIMPLE: u8 = 1;
const CHECKSUM: i32 = 1_234_567_890;
const CHECKSUM_EKF_OK: i32 = 1_234_567_891;

/// PX4 state as reported over MAVLink, kept for the state packets.
#[derive(Default, Clone, Debug)]
pub struct Telemetry {
    pub time_boot_ms: u32,
    pub gps: [i32; 3],
    pub gps_vel: [i32; 3],
    pub home: [i32; 3],
    pub relative_alt: i32,
    pub hdg: i32,
    pub sats: i32,
    pub fix: i32,
    pub att: [f32; 3],
    pub pos: [f32; 3],
    pub vel: [f32; 3],
    pub has_home: bool,
    pub has_local: bool,
    pub armed: bool,
    pub custom_mode: u32,
}

impl Telemetry {
    pub fn update(&mut self, m: &MavMessage) {
        match m {
            MavMessage::ATTITUDE(a) => {
                self.att = [a.roll, a.pitch, a.yaw];
                self.time_boot_ms = a.time_boot_ms;
            }
            MavMessage::LOCAL_POSITION_NED(p) => {
                self.pos = [p.x, p.y, p.z];
                self.vel = [p.vx, p.vy, p.vz];
                self.has_local = true;
            }
            MavMessage::GLOBAL_POSITION_INT(g) => {
                self.gps = [g.lat, g.lon, g.alt];
                self.gps_vel = [g.vx as i32, g.vy as i32, g.vz as i32];
                self.relative_alt = g.relative_alt;
                self.hdg = g.hdg as i32 * 10;
            }
            MavMessage::GPS_RAW_INT(g) => {
                self.sats = g.satellites_visible as i32;
                self.fix = g.fix_type as i32;
            }
            MavMessage::HOME_POSITION(h) => {
                self.home = [h.latitude, h.longitude, h.altitude];
                self.has_home = true;
            }
            MavMessage::HEARTBEAT(h) if h.autopilot != mavlink::dialects::common::MavAutopilot::MAV_AUTOPILOT_INVALID => {
                self.armed = h.base_mode.contains(mavlink::dialects::common::MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
                self.custom_mode = h.custom_mode;
            }
            _ => {}
        }
    }

    pub fn ekf_ok(&self) -> bool {
        self.has_local && self.has_home
    }
}

pub type Sender = Arc<dyn Fn(&MavMessage) + Send + Sync>;

/// One decoded control packet from Python.
#[derive(Debug, Clone, PartialEq)]
pub enum Control {
    Setpoint { mask: u16, frame: MavFrame, pos: [f32; 3], vel: [f32; 3], acc: [f32; 3], yaw: f32, yaw_rate: f32 },
    Arm(bool),
    Takeoff([f32; 3]),
    CruiseSpeed(f32),
    LoiterRadius(f32),
}

const IGN_POS: u16 = 0b111;
const IGN_VEL: u16 = 0b111_000;
const IGN_ACC: u16 = 0b111_000_000;
const IGN_YAW: u16 = 1024;
const IGN_YAW_RATE: u16 = 2048;

fn finite(v: f32) -> f32 {
    if v.is_finite() {
        v
    } else {
        0.0
    }
}

/// Decodes a Python → CopterSim packet (24-byte short form in any mode, 120-byte UDP_Full form).
pub fn decode(buf: &[u8]) -> Option<Control> {
    let i32_at = |o: usize| i32::from_le_bytes(buf[o..o + 4].try_into().unwrap());
    let u32_at = |o: usize| u32::from_le_bytes(buf[o..o + 4].try_into().unwrap());
    let f32_at = |o: usize| f32::from_le_bytes(buf[o..o + 4].try_into().unwrap());
    if buf.len() == 24 && i32_at(0) == CHECKSUM {
        let mode = i32_at(4);
        let c = [f32_at(8), f32_at(12), f32_at(16), f32_at(20)];
        let nan = f32::NAN;
        let sp = |mask: u16, frame: MavFrame, pos: [f32; 3], vel: [f32; 3], acc: [f32; 3], yaw: f32, yaw_rate: f32| {
            // NaN means "not set" in RflySimSDK: ignore those axes.
            let mut m = mask;
            for (k, v) in pos.iter().enumerate() {
                if v.is_nan() {
                    m |= 1 << k;
                }
            }
            for (k, v) in vel.iter().enumerate() {
                if v.is_nan() {
                    m |= 8 << k;
                }
            }
            if yaw.is_nan() {
                m |= IGN_YAW;
            }
            if yaw_rate.is_nan() {
                m |= IGN_YAW_RATE;
            }
            Some(Control::Setpoint { mask: m, frame, pos: pos.map(finite), vel: vel.map(finite), acc: acc.map(finite), yaw: finite(yaw), yaw_rate: finite(yaw_rate) })
        };
        let v3 = [c[0], c[1], c[2]];
        return match mode {
            0 => sp(IGN_POS | IGN_ACC | IGN_YAW, MavFrame::MAV_FRAME_LOCAL_NED, [nan; 3], v3, [0.0; 3], nan, c[3]),
            1 => sp(IGN_POS | IGN_ACC | IGN_YAW, MavFrame::MAV_FRAME_BODY_NED, [nan; 3], v3, [0.0; 3], nan, c[3]),
            2 => sp(IGN_VEL | IGN_ACC | IGN_YAW_RATE, MavFrame::MAV_FRAME_LOCAL_NED, v3, [nan; 3], [0.0; 3], c[3], nan),
            3 => sp(IGN_VEL | IGN_ACC | IGN_YAW_RATE, MavFrame::MAV_FRAME_BODY_NED, v3, [nan; 3], [0.0; 3], c[3], nan),
            6 => sp(IGN_POS | IGN_VEL | IGN_YAW | IGN_YAW_RATE, MavFrame::MAV_FRAME_LOCAL_NED, [nan; 3], [nan; 3], v3, nan, nan),
            7 => sp(IGN_POS | IGN_VEL | IGN_YAW, MavFrame::MAV_FRAME_LOCAL_NED, [nan; 3], [nan; 3], v3, nan, c[3]),
            8 => sp(IGN_POS | IGN_VEL | IGN_YAW_RATE, MavFrame::MAV_FRAME_LOCAL_NED, [nan; 3], [nan; 3], v3, c[3], nan),
            9 => Some(Control::Arm(c[0] > 0.5)),
            10 if c[0] > 0.0 => Some(Control::CruiseSpeed(c[0])),
            10 => Some(Control::LoiterRadius(c[1])),
            11 => Some(Control::Takeoff(v3.map(finite))),
            _ => None,
        };
    }
    if buf.len() == 120 && i32_at(0) == 3 && i32_at(4) >= 80 {
        let b = &buf[8..];
        let f = |k: usize| f32::from_le_bytes(b[16 + 4 * k..20 + 4 * k].try_into().unwrap());
        let mask = u32::from_le_bytes(b[8..12].try_into().unwrap()) as u16;
        let frame = match u32::from_le_bytes(b[12..16].try_into().unwrap()) {
            8 => MavFrame::MAV_FRAME_BODY_NED,
            9 => MavFrame::MAV_FRAME_BODY_OFFSET_NED,
            7 => MavFrame::MAV_FRAME_LOCAL_OFFSET_NED,
            12 => MavFrame::MAV_FRAME_BODY_FRD,
            _ => MavFrame::MAV_FRAME_LOCAL_NED,
        };
        let _ = u32_at;
        return Some(Control::Setpoint {
            mask,
            frame,
            pos: [f(0), f(1), f(2)].map(finite),
            vel: [f(3), f(4), f(5)].map(finite),
            acc: [f(6), f(7), f(8)].map(finite),
            yaw: finite(f(9)),
            yaw_rate: finite(f(10)),
        });
    }
    None
}

/// Encodes the CopterSim → Python state packet for the mode.
pub fn encode_state(mode: u8, id: u32, t: &Telemetry, crash_with: i32) -> Vec<u8> {
    let mut b = Vec::with_capacity(120);
    if mode == UDP_SIMPLE {
        b.extend(CHECKSUM.to_le_bytes());
        for v in t.home {
            b.extend(v.to_le_bytes());
        }
        for v in t.att.iter().chain(&t.pos).chain(&t.vel) {
            b.extend(v.to_le_bytes());
        }
        return b;
    }
    b.extend(crash_with.to_le_bytes());
    b.extend(112i32.to_le_bytes());
    b.extend(t.time_boot_ms.to_le_bytes());
    b.extend(id.to_le_bytes());
    for v in t.gps.iter().chain(&t.gps_vel).chain(&t.home) {
        b.extend(v.to_le_bytes());
    }
    for v in [t.relative_alt, t.hdg, t.sats, t.fix, 0] {
        b.extend(v.to_le_bytes());
    }
    for v in t.att.iter().chain(&t.pos).chain(&t.vel) {
        b.extend(v.to_le_bytes());
    }
    for v in [0.5f32, 0.8, if t.ekf_ok() { 1.0 } else { 0.0 }] {
        b.extend(v.to_le_bytes());
    }
    b
}

/// MAVLink for one control packet.
fn to_mavlink(c: &Control, sysid: u8) -> Vec<MavMessage> {
    let cmd = |command: MavCmd, p: [f32; 7]| {
        MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            target_system: sysid,
            target_component: 1,
            command,
            confirmation: 0,
            param1: p[0],
            param2: p[1],
            param3: p[2],
            param4: p[3],
            param5: p[4],
            param6: p[5],
            param7: p[6],
        })
    };
    match *c {
        Control::Setpoint { mask, frame, pos, vel, acc, yaw, yaw_rate } => vec![MavMessage::SET_POSITION_TARGET_LOCAL_NED(SET_POSITION_TARGET_LOCAL_NED_DATA {
            time_boot_ms: 0,
            target_system: sysid,
            target_component: 1,
            coordinate_frame: frame,
            type_mask: PositionTargetTypemask::from_bits_truncate(mask),
            x: pos[0],
            y: pos[1],
            z: pos[2],
            vx: vel[0],
            vy: vel[1],
            vz: vel[2],
            afx: acc[0],
            afy: acc[1],
            afz: acc[2],
            yaw,
            yaw_rate,
        })],
        Control::Arm(on) => vec![cmd(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, [if on { 1.0 } else { 0.0 }, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])],
        // PX4 takes off to the altitude in param7 (AMSL); RflySim gives a local target. Climb in offboard instead.
        Control::Takeoff(p) => to_mavlink(
            &Control::Setpoint { mask: IGN_VEL | IGN_ACC | IGN_YAW_RATE, frame: MavFrame::MAV_FRAME_LOCAL_NED, pos: p, vel: [0.0; 3], acc: [0.0; 3], yaw: 0.0, yaw_rate: 0.0 },
            sysid,
        ),
        Control::CruiseSpeed(v) => vec![cmd(MavCmd::MAV_CMD_DO_CHANGE_SPEED, [1.0, v, -1.0, 0.0, 0.0, 0.0, 0.0])],
        Control::LoiterRadius(_) => vec![],
    }
}

/// PX4 custom mode for OFFBOARD (main mode 6, in bits 16..23).
const PX4_OFFBOARD: u32 = 6 << 16;

/// Serves the UDP mode on 20100+2(id-1) (Python sends) / 20101+2(id-1) (Python listens).
pub fn spawn(mode: u8, id: u32, send: Sender, tele: Arc<Mutex<Telemetry>>, crash_with: Arc<Mutex<i32>>) -> std::io::Result<()> {
    let port = rfly_proto::mav_port(id);
    let rx = UdpSocket::bind(("0.0.0.0", port))?;
    rx.set_read_timeout(Some(Duration::from_millis(5)))?;
    let tx = UdpSocket::bind("0.0.0.0:0")?;
    let default_dest: SocketAddr = ([127, 0, 0, 1], port + 1).into();
    println!("coptersim: {} mode for Python on UDP {port} (in) / {} (out)", if mode == UDP_FULL { "UDP_Full" } else { "UDP_Simple" }, port + 1);
    std::thread::spawn(move || {
        let period = Duration::from_secs_f64(if mode == UDP_FULL { 0.01 } else { 1.0 / 30.0 });
        let mut dest = default_dest;
        let mut next = Instant::now();
        let mut last_setpoint: Option<Control> = None;
        let mut setpoint_since: Option<Instant> = None;
        let mut last_mode_cmd = Instant::now() - Duration::from_secs(10);
        let mut ekf_sent = false;
        let mut buf = [0u8; 512];
        loop {
            if let Ok((n, from)) = rx.recv_from(&mut buf) {
                // Answer the sender's IP on port+1 (RflySimSDK listens there).
                dest = SocketAddr::new(from.ip(), port + 1);
                if let Some(c) = decode(&buf[..n]) {
                    for m in to_mavlink(&c, 1) {
                        send(&m);
                    }
                    match c {
                        Control::Setpoint { .. } | Control::Takeoff(_) => {
                            setpoint_since.get_or_insert_with(Instant::now);
                            last_setpoint = Some(c);
                        }
                        Control::Arm(false) => setpoint_since = None,
                        _ => {}
                    }
                }
            }
            // RflySimSDK sends no mode change in UDP modes: once setpoints stream, switch PX4 to OFFBOARD
            // (PX4 needs setpoints first), and keep the last setpoint alive at 10+ Hz.
            if let (Some(since), Some(sp)) = (setpoint_since, &last_setpoint) {
                let t = tele.lock().unwrap().clone();
                if since.elapsed() > Duration::from_millis(500) && t.custom_mode & 0x00ff_0000 != PX4_OFFBOARD && last_mode_cmd.elapsed() > Duration::from_secs(1) {
                    last_mode_cmd = Instant::now();
                    send(&MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                        target_system: 1,
                        target_component: 1,
                        command: MavCmd::MAV_CMD_DO_SET_MODE,
                        confirmation: 0,
                        param1: 1.0,
                        param2: 6.0,
                        param3: 0.0,
                        param4: 0.0,
                        param5: 0.0,
                        param6: 0.0,
                        param7: 0.0,
                    }));
                }
                if Instant::now() >= next {
                    for m in to_mavlink(sp, 1) {
                        send(&m);
                    }
                }
            }
            if Instant::now() >= next {
                next += period;
                if next < Instant::now() {
                    next = Instant::now() + period;
                }
                let t = tele.lock().unwrap().clone();
                let crash = *crash_with.lock().unwrap();
                let _ = tx.send_to(&encode_state(mode, id, &t, crash), dest);
                if mode == UDP_SIMPLE && t.ekf_ok() && !ekf_sent {
                    ekf_sent = true;
                    let mut p = CHECKSUM_EKF_OK.to_le_bytes().to_vec();
                    p.resize(52, 0);
                    let _ = tx.send_to(&p, dest);
                }
            }
        }
    });
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn short(mode: i32, c: [f32; 4]) -> Vec<u8> {
        let mut b = CHECKSUM.to_le_bytes().to_vec();
        b.extend(mode.to_le_bytes());
        for v in c {
            b.extend(v.to_le_bytes());
        }
        b
    }

    #[test]
    fn short_packets_decode_like_rflysimsdk() {
        match decode(&short(2, [10.0, 5.0, -8.0, 0.5])).unwrap() {
            Control::Setpoint { mask, frame, pos, yaw, .. } => {
                assert_eq!((pos, yaw, frame), ([10.0, 5.0, -8.0], 0.5, MavFrame::MAV_FRAME_LOCAL_NED));
                assert_eq!(mask & IGN_POS, 0, "position used");
                assert_eq!(mask & IGN_VEL, IGN_VEL, "velocity ignored");
                assert_eq!(mask & IGN_YAW, 0, "yaw used");
            }
            c => panic!("{c:?}"),
        }
        // SendPosNEDNoYaw sends NaN yaw → ignored
        match decode(&short(2, [1.0, 2.0, -3.0, f32::NAN])).unwrap() {
            Control::Setpoint { mask, .. } => assert_eq!(mask & IGN_YAW, IGN_YAW),
            c => panic!("{c:?}"),
        }
        match decode(&short(1, [2.0, 0.0, 0.0, 0.3])).unwrap() {
            Control::Setpoint { frame, vel, yaw_rate, .. } => assert_eq!((frame, vel, yaw_rate), (MavFrame::MAV_FRAME_BODY_NED, [2.0, 0.0, 0.0], 0.3)),
            c => panic!("{c:?}"),
        }
        assert_eq!(decode(&short(9, [1.0, 0.0, 0.0, 0.0])), Some(Control::Arm(true)));
        assert_eq!(decode(&short(9, [0.0, 0.0, 0.0, 0.0])), Some(Control::Arm(false)));
        assert_eq!(decode(&short(11, [0.0, 0.0, -5.0, 0.0])), Some(Control::Takeoff([0.0, 0.0, -5.0])));
        assert!(decode(&[0u8; 24]).is_none(), "wrong checksum");
    }

    #[test]
    fn full_packets_decode_and_states_have_rflysim_sizes() {
        // sendMavOffboardAPI, UDP_Full: header (3, 80) + <4I16f> + zero padding to 112.
        let mut body = Vec::new();
        for v in [123u32, 1, 0b0000_1101_1111_1000, 1] {
            body.extend(v.to_le_bytes());
        }
        for v in [1.0f32, 2.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] {
            body.extend(v.to_le_bytes());
        }
        let mut b = 3i32.to_le_bytes().to_vec();
        b.extend((body.len() as i32).to_le_bytes());
        b.extend(&body);
        b.resize(120, 0);
        match decode(&b).unwrap() {
            Control::Setpoint { pos, yaw, mask, .. } => assert_eq!((pos, yaw, mask), ([1.0, 2.0, -3.0], 0.7, 0b0000_1101_1111_1000)),
            c => panic!("{c:?}"),
        }
        let t = Telemetry { home: [473977000, 85456000, 488000], att: [0.1, 0.2, 0.3], pos: [1.0, 2.0, -3.0], has_home: true, has_local: true, ..Default::default() };
        let full = encode_state(UDP_FULL, 1, &t, -1);
        assert_eq!(full.len(), 120);
        assert_eq!(i32::from_le_bytes(full[4..8].try_into().unwrap()), 112);
        // RflySimSDK: struct.unpack('2I14i12f') → UIV[19..22] is localPos, UIV[27] the EKF flag.
        let f = |k: usize| f32::from_le_bytes(full[8 + 4 * k..12 + 4 * k].try_into().unwrap());
        assert_eq!((f(19), f(20), f(21), f(27)), (1.0, 2.0, -3.0, 1.0));
        let simple = encode_state(UDP_SIMPLE, 1, &t, -1);
        assert_eq!(simple.len(), 52);
        // '4i9f': UIV[7..10] localPos
        let g = |k: usize| f32::from_le_bytes(simple[4 * k..4 * k + 4].try_into().unwrap());
        assert_eq!((g(7), g(8), g(9)), (1.0, 2.0, -3.0));
    }
}
