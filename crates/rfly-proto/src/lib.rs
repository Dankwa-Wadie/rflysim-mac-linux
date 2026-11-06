//! rfly-proto: RflySim's UDP wire formats (CopterSim ⇄ RflySim3D ⇄ Python/Simulink).
//!
//! Every packet is a packed C struct with **native x86-64/arm64 alignment**, which matches
//! Python's `struct.pack` without a byte-order prefix. All values are little-endian.
//! Packets are told apart by **length first, then checksum**, the same way the
//! RflySimSDK Python does it. Layouts come from the public RflySimSDK
//! (`UE4CtrlAPI.py`, `PX4MavCtrlV4.py`, `VisionCaptureApi.py`). See docs/RFLYSIM3D_PROTOCOL.md.
//!
//! Ports:
//! * `20010 + windowID`: RflySim3D window inbox (vehicle states, commands, sensor requests).
//! * `224.0.0.10:20009`: multicast commands to every RflySim3D on the LAN.
//! * `20006` (+ multicast `224.0.0.10:20006`): RflySim3D → clients (crash, request replies).
//! * `20100 + 2(id-1)` / `+1`: CopterSim MAVLink in / out (Python `PX4MavCtrler`).
//! * `30100 + 2(id-1)` / `+1`: CopterSim true-state in / out.
//! * `9999 + SeqID`: default image return port for vision sensors.

pub const UE4_BASE_PORT: u16 = 20010;
pub const UE4_MCAST_GROUP: [u8; 4] = [224, 0, 0, 10];
pub const UE4_MCAST_PORT: u16 = 20009;
pub const UE4_REPLY_PORT: u16 = 20006;
pub const START_SIGNAL_PORT: u16 = 20007;
pub const MAV_BASE_PORT: u16 = 20100;
pub const TRUE_BASE_PORT: u16 = 30100;

pub const CK_0: i32 = 1234567890;
pub const CK_1: i32 = 1234567891;
pub const CK_ATTACH: i32 = 1234567892;
pub const CK_IMG_FRAMEID: i32 = 1234567893;
pub const CK_EXT: i32 = 1234567894;
pub const CK_NETCMD: i32 = 1234567897;
pub const CK_LABEL: i32 = 1234567899;
pub const CK_FULL: i32 = 123456789;
pub const CK_VISION: u16 = 12345;

/// CopterSim MAVLink port for a 1-based CopterID (Python sends here; CopterSim listens).
pub fn mav_port(copter_id: u32) -> u16 {
    MAV_BASE_PORT + 2 * (copter_id.max(1) as u16 - 1)
}
/// CopterSim true-data port for a 1-based CopterID (CopterSim → Python is this + 1).
pub fn true_port(copter_id: u32) -> u16 {
    TRUE_BASE_PORT + 2 * (copter_id.max(1) as u16 - 1)
}

// ---------------------------------------------------------------- byte helpers

struct Rd<'a>(&'a [u8]);
impl Rd<'_> {
    fn i32(&self, o: usize) -> i32 {
        i32::from_le_bytes(self.0[o..o + 4].try_into().unwrap())
    }
    fn u32(&self, o: usize) -> u32 {
        u32::from_le_bytes(self.0[o..o + 4].try_into().unwrap())
    }
    fn u16(&self, o: usize) -> u16 {
        u16::from_le_bytes(self.0[o..o + 2].try_into().unwrap())
    }
    fn f32(&self, o: usize) -> f32 {
        f32::from_le_bytes(self.0[o..o + 4].try_into().unwrap())
    }
    fn f64(&self, o: usize) -> f64 {
        f64::from_le_bytes(self.0[o..o + 8].try_into().unwrap())
    }
    fn f32s<const N: usize>(&self, o: usize) -> [f32; N] {
        std::array::from_fn(|i| self.f32(o + 4 * i))
    }
    fn f64s<const N: usize>(&self, o: usize) -> [f64; N] {
        std::array::from_fn(|i| self.f64(o + 8 * i))
    }
    fn cstr(&self, o: usize, n: usize) -> String {
        let s = &self.0[o..o + n];
        let end = s.iter().position(|&b| b == 0).unwrap_or(n);
        String::from_utf8_lossy(&s[..end]).into_owned()
    }
}

#[derive(Default)]
pub struct Wr(pub Vec<u8>);
impl Wr {
    pub fn i32(&mut self, v: i32) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    pub fn u16(&mut self, v: u16) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    pub fn f32(&mut self, v: f32) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    pub fn f64(&mut self, v: f64) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    pub fn f32s(&mut self, v: &[f32]) -> &mut Self {
        v.iter().for_each(|x| {
            self.f32(*x);
        });
        self
    }
    pub fn f64s(&mut self, v: &[f64]) -> &mut Self {
        v.iter().for_each(|x| {
            self.f64(*x);
        });
        self
    }
    pub fn bytes(&mut self, v: &[u8], n: usize) -> &mut Self {
        let k = v.len().min(n);
        self.0.extend_from_slice(&v[..k]);
        self.0.resize(self.0.len() + (n - k), 0);
        self
    }
    pub fn pad(&mut self, n: usize) -> &mut Self {
        self.0.resize(self.0.len() + n, 0);
        self
    }
}

// ---------------------------------------------------------------- inbound (→ RflySim3D)

/// One vehicle state update, normalised from any of the RflySim position packets.
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleUpdate {
    pub copter_id: i32,
    pub vehicle_type: i32,
    /// NED position relative to the map origin (m).
    pub pos: [f64; 3],
    /// Roll, pitch, yaw (rad).
    pub euler: [f32; 3],
    pub vel: [f32; 3],
    /// Motor speeds (RPM) or PWMs, depending on the sender; used for propeller spin.
    pub motors: [f32; 8],
    pub scale: Option<[f32; 3]>,
    /// Keep the model on the terrain (checksum 1234567891 in the simple formats).
    pub fit_ground: bool,
    pub time: f64,
}

/// A vision sensor request (`VisionSensorReq`, 16H28f / 16H15f / 2H1I14H28f).
#[derive(Clone, Debug, PartialEq)]
pub struct VisionSensorReq {
    pub seq_id: u16,
    pub type_id: u16,
    pub target_copter: u16,
    pub mount_type: u16,
    pub width: u16,
    pub height: u16,
    pub check_freq: u16,
    pub send_protocol: [u16; 8],
    pub fov_deg: f32,
    pub pos: [f32; 3],
    pub use_quat: bool,
    pub euler: [f32; 3],
    pub quat: [f32; 4],
    pub other: [f32; 16],
}

impl VisionSensorReq {
    pub fn dest_ip(&self) -> [u8; 4] {
        let p = &self.send_protocol;
        let ip = [p[1] as u8, p[2] as u8, p[3] as u8, p[4] as u8];
        if ip == [0, 0, 0, 0] {
            [127, 0, 0, 1]
        } else {
            ip
        }
    }
    pub fn dest_port(&self) -> u16 {
        if self.send_protocol[5] == 0 {
            9999 + self.seq_id
        } else {
            self.send_protocol[5]
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct LabelMsg {
    pub copter_id: i32,
    pub disp_flag: i32,
    pub rgb: [i32; 3],
    pub disp_time: f32,
    pub font_size: f32,
    pub text: String,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Ue4In {
    /// Console-style command, e.g. `RflyChangeMapbyName Grasslands`.
    Cmd(String),
    Vehicles(Vec<VehicleUpdate>),
    Vision(VisionSensorReq),
    Label(LabelMsg),
    Attach { copter_ids: Vec<i32>, attach_ids: Vec<i32>, types: Vec<i32> },
    ExtAct { copter_id: i32, time: f64, act: [f64; 16] },
    /// `PX4SILIntFloat` (10i20f): forwarded payload, kept for completeness.
    SilIntFloat { copter_id: i32, ints: [i32; 8], floats: [f32; 20] },
}

/// Decodes one datagram addressed to RflySim3D. Returns `None` for unknown packets.
pub fn decode_ue4(buf: &[u8]) -> Option<Ue4In> {
    let r = Rd(buf);
    let n = buf.len();
    if n < 4 {
        return None;
    }
    let ck = r.i32(0);
    let ck16 = r.u16(0);
    match n {
        56 | 256 | 96 if ck == CK_0 || ck == CK_NETCMD => Some(Ue4In::Cmd(r.cstr(4, n - 4))),
        40 | 52 if ck == CK_0 || ck == CK_1 => {
            let scale = (n == 52).then(|| r.f32s::<3>(40));
            Some(Ue4In::Vehicles(vec![VehicleUpdate {
                copter_id: r.i32(4),
                vehicle_type: r.i32(8),
                motors: [r.f32(12); 8],
                pos: r.f32s::<3>(16).map(f64::from),
                euler: r.f32s::<3>(28),
                vel: [0.0; 3],
                scale,
                fit_ground: ck == CK_1,
                time: -1.0,
            }]))
        }
        // SOut2SimulatorSimpleTime (old): 3i17f1d
        88 if ck == CK_0 || ck == CK_1 => Some(Ue4In::Vehicles(vec![VehicleUpdate {
            copter_id: r.i32(4),
            vehicle_type: r.i32(8),
            motors: r.f32s::<8>(12),
            pos: r.f32s::<3>(44).map(f64::from),
            vel: r.f32s::<3>(56),
            euler: r.f32s::<3>(68),
            scale: None,
            fit_ground: false,
            time: r.f64(80),
        }])),
        // SOut2SimulatorSimpleTime (new): 6i14f4d
        112 if ck == CK_1 || ck == CK_0 => Some(Ue4In::Vehicles(vec![VehicleUpdate {
            copter_id: r.i32(4),
            vehicle_type: r.i32(8),
            motors: r.f32s::<8>(24),
            vel: r.f32s::<3>(56),
            euler: r.f32s::<3>(68),
            pos: r.f64s::<3>(80),
            scale: None,
            fit_ground: false,
            time: r.f64(104),
        }])),
        // SOut2Simulator: 4i24f7d
        168 if ck == CK_FULL => Some(Ue4In::Vehicles(vec![VehicleUpdate {
            copter_id: r.i32(4),
            vehicle_type: r.i32(8),
            vel: r.f32s::<3>(16),
            euler: r.f32s::<3>(28),
            motors: r.f32s::<8>(56),
            time: r.f64(112),
            pos: r.f64s::<3>(120),
            scale: None,
            fit_ground: false,
        }])),
        152 if ck == CK_LABEL => Some(Ue4In::Label(LabelMsg {
            copter_id: r.i32(4),
            disp_flag: r.i32(8),
            rgb: [r.i32(12), r.i32(16), r.i32(20)],
            disp_time: r.f32(24),
            font_size: r.f32(28),
            text: r.cstr(32, 120),
        })),
        304 if ck == CK_ATTACH => {
            let v = |o: usize| (0..25).map(|i| r.i32(o + 4 * i)).collect::<Vec<_>>();
            Some(Ue4In::Attach { copter_ids: v(4), attach_ids: v(104), types: v(204) })
        }
        144 if ck == CK_EXT => {
            Some(Ue4In::ExtAct { copter_id: r.i32(4), time: r.f64(8), act: r.f64s::<16>(16) })
        }
        144 | 92 if ck16 == CK_VISION => {
            let h = |i: usize| r.u16(2 * i);
            let mut v = VisionSensorReq {
                seq_id: h(1),
                type_id: h(2),
                target_copter: h(3),
                mount_type: h(4),
                width: h(5),
                height: h(6),
                check_freq: h(7),
                send_protocol: std::array::from_fn(|i| h(8 + i)),
                fov_deg: r.f32(32),
                pos: r.f32s::<3>(36),
                use_quat: false,
                euler: [0.0; 3],
                quat: [1.0, 0.0, 0.0, 0.0],
                other: [0.0; 16],
            };
            if n == 144 {
                v.use_quat = r.f32(48) > 0.5;
                v.euler = r.f32s::<3>(52);
                v.quat = r.f32s::<4>(64);
                v.other = r.f32s::<16>(80);
            } else {
                v.euler = r.f32s::<3>(48);
                v.other[..8].copy_from_slice(&r.f32s::<8>(60));
            }
            Some(Ue4In::Vision(v))
        }
        148 if ck16 == CK_VISION => {
            // 2H1I14H28f: checksum, SeqID, bitmask(u32), then TypeID.. as above.
            let h = |i: usize| r.u16(8 + 2 * i);
            let _bitmask = r.u32(4);
            Some(Ue4In::Vision(VisionSensorReq {
                seq_id: r.u16(2),
                type_id: h(0),
                target_copter: h(1),
                mount_type: h(2),
                width: h(3),
                height: h(4),
                check_freq: h(5),
                send_protocol: std::array::from_fn(|i| h(6 + i)),
                fov_deg: r.f32(36),
                pos: r.f32s::<3>(40),
                use_quat: r.f32(52) > 0.5,
                euler: r.f32s::<3>(56),
                quat: r.f32s::<4>(68),
                other: r.f32s::<16>(84),
            }))
        }
        120 if ck == CK_NETCMD => Some(Ue4In::SilIntFloat {
            copter_id: r.i32(4),
            ints: std::array::from_fn(|i| r.i32(8 + 4 * i)),
            floats: r.f32s::<20>(40),
        }),
        // 100 vehicles at once: i200H600f300H100f
        3804 if ck == CK_0 || ck == CK_1 => {
            let mut out = Vec::new();
            for k in 0..100 {
                let id = r.u16(4 + 2 * k) as i32;
                if id == 0 {
                    continue;
                }
                let f = |i: usize| r.f32(404 + 4 * i);
                out.push(VehicleUpdate {
                    copter_id: id,
                    vehicle_type: r.u16(204 + 2 * k) as i32,
                    pos: [f(3 * k) as f64, f(3 * k + 1) as f64, f(3 * k + 2) as f64],
                    euler: [f(300 + 3 * k), f(300 + 3 * k + 1), f(300 + 3 * k + 2)],
                    // Scale is u16 ×100 in this format.
                    scale: Some(std::array::from_fn(|i| r.u16(2804 + 2 * (3 * k + i)) as f32 / 100.0))
                        .filter(|s: &[f32; 3]| s.iter().any(|v| *v > 0.0)),
                    motors: [r.f32(3404 + 4 * k); 8],
                    vel: [0.0; 3],
                    fit_ground: ck == CK_1,
                    time: -1.0,
                });
            }
            Some(Ue4In::Vehicles(out))
        }
        // 20 vehicles with PWMs: i40H120f60H160f
        1324 if ck == CK_0 || ck == CK_1 => {
            let mut out = Vec::new();
            for k in 0..20 {
                let id = r.u16(4 + 2 * k) as i32;
                if id == 0 {
                    continue;
                }
                let f = |i: usize| r.f32(84 + 4 * i);
                out.push(VehicleUpdate {
                    copter_id: id,
                    vehicle_type: r.u16(44 + 2 * k) as i32,
                    pos: [f(3 * k) as f64, f(3 * k + 1) as f64, f(3 * k + 2) as f64],
                    euler: [f(60 + 3 * k), f(60 + 3 * k + 1), f(60 + 3 * k + 2)],
                    scale: Some(std::array::from_fn(|i| r.u16(564 + 2 * (3 * k + i)) as f32 / 100.0))
                        .filter(|s: &[f32; 3]| s.iter().any(|v| *v > 0.0)),
                    motors: std::array::from_fn(|i| r.f32(684 + 4 * (8 * k + i))),
                    vel: [0.0; 3],
                    fit_ground: ck == CK_1,
                    time: -1.0,
                });
            }
            Some(Ue4In::Vehicles(out))
        }
        _ => None,
    }
}

// ---------------------------------------------------------------- encoders

/// `Ue4CMD` (i52s or i252s).
pub fn encode_cmd(cmd: &str) -> Vec<u8> {
    let b = cmd.as_bytes();
    let n = if b.len() <= 51 { 52 } else { 252 };
    let mut w = Wr::default();
    w.i32(CK_0).bytes(b, n);
    w.0
}

/// `SOut2SimulatorSimpleTime` new form (6i14f4d, 112 B, checksum 1234567891).
/// This is what CopterSim streams to RflySim3D.
pub fn encode_vehicle_simple_time(v: &VehicleUpdate, gps: [f64; 3]) -> Vec<u8> {
    let mut w = Wr::default();
    w.i32(CK_1).i32(v.copter_id).i32(v.vehicle_type);
    w.i32((gps[0] * 1e7) as i32).i32((gps[1] * 1e7) as i32).i32((gps[2] * 1e3) as i32);
    w.f32s(&v.motors).f32s(&v.vel).f32s(&v.euler).f64s(&v.pos).f64(v.time);
    debug_assert_eq!(w.0.len(), 112);
    w.0
}

/// Full vehicle state, `SOut2Simulator` (4i24f7d, 168 B, checksum 123456789).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FullState {
    pub copter_id: i32,
    pub vehicle_type: i32,
    pub vel: [f32; 3],
    pub euler: [f32; 3],
    pub quat: [f32; 4],
    pub motor_rpm: [f32; 8],
    pub acc_b: [f32; 3],
    pub rate_b: [f32; 3],
    pub time: f64,
    pub pos: [f64; 3],
    /// lat (deg), lon (deg), alt (m)
    pub gps: [f64; 3],
}

pub fn encode_full_state(s: &FullState) -> Vec<u8> {
    let mut w = Wr::default();
    w.i32(CK_FULL).i32(s.copter_id).i32(s.vehicle_type).i32(0);
    w.f32s(&s.vel).f32s(&s.euler).f32s(&s.quat).f32s(&s.motor_rpm).f32s(&s.acc_b).f32s(&s.rate_b);
    w.f64(s.time).f64s(&s.pos).f64s(&s.gps);
    debug_assert_eq!(w.0.len(), 168);
    w.0
}

pub fn decode_full_state(buf: &[u8]) -> Option<FullState> {
    if buf.len() != 168 {
        return None;
    }
    let r = Rd(buf);
    (r.i32(0) == CK_FULL).then(|| FullState {
        copter_id: r.i32(4),
        vehicle_type: r.i32(8),
        vel: r.f32s(16),
        euler: r.f32s(28),
        quat: r.f32s(40),
        motor_rpm: r.f32s(56),
        acc_b: r.f32s(88),
        rate_b: r.f32s(100),
        time: r.f64(112),
        pos: r.f64s(120),
        gps: r.f64s(144),
    })
}

/// Crash notice (3i): `CopterSimCrash {checksum, CopterID, TargetID}`.
/// TargetID: other vehicle's id, or -2 ground / -1 static scene object.
pub fn encode_crash(copter_id: i32, target_id: i32) -> Vec<u8> {
    let mut w = Wr::default();
    w.i32(CK_0).i32(copter_id).i32(target_id);
    w.0
}

/// `CoptReqData` (2i16f1d, 80 B): reply to `RflyReqObjData 1 <id>`.
pub fn encode_copt_req(
    copter_id: i32,
    pos_ue: [f32; 3],
    euler: [f32; 3],
    quat: [f32; 4],
    box_origin: [f32; 3],
    box_extent: [f32; 3],
    t: f64,
) -> Vec<u8> {
    let mut w = Wr::default();
    w.i32(CK_1).i32(copter_id);
    w.f32s(&pos_ue).f32s(&euler).f32s(&quat).f32s(&box_origin).f32s(&box_extent).f64(t);
    debug_assert_eq!(w.0.len(), 80);
    w.0
}

/// `CameraData` (5i11f1d, 72 B): reply to `RflyReqObjData 0 <seq>`.
pub fn encode_camera_data(
    seq_id: i32,
    type_id: i32,
    h: i32,
    wdt: i32,
    fov: f32,
    pos: [f32; 3],
    euler: [f32; 3],
    quat: [f32; 4],
    t: f64,
) -> Vec<u8> {
    let mut w = Wr::default();
    w.i32(CK_1).i32(seq_id).i32(type_id).i32(h).i32(wdt);
    w.f32(fov).f32s(&pos).f32s(&euler).f32s(&quat).f64(t);
    debug_assert_eq!(w.0.len(), 72);
    w.0
}

/// GPS of the map origin (`Ue4CMDGPS`, ii3d).
pub fn encode_map_gps(copter_id: i32, lat: f64, lon: f64, alt: f64) -> Vec<u8> {
    let mut w = Wr::default();
    w.i32(CK_0).i32(copter_id).f64(lat).f64(lon).f64(alt);
    w.0
}

/// Start signal (3i to 224.0.0.10:20007).
pub fn decode_start_signal(buf: &[u8]) -> Option<i32> {
    (buf.len() == 12 && Rd(buf).i32(0) == CK_0 && Rd(buf).i32(4) != 0).then(|| Rd(buf).i32(8))
}

// ---------------------------------------------------------------- images

/// Largest image payload per UDP packet (matches RflySimSDK's `imgPackUnit`).
pub const IMG_PACK_UNIT: usize = 60000;

/// Splits an encoded image into RflySim image packets: header `4i1d`
/// `{checksum 1234567890, packet length incl. header, packet seq, packet count, timestamp}`.
pub fn image_packets(data: &[u8], timestamp: f64) -> Vec<Vec<u8>> {
    let chunks: Vec<&[u8]> = if data.is_empty() { vec![&[]] } else { data.chunks(IMG_PACK_UNIT).collect() };
    let total = chunks.len() as i32;
    chunks
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let mut w = Wr::default();
            w.i32(CK_0).i32((24 + c.len()) as i32).i32(i as i32).i32(total).f64(timestamp);
            w.0.extend_from_slice(c);
            w.0
        })
        .collect()
}

/// Reassembles packets from [`image_packets`]. Returns `(timestamp, data)` when complete.
#[derive(Default)]
pub struct ImageAssembler {
    parts: Vec<Option<Vec<u8>>>,
    stamp: f64,
}

impl ImageAssembler {
    pub fn push(&mut self, pkt: &[u8]) -> Option<(f64, Vec<u8>)> {
        if pkt.len() < 24 {
            return None;
        }
        let r = Rd(pkt);
        if r.i32(0) != CK_0 || r.i32(4) as usize != pkt.len() {
            return None;
        }
        let (seq, total, t) = (r.i32(8) as usize, r.i32(12) as usize, r.f64(16));
        if self.parts.len() != total || t != self.stamp {
            self.parts = vec![None; total];
            self.stamp = t;
        }
        if seq < total {
            self.parts[seq] = Some(pkt[24..].to_vec());
        }
        if self.parts.iter().all(Option::is_some) {
            let data = self.parts.drain(..).flatten().flatten().collect();
            return Some((t, data));
        }
        None
    }
}

/// Splits a console command into its name and whitespace-separated args.
pub fn split_cmd(cmd: &str) -> (String, Vec<String>) {
    let mut it = cmd.split_whitespace();
    let name = it.next().unwrap_or("").to_string();
    (name, it.map(str::to_string).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vu() -> VehicleUpdate {
        VehicleUpdate {
            copter_id: 3,
            vehicle_type: 3,
            pos: [1.5, -2.0, -10.25],
            euler: [0.1, -0.2, 1.0],
            vel: [1.0, 2.0, 3.0],
            motors: [1000.0; 8],
            scale: None,
            fit_ground: false,
            time: 12.5,
        }
    }

    #[test]
    fn simple_time_roundtrip() {
        let b = encode_vehicle_simple_time(&vu(), [47.0, 8.0, 500.0]);
        assert_eq!(b.len(), 112);
        assert_eq!(decode_ue4(&b), Some(Ue4In::Vehicles(vec![vu()])));
    }

    #[test]
    fn full_state_roundtrip() {
        let s = FullState { copter_id: 2, pos: [1.0, 2.0, -3.0], time: 4.0, ..Default::default() };
        let b = encode_full_state(&s);
        assert_eq!(decode_full_state(&b), Some(s));
        match decode_ue4(&b) {
            Some(Ue4In::Vehicles(v)) => assert_eq!(v[0].pos, [1.0, 2.0, -3.0]),
            o => panic!("{o:?}"),
        }
    }

    #[test]
    fn cmd_roundtrip_short_and_long() {
        assert_eq!(decode_ue4(&encode_cmd("RflyChangeMapbyName Grasslands")), Some(Ue4In::Cmd("RflyChangeMapbyName Grasslands".into())));
        let long = "RflyShowTextTime ".to_string() + &"x".repeat(100) + " 5";
        let b = encode_cmd(&long);
        assert_eq!(b.len(), 256);
        assert_eq!(decode_ue4(&b), Some(Ue4In::Cmd(long)));
    }

    #[test]
    fn simple_pos_3i7f() {
        // Same bytes Python's struct.pack("3i7f", 1234567891, 1, 3, 500, 1,2,-3, 0,0,0.5) makes.
        let mut w = Wr::default();
        w.i32(CK_1).i32(1).i32(3).f32s(&[500.0, 1.0, 2.0, -3.0, 0.0, 0.0, 0.5]);
        match decode_ue4(&w.0) {
            Some(Ue4In::Vehicles(v)) => {
                assert!(v[0].fit_ground);
                assert_eq!(v[0].pos, [1.0, 2.0, -3.0]);
                assert_eq!(v[0].euler[2], 0.5);
            }
            o => panic!("{o:?}"),
        }
    }

    #[test]
    fn vision_req_16h28f() {
        let mut w = Wr::default();
        for h in [12345u16, 0, 1, 1, 0, 640, 480, 30, 1, 127, 0, 0, 1, 9999, 0, 0] {
            w.u16(h);
        }
        w.f32(90.0).f32s(&[0.3, 0.0, 0.0]).f32(0.0).f32s(&[0.0, -0.5, 0.0]).f32s(&[0.0; 4]).f32s(&[0.0; 16]);
        assert_eq!(w.0.len(), 144);
        match decode_ue4(&w.0) {
            Some(Ue4In::Vision(v)) => {
                assert_eq!((v.width, v.height, v.type_id), (640, 480, 1));
                assert_eq!(v.dest_ip(), [127, 0, 0, 1]);
                assert_eq!(v.dest_port(), 9999);
                assert_eq!(v.euler[1], -0.5);
            }
            o => panic!("{o:?}"),
        }
    }

    #[test]
    fn image_packets_reassemble() {
        let data: Vec<u8> = (0..150_000u32).map(|i| (i % 251) as u8).collect();
        let pk = image_packets(&data, 3.25);
        assert_eq!(pk.len(), 3);
        let mut a = ImageAssembler::default();
        assert!(a.push(&pk[0]).is_none());
        assert!(a.push(&pk[2]).is_none());
        let (t, out) = a.push(&pk[1]).unwrap();
        assert_eq!(t, 3.25);
        assert_eq!(out, data);
    }

    #[test]
    fn ports() {
        assert_eq!(mav_port(1), 20100);
        assert_eq!(mav_port(3), 20104);
        assert_eq!(true_port(2), 30102);
    }
}
