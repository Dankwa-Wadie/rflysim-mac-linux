//! coptersim — headless multicopter simulator for PX4 (SITL over TCP, HIL over serial).
//!
//! Replaces the simulation/MAVLink part of RflySim's Windows CopterSim:
//! physics at 1 kHz, HIL_SENSOR at 250 Hz, HIL_GPS at 10 Hz, motor commands from
//! HIL_ACTUATOR_CONTROLS.

use clap::Parser;
use mavlink::dialects::common::{
    HilSensorUpdatedFlags, MavAutopilot, MavMessage, MavModeFlag, MavState, MavType,
    HEARTBEAT_DATA, HIL_GPS_DATA, HIL_SENSOR_DATA,
};
use mavlink::{MavConnection, MavHeader, MavlinkVersion};
use rfly_core::{Home, Params, SensorModel, Vehicle};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[command(version, about)]
struct Args {
    /// MAVLink link to the autopilot. PX4 SITL connects to TCP 4560 by default;
    /// for HIL with a Pixhawk use e.g. `serial:/dev/cu.usbmodem01:921600` (macOS) or `serial:/dev/ttyACM0:921600` (Linux).
    /// Default: `tcpin:0.0.0.0:<4560 + id - 1>` (PX4 SITL instance id-1).
    #[arg(long)]
    link: Option<String>,
    /// RflySim CopterID (1-based). Selects the PX4 TCP port and the RflySim UDP ports.
    #[arg(long, default_value_t = 1)]
    id: u32,
    /// RflySim3D vehicle type (3 = quadcopter).
    #[arg(long, default_value_t = 3)]
    vehicle_type: i32,
    /// Initial NED position offset (m) and yaw (deg), like CopterSim's initial X/Y/Yaw boxes.
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    init_x: f64,
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    init_y: f64,
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    init_yaw: f64,
    /// Host running RflySim3D (vehicle state goes to UDP 20010.. on this host).
    #[arg(long, default_value = "127.0.0.1")]
    ue4_ip: String,
    /// Number of RflySim3D windows to feed (20010, 20011, ...).
    #[arg(long, default_value_t = 3)]
    ue4_windows: u16,
    /// Relay autopilot MAVLink to/from this address (repeatable), e.g. `udpout:127.0.0.1:14550`
    /// for QGroundControl. Default for serial (HIL) links: QGC on 14550 plus the RflySim
    /// Python ports (PX4 → 20101+2(id-1), Python → 20100+2(id-1)), like Windows CopterSim.
    #[arg(long)]
    forward: Vec<String>,
    /// Disable MAVLink relaying (also the HIL defaults).
    #[arg(long)]
    no_forward: bool,
    /// Disable the RflySim3D / true-data UDP outputs.
    #[arg(long)]
    no_rfly: bool,
    /// Also drive FlightGear (native-fdm over UDP), e.g. `127.0.0.1:5502`.
    #[arg(long, value_name = "HOST:PORT", num_args = 0..=1, default_missing_value = "127.0.0.1:5502")]
    fg: Option<String>,
    /// Path to RflySim's ModelData.db. Default: $RFLY_DB, or the copy imported from your RflySim ISO
    /// (scripts/import_rflysim_data.sh); built-in F450 defaults when neither exists.
    #[arg(long)]
    db: Option<String>,
    /// Parameter set name inside the DB.
    #[arg(long, default_value = "default")]
    params: String,
    /// Home latitude / longitude (deg) and altitude (m AMSL).
    #[arg(long, default_value_t = 47.397742)]
    lat: f64,
    #[arg(long, default_value_t = 8.545594)]
    lon: f64,
    #[arg(long, default_value_t = 488.0)]
    alt: f64,
    /// Stop motors if no actuator message arrives for this long (ms).
    #[arg(long, default_value_t = 500)]
    actuator_timeout_ms: u64,
    /// External-controller lockstep mode (MATLAB/Simulink, Python, C) on this UDP port
    /// instead of PX4. RflySim's CopterSim used 20100 as its SIL port.
    #[arg(long, value_name = "PORT", num_args = 0..=1, default_missing_value = "20100")]
    ext: Option<u16>,
}

mod ext;
mod fg_out;
mod rfly_out;

#[derive(Default)]
struct Actuators {
    controls: [f32; 16],
    armed: bool,
    last: Option<Instant>,
    count: u64,
}

type Conn = Arc<dyn MavConnection<MavMessage> + Sync + Send>;

const PHYSICS_DT: f64 = 0.001;
const SENSOR_EVERY: u64 = 4; // 250 Hz
const GPS_EVERY: u64 = 100; // 10 Hz
const HEARTBEAT_EVERY: u64 = 1000; // 1 Hz

fn main() {
    let args = Args::parse();

    let params = load_params(&args);
    println!(
        "coptersim: mass {:.2} kg, arm {:.3} m, hover throttle {:.3}",
        params.mass,
        params.arm,
        params.hover_throttle()
    );

    let home = Home { lat_deg: args.lat, lon_deg: args.lon, alt_m: args.alt, ..Home::default() };
    let rfly = if args.no_rfly && args.fg.is_none() {
        None
    } else {
        let windows = if args.no_rfly { 0 } else { args.ue4_windows };
        match rfly_out::RflyOut::new(args.id, args.vehicle_type, &args.ue4_ip, windows, home)
            .and_then(|mut o| {
                if let Some(fg) = &args.fg {
                    o.fg = Some(fg_out::FgOut::new(fg, home)?);
                }
                Ok(o)
            }) {
            Ok(o) => Some(o),
            Err(e) => {
                eprintln!("coptersim: RflySim outputs disabled: {e}");
                None
            }
        }
    };
    let init = |v: &mut Vehicle| {
        v.s.pos.x = args.init_x;
        v.s.pos.y = args.init_y;
        v.s.att = rfly_core::Quat::from_euler(0.0, 0.0, args.init_yaw.to_radians());
    };
    if let Some(port) = args.ext {
        if let Err(e) = ext::run(port, params, home, rfly, &init) {
            eprintln!("coptersim: external mode failed: {e}");
            std::process::exit(1);
        }
        return;
    }
    let mut sensors = SensorModel::new(home);
    let mut vehicle = Vehicle::new(params);
    init(&mut vehicle);
    let mut rfly = rfly;
    let link = args.link.clone().unwrap_or_else(|| format!("tcpin:0.0.0.0:{}", 4560 + args.id - 1));

    if link.starts_with("tcpin:") {
        println!("coptersim: awaiting PX4 SITL to connect on {} ...", &link[6..]);
    }
    let mut conn = match open_link(&link) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("coptersim: failed to open {}: {e}", link);
            std::process::exit(1);
        }
    };
    conn.set_protocol_version(MavlinkVersion::V2);
    let conn = Arc::new(conn);
    println!("coptersim: link open ({link})");

    // MAVLink relays (HIL: the Pixhawk's USB port is ours, so QGC/Python go through us).
    let mut relays: Vec<(String, bool)> = args.forward.iter().map(|f| (f.clone(), true)).collect();
    if relays.is_empty() && !args.no_forward && link.starts_with("serial:") {
        relays.push(("udpout:127.0.0.1:14550".into(), true));
        relays.push((format!("udpout:127.0.0.1:{}", rfly_proto::mav_port(args.id) + 1), false));
        relays.push((format!("udpin:0.0.0.0:{}", rfly_proto::mav_port(args.id)), true));
    }
    let mut outs: Vec<Conn> = Vec::new();
    for (addr, back) in relays.iter().filter(|_| !args.no_forward) {
        match mavlink::connect::<MavMessage>(addr) {
            Ok(mut c) => {
                c.set_protocol_version(MavlinkVersion::V2);
                let c: Conn = Arc::new(c);
                println!("coptersim: relaying MAVLink {}{addr}", if *back { "<-> " } else { "-> " });
                if *back {
                    let (c2, ap) = (c.clone(), conn.clone());
                    std::thread::spawn(move || loop {
                        if let Ok((h, m)) = c2.recv() {
                            let _ = ap.send(&h, &m);
                        }
                    });
                }
                if !addr.starts_with("udpin:") {
                    outs.push(c);
                }
            }
            Err(e) => eprintln!("coptersim: cannot relay to {addr}: {e}"),
        }
    }

    // Receiver thread: collect actuator commands, report PX4 status text, feed relays.
    let act = Arc::new(Mutex::new(Actuators::default()));
    {
        let (conn, act) = (conn.clone(), act.clone());
        std::thread::spawn(move || loop {
            let r = conn.recv();
            if let Ok((h, m)) = &r {
                for o in &outs {
                    let _ = o.send(h, m);
                }
            }
            match r {
                Ok((_, MavMessage::HIL_ACTUATOR_CONTROLS(m))) => {
                    let mut a = act.lock().unwrap();
                    a.controls = m.controls;
                    a.armed = m.mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
                    a.last = Some(Instant::now());
                    a.count += 1;
                }
                Ok((_, MavMessage::STATUSTEXT(m))) => {
                    println!("PX4: {}", m.text.to_str().unwrap_or("?"));
                }
                Ok(_) => {}
                Err(mavlink::error::MessageReadError::Io(e))
                    if e.kind() == std::io::ErrorKind::UnexpectedEof =>
                {
                    eprintln!("coptersim: autopilot disconnected");
                    std::process::exit(0);
                }
                Err(_) => {}
            }
        });
    }

    let header = MavHeader { system_id: 1, component_id: 51, sequence: 0 };
    let send = |m: &MavMessage| {
        if let Err(e) = conn.send(&header, m) {
            eprintln!("coptersim: send failed: {e}");
            std::process::exit(1);
        }
    };

    let timeout = Duration::from_millis(args.actuator_timeout_ms);
    let start = Instant::now();
    let mut tick: u64 = 0;
    let mut was_armed = false;
    loop {
        // Motor commands (PX4 sends normalised 0..1 for rotors).
        let (u, armed) = {
            let a = act.lock().unwrap();
            let fresh = a.last.is_some_and(|t| t.elapsed() < timeout);
            let mut u = [0.0f64; 8];
            if a.armed && fresh {
                for (dst, src) in u.iter_mut().zip(a.controls.iter()) {
                    *dst = (*src as f64).clamp(0.0, 1.0);
                }
            }
            (u, a.armed)
        };
        if armed != was_armed {
            println!("coptersim: PX4 {}", if armed { "armed" } else { "disarmed" });
            was_armed = armed;
        }

        vehicle.step(&u, PHYSICS_DT);
        tick += 1;
        let time_usec = tick * 1000;
        if let Some(o) = rfly.as_mut() {
            o.publish(time_usec as f64 * 1e-6, &vehicle);
        }

        if tick % SENSOR_EVERY == 0 {
            let imu = sensors.imu(&vehicle);
            send(&MavMessage::HIL_SENSOR(HIL_SENSOR_DATA {
                time_usec,
                xacc: imu.acc.x as f32,
                yacc: imu.acc.y as f32,
                zacc: imu.acc.z as f32,
                xgyro: imu.gyro.x as f32,
                ygyro: imu.gyro.y as f32,
                zgyro: imu.gyro.z as f32,
                xmag: imu.mag.x as f32,
                ymag: imu.mag.y as f32,
                zmag: imu.mag.z as f32,
                abs_pressure: imu.abs_pressure as f32,
                diff_pressure: 0.0,
                pressure_alt: imu.pressure_alt as f32,
                temperature: imu.temperature as f32,
                fields_updated: HilSensorUpdatedFlags::from_bits_truncate(0x1fff),
                ..Default::default()
            }));
        }
        if tick % GPS_EVERY == 0 {
            let g = sensors.gps(&vehicle);
            let v = g.vel_ned;
            let ground_speed = (v.x * v.x + v.y * v.y).sqrt();
            let cog = v.y.atan2(v.x).to_degrees().rem_euclid(360.0);
            send(&MavMessage::HIL_GPS(HIL_GPS_DATA {
                time_usec,
                lat: (g.lat_deg * 1e7) as i32,
                lon: (g.lon_deg * 1e7) as i32,
                alt: (g.alt_m * 1000.0) as i32,
                eph: 30,
                epv: 40,
                vel: (ground_speed * 100.0) as u16,
                vn: (v.x * 100.0) as i16,
                ve: (v.y * 100.0) as i16,
                vd: (v.z * 100.0) as i16,
                cog: (cog * 100.0) as u16,
                fix_type: 3,
                satellites_visible: 10,
                ..Default::default()
            }));
        }
        if tick % HEARTBEAT_EVERY == 0 {
            send(&MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: MavType::MAV_TYPE_GENERIC,
                autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
                base_mode: MavModeFlag::empty(),
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 3,
            }));
            let s = &vehicle.s;
            let (r, p, y) = s.att.to_euler();
            let n = act.lock().unwrap().count;
            println!(
                "t={:6.1}s  NED=({:7.2},{:7.2},{:7.2})  rpy=({:6.1},{:6.1},{:6.1})°  u={:.2}/{:.2}/{:.2}/{:.2}  act_msgs={n}",
                time_usec as f64 * 1e-6,
                s.pos.x, s.pos.y, s.pos.z,
                r.to_degrees(), p.to_degrees(), y.to_degrees(),
                u[0], u[1], u[2], u[3],
            );
        }

        // Real-time pacing.
        let target = start + Duration::from_micros(time_usec);
        if let Some(wait) = target.checked_duration_since(Instant::now()) {
            std::thread::sleep(wait);
        }
    }
}

/// Opens the autopilot link. TCP links get TCP_NODELAY: each HIL_SENSOR is a small packet, and with
/// Nagle's algorithm a packet waits for the previous one's ACK. Linux delays loopback ACKs (up to
/// 40 ms), so sensor data reached PX4 in clumps, PX4 dropped IMU samples, and its EKF reported
/// "High Accelerometer Bias" and refused to arm. macOS happened not to show it.
fn open_link(link: &str) -> std::io::Result<mavlink::Connection<MavMessage>> {
    use mavlink::{Connectable, TcpConfig};
    use std::net::{TcpListener, TcpStream};
    let stream = if let Some(addr) = link.strip_prefix("tcpin:") {
        Some(TcpListener::bind(addr)?.accept()?.0)
    } else if let Some(addr) = link.strip_prefix("tcpout:") {
        Some(TcpStream::connect(addr)?)
    } else {
        None
    };
    match stream {
        Some(s) => {
            s.set_nodelay(true)?;
            TcpConfig::from_stream(s)?.connect::<MavMessage>()
        }
        None => mavlink::connect::<MavMessage>(link),
    }
}

fn load_params(args: &Args) -> Params {
    let found = args.db.clone().or_else(|| rfly_db::find_model_db(&[]).map(|p| p.display().to_string()));
    let Some(path) = &found else { return Params::default() };
    match rfly_db::ModelDb::open(path).and_then(|db| db.params(&args.params)) {
        Ok(Some(p)) => {
            println!("coptersim: loaded '{}' from {path}", args.params);
            p
        }
        Ok(None) => {
            eprintln!("coptersim: no valid param set '{}' in {path}; using defaults", args.params);
            Params::default()
        }
        Err(e) => {
            eprintln!("coptersim: cannot read {path}: {e}; using defaults");
            Params::default()
        }
    }
}
