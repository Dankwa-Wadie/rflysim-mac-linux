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
    /// RflySim3D vehicle type (3D model): default from the frame (3 quad, 5 hexa, 8 octo).
    #[arg(long)]
    vehicle_type: Option<i32>,
    /// Frame (overrides the parameter file): quad_x, quad_plus, hexa_x, octo_x, y6.
    #[arg(long, value_parser = |s: &str| rfly_core::Frame::parse(s).ok_or_else(|| format!("unknown frame {s}")))]
    frame: Option<rfly_core::Frame>,
    /// Print the PX4 parameters (control allocation, HIL outputs) for this vehicle's frame and exit.
    #[arg(long)]
    print_px4_params: bool,
    /// Print the ArduPilot parameters (FRAME_CLASS, FRAME_TYPE) for this vehicle's frame and exit.
    #[arg(long)]
    print_ardupilot_params: bool,
    /// Fixed-wing aircraft (RflySim SITLRunFw/HITLRunFw): PX4's Gazebo-Classic plane (PX4 airframe 1030,
    /// PX4_SIM_MODEL=gazebo-classic_plane) instead of a multicopter.
    #[arg(long)]
    fixed_wing: bool,
    /// Initial NED position offset (m) and yaw (deg), like CopterSim's initial X/Y/Yaw boxes.
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    init_x: f64,
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    init_y: f64,
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    init_yaw: f64,
    /// Where RflySim3D runs (vehicle state goes to UDP 20010..), as RflySim's IS_BROADCAST: `0` this computer,
    /// `1` broadcast to the LAN (LANSwarm), an IP, or a comma-separated list of IPs.
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
    /// ArduPilot SITL instead of PX4: serve ArduPilot's JSON physics interface on this UDP port
    /// (ArduPilot started with `--model JSON`; 9002 + 10 × instance). Copter (quad X/+, hexa X) or,
    /// with --fixed-wing, ArduPlane.
    #[arg(long, value_name = "PORT", num_args = 0..=1, default_missing_value = "9002")]
    ardupilot: Option<u16>,
    /// Vehicle parameters from a file (written by the CopterSim window's configurator; `name = value`
    /// lines, the 14 ModelData.db modelInfo values). Takes precedence over --db/--params.
    #[arg(long, value_name = "FILE")]
    params_file: Option<String>,
    /// Send a 10 Hz status line to 127.0.0.1:PORT (the CopterSim window's live panel):
    /// `RFLYSTAT id t x y z vx vy vz roll pitch yaw u1 u2 u3 u4 armed` (NED m, m/s, deg, throttle 0..1).
    #[arg(long, value_name = "PORT")]
    status_port: Option<u16>,
    /// RflySim communication mode with Python (UDPSIMMODE): 0 UDP_Full, 1 UDP_Simple, 2 Mavlink_Full (default),
    /// 3 Mavlink_Simple, 4 Mavlink_NoSend, 5 Mavlink_NoGPS, 6 Mavlink_Vision. Number or name.
    #[arg(long, value_name = "MODE", default_value = "2", value_parser = parse_udp_mode)]
    udp_mode: u8,
    /// SITL in UDP_Full/UDP_Simple: PX4's MAVLink (onboard) link for telemetry and commands, e.g.
    /// `udpin:0.0.0.0:25100` (SITLRun sets it). HIL uses the serial link instead.
    #[arg(long, value_name = "ADDR")]
    px4_mav: Option<String>,
}

const UDP_MODE_NAMES: [&str; 7] = ["UDP_Full", "UDP_Simple", "Mavlink_Full", "Mavlink_Simple", "Mavlink_NoSend", "Mavlink_NoGPS", "Mavlink_Vision"];

fn parse_udp_mode(s: &str) -> Result<u8, String> {
    if let Ok(n) = s.parse::<u8>() {
        return if (n as usize) < UDP_MODE_NAMES.len() { Ok(n) } else { Err(format!("UDP mode {n} is not 0..6")) };
    }
    UDP_MODE_NAMES.iter().position(|n| n.eq_ignore_ascii_case(s)).map(|i| i as u8).ok_or_else(|| format!("unknown UDP mode {s} ({})", UDP_MODE_NAMES.join(", ")))
}

/// Messages forwarded to Python in Mavlink_Simple mode (enough for RflySimSDK's PX4MavCtrler).
fn simple_mode_msg(m: &MavMessage) -> bool {
    matches!(
        m,
        MavMessage::HEARTBEAT(_) | MavMessage::ATTITUDE(_) | MavMessage::LOCAL_POSITION_NED(_) | MavMessage::GLOBAL_POSITION_INT(_)
            | MavMessage::HOME_POSITION(_) | MavMessage::SYS_STATUS(_) | MavMessage::COMMAND_ACK(_) | MavMessage::STATUSTEXT(_)
            | MavMessage::GPS_RAW_INT(_) | MavMessage::EXTENDED_SYS_STATE(_) | MavMessage::PARAM_VALUE(_)
    )
}

mod ardupilot;
mod ext;
mod fg_out;
mod rfly_out;
mod status;
mod udp_bridge;

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

    let mut params = load_params(&args);
    if let Some(f) = args.frame {
        params.frame = f;
    }
    if args.print_ardupilot_params {
        if args.fixed_wing {
            for (k, v) in ardupilot::PLANE_PARAMS {
                println!("{k} {v}");
            }
        } else {
            match ardupilot::ardupilot_frame_params(params.frame) {
                Some((c, t)) => println!("FRAME_CLASS {c}\nFRAME_TYPE {t}"),
                None => {
                    eprintln!("coptersim: frame {} is not supported with ArduPilot yet", params.frame.name());
                    std::process::exit(2);
                }
            }
        }
        return;
    }
    if args.print_px4_params {
        if args.fixed_wing {
            for (k, v) in rfly_core::fixedwing::FixedWing::px4_params() {
                println!("{k} {v}");
            }
            return;
        }
        for (k, v) in params.frame.px4_params(params.arm) {
            println!("{k} {v}");
        }
        return;
    }
    if args.fixed_wing {
        println!("coptersim: fixed-wing aircraft (PX4 Gazebo-Classic plane model, 1.5 kg)");
    } else {
        println!("coptersim: frame {} ({} rotors)", params.frame.label(), params.frame.rotors().len());
    }
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
        match rfly_out::RflyOut::new(args.id, args.vehicle_type.unwrap_or(if args.fixed_wing { 100 } else { params.frame.rflysim3d_type() }), &args.ue4_ip, windows, home)
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
        let status = args.status_port.and_then(|p| status::StatusOut::new(p, args.id, &params).ok());
        if let Err(e) = ext::run(port, params, home, rfly, status, &init) {
            eprintln!("coptersim: external mode failed: {e}");
            std::process::exit(1);
        }
        return;
    }
    if let Some(port) = args.ardupilot {
        let status = args.status_port.and_then(|p| status::StatusOut::new(p, args.id, &params).ok());
        let mixer = ardupilot::mixer_for(&params, args.fixed_wing).unwrap_or_else(|e| {
            eprintln!("coptersim: {e}");
            std::process::exit(2);
        });
        let vehicle = if args.fixed_wing {
            Vehicle::new_fixed_wing(rfly_core::fixedwing::FixedWing::px4_plane())
        } else {
            Vehicle::new(params)
        };
        if let Err(e) = ardupilot::run(port, vehicle, mixer, rfly, status, &init) {
            eprintln!("coptersim: ArduPilot mode failed: {e}");
            std::process::exit(1);
        }
        return;
    }
    let mut status = args.status_port.and_then(|p| status::StatusOut::new(p, args.id, &params).ok());
    let mut sensors = SensorModel::new(home);
    let fixed_wing = args.fixed_wing;
    let mut vehicle = if fixed_wing { Vehicle::new_fixed_wing(rfly_core::fixedwing::FixedWing::px4_plane()) } else { Vehicle::new(params) };
    init(&mut vehicle);
    let mut rfly = rfly;
    // Cached result of the daily update check (no network access here).
    if let Some(n) = rfly_update::notice() {
        eprintln!("coptersim: {n}");
    }
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
    let udp_mode = args.udp_mode;
    println!("coptersim: communication mode {} ({})", udp_mode, UDP_MODE_NAMES[udp_mode as usize]);
    let mut relays: Vec<(String, bool)> = args.forward.iter().map(|f| (f.clone(), true)).collect();
    if relays.is_empty() && !args.no_forward && link.starts_with("serial:") {
        relays.push(("udpout:127.0.0.1:14550".into(), true));
        // Python's MAVLink ports, except in UDP modes (served by the UDP bridge) and Mavlink_NoSend.
        if udp_mode >= 2 && udp_mode != 4 {
            relays.push((format!("udpout:127.0.0.1:{}", rfly_proto::mav_port(args.id) + 1), false));
            relays.push((format!("udpin:0.0.0.0:{}", rfly_proto::mav_port(args.id)), true));
        }
    }
    let python_port = format!(":{}", rfly_proto::mav_port(args.id) + 1);
    let mut outs: Vec<(Conn, bool)> = Vec::new();
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
                    outs.push((c, addr.ends_with(&python_port)));
                }
            }
            Err(e) => eprintln!("coptersim: cannot relay to {addr}: {e}"),
        }
    }

    // UDP_Full / UDP_Simple: serve Python's RflySim structs, from PX4 telemetry (serial link in HIL, a separate
    // MAVLink link in SITL).
    let tele = Arc::new(Mutex::new(udp_bridge::Telemetry::default()));
    if udp_mode <= 1 {
        let crash = Arc::new(Mutex::new(-1i32));
        let sender: udp_bridge::Sender = if link.starts_with("serial:") {
            let c = conn.clone();
            Arc::new(move |m: &MavMessage| {
                let _ = c.send(&MavHeader { system_id: 255, component_id: 190, sequence: 0 }, m);
            })
        } else {
            let Some(addr) = &args.px4_mav else {
                eprintln!("coptersim: UDP_Full/UDP_Simple in SITL needs --px4-mav (PX4's onboard MAVLink link; SITLRun sets it)");
                std::process::exit(2);
            };
            let mut pc = match mavlink::connect::<MavMessage>(addr) {
                Ok(c) => c,
                Err(e) => {
                    eprintln!("coptersim: cannot open PX4 MAVLink link {addr}: {e}");
                    std::process::exit(2);
                }
            };
            pc.set_protocol_version(MavlinkVersion::V2);
            let pc = Arc::new(pc);
            let (rx, t) = (pc.clone(), tele.clone());
            std::thread::spawn(move || loop {
                if let Ok((_, m)) = rx.recv() {
                    t.lock().unwrap().update(&m);
                }
            });
            Arc::new(move |m: &MavMessage| {
                let _ = pc.send(&MavHeader { system_id: 255, component_id: 190, sequence: 0 }, m);
            })
        };
        if let Err(e) = udp_bridge::spawn(udp_mode, args.id, sender, tele.clone(), crash) {
            eprintln!("coptersim: cannot open the Python port {}: {e}", rfly_proto::mav_port(args.id));
            std::process::exit(2);
        }
    }

    // Receiver thread: collect actuator commands, report PX4 status text, feed relays.
    let act = Arc::new(Mutex::new(Actuators::default()));
    {
        let (conn, act, tele) = (conn.clone(), act.clone(), tele.clone());
        std::thread::spawn(move || loop {
            let r = conn.recv();
            if let Ok((h, m)) = &r {
                for (o, python) in &outs {
                    // Mavlink_Simple: Python gets a reduced message set.
                    if *python && udp_mode == 3 && !simple_mode_msg(m) {
                        continue;
                    }
                    let _ = o.send(h, m);
                }
                if udp_mode <= 1 {
                    tele.lock().unwrap().update(m);
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
            let mut u = [0.0f64; 16];
            if a.armed && fresh {
                // Multicopter: motor throttles 0..1. Fixed wing: raw outputs (servos -1..1, motor 0..1).
                for (dst, src) in u.iter_mut().zip(a.controls.iter()) {
                    *dst = if fixed_wing { (*src as f64).clamp(-1.0, 1.0) } else { (*src as f64).clamp(0.0, 1.0) };
                }
            }
            (u, a.armed)
        };
        if armed != was_armed {
            println!("coptersim: PX4 {}", if armed { "armed" } else { "disarmed" });
            was_armed = armed;
        }

        vehicle.step(&u, PHYSICS_DT);
        if let Some(st) = status.as_mut() {
            st.publish(tick as f64 * PHYSICS_DT, &vehicle, armed);
        }
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
                diff_pressure: imu.diff_pressure as f32,
                pressure_alt: imu.pressure_alt as f32,
                temperature: imu.temperature as f32,
                fields_updated: HilSensorUpdatedFlags::from_bits_truncate(0x1fff),
                ..Default::default()
            }));
        }
        // Mavlink_Vision: true pose as external vision (PX4 fuses it instead of GPS).
        if udp_mode == 6 && tick % 33 == 0 {
            let s = &vehicle.s;
            let mut cov = [0f32; 21];
            cov[0] = f32::NAN;
            send(&MavMessage::ODOMETRY(mavlink::dialects::common::ODOMETRY_DATA {
                time_usec,
                frame_id: mavlink::dialects::common::MavFrame::MAV_FRAME_LOCAL_NED,
                child_frame_id: mavlink::dialects::common::MavFrame::MAV_FRAME_BODY_FRD,
                x: s.pos.x as f32,
                y: s.pos.y as f32,
                z: s.pos.z as f32,
                q: [s.att.w as f32, s.att.x as f32, s.att.y as f32, s.att.z as f32],
                vx: f32::NAN,
                vy: f32::NAN,
                vz: f32::NAN,
                rollspeed: f32::NAN,
                pitchspeed: f32::NAN,
                yawspeed: f32::NAN,
                pose_covariance: cov,
                velocity_covariance: cov,
                ..Default::default()
            }));
        }
        // Mavlink_NoGPS and Mavlink_Vision: no GPS.
        if tick % GPS_EVERY == 0 && udp_mode != 5 && udp_mode != 6 {
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
            if fixed_wing {
                let vb = s.att.rotate_inv(s.vel);
                println!(
                    "    fw: airspeed {:5.1} m/s  α {:5.1}°  β {:5.1}°  thr {:.2}  ail L/R {:+.2}/{:+.2}  elev {:+.2}  rud {:+.2}",
                    vb.norm(), vb.z.atan2(vb.x).to_degrees(), (vb.y / vb.norm().max(0.1)).asin().to_degrees(), u[4], u[5], u[6], u[7], u[2]
                );
            }
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
    if let Some(f) = &args.params_file {
        match std::fs::read_to_string(f).map_err(|e| e.to_string()).and_then(|t| Params::from_text(&t)) {
            Ok(p) => {
                println!("coptersim: loaded vehicle parameters from {f} (mass {:.2} kg, hover throttle {:.0} %)", p.mass, p.hover_throttle() * 100.0);
                return p;
            }
            Err(e) => {
                eprintln!("coptersim: cannot use {f}: {e}");
                std::process::exit(2);
            }
        }
    }
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
