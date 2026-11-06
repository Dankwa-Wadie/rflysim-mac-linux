//! CopterSim window for macOS and Linux, laid out like RflySim's Windows CopterSim:
//! vehicle configurator (frame, weight, size, motor / propeller / ESC / battery from RflySim's catalog, or a
//! custom design), design estimates and model parameters, simulation settings (mode, vehicles, map, Pixhawk
//! port, start position) with Start / Stop / Restart, and the live vehicle state.
//!
//! Start runs the same launchers as the RflyTools shortcuts (SITLRun / HITLRun, or coptersim's external
//! controller port), with the configured vehicle passed as a parameter file.

mod widgets;

use macroquad::prelude::*;
use rfly_core::Frame;
use rfly_db::{Battery, Esc, ModelDb, Motor, Propeller};
use rflygame::design::{estimate, Design, Estimate};
use std::collections::VecDeque;
use std::io::{BufRead, BufReader};
use std::net::UdpSocket;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use widgets::*;

const STATUS_BASE: u16 = 30400;
const OS: &str = if cfg!(target_os = "macos") { "macOS" } else { "Linux" };
const SIM_MODES: [&str; 3] = ["PX4_SITL  (PX4 on this computer)", "PX4_HITL  (Pixhawk over USB)", "Simulink / Python  (external controller, UDP 20100)"];
const UDP_MODES: [&str; 7] = ["UDP_Full", "UDP_Simple", "Mavlink_Full", "Mavlink_Simple", "Mavlink_NoSend", "Mavlink_NoGPS", "Mavlink_Vision"];

fn window_conf() -> Conf {
    Conf { window_title: format!("CopterSim ({OS})"), window_width: 1280, window_height: 820, high_dpi: false, sample_count: 4, ..Default::default() }
}

/// Folder with scripts/ and bin/ (installed) or the source checkout.
fn tools_root() -> PathBuf {
    let exe = std::env::current_exe().ok().and_then(|p| p.canonicalize().ok()).unwrap_or_default();
    if let Some(r) = exe.ancestors().find(|a| a.join("Cargo.toml").is_file() && a.join("crates").is_dir()) {
        return r.to_path_buf();
    }
    exe.parent().and_then(Path::parent).map(Path::to_path_buf).unwrap_or_else(|| ".".into())
}

fn bin(root: &Path, name: &str) -> PathBuf {
    let b = root.join("bin").join(name);
    if b.is_file() {
        b
    } else {
        root.join("target/release").join(name)
    }
}

// ---------------------------------------------------------------------------------------------------------
// Catalog

struct Catalog {
    motors: Vec<Motor>,
    props: Vec<Propeller>,
    escs: Vec<Esc>,
    batteries: Vec<Battery>,
    /// RflySim's assembled airframes (planeItems).
    airframes: Vec<(String, String)>,
    source: String,
}

impl Catalog {
    fn load() -> Option<Catalog> {
        let path = rflygame::garage::db_path()?;
        let db = ModelDb::open(&path).ok()?;
        Some(Catalog {
            motors: db.motors().ok()?,
            props: db.propellers().ok()?,
            escs: db.escs().ok()?,
            batteries: db.batteries().ok()?,
            airframes: db.airframes().ok()?,
            source: path.display().to_string(),
        })
    }
}

fn brands<T>(v: &[T], f: impl Fn(&T) -> &str) -> Vec<String> {
    let mut b: Vec<String> = v.iter().map(|x| f(x).to_string()).collect();
    b.dedup();
    b.sort();
    b.dedup();
    b
}

/// A brand/model picker over one catalog list.
struct Pick {
    brand: usize,
    model: usize,
}

fn models<'a, T>(v: &'a [T], brand: &str, b: impl Fn(&T) -> &str) -> Vec<&'a T> {
    v.iter().filter(|x| b(x) == brand).collect()
}

// ---------------------------------------------------------------------------------------------------------
// User database: designs saved with "Add to Database" (RflySim's planeItems record format).

fn user_db_path() -> PathBuf {
    rfly_db::tools_data_dir().unwrap_or_else(|| ".".into()).join("vehicles.txt")
}

fn load_user_db() -> Vec<(String, String)> {
    std::fs::read_to_string(user_db_path())
        .unwrap_or_default()
        .lines()
        .filter_map(|l| l.split_once('|').map(|(a, b)| (a.to_string(), b.to_string())))
        .collect()
}

fn save_user_db(v: &[(String, String)]) {
    let _ = std::fs::create_dir_all(user_db_path().parent().unwrap());
    let s: String = v.iter().map(|(n, i)| format!("{n}|{i}\n")).collect();
    let _ = std::fs::write(user_db_path(), format!("# RflyTools vehicles (CopterSim window \"Add to Database\"), RflySim planeItems format\n{s}"));
}

// ---------------------------------------------------------------------------------------------------------
// Running simulation

struct Run {
    children: Vec<Child>,
    status: Vec<UdpSocket>,
    latest: Vec<Option<Vec<f64>>>,
    what: String,
}

fn push_log(log: &Arc<Mutex<VecDeque<String>>>, line: String) {
    if let Ok(mut l) = log.lock() {
        l.push_back(line);
        while l.len() > 300 {
            l.pop_front();
        }
    }
}

impl Run {
    fn spawn(cmds: Vec<Command>, n: usize, what: String, log: Arc<Mutex<VecDeque<String>>>) -> Result<Run, String> {
        let mut children = vec![];
        for mut c in cmds {
            c.stdin(Stdio::null()).stdout(Stdio::piped()).stderr(Stdio::piped());
            let mut ch = c.spawn().map_err(|e| format!("cannot start {c:?}: {e}"))?;
            for out in [ch.stdout.take().map(|o| Box::new(o) as Box<dyn std::io::Read + Send>), ch.stderr.take().map(|o| Box::new(o) as Box<dyn std::io::Read + Send>)].into_iter().flatten() {
                let log = log.clone();
                std::thread::spawn(move || {
                    for line in BufReader::new(out).lines().map_while(Result::ok) {
                        push_log(&log, line);
                    }
                });
            }
            children.push(ch);
        }
        let status = (0..n)
            .filter_map(|k| UdpSocket::bind(("127.0.0.1", STATUS_BASE + k as u16)).ok())
            .inspect(|s| {
                let _ = s.set_nonblocking(true);
            })
            .collect::<Vec<_>>();
        Ok(Run { children, latest: vec![None; status.len()], status, what })
    }

    fn poll(&mut self) {
        let mut buf = [0u8; 512];
        for (k, s) in self.status.iter().enumerate() {
            while let Ok(n) = s.recv(&mut buf) {
                let t = String::from_utf8_lossy(&buf[..n]);
                // "RFLYSTAT id t x y z vx vy vz roll pitch yaw u1 u2 u3 u4 armed" → [t, x, ..., armed] (15 values)
                let v: Vec<f64> = t.split_whitespace().skip(2).filter_map(|x| x.parse().ok()).collect();
                if v.len() >= 15 {
                    self.latest[k] = Some(v);
                }
            }
        }
    }

    fn running(&mut self) -> bool {
        self.children.iter_mut().any(|c| matches!(c.try_wait(), Ok(None)))
    }

    fn stop(&mut self) {
        for c in &mut self.children {
            // SIGTERM lets SITLRun/HITLRun clean up their own processes.
            let _ = Command::new("kill").arg("-TERM").arg(c.id().to_string()).status();
        }
        let t0 = std::time::Instant::now();
        while self.running() && t0.elapsed().as_secs_f32() < 8.0 {
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        for c in &mut self.children {
            let _ = c.kill();
            let _ = c.wait();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// App state

struct App {
    root: PathBuf,
    cat: Option<Catalog>,
    custom: bool,
    // configurator fields
    name: String,
    mass: String,
    diag: String,
    alt: String,
    temp: String,
    motor: Pick,
    prop: Pick,
    esc: Pick,
    battery: Pick,
    // custom-design fields
    c_kv: String,
    c_rm: String,
    c_i0: String,
    c_pd: String,
    c_pp: String,
    c_esc: String,
    c_cells: String,
    c_mah: String,
    c_c: String,
    // database
    db_sel: usize,
    user_db: Vec<(String, String)>,
    show_params: bool,
    // simulation settings
    vid: String,
    count: String,
    mode: usize,
    udp: usize,
    map: usize,
    ports: Vec<String>,
    port: usize,
    px: String,
    py: String,
    yaw: String,
    qgc: bool,
    fg: bool,
    low_gpu: bool,
    no_3d: bool,
    live_sel: usize,
    run: Option<Run>,
    log: Arc<Mutex<VecDeque<String>>>,
    message: String,
}

fn num(s: &str) -> Option<f64> {
    s.trim().parse::<f64>().ok().filter(|v| v.is_finite())
}

impl App {
    fn new() -> App {
        let cat = Catalog::load();
        let mut a = App {
            root: tools_root(),
            custom: cat.is_none(),
            cat,
            name: "My quadcopter".into(),
            mass: "1.5".into(),
            diag: "450".into(),
            alt: "50".into(),
            temp: "25".into(),
            motor: Pick { brand: 0, model: 0 },
            prop: Pick { brand: 0, model: 0 },
            esc: Pick { brand: 0, model: 0 },
            battery: Pick { brand: 0, model: 0 },
            c_kv: "960".into(),
            c_rm: "117".into(),
            c_i0: "0.45".into(),
            c_pd: "10".into(),
            c_pp: "4.7".into(),
            c_esc: "20".into(),
            c_cells: "3".into(),
            c_mah: "5500".into(),
            c_c: "25".into(),
            db_sel: 0,
            user_db: load_user_db(),
            show_params: false,
            vid: "1".into(),
            count: "1".into(),
            mode: 0,
            udp: 2,
            map: 0,
            ports: vec![],
            port: 0,
            px: "0".into(),
            py: "0".into(),
            yaw: "0".into(),
            qgc: false,
            fg: false,
            low_gpu: false,
            no_3d: false,
            live_sel: 0,
            run: None,
            log: Arc::new(Mutex::new(VecDeque::new())),
            message: String::new(),
        };
        // Start with RflySim's DJI F450, like CopterSim does.
        if let Some(i) = a.db_names().iter().position(|n| n == "DJI F450") {
            a.db_sel = i;
            a.activate();
        }
        a.refresh_ports();
        a
    }

    /// RflySim airframes, then the user's saved designs.
    fn db_names(&self) -> Vec<String> {
        let mut v: Vec<String> = self.cat.as_ref().map(|c| c.airframes.iter().map(|a| a.0.clone()).collect()).unwrap_or_default();
        v.extend(self.user_db.iter().map(|(n, _)| format!("{n}  (yours)")));
        v
    }

    fn db_item(&self, i: usize) -> Option<(String, String, bool)> {
        let n_rfly = self.cat.as_ref().map(|c| c.airframes.len()).unwrap_or(0);
        if i < n_rfly {
            self.cat.as_ref().map(|c| (c.airframes[i].0.clone(), c.airframes[i].1.clone(), false))
        } else {
            self.user_db.get(i - n_rfly).map(|(n, it)| (n.clone(), it.clone(), true))
        }
    }

    /// "Activate!": load the selected database entry into the configurator.
    fn activate(&mut self) {
        let Some((name, item, _)) = self.db_item(self.db_sel) else { return };
        if let Some(c) = item.strip_prefix("CUSTOM,") {
            let v: Vec<&str> = c.split(',').collect();
            let g = |i: usize| v.get(i).unwrap_or(&"").to_string();
            self.custom = true;
            self.name = name;
            (self.mass, self.diag, self.alt, self.temp) = (g(0), g(1), g(2), g(3));
            (self.c_kv, self.c_rm, self.c_i0, self.c_pd, self.c_pp, self.c_esc, self.c_cells, self.c_mah, self.c_c) = (g(4), g(5), g(6), g(7), g(8), g(9), g(10), g(11), g(12));
            self.message = format!("Loaded {}", self.name);
            return;
        }
        let Some(af) = rfly_db::parse_airframe(&name, &item) else {
            self.message = format!("{name}: cannot read this database entry");
            return;
        };
        let head: Vec<&str> = item.split(';').next().unwrap_or("").split(',').collect();
        self.custom = false;
        self.name = name;
        self.mass = format!("{}", af.mass_kg);
        self.diag = format!("{}", af.wheelbase_mm);
        if let Some(a) = head.get(4).filter(|s| !s.is_empty()) {
            self.alt = a.to_string();
        }
        if let Some(t) = head.get(5).filter(|s| !s.is_empty()) {
            self.temp = t.to_string();
        }
        let Some(cat) = &self.cat else { return };
        fn set<T>(p: &mut Pick, v: &[T], want: &(String, String), b: impl Fn(&T) -> &str + Copy, m: impl Fn(&T) -> &str) {
            let bs = brands(v, b);
            if let Some(bi) = bs.iter().position(|x| x.eq_ignore_ascii_case(&want.0)) {
                p.brand = bi;
                let ms = models(v, &bs[bi], b);
                p.model = ms.iter().position(|x| m(x).eq_ignore_ascii_case(&want.1)).unwrap_or(0);
            }
        }
        set(&mut self.motor, &cat.motors, &af.motor, |x| &x.brand, |x| &x.model);
        set(&mut self.prop, &cat.props, &af.prop, |x| &x.brand, |x| &x.model);
        set(&mut self.esc, &cat.escs, &af.esc, |x| &x.brand, |x| &x.model);
        set(&mut self.battery, &cat.batteries, &af.battery, |x| &x.brand, |x| &x.model);
        self.message = format!("Loaded {}", self.name);
    }

    fn picked<'a, T>(v: &'a [T], p: &Pick, b: impl Fn(&T) -> &str + Copy) -> Option<&'a T> {
        let bs = brands(v, b);
        let ms = models(v, bs.get(p.brand)?, b);
        ms.get(p.model).copied()
    }

    fn design(&self) -> Result<Design, String> {
        let mass = num(&self.mass).filter(|v| *v > 0.05).ok_or("Total weight must be a positive number (kg)")?;
        let diag = num(&self.diag).filter(|v| *v > 50.0).ok_or("Diagonal size must be at least 50 mm")?;
        let alt = num(&self.alt).ok_or("Altitude must be a number (m)")?;
        let temp = num(&self.temp).ok_or("Temperature must be a number (°C)")?;
        let (motor, prop, esc, battery) = if self.custom {
            let f = |s: &str, what: &str| num(s).filter(|v| *v > 0.0).ok_or(format!("{what} must be a positive number"));
            let cells = f(&self.c_cells, "Battery cells")?;
            (
                Motor { brand: "Custom".into(), model: "Custom".into(), kv: f(&self.c_kv, "Motor KV")?, resistance_mohm: f(&self.c_rm, "Motor resistance")?, i0: f(&self.c_i0, "No-load current")?, u0: 10.0, ..Default::default() },
                Propeller { brand: "Custom".into(), model: "Custom".into(), diameter_in: f(&self.c_pd, "Propeller diameter")?, pitch_in: f(&self.c_pp, "Propeller pitch")?, ct_correct: 1.0, cm_correct: 1.0, ..Default::default() },
                Esc { brand: "Custom".into(), model: "Custom".into(), max_current: f(&self.c_esc, "ESC current")?, ..Default::default() },
                Battery { brand: "Custom".into(), model: "Custom".into(), capacity_mah: f(&self.c_mah, "Battery capacity")?, cells, volts: cells * 3.7, c_cont: f(&self.c_c, "Battery C rating")?, ..Default::default() },
            )
        } else {
            let c = self.cat.as_ref().ok_or("RflySim's catalog is not imported")?;
            (
                Self::picked(&c.motors, &self.motor, |x| &x.brand).cloned().ok_or("choose a motor")?,
                Self::picked(&c.props, &self.prop, |x| &x.brand).cloned().ok_or("choose a propeller")?,
                Self::picked(&c.escs, &self.esc, |x| &x.brand).cloned().ok_or("choose an ESC")?,
                Self::picked(&c.batteries, &self.battery, |x| &x.brand).cloned().ok_or("choose a battery")?,
            )
        };
        Ok(Design { name: self.name.clone(), frame: Frame::QuadX, mass_kg: mass, wheelbase_mm: diag, altitude_m: alt, temp_c: temp, motor, prop, esc, battery })
    }

    fn db_record(&self, d: &Design) -> String {
        if self.custom {
            format!(
                "CUSTOM,{},{},{},{},{},{},{},{},{},{},{},{},{}",
                self.mass, self.diag, self.alt, self.temp, self.c_kv, self.c_rm, self.c_i0, self.c_pd, self.c_pp, self.c_esc, self.c_cells, self.c_mah, self.c_c
            )
        } else {
            format!(
                "QUAD,0,{},{},{},{},1;10,85,90,0.5;{},{},,,,,,,,;{},{},,,,;{},{},,,,;{},{},1,1,,,,,,,",
                d.mass_kg, d.wheelbase_mm, d.altitude_m, d.temp_c, d.motor.brand, d.motor.model, d.prop.brand, d.prop.model, d.esc.brand, d.esc.model, d.battery.brand, d.battery.model
            )
        }
    }

    fn refresh_ports(&mut self) {
        let out = Command::new(self.root.join("scripts/ComList.sh")).output().ok().map(|o| String::from_utf8_lossy(&o.stdout).to_string()).unwrap_or_default();
        self.ports = out
            .lines()
            .filter_map(|l| {
                let t = l.trim_start();
                let (n, rest) = t.split_once(char::is_whitespace)?;
                n.parse::<u32>().ok()?;
                Some(rest.trim().to_string())
            })
            .collect();
        if self.ports.is_empty() {
            self.ports.push("(no serial port: plug in the Pixhawk, then Refresh)".into());
        }
        self.port = self.port.min(self.ports.len() - 1);
    }

    fn params_file(&self) -> PathBuf {
        rfly_db::tools_data_dir().unwrap_or_else(std::env::temp_dir).join("coptersim-vehicle.txt")
    }

    fn start(&mut self) {
        let d = match self.design() {
            Ok(d) => d,
            Err(e) => {
                self.message = e;
                return;
            }
        };
        let e = estimate(&d);
        let pf = self.params_file();
        let _ = std::fs::create_dir_all(pf.parent().unwrap());
        if let Err(err) = std::fs::write(&pf, e.params.to_text(&d.name)) {
            self.message = format!("cannot write {}: {err}", pf.display());
            return;
        }
        let id: u32 = self.vid.trim().parse().unwrap_or(1).max(1);
        let n: usize = if self.mode == 0 { self.count.trim().parse().unwrap_or(1).clamp(1, 50) } else { 1 };
        let map = rflysim3d::scene::MAP_NAMES[self.map];
        let (x, y, yaw) = (num(&self.px).unwrap_or(0.0), num(&self.py).unwrap_or(0.0), num(&self.yaw).unwrap_or(0.0));
        let scripts = self.root.join("scripts");
        let mut common: Vec<String> = vec![
            "--start-index".into(), id.to_string(), "--map".into(), map.into(), "--yaw".into(), yaw.to_string(),
            "--params-file".into(), pf.display().to_string(), "--status-port".into(), STATUS_BASE.to_string(),
        ];
        if self.low_gpu {
            common.push("--low-gpu".into());
        }
        common.extend(["--udp-mode".into(), self.udp.to_string()]);
        if self.no_3d {
            common.push("--no-3d".into());
        }
        if self.qgc {
            common.push("--qgc".into());
        }
        let cmds = match self.mode {
            0 => {
                let mut c = Command::new(scripts.join("SITLRun.sh"));
                c.arg(n.to_string()).args(&common).args(["--origin", &x.to_string(), &y.to_string()]);
                if self.fg {
                    c.arg("--fg");
                }
                vec![c]
            }
            1 => {
                let port = self.ports.get(self.port).and_then(|p| p.split_whitespace().next()).unwrap_or("");
                if !port.starts_with('/') {
                    self.message = "Choose the Pixhawk's serial port (plug it in, then Refresh)".into();
                    return;
                }
                let mut c = Command::new(scripts.join("HITLRun.sh"));
                c.arg(port).args(&common).args(["--origin", &x.to_string(), &y.to_string()]);
                vec![c]
            }
            _ => {
                let mut sim = Command::new(bin(&self.root, "coptersim"));
                sim.args(["--ext", "20100", "--id", &id.to_string(), "--params-file", &pf.display().to_string()])
                    .args(["--status-port", &STATUS_BASE.to_string()])
                    .arg(format!("--init-x={x}"))
                    .arg(format!("--init-y={y}"))
                    .arg(format!("--init-yaw={yaw}"));
                if self.fg {
                    sim.arg("--fg");
                }
                let mut v = vec![sim];
                if !self.no_3d {
                    let mut view = Command::new(bin(&self.root, "rflysim3d"));
                    view.args(["--map", map]);
                    if self.low_gpu {
                        view.arg("--low-gpu");
                    }
                    v.push(view);
                }
                v
            }
        };
        if let Ok(mut l) = self.log.lock() {
            l.clear();
        }
        let what = format!("{} · {} vehicle(s) from #{id} · {}", SIM_MODES[self.mode].split("  ").next().unwrap_or(""), n, d.name);
        match Run::spawn(cmds, n, what, self.log.clone()) {
            Ok(r) => {
                self.run = Some(r);
                self.live_sel = 0;
                self.message = format!("Started. Vehicle: {} (hover throttle {:.0} %)", d.name, e.hover_throttle * 100.0);
            }
            Err(err) => self.message = err,
        }
    }

    fn stop(&mut self) {
        if let Some(mut r) = self.run.take() {
            r.stop();
            self.message = "Simulation stopped.".into();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// Drawing

fn row_label(ui: &Ui, s: &str, x: f32, y: f32) {
    ui.label(s, x, y);
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut app = App::new();
    let mut ui = Ui::default();
    // Automation for tests and docs: --auto sitl|hitl|ext presses Start after 1 s; --screenshot F --after S.
    let argv: Vec<String> = std::env::args().collect();
    let arg = |k: &str| argv.iter().position(|a| a == k).and_then(|i| argv.get(i + 1)).cloned();
    let auto = arg("--auto");
    let shot = arg("--screenshot");
    let shot_after: f64 = arg("--after").and_then(|v| v.parse().ok()).unwrap_or(3.0);
    let mut auto_started = false;
    if let Some(m) = &auto {
        app.mode = match m.as_str() { "hitl" => 1, "ext" => 2, _ => 0 };
        app.no_3d = argv.iter().any(|a| a == "--no-3d");
    }
    let maps: Vec<String> = rflysim3d::scene::MAP_NAMES.iter().map(|s| s.to_string()).collect();
    loop {
        clear_background(BG);
        ui.begin();
        let est = app.design().map(|d| (estimate(&d), d));
        if let Some(r) = app.run.as_mut() {
            r.poll();
            if !r.running() {
                app.message = "The simulation ended (see the log).".into();
                app.run = None;
            }
        }
        let running = app.run.is_some();

        // ---------------- Vehicle configurator ----------------
        let p = Rect::new(10.0, 10.0, 640.0, 470.0);
        ui.panel(p, "Vehicle");
        let (x0, mut y) = (p.x + 12.0, p.y + 38.0);
        if ui.radio("Brand model (RflySim catalog)", x0, y, !app.custom) && app.cat.is_some() {
            app.custom = false;
        }
        if ui.radio("Custom design", x0 + 290.0, y, app.custom) {
            app.custom = true;
        }
        y += ROW + 6.0;
        row_label(&ui, "Name", x0, y);
        ui.field(id("name", 0), Rect::new(x0 + 110.0, y, 200.0, 24.0), &mut app.name);
        row_label(&ui, "Frame", x0 + 330.0, y);
        let mut frame = 0usize;
        ui.dropdown(id("frame", 0), Rect::new(x0 + 390.0, y, 220.0, 24.0), &["Quadcopter (X)".to_string()], &mut frame);
        y += ROW + 6.0;
        for (k, (lbl, unit, val)) in [("Total weight", "kg", &mut app.mass), ("Diagonal size", "mm", &mut app.diag), ("Altitude", "m", &mut app.alt), ("Temperature", "°C", &mut app.temp)].into_iter().enumerate() {
            let (cx, cy) = (x0 + (k % 2) as f32 * 310.0, y + (k / 2) as f32 * (ROW + 6.0));
            row_label(&ui, lbl, cx, cy);
            ui.field(id("num", k), Rect::new(cx + 110.0, cy, 120.0, 24.0), val);
            text(unit, cx + 238.0, cy + 4.0, FS, DIM);
        }
        y += 2.0 * (ROW + 6.0) + 6.0;
        if app.custom {
            let fields: [(&str, &str, &mut String); 9] = [
                ("Motor KV", "rpm/V", &mut app.c_kv),
                ("Motor resistance", "mΩ", &mut app.c_rm),
                ("No-load current", "A", &mut app.c_i0),
                ("Propeller diameter", "in", &mut app.c_pd),
                ("Propeller pitch", "in", &mut app.c_pp),
                ("ESC max current", "A", &mut app.c_esc),
                ("Battery cells", "S", &mut app.c_cells),
                ("Battery capacity", "mAh", &mut app.c_mah),
                ("Battery C rating", "C", &mut app.c_c),
            ];
            for (k, (lbl, unit, val)) in fields.into_iter().enumerate() {
                let (cx, cy) = (x0 + (k % 2) as f32 * 310.0, y + (k / 2) as f32 * (ROW + 6.0));
                row_label(&ui, lbl, cx, cy);
                ui.field(id("custom", k), Rect::new(cx + 150.0, cy, 90.0, 24.0), val);
                text(unit, cx + 248.0, cy + 4.0, FS, DIM);
            }
            y += 5.0 * (ROW + 6.0);
        } else if let Some(cat) = &app.cat {
            // Motor / Propeller / ESC / Battery: brand and model, as in CopterSim.
            fn part<T>(ui: &mut Ui, label: &str, k: usize, x0: f32, y: f32, v: &[T], p: &mut Pick, b: impl Fn(&T) -> &str + Copy, m: impl Fn(&T) -> String) {
                row_label(ui, label, x0, y);
                let bs = brands(v, b);
                if ui.dropdown(id("brand", k), Rect::new(x0 + 110.0, y, 190.0, 24.0), &bs, &mut p.brand) {
                    p.model = 0;
                }
                let ms: Vec<String> = models(v, bs.get(p.brand).map(String::as_str).unwrap_or(""), b).into_iter().map(m).collect();
                ui.dropdown(id("model", k), Rect::new(x0 + 310.0, y, 300.0, 24.0), &ms, &mut p.model);
            }
            part(&mut ui, "Motor", 0, x0, y, &cat.motors, &mut app.motor, |x| &x.brand, |x| format!("{}  (KV{:.0}, {:.0} g)", x.model, x.kv, x.weight_g));
            part(&mut ui, "Propeller", 1, x0, y + ROW + 6.0, &cat.props, &mut app.prop, |x| &x.brand, |x| format!("{}  ({}x{} in)", x.model, x.diameter_in, x.pitch_in));
            part(&mut ui, "ESC", 2, x0, y + 2.0 * (ROW + 6.0), &cat.escs, &mut app.esc, |x| &x.brand, |x| format!("{}  ({:.0} A)", x.model, x.max_current));
            part(&mut ui, "Battery", 3, x0, y + 3.0 * (ROW + 6.0), &cat.batteries, &mut app.battery, |x| &x.brand, |x| format!("{}  ({:.0} mAh)", x.model, x.capacity_mah));
            y += 4.0 * (ROW + 6.0) + 32.0;
        }
        if app.cat.is_none() {
            text("RflySim's parts catalog (ModelData.db) is not imported: Custom design only.", x0, y, FS, WARN);
            if ui.button("Import RflySim Data...", Rect::new(x0, y + 24.0, 220.0, 28.0), true) {
                let _ = Command::new("open").arg("-a").arg("Terminal").arg(app.root.join("scripts/import_rflysim_data.sh")).spawn();
            }
            y += 60.0;
        }
        // Assembled UAV database
        let y_db = p.y + p.h - 76.0;
        let _ = y;
        row_label(&ui, "Assembled UAV Database", x0, y_db);
        let names = app.db_names();
        if !names.is_empty() {
            ui.dropdown(id("db", 0), Rect::new(x0 + 210.0, y_db, 400.0, 24.0), &names, &mut app.db_sel);
        } else {
            text("(empty)", x0 + 210.0, y_db + 4.0, FS, DIM);
        }
        let by = y_db + 36.0;
        let bw = 146.0;
        if ui.button("Activate!", Rect::new(x0, by, bw, 28.0), !names.is_empty() && !running) {
            app.activate();
        }
        if ui.button(if app.show_params { "Estimates" } else { "Model Parameter" }, Rect::new(x0 + bw + 8.0, by, bw, 28.0), true) {
            app.show_params = !app.show_params;
        }
        if ui.button("Add to Database", Rect::new(x0 + 2.0 * (bw + 8.0), by, bw, 28.0), est.is_ok()) {
            if let Ok((_, d)) = &est {
                let rec = app.db_record(d);
                let name = app.name.trim().replace('|', "/");
                app.user_db.retain(|(n, _)| *n != name);
                app.user_db.push((name.clone(), rec));
                save_user_db(&app.user_db);
                app.db_sel = app.db_names().len() - 1;
                app.message = format!("Saved \"{name}\" to your database ({})", user_db_path().display());
            }
        }
        let user_sel = app.db_item(app.db_sel).is_some_and(|x| x.2);
        if ui.button("Delete from Database", Rect::new(x0 + 3.0 * (bw + 8.0), by, bw, 28.0), user_sel) {
            let n_rfly = app.cat.as_ref().map(|c| c.airframes.len()).unwrap_or(0);
            let removed = app.user_db.remove(app.db_sel - n_rfly);
            save_user_db(&app.user_db);
            app.db_sel = 0;
            app.message = format!("Deleted \"{}\" (RflySim's own entries cannot be deleted)", removed.0);
        }

        // ---------------- Estimates / model parameters ----------------
        let q = Rect::new(660.0, 10.0, 610.0, 470.0);
        ui.panel(q, if app.show_params { "Model Parameter (used by the simulation)" } else { "Design estimate" });
        let (qx, mut qy) = (q.x + 14.0, q.y + 40.0);
        match &est {
            Err(e) => text(e, qx, qy, FS, BAD),
            Ok((e, _)) if app.show_params => {
                let labels = [
                    ("Mass", "kg"), ("Gravity", "m/s²"), ("Jxx", "kg·m²"), ("Jyy", "kg·m²"), ("Jzz", "kg·m²"), ("Arm length", "m"), ("Thrust coefficient Ct", "N/(rad/s)²"),
                    ("Torque coefficient Cm", "N·m/(rad/s)²"), ("Motor curve Cr", "rad/s"), ("Motor curve ωb", "rad/s"), ("Motor+prop inertia Jm", "kg·m²"), ("Motor time constant Tm", "s"),
                    ("Drag coefficient Cd", ""), ("Rotational drag Cmd", ""),
                ];
                for ((l, u), v) in labels.iter().zip(e.params.model_info()) {
                    text(l, qx, qy, FS, TEXT);
                    text(&format!("{v:.5e}"), qx + 260.0, qy, FS, TEXT);
                    text(u, qx + 400.0, qy, FS, DIM);
                    qy += 22.0;
                }
                qy += 8.0;
                text(&format!("Saved for the simulation at Start: {}", app.params_file().display()), qx, qy, 14.0, DIM);
            }
            Ok((e, d)) => {
                draw_estimate(e, d, qx, qy);
            }
        }

        // ---------------- Simulation settings ----------------
        let s = Rect::new(10.0, 490.0, 640.0, 300.0);
        ui.panel(s, "Simulation");
        let (sx, sy) = (s.x + 12.0, s.y + 40.0);
        row_label(&ui, "Vehicle ID", sx, sy);
        ui.field(id("vid", 0), Rect::new(sx + 100.0, sy, 60.0, 24.0), &mut app.vid);
        row_label(&ui, "Vehicles", sx + 180.0, sy);
        if app.mode == 0 {
            ui.field(id("count", 0), Rect::new(sx + 260.0, sy, 60.0, 24.0), &mut app.count);
        } else {
            text("1", sx + 265.0, sy + 4.0, FS, DIM);
        }
        row_label(&ui, "UDP Mode", sx + 340.0, sy);
        ui.dropdown(id("udp", 0), Rect::new(sx + 430.0, sy, 180.0, 24.0), &UDP_MODES.iter().map(|s| s.to_string()).collect::<Vec<_>>(), &mut app.udp);
        let sy2 = sy + ROW + 8.0;
        row_label(&ui, "Simulation Mode", sx, sy2);
        ui.dropdown(id("mode", 0), Rect::new(sx + 150.0, sy2, 460.0, 24.0), &SIM_MODES.iter().map(|s| s.to_string()).collect::<Vec<_>>(), &mut app.mode);
        let sy3 = sy2 + ROW + 8.0;
        row_label(&ui, "3D Scene", sx, sy3);
        ui.dropdown(id("map", 0), Rect::new(sx + 150.0, sy3, 200.0, 24.0), &maps, &mut app.map);
        ui.checkbox("No 3D window", sx + 370.0, sy3, &mut app.no_3d);
        ui.checkbox("Low GPU", sx + 510.0, sy3, &mut app.low_gpu);
        let sy4 = sy3 + ROW + 8.0;
        row_label(&ui, "Pixhawk port", sx, sy4);
        ui.dropdown(id("port", 0), Rect::new(sx + 150.0, sy4, 360.0, 24.0), &app.ports.clone(), &mut app.port);
        if ui.button("Refresh", Rect::new(sx + 520.0, sy4, 90.0, 24.0), !running) {
            app.refresh_ports();
        }
        let sy5 = sy4 + ROW + 8.0;
        row_label(&ui, "Initial position", sx, sy5);
        for (k, (l, v)) in [("x", &mut app.px), ("y", &mut app.py), ("yaw", &mut app.yaw)].into_iter().enumerate() {
            let cx = sx + 150.0 + k as f32 * 150.0;
            text(l, cx, sy5 + 4.0, FS, DIM);
            ui.field(id("pos", k), Rect::new(cx + 34.0, sy5, 90.0, 24.0), v);
        }
        let sy6 = sy5 + ROW + 8.0;
        ui.checkbox("Start QGroundControl", sx, sy6, &mut app.qgc);
        ui.checkbox("FlightGear (F450)", sx + 240.0, sy6, &mut app.fg);
        let sy7 = sy6 + ROW + 4.0;
        let ok = est.is_ok();
        if ui.button("Start Simulation", Rect::new(sx, sy7, 190.0, 34.0), ok && !running) {
            app.start();
        }
        if ui.button("Stop Simulation", Rect::new(sx + 205.0, sy7, 190.0, 34.0), running) {
            app.stop();
        }
        if ui.button("Restart Simulation", Rect::new(sx + 410.0, sy7, 200.0, 34.0), running) {
            app.stop();
            app.start();
        }

        // ---------------- Live state + log ----------------
        let l = Rect::new(660.0, 490.0, 610.0, 300.0);
        ui.panel(l, "State");
        let (lx, ly) = (l.x + 14.0, l.y + 38.0);
        if let Some(r) = &app.run {
            text(&fit(&r.what, l.w - 28.0, FS), lx, ly, FS, ACCENT);
            let n = r.latest.len();
            if n > 1 {
                let labels: Vec<String> = (0..n).map(|k| format!("Vehicle #{}", app.vid.trim().parse::<usize>().unwrap_or(1) + k)).collect();
                ui.dropdown(id("live", 0), Rect::new(lx + 400.0, ly - 4.0, 180.0, 24.0), &labels, &mut app.live_sel);
            }
            match r.latest.get(app.live_sel).cloned().flatten() {
                Some(v) => {
                    let rows = [
                        ("X", v[1], "Y", v[2], "Z", v[3], "m"),
                        ("Vx", v[4], "Vy", v[5], "Vz", v[6], "m/s"),
                        ("Roll", v[7], "Pitch", v[8], "Yaw", v[9], "deg"),
                    ];
                    for (k, (a, av, b, bv, c, cv, u)) in rows.iter().enumerate() {
                        let yy = ly + 28.0 + k as f32 * 24.0;
                        for (j, (n, val)) in [(a, av), (b, bv), (c, cv)].iter().enumerate() {
                            text(&format!("{n:<5} {val:8.2}"), lx + j as f32 * 170.0, yy, FS, TEXT);
                        }
                        text(u, lx + 520.0, yy, FS, DIM);
                    }
                    let yy = ly + 28.0 + 3.0 * 24.0;
                    text(&format!("motors {:.2} {:.2} {:.2} {:.2}   {}   t = {:.1} s", v[10], v[11], v[12], v[13], if v[14] > 0.5 { "ARMED" } else { "disarmed" }, v[0]), lx, yy, FS, TEXT);
                }
                None => text("Waiting for the simulator...", lx, ly + 28.0, FS, DIM),
            }
        } else {
            text("Not running. Configure the vehicle, then Start Simulation.", lx, ly, FS, DIM);
        }
        let logy = ly + 130.0;
        draw_rectangle(l.x + 8.0, logy - 4.0, l.w - 16.0, l.y + l.h - logy - 4.0, Color::new(0.97, 0.97, 0.98, 1.0));
        if let Ok(lines) = app.log.lock() {
            let rows = ((l.y + l.h - logy - 8.0) / 17.0) as usize;
            for (k, line) in lines.iter().rev().take(rows).rev().enumerate() {
                text(&fit(line, l.w - 30.0, 14.0), l.x + 14.0, logy + k as f32 * 17.0, 14.0, TEXT);
            }
        }

        // Status line
        if !app.message.is_empty() {
            let c = if app.message.contains("cannot") || app.message.contains("must") || app.message.contains("Choose") { BAD } else { DIM };
            text(&fit(&app.message, 640.0, 15.0), 12.0, screen_height() - 24.0, 15.0, c);
        }
        if let Some(c) = &app.cat {
            text(&fit(&format!("Catalog: {}", c.source), 600.0, 13.0), screen_width() - 610.0, screen_height() - 22.0, 13.0, DIM);
        }
        ui.overlay();
        if auto.is_some() && !auto_started && get_time() > 1.0 {
            auto_started = true;
            app.start();
        }
        if let Some(path) = &shot {
            if get_time() > shot_after {
                get_screen_data().export_png(path);
                if let Some(r) = app.run.as_ref() {
                    println!("coptersim-ui: state {:?}", r.latest.first().cloned().flatten());
                }
                app.stop();
                break;
            }
        }
        next_frame().await;
    }
}

fn draw_estimate(e: &Estimate, d: &Design, x: f32, mut y: f32) {
    let line = |y: &mut f32, k: &str, v: String| {
        text(k, x, *y, FS, TEXT);
        text(&v, x + 260.0, *y, FS, TEXT);
        *y += 24.0;
    };
    text("Hover", x, y, 19.0, ACCENT);
    y += 26.0;
    line(&mut y, "Throttle", format!("{:.1} %", e.hover_throttle * 100.0));
    line(&mut y, "Speed", format!("{:.0} rpm", e.hover_rpm));
    line(&mut y, "Current per motor", format!("{:.2} A", e.hover_current));
    line(&mut y, "Power (all motors)", format!("{:.0} W", e.hover_power_w));
    line(&mut y, "Flight time", format!("{:.1} min", e.hover_minutes));
    line(&mut y, "Efficiency", format!("{:.1} g/W", e.hover_efficiency));
    y += 8.0;
    text("Full throttle", x, y, 19.0, ACCENT);
    y += 26.0;
    line(&mut y, "Max thrust", format!("{:.2} kg", e.max_thrust_kg));
    line(&mut y, "Thrust / weight", format!("{:.2}", e.thrust_to_weight));
    line(&mut y, "Current per motor", format!("{:.1} A   (ESC {:.0} A)", e.max_current, d.esc.max_current));
    line(&mut y, "Speed", format!("{:.0} rpm", e.max_rpm));
    line(&mut y, "Air density", format!("{:.3} kg/m³", e.air_density));
    y += 6.0;
    if e.warnings.is_empty() {
        text("All parts within their limits.", x, y, FS, ACCENT);
    }
    for w in &e.warnings {
        text(&fit(&format!("! {w}"), 580.0, FS), x, y, FS, WARN);
        y += 22.0;
    }
}
