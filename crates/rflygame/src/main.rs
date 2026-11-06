//! RflyGame: drone flight game on the RflySim-for-macOS physics and 3D world.

use macroquad::prelude::*;
use rflygame::flight::{DronePreset, FlightMode, Sticks};
use rflygame::game::{autopilot, OtherDrone, Sample, Session};
use rflygame::garage::{self, Garage};
use rflygame::input::{AxisBind, Input};
use rflygame::mission::{missions, Mission};
use rflygame::net::{self, Activity as NetAct, Browser, Client, DroneState};
use rflygame::race::{tracks, Outcome, Race};
use rflygame::records;
use rflygame::school::{lessons, School};
use rflygame::settings::Settings;
use rflygame::ui::{self, Item, Menu, MenuEvent};
use rflysim3d::render::{CamPose, DrawVehicle, Renderer};
use rflysim3d::scene::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Instant;

fn window_conf() -> macroquad::conf::Conf {
    macroquad::conf::Conf {
        miniquad_conf: Conf {
            window_title: "RflyGame".into(),
            window_width: 1280,
            window_height: 760,
            high_dpi: false,
            sample_count: 4,
            window_resizable: true,
            ..Default::default()
        },
        draw_call_vertex_capacity: 65536,
        draw_call_index_capacity: 262144,
        ..Default::default()
    }
}

const MAP_LABELS: [&str; 6] = ["Grasslands", "Raceway", "City", "Desert", "VisionRingBlank", "Blank"];

enum Act {
    Free,
    Race(Race),
    Mission(Mission),
    School(School),
}

/// What to rebuild on "Retry".
#[derive(Clone)]
enum Setup {
    Free(String, DronePreset, FlightMode),
    Race(usize, DronePreset, FlightMode),
    Mission(usize, DronePreset, FlightMode),
    School(usize, DronePreset),
}

enum Screen {
    Main,
    FreeSetup,
    RaceSetup,
    MissionSetup,
    SchoolSetup,
    Playing,
    Paused,
    Results(Outcome),
    Boards,
    Replays(Vec<(std::path::PathBuf, String, String)>),
    Watch { map: Map, samples: Vec<Sample>, t: f32, paused: bool, orbit: bool },
    Garage(Box<Garage>),
    Settings,
    Controls,
    Wizard { step: usize, base: Vec<f32> },
    Together,
    MpMenu,
    Online,
    TextEdit { field: u8, buf: String },
    Lobby,
}

/// Newer RflyTools version found by the background update check (shown in the main menu).
static UPDATE: std::sync::Mutex<Option<String>> = std::sync::Mutex::new(None);
/// Index of the "Update" item in the main menu (after Quit, so the other indices stay fixed).
const MENU_UPDATE: usize = 11;

fn main_menu() -> Menu {
    let mut items: Vec<Item> = ["Free Flight", "Racing", "Missions", "Flight School", "Play together", "Garage", "Leaderboards", "Replays", "Controls", "Settings", "Quit"]
        .iter()
        .map(|s| Item::Button(s.to_string()))
        .collect();
    let update = UPDATE.lock().ok().and_then(|u| u.clone());
    if let Some(v) = &update {
        items.push(Item::Button(format!("Update to RflyTools {v}")));
    }
    let sub = match &update {
        Some(v) => format!("Update available: RflyTools {v} (you have {})", rfly_update::VERSION),
        None => "Fly the RflySim quad. Keyboard, mouse, gamepad or RC transmitter.".to_string(),
    };
    Menu::new("RflyGame", &sub, items)
}

fn drone_names(d: &[DronePreset]) -> Vec<String> {
    d.iter().map(|x| x.name.clone()).collect()
}
fn modes() -> Vec<String> {
    FlightMode::ALL.iter().map(|m| m.name().to_string()).collect()
}

fn setup_menu(title: &str, sub: &str, first: Item, drones: &[DronePreset], board: Vec<String>) -> Menu {
    let mut items = vec![first, Item::Choice("Drone".into(), drone_names(drones), 0), Item::Choice("Flight mode".into(), modes(), 0)];
    items.extend(board.into_iter().map(Item::Info));
    items.push(Item::Button("Fly!".into()));
    items.push(Item::Button("Back".into()));
    Menu::new(title, sub, items)
}

fn board_lines(key: &str, lower: bool) -> Vec<String> {
    let top = records::top(key, lower, 5);
    if top.is_empty() {
        return vec!["No times yet - be the first!".into()];
    }
    top.iter().enumerate().map(|(i, e)| format!("{}. {:<12} {:>8.2}  {}", i + 1, e.player, e.value, e.drone)).collect()
}

fn race_menu(drones: &[DronePreset], sel: usize) -> Menu {
    let t = tracks();
    let key = format!("Race: {}", t[sel].name);
    let mut m = setup_menu("Racing", t[sel].desc, Item::Choice("Track".into(), t.iter().map(|x| x.name.to_string()).collect(), sel), drones, board_lines(&key, true));
    m.sel = 0;
    m
}

fn mission_menu(drones: &[DronePreset], sel: usize) -> Menu {
    let ms = missions();
    let key = format!("Mission: {}", ms[sel].name);
    let mut m = setup_menu("Missions", ms[sel].desc, Item::Choice("Mission".into(), ms.iter().map(|x| x.name.to_string()).collect(), sel), drones, board_lines(&key, false));
    m.sel = 0;
    m
}

fn school_menu() -> Menu {
    let done = records::school_done();
    let mut items: Vec<Item> = lessons().iter().enumerate().map(|(i, l)| Item::Button(format!("{} {}", if done.contains(&i) { "[x]" } else { "[ ]" }, l.name))).collect();
    items.push(Item::Button("Back".into()));
    Menu::new("Flight School", "Lessons check what you do and tell you what's next.", items)
}

fn settings_menu(s: &Settings) -> Menu {
    Menu::new(
        "Settings",
        "Saved automatically.",
        vec![
            Item::Number("Camera FOV".into(), s.fov_deg as f64, 60.0, 130.0, 5.0, " deg"),
            Item::Number("FPV camera uptilt".into(), s.fpv_uptilt_deg as f64, 0.0, 50.0, 5.0, " deg"),
            Item::Number("Acro rate".into(), s.acro_rate, 200.0, 1200.0, 50.0, " deg/s"),
            Item::Number("Acro expo".into(), s.expo, 0.0, 0.9, 0.05, ""),
            Item::Number("Angle mode max tilt".into(), s.max_angle_deg, 15.0, 60.0, 5.0, " deg"),
            Item::Toggle("Show sticks".into(), s.show_sticks),
            Item::Toggle("Realistic battery".into(), s.battery),
            Item::Choice("Default camera".into(), vec!["Auto (FPV, chase in Beginner)".into(), "Always FPV".into(), "Always chase".into()], s.default_cam as usize),
            Item::Choice("Crash behaviour".into(), vec!["Realistic: drone falls, you recover".into(), "Arcade: crash screen and reset".into()], s.crash_style as usize),
            Item::Choice("Keyboard layout".into(), vec!["Mode 2: W/S throttle, A/D yaw, arrows pitch/roll".into(), "Arcade: W/S/A/D move, arrows throttle/yaw".into()], if s.kb_mode2 { 0 } else { 1 }),
            Item::Number("Sound volume".into(), s.volume as f64, 0.0, 10.0, 1.0, ""),
            Item::Toggle("Check for RflyTools updates daily".into(), rfly_update::enabled()),
            Item::Button("Back".into()),
        ],
    )
}

fn controls_menu(s: &Settings, input: &Input) -> Menu {
    Menu::new(
        "Controls",
        &format!("Active device: {}", input.device_name),
        vec![
            Item::Info(if s.kb_mode2 { "Keyboard (Mode 2): W/S throttle, A/D yaw, arrows pitch/roll" } else { "Keyboard (arcade): W/S pitch, A/D roll, Up/Down throttle, Left/Right yaw" }.into()),
            Item::Info("Shift = full stick.  Hold right mouse = mouse stick".into()),
            Item::Info("Gamepad (Mode 2): left = throttle/yaw, right = pitch/roll".into()),
            Item::Info("A arm, Y camera, X flight mode, Select reset, Start pause".into()),
            Item::Toggle("Invert pitch".into(), s.invert_pitch),
            Item::Number("Stick deadzone".into(), s.deadzone as f64, 0.0, 0.2, 0.01, ""),
            Item::Button("Map RC transmitter / joystick axes".into()),
            Item::Button("Back".into()),
        ],
    )
}

fn together_menu() -> Menu {
    Menu::new(
        "Play together",
        "Same screen, same network, or anywhere.",
        vec![
            Item::Button("Split screen - up to 4 pilots on this Mac".into()),
            Item::Button("Local network - friends on the same Wi-Fi".into()),
            Item::Button("Online - over the internet".into()),
            Item::Button("Back".into()),
        ],
    )
}

/// Local network: host, or join a game found by its broadcast beacon.
fn mp_menu(s: &Settings, browser: &Browser) -> Menu {
    let mut items = vec![Item::Button(format!("Player name: {}", s.player)), Item::Button("Host a game on this network".into())];
    if browser.found.is_empty() {
        items.push(Item::Info("Searching this network for games...".into()));
    }
    for f in &browser.found {
        items.push(Item::Button(format!("Join {} - {} players, {} ({})", f.name, f.players, f.activity, f.addr)));
    }
    items.push(Item::Info("Not listed? Some networks block discovery: use Online > Join by address.".into()));
    items.push(Item::Button("Back".into()));
    Menu::new("Local network", "Everyone on the same Wi-Fi or wired network.", items)
}

/// Internet: join a server by address, or host one others can reach.
fn online_menu(s: &Settings) -> Menu {
    Menu::new(
        "Online",
        "Fly with players anywhere. Everyone needs the same RflyGame version.",
        vec![
            Item::Button(format!("Player name: {}", s.player)),
            Item::Button(format!("Join by address: {}", if s.last_server.is_empty() { "<enter>" } else { &s.last_server })),
            Item::Button("Host on this Mac (forward UDP 27150 on your router)".into()),
            Item::Info("Or run rflygame-server on a cloud server and everyone joins its address.".into()),
            Item::Info("Phone hotspots and most mobile data cannot host: use a server.".into()),
            Item::Button("Back".into()),
        ],
    )
}

const WIZ: [(&str, &str); 4] = [
    ("THROTTLE", "Push the THROTTLE stick fully UP"),
    ("YAW", "Push the YAW (rudder) stick fully RIGHT"),
    ("PITCH", "Push the PITCH (elevator) stick fully UP / forward"),
    ("ROLL", "Push the ROLL (aileron) stick fully RIGHT"),
];

fn backdrop(r: &Renderer, map: &Map, model: &VehicleModel, t: f32) {
    let p = vec3(0.0, 0.0, -2.5 - (t * 0.8).sin() * 0.3);
    let eye = p + vec3((t * 0.15).cos() * 7.0, (t * 0.15).sin() * 7.0, -1.8);
    let cam = CamPose::look_at(eye, p, 1.4, screen_width() / screen_height().max(1.0));
    let dv = DrawVehicle { model, kind: ModelKind::Multirotor(4), pos: p, q: Quat::from_rotation_z(t * 0.3), scale: Vec3::ONE, prop_angle: t * 30.0, spinning: true, seg_id: 1 };
    r.render(map, &[dv], &[], &cam, None, 0.0);
    set_default_camera();
    draw_rectangle(0.0, 0.0, screen_width(), screen_height(), Color::new(0.0, 0.0, 0.0, 0.25));
}

fn color_u32(c: u32) -> Color {
    Color::from_rgba((c >> 24) as u8, (c >> 16) as u8, (c >> 8) as u8, 255)
}

fn player_color(name: &str) -> u32 {
    let h = name.bytes().fold(2166136261u32, |h, b| (h ^ b as u32).wrapping_mul(16777619));
    let c = seg_color(h);
    ((c.r * 255.0) as u32) << 24 | ((c.g * 255.0) as u32) << 16 | ((c.b * 255.0) as u32) << 8 | 0xff
}

struct Mp {
    client: Client,
    server_stop: Option<Arc<AtomicBool>>,
    lobby: Menu,
    seen: Option<(NetAct, Option<Instant>)>,
    last_send: f64,
    /// Index into FlightMode::ALL chosen in the lobby (default Angle → FPV camera).
    mode: usize,
}

#[macroquad::main(window_conf)]
async fn main() {
    let args: Vec<String> = std::env::args().collect();
    let arg = |k: &str| args.iter().position(|a| a == k).and_then(|i| args.get(i + 1)).cloned();
    let shot = arg("--screenshot");
    let shot_after: f32 = arg("--after").and_then(|v| v.parse().ok()).unwrap_or(3.0);
    let auto = arg("--auto"); // free | race | mission | school | host | join=ADDR
    let mut settings = Settings::load();
    // Daily update check in the background (never during automated runs).
    let update_rx = if (auto.is_none() && arg("--screenshot").is_none()) || std::env::var_os("RFLY_UPDATE_URL").is_some() { Some(rfly_update::spawn_check()) } else { None };
    if let Some(n) = arg("--name") {
        settings.player = n;
    }
    let mut input = Input::new();
    // Virtual controllers for scripts/tests: --vpad [port] (default 27170, localhost only).
    if let Some(i) = args.iter().position(|a| a == "--vpad") {
        let port = args.get(i + 1).and_then(|p| p.parse().ok()).unwrap_or(rflygame::input::VPAD_PORT);
        match input.enable_virtual_pads(port) {
            Ok(p) => println!("rflygame: virtual controllers on 127.0.0.1:{p}"),
            Err(e) => eprintln!("rflygame: cannot open virtual controller port {port}: {e}"),
        }
    }
    let mut audio = rflygame::sound::Audio::new(auto.is_some() && std::env::var("RFLY_SOUND").is_err()).await;
    let r = Renderer::new();
    let bg_map = build_map("Grasslands");
    let bg_model = build_model(ModelKind::Multirotor(4), Color::new(0.95, 0.55, 0.15, 1.0));
    let mut drones = garage::all_drones();
    let mut screen = Screen::Main;
    let mut menu = main_menu();
    let mut toast: Option<(String, f64)> = None;
    let mut session: Option<Session> = None;
    let mut act = Act::Free;
    let mut last_setup: Option<Setup> = None;
    let mut spring = true;
    let mut browser: Option<Browser> = None;
    let mut mp: Option<Mp> = None;
    let mut auto_done = false;
    let mut quit = false;
    let quit_after: f32 = arg("--quit-after").and_then(|v| v.parse().ok()).unwrap_or(f32::MAX);
    let t0 = get_time();

    let start = |setup: &Setup, settings: &Settings| -> (Session, Act) {
        match setup.clone() {
            Setup::Free(map, d, m) => (Session::new(&map, d, m, settings), Act::Free),
            Setup::Race(i, d, m) => {
                let tr = tracks().remove(i);
                let mut s = Session::new(tr.map, d, m, settings);
                let race = Race::new(tr, &mut s);
                (s, Act::Race(race))
            }
            Setup::Mission(i, d, m) => {
                let def = missions().remove(i);
                let mut s = Session::new(def.map, d, m, settings);
                let mi = Mission::new(def, &mut s);
                (s, Act::Mission(mi))
            }
            Setup::School(i, d) => {
                let mut s = Session::new("Grasslands", d, FlightMode::Beginner, settings);
                let sc = School::new(i, &mut s);
                (s, Act::School(sc))
            }
        }
    };

    // Automated entry points (screenshots / CI).
    if let Some(a) = &auto {
        let d0 = drones[0].clone();
        let (kind, idx) = a.split_once('=').map_or((a.as_str(), None), |(k, v)| (k, v.parse::<usize>().ok()));
        let setup = match kind {
            "race" => Some(Setup::Race(idx.unwrap_or(3), d0.clone(), FlightMode::Beginner)),
            "mission" => Some(Setup::Mission(idx.unwrap_or(0), d0.clone(), FlightMode::Beginner)),
            "school" => Some(Setup::School(idx.unwrap_or(0), d0.clone())),
            "free" | "collide" | "bump" | "drop" | "flip" => Some(Setup::Free("City".into(), d0.clone(), FlightMode::Beginner)),
            _ => None,
        };
        if let Some(su) = setup {
            let (mut s, a2) = start(&su, &settings);
            if std::env::var("RFLY_FPV").is_ok() {
                s.cam = rflygame::game::CamMode::Fpv;
            }
            if kind == "bump" {
                s.pilot.rates.max_speed = 1.5;
            }
            if kind == "flip" {
                // Upside down, 2 m up, motors off: it must land on its back, then turtle over.
                s.v.s.att = rfly_core::Quat::from_euler(std::f64::consts::PI, 0.0, 0.0);
                s.v.s.pos.z = -0.4;
            }
            session = Some(s);
            act = a2;
            last_setup = Some(su);
            screen = Screen::Playing;
        } else if kind == "split" {
            let act = std::env::var("RFLY_SPLIT_ACT").ok().and_then(|v| v.parse().ok()).unwrap_or(0);
            let au = rflygame::split::Auto { players: idx.unwrap_or(2), activity: act, screenshot: shot.clone(), after: shot_after };
            rflygame::split::run(&r, &settings, &mut input, &d0, &mut audio, Some(au)).await;
            return;
        } else if a == "host" || a.starts_with("join=") {
            let addr = if a == "host" { None } else { Some(a[5..].to_string()) };
            let stop = Arc::new(AtomicBool::new(false));
            let (server, stop) = match addr {
                Some(x) => (x, None),
                None => {
                    let port = net::spawn_server(net::ServerConfig { port: net::DEFAULT_PORT, name: format!("{}'s game", settings.player), lan_beacon: true }, stop.clone()).unwrap_or(net::DEFAULT_PORT);
                    (format!("127.0.0.1:{port}"), Some(stop))
                }
            };
            if let Ok(c) = Client::connect(&server, &settings.player, player_color(&settings.player), &d0.name) {
                mp = Some(Mp { client: c, server_stop: stop, lobby: Menu::new("Lobby", "", vec![]), seen: None, last_send: 0.0, mode: 1 });
                screen = Screen::Lobby;
            }
        }
    }

    loop {
        let dt = get_frame_time();
        input.deadzone = settings.deadzone;
        input.invert_pitch = settings.invert_pitch;
        input.kb_mode2 = settings.kb_mode2;
        input.map = settings.map.clone();
        let editing = matches!(screen, Screen::TextEdit { .. });
        let (mut sticks, mut a) = if editing { (Sticks::default(), Default::default()) } else { input.poll(dt, spring) };
        let tt = (get_time() - t0) as f32;

        // Autopilot for automated runs.
        if auto.is_some() && matches!(screen, Screen::Playing) {
            if let Some(s) = session.as_mut() {
                s.max_steps = 2000;
                let dropping = (auto.as_deref() == Some("drop") && s.agl() > 1.0) || auto.as_deref() == Some("flip");
                let tumbled = matches!(auto.as_deref(), Some("collide") | Some("bump") | Some("drop") | Some("flip")) && (s.impact.is_some() || s.inverted);
                if s.inverted && !s.turtle && tumbled && (tt * 2.0).fract() < 0.05 {
                    a.arm = true; // try turtle mode
                } else if !s.armed && !s.locked && s.crashed.is_none() && tt > 0.5 && !dropping && !tumbled {
                    a.arm = true;
                }
                let p0 = s.pos();
                let tgt = match &act {
                    Act::School(sc) => match sc.current() {
                        Some(rflygame::school::Check::Alt(lo, hi)) => Some(vec3(p0.x, p0.y, -(lo + hi) / 2.0)),
                        Some(rflygame::school::Check::Near(c, _)) | Some(rflygame::school::Check::Visit(c, _)) => Some(c),
                        Some(rflygame::school::Check::Land) => Some(if vec2(p0.x, p0.y).length() > 1.0 { vec3(0.0, 0.0, p0.z.min(-2.0)) } else { vec3(0.0, 0.0, 1.0) }),
                        Some(rflygame::school::Check::Yaw(_)) => {
                            sticks = Sticks { yaw: 0.7, ..Default::default() };
                            None
                        }
                        Some(rflygame::school::Check::Mode(m)) => {
                            if s.pilot.mode != m && (tt * 2.0).fract() < 0.05 {
                                a.mode = true;
                            }
                            if m == FlightMode::Acro {
                                sticks.throttle = -1.0;
                            }
                            None
                        }
                        _ => Some(vec3(p0.x, p0.y, -3.0)),
                    },
                    Act::Race(r) => r.auto_target(s.pos()),
                    Act::Mission(m) if m.def.name == "Follow the Car" => m.target(s).map(|p| {
                        let car_dir = s.others.first().map_or(Vec3::X, |o| o.q * Vec3::X);
                        p + car_dir * 3.0 - vec3(0.0, 0.0, 4.0)
                    }),
                    Act::Mission(m) if m.def.name == "Windy Hover" || m.def.name == "City Run" => m.target(s),
                    Act::Mission(m) => m.target(s).map(|p| if vec2(p.x - s.pos().x, p.y - s.pos().y).length() > 0.6 { p - vec3(0.0, 0.0, 3.0) } else { p + vec3(0.0, 0.0, 1.0) }),
                    _ => match auto.as_deref() {
                        // Fly into the tower east of the start (Building_1_0 at x = 40).
                        Some("collide") | Some("bump") => Some(vec3(60.0, 0.0, -5.0)),
                        Some("drop") => {
                            if s.agl() > 14.5 && s.armed {
                                s.armed = false; // motors off at 15 m
                                s.msg("Motors cut!", 2.0);
                            }
                            if s.crashed.is_none() && !s.armed && s.agl() > 1.0 { None } else { Some(vec3(0.0, 0.0, -16.0)) }
                        }
                        _ => Some(vec3(15.0 * (tt * 0.2).cos(), 15.0 * (tt * 0.2).sin(), -6.0)),
                    },
                };
                if let Some(t) = tgt {
                    sticks = autopilot(s, t);
                    if std::env::var("RFLY_DEBUG").is_ok() && (s.t * 2.0).fract() < 0.02 {
                        let prog = if let Act::Race(rc) = &act { rc.progress() as i32 } else { -1 };
                        eprintln!("t={:.1} pos=({:.1},{:.1},{:.1}) tgt=({:.1},{:.1},{:.1}) prog={prog} sticks=({:.2},{:.2},{:.2})", s.t, s.pos().x, s.pos().y, s.pos().z, t.x, t.y, t.z, sticks.pitch, sticks.yaw, sticks.throttle);
                    }
                }
            }
        }

        // Multiplayer networking (all screens).
        if let Some(m) = mp.as_mut() {
            m.client.poll();
        }
        if let Some(b) = browser.as_mut() {
            b.poll();
        }

        if let Some(v) = update_rx.as_ref().and_then(|rx| rx.try_recv().ok()) {
            if let Ok(mut u) = UPDATE.lock() {
                *u = Some(v.to_string());
            }
            if matches!(screen, Screen::Main) {
                let sel = menu.sel;
                menu = main_menu();
                menu.sel = sel;
            }
        }
        match &mut screen {
            Screen::Main => {
                backdrop(&r, &bg_map, &bg_model, tt);
                match menu.run(&a) {
                    MenuEvent::Activate(0) => {
                        menu = setup_menu("Free Flight", "Explore, practise, crash safely.", Item::Choice("Map".into(), MAP_LABELS.iter().map(|s| s.to_string()).collect(), 0), &drones, vec![]);
                        screen = Screen::FreeSetup;
                    }
                    MenuEvent::Activate(1) => {
                        menu = race_menu(&drones, 0);
                        screen = Screen::RaceSetup;
                    }
                    MenuEvent::Activate(2) => {
                        menu = mission_menu(&drones, 0);
                        screen = Screen::MissionSetup;
                    }
                    MenuEvent::Activate(3) => {
                        menu = school_menu();
                        screen = Screen::SchoolSetup;
                    }
                    MenuEvent::Activate(4) => {
                        menu = together_menu();
                        screen = Screen::Together;
                    }
                    MenuEvent::Activate(5) => screen = Screen::Garage(Box::new(Garage::open())),
                    MenuEvent::Activate(6) => {
                        let keys = records::keys();
                        let mut items = vec![Item::Choice("Board".into(), if keys.is_empty() { vec!["(none yet)".into()] } else { keys.clone() }, 0)];
                        items.extend((0..10).map(|_| Item::Info(String::new())));
                        items.push(Item::Button("Back".into()));
                        menu = Menu::new("Leaderboards", "Local best times and scores.", items);
                        screen = Screen::Boards;
                    }
                    MenuEvent::Activate(7) => {
                        let list = records::list_replays();
                        let mut items: Vec<Item> = list.iter().map(|(_, l, m)| Item::Button(format!("{l}  ({m})"))).collect();
                        if items.is_empty() {
                            items.push(Item::Info("No replays yet: finish a race or mission.".into()));
                        }
                        items.push(Item::Button("Back".into()));
                        menu = Menu::new("Replays", "Watch your recorded runs.", items);
                        screen = Screen::Replays(list);
                    }
                    MenuEvent::Activate(8) => {
                        menu = controls_menu(&settings, &input);
                        screen = Screen::Controls;
                    }
                    MenuEvent::Activate(9) => {
                        menu = settings_menu(&settings);
                        screen = Screen::Settings;
                    }
                    MenuEvent::Activate(10) => break,
                    MenuEvent::Activate(MENU_UPDATE) => {
                        if let Err(e) = rfly_update::open_updater_terminal() {
                            eprintln!("rflygame: {e}");
                        }
                    }
                    _ => {}
                }
            }
            Screen::FreeSetup | Screen::RaceSetup | Screen::MissionSetup => {
                backdrop(&r, &bg_map, &bg_model, tt);
                let d = drones[menu.choice(1).min(drones.len() - 1)].clone();
                ui::centered(&d.desc, screen_height() - 50.0, 20.0, WHITE);
                ui::centered(&format!("thrust/weight {:.1}   top speed ~{:.0} m/s   flight time ~{:.0} min", d.thrust_to_weight(), d.top_speed(), d.hover_minutes()), screen_height() - 24.0, 18.0, ui::DIM);
                let ev = menu.run(&a);
                let n = menu.items.len();
                let mode = FlightMode::ALL[menu.choice(2)];
                match ev {
                    MenuEvent::Changed(0) => {
                        let k = menu.choice(0);
                        let d1 = menu.choice(1);
                        let m2 = menu.choice(2);
                        if matches!(screen, Screen::RaceSetup) {
                            menu = race_menu(&drones, k);
                        } else if matches!(screen, Screen::MissionSetup) {
                            menu = mission_menu(&drones, k);
                        }
                        if let Item::Choice(_, _, x) = &mut menu.items[1] {
                            *x = d1;
                        }
                        if let Item::Choice(_, _, x) = &mut menu.items[2] {
                            *x = m2;
                        }
                    }
                    MenuEvent::Activate(i) if i == n - 2 => {
                        let k = menu.choice(0);
                        let su = match screen {
                            Screen::FreeSetup => Setup::Free(MAP_LABELS[k].into(), d, mode),
                            Screen::RaceSetup => Setup::Race(k, d, mode),
                            _ => Setup::Mission(k, d, mode),
                        };
                        let (s, a2) = start(&su, &settings);
                        session = Some(s);
                        act = a2;
                        last_setup = Some(su);
                        screen = Screen::Playing;
                    }
                    MenuEvent::Activate(i) if i == n - 1 => {
                        menu = main_menu();
                        screen = Screen::Main;
                    }
                    MenuEvent::Back => {
                        menu = main_menu();
                        screen = Screen::Main;
                    }
                    _ => {}
                }
            }
            Screen::SchoolSetup => {
                backdrop(&r, &bg_map, &bg_model, tt);
                let ls = lessons();
                if menu.sel < ls.len() {
                    ui::centered(ls[menu.sel].desc, screen_height() - 40.0, 22.0, WHITE);
                }
                match menu.run(&a) {
                    MenuEvent::Activate(i) if i < ls.len() => {
                        let su = Setup::School(i, drones[0].clone());
                        let (s, a2) = start(&su, &settings);
                        session = Some(s);
                        act = a2;
                        last_setup = Some(su);
                        screen = Screen::Playing;
                    }
                    MenuEvent::Activate(_) | MenuEvent::Back => {
                        menu = main_menu();
                        screen = Screen::Main;
                    }
                    _ => {}
                }
            }
            Screen::Playing => {
                let s = session.as_mut().unwrap();
                if a.pause {
                    let restart = if mp.is_some() { "Leave multiplayer" } else { "Restart" };
                    menu = Menu::new(
                        "Paused",
                        &format!("Air time {:.1}s, top speed {:.1} m/s", s.flight_time, s.max_speed),
                        vec![Item::Button("Resume".into()), Item::Button(restart.into()), Item::Button("Save flight log (CSV)".into()), Item::Button("Settings".into()), Item::Button("Quit to menu".into())],
                    );
                    screen = Screen::Paused;
                } else {
                    s.update(sticks, &a, dt, &mut spring);
                    let player = settings.player.clone();
                    let out = match &mut act {
                        Act::Free => None,
                        Act::Race(rc) => rc.update(s, dt, &player),
                        Act::Mission(mi) => mi.update(s, dt, &player),
                        Act::School(sc) => sc.update(s, dt),
                    };
                    // Multiplayer: publish our state, show the others.
                    if let Some(m) = mp.as_mut() {
                        if s.t - m.last_send > 0.05 {
                            m.last_send = s.t;
                            let q = s.quat();
                            let p = s.pos();
                            let (progress, finish) = match &act {
                                Act::Race(rc) => (rc.progress(), rc.finished().map_or(-1.0, |f| f as f32)),
                                _ => (0, -1.0),
                            };
                            m.client.send_state(&DroneState { pos: p.into(), quat: [q.x, q.y, q.z, q.w], spin: s.spin(), progress, finish });
                        }
                        s.others.retain(|o| o.name == "Best run");
                        for pl in m.client.players.iter().filter(|p| Some(p.info.id) != m.client.id) {
                            let st = m.client.view(pl);
                            s.others.push(OtherDrone {
                                name: pl.info.name.clone(),
                                pos: Vec3::from(st.pos),
                                q: Quat::from_xyzw(st.quat[0], st.quat[1], st.quat[2], st.quat[3]).normalize(),
                                spin: st.spin,
                                color: color_u32(pl.info.color),
                                kind: ModelKind::Multirotor(4),
                                scale: 1.0,
                            });
                        }
                        // Activity changed by the host → rebuild.
                        let cur = (m.client.activity.clone(), m.client.start_at);
                        if m.seen.as_ref() != Some(&cur) {
                            m.seen = Some(cur.clone());
                            if let NetAct::Lobby = cur.0 {
                                screen = Screen::Lobby;
                            }
                        }
                        if let (Act::Race(rc), Some(st)) = (&mut act, m.client.start_at) {
                            rc.set_countdown(st.saturating_duration_since(Instant::now()).as_secs_f64());
                        }
                    }
                    if let Some(o) = out {
                        if mp.is_none() {
                            screen = Screen::Results(o);
                        } else {
                            s.msg(o.title.clone(), 5.0);
                        }
                    }
                }
                if let Some(s) = session.as_mut() {
                    let cam = s.draw(&r, &settings);
                    s.hud(&settings, &input.device_name, &cam);
                    match &act {
                        Act::Race(rc) => rc.hud(s, &cam),
                        Act::Mission(mi) => mi.hud(s, &cam),
                        Act::School(sc) => sc.hud(s, &cam),
                        Act::Free => {}
                    }
                    if let Some(m) = &mp {
                        let mut ps: Vec<_> = m.client.players.iter().collect();
                        ps.sort_by(|x, y| {
                            let fx = if x.state.finish >= 0.0 { x.state.finish } else { f32::MAX };
                            let fy = if y.state.finish >= 0.0 { y.state.finish } else { f32::MAX };
                            fx.total_cmp(&fy).then(y.state.progress.cmp(&x.state.progress))
                        });
                        let y0 = 180.0;
                        draw_rectangle(10.0, y0, 320.0, 30.0 + ps.len() as f32 * 24.0, ui::PANEL);
                        let ping = m.client.rtt_ms.map_or(String::new(), |r| format!("  {r:.0} ms"));
                        let head = if m.client.connected() { format!("{} ({} online){ping}", m.client.server_name, ps.len()) } else { "Connection lost - reconnecting...".into() };
                        draw_text(&head, 20.0, y0 + 22.0, 20.0, if m.client.connected() { ui::ACCENT } else { YELLOW });
                        for (i, p) in ps.iter().enumerate() {
                            let racing = matches!(act, Act::Race(_));
                            let fin = if !racing {
                                "flying".to_string()
                            } else if p.state.finish >= 0.0 {
                                format!("{:.2}s", p.state.finish)
                            } else {
                                format!("gate {}", p.state.progress)
                            };
                            draw_text(&format!("{}. {}  {}", i + 1, p.info.name, fin), 20.0, y0 + 46.0 + i as f32 * 24.0, 20.0, color_u32(p.info.color));
                        }
                    }
                }
            }
            Screen::Paused => {
                let s = session.as_mut().unwrap();
                s.draw(&r, &settings);
                set_default_camera();
                match menu.run(&a) {
                    MenuEvent::Activate(0) | MenuEvent::Back => screen = Screen::Playing,
                    MenuEvent::Activate(1) => {
                        if mp.is_some() {
                            if let Some(m) = mp.take() {
                                if let Some(st) = m.server_stop {
                                    st.store(true, Ordering::Relaxed);
                                }
                            }
                            session = None;
                            menu = main_menu();
                            screen = Screen::Main;
                        } else if let Some(su) = &last_setup {
                            let (s2, a2) = start(su, &settings);
                            session = Some(s2);
                            act = a2;
                            screen = Screen::Playing;
                        }
                    }
                    MenuEvent::Activate(2) => {
                        let tag = last_setup.as_ref().map_or("flight", |s| match s {
                            Setup::Free(..) => "free",
                            Setup::Race(..) => "race",
                            Setup::Mission(..) => "mission",
                            Setup::School(..) => "school",
                        });
                        toast = Some((s.save_log(tag).map_or("Could not save the log".into(), |p| format!("Saved {}", p.display())), get_time() + 4.0));
                    }
                    MenuEvent::Activate(3) => {
                        menu = settings_menu(&settings);
                        screen = Screen::Settings;
                    }
                    MenuEvent::Activate(4) => {
                        if let Some(m) = mp.take() {
                            if let Some(st) = m.server_stop {
                                st.store(true, Ordering::Relaxed);
                            }
                        }
                        session = None;
                        menu = main_menu();
                        screen = Screen::Main;
                    }
                    _ => {}
                }
            }
            Screen::Results(o) => {
                if let Some(s) = session.as_mut() {
                    s.draw(&r, &settings);
                }
                set_default_camera();
                let mut items: Vec<Item> = o.lines.iter().map(|l| Item::Info(l.clone())).collect();
                if !o.board_key.is_empty() {
                    items.push(Item::Info(String::new()));
                    items.extend(board_lines(&o.board_key, o.lower_is_better).into_iter().map(Item::Info));
                }
                let n = items.len();
                items.push(Item::Button("Retry".into()));
                items.push(Item::Button("Watch replay".into()));
                items.push(Item::Button("Main menu".into()));
                if menu.title != o.title {
                    audio.play(if o.success { rflygame::game::Sfx::Win } else { rflygame::game::Sfx::Lose }, settings.volume);
                    menu = Menu::new(&o.title, if o.success { "Well flown!" } else { "Not this time." }, items);
                    menu.sel = n;
                }
                if auto.is_some() && !auto_done {
                    auto_done = true;
                    println!("rflygame auto: {} | {}", o.title, o.lines.join(" | "));
                    if shot.is_none() {
                        quit = true;
                    }
                }
                match menu.run(&a) {
                    MenuEvent::Activate(i) if i == n => {
                        if let Some(su) = &last_setup {
                            let (s2, a2) = start(su, &settings);
                            session = Some(s2);
                            act = a2;
                            screen = Screen::Playing;
                        }
                    }
                    MenuEvent::Activate(i) if i == n + 1 => {
                        if let Some(s) = &session {
                            screen = Screen::Watch { map: build_map(&s.map.name), samples: s.recording.clone(), t: 0.0, paused: false, orbit: false };
                        }
                    }
                    MenuEvent::Activate(_) | MenuEvent::Back => {
                        session = None;
                        menu = main_menu();
                        screen = Screen::Main;
                    }
                    _ => {}
                }
            }
            Screen::Boards => {
                backdrop(&r, &bg_map, &bg_model, tt);
                let keys = records::keys();
                if let Some(k) = keys.get(menu.choice(0)) {
                    let lower = k.starts_with("Race");
                    let top = records::top(k, lower, 10);
                    for i in 0..10 {
                        menu.items[1 + i] = Item::Info(top.get(i).map_or(String::new(), |e| format!("{:>2}. {:<14} {:>9.2}  {}  {}  {}", i + 1, e.player, e.value, e.drone, e.mode, &e.date[..8.min(e.date.len())])));
                    }
                }
                match menu.run(&a) {
                    MenuEvent::Activate(_) | MenuEvent::Back => {
                        menu = main_menu();
                        screen = Screen::Main;
                    }
                    _ => {}
                }
            }
            Screen::Replays(list) => {
                backdrop(&r, &bg_map, &bg_model, tt);
                match menu.run(&a) {
                    MenuEvent::Activate(i) if i < list.len() => {
                        let (p, _, m) = list[i].clone();
                        screen = Screen::Watch { map: build_map(&m), samples: records::load_samples(&p), t: 0.0, paused: false, orbit: false };
                    }
                    MenuEvent::Activate(_) | MenuEvent::Back => {
                        menu = main_menu();
                        screen = Screen::Main;
                    }
                    _ => {}
                }
            }
            Screen::Watch { map, samples, t, paused, orbit } => {
                if is_key_pressed(KeyCode::Space) {
                    *paused = !*paused;
                }
                if a.camera {
                    *orbit = !*orbit;
                }
                if is_key_down(KeyCode::Right) {
                    *t += dt * 4.0;
                }
                if is_key_down(KeyCode::Left) {
                    *t = (*t - dt * 4.0).max(0.0);
                }
                if !*paused {
                    *t += dt;
                }
                let t0s = samples.first().map_or(0.0, |s| s.t);
                let tend = samples.last().map_or(0.0, |s| s.t) - t0s;
                if let Some(smp) = records::sample_at(samples, t0s + *t) {
                    let aspect = screen_width() / screen_height().max(1.0);
                    let cam = if *orbit {
                        let e = smp.pos + vec3((tt * 0.3).cos() * 8.0, (tt * 0.3).sin() * 8.0, -3.0);
                        CamPose::look_at(e, smp.pos, 1.3, aspect)
                    } else {
                        let back = smp.q * vec3(-4.0, 0.0, 0.0);
                        CamPose::look_at(smp.pos + vec3(back.x, back.y, -1.5), smp.pos, 1.3, aspect)
                    };
                    let dv = DrawVehicle { model: &bg_model, kind: ModelKind::Multirotor(4), pos: smp.pos, q: smp.q, scale: Vec3::ONE, prop_angle: smp.spin, spinning: true, seg_id: 1 };
                    r.render(map, &[dv], &[], &cam, None, 0.0);
                }
                set_default_camera();
                draw_rectangle(0.0, screen_height() - 60.0, screen_width(), 60.0, ui::PANEL);
                let frac = if tend > 0.0 { (*t / tend).min(1.0) } else { 1.0 };
                draw_rectangle(20.0, screen_height() - 46.0, (screen_width() - 40.0) * frac, 6.0, ui::ACCENT);
                draw_text(&format!("REPLAY {:.1}/{:.1}s   Space pause   Left/Right seek   C camera   Esc back", t, tend), 20.0, screen_height() - 16.0, 20.0, WHITE);
                if a.back {
                    menu = main_menu();
                    screen = Screen::Main;
                }
            }
            Screen::Garage(g) => {
                backdrop(&r, &bg_map, &bg_model, tt);
                if !g.ok {
                    ui::centered("ModelData.db not found - import it from your RflySim ISO (Import RflySim Data).", screen_height() - 40.0, 22.0, ORANGE);
                }
                match g.menu.run(&a) {
                    MenuEvent::Changed(i) => g.changed(i),
                    MenuEvent::Activate(garage::SAVE) | MenuEvent::Activate(garage::SAVE_FLY) => {
                        garage::save_build(&g.build);
                        drones = garage::all_drones();
                        toast = Some((format!("Saved '{}' - it's in every drone list now", g.build.name), get_time() + 3.0));
                        if g.menu.sel == garage::SAVE_FLY {
                            let su = Setup::Free("Grasslands".into(), g.build.drone(), FlightMode::Beginner);
                            let (s, a2) = start(&su, &settings);
                            session = Some(s);
                            act = a2;
                            last_setup = Some(su);
                            screen = Screen::Playing;
                        }
                    }
                    MenuEvent::Activate(garage::BACK) | MenuEvent::Back => {
                        menu = main_menu();
                        screen = Screen::Main;
                    }
                    _ => {}
                }
            }
            Screen::Settings => {
                backdrop(&r, &bg_map, &bg_model, tt);
                let ev = menu.run(&a);
                settings.fov_deg = menu.number(0) as f32;
                settings.fpv_uptilt_deg = menu.number(1) as f32;
                settings.acro_rate = menu.number(2);
                settings.expo = menu.number(3);
                settings.max_angle_deg = menu.number(4);
                settings.show_sticks = menu.toggle(5);
                settings.battery = menu.toggle(6);
                settings.default_cam = menu.choice(7) as u8;
                settings.crash_style = menu.choice(8) as u8;
                settings.kb_mode2 = menu.choice(9) == 0;
                settings.volume = menu.number(10).round() as u8;
                if menu.toggle(11) != rfly_update::enabled() {
                    let _ = rfly_update::set_enabled(menu.toggle(11));
                }
                if let Some(s) = session.as_mut() {
                    s.realistic = settings.crash_style == 0;
                }
                if let Some(s) = session.as_mut() {
                    s.pilot.rates.acro_rate = settings.acro_rate;
                    s.pilot.rates.expo = settings.expo;
                    s.pilot.rates.max_angle_deg = settings.max_angle_deg;
                }
                if matches!(ev, MenuEvent::Activate(12) | MenuEvent::Back) {
                    settings.save();
                    if session.is_some() {
                        screen = Screen::Playing;
                    } else {
                        menu = main_menu();
                        screen = Screen::Main;
                    }
                }
            }
            Screen::Controls => {
                backdrop(&r, &bg_map, &bg_model, tt);
                ui::draw_sticks(&sticks);
                let ev = menu.run(&a);
                settings.invert_pitch = menu.toggle(4);
                settings.deadzone = menu.number(5) as f32;
                match ev {
                    MenuEvent::Activate(6) => screen = Screen::Wizard { step: 0, base: input.raw_axes() },
                    MenuEvent::Activate(7) | MenuEvent::Back => {
                        settings.save();
                        menu = main_menu();
                        screen = Screen::Main;
                    }
                    _ => {}
                }
            }
            Screen::Wizard { step, base } => {
                backdrop(&r, &bg_map, &bg_model, tt);
                let raw = input.raw_axes();
                let (w, h) = (screen_width(), screen_height());
                draw_rectangle(w / 2.0 - 380.0, h / 2.0 - 170.0, 760.0, 340.0, ui::PANEL);
                ui::centered(&format!("Axis setup {}/4: {}", *step + 1, WIZ[*step].0), h / 2.0 - 110.0, 36.0, WHITE);
                ui::centered(WIZ[*step].1, h / 2.0 - 60.0, 26.0, ui::ACCENT);
                ui::centered("hold it there and press Enter  (Esc = cancel)", h / 2.0 - 25.0, 22.0, ui::DIM);
                if raw.is_empty() {
                    ui::centered("No joystick / RC transmitter detected. Plug it in (USB joystick mode).", h / 2.0 + 30.0, 22.0, ORANGE);
                }
                for (i, v) in raw.iter().enumerate().take(8) {
                    let x = w / 2.0 - 300.0 + i as f32 * 76.0;
                    let bh = v.abs() * 60.0;
                    let y = h / 2.0 + 110.0;
                    draw_rectangle(x, if *v > 0.0 { y - bh } else { y }, 40.0, bh, ui::ACCENT);
                    draw_text(&format!("ax{i}"), x, y + 80.0, 18.0, ui::DIM);
                }
                if is_key_pressed(KeyCode::Escape) {
                    menu = controls_menu(&settings, &input);
                    screen = Screen::Controls;
                } else if is_key_pressed(KeyCode::Enter) && !raw.is_empty() {
                    let (idx, dv) = raw.iter().enumerate().map(|(i, v)| (i, v - base.get(i).copied().unwrap_or(0.0))).max_by(|a, b| a.1.abs().total_cmp(&b.1.abs())).unwrap();
                    let bind = AxisBind { index: idx, invert: dv < 0.0 };
                    match *step {
                        0 => settings.map.throttle = bind,
                        1 => settings.map.yaw = bind,
                        2 => settings.map.pitch = bind,
                        _ => settings.map.roll = bind,
                    }
                    if *step == 3 {
                        settings.save();
                        toast = Some(("Joystick axes saved".into(), get_time() + 2.5));
                        menu = controls_menu(&settings, &input);
                        screen = Screen::Controls;
                    } else {
                        *step += 1;
                        *base = raw;
                    }
                }
            }
            Screen::Together => {
                backdrop(&r, &bg_map, &bg_model, tt);
                match menu.run(&a) {
                    MenuEvent::Activate(0) => {
                        rflygame::split::run(&r, &settings, &mut input, &drones[0], &mut audio, None).await;
                        menu = together_menu();
                    }
                    MenuEvent::Activate(1) => {
                        browser = Some(Browser::new());
                        menu = mp_menu(&settings, browser.as_ref().unwrap());
                        screen = Screen::MpMenu;
                    }
                    MenuEvent::Activate(2) => {
                        menu = online_menu(&settings);
                        screen = Screen::Online;
                    }
                    MenuEvent::Activate(_) | MenuEvent::Back => {
                        menu = main_menu();
                        screen = Screen::Main;
                    }
                    _ => {}
                }
            }
            Screen::Online => {
                backdrop(&r, &bg_map, &bg_model, tt);
                match menu.run(&a) {
                    MenuEvent::Activate(0) => screen = Screen::TextEdit { field: 2, buf: settings.player.clone() },
                    MenuEvent::Activate(1) => screen = Screen::TextEdit { field: 1, buf: settings.last_server.clone() },
                    MenuEvent::Activate(2) => {
                        let stop = Arc::new(AtomicBool::new(false));
                        match net::spawn_server(net::ServerConfig { port: net::DEFAULT_PORT, name: format!("{}'s game", settings.player), lan_beacon: false }, stop.clone()) {
                            Ok(port) => {
                                let d = &drones[0];
                                if let Ok(c) = Client::connect(&format!("127.0.0.1:{port}"), &settings.player, player_color(&settings.player), &d.name) {
                                    mp = Some(Mp { client: c, server_stop: Some(stop), lobby: Menu::new("Lobby", "", vec![]), seen: None, last_send: 0.0, mode: 1 });
                                    screen = Screen::Lobby;
                                }
                            }
                            Err(e) => toast = Some((format!("Cannot host on UDP {}: {e}", net::DEFAULT_PORT), get_time() + 3.0)),
                        }
                    }
                    MenuEvent::Activate(_) | MenuEvent::Back => {
                        menu = together_menu();
                        screen = Screen::Together;
                    }
                    _ => {}
                }
            }
            Screen::MpMenu => {
                backdrop(&r, &bg_map, &bg_model, tt);
                let b = browser.as_ref().unwrap();
                let found = b.found.clone();
                let sel = menu.sel;
                let n_items = menu.items.len();
                if n_items != mp_menu(&settings, b).items.len() || (get_time() as i64 % 2 == 0 && !found.is_empty()) {
                    menu = mp_menu(&settings, b);
                    menu.sel = sel.min(menu.items.len() - 1);
                }
                let first_found = if found.is_empty() { 3 } else { 2 };
                let ev = menu.run(&a);
                let n = menu.items.len();
                let connect = |addr: &str, settings: &Settings, stop: Option<Arc<AtomicBool>>| -> Option<Mp> {
                    let d = &garage::all_drones()[0];
                    Client::connect(addr, &settings.player, player_color(&settings.player), &d.name).ok().map(|c| Mp { client: c, server_stop: stop, lobby: Menu::new("Lobby", "", vec![]), seen: None, last_send: 0.0, mode: 1 })
                };
                match ev {
                    MenuEvent::Activate(0) => screen = Screen::TextEdit { field: 0, buf: settings.player.clone() },
                    MenuEvent::Activate(1) => {
                        let stop = Arc::new(AtomicBool::new(false));
                        match net::spawn_server(net::ServerConfig { port: net::DEFAULT_PORT, name: format!("{}'s game", settings.player), lan_beacon: true }, stop.clone()) {
                            Ok(port) => {
                                mp = connect(&format!("127.0.0.1:{port}"), &settings, Some(stop));
                                screen = Screen::Lobby;
                            }
                            Err(e) => toast = Some((format!("Cannot host on UDP {}: {e}", net::DEFAULT_PORT), get_time() + 3.0)),
                        }
                    }
                    MenuEvent::Activate(i) if i >= first_found && i < first_found + found.len() => {
                        mp = connect(&found[i - first_found].addr.to_string(), &settings, None);
                        screen = Screen::Lobby;
                    }
                    MenuEvent::Activate(i) if i == n - 1 => {
                        browser = None;
                        menu = together_menu();
                        screen = Screen::Together;
                    }
                    MenuEvent::Back => {
                        browser = None;
                        menu = together_menu();
                        screen = Screen::Together;
                    }
                    _ => {}
                }
            }
            Screen::TextEdit { field, buf } => {
                backdrop(&r, &bg_map, &bg_model, tt);
                ui::text_box(if *field != 1 { "Player name" } else { "Server address (host or host:port)" }, buf);
                match ui::text_edit(buf, 40) {
                    Some(true) => {
                        if *field == 2 {
                            settings.player = buf.trim().to_string();
                            settings.save();
                            menu = online_menu(&settings);
                            screen = Screen::Online;
                        } else if *field == 0 {
                            settings.player = buf.trim().to_string();
                            settings.save();
                            screen = Screen::MpMenu;
                        } else {
                            settings.last_server = buf.trim().to_string();
                            settings.save();
                            let d = &drones[0];
                            match Client::connect(&settings.last_server, &settings.player, player_color(&settings.player), &d.name) {
                                Ok(c) => {
                                    mp = Some(Mp { client: c, server_stop: None, lobby: Menu::new("Lobby", "", vec![]), seen: None, last_send: 0.0, mode: 1 });
                                    screen = Screen::Lobby;
                                }
                                Err(e) => {
                                    toast = Some((format!("Cannot reach {}: {e}", settings.last_server), get_time() + 3.0));
                                    menu = online_menu(&settings);
                                    screen = Screen::Online;
                                }
                            }
                        }
                        if matches!(screen, Screen::MpMenu) {
                            if let Some(b) = &browser {
                                menu = mp_menu(&settings, b);
                            }
                        }
                    }
                    Some(false) => {
                        if *field == 0 {
                            if let Some(b) = &browser {
                                menu = mp_menu(&settings, b);
                            }
                            screen = Screen::MpMenu;
                        } else {
                            menu = online_menu(&settings);
                            screen = Screen::Online;
                        }
                    }
                    None => {}
                }
            }
            Screen::Lobby => {
                backdrop(&r, &bg_map, &bg_model, tt);
                let Some(m) = mp.as_mut() else {
                    menu = main_menu();
                    screen = Screen::Main;
                    continue;
                };
                let host = m.client.is_host();
                let ts = tracks();
                let mut opts: Vec<String> = MAP_LABELS.iter().map(|x| format!("Free flight: {x}")).collect();
                opts.extend(ts.iter().map(|t| format!("Race: {}", t.name)));
                let roster: Vec<String> = m.client.players.iter().map(|p| format!("{} {}{}", if Some(p.info.id) == m.client.id { ">" } else { " " }, p.info.name, if p.info.host { "  (host)" } else { "" })).collect();
                let rebuild = m.lobby.items.len() != roster.len() + 5 || m.lobby.title.is_empty();
                if rebuild {
                    let sel_opt = m.lobby.items.first().map_or(0, |_| m.lobby.choice(0));
                    let modes: Vec<String> = FlightMode::ALL.iter().map(|f| format!("{}{}", f.name(), if *f == FlightMode::Beginner { " (chase cam)" } else { " (FPV)" })).collect();
                    let mut items = vec![Item::Choice("Activity".into(), opts.clone(), sel_opt), Item::Choice("Your flight mode".into(), modes, m.mode)];
                    items.extend(roster.iter().map(|r| Item::Info(r.clone())));
                    items.push(Item::Button(if host { "Start for everyone".into() } else { "Waiting for the host...".into() }));
                    items.push(Item::Button("Leave".into()));
                    items.push(Item::Info(String::new()));
                    let sel = m.lobby.sel;
                    m.lobby = Menu::new("Lobby", "", items);
                    m.lobby.sel = sel.min(m.lobby.items.len() - 1);
                }
                m.lobby.subtitle = if m.client.connected() { format!("{} - {} players{}", m.client.server_name, m.client.players.len(), m.client.rtt_ms.map_or(String::new(), |r| format!(" - ping {r:.0} ms"))) } else { format!("Connecting to {} ...", m.client.server) };
                let ev = m.lobby.run(&a);
                m.mode = m.lobby.choice(1);
                let start_i = roster.len() + 2;
                let mut go_main = false;
                match ev {
                    MenuEvent::Activate(i) if i == start_i && host => {
                        let k = m.lobby.choice(0);
                        let act_n = if k < MAP_LABELS.len() { NetAct::Free(MAP_LABELS[k].into()) } else { NetAct::Race(ts[k - MAP_LABELS.len()].name.into()) };
                        m.client.host_cmd(&act_n);
                    }
                    MenuEvent::Activate(i) if i == start_i + 1 => go_main = true,
                    MenuEvent::Back => go_main = true,
                    _ => {}
                }
                if auto.as_deref() == Some("host") && host && tt > std::env::var("RFLY_MP_WAIT").ok().and_then(|v| v.parse().ok()).unwrap_or(3.0) && m.client.players.len() >= 2 && m.client.activity == NetAct::Lobby {
                    let a = match std::env::var("RFLY_MP_RACE") {
                        Ok(t) => NetAct::Race(t),
                        Err(_) => NetAct::Free("Raceway".into()),
                    };
                    m.client.host_cmd(&a);
                }
                // The host started something → go fly it.
                let cur = (m.client.activity.clone(), m.client.start_at);
                if cur.0 != NetAct::Lobby && m.seen.as_ref() != Some(&cur) {
                    m.seen = Some(cur.clone());
                    let d = drones[0].clone();
                    let su = match &cur.0 {
                        NetAct::Free(map) => Setup::Free(map.clone(), d, FlightMode::ALL[m.mode]),
                        NetAct::Race(t) => Setup::Race(ts.iter().position(|x| x.name == t).unwrap_or(0), d, FlightMode::ALL[m.mode]),
                        NetAct::Lobby => unreachable!(),
                    };
                    let (mut s, a2) = start(&su, &settings);
                    // Spread players out on the start line.
                    let k = m.client.players.iter().position(|p| Some(p.info.id) == m.client.id).unwrap_or(0) as f64;
                    let sp = s.spawn;
                    s.set_spawn(sp.0.x - (k * 2.5), sp.0.y + (k % 2.0) * 2.0, sp.1.to_degrees());
                    session = Some(s);
                    act = a2;
                    last_setup = Some(su);
                    screen = Screen::Playing;
                }
                if go_main {
                    if let Some(m) = mp.take() {
                        if let Some(st) = m.server_stop {
                            st.store(true, Ordering::Relaxed);
                        }
                    }
                    menu = main_menu();
                    screen = Screen::Main;
                }
            }
        }

        if let Some((m, until)) = &toast {
            if get_time() < *until {
                ui::centered(m, screen_height() - 90.0, 24.0, YELLOW);
            } else {
                toast = None;
            }
        }
        if let Some(path) = &shot {
            if tt > shot_after {
                get_screen_data().export_png(path);
                if let (Some(s), Some(a)) = (&session, &auto) {
                    let mp_info = mp.as_ref().map_or(String::new(), |m| format!(" players={} others={}", m.client.players.len(), s.others.len()));
                    println!("rflygame auto={a}: fps={} turtle={} touching={} grounded={} inverted={} armed={} impact={:?} agl={:.2} t={:.1} pos=({:.1},{:.1},{:.1}) crashed={:?}{mp_info}", get_fps(), s.turtle, s.touching, s.grounded, s.inverted, s.armed, s.impact.as_ref().map(|i| i.0.clone()), s.agl(), s.t, s.pos().x, s.pos().y, s.pos().z, s.crashed);
                }
                break;
            }
        }
        if quit || tt > quit_after {
            if let (Some(s), Some(a)) = (&session, &auto) {
                println!("rflygame auto={a}: quit at t={:.1} pos=({:.1},{:.1},{:.1}) crashed={:?}", s.t, s.pos().x, s.pos().y, s.pos().z, s.crashed);
            }
            break;
        }
        audio.update(session.as_mut(), matches!(screen, Screen::Playing), settings.volume);
        rflysim3d::record::tick();
        next_frame().await;
    }
}
