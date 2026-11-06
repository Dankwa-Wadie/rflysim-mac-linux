//! Split screen: up to four pilots on one Mac.
//!
//! Each player flies their own `Session` and sees the others as live drones. Each view and
//! its HUD are drawn into an off-screen target slightly larger than the screen area, then
//! scaled down, so the HUD shrinks to fit. Layout: 2 players top/bottom; 3–4 in quarters;
//! with 3 the spare quarter shows an overhead map and the standings.
//! Players join on the join screen with their own controller or half of the keyboard.

use crate::flight::{DronePreset, FlightMode, Sticks};
use crate::game::{autopilot, OtherDrone, Session};
use crate::input::{Actions, Input, Slot, SlotState};
use crate::race::{tracks, Race};
use crate::settings::Settings;
use crate::sound::Audio;
use crate::ui::{self, Item, Menu, MenuEvent};
use macroquad::prelude::*;
use rflysim3d::render::{CamPose, DrawVehicle, Renderer};
use rflysim3d::scene::*;

const MAPS: [&str; 4] = ["Grasslands", "Raceway", "City", "Desert"];
pub const COLORS: [Color; 4] = [
    Color::new(0.98, 0.55, 0.15, 1.0),
    Color::new(0.25, 0.60, 1.00, 1.0),
    Color::new(0.35, 0.85, 0.35, 1.0),
    Color::new(0.90, 0.35, 0.85, 1.0),
];

/// Automated run options (`--auto split=N`).
pub struct Auto {
    /// Pre-joined autopilot pilots; 0 = run the real join screen (e.g. virtual controllers).
    pub players: usize,
    /// Activity index in the join menu (maps first, then tracks).
    pub activity: usize,
    pub screenshot: Option<String>,
    pub after: f32,
}

struct Pilot {
    slot: Slot,
    st: SlotState,
    mode: usize,
    name: String,
}

struct Player {
    p: Pilot,
    s: Session,
    race: Option<Race>,
    spring: bool,
    done: Option<f64>,
    rt: Option<(RenderTarget, u32, u32)>,
}

fn activities() -> Vec<String> {
    let mut v: Vec<String> = MAPS.iter().map(|m| format!("Free flight: {m}")).collect();
    v.extend(tracks().iter().map(|t| format!("Race: {}", t.name)));
    v
}

/// Screen areas for n players (window pixels). Index n is the spare quarter when n == 3.
pub fn layout(n: usize, w: f32, h: f32) -> Vec<Rect> {
    let (hw, hh) = (w / 2.0, h / 2.0);
    match n {
        0 | 1 => vec![Rect::new(0.0, 0.0, w, h)],
        2 => vec![Rect::new(0.0, 0.0, w, hh), Rect::new(0.0, hh, w, hh)],
        _ => vec![Rect::new(0.0, 0.0, hw, hh), Rect::new(hw, 0.0, hw, hh), Rect::new(0.0, hh, hw, hh), Rect::new(hw, hh, hw, hh)],
    }
}

/// Off-screen resolution relative to the screen area: >1 makes the HUD smaller.
fn hud_scale(n: usize) -> f32 {
    match n {
        0 | 1 => 1.0,
        2 => 1.3,
        _ => 1.6,
    }
}

/// Keyboard and every controller, for menus.
fn menu_actions(input: &Input, pads: &mut [(Slot, SlotState)]) -> Actions {
    let mut a = Actions {
        up: is_key_pressed(KeyCode::Up),
        down: is_key_pressed(KeyCode::Down),
        left: is_key_pressed(KeyCode::Left),
        right: is_key_pressed(KeyCode::Right),
        select: is_key_pressed(KeyCode::Enter) || is_key_pressed(KeyCode::KpEnter),
        back: is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::Backspace),
        ..Default::default()
    };
    for (slot, st) in pads.iter_mut() {
        let (_, b) = input.poll_slot(*slot, st, 0.0, true);
        a.up |= b.up;
        a.down |= b.down;
        a.left |= b.left;
        a.right |= b.right;
        a.select |= b.select;
        a.back |= b.back;
    }
    a
}

/// Runs split screen until the players go back to the main menu.
pub async fn run(r: &Renderer, settings: &Settings, input: &mut Input, drone: &DronePreset, audio: &mut Audio, auto: Option<Auto>) {
    let mut pilots: Vec<Pilot> = vec![];
    let mut menu = Menu::new("Split screen", "", vec![Item::Choice("Activity".into(), activities(), 0), Item::Button("Start".into()), Item::Button("Back".into())]);
    let mut pad_state: Vec<(Slot, SlotState)> = vec![];
    let bg = build_map("Grasslands");
    if let Some(a) = &auto {
        for i in 0..a.players {
            let slot = if i % 2 == 0 { Slot::KbLeft } else { Slot::KbRight };
            pilots.push(Pilot { slot, st: SlotState::new(), mode: 1, name: format!("P{}", i + 1) });
        }
        if let Item::Choice(_, _, x) = &mut menu.items[0] {
            *x = a.activity;
        }
    }
    let mut restart = false;
    loop {
        // ---------------------------------------------------------------- join screen
        let join_shot = std::env::var("RFLY_SPLIT_JOIN").is_ok();
        let prejoined = auto.as_ref().is_some_and(|a| a.players > 0);
        let mut go = restart || (prejoined && !join_shot);
        let tj = get_time();
        while !go {
            input.pump();
            for slot in input.pads() {
                if !pad_state.iter().any(|(s, _)| *s == slot) {
                    pad_state.push((slot, SlotState::new()));
                }
            }
            let mut joined_now = false;
            for slot in input.join_presses(&mut pad_state) {
                if pilots.len() < 4 && !pilots.iter().any(|p| p.slot == slot) {
                    joined_now = true;
                    let n = pilots.len();
                    let name = if n == 0 { settings.player.clone() } else { format!("Player {}", n + 1) };
                    pilots.push(Pilot { slot, st: SlotState::new(), mode: 1, name });
                }
            }
            // Read every controller once (button edges are consumed by reading).
            let pad_acts: Vec<(Slot, Actions)> = pad_state.iter_mut().map(|(s, st)| (*s, input.poll_slot(*s, st, 0.0, true).1)).collect();
            // Each pilot cycles their own flight mode with their mode button.
            for p in pilots.iter_mut() {
                let a = match pad_acts.iter().find(|(s, _)| *s == p.slot) {
                    Some((_, a)) => *a,
                    None => input.poll_slot(p.slot, &mut p.st, 0.0, true).1,
                };
                if a.mode {
                    p.mode = (p.mode + 1) % FlightMode::ALL.len();
                }
            }
            clear_background(BLACK);
            let cam = CamPose::look_at(vec3((get_time() as f32 * 0.1).cos() * 30.0, (get_time() as f32 * 0.1).sin() * 30.0, -10.0), vec3(0.0, 0.0, -2.0), 1.4, screen_width() / screen_height());
            r.render(&bg, &[], &[], &cam, None, 0.0);
            set_default_camera();
            // Four player cards side by side under the menu.
            let (sw, sh) = (screen_width(), screen_height());
            let top = (sh * 0.42).max(300.0);
            let rects: Vec<Rect> = (0..4).map(|i| Rect::new(sw / 4.0 * i as f32, top, sw / 4.0, sh - top)).collect();
            for (i, rc) in rects.iter().enumerate() {
                let rc = Rect::new(rc.x + 8.0, rc.y + 8.0, rc.w - 16.0, rc.h - 16.0);
                draw_rectangle(rc.x, rc.y, rc.w, rc.h, Color::new(0.0, 0.0, 0.0, 0.55));
                draw_rectangle_lines(rc.x, rc.y, rc.w, rc.h, 3.0, if i < pilots.len() { COLORS[i] } else { ui::DIM });
                let cx = |t: &str, y: f32, sz: f32, c: Color| {
                    let tw = measure_text(t, None, sz as u16, 1.0).width;
                    draw_text(t, rc.x + (rc.w - tw) / 2.0, rc.y + y, sz, c);
                };
                if let Some(p) = pilots.get(i) {
                    cx(&format!("P{}  {}", i + 1, p.name), 50.0, 32.0, COLORS[i]);
                    cx(&p.slot.label(input), 85.0, 18.0, WHITE);
                    let m = FlightMode::ALL[p.mode];
                    cx(&format!("{}  ({})", m.name(), if m == FlightMode::Beginner { "chase cam" } else { "FPV" }), 125.0, 26.0, ui::ACCENT);
                    let key = match p.slot {
                        Slot::KbLeft => "Z",
                        Slot::KbRight => "M",
                        Slot::Pad(_) | Slot::Virtual(_) => "X",
                    };
                    cx(&format!("{key}: change flight mode"), 152.0, 18.0, ui::DIM);
                    for (k, part) in p.slot.help().split("  ").enumerate() {
                        cx(part, 200.0 + k as f32 * 22.0, 18.0, ui::DIM);
                    }
                } else {
                    cx(&format!("P{}", i + 1), rc.h / 2.0 - 30.0, 34.0, ui::DIM);
                    cx("Press to join:", rc.h / 2.0 + 5.0, 22.0, WHITE);
                    cx("Q  left keyboard", rc.h / 2.0 + 35.0, 20.0, WHITE);
                    cx("U  right keyboard", rc.h / 2.0 + 60.0, 20.0, WHITE);
                    cx("A / Start  controller", rc.h / 2.0 + 85.0, 20.0, WHITE);
                }
            }
            menu.subtitle = match pilots.len() {
                0 => "Waiting for players".into(),
                n => format!("{n} player{} - arrows/Enter or a controller's D-pad/A", if n == 1 { "" } else { "s" }),
            };
            // Menus: keyboard plus every controller; not on the frame someone pressed A to join.
            let mut a = menu_actions(input, &mut []);
            for (_, b) in &pad_acts {
                a.up |= b.up;
                a.down |= b.down;
                a.left |= b.left;
                a.right |= b.right;
                a.select |= b.select;
                a.back |= b.back;
            }
            if joined_now {
                a = Actions::default();
            }
            // Draw the menu in the band above the cards.
            let ph = 160.0 + menu.items.len() as f32 * 46.0;
            let shift = ((sh - ph) / 2.0).max(20.0) - 16.0;
            let mut c = Camera2D::from_display_rect(Rect::new(0.0, shift, sw, sh));
            c.zoom.y = -c.zoom.y;
            set_camera(&c);
            let ev = menu.run(&a);
            set_default_camera();
            if let Some(au) = &auto {
                if get_time() - tj > au.after as f64 && (join_shot || !prejoined) {
                    if !join_shot {
                        println!("rflygame auto=split: still on the join screen after {:.0}s with {} pilots", au.after, pilots.len());
                    }
                    if let Some(path) = &au.screenshot {
                        get_screen_data().export_png(path);
                    }
                    return;
                }
            }
            match ev {
                MenuEvent::Activate(1) if !pilots.is_empty() => go = true,
                MenuEvent::Activate(2) | MenuEvent::Back => return,
                _ => {}
            }
            rflysim3d::record::tick();
            next_frame().await;
        }

        // ---------------------------------------------------------------- play
        let act = menu.choice(0);
        let mut players: Vec<Player> = pilots
            .drain(..)
            .enumerate()
            .map(|(i, p)| {
                let mode = FlightMode::ALL[p.mode];
                let (mut s, race) = if act < MAPS.len() {
                    (Session::new(MAPS[act], drone.clone(), mode, settings), None)
                } else {
                    let tr = tracks().remove(act - MAPS.len());
                    let mut s = Session::new(tr.map, drone.clone(), mode, settings);
                    let race = Race::new(tr, &mut s);
                    (s, Some(race))
                };
                let sp = s.spawn;
                s.set_spawn(sp.0.x - i as f64 * 2.5, sp.0.y + (i % 2) as f64 * 2.0, sp.1.to_degrees());
                Player { p, s, race, spring: true, done: None, rt: None }
            })
            .collect();
        let n = players.len();
        // Quarter views have no room for the stick widgets.
        let mut view_settings = settings.clone();
        view_settings.show_sticks &= n <= 2;
        let settings = &view_settings;
        let mut spare: Option<(RenderTarget, u32, u32)> = None;
        let models: Vec<VehicleModel> = (0..n).map(|i| build_model(ModelKind::Multirotor(4), COLORS[i])).collect();
        let mut paused: Option<Menu> = None;
        let mut results: Option<Menu> = None;
        let t0 = get_time();
        let mut leave = false;
        while !leave {
            let dt = get_frame_time();
            input.pump();
            let tt = (get_time() - t0) as f32;
            // Input and physics.
            let mut pause = is_key_pressed(KeyCode::Escape);
            if paused.is_none() && results.is_none() {
                for pl in players.iter_mut() {
                    let (mut sticks, mut a) = input.poll_slot(pl.p.slot, &mut pl.p.st, dt, pl.spring);
                    pause |= a.pause;
                    if prejoined {
                        let s = &mut pl.s;
                        s.max_steps = 2000;
                        if !s.armed && !s.locked && s.crashed.is_none() && tt > 0.5 {
                            a.arm = true;
                        }
                        let p0 = s.pos();
                        let k = pl.p.name.as_bytes()[1] as f32;
                        let tgt = match &pl.race {
                            Some(rc) => rc.auto_target(p0),
                            None => Some(vec3(12.0 * (tt * 0.2 + k).cos(), 12.0 * (tt * 0.2 + k).sin(), -5.0 - k % 3.0)),
                        };
                        sticks = tgt.map_or(Sticks::default(), |t| autopilot(s, t));
                    }
                    let player = pl.p.name.clone();
                    pl.s.update(sticks, &a, dt, &mut pl.spring);
                    if let Some(rc) = pl.race.as_mut() {
                        if rc.update(&mut pl.s, dt, &player).is_some() || (pl.done.is_none() && rc.finished().is_some()) {
                            pl.done = rc.finished().or(Some(f64::MAX));
                        }
                    }
                }
            }
            // Everyone sees everyone else.
            let snap: Vec<(String, Vec3, Quat, f32)> = players.iter().map(|p| (p.p.name.clone(), p.s.pos(), p.s.quat(), p.s.spin())).collect();
            for (i, pl) in players.iter_mut().enumerate() {
                pl.s.others.retain(|o| o.name == "Best run");
                for (j, o) in snap.iter().enumerate() {
                    if i != j {
                        pl.s.others.push(OtherDrone { name: o.0.clone(), pos: o.1, q: o.2, spin: o.3, color: COLORS[j], kind: ModelKind::Multirotor(4), scale: 1.0 });
                    }
                }
            }
            let racing = players.iter().any(|p| p.race.is_some());
            if racing && results.is_none() && players.iter().all(|p| p.done.is_some()) {
                results = Some(results_menu(&players));
            }

            // Draw every view into its own target, then place them on screen.
            let rects = layout(n, screen_width(), screen_height());
            let k = hud_scale(n);
            for (i, pl) in players.iter_mut().enumerate() {
                let (w, h) = ((rects[i].w * k) as u32, (rects[i].h * k) as u32);
                if pl.rt.as_ref().is_none_or(|x| x.1 != w || x.2 != h) {
                    let rt = render_target_ex(w, h, RenderTargetParams { sample_count: 1, depth: true });
                    rt.texture.set_filter(FilterMode::Linear);
                    pl.rt = Some((rt, w, h));
                }
                let rt = pl.rt.as_ref().unwrap().0.clone();
                ui::set_view(w as f32, h as f32, rt);
                let cam = pl.s.draw(r, settings);
                pl.s.hud(settings, &pl.p.slot.label(input), &cam);
                if let Some(rc) = &pl.race {
                    rc.hud(&pl.s, &cam);
                }
                // Name tag, top centre, in the player's colour.
                let tag = format!("P{} {}", i + 1, pl.p.name);
                let tw = measure_text(&tag, None, 26, 1.0).width;
                draw_rectangle(w as f32 / 2.0 - tw / 2.0 - 10.0, h as f32 - 40.0, tw + 20.0, 32.0, Color::new(0.0, 0.0, 0.0, 0.5));
                draw_text(&tag, w as f32 / 2.0 - tw / 2.0, h as f32 - 16.0, 26.0, COLORS[i]);
                if let Some(t) = pl.done {
                    let msg = if t == f64::MAX { "Finished".to_string() } else { format!("Finished  {}", fmt_time(t)) };
                    ui::centered(&msg, h as f32 * 0.35, 48.0, COLORS[i]);
                }
            }
            ui::clear_view();
            clear_background(BLACK);
            for (i, pl) in players.iter().enumerate() {
                let rc = rects[i];
                draw_texture_ex(&pl.rt.as_ref().unwrap().0.texture, rc.x, rc.y, WHITE, DrawTextureParams { dest_size: Some(vec2(rc.w, rc.h)), flip_y: true, ..Default::default() });
            }
            if n == 3 {
                spare_quarter(r, &players, &models, rects[3], &mut spare);
            }
            // Dividers.
            if n >= 2 {
                draw_line(0.0, screen_height() / 2.0, screen_width(), screen_height() / 2.0, 4.0, BLACK);
            }
            if n >= 3 {
                draw_line(screen_width() / 2.0, 0.0, screen_width() / 2.0, screen_height(), 4.0, BLACK);
            }

            // Pause and results overlays (anyone can drive them).
            if pause && paused.is_none() && results.is_none() {
                paused = Some(Menu::new("Paused", "", vec![Item::Button("Resume".into()), Item::Button("Restart".into()), Item::Button("Back to the join screen".into())]));
            }
            let mut pads: Vec<(Slot, SlotState)> = players.iter().filter(|p| matches!(p.p.slot, Slot::Pad(_) | Slot::Virtual(_))).map(|p| (p.p.slot, p.p.st)).collect();
            let ma = menu_actions(input, &mut pads);
            for (slot, st) in pads {
                if let Some(pl) = players.iter_mut().find(|p| p.p.slot == slot) {
                    pl.p.st = st;
                }
            }
            if let Some(m) = paused.as_mut() {
                match m.run(&ma) {
                    MenuEvent::Activate(0) => paused = None,
                    MenuEvent::Back if !pause => paused = None,
                    MenuEvent::Activate(1) => {
                        pilots = players.drain(..).map(|p| p.p).collect();
                        go = true;
                        leave = true;
                    }
                    MenuEvent::Activate(2) => leave = true,
                    _ => {}
                }
            } else if let Some(m) = results.as_mut() {
                match m.run(&ma) {
                    MenuEvent::Activate(i) if i + 2 == m.items.len() => {
                        pilots = players.drain(..).map(|p| p.p).collect();
                        go = true;
                        leave = true;
                    }
                    MenuEvent::Activate(i) if i + 1 == m.items.len() => leave = true,
                    _ => {}
                }
            }
            audio.update(players.first_mut().map(|p| &mut p.s), paused.is_none(), settings.volume);
            if let Some(a) = &auto {
                if tt > a.after {
                    if let Some(path) = &a.screenshot {
                        get_screen_data().export_png(path);
                    }
                    let st: Vec<String> = players
                        .iter()
                        .map(|p| format!("{}:{}:{}:armed={},alt={:.1},pos=({:.1},{:.1},{:.1}),others={},gate={},done={}", p.p.name, p.p.slot.label(input).replace(' ', "_"), p.s.pilot.mode.name(), p.s.armed, p.s.agl(), p.s.pos().x, p.s.pos().y, p.s.pos().z, p.s.others.len(), p.race.as_ref().map_or(0, |r| r.progress()), p.done.map_or("-".into(), fmt_time)))
                        .collect();
                    println!("rflygame auto=split: players={n} fps={} {}", get_fps(), st.join(" "));
                    return;
                }
            }
            rflysim3d::record::tick();
            next_frame().await;
        }
        // Back to the join screen with the same pilots, or (go set) straight into a restart.
        if !go {
            pilots = players.drain(..).map(|p| p.p).collect();
        }
        restart = go;
    }
}

fn fmt_time(t: f64) -> String {
    if t == f64::MAX {
        "DNF".into()
    } else {
        format!("{:.2}s", t)
    }
}

fn results_menu(players: &[Player]) -> Menu {
    let mut order: Vec<(usize, f64)> = players.iter().enumerate().map(|(i, p)| (i, p.done.unwrap_or(f64::MAX))).collect();
    order.sort_by(|a, b| a.1.total_cmp(&b.1));
    let mut items: Vec<Item> = order.iter().enumerate().map(|(k, (i, t))| Item::Info(format!("{}.  P{} {}   {}", k + 1, i + 1, players[*i].p.name, fmt_time(*t)))).collect();
    items.push(Item::Button("Race again".into()));
    items.push(Item::Button("Back to the join screen".into()));
    let mut m = Menu::new("Results", "", items);
    m.sel = m.items.len() - 2;
    m
}

/// The fourth quarter with three players: overhead view of everyone plus standings.
fn spare_quarter(r: &Renderer, players: &[Player], models: &[VehicleModel], rc: Rect, cache: &mut Option<(RenderTarget, u32, u32)>) {
    let c = players.iter().map(|p| p.s.pos()).fold(Vec3::ZERO, |a, b| a + b) / players.len() as f32;
    let spread = players.iter().map(|p| (p.s.pos() - c).length()).fold(10.0f32, f32::max);
    let eye = c + vec3(-spread * 0.8, 0.0, -spread * 2.2 - 10.0);
    let cam = CamPose::look_at(eye, c, 1.2, rc.w / rc.h);
    let list: Vec<DrawVehicle> = players
        .iter()
        .enumerate()
        .map(|(i, p)| DrawVehicle { model: &models[i], kind: ModelKind::Multirotor(4), pos: p.s.pos(), q: p.s.quat(), scale: Vec3::splat(2.5), prop_angle: p.s.spin() * 50.0, spinning: true, seg_id: 1 })
        .collect();
    let (w, h) = (rc.w as u32, rc.h as u32);
    if cache.as_ref().is_none_or(|x| x.1 != w || x.2 != h) {
        *cache = Some((render_target_ex(w, h, RenderTargetParams { sample_count: 1, depth: true }), w, h));
    }
    let rt = cache.as_ref().unwrap().0.clone();
    r.render(&players[0].s.map, &list, &[], &cam, Some(rt.clone()), 0.0);
    set_default_camera();
    draw_texture_ex(&rt.texture, rc.x, rc.y, WHITE, DrawTextureParams { dest_size: Some(vec2(rc.w, rc.h)), flip_y: true, ..Default::default() });
    // Standings.
    let mut order: Vec<usize> = (0..players.len()).collect();
    let key = |p: &Player| (p.done.unwrap_or(f64::MAX), -(p.race.as_ref().map_or(0, |r| r.progress()) as i32));
    order.sort_by(|a, b| key(&players[*a]).partial_cmp(&key(&players[*b])).unwrap());
    draw_rectangle(rc.x + 12.0, rc.y + 12.0, 250.0, 34.0 + 26.0 * players.len() as f32, ui::PANEL);
    draw_text("Standings", rc.x + 24.0, rc.y + 36.0, 22.0, ui::ACCENT);
    for (k, i) in order.iter().enumerate() {
        let p = &players[*i];
        let what = match (&p.race, p.done) {
            (_, Some(t)) => fmt_time(t),
            (Some(r), None) => format!("gate {}", r.progress()),
            (None, None) => format!("{:.0} m up", p.s.agl()),
        };
        draw_text(&format!("{}. P{} {}  {}", k + 1, i + 1, p.p.name, what), rc.x + 24.0, rc.y + 62.0 + 26.0 * k as f32, 20.0, COLORS[*i]);
    }
}
