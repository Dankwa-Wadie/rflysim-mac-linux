//! Gamepad/keyboard/mouse-navigable menus and HUD widgets.

use crate::input::Actions;
use macroquad::prelude::*;

pub const ACCENT: Color = Color::new(0.20, 0.78, 0.72, 1.0);
pub const PANEL: Color = Color::new(0.05, 0.07, 0.10, 0.82);
pub const DIM: Color = Color::new(1.0, 1.0, 1.0, 0.55);

// ---------------------------------------------------------------- views (split screen)

/// The area the HUD draws into. Full window by default; split screen sets a per-player
/// off-screen target so every HUD widget lays itself out for that player's view.
#[derive(Clone)]
struct View {
    w: f32,
    h: f32,
    rt: Option<RenderTarget>,
}

thread_local! {
    static VIEW: std::cell::RefCell<Option<View>> = const { std::cell::RefCell::new(None) };
}

/// Width of the current view (the window unless split screen set one).
pub fn vw() -> f32 {
    VIEW.with(|v| v.borrow().as_ref().map_or_else(screen_width, |v| v.w))
}

/// Height of the current view.
pub fn vh() -> f32 {
    VIEW.with(|v| v.borrow().as_ref().map_or_else(screen_height, |v| v.h))
}

/// Render target of the current view (None = the window).
pub fn view_rt() -> Option<RenderTarget> {
    VIEW.with(|v| v.borrow().as_ref().and_then(|v| v.rt.clone()))
}

/// Draw the following 3D scene and HUD into `rt` (w × h pixels).
pub fn set_view(w: f32, h: f32, rt: RenderTarget) {
    VIEW.with(|v| *v.borrow_mut() = Some(View { w, h, rt: Some(rt) }));
}

/// Back to drawing on the window.
pub fn clear_view() {
    VIEW.with(|v| *v.borrow_mut() = None);
    set_default_camera();
}

/// 2D camera for HUD drawing in pixel coordinates of the current view.
pub fn hud_camera() {
    match view_rt() {
        None => set_default_camera(),
        Some(rt) => {
            let (w, h) = (vw(), vh());
            // from_display_rect already suits a render target: (0,0) is the top left once the
            // texture is drawn with flip_y.
            let mut c = Camera2D::from_display_rect(Rect::new(0.0, 0.0, w, h));
            c.render_target = Some(rt);
            set_camera(&c);
        }
    }
}

pub enum Item {
    /// Plain button.
    Button(String),
    /// Choice: label, options, selected index.
    Choice(String, Vec<String>, usize),
    /// Number: label, value, min, max, step, unit.
    Number(String, f64, f64, f64, f64, &'static str),
    /// Toggle: label, value.
    Toggle(String, bool),
    /// Non-selectable text line.
    Info(String),
}

impl Item {
    fn selectable(&self) -> bool {
        !matches!(self, Item::Info(_))
    }
}

pub struct Menu {
    pub title: String,
    pub subtitle: String,
    pub items: Vec<Item>,
    pub sel: usize,
}

#[derive(Debug, PartialEq, Eq)]
pub enum MenuEvent {
    None,
    /// Button activated (index).
    Activate(usize),
    /// Value of item index changed.
    Changed(usize),
    Back,
}

impl Menu {
    pub fn new(title: &str, subtitle: &str, items: Vec<Item>) -> Self {
        let mut m = Menu { title: title.into(), subtitle: subtitle.into(), items, sel: 0 };
        m.fix_sel(1);
        m
    }

    fn fix_sel(&mut self, dir: i32) {
        let n = self.items.len() as i32;
        for _ in 0..n {
            if self.items[self.sel].selectable() {
                return;
            }
            self.sel = ((self.sel as i32 + dir).rem_euclid(n)) as usize;
        }
    }

    pub fn choice(&self, i: usize) -> usize {
        match &self.items[i] {
            Item::Choice(_, _, k) => *k,
            _ => 0,
        }
    }
    pub fn number(&self, i: usize) -> f64 {
        match &self.items[i] {
            Item::Number(_, v, ..) => *v,
            _ => 0.0,
        }
    }
    pub fn toggle(&self, i: usize) -> bool {
        matches!(&self.items[i], Item::Toggle(_, true))
    }

    fn step(&mut self, i: usize, dir: i32) -> bool {
        match &mut self.items[i] {
            Item::Choice(_, opts, k) => {
                *k = ((*k as i32 + dir).rem_euclid(opts.len() as i32)) as usize;
                true
            }
            Item::Number(_, v, lo, hi, st, _) => {
                *v = (*v + *st * dir as f64).clamp(*lo, *hi);
                true
            }
            Item::Toggle(_, b) => {
                *b = !*b;
                true
            }
            _ => false,
        }
    }

    /// Handles input and draws the menu; returns what happened.
    pub fn run(&mut self, a: &Actions) -> MenuEvent {
        let n = self.items.len();
        let mut ev = MenuEvent::None;
        if a.down {
            self.sel = (self.sel + 1) % n;
            self.fix_sel(1);
        }
        if a.up {
            self.sel = (self.sel + n - 1) % n;
            self.fix_sel(-1);
        }
        if a.left && self.step(self.sel, -1) {
            ev = MenuEvent::Changed(self.sel);
        }
        if a.right && self.step(self.sel, 1) {
            ev = MenuEvent::Changed(self.sel);
        }
        if a.select {
            ev = match self.items[self.sel] {
                Item::Button(_) => MenuEvent::Activate(self.sel),
                _ => {
                    if self.step(self.sel, 1) {
                        MenuEvent::Changed(self.sel)
                    } else {
                        MenuEvent::None
                    }
                }
            };
        }
        if a.back {
            ev = MenuEvent::Back;
        }

        // ---- draw
        let (w, h) = (vw(), vh());
        let pw = 620.0f32.min(w - 40.0);
        let row = 46.0;
        let ph = 160.0 + n as f32 * row;
        let (px, py) = ((w - pw) / 2.0, ((h - ph) / 2.0).max(20.0));
        draw_rectangle(px, py, pw, ph, PANEL);
        draw_rectangle(px, py, pw, 4.0, ACCENT);
        draw_text(&self.title, px + 30.0, py + 56.0, 44.0, WHITE);
        draw_text(&self.subtitle, px + 32.0, py + 86.0, 22.0, DIM);
        let (mx, my) = mouse_position();
        for (i, it) in self.items.iter().enumerate() {
            let y = py + 120.0 + i as f32 * row;
            let hover = mx > px && mx < px + pw && my > y - 4.0 && my < y + row - 8.0;
            if hover && it.selectable() && (mouse_delta_position() != Vec2::ZERO || is_mouse_button_pressed(MouseButton::Left)) {
                self.sel = i;
            }
            let selected = i == self.sel && it.selectable();
            if selected {
                draw_rectangle(px + 16.0, y - 4.0, pw - 32.0, row - 6.0, Color::new(ACCENT.r, ACCENT.g, ACCENT.b, 0.25));
                draw_rectangle(px + 16.0, y - 4.0, 4.0, row - 6.0, ACCENT);
            }
            let col = if selected { WHITE } else if it.selectable() { Color::new(0.85, 0.88, 0.9, 1.0) } else { DIM };
            let (label, value) = match it {
                Item::Button(l) | Item::Info(l) => (l.clone(), String::new()),
                Item::Choice(l, o, k) => (l.clone(), format!("< {} >", o[*k])),
                Item::Number(l, v, _, _, st, u) => (l.clone(), if *st >= 1.0 { format!("< {:.0}{u} >", v) } else { format!("< {:.2}{u} >", v) }),
                Item::Toggle(l, b) => (l.clone(), if *b { "< On >".into() } else { "< Off >".into() }),
            };
            draw_text(&label, px + 34.0, y + 26.0, 28.0, col);
            if !value.is_empty() {
                let tw = measure_text(&value, None, 26, 1.0).width;
                draw_text(&value, px + pw - 34.0 - tw, y + 26.0, 26.0, if selected { ACCENT } else { col });
            }
        }
        if is_mouse_button_pressed(MouseButton::Left) {
            let y0 = py + 120.0;
            if mx > px && mx < px + pw && my > y0 - 4.0 {
                let i = ((my - y0 + 4.0) / row) as usize;
                if i < n && i == self.sel && self.items[i].selectable() {
                    ev = match self.items[i] {
                        Item::Button(_) => MenuEvent::Activate(i),
                        _ => {
                            let dir = if mx < px + pw / 2.0 { -1 } else { 1 };
                            self.step(i, dir);
                            MenuEvent::Changed(i)
                        }
                    };
                }
            }
        }
        draw_text("Up/Down: move   Left/Right: change   Enter / A: select   Esc / B: back", px + 30.0, py + ph - 18.0, 18.0, DIM);
        ev
    }
}

/// Draws two RC-style stick boxes (Mode 2) at the bottom-right.
pub fn draw_sticks(s: &crate::flight::Sticks) {
    let (w, h) = (vw(), vh());
    let size = 90.0;
    for (k, (x, y)) in [(s.yaw, s.throttle), (s.roll, s.pitch)].iter().enumerate() {
        let ox = w - 2.0 * size - 40.0 + k as f32 * (size + 14.0);
        let oy = h - size - 40.0;
        draw_rectangle(ox, oy, size, size, Color::new(0.0, 0.0, 0.0, 0.35));
        draw_rectangle_lines(ox, oy, size, size, 1.5, DIM);
        draw_line(ox + size / 2.0, oy, ox + size / 2.0, oy + size, 1.0, Color::new(1.0, 1.0, 1.0, 0.2));
        draw_line(ox, oy + size / 2.0, ox + size, oy + size / 2.0, 1.0, Color::new(1.0, 1.0, 1.0, 0.2));
        let cx = ox + size / 2.0 + x * size / 2.0;
        let cy = oy + size / 2.0 - y * size / 2.0;
        draw_circle(cx, cy, 7.0, ACCENT);
    }
}

pub fn centered(text: &str, y: f32, size: f32, col: Color) {
    let tw = measure_text(text, None, size as u16, 1.0).width;
    draw_text(text, (vw() - tw) / 2.0, y, size, col);
}

/// Edits `buf` with typed characters. Returns Some(true) on Enter, Some(false) on Esc.
pub fn text_edit(buf: &mut String, max: usize) -> Option<bool> {
    while let Some(c) = get_char_pressed() {
        if !c.is_control() && buf.len() < max {
            buf.push(c);
        }
    }
    if is_key_pressed(KeyCode::Backspace) {
        buf.pop();
    }
    if is_key_pressed(KeyCode::Enter) || is_key_pressed(KeyCode::KpEnter) {
        return Some(true);
    }
    if is_key_pressed(KeyCode::Escape) {
        return Some(false);
    }
    None
}

/// Modal text box.
pub fn text_box(title: &str, buf: &str) {
    let (w, h) = (vw(), vh());
    draw_rectangle(w / 2.0 - 320.0, h / 2.0 - 80.0, 640.0, 160.0, PANEL);
    draw_rectangle(w / 2.0 - 320.0, h / 2.0 - 80.0, 640.0, 4.0, ACCENT);
    draw_text(title, w / 2.0 - 300.0, h / 2.0 - 36.0, 26.0, WHITE);
    let caret = if (get_time() * 2.0) as i64 % 2 == 0 { "_" } else { " " };
    draw_rectangle(w / 2.0 - 300.0, h / 2.0 - 14.0, 600.0, 44.0, Color::new(1.0, 1.0, 1.0, 0.08));
    draw_text(&format!("{buf}{caret}"), w / 2.0 - 290.0, h / 2.0 + 16.0, 30.0, ACCENT);
    draw_text("Enter = OK   Esc = cancel", w / 2.0 - 300.0, h / 2.0 + 60.0, 18.0, DIM);
}
