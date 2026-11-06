//! A small immediate-mode form toolkit on macroquad: labels, buttons, text fields, checkboxes, radio
//! buttons and a searchable dropdown (type to filter, scroll with the wheel). Desktop-style, mouse-driven.

use macroquad::prelude::*;

pub const BG: Color = Color::new(0.94, 0.94, 0.95, 1.0);
pub const PANEL: Color = Color::new(1.0, 1.0, 1.0, 1.0);
pub const LINE: Color = Color::new(0.78, 0.79, 0.82, 1.0);
pub const TEXT: Color = Color::new(0.13, 0.14, 0.16, 1.0);
pub const DIM: Color = Color::new(0.45, 0.47, 0.52, 1.0);
pub const ACCENT: Color = Color::new(0.10, 0.55, 0.52, 1.0);
pub const WARN: Color = Color::new(0.80, 0.42, 0.05, 1.0);
pub const BAD: Color = Color::new(0.78, 0.16, 0.16, 1.0);
pub const FS: f32 = 17.0;
pub const ROW: f32 = 26.0;

/// Which widget has the keyboard, and the open dropdown.
#[derive(Default)]
pub struct Ui {
    pub focus: Option<u64>,
    open: Option<Open>,
    /// Set when a click was consumed by an overlay this frame (so widgets underneath ignore it).
    eaten: bool,
    clicked_pos: Option<Vec2>,
}

struct Open {
    id: u64,
    rect: Rect,
    filter: String,
    scroll: usize,
    items: Vec<String>,
}

pub fn id(s: &str, n: usize) -> u64 {
    // FNV-1a
    let mut h: u64 = 0xcbf29ce484222325;
    for b in s.bytes().chain(n.to_le_bytes()) {
        h ^= b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

pub fn text(s: &str, x: f32, y: f32, size: f32, c: Color) {
    draw_text(s, x, y + size * 0.75, size, c);
}

pub fn text_width(s: &str, size: f32) -> f32 {
    measure_text(s, None, size as u16, 1.0).width
}

/// Cuts `s` to fit `w` pixels, with an ellipsis.
pub fn fit(s: &str, w: f32, size: f32) -> String {
    if text_width(s, size) <= w {
        return s.to_string();
    }
    let mut t: String = s.to_string();
    while !t.is_empty() && text_width(&(t.clone() + "..."), size) > w {
        t.pop();
    }
    t + "..."
}

impl Ui {
    /// Call at the start of each frame.
    pub fn begin(&mut self) {
        self.eaten = false;
        self.clicked_pos = is_mouse_button_pressed(MouseButton::Left).then(|| mouse_position().into());
        // A click inside the open dropdown belongs to it.
        if let (Some(o), Some(p)) = (&self.open, self.clicked_pos) {
            let list = Rect::new(o.rect.x, o.rect.y + o.rect.h, o.rect.w.max(360.0), ROW * 13.0);
            if list.contains(p) || o.rect.contains(p) {
                self.eaten = true;
            } else {
                self.open = None;
                self.eaten = true;
            }
        }
    }

    fn clicked(&self, r: Rect) -> bool {
        !self.eaten && self.clicked_pos.is_some_and(|p| r.contains(p))
    }

    pub fn panel(&self, r: Rect, title: &str) {
        draw_rectangle(r.x, r.y, r.w, r.h, PANEL);
        draw_rectangle_lines(r.x, r.y, r.w, r.h, 1.0, LINE);
        draw_rectangle(r.x, r.y, r.w, 28.0, Color::new(0.90, 0.92, 0.94, 1.0));
        text(title, r.x + 10.0, r.y + 5.0, 19.0, TEXT);
    }

    pub fn label(&self, s: &str, x: f32, y: f32) {
        text(s, x, y + 4.0, FS, TEXT);
    }

    pub fn button(&mut self, label: &str, r: Rect, enabled: bool) -> bool {
        let hover = enabled && r.contains(mouse_position().into()) && self.open.is_none();
        let fill = if !enabled {
            Color::new(0.90, 0.90, 0.91, 1.0)
        } else if hover {
            Color::new(0.86, 0.93, 0.92, 1.0)
        } else {
            Color::new(0.97, 0.97, 0.98, 1.0)
        };
        draw_rectangle(r.x, r.y, r.w, r.h, fill);
        draw_rectangle_lines(r.x, r.y, r.w, r.h, 1.0, if hover { ACCENT } else { LINE });
        let w = text_width(label, FS);
        text(label, r.x + (r.w - w) / 2.0, r.y + (r.h - FS) / 2.0, FS, if enabled { TEXT } else { DIM });
        enabled && self.clicked(r)
    }

    /// Text field; returns true when the text changed this frame.
    pub fn field(&mut self, wid: u64, r: Rect, value: &mut String) -> bool {
        let focused = self.focus == Some(wid);
        if self.clicked(r) {
            self.focus = Some(wid);
        } else if self.clicked_pos.is_some() && !r.contains(self.clicked_pos.unwrap()) && focused && !self.eaten {
            self.focus = None;
        }
        let mut changed = false;
        if self.focus == Some(wid) {
            while let Some(c) = get_char_pressed() {
                if !c.is_control() && value.len() < 64 {
                    value.push(c);
                    changed = true;
                }
            }
            if is_key_pressed(KeyCode::Backspace) && value.pop().is_some() {
                changed = true;
            }
            if is_key_pressed(KeyCode::Enter) || is_key_pressed(KeyCode::Tab) {
                self.focus = None;
            }
        }
        draw_rectangle(r.x, r.y, r.w, r.h, WHITE);
        draw_rectangle_lines(r.x, r.y, r.w, r.h, if focused { 2.0 } else { 1.0 }, if focused { ACCENT } else { LINE });
        text(&fit(value, r.w - 10.0, FS), r.x + 5.0, r.y + (r.h - FS) / 2.0, FS, TEXT);
        if focused && (get_time() * 2.0) as i64 % 2 == 0 {
            let x = r.x + 6.0 + text_width(value, FS).min(r.w - 12.0);
            draw_line(x, r.y + 4.0, x, r.y + r.h - 4.0, 1.0, TEXT);
        }
        changed
    }

    pub fn checkbox(&mut self, label: &str, x: f32, y: f32, value: &mut bool) -> bool {
        let b = Rect::new(x, y + 4.0, 16.0, 16.0);
        let hit = Rect::new(x, y, 24.0 + text_width(label, FS), ROW);
        let mut changed = false;
        if self.clicked(hit) {
            *value = !*value;
            changed = true;
        }
        draw_rectangle(b.x, b.y, b.w, b.h, WHITE);
        draw_rectangle_lines(b.x, b.y, b.w, b.h, 1.0, if *value { ACCENT } else { LINE });
        if *value {
            draw_rectangle(b.x + 3.0, b.y + 3.0, 10.0, 10.0, ACCENT);
        }
        text(label, x + 22.0, y + 4.0, FS, TEXT);
        changed
    }

    pub fn radio(&mut self, label: &str, x: f32, y: f32, selected: bool) -> bool {
        let hit = Rect::new(x, y, 24.0 + text_width(label, FS), ROW);
        draw_circle_lines(x + 8.0, y + 12.0, 7.0, 1.0, if selected { ACCENT } else { LINE });
        if selected {
            draw_circle(x + 8.0, y + 12.0, 4.0, ACCENT);
        }
        text(label, x + 22.0, y + 4.0, FS, TEXT);
        !selected && self.clicked(hit)
    }

    /// Searchable dropdown. Returns true when the selection changed. Call [`Ui::overlay`] after all widgets.
    pub fn dropdown(&mut self, wid: u64, r: Rect, items: &[String], sel: &mut usize) -> bool {
        let open = self.open.as_ref().is_some_and(|o| o.id == wid);
        if self.clicked(r) {
            self.open = Some(Open { id: wid, rect: r, filter: String::new(), scroll: 0, items: items.to_vec() });
            self.focus = None;
            self.eaten = true;
        }
        draw_rectangle(r.x, r.y, r.w, r.h, WHITE);
        draw_rectangle_lines(r.x, r.y, r.w, r.h, if open { 2.0 } else { 1.0 }, if open { ACCENT } else { LINE });
        let cur = items.get(*sel).map(String::as_str).unwrap_or("");
        text(&fit(cur, r.w - 30.0, FS), r.x + 5.0, r.y + (r.h - FS) / 2.0, FS, TEXT);
        draw_triangle(vec2(r.x + r.w - 18.0, r.y + 10.0), vec2(r.x + r.w - 8.0, r.y + 10.0), vec2(r.x + r.w - 13.0, r.y + 16.0), DIM);
        // The pick is made in `overlay` and delivered here on the next frame.
        if let Some(p) = PICK.with(|p| p.borrow_mut().take_if(|(i, _)| *i == wid)) {
            if p.1 < items.len() && p.1 != *sel {
                *sel = p.1;
                return true;
            }
        }
        false
    }

    /// Draws the open dropdown list on top of everything (call last in the frame).
    pub fn overlay(&mut self) {
        let Some(o) = &mut self.open else { return };
        let items = o.items.clone();
        while let Some(c) = get_char_pressed() {
            if !c.is_control() {
                o.filter.push(c);
                o.scroll = 0;
            }
        }
        if is_key_pressed(KeyCode::Backspace) {
            o.filter.pop();
            o.scroll = 0;
        }
        if is_key_pressed(KeyCode::Escape) {
            self.open = None;
            return;
        }
        let f = o.filter.to_lowercase();
        let shown: Vec<(usize, &String)> = items.iter().enumerate().filter(|(_, s)| f.is_empty() || s.to_lowercase().contains(&f)).collect();
        let rows = 12usize;
        let (_, wheel) = mouse_wheel();
        if wheel < 0.0 && o.scroll + rows < shown.len() {
            o.scroll += 3.min(shown.len() - rows - o.scroll);
        } else if wheel > 0.0 {
            o.scroll = o.scroll.saturating_sub(3);
        }
        let w = o.rect.w.max(360.0);
        let x = o.rect.x.min(screen_width() - w - 4.0);
        let mut y = o.rect.y + o.rect.h;
        if y + ROW * 13.0 > screen_height() {
            y = (o.rect.y - ROW * 13.0).max(0.0);
        }
        draw_rectangle(x + 3.0, y + 3.0, w, ROW * 13.0, Color::new(0.0, 0.0, 0.0, 0.15));
        draw_rectangle(x, y, w, ROW * 13.0, WHITE);
        draw_rectangle_lines(x, y, w, ROW * 13.0, 1.0, ACCENT);
        let hint = if o.filter.is_empty() { format!("Type to search ({} items)", items.len()) } else { format!("Search: {}  ({} found)", o.filter, shown.len()) };
        text(&hint, x + 6.0, y + 4.0, FS, DIM);
        let mouse: Vec2 = mouse_position().into();
        for (row, (i, s)) in shown.iter().skip(o.scroll).take(rows).enumerate() {
            let r = Rect::new(x, y + ROW * (row as f32 + 1.0), w, ROW);
            if r.contains(mouse) {
                draw_rectangle(r.x, r.y, r.w, r.h, Color::new(0.86, 0.93, 0.92, 1.0));
                if is_mouse_button_pressed(MouseButton::Left) {
                    PICK.with(|p| *p.borrow_mut() = Some((o.id, *i)));
                    self.open = None;
                    return;
                }
            }
            text(&fit(s, w - 12.0, FS), r.x + 6.0, r.y + 4.0, FS, TEXT);
        }
        if shown.len() > rows {
            let frac = o.scroll as f32 / (shown.len() - rows) as f32;
            draw_rectangle(x + w - 5.0, y + ROW + frac * (ROW * 11.0), 4.0, ROW, LINE);
        }
    }

}

thread_local! {
    /// The item picked in an open dropdown, delivered to that dropdown on the next frame.
    static PICK: std::cell::RefCell<Option<(u64, usize)>> = const { std::cell::RefCell::new(None) };
}
