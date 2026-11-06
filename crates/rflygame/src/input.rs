//! Keyboard, mouse, gamepad and RC-transmitter (USB joystick) input → sticks + actions.
//!
//! Gamepads with a known layout use Mode 2 (left stick: throttle/yaw, right: pitch/roll).
//! Anything else (e.g. an RC transmitter in USB-joystick mode) is read as raw axes through
//! an `AxisMap` that the Controls screen can set by "move the stick" detection.

use crate::flight::Sticks;
use gilrs::{Axis, Button, GamepadId, Gilrs, MappingSource};
use macroquad::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisBind {
    pub index: usize,
    pub invert: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AxisMap {
    pub roll: AxisBind,
    pub pitch: AxisBind,
    pub yaw: AxisBind,
    pub throttle: AxisBind,
}

impl Default for AxisMap {
    /// Typical EdgeTX/OpenTX USB joystick order: CH1 ail, CH2 ele, CH3 thr, CH4 rud.
    fn default() -> Self {
        AxisMap {
            roll: AxisBind { index: 0, invert: false },
            pitch: AxisBind { index: 1, invert: false },
            throttle: AxisBind { index: 2, invert: false },
            yaw: AxisBind { index: 3, invert: false },
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Actions {
    pub arm: bool,
    pub reset: bool,
    pub camera: bool,
    pub mode: bool,
    pub pause: bool,
    pub up: bool,
    pub down: bool,
    pub left: bool,
    pub right: bool,
    pub select: bool,
    pub back: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Device {
    Keyboard,
    Gamepad,
    Joystick,
}

pub struct Input {
    gilrs: Option<Gilrs>,
    pub device: Device,
    pub device_name: String,
    pub map: AxisMap,
    pub deadzone: f32,
    pub invert_pitch: bool,
    /// Keyboard Mode 2 (W/S throttle, A/D yaw, arrows pitch/roll) vs arcade (W/S/A/D move).
    pub kb_mode2: bool,
    kb: Sticks,
    kb_throttle_hold: f32,
    mouse: Vec2,
    prev_buttons: u32,
    vpad: Option<VirtualPads>,
    vpad_prev: u32,
}

const BTN: [Button; 9] = [
    Button::South,
    Button::East,
    Button::North,
    Button::West,
    Button::Start,
    Button::Select,
    Button::DPadUp,
    Button::DPadDown,
    Button::DPadLeft,
];

fn dz(d: f32, x: f32) -> f32 {
    if x.abs() < d {
        0.0
    } else {
        (x - d * x.signum()) / (1.0 - d)
    }
}

impl Input {
    pub fn new() -> Self {
        let gilrs = Gilrs::new().map_err(|e| eprintln!("rflygame: no gamepad support: {e}")).ok();
        Input {
            gilrs,
            device: Device::Keyboard,
            device_name: "Keyboard + mouse".into(),
            map: AxisMap::default(),
            deadzone: 0.04,
            invert_pitch: false,
            kb_mode2: true,
            kb: Sticks::default(),
            kb_throttle_hold: -1.0,
            mouse: Vec2::ZERO,
            prev_buttons: 0,
            vpad: None,
            vpad_prev: 0,
        }
    }


    /// Raw axis values of the first connected controller (for the mapping wizard).
    pub fn raw_axes(&self) -> Vec<f32> {
        let Some(g) = &self.gilrs else { return vec![] };
        let Some((_, pad)) = g.gamepads().next() else { return vec![] };
        let mut axes: Vec<(u32, f32)> = pad.state().axes().map(|(c, d)| (c.into_u32(), d.value())).collect();
        axes.sort_by_key(|a| a.0);
        axes.into_iter().map(|a| a.1).collect()
    }

    /// Polls all devices. `spring_throttle`: keyboard throttle returns to centre
    /// (assisted modes) instead of holding its value (Acro).
    pub fn poll(&mut self, dt: f32, spring_throttle: bool) -> (Sticks, Actions) {
        let mut act = Actions::default();
        // ---- keyboard
        act.arm |= is_key_pressed(KeyCode::Space);
        act.reset |= is_key_pressed(KeyCode::R);
        act.camera |= is_key_pressed(KeyCode::C);
        act.mode |= is_key_pressed(KeyCode::M);
        act.pause |= is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::P);
        act.up |= is_key_pressed(KeyCode::Up) || is_key_pressed(KeyCode::W);
        act.down |= is_key_pressed(KeyCode::Down) || is_key_pressed(KeyCode::S);
        act.left |= is_key_pressed(KeyCode::Left) || is_key_pressed(KeyCode::A);
        act.right |= is_key_pressed(KeyCode::Right) || is_key_pressed(KeyCode::D);
        act.select |= is_key_pressed(KeyCode::Enter) || is_key_pressed(KeyCode::KpEnter);
        act.back |= is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::Backspace);

        let full = if is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift) { 1.0 } else { 0.6 };
        let axis = |pos: KeyCode, neg: KeyCode| (is_key_down(pos) as i32 - is_key_down(neg) as i32) as f32 * full;
        // Mode 2, like a transmitter: left hand W/S throttle + A/D yaw, right hand arrows pitch/roll.
        let target = if self.kb_mode2 {
            Sticks {
                throttle: axis(KeyCode::W, KeyCode::S),
                yaw: axis(KeyCode::D, KeyCode::A),
                pitch: axis(KeyCode::Up, KeyCode::Down),
                roll: axis(KeyCode::Right, KeyCode::Left),
            }
        } else {
            Sticks {
                pitch: axis(KeyCode::W, KeyCode::S),
                roll: axis(KeyCode::D, KeyCode::A),
                yaw: axis(KeyCode::Right, KeyCode::Left),
                throttle: axis(KeyCode::Up, KeyCode::Down),
            }
        };
        let k = (dt * 8.0).min(1.0);
        let ease = |a: f32, b: f32| a + (b - a) * k;
        self.kb.pitch = ease(self.kb.pitch, target.pitch);
        self.kb.roll = ease(self.kb.roll, target.roll);
        self.kb.yaw = ease(self.kb.yaw, target.yaw);
        if spring_throttle {
            self.kb.throttle = ease(self.kb.throttle, target.throttle);
            self.kb_throttle_hold = -1.0;
        } else {
            self.kb_throttle_hold = (self.kb_throttle_hold + target.throttle * dt * 1.2).clamp(-1.0, 1.0);
            self.kb.throttle = self.kb_throttle_hold;
        }
        let mut sticks = self.kb;
        let kb_active = target != Sticks::default();

        // ---- mouse virtual stick (hold right button)
        if is_mouse_button_down(MouseButton::Right) {
            let d = mouse_delta_position();
            self.mouse = (self.mouse - d * 4.0).clamp(Vec2::splat(-1.0), Vec2::splat(1.0));
            sticks.roll = self.mouse.x;
            sticks.pitch = self.mouse.y;
        } else {
            self.mouse *= 0.0;
        }

        // ---- gamepad / joystick
        let dzv = self.deadzone;
        let map = self.map.clone();
        if let Some(g) = &mut self.gilrs {
            while g.next_event().is_some() {}
            if let Some((_, pad)) = g.gamepads().next() {
                let mapped = pad.mapping_source() != MappingSource::None;
                self.device = if mapped { Device::Gamepad } else { Device::Joystick };
                self.device_name = pad.name().to_string();
                let mut bits = 0u32;
                for (i, b) in BTN.iter().enumerate() {
                    if pad.is_pressed(*b) {
                        bits |= 1 << i;
                    }
                }
                if pad.is_pressed(Button::DPadRight) {
                    bits |= 1 << 9;
                }
                let edge = bits & !self.prev_buttons;
                self.prev_buttons = bits;
                let e = |i: u32| edge & (1 << i) != 0;
                act.arm |= e(0);
                act.select |= e(0);
                act.back |= e(1);
                act.camera |= e(2);
                act.mode |= e(3);
                act.pause |= e(4);
                act.reset |= e(5);
                act.up |= e(6);
                act.down |= e(7);
                act.left |= e(8);
                act.right |= e(9);

                let pad_sticks = if mapped {
                    Sticks {
                        throttle: dz(dzv, pad.value(Axis::LeftStickY)),
                        yaw: dz(dzv, pad.value(Axis::LeftStickX)),
                        pitch: dz(dzv, pad.value(Axis::RightStickY)),
                        roll: dz(dzv, pad.value(Axis::RightStickX)),
                    }
                } else {
                    let raw = {
                        let mut axes: Vec<(u32, f32)> = pad.state().axes().map(|(c, d)| (c.into_u32(), d.value())).collect();
                        axes.sort_by_key(|a| a.0);
                        axes.into_iter().map(|a| a.1).collect::<Vec<f32>>()
                    };
                    let get = |b: AxisBind| raw.get(b.index).map(|v| if b.invert { -v } else { *v }).unwrap_or(0.0);
                    Sticks {
                        throttle: get(map.throttle),
                        yaw: dz(dzv, get(map.yaw)),
                        pitch: dz(dzv, get(map.pitch)),
                        roll: dz(dzv, get(map.roll)),
                    }
                };
                // Controller wins whenever it's touched and the keyboard is idle.
                let pad_active = pad_sticks.roll != 0.0 || pad_sticks.pitch != 0.0 || pad_sticks.yaw != 0.0 || (!mapped || pad_sticks.throttle != 0.0);
                if pad_active && !kb_active {
                    sticks = pad_sticks;
                }
            } else {
                self.device = Device::Keyboard;
                self.device_name = "Keyboard + mouse".into();
            }
        }
        // ---- virtual controller 0 (scripts, tests), when enabled
        self.drain_virtual();
        if let Some((bits, ax)) = self.vpad.as_ref().and_then(|v| v.get(0)) {
            self.device = Device::Gamepad;
            self.device_name = "Virtual controller 1".into();
            let edge = bits & !self.vpad_prev;
            self.vpad_prev = bits;
            merge_edges(&mut act, edge);
            let d = self.deadzone;
            let vs = Sticks { throttle: dz(d, ax[1]), yaw: dz(d, ax[0]), pitch: dz(d, ax[3]), roll: dz(d, ax[2]) };
            if vs != Sticks::default() && !kb_active {
                sticks = vs;
            }
        }
        if self.invert_pitch {
            sticks.pitch = -sticks.pitch;
        }
        (sticks, act)
    }
}

// ---------------------------------------------------------------- split screen

/// One local player's controls in split screen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Slot {
    /// Left half of the keyboard: W/S throttle, A/D yaw, T/G pitch, F/H roll; Q arm, E camera,
    /// Z mode, R reset, left Shift full stick.
    KbLeft,
    /// Right half: I/K throttle, J/L yaw, arrows pitch/roll; U arm, O camera, M mode,
    /// P reset, right Shift full stick.
    KbRight,
    Pad(GamepadId),
    /// A virtual controller (0–3) driven over UDP by a script; see `VirtualPads`.
    Virtual(u8),
}

impl Slot {
    pub fn label(&self, input: &Input) -> String {
        match self {
            Slot::KbLeft => "Keyboard left (WASD + TFGH)".into(),
            Slot::KbRight => "Keyboard right (IJKL + arrows)".into(),
            Slot::Pad(id) => input.gilrs.as_ref().map_or("Controller".into(), |g| g.gamepad(*id).name().to_string()),
            Slot::Virtual(i) => format!("Virtual controller {}", i + 1),
        }
    }

    pub fn help(&self) -> &'static str {
        match self {
            Slot::KbLeft => "W/S throttle  A/D yaw  T/G pitch  F/H roll  Q arm  E cam  Z mode  R reset",
            Slot::KbRight => "I/K throttle  J/L yaw  arrows pitch/roll  U arm  O cam  M mode  P reset",
            Slot::Pad(_) | Slot::Virtual(_) => "Mode 2 sticks  A arm  Y cam  X mode  Select reset  Start pause",
        }
    }
}

/// Per-player smoothing and button-edge state.
#[derive(Clone, Copy, Debug, Default)]
pub struct SlotState {
    kb: Sticks,
    hold: f32,
    prev: u32,
    join_prev: u32,
}

impl SlotState {
    pub fn new() -> Self {
        SlotState { hold: -1.0, ..Default::default() }
    }
}

impl Input {
    /// Call once per frame in split screen, before `poll_slot` / `join_presses`.
    pub fn pump(&mut self) {
        if let Some(g) = &mut self.gilrs {
            while g.next_event().is_some() {}
        }
        self.drain_virtual();
    }

    /// Controllers currently connected (real, then virtual).
    pub fn pads(&self) -> Vec<Slot> {
        let mut v: Vec<Slot> = self.gilrs.as_ref().map_or(vec![], |g| g.gamepads().map(|(id, _)| Slot::Pad(id)).collect());
        if let Some(vp) = &self.vpad {
            v.extend((0..4u8).filter(|i| vp.get(*i as usize).is_some()).map(Slot::Virtual));
        }
        v
    }

    /// Buttons (bit order of `BTN`, D-pad right = bit 9) and axes (lx, ly, rx, ry) of a controller slot.
    fn pad_raw(&self, slot: Slot) -> Option<(u32, [f32; 4])> {
        match slot {
            Slot::Pad(id) => {
                let pad = self.gilrs.as_ref()?.connected_gamepad(id)?;
                let mut bits = 0u32;
                for (i, b) in BTN.iter().enumerate() {
                    if pad.is_pressed(*b) {
                        bits |= 1 << i;
                    }
                }
                if pad.is_pressed(Button::DPadRight) {
                    bits |= 1 << 9;
                }
                Some((bits, [pad.value(Axis::LeftStickX), pad.value(Axis::LeftStickY), pad.value(Axis::RightStickX), pad.value(Axis::RightStickY)]))
            }
            Slot::Virtual(i) => self.vpad.as_ref()?.get(i as usize),
            _ => None,
        }
    }

    /// Accept virtual controllers on 127.0.0.1:`port` (0 = any). Returns the bound port.
    pub fn enable_virtual_pads(&mut self, port: u16) -> std::io::Result<u16> {
        let v = VirtualPads::bind(port)?;
        let p = v.port;
        self.vpad = Some(v);
        Ok(p)
    }

    fn drain_virtual(&mut self) {
        if let Some(v) = self.vpad.as_mut() {
            v.drain();
        }
    }

    /// Slots whose join button (Q, U, or a controller's A/Start) went down this frame.
    pub fn join_presses(&self, st: &mut [(Slot, SlotState)]) -> Vec<Slot> {
        let mut v = vec![];
        if is_key_pressed(KeyCode::Q) {
            v.push(Slot::KbLeft);
        }
        if is_key_pressed(KeyCode::U) {
            v.push(Slot::KbRight);
        }
        for slot in self.pads() {
            let Some((b, _)) = self.pad_raw(slot) else { continue };
            let bits = b & (1 | 1 << 4); // A or Start
            match st.iter_mut().find(|(s, _)| *s == slot) {
                Some((_, ss)) => {
                    if bits & !ss.join_prev != 0 {
                        v.push(slot);
                    }
                    ss.join_prev = bits;
                }
                None if bits != 0 => v.push(slot),
                None => {}
            }
        }
        v
    }

    /// Sticks and buttons for one split-screen player.
    pub fn poll_slot(&self, slot: Slot, st: &mut SlotState, dt: f32, spring_throttle: bool) -> (Sticks, Actions) {
        let mut act = Actions::default();
        let keys = |k: [KeyCode; 8], shift: KeyCode, buttons: [KeyCode; 4], st: &mut SlotState, act: &mut Actions| {
            let full = if is_key_down(shift) { 1.0 } else { 0.6 };
            let ax = |p: KeyCode, n: KeyCode| (is_key_down(p) as i32 - is_key_down(n) as i32) as f32 * full;
            let target = Sticks { throttle: ax(k[0], k[1]), yaw: ax(k[3], k[2]), pitch: ax(k[4], k[5]), roll: ax(k[7], k[6]) };
            act.arm = is_key_pressed(buttons[0]);
            act.camera = is_key_pressed(buttons[1]);
            act.mode = is_key_pressed(buttons[2]);
            act.reset = is_key_pressed(buttons[3]);
            let kk = (dt * 8.0).min(1.0);
            let ease = |a: f32, b: f32| a + (b - a) * kk;
            st.kb.pitch = ease(st.kb.pitch, target.pitch);
            st.kb.roll = ease(st.kb.roll, target.roll);
            st.kb.yaw = ease(st.kb.yaw, target.yaw);
            if spring_throttle {
                st.kb.throttle = ease(st.kb.throttle, target.throttle);
                st.hold = -1.0;
            } else {
                st.hold = (st.hold + target.throttle * dt * 1.2).clamp(-1.0, 1.0);
                st.kb.throttle = st.hold;
            }
            st.kb
        };
        use KeyCode::*;
        let mut sticks = match slot {
            Slot::KbLeft => keys([W, S, A, D, T, G, F, H], LeftShift, [Q, E, Z, R], st, &mut act),
            Slot::KbRight => keys([I, K, J, L, Up, Down, Left, Right], RightShift, [U, O, M, P], st, &mut act),
            Slot::Pad(_) | Slot::Virtual(_) => {
                let Some((bits, ax)) = self.pad_raw(slot) else { return (Sticks::default(), act) };
                let edge = bits & !st.prev;
                st.prev = bits;
                merge_edges(&mut act, edge);
                let d = self.deadzone;
                Sticks { throttle: dz(d, ax[1]), yaw: dz(d, ax[0]), pitch: dz(d, ax[3]), roll: dz(d, ax[2]) }
            }
        };
        if self.invert_pitch {
            sticks.pitch = -sticks.pitch;
        }
        (sticks, act)
    }
}

/// Button-press edges (bit order of `BTN`, D-pad right = bit 9) → actions.
fn merge_edges(act: &mut Actions, edge: u32) {
    let e = |i: u32| edge & (1 << i) != 0;
    act.arm |= e(0);
    act.select |= e(0);
    act.back |= e(1);
    act.camera |= e(2);
    act.mode |= e(3);
    act.pause |= e(4);
    act.reset |= e(5);
    act.up |= e(6);
    act.down |= e(7);
    act.left |= e(8);
    act.right |= e(9);
}

// ---------------------------------------------------------------- virtual controllers

/// Virtual controllers for scripts and tests, on a localhost-only UDP port.
///
/// Packet (little endian): b"RVP1", u8 pad (0–3), u16 buttons, f32 lx, ly, rx, ry.
/// Buttons: 0 A, 1 B, 2 Y, 3 X, 4 Start, 5 Select, 6 D-up, 7 D-down, 8 D-left, 9 D-right.
/// Axes are −1..1 with up/right positive (Mode 2: left = yaw/throttle, right = roll/pitch).
/// A pad disconnects 2 s after its last packet. See python/rflyctl/vpad.py.
pub struct VirtualPads {
    sock: std::net::UdpSocket,
    pub port: u16,
    pads: [Option<(u32, [f32; 4], std::time::Instant)>; 4],
}

pub const VPAD_PORT: u16 = 27170;

impl VirtualPads {
    pub fn bind(port: u16) -> std::io::Result<Self> {
        let sock = std::net::UdpSocket::bind(("127.0.0.1", port))?;
        sock.set_nonblocking(true)?;
        let port = sock.local_addr()?.port();
        Ok(VirtualPads { sock, port, pads: [None; 4] })
    }

    pub fn drain(&mut self) {
        let mut b = [0u8; 64];
        while let Ok((n, _)) = self.sock.recv_from(&mut b) {
            if n < 23 || &b[..4] != b"RVP1" || b[4] > 3 {
                continue;
            }
            let f = |o: usize| f32::from_le_bytes(b[o..o + 4].try_into().unwrap()).clamp(-1.0, 1.0);
            let bits = u16::from_le_bytes([b[5], b[6]]) as u32;
            self.pads[b[4] as usize] = Some((bits, [f(7), f(11), f(15), f(19)], std::time::Instant::now()));
        }
        for p in self.pads.iter_mut() {
            if p.is_some_and(|x| x.2.elapsed().as_secs_f32() > 2.0) {
                *p = None;
            }
        }
    }

    pub fn get(&self, i: usize) -> Option<(u32, [f32; 4])> {
        self.pads.get(i)?.map(|p| (p.0, p.1))
    }
}

impl Default for Input {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn virtual_pad_packets_are_parsed_and_expire() {
        let mut v = VirtualPads::bind(0).unwrap();
        let tx = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
        let mut p = b"RVP1".to_vec();
        p.push(2);
        p.extend_from_slice(&((1u16 << 0) | (1 << 9)).to_le_bytes()); // A + D-pad right
        for a in [0.25f32, 1.0, -0.5, 3.0] {
            p.extend_from_slice(&a.to_le_bytes());
        }
        tx.send_to(&p, ("127.0.0.1", v.port)).unwrap();
        tx.send_to(b"junk", ("127.0.0.1", v.port)).unwrap();
        std::thread::sleep(std::time::Duration::from_millis(50));
        v.drain();
        let (bits, ax) = v.get(2).expect("pad 2 connected");
        assert_eq!(bits, 1 | 1 << 9);
        assert_eq!(ax, [0.25, 1.0, -0.5, 1.0], "axes clamped to ±1");
        assert!(v.get(0).is_none());
        let mut act = Actions::default();
        merge_edges(&mut act, bits);
        assert!(act.arm && act.select && act.right && !act.left);
    }
}
