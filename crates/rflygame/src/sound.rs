//! Game audio, all synthesised at start-up (no sound files to ship).
//!
//! - Motor hum: four looping layers at rising pitch, cross-faded by motor speed (macroquad's audio
//!   has no pitch control, so the blend between layers carries the "spool up" feel).
//! - Wind: a noise loop whose volume follows airspeed.
//! - One-shots for `Sfx` events (impacts, bumps, countdown, gates, laps, results) and arm/disarm beeps.
//!
//! Muted in `--auto` test runs unless RFLY_SOUND=1.

use crate::game::{Session, Sfx};
use macroquad::audio::{load_sound_from_bytes, play_sound, set_sound_volume, PlaySoundParams, Sound};

const RATE: u32 = 44100;

/// 16-bit mono PCM WAV.
pub fn wav(samples: &[f32]) -> Vec<u8> {
    let data_len = (samples.len() * 2) as u32;
    let mut b = Vec::with_capacity(44 + data_len as usize);
    b.extend_from_slice(b"RIFF");
    b.extend_from_slice(&(36 + data_len).to_le_bytes());
    b.extend_from_slice(b"WAVEfmt ");
    b.extend_from_slice(&16u32.to_le_bytes());
    b.extend_from_slice(&1u16.to_le_bytes()); // PCM
    b.extend_from_slice(&1u16.to_le_bytes()); // mono
    b.extend_from_slice(&RATE.to_le_bytes());
    b.extend_from_slice(&(RATE * 2).to_le_bytes());
    b.extend_from_slice(&2u16.to_le_bytes());
    b.extend_from_slice(&16u16.to_le_bytes());
    b.extend_from_slice(b"data");
    b.extend_from_slice(&data_len.to_le_bytes());
    for &s in samples {
        b.extend_from_slice(&((s.clamp(-1.0, 1.0) * 32000.0) as i16).to_le_bytes());
    }
    b
}

/// Small deterministic noise source (xorshift).
struct Noise(u32);
impl Noise {
    fn next(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        (self.0 as f32 / u32::MAX as f32) * 2.0 - 1.0
    }
}

fn secs(s: f32) -> usize {
    (s * RATE as f32) as usize
}

/// One motor layer: 1 s loop with an integer fundamental (seamless), prop blade-pass harmonics,
/// a little filtered noise for air.
pub fn motor_layer(f: f32, seed: u32) -> Vec<f32> {
    let n = secs(1.0);
    let mut nz = Noise(seed);
    let mut lp = 0.0f32;
    (0..n)
        .map(|i| {
            let t = i as f32 / RATE as f32;
            let w = std::f32::consts::TAU * f * t;
            let tone = (1..=6).map(|k| (w * k as f32).sin() / (k as f32).powf(1.3)).sum::<f32>();
            lp += (nz.next() - lp) * 0.15;
            (tone * 0.32 + lp * 0.35) * 0.8
        })
        .collect()
}

/// Wind noise loop (2 s, low-passed, ends cross-faded into its start so it loops without a click).
pub fn wind() -> Vec<f32> {
    let n = secs(2.0);
    let fade = secs(0.2);
    let mut nz = Noise(0x9e3779b9);
    let (mut a, mut b) = (0.0f32, 0.0f32);
    let mut v: Vec<f32> = (0..n + fade)
        .map(|_| {
            a += (nz.next() - a) * 0.05;
            b += (a - b) * 0.08;
            b * 2.5
        })
        .collect();
    for i in 0..fade {
        let k = i as f32 / fade as f32;
        v[i] = v[i] * k + v[n + i] * (1.0 - k);
    }
    v.truncate(n);
    v
}

fn tone(out: &mut Vec<f32>, f: f32, dur: f32, amp: f32) {
    let n = secs(dur);
    let start = out.len();
    out.resize(start + n, 0.0);
    for i in 0..n {
        let t = i as f32 / RATE as f32;
        let env = (t / 0.005).min(1.0) * (1.0 - i as f32 / n as f32).powf(0.6);
        let w = std::f32::consts::TAU * f * t;
        out[start + i] += (w.sin() * 0.8 + (2.0 * w).sin() * 0.2) * amp * env;
    }
}

fn gap(out: &mut Vec<f32>, dur: f32) {
    out.resize(out.len() + secs(dur), 0.0);
}

/// A thud: decaying low sine plus a burst of filtered noise.
pub fn thud(dur: f32, amp: f32, seed: u32) -> Vec<f32> {
    let n = secs(dur);
    let mut nz = Noise(seed);
    let mut lp = 0.0f32;
    (0..n)
        .map(|i| {
            let t = i as f32 / RATE as f32;
            let env = (-t * 9.0 / dur).exp();
            lp += (nz.next() - lp) * 0.25;
            let low = (std::f32::consts::TAU * (70.0 - 30.0 * t / dur) * t).sin();
            (low * 0.7 + lp * 0.9) * env * amp
        })
        .collect()
}

pub fn effect(e: Sfx) -> Vec<f32> {
    let mut v = Vec::new();
    match e {
        Sfx::Impact => return thud(0.5, 0.9, 7),
        Sfx::Bump => return thud(0.18, 0.45, 11),
        Sfx::Beep => tone(&mut v, 880.0, 0.15, 0.5),
        Sfx::Go => tone(&mut v, 1320.0, 0.45, 0.55),
        Sfx::Gate => {
            tone(&mut v, 990.0, 0.09, 0.4);
            tone(&mut v, 1480.0, 0.14, 0.4);
        }
        Sfx::Lap => {
            for f in [880.0, 1108.7, 1318.5] {
                tone(&mut v, f, 0.11, 0.45);
            }
        }
        Sfx::Win => {
            for f in [523.3, 659.3, 784.0] {
                tone(&mut v, f, 0.13, 0.45);
            }
            tone(&mut v, 1046.5, 0.5, 0.5);
        }
        Sfx::Lose => {
            tone(&mut v, 440.0, 0.22, 0.45);
            tone(&mut v, 330.0, 0.45, 0.45);
        }
    }
    v
}

fn arm_beep(arm: bool) -> Vec<f32> {
    let mut v = Vec::new();
    if arm {
        tone(&mut v, 1046.5, 0.08, 0.35);
        gap(&mut v, 0.05);
        tone(&mut v, 1568.0, 0.12, 0.35);
    } else {
        tone(&mut v, 1046.5, 0.08, 0.35);
        gap(&mut v, 0.05);
        tone(&mut v, 698.5, 0.14, 0.35);
    }
    v
}

/// Motor layer fundamentals (Hz), low → high spool.
pub const LAYERS: [f32; 4] = [85.0, 130.0, 195.0, 290.0];

/// Layer volumes for a normalised motor speed n ∈ [0, 1].
pub fn layer_mix(n: f32) -> [f32; 4] {
    let mut m = [0.0; 4];
    let x = n.clamp(0.0, 1.0) * 3.0;
    for (i, w) in m.iter_mut().enumerate() {
        *w = (1.0 - (x - i as f32).abs()).max(0.0);
    }
    m
}

pub struct Audio {
    motor: Vec<Sound>,
    wind: Option<Sound>,
    fx: Vec<(Sfx, Sound)>,
    arm: Option<Sound>,
    disarm: Option<Sound>,
    vols: [f32; 5],
    last_armed: bool,
    muted: bool,
}

async fn load(samples: &[f32]) -> Option<Sound> {
    load_sound_from_bytes(&wav(samples)).await.ok()
}

impl Audio {
    pub async fn new(muted: bool) -> Self {
        let mut a = Audio { motor: Vec::new(), wind: None, fx: Vec::new(), arm: None, disarm: None, vols: [-1.0; 5], last_armed: false, muted };
        if muted {
            return a;
        }
        for (i, f) in LAYERS.iter().enumerate() {
            if let Some(s) = load(&motor_layer(*f, 1 + i as u32)).await {
                play_sound(&s, PlaySoundParams { looped: true, volume: 0.0 });
                a.motor.push(s);
            }
        }
        a.wind = load(&wind()).await;
        if let Some(s) = &a.wind {
            play_sound(s, PlaySoundParams { looped: true, volume: 0.0 });
        }
        for e in [Sfx::Impact, Sfx::Bump, Sfx::Beep, Sfx::Go, Sfx::Gate, Sfx::Lap, Sfx::Win, Sfx::Lose] {
            if let Some(s) = load(&effect(e)).await {
                a.fx.push((e, s));
            }
        }
        a.arm = load(&arm_beep(true)).await;
        a.disarm = load(&arm_beep(false)).await;
        a
    }

    fn master(volume: u8) -> f32 {
        (volume.min(10) as f32 / 10.0).powf(1.5)
    }

    pub fn play(&self, e: Sfx, volume: u8) {
        if self.muted || volume == 0 {
            return;
        }
        if let Some((_, s)) = self.fx.iter().find(|(k, _)| *k == e) {
            play_sound(s, PlaySoundParams { looped: false, volume: Self::master(volume) });
        }
    }

    fn set(&mut self, i: usize, v: f32) {
        if (self.vols[i] - v).abs() > 0.01 {
            self.vols[i] = v;
            let s = if i < 4 { self.motor.get(i) } else { self.wind.as_ref() };
            if let Some(s) = s {
                set_sound_volume(s, v);
            }
        }
    }

    /// Call once per frame. `live` = the session is being flown or watched (not a menu or pause).
    pub fn update(&mut self, sess: Option<&mut Session>, live: bool, volume: u8) {
        let master = if self.muted { 0.0 } else { Self::master(volume) };
        let (mut spin, mut air) = (0.0f32, 0.0f32);
        if let Some(s) = sess {
            for e in s.sfx.drain(..) {
                if live {
                    self.play(e, volume);
                }
            }
            if s.armed != self.last_armed {
                self.last_armed = s.armed;
                let b = if s.armed { &self.arm } else { &self.disarm };
                if let (Some(b), true, false) = (b, live, self.muted) {
                    play_sound(b, PlaySoundParams { looped: false, volume: master * 0.8 });
                }
            }
            if live {
                let w_max = (s.v.p.cr + s.v.p.wb).max(1.0) as f32;
                let w = s.v.s.motor_w.iter().cloned().fold(0.0f64, f64::max) as f32;
                spin = (w / w_max).clamp(0.0, 1.0);
                air = (s.speed() as f32 / 25.0).clamp(0.0, 1.0);
            }
        }
        let mix = layer_mix(spin);
        let loud = if spin > 0.02 { 0.25 + 0.75 * spin } else { 0.0 };
        for i in 0..4 {
            self.set(i, mix[i] * loud * master * 0.55);
        }
        self.set(4, air * air * master * 0.6);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wav_header_and_length() {
        let b = wav(&[0.0, 0.5, -0.5]);
        assert_eq!(&b[0..4], b"RIFF");
        assert_eq!(&b[8..12], b"WAVE");
        assert_eq!(b.len(), 44 + 6);
        assert_eq!(u32::from_le_bytes([b[24], b[25], b[26], b[27]]), RATE);
    }

    #[test]
    fn motor_layer_loops_seamlessly_and_stays_in_range() {
        for f in LAYERS {
            let v = motor_layer(f, 3);
            assert_eq!(v.len(), RATE as usize);
            assert!(v.iter().all(|x| x.abs() <= 1.0));
            // Integer fundamental over exactly 1 s: the tone part wraps to its start.
            let jump = (v[v.len() - 1] - v[0]).abs();
            assert!(jump < 0.4, "loop seam jump {jump}");
        }
    }

    #[test]
    fn layer_mix_crossfades_with_motor_speed() {
        assert_eq!(layer_mix(0.0), [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(layer_mix(1.0), [0.0, 0.0, 0.0, 1.0]);
        let m = layer_mix(0.5);
        assert!((m.iter().sum::<f32>() - 1.0).abs() < 1e-5);
        assert!(m[1] > 0.0 && m[2] > 0.0);
    }

    #[test]
    fn effects_are_short_and_clipped() {
        for e in [Sfx::Impact, Sfx::Bump, Sfx::Beep, Sfx::Go, Sfx::Gate, Sfx::Lap, Sfx::Win, Sfx::Lose] {
            let v = effect(e);
            assert!(!v.is_empty() && v.len() < secs(1.5), "{e:?}");
            assert!(v.iter().all(|x| x.abs() <= 1.2), "{e:?}");
        }
    }
}
