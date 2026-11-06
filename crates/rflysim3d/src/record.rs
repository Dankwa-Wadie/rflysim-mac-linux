//! Frame recorder for demo videos: `RFLY_RECORD=DIR` writes the rendered window as numbered JPEGs
//! (`DIR/f000000.jpg`, ...) at `RFLY_RECORD_FPS` (default 30) for `RFLY_RECORD_SECS` seconds (default 30),
//! starting after `RFLY_RECORD_AFTER` seconds. Only the app's own frame is captured, never the desktop.
//! Encode with: `ffmpeg -framerate 30 -i DIR/f%06d.jpg -pix_fmt yuv420p out.mp4`.

use macroquad::prelude::*;
use std::sync::mpsc::{sync_channel, SyncSender};

pub struct Recorder {
    tx: Option<SyncSender<(u32, Image)>>,
    fps: f64,
    start: f64,
    end: f64,
    n: u32,
}

impl Recorder {
    pub fn from_env() -> Self {
        let var = |k: &str| std::env::var(k).ok().filter(|v| !v.is_empty());
        let num = |k: &str, d: f64| var(k).and_then(|v| v.parse().ok()).unwrap_or(d);
        let (fps, after, secs) = (num("RFLY_RECORD_FPS", 30.0), num("RFLY_RECORD_AFTER", 0.0), num("RFLY_RECORD_SECS", 30.0));
        let tx = var("RFLY_RECORD").map(|dir| {
            let _ = std::fs::create_dir_all(&dir);
            // Bounded queue: if encoding falls behind, the render loop waits instead of using unbounded memory.
            let (tx, rx) = sync_channel::<(u32, Image)>(8);
            let out = dir.clone();
            std::thread::spawn(move || {
                for (n, img) in rx {
                    let (w, h) = (img.width as u32, img.height as u32);
                    let Some(rgba) = image::RgbaImage::from_raw(w, h, img.bytes) else { continue };
                    // GL reads rows bottom-up.
                    let rgb = image::DynamicImage::ImageRgba8(image::imageops::flip_vertical(&rgba)).to_rgb8();
                    let _ = rgb.save_with_format(format!("{out}/f{n:06}.jpg"), image::ImageFormat::Jpeg);
                }
            });
            eprintln!("recording frames to {dir} ({fps} fps, {secs} s after {after} s)");
            tx
        });
        let t = get_time();
        Recorder { tx, fps, start: t + after, end: t + after + secs, n: 0 }
    }

    /// Call once per frame after drawing, before `next_frame()`. Captures when a frame is due.
    pub fn frame(&mut self) {
        let Some(tx) = &self.tx else { return };
        let t = get_time();
        if t < self.start {
            return;
        }
        if t > self.end {
            self.tx = None; // closes the channel; the encoder thread drains and exits
            eprintln!("recording done: {} frames", self.n);
            return;
        }
        // Fixed-rate sampling on wall time; frames that come faster than the target rate are skipped.
        if (t - self.start) * self.fps >= self.n as f64 {
            let _ = tx.send((self.n, get_screen_data()));
            self.n += 1;
        }
    }

    pub fn active(&self) -> bool {
        self.tx.is_some()
    }
}

thread_local! {
    static REC: std::cell::RefCell<Option<Recorder>> = const { std::cell::RefCell::new(None) };
}

/// Per-frame hook for any render loop: records when `RFLY_RECORD` is set, does nothing otherwise.
pub fn tick() {
    REC.with(|r| r.borrow_mut().get_or_insert_with(Recorder::from_env).frame());
}
