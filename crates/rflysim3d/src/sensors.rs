//! Vision sensors (`VisionSensorReq`): off-screen cameras whose frames are encoded and
//! streamed over UDP in RflySim's image packet format.
//!
//! TypeID: 1 RGB, 2 depth (uint16 mm), 3 grey, 4 segmentation.
//! SendProtocol[0]: 1 JPEG, 2 raw (BGR / uint16 / grey), 3 PNG; 0 (Windows shared memory)
//! is served as JPEG over UDP, since macOS has no RflySim shared memory.

use macroquad::prelude::*;
use rfly_proto::{image_packets, VisionSensorReq};
use std::io::Cursor;
use std::net::{SocketAddr, UdpSocket};
use std::sync::mpsc::{channel, Sender};

pub struct Sensor {
    pub req: VisionSensorReq,
    pub rt: RenderTarget,
    pub next_due: f64,
    pub pending: Option<f64>,
    pub frames: u64,
}

impl Sensor {
    pub fn new(req: VisionSensorReq) -> Self {
        let (w, h) = (req.width.clamp(16, 4096) as u32, req.height.clamp(16, 4096) as u32);
        let rt = render_target_ex(w, h, RenderTargetParams { sample_count: 1, depth: true });
        rt.texture.set_filter(FilterMode::Nearest);
        Sensor { req, rt, next_due: 0.0, pending: None, frames: 0 }
    }
    pub fn period(&self) -> f64 {
        1.0 / (self.req.check_freq.max(1) as f64).min(120.0)
    }
}

pub struct Job {
    pub req: VisionSensorReq,
    pub rgba: Vec<u8>,
    pub w: u32,
    pub h: u32,
    pub stamp: f64,
}

/// Encoder/sender thread so JPEG/PNG encoding never stalls the render loop.
pub fn spawn_encoder(sock: UdpSocket) -> Sender<Job> {
    let (tx, rx) = channel::<Job>();
    std::thread::spawn(move || {
        for job in rx {
            let data = encode(&job);
            let ip = job.req.dest_ip();
            let to = SocketAddr::from((ip, job.req.dest_port()));
            for p in image_packets(&data, job.stamp) {
                if let Err(e) = sock.send_to(&p, to) {
                    eprintln!("rflysim3d: image send to {to} failed: {e}");
                }
            }
        }
    });
    tx
}

/// Converts the RGBA readback (bottom-up rows) into what the requester expects.
pub fn encode(job: &Job) -> Vec<u8> {
    let (w, h) = (job.w as usize, job.h as usize);
    let row = |y: usize| &job.rgba[(h - 1 - y) * w * 4..(h - y) * w * 4];
    let mode = job.req.send_protocol[0];
    let mut out = Vec::new();
    match job.req.type_id {
        2 => {
            // depth: R = high byte, G = low byte of millimetres.
            let mut d = Vec::with_capacity(w * h);
            for y in 0..h {
                for px in row(y).chunks_exact(4) {
                    d.push(((px[0] as u16) << 8) | px[1] as u16);
                }
            }
            if mode == 2 {
                out = d.iter().flat_map(|v| v.to_le_bytes()).collect();
            } else {
                let img = image::ImageBuffer::<image::Luma<u16>, _>::from_raw(job.w, job.h, d).unwrap();
                img.write_to(&mut Cursor::new(&mut out), image::ImageFormat::Png).unwrap();
            }
        }
        3 => {
            let g: Vec<u8> = (0..h)
                .flat_map(|y| row(y).chunks_exact(4).map(|p| (0.299 * p[0] as f32 + 0.587 * p[1] as f32 + 0.114 * p[2] as f32) as u8).collect::<Vec<_>>())
                .collect();
            if mode == 2 {
                out = g;
            } else {
                let img = image::GrayImage::from_raw(job.w, job.h, g).unwrap();
                let fmt = if mode == 3 { image::ImageFormat::Png } else { image::ImageFormat::Jpeg };
                img.write_to(&mut Cursor::new(&mut out), fmt).unwrap();
            }
        }
        _ => {
            let rgb: Vec<u8> = (0..h).flat_map(|y| row(y).chunks_exact(4).flat_map(|p| [p[0], p[1], p[2]]).collect::<Vec<_>>()).collect();
            if mode == 2 {
                // OpenCV order.
                out = rgb.chunks_exact(3).flat_map(|p| [p[2], p[1], p[0]]).collect();
            } else {
                let img = image::RgbImage::from_raw(job.w, job.h, rgb).unwrap();
                if mode == 3 {
                    img.write_to(&mut Cursor::new(&mut out), image::ImageFormat::Png).unwrap();
                } else {
                    let mut enc = image::codecs::jpeg::JpegEncoder::new_with_quality(&mut out, 85);
                    enc.encode_image(&img).unwrap();
                }
            }
        }
    }
    out
}
