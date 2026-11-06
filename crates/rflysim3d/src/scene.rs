//! Geometry, maps and the lit/depth/segmentation material.
//!
//! All geometry is authored in **NED metres**: world objects in world NED, vehicle parts in
//! body FRD. One fixed matrix (`ned_to_gl`) maps NED onto macroquad's Y-up GL frame.

use macroquad::models::Vertex;
use macroquad::miniquad::{BlendFactor, BlendState, BlendValue, Equation};
use macroquad::prelude::*;

pub fn ned_to_gl() -> Mat4 {
    Mat4::from_cols(
        vec4(1.0, 0.0, 0.0, 0.0), // N → +X
        vec4(0.0, 0.0, 1.0, 0.0), // E → +Z
        vec4(0.0, -1.0, 0.0, 0.0), // D → −Y
        vec4(0.0, 0.0, 0.0, 1.0),
    )
}
pub fn v_ned_to_gl(v: Vec3) -> Vec3 {
    vec3(v.x, -v.z, v.y)
}
/// Body FRD → NED rotation from roll/pitch/yaw (rad), ZYX order.
pub fn rot(euler: [f32; 3]) -> Quat {
    Quat::from_euler(EulerRot::ZYX, euler[2], euler[1], euler[0])
}

// ---------------------------------------------------------------- mesh building

#[derive(Default, Clone)]
pub struct MeshBuf {
    pub v: Vec<Vertex>,
    pub i: Vec<u16>,
}

impl MeshBuf {
    fn vert(&mut self, p: Vec3, n: Vec3, uv: Vec2, c: Color) -> u16 {
        self.v.push(Vertex {
            position: p,
            uv,
            color: [(c.r * 255.0) as u8, (c.g * 255.0) as u8, (c.b * 255.0) as u8, (c.a * 255.0) as u8],
            normal: vec4(n.x, n.y, n.z, 0.0),
        });
        (self.v.len() - 1) as u16
    }
    /// Quad with corners a,b,c,d (counter-clockwise seen from the normal side).
    pub fn quad(&mut self, a: Vec3, b: Vec3, c: Vec3, d: Vec3, col: Color, uv: [Vec2; 4]) {
        let n = (b - a).cross(d - a).normalize_or_zero();
        let k = [self.vert(a, n, uv[0], col), self.vert(b, n, uv[1], col), self.vert(c, n, uv[2], col), self.vert(d, n, uv[3], col)];
        self.i.extend_from_slice(&[k[0], k[1], k[2], k[0], k[2], k[3]]);
    }
    /// Box centred at `c` with half extents `h`, optionally rotated by `q`.
    pub fn cuboid_rot(&mut self, c: Vec3, h: Vec3, q: Quat, col: Color) {
        let p = |x: f32, y: f32, z: f32| c + q * vec3(x * h.x, y * h.y, z * h.z);
        let uv = [vec2(0., 0.), vec2(1., 0.), vec2(1., 1.), vec2(0., 1.)];
        // Faces: winding chosen so (b-a)×(d-a) points outward.
        self.quad(p(1., -1., -1.), p(1., 1., -1.), p(1., 1., 1.), p(1., -1., 1.), col, uv); // +x
        self.quad(p(-1., 1., -1.), p(-1., -1., -1.), p(-1., -1., 1.), p(-1., 1., 1.), col, uv); // -x
        self.quad(p(1., 1., -1.), p(-1., 1., -1.), p(-1., 1., 1.), p(1., 1., 1.), col, uv); // +y
        self.quad(p(-1., -1., -1.), p(1., -1., -1.), p(1., -1., 1.), p(-1., -1., 1.), col, uv); // -y
        self.quad(p(-1., -1., 1.), p(1., -1., 1.), p(1., 1., 1.), p(-1., 1., 1.), col, uv); // +z (down)
        self.quad(p(-1., 1., -1.), p(1., 1., -1.), p(1., -1., -1.), p(-1., -1., -1.), col, uv); // -z (up)
    }
    pub fn cuboid(&mut self, c: Vec3, h: Vec3, col: Color) {
        self.cuboid_rot(c, h, Quat::IDENTITY, col)
    }
    /// Cylinder along +z (down in NED) from `c - z*h/2` to `c + z*h/2`.
    pub fn cylinder(&mut self, c: Vec3, r_top: f32, r_bot: f32, h: f32, segs: usize, col: Color) {
        let top = c - vec3(0., 0., h / 2.);
        let bot = c + vec3(0., 0., h / 2.);
        for s in 0..segs {
            let a0 = s as f32 / segs as f32 * std::f32::consts::TAU;
            let a1 = (s + 1) as f32 / segs as f32 * std::f32::consts::TAU;
            let (d0, d1) = (vec3(a0.cos(), a0.sin(), 0.), vec3(a1.cos(), a1.sin(), 0.));
            let uv = [vec2(0., 0.), vec2(1., 0.), vec2(1., 1.), vec2(0., 1.)];
            let slope = (r_bot - r_top) / h.max(1e-3);
            let n0 = (d0 - vec3(0., 0., slope)).normalize();
            let n1 = (d1 - vec3(0., 0., slope)).normalize();
            let k = [
                self.vert(top + d0 * r_top, n0, uv[0], col),
                self.vert(bot + d0 * r_bot, n0, uv[1], col),
                self.vert(bot + d1 * r_bot, n1, uv[2], col),
                self.vert(top + d1 * r_top, n1, uv[3], col),
            ];
            // Outward winding for a NED (right-handed) frame.
            self.i.extend_from_slice(&[k[0], k[2], k[1], k[0], k[3], k[2]]);
            if r_top > 0.0 {
                let t = [self.vert(top, vec3(0., 0., -1.), uv[0], col), self.vert(top + d0 * r_top, vec3(0., 0., -1.), uv[0], col), self.vert(top + d1 * r_top, vec3(0., 0., -1.), uv[0], col)];
                self.i.extend_from_slice(&[t[0], t[1], t[2]]);
            }
            if r_bot > 0.0 {
                let b = [self.vert(bot, vec3(0., 0., 1.), uv[0], col), self.vert(bot + d1 * r_bot, vec3(0., 0., 1.), uv[0], col), self.vert(bot + d0 * r_bot, vec3(0., 0., 1.), uv[0], col)];
                self.i.extend_from_slice(&[b[0], b[1], b[2]]);
            }
        }
    }
    pub fn sphere(&mut self, c: Vec3, r: f32, col: Color) {
        let (rings, segs) = (8, 12);
        let base = self.v.len() as u16;
        for a in 0..=rings {
            let th = a as f32 / rings as f32 * std::f32::consts::PI;
            for b in 0..=segs {
                let ph = b as f32 / segs as f32 * std::f32::consts::TAU;
                let n = vec3(th.sin() * ph.cos(), th.sin() * ph.sin(), th.cos());
                self.vert(c + n * r, n, vec2(0., 0.), col);
            }
        }
        for a in 0..rings {
            for b in 0..segs {
                let i0 = base + (a * (segs + 1) + b) as u16;
                let i1 = i0 + (segs + 1) as u16;
                self.i.extend_from_slice(&[i0, i0 + 1, i1, i1, i0 + 1, i1 + 1]);
            }
        }
    }
    pub fn to_mesh(&self, tex: Option<Texture2D>) -> Mesh {
        Mesh { vertices: self.v.clone(), indices: self.i.clone(), texture: tex }
    }
}

// ---------------------------------------------------------------- vehicle models

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ModelKind {
    Multirotor(usize),
    FixedWing,
    Car,
    Box,
}

/// RflySim `vehicleType` → our procedural model. RflySim3D's own table is configurable per
/// install; these defaults follow the common SDK examples (3 = quad, 5/6 = hexa, 8 = octo,
/// 100s = fixed wing, 30s/50s = ground vehicles, 150+ = generic objects).
pub fn model_for_type(t: i32) -> ModelKind {
    match t {
        5 | 6 | 11 => ModelKind::Multirotor(6),
        8 | 9 | 12 => ModelKind::Multirotor(8),
        100..=149 => ModelKind::FixedWing,
        30..=99 => ModelKind::Car,
        150.. => ModelKind::Box,
        _ => ModelKind::Multirotor(4),
    }
}

pub struct VehicleModel {
    pub body: Mesh,
    pub prop: Mesh,
    pub disk: Mesh,
    /// Rotor hub positions (FRD) and spin direction.
    pub rotors: Vec<(Vec3, f32)>,
    /// Half extents of the bounding box (FRD).
    pub half: Vec3,
    /// How far the lowest point (e.g. landing legs) is below the model origin (m, before scaling).
    /// CopterSim reports z ≈ 0 for a landed vehicle, so RflySim3D-style viewers lift the model by this.
    pub ground: f32,
}

pub fn build_model(kind: ModelKind, tint: Color) -> VehicleModel {
    let mut b = MeshBuf::default();
    let dark = Color::new(0.15, 0.15, 0.17, 1.0);
    let mut rotors = Vec::new();
    let half;
    match kind {
        ModelKind::Multirotor(n) => {
            let arm = if n == 4 { 0.225 } else { 0.28 };
            b.cuboid(vec3(0., 0., 0.), vec3(0.08, 0.08, 0.035), tint);
            b.cuboid(vec3(0.03, 0., -0.045), vec3(0.05, 0.05, 0.012), dark); // top plate / GPS
            b.cuboid(vec3(0.11, 0., 0.0), vec3(0.02, 0.025, 0.02), RED); // nose marker
            for k in 0..n {
                // PX4 quad-x: rotors at 45°,225° (CCW) / 135°,315° (CW).
                let a = if n == 4 { [45f32, -135., -45., 135.][k] } else { (k as f32 + 0.5) * 360.0 / n as f32 };
                let dir = vec3(a.to_radians().cos(), a.to_radians().sin(), 0.);
                let q = Quat::from_rotation_z(a.to_radians());
                b.cuboid_rot(dir * arm * 0.5, vec3(arm * 0.5, 0.012, 0.012), q, dark);
                b.cylinder(dir * arm + vec3(0., 0., -0.02), 0.018, 0.018, 0.035, 10, Color::new(0.6, 0.6, 0.62, 1.));
                let spin = if n == 4 { [1., 1., -1., -1.][k] } else if k % 2 == 0 { 1. } else { -1. };
                rotors.push((dir * arm + vec3(0., 0., -0.04), spin));
                b.cuboid(dir * arm + vec3(0., 0., 0.06), vec3(0.006, 0.006, 0.05), dark); // leg
            }
            half = vec3(arm + 0.12, arm + 0.12, 0.1);
        }
        ModelKind::FixedWing => {
            b.cylinder(vec3(0., 0., 0.), 0.06, 0.06, 0.1, 10, tint); // placeholder hub
            b.cuboid(vec3(0., 0., 0.), vec3(0.6, 0.07, 0.07), tint); // fuselage
            b.cuboid(vec3(0.08, 0., -0.03), vec3(0.14, 0.9, 0.012), Color::new(0.9, 0.9, 0.92, 1.)); // wing
            b.cuboid(vec3(-0.55, 0., -0.03), vec3(0.07, 0.3, 0.008), Color::new(0.9, 0.9, 0.92, 1.)); // tailplane
            b.cuboid(vec3(-0.55, 0., -0.15), vec3(0.07, 0.008, 0.12), RED); // fin
            rotors.push((vec3(0.62, 0., 0.), 1.0));
            half = vec3(0.65, 0.9, 0.2);
        }
        ModelKind::Car => {
            b.cuboid(vec3(0., 0., -0.35), vec3(1.1, 0.5, 0.25), tint);
            b.cuboid(vec3(-0.1, 0., -0.75), vec3(0.6, 0.45, 0.18), Color::new(0.5, 0.7, 0.9, 1.));
            for (x, y) in [(0.7, 0.5), (0.7, -0.5), (-0.7, 0.5), (-0.7, -0.5)] {
                b.cuboid(vec3(x, y, -0.15), vec3(0.15, 0.06, 0.15), dark);
            }
            half = vec3(1.1, 0.55, 0.47);
        }
        ModelKind::Box => {
            b.cuboid(vec3(0., 0., 0.), vec3(0.5, 0.5, 0.5), tint);
            half = vec3(0.5, 0.5, 0.5);
        }
    }
    let mut p = MeshBuf::default();
    let blade = if kind == ModelKind::FixedWing { 0.12 } else { 0.115 };
    p.cuboid(vec3(0., 0., 0.), vec3(blade, 0.012, 0.003), Color::new(0.1, 0.1, 0.1, 1.));
    let mut d = MeshBuf::default();
    d.cylinder(vec3(0., 0., 0.), blade, blade, 0.002, 20, Color::new(0.2, 0.2, 0.2, 0.35));
    let ground = b.v.iter().map(|v| v.position.z).fold(0.0f32, f32::max);
    VehicleModel { body: b.to_mesh(None), prop: p.to_mesh(None), disk: d.to_mesh(None), rotors, half, ground }
}

// ---------------------------------------------------------------- maps

pub struct StaticObj {
    pub name: String,
    /// Axis-aligned box in NED (centre, half extents) for collision / segmentation / requests.
    pub centre: Vec3,
    pub half: Vec3,
}

pub struct Map {
    pub name: String,
    pub sky: Color,
    pub fog: f32,
    pub meshes: Vec<Mesh>,
    pub objects: Vec<StaticObj>,
    pub objects_mesh: Vec<Mesh>,
}

pub const MAP_NAMES: &[&str] = &["Grasslands", "City", "Blank", "VisionRingBlank", "Desert", "Raceway"];

struct Rng(u64);
impl Rng {
    fn f(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 40) as f32 / (1u64 << 24) as f32
    }
    fn r(&mut self, a: f32, b: f32) -> f32 {
        a + (b - a) * self.f()
    }
}

fn ground_texture(base: [f32; 3], line: [f32; 3], noise: f32, seed: u64) -> Texture2D {
    const N: usize = 256;
    let mut rng = Rng(seed);
    let mut px = vec![0u8; N * N * 4];
    for y in 0..N {
        for x in 0..N {
            let grid = x % 64 == 0 || y % 64 == 0;
            let fine = x % 8 == 0 || y % 8 == 0;
            let n = 1.0 + (rng.f() - 0.5) * noise;
            let c = if grid {
                line
            } else if fine {
                [base[0] * 0.93, base[1] * 0.93, base[2] * 0.93]
            } else {
                [base[0] * n, base[1] * n, base[2] * n]
            };
            let o = (y * N + x) * 4;
            for k in 0..3 {
                px[o + k] = (c[k].clamp(0., 1.) * 255.0) as u8;
            }
            px[o + 3] = 255;
        }
    }
    let t = Texture2D::from_rgba8(N as u16, N as u16, &px);
    t.set_filter(FilterMode::Linear);
    t
}

/// Flat ground made of 40 m tiles (one texture repeat each; 64 px = 10 m grid).
fn ground(size: f32, tex: Texture2D) -> Vec<Mesh> {
    let tile = 40.0;
    let n = (size / tile) as i32;
    let mut out = Vec::new();
    let mut b = MeshBuf::default();
    let uv = [vec2(0., 0.), vec2(1., 0.), vec2(1., 1.), vec2(0., 1.)];
    for i in -n / 2..n / 2 {
        for j in -n / 2..n / 2 {
            let (x, y) = (i as f32 * tile, j as f32 * tile);
            // Up-facing (−z) quad: winding a→b→c with (b−a)×(d−a) = −z.
            b.quad(vec3(x, y, 0.), vec3(x, y + tile, 0.), vec3(x + tile, y + tile, 0.), vec3(x + tile, y, 0.), WHITE, uv);
            if b.v.len() > 60000 {
                out.push(std::mem::take(&mut b).to_mesh(Some(tex.clone())));
            }
        }
    }
    out.push(b.to_mesh(Some(tex)));
    out
}

fn push_obj(objs: &mut Vec<StaticObj>, b: &mut MeshBuf, meshes: &mut Vec<Mesh>, name: String, c: Vec3, h: Vec3) {
    objs.push(StaticObj { name, centre: c, half: h });
    if b.v.len() > 55000 {
        meshes.push(std::mem::take(b).to_mesh(None));
    }
}

pub fn build_map(name: &str) -> Map {
    let key = name.to_ascii_lowercase();
    let mut objects = Vec::new();
    let mut ob = MeshBuf::default();
    let mut objects_mesh = Vec::new();
    let mut rng = Rng(0x9e3779b97f4a7c15);
    let size = 2000.0;
    let (sky, fog, tex) = if key.contains("city") {
        (Color::new(0.62, 0.74, 0.86, 1.), 900.0, ground_texture([0.42, 0.42, 0.44], [0.75, 0.75, 0.7], 0.08, 7))
    } else if key.contains("desert") {
        (Color::new(0.80, 0.78, 0.70, 1.), 700.0, ground_texture([0.80, 0.68, 0.48], [0.72, 0.60, 0.42], 0.12, 3))
    } else if key.contains("blank") || key.contains("vision") {
        (Color::new(0.85, 0.87, 0.90, 1.), 1500.0, ground_texture([0.78, 0.78, 0.78], [0.45, 0.45, 0.45], 0.0, 1))
    } else {
        (Color::new(0.55, 0.72, 0.92, 1.), 800.0, ground_texture([0.33, 0.52, 0.22], [0.42, 0.60, 0.30], 0.18, 11))
    };

    if key.contains("city") {
        for i in -8..=8 {
            for j in -8..=8 {
                if i == 0 && j == 0 {
                    continue;
                }
                let h = rng.r(8.0, if (i * i + j * j) < 9 { 30.0 } else { 70.0 });
                let c = vec3(i as f32 * 40.0, j as f32 * 40.0, -h / 2.0);
                let half = vec3(rng.r(8.0, 15.0), rng.r(8.0, 15.0), h / 2.0);
                let g = rng.r(0.55, 0.85);
                ob.cuboid(c, half, Color::new(g, g * 0.98, g * 0.95, 1.));
                // Window bands for scale.
                let mut z = -3.0;
                while z > -h + 2.0 {
                    ob.cuboid(vec3(c.x, c.y, z), vec3(half.x + 0.05, half.y + 0.05, 0.6), Color::new(0.25, 0.32, 0.42, 1.));
                    z -= 4.0;
                }
                push_obj(&mut objects, &mut ob, &mut objects_mesh, format!("Building_{}_{}", i, j), c, half);
            }
        }
    } else if key.contains("vision") {
        // Ring gates along the north axis, like RflySim's VisionRing scenes.
        for k in 0..6 {
            let c = vec3(10.0 + k as f32 * 12.0, (k as f32 * 1.3).sin() * 4.0, -2.5);
            let segs = 16;
            for s in 0..segs {
                let a = s as f32 / segs as f32 * std::f32::consts::TAU;
                let p = c + vec3(0., a.cos() * 1.2, a.sin() * 1.2);
                ob.cuboid_rot(p, vec3(0.06, 0.06, 0.26), Quat::from_rotation_x(-a), Color::new(0.9, 0.2, 0.2, 1.));
            }
            ob.cylinder(c + vec3(0., 0., 1.25), 0.05, 0.05, 1.3, 6, Color::new(0.3, 0.3, 0.3, 1.));
            push_obj(&mut objects, &mut ob, &mut objects_mesh, format!("Ring_{k}"), c, vec3(0.1, 1.3, 1.3));
        }
    } else if !key.contains("blank") {
        let desert = key.contains("desert");
        let raceway = key.contains("race");
        for k in 0..if desert { 60 } else { 400 } {
            let r = rng.r(if raceway { 170.0 } else { 25.0 }, 600.0);
            let a = rng.r(0.0, std::f32::consts::TAU);
            let p = vec3(r * a.cos(), r * a.sin(), 0.0);
            if desert {
                let s = rng.r(1.0, 4.0);
                ob.sphere(p + vec3(0., 0., 0.2 * s), s, Color::new(0.65, 0.52, 0.38, 1.));
                push_obj(&mut objects, &mut ob, &mut objects_mesh, format!("Rock_{k}"), p, vec3(s, s, s));
            } else {
                let h = rng.r(4.0, 11.0);
                ob.cylinder(p - vec3(0., 0., h * 0.25), 0.2, 0.3, h * 0.5, 6, Color::new(0.40, 0.28, 0.16, 1.));
                let g = rng.r(0.75, 1.1);
                ob.cylinder(p - vec3(0., 0., h * 0.7), 0.0, h * 0.28, h * 0.7, 8, Color::new(0.14 * g, 0.36 * g, 0.14 * g, 1.));
                push_obj(&mut objects, &mut ob, &mut objects_mesh, format!("Tree_{k}"), p - vec3(0., 0., h / 2.), vec3(h * 0.28, h * 0.28, h / 2.));
            }
        }
        if raceway {
            // Grandstands along the south side (landable roofs).
            for k in 0..4 {
                let c = vec3(10.0 + k as f32 * 25.0, -62.0, -2.0);
                let half = vec3(9.0, 4.0, 2.0);
                ob.cuboid(c, half, Color::new(0.55, 0.58, 0.65, 1.));
                ob.cuboid(c - vec3(0., 0., 2.1), vec3(9.2, 4.2, 0.1), Color::new(0.85, 0.2, 0.2, 1.));
                push_obj(&mut objects, &mut ob, &mut objects_mesh, format!("Building_stand{k}"), c, half + vec3(0., 0., 0.1));
            }
        }
        if !desert && !raceway {
            for k in 0..10 {
                let p = vec3(rng.r(-120.0, 120.0), rng.r(40.0, 160.0) * if k % 2 == 0 { 1. } else { -1. }, 0.0);
                let h = rng.r(4.0, 7.0);
                let half = vec3(rng.r(4.0, 7.0), rng.r(4.0, 6.0), h / 2.0);
                ob.cuboid(p - vec3(0., 0., h / 2.), half, Color::new(0.88, 0.84, 0.76, 1.));
                ob.cuboid(p - vec3(0., 0., h + 0.6), half + vec3(0.3, 0.3, -half.z + 0.6), Color::new(0.6, 0.22, 0.18, 1.));
                push_obj(&mut objects, &mut ob, &mut objects_mesh, format!("House_{k}"), p - vec3(0., 0., h / 2.), half);
            }
        }
    }
    // Take-off pad at the origin.
    let mut pad = MeshBuf::default();
    pad.cylinder(vec3(0., 0., -0.01), 1.5, 1.5, 0.02, 24, Color::new(0.25, 0.25, 0.27, 1.));
    pad.cuboid(vec3(0., 0., -0.025), vec3(0.8, 0.12, 0.001), WHITE);
    pad.cuboid(vec3(0., 0.5, -0.025), vec3(0.12, 0.12, 0.001), WHITE);
    objects_mesh.push(ob.to_mesh(None));
    objects_mesh.push(pad.to_mesh(None));
    Map { name: name.to_string(), sky, fog, meshes: ground(size, tex), objects, objects_mesh }
}

// ---------------------------------------------------------------- material

pub const VS: &str = r#"#version 100
attribute vec3 position;
attribute vec2 texcoord;
attribute vec4 color0;
attribute vec4 normal;
varying lowp vec4 color;
varying vec2 uv;
varying vec3 nrm;
varying vec3 wpos;
uniform mat4 Model;
uniform mat4 Projection;
void main() {
    vec4 w = Model * vec4(position, 1.0);
    wpos = w.xyz;
    gl_Position = Projection * w;
    color = color0 / 255.0;
    uv = texcoord;
    nrm = (Model * vec4(normal.xyz, 0.0)).xyz;
}
"#;

pub const FS: &str = r#"#version 100
precision highp float;
varying lowp vec4 color;
varying vec2 uv;
varying vec3 nrm;
varying vec3 wpos;
uniform sampler2D Texture;
uniform vec3 LightDir;
uniform vec3 CamPos;
uniform vec3 CamFwd;
uniform vec4 SkyColor;
uniform float FogDist;
uniform float Mode;
uniform vec4 SegColor;
void main() {
    if (Mode > 2.5) {               // flat colour (shadows, overlays)
        gl_FragColor = SegColor;
        return;
    }
    if (Mode > 1.5) {               // segmentation
        gl_FragColor = vec4(SegColor.rgb, 1.0);
        return;
    }
    if (Mode > 0.5) {               // depth: millimetres, 16 bit split over R (hi) / G (lo)
        float d = dot(wpos - CamPos, CamFwd);
        float mm = clamp(floor(d * 1000.0 + 0.5), 0.0, 65535.0);
        float hi = floor(mm / 256.0);
        float lo = mm - hi * 256.0;
        gl_FragColor = vec4(hi / 255.0, lo / 255.0, 0.0, 1.0);
        return;
    }
    vec4 base = texture2D(Texture, uv) * color;
    float lit = 1.0;
    if (length(nrm) > 0.01) {
        float d = dot(normalize(nrm), normalize(LightDir));
        lit = 0.5 + 0.5 * max(d, 0.0) + 0.08 * min(d, 0.0);
    }
    vec3 c = base.rgb * lit;
    float f = clamp(length(wpos - CamPos) / FogDist, 0.0, 1.0);
    gl_FragColor = vec4(mix(c, SkyColor.rgb, f * f), base.a);
}
"#;

pub fn make_material() -> Material {
    load_material(
        ShaderSource::Glsl { vertex: VS, fragment: FS },
        MaterialParams {
            uniforms: vec![
                UniformDesc::new("LightDir", UniformType::Float3),
                UniformDesc::new("CamPos", UniformType::Float3),
                UniformDesc::new("CamFwd", UniformType::Float3),
                UniformDesc::new("SkyColor", UniformType::Float4),
                UniformDesc::new("FogDist", UniformType::Float1),
                UniformDesc::new("Mode", UniformType::Float1),
                UniformDesc::new("SegColor", UniformType::Float4),
            ],
            pipeline_params: PipelineParams {
                depth_test: Comparison::LessOrEqual,
                depth_write: true,
                color_blend: Some(BlendState::new(
                    Equation::Add,
                    BlendFactor::Value(BlendValue::SourceAlpha),
                    BlendFactor::OneMinusValue(BlendValue::SourceAlpha),
                )),
                ..Default::default()
            },
            ..Default::default()
        },
    )
    .expect("shader compile")
}

/// Deterministic segmentation colour for an object/vehicle index.
pub fn seg_color(k: u32) -> Color {
    let h = k.wrapping_mul(2654435761);
    Color::from_rgba((h >> 24) as u8 | 0x20, (h >> 16) as u8 | 0x20, (h >> 8) as u8 | 0x20, 255)
}
