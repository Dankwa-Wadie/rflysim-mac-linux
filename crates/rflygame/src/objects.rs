//! Course objects: race gates (square / ring), checkpoint beacons, pads, and the
//! on-screen target indicator.

use macroquad::prelude::*;
use rflysim3d::render::CamPose;
use rflysim3d::scene::MeshBuf;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    /// Square opening of this size (m), with legs to the ground.
    Square(f32),
    /// Ring of this radius (m). `drawn` = false when the map already has it (VisionRing).
    Ring(f32, bool),
    /// Fly within this radius of the point (no frame).
    Sphere(f32),
}

#[derive(Clone, Debug)]
pub struct Gate {
    /// Centre of the opening (NED).
    pub c: Vec3,
    /// Heading you must fly through it (rad, 0 = north).
    pub yaw: f32,
    pub shape: Shape,
}

pub const BAR: f32 = 0.16;

impl Gate {
    pub fn new(x: f32, y: f32, alt: f32, yaw_deg: f32, shape: Shape) -> Self {
        Gate { c: vec3(x, y, -alt), yaw: yaw_deg.to_radians(), shape }
    }

    pub fn to_local(&self, p: Vec3) -> Vec3 {
        Quat::from_rotation_z(-self.yaw) * (p - self.c)
    }

    /// True if the segment a→b passes through the opening in the right direction.
    pub fn crossed(&self, a: Vec3, b: Vec3) -> bool {
        if let Shape::Sphere(r) = self.shape {
            // Closest approach of the segment.
            let ab = b - a;
            let t = if ab.length_squared() > 0.0 { ((self.c - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0) } else { 0.0 };
            return (a + ab * t).distance(self.c) < r;
        }
        let (la, lb) = (self.to_local(a), self.to_local(b));
        if !(la.x < 0.0 && lb.x >= 0.0) {
            return false;
        }
        let t = -la.x / (lb.x - la.x);
        let p = la + (lb - la) * t;
        match self.shape {
            Shape::Square(s) => p.y.abs() < s / 2.0 && p.z.abs() < s / 2.0,
            Shape::Ring(r, _) => (p.y * p.y + p.z * p.z).sqrt() < r - 0.05,
            Shape::Sphere(_) => unreachable!(),
        }
    }

    /// Closest point (world NED) on the gate's frame or legs to `p`, and the bar radius.
    pub fn closest_on_frame(&self, p: Vec3) -> Option<(Vec3, f32)> {
        let l = self.to_local(p);
        let q = Quat::from_rotation_z(self.yaw);
        let seg = |a: Vec3, b: Vec3| {
            let ab = b - a;
            let t = ((l - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
            a + ab * t
        };
        let best = match self.shape {
            Shape::Sphere(_) => return None,
            Shape::Ring(r, _) => {
                let yz = vec2(l.y, l.z);
                let d = if yz.length() > 1e-4 { yz.normalize() * r } else { vec2(r, 0.0) };
                vec3(0.0, d.x, d.y)
            }
            Shape::Square(s) => {
                let h = s / 2.0 + BAR / 2.0;
                let ground = -self.c.z; // local z of the ground (z down)
                let segs = [
                    (vec3(0., -h, -h), vec3(0., h, -h)),
                    (vec3(0., -h, h), vec3(0., h, h)),
                    (vec3(0., -h, -h), vec3(0., -h, h)),
                    (vec3(0., h, -h), vec3(0., h, h)),
                    (vec3(0., -h, h), vec3(0., -h, ground)),
                    (vec3(0., h, h), vec3(0., h, ground)),
                ];
                segs.iter().map(|(a, b)| seg(*a, *b)).min_by(|a, b| a.distance(l).total_cmp(&b.distance(l))).unwrap()
            }
        };
        Some((self.c + q * best, if matches!(self.shape, Shape::Ring(..)) { 0.07 } else { BAR / 2.0 }))
    }

    /// True if a sphere of radius `rad` at `p` touches the gate's frame.
    pub fn hits_frame(&self, p: Vec3, rad: f32) -> bool {
        let l = self.to_local(p);
        match self.shape {
            Shape::Sphere(_) => false,
            Shape::Ring(r, _) => {
                let d = ((l.y * l.y + l.z * l.z).sqrt() - r).hypot(l.x);
                d < rad + 0.06
            }
            Shape::Square(s) => {
                if l.x.abs() > BAR / 2.0 + rad {
                    return false;
                }
                let h = s / 2.0;
                let (ay, az) = (l.y.abs(), l.z.abs());
                let in_bar_y = ay > h - rad && ay < h + BAR + rad && az < h + BAR + rad;
                let in_bar_z = az > h - rad && az < h + BAR + rad && ay < h + BAR + rad;
                // Legs below the frame down to the ground.
                let leg = ay > h - rad && ay < h + BAR + rad && l.z > h && p.z < 0.0;
                in_bar_y || in_bar_z || leg
            }
        }
    }

    /// Builds the gate mesh in world NED coordinates with the given colour.
    pub fn mesh(&self, col: Color) -> Mesh {
        let mut b = MeshBuf::default();
        let q = Quat::from_rotation_z(self.yaw);
        match self.shape {
            Shape::Square(s) => {
                let h = s / 2.0 + BAR / 2.0;
                let t = BAR / 2.0;
                b.cuboid_rot(self.c + q * vec3(0., 0., -h), vec3(t, h + t, t), q, col);
                b.cuboid_rot(self.c + q * vec3(0., 0., h), vec3(t, h + t, t), q, col);
                b.cuboid_rot(self.c + q * vec3(0., -h, 0.), vec3(t, t, h + t), q, col);
                b.cuboid_rot(self.c + q * vec3(0., h, 0.), vec3(t, t, h + t), q, col);
                // Legs to the ground.
                let bottom = self.c.z + h;
                if bottom < 0.0 {
                    let len = -bottom;
                    for side in [-1.0, 1.0] {
                        let base = self.c + q * vec3(0., side * h, 0.);
                        b.cuboid_rot(vec3(base.x, base.y, bottom + len / 2.0), vec3(0.05, 0.05, len / 2.0), q, Color::new(0.3, 0.3, 0.32, 1.0));
                    }
                }
            }
            Shape::Ring(r, _) => {
                let segs = 24;
                for k in 0..segs {
                    let a = k as f32 / segs as f32 * std::f32::consts::TAU;
                    let p = self.c + q * vec3(0., a.cos() * r, a.sin() * r);
                    b.cuboid_rot(p, vec3(0.07, 0.07, r * 0.14), q * Quat::from_rotation_x(-a), col);
                }
            }
            Shape::Sphere(r) => {
                b.sphere(self.c, r * 0.25, col);
                b.cylinder(vec3(self.c.x, self.c.y, self.c.z / 2.0), 0.05, 0.05, -self.c.z, 6, Color::new(col.r, col.g, col.b, 0.5));
            }
        }
        b.to_mesh(None)
    }
}

/// Coloured gate states.
pub const NEXT: Color = Color::new(0.15, 0.95, 0.55, 1.0);
pub const LATER: Color = Color::new(1.0, 0.45, 0.12, 1.0);
pub const DONE: Color = Color::new(0.45, 0.45, 0.5, 1.0);

/// Landing pad mesh (ground or roof), radius r at NED height z.
pub fn pad_mesh(c: Vec3, r: f32, col: Color) -> Mesh {
    let mut b = MeshBuf::default();
    b.cylinder(c - vec3(0., 0., 0.02), r, r, 0.04, 28, col);
    b.cylinder(c - vec3(0., 0., 0.045), r * 0.7, r * 0.7, 0.01, 28, Color::new(0.95, 0.95, 0.95, 1.0));
    b.cylinder(c - vec3(0., 0., 0.05), r * 0.55, r * 0.55, 0.01, 28, col);
    b.cuboid(c - vec3(0., 0., 0.06), vec3(r * 0.35, 0.08, 0.005), WHITE);
    b.cuboid(c - vec3(0., 0., 0.06), vec3(0.08, r * 0.35, 0.005), WHITE);
    b.to_mesh(None)
}

/// Translucent vertical beacon (target marker), radius r around point c.
pub fn beacon_mesh(c: Vec3, r: f32, col: Color) -> Mesh {
    let mut b = MeshBuf::default();
    let h = 40.0;
    b.cylinder(vec3(c.x, c.y, -h / 2.0), r * 0.08, r * 0.08, h, 8, Color::new(col.r, col.g, col.b, 0.35));
    b.sphere(c, r, Color::new(col.r, col.g, col.b, 0.28));
    b.to_mesh(None)
}

/// A box (parcel) mesh centred at the origin.
pub fn parcel_mesh() -> Mesh {
    let mut b = MeshBuf::default();
    b.cuboid(Vec3::ZERO, vec3(0.15, 0.15, 0.12), Color::new(0.75, 0.55, 0.3, 1.0));
    b.cuboid(Vec3::ZERO, vec3(0.152, 0.03, 0.122), Color::new(0.9, 0.85, 0.6, 1.0));
    b.to_mesh(None)
}

/// Draws an on-screen marker at the target, or an edge arrow when it is off-screen.
pub fn indicator(cam: &CamPose, target: Vec3, label: &str, col: Color) {
    let (w, h) = (screen_width(), screen_height());
    let d = cam.pos.distance(target);
    let txt = format!("{label} {d:.0} m");
    if let Some(s) = cam.project(target, w, h) {
        if s.x > 30.0 && s.x < w - 30.0 && s.y > 30.0 && s.y < h - 30.0 {
            draw_circle_lines(s.x, s.y, 16.0, 2.5, col);
            draw_text(&txt, s.x + 20.0, s.y - 10.0, 22.0, col);
            return;
        }
    }
    // Off-screen: arrow at the edge in the target's direction (camera frame).
    let rel = target - cam.pos;
    let right = cam.fwd.cross(cam.up).normalize_or_zero();
    let dir = vec2(rel.dot(right), -rel.dot(cam.up));
    let dir = if dir.length() < 1e-3 { vec2(0.0, 1.0) } else { dir.normalize() };
    let c = vec2(w / 2.0, h / 2.0);
    let edge = c + dir * (h / 2.0 - 50.0).min(w / 2.0 - 50.0);
    let a = edge + dir * 18.0;
    let perp = vec2(-dir.y, dir.x) * 10.0;
    draw_triangle(a, edge - dir * 4.0 + perp, edge - dir * 4.0 - perp, col);
    draw_text(&txt, edge.x - 40.0, edge.y - 18.0, 20.0, col);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn square_gate_crossing_and_direction() {
        let g = Gate::new(10.0, 0.0, 3.0, 0.0, Shape::Square(2.0));
        assert!(g.crossed(vec3(9.0, 0.2, -3.1), vec3(11.0, 0.2, -3.0)));
        assert!(!g.crossed(vec3(11.0, 0.0, -3.0), vec3(9.0, 0.0, -3.0)), "wrong way");
        assert!(!g.crossed(vec3(9.0, 1.5, -3.0), vec3(11.0, 1.5, -3.0)), "outside opening");
        let g90 = Gate::new(0.0, 10.0, 3.0, 90.0, Shape::Square(2.0));
        assert!(g90.crossed(vec3(0.0, 9.0, -3.0), vec3(0.0, 11.0, -3.0)));
    }

    #[test]
    fn frame_collision() {
        let g = Gate::new(0.0, 0.0, 3.0, 0.0, Shape::Square(2.0));
        assert!(g.hits_frame(vec3(0.0, 1.05, -3.0), 0.1), "side bar");
        assert!(!g.hits_frame(vec3(0.0, 0.0, -3.0), 0.1), "centre is free");
        let r = Gate::new(0.0, 0.0, 3.0, 0.0, Shape::Ring(1.2, true));
        assert!(r.hits_frame(vec3(0.0, 1.2, -3.0), 0.1));
        assert!(!r.hits_frame(vec3(0.0, 0.0, -3.0), 0.1));
    }

    #[test]
    fn sphere_checkpoint() {
        let g = Gate::new(10.0, 0.0, 3.0, 0.0, Shape::Sphere(3.0));
        assert!(g.crossed(vec3(0.0, 1.0, -3.0), vec3(20.0, 1.0, -3.0)));
        assert!(!g.crossed(vec3(0.0, 8.0, -3.0), vec3(20.0, 8.0, -3.0)));
    }
}
