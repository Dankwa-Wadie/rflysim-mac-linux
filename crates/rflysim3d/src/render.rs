//! Shared world renderer (rflysim3d viewer and rflygame).

use crate::scene::*;
use macroquad::prelude::*;

#[derive(Clone, Copy, Debug)]
pub struct CamPose {
    pub pos: Vec3,
    pub fwd: Vec3,
    pub up: Vec3,
    /// Horizontal field of view (rad).
    pub fov_h: f32,
    pub aspect: f32,
}

impl CamPose {
    pub fn fovy(&self) -> f32 {
        2.0 * ((self.fov_h / 2.0).tan() / self.aspect).atan()
    }
    /// Look from `pos` towards `target` (NED), keeping "up" as world up.
    pub fn look_at(pos: Vec3, target: Vec3, fov_h: f32, aspect: f32) -> Self {
        let fwd = (target - pos).normalize_or(Vec3::X);
        let up = if fwd.z.abs() > 0.99 { Vec3::X } else { vec3(0., 0., -1.) };
        let up = (up - fwd * up.dot(fwd)).normalize();
        CamPose { pos, fwd, up, fov_h, aspect }
    }
    /// Camera rigidly attached with orientation `q` (body FRD: looks along +X).
    pub fn from_quat(pos: Vec3, q: Quat, fov_h: f32, aspect: f32) -> Self {
        CamPose { pos, fwd: q * Vec3::X, up: q * vec3(0., 0., -1.), fov_h, aspect }
    }
    /// World NED point → screen pixels (None if behind the camera).
    pub fn project(&self, p: Vec3, w: f32, h: f32) -> Option<Vec2> {
        let view = Mat4::look_at_rh(v_ned_to_gl(self.pos), v_ned_to_gl(self.pos + self.fwd), v_ned_to_gl(self.up));
        let vp = Mat4::perspective_rh_gl(self.fovy(), self.aspect, 0.05, 4000.0) * view;
        let c = vp * v_ned_to_gl(p).extend(1.0);
        (c.w > 0.0).then(|| vec2((c.x / c.w + 1.0) * 0.5 * w, (1.0 - c.y / c.w) * 0.5 * h))
    }
}

/// One vehicle to draw.
pub struct DrawVehicle<'a> {
    pub model: &'a VehicleModel,
    pub kind: ModelKind,
    pub pos: Vec3,
    pub q: Quat,
    pub scale: Vec3,
    pub prop_angle: f32,
    pub spinning: bool,
    pub seg_id: u32,
}

/// Extra world-space meshes (NED) such as race gates, pads, parcels.
pub struct DrawExtra<'a> {
    pub mesh: &'a Mesh,
    /// NED transform.
    pub xf: Mat4,
}

pub struct Renderer {
    pub mat: Material,
    pub shadow: Mesh,
}

impl Default for Renderer {
    fn default() -> Self {
        Self::new()
    }
}

impl Renderer {
    pub fn new() -> Self {
        let mut s = MeshBuf::default();
        s.cylinder(Vec3::ZERO, 1.0, 1.0, 1.0, 24, WHITE);
        Renderer { mat: make_material(), shadow: s.to_mesh(None) }
    }

    /// Renders the world. mode: 0 lit, 1 depth (mm in R/G), 2 segmentation.
    pub fn render(
        &self,
        map: &Map,
        vehicles: &[DrawVehicle],
        extras: &[DrawExtra],
        p: &CamPose,
        rt: Option<RenderTarget>,
        mode: f32,
    ) {
        let cam = Camera3D {
            position: v_ned_to_gl(p.pos),
            target: v_ned_to_gl(p.pos + p.fwd),
            up: v_ned_to_gl(p.up),
            fovy: p.fovy(),
            aspect: Some(p.aspect),
            projection: Projection::Perspective,
            render_target: rt,
            viewport: None,
            z_near: 0.05,
            z_far: 4000.0,
        };
        set_camera(&cam);
        clear_background(match mode as i32 {
            0 => map.sky,
            1 => Color::new(1.0, 1.0, 0.0, 1.0), // 65535 mm = no return
            _ => BLACK,
        });
        let m = &self.mat;
        gl_use_material(m);
        m.set_uniform("LightDir", vec3(0.45, 0.85, 0.3));
        m.set_uniform("CamPos", v_ned_to_gl(p.pos));
        m.set_uniform("CamFwd", v_ned_to_gl(p.fwd));
        m.set_uniform("SkyColor", map.sky.to_vec());
        m.set_uniform("FogDist", map.fog);
        m.set_uniform("Mode", mode);
        let gl = unsafe { get_internal_gl() }.quad_gl;
        let root = ned_to_gl();
        let seg = |c: Color| {
            if mode > 1.5 {
                m.set_uniform("SegColor", c.to_vec());
            }
        };

        seg(BLACK);
        gl.push_model_matrix(root);
        for mesh in &map.meshes {
            draw_mesh(mesh);
        }
        seg(Color::from_rgba(128, 128, 128, 255));
        for mesh in &map.objects_mesh {
            draw_mesh(mesh);
        }
        gl.pop_model_matrix();
        for e in extras {
            gl.push_model_matrix(root * e.xf);
            draw_mesh(e.mesh);
            gl.pop_model_matrix();
        }

        for v in vehicles {
            seg(seg_color(v.seg_id));
            let mdl = root * Mat4::from_scale_rotation_translation(v.scale, v.q, v.pos);
            gl.push_model_matrix(mdl);
            draw_mesh(&v.model.body);
            let fw = v.kind == ModelKind::FixedWing;
            for (k, (hub, spin)) in v.model.rotors.iter().enumerate() {
                let ang = v.prop_angle * spin + k as f32;
                let r = if fw { Quat::from_rotation_x(ang) * Quat::from_rotation_y(std::f32::consts::FRAC_PI_2) } else { Quat::from_rotation_z(ang) };
                gl.push_model_matrix(Mat4::from_rotation_translation(r, *hub));
                draw_mesh(&v.model.prop);
                gl.pop_model_matrix();
            }
            gl.pop_model_matrix();
        }
        if mode < 0.5 {
            // Blob shadows + translucent rotor disks.
            m.set_uniform("Mode", 3.0f32);
            m.set_uniform("SegColor", Color::new(0.0, 0.0, 0.0, 0.28).to_vec());
            for v in vehicles {
                let h = (-v.pos.z).max(0.0);
                let r = (v.model.half * v.scale).max_element() * (1.0 + h * 0.02);
                let s = Mat4::from_scale_rotation_translation(vec3(r, r, 0.001), Quat::IDENTITY, vec3(v.pos.x, v.pos.y, -0.02));
                gl.push_model_matrix(root * s);
                draw_mesh(&self.shadow);
                gl.pop_model_matrix();
            }
            m.set_uniform("Mode", 0.0f32);
            for v in vehicles.iter().filter(|v| v.spinning && v.kind != ModelKind::FixedWing) {
                let mdl = root * Mat4::from_scale_rotation_translation(v.scale, v.q, v.pos);
                for (hub, _) in &v.model.rotors {
                    gl.push_model_matrix(mdl * Mat4::from_translation(*hub));
                    draw_mesh(&v.model.disk);
                    gl.pop_model_matrix();
                }
            }
        }
        gl_use_default_material();
    }
}
