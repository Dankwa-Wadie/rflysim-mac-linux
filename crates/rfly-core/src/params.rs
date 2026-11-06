//! Vehicle parameters, following the multicopter model used by RflySim's CopterSim
//! (Quan Quan, *Introduction to Multicopter Design and Control*).

/// Rotor layout (CopterSim's frame types). For QuadX the rotor order matches PX4's quad_x mixer; for the
/// others PX4 gets the same geometry as control-allocation parameters ([`Frame::px4_params`]).
/// RflySim's tricopter needs a tail servo for yaw and is not modelled.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Frame {
    #[default]
    QuadX,
    QuadPlus,
    HexaX,
    OctoX,
    /// Three arms with coaxial rotor pairs (upper CCW, lower CW).
    Y6,
}

/// One rotor: position angle (rad, from body +x towards +y) and spin direction.
#[derive(Clone, Copy, Debug)]
pub struct Rotor {
    pub angle: f64,
    /// +1 for CCW (seen from above), -1 for CW.
    pub dir: f64,
}

const fn r(deg: f64, dir: f64) -> Rotor {
    Rotor { angle: deg * std::f64::consts::PI / 180.0, dir }
}

// PX4 quad_x: 1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right CW.
const QUAD_X: [Rotor; 4] = [r(45.0, 1.0), r(-135.0, 1.0), r(-45.0, -1.0), r(135.0, -1.0)];
// PX4 quad_+: 1 right CCW, 2 left CCW, 3 front CW, 4 rear CW.
const QUAD_PLUS: [Rotor; 4] = [r(90.0, 1.0), r(-90.0, 1.0), r(0.0, -1.0), r(180.0, -1.0)];
// Arms every 60° from 30°, alternating spin.
const HEXA_X: [Rotor; 6] = [r(30.0, 1.0), r(90.0, -1.0), r(150.0, 1.0), r(-150.0, -1.0), r(-90.0, 1.0), r(-30.0, -1.0)];
// Arms every 45° from 22.5°, alternating spin.
const OCTO_X: [Rotor; 8] = [
    r(22.5, 1.0), r(67.5, -1.0), r(112.5, 1.0), r(157.5, -1.0),
    r(-157.5, 1.0), r(-112.5, -1.0), r(-67.5, 1.0), r(-22.5, -1.0),
];
// Front-right, front-left, rear arms; each with an upper (CCW) and lower (CW) rotor.
const Y6: [Rotor; 6] = [r(60.0, 1.0), r(60.0, -1.0), r(-60.0, 1.0), r(-60.0, -1.0), r(180.0, 1.0), r(180.0, -1.0)];

impl Frame {
    pub const ALL: [Frame; 5] = [Frame::QuadX, Frame::QuadPlus, Frame::HexaX, Frame::OctoX, Frame::Y6];

    pub fn rotors(self) -> &'static [Rotor] {
        match self {
            Frame::QuadX => &QUAD_X,
            Frame::QuadPlus => &QUAD_PLUS,
            Frame::HexaX => &HEXA_X,
            Frame::OctoX => &OCTO_X,
            Frame::Y6 => &Y6,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Frame::QuadX => "quad_x",
            Frame::QuadPlus => "quad_plus",
            Frame::HexaX => "hexa_x",
            Frame::OctoX => "octo_x",
            Frame::Y6 => "y6",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Frame::QuadX => "Quadcopter (X)",
            Frame::QuadPlus => "Quadcopter (+)",
            Frame::HexaX => "Hexacopter (X)",
            Frame::OctoX => "Octocopter (X)",
            Frame::Y6 => "Y6 (coaxial)",
        }
    }

    pub fn parse(s: &str) -> Option<Frame> {
        let k = s.trim().to_ascii_lowercase().replace(['-', ' '], "_");
        Frame::ALL.into_iter().find(|f| f.name() == k).or(match k.as_str() {
            "quad" | "quadx" | "x" => Some(Frame::QuadX),
            "quadplus" | "quad+" | "plus" | "+" => Some(Frame::QuadPlus),
            "hexa" | "hexax" => Some(Frame::HexaX),
            "octo" | "octox" => Some(Frame::OctoX),
            _ => None,
        })
    }

    /// RflySim3D vehicle type for the 3D model (3 quad, 5 hexa, 8 octo).
    pub fn rflysim3d_type(self) -> i32 {
        match self {
            Frame::QuadX | Frame::QuadPlus => 3,
            Frame::HexaX | Frame::Y6 => 5,
            Frame::OctoX => 8,
        }
    }

    /// PX4 (v1.14+) parameters that give its control allocation this frame's geometry, plus the output mapping
    /// for every motor (SITL and HIL names; each setup ignores the other's): (name, value). `arm` in m.
    pub fn px4_params(self, arm: f64) -> Vec<(String, f64)> {
        let rotors = self.rotors();
        let mut v = vec![("CA_AIRFRAME".to_string(), 0.0), ("CA_ROTOR_COUNT".to_string(), rotors.len() as f64)];
        for (i, rt) in rotors.iter().enumerate() {
            let (sa, ca) = rt.angle.sin_cos();
            v.push((format!("CA_ROTOR{i}_PX"), (arm * ca * 1000.0).round() / 1000.0));
            v.push((format!("CA_ROTOR{i}_PY"), (arm * sa * 1000.0).round() / 1000.0));
            v.push((format!("CA_ROTOR{i}_PZ"), 0.0));
            // PX4: positive moment coefficient = counter-clockwise rotor.
            v.push((format!("CA_ROTOR{i}_KM"), 0.05 * rt.dir));
            // Output i → motor i+1 (101 = Motor 1): PWM_MAIN_FUNC in SITL, HIL_ACT_FUNC on a Pixhawk in HIL.
            for prefix in ["PWM_MAIN_FUNC", "HIL_ACT_FUNC"] {
                v.push((format!("{prefix}{}", i + 1), 101.0 + i as f64));
            }
        }
        // Unused outputs off (PX4 keeps parameters, e.g. from an earlier hexa run).
        for i in rotors.len()..8 {
            for prefix in ["PWM_MAIN_FUNC", "HIL_ACT_FUNC"] {
                v.push((format!("{prefix}{}", i + 1), 0.0));
            }
        }
        v
    }
}

/// Names of the 14 `modelInfo` values, in order.
pub const PARAM_KEYS: [&str; 14] = ["mass", "g", "jxx", "jyy", "jzz", "arm", "ct", "cm", "cr", "wb", "jm", "tm", "cd", "cmd"];

#[derive(Clone, Debug)]
pub struct Params {
    pub frame: Frame,
    /// Mass (kg).
    pub mass: f64,
    /// Gravity (m/s²).
    pub g: f64,
    /// Principal moments of inertia (kg·m²).
    pub jxx: f64,
    pub jyy: f64,
    pub jzz: f64,
    /// Motor-to-centre distance (m).
    pub arm: f64,
    /// Rotor thrust coefficient: T = ct·ω² (N/(rad/s)²).
    pub ct: f64,
    /// Rotor reaction-torque coefficient: M = cm·ω² (N·m/(rad/s)²).
    pub cm: f64,
    /// Motor steady-state speed: ω = cr·σ + wb (σ = throttle 0..1).
    pub cr: f64,
    pub wb: f64,
    /// Motor + propeller inertia (kg·m²). Kept for completeness; unused by the current model.
    pub jm: f64,
    /// Motor time constant (s).
    pub tm: f64,
    /// Translational drag coefficient: D = cd·|v|·v.
    pub cd: f64,
    /// Rotational air-drag coefficient: M = cmd·|ω|·ω.
    pub cmd: f64,
}

impl Default for Params {
    /// `uavparams.Default` row of RflySim's ModelData.db.
    fn default() -> Self {
        Self::from_model_info(&[
            1.4, 9.8, 0.0211, 0.0219, 0.0366, 0.225, 1.105e-05, 1.779e-07, 1148.0, -141.4,
            1.287e-04, 0.05, 0.073, 0.0055,
        ])
        .expect("default model info is valid")
    }
}

impl Params {
    /// Build from the 14-value `modelInfo` record in ModelData.db.
    /// Field order is inferred from CopterSim's UI labels (see docs/RE_FINDINGS.md).
    pub fn from_model_info(v: &[f64]) -> Option<Params> {
        if v.len() < 14 {
            return None;
        }
        Some(Params {
            frame: Frame::QuadX,
            mass: v[0],
            g: v[1],
            jxx: v[2],
            jyy: v[3],
            jzz: v[4],
            arm: v[5],
            ct: v[6],
            cm: v[7],
            cr: v[8],
            wb: v[9],
            jm: v[10],
            tm: v[11],
            cd: v[12],
            cmd: v[13],
        })
    }

    /// The 14 values in ModelData.db's `modelInfo` order (inverse of [`Params::from_model_info`]).
    pub fn model_info(&self) -> [f64; 14] {
        [self.mass, self.g, self.jxx, self.jyy, self.jzz, self.arm, self.ct, self.cm, self.cr, self.wb, self.jm, self.tm, self.cd, self.cmd]
    }

    /// Text form for `coptersim --params-file` (one `name = value` per line, `#` comments).
    pub fn to_text(&self, title: &str) -> String {
        let mut s = format!("# RflyTools vehicle parameters: {title}\n# Frame, then the same 14 values as RflySim's ModelData.db modelInfo.\nframe = {}\n", self.frame.name());
        for (k, v) in PARAM_KEYS.iter().zip(self.model_info()) {
            s += &format!("{k} = {v:e}\n");
        }
        s
    }

    /// Parses [`Params::to_text`]. Missing keys keep the default F450 value; unknown keys are errors.
    pub fn from_text(text: &str) -> Result<Params, String> {
        let mut v = Params::default().model_info();
        let mut frame = Frame::QuadX;
        for (n, line) in text.lines().enumerate() {
            let line = line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            let (k, val) = line.split_once('=').ok_or_else(|| format!("line {}: expected name = value", n + 1))?;
            if k.trim() == "frame" {
                frame = Frame::parse(val).ok_or_else(|| format!("line {}: unknown frame {} (quad_x, quad_plus, hexa_x, octo_x, y6)", n + 1, val.trim()))?;
                continue;
            }
            let i = PARAM_KEYS.iter().position(|x| *x == k.trim()).ok_or_else(|| format!("line {}: unknown parameter {}", n + 1, k.trim()))?;
            v[i] = val.trim().parse().map_err(|_| format!("line {}: {} is not a number", n + 1, val.trim()))?;
        }
        let mut p = Params::from_model_info(&v).ok_or("bad parameters")?;
        p.frame = frame;
        if p.mass <= 0.0 || p.ct <= 0.0 || p.cr <= 0.0 || p.arm <= 0.0 {
            return Err("mass, ct, cr and arm must be positive".into());
        }
        Ok(p)
    }

    /// Throttle at which total thrust equals weight.
    pub fn hover_throttle(&self) -> f64 {
        let n = self.frame.rotors().len() as f64;
        let w = (self.mass * self.g / (n * self.ct)).sqrt();
        (w - self.wb) / self.cr
    }
}

#[cfg(test)]
mod text_tests {
    use super::*;

    #[test]
    fn params_round_trip_through_text() {
        let mut p = Params::default();
        p.mass = 2.25;
        p.cr = 987.5;
        let q = Params::from_text(&p.to_text("test")).unwrap();
        assert_eq!(q.model_info(), p.model_info());
        assert_eq!(Params::from_text("mass = 3\n").unwrap().mass, 3.0);
        assert!(Params::from_text("weight = 3").is_err());
        assert!(Params::from_text("mass = -1").is_err());
        assert_eq!(Params::from_text("frame = hexa_x\n").unwrap().frame, Frame::HexaX);
        let mut h = Params::default();
        h.frame = Frame::OctoX;
        assert_eq!(Params::from_text(&h.to_text("o")).unwrap().frame, Frame::OctoX);
    }
}

#[cfg(test)]
mod frame_tests {
    use super::*;

    #[test]
    fn frames_are_balanced_and_px4_params_match_the_geometry() {
        for f in Frame::ALL {
            let rs = f.rotors();
            // Equal thrust gives no roll/pitch moment and no net yaw torque.
            let (mx, my, mz) = rs.iter().fold((0.0, 0.0, 0.0), |a, r| (a.0 + r.angle.cos(), a.1 + r.angle.sin(), a.2 + r.dir));
            assert!(mx.abs() < 1e-9 && my.abs() < 1e-9 && mz.abs() < 1e-9, "{f:?} unbalanced");
            let p = f.px4_params(0.25);
            let count = p.iter().find(|(k, _)| k == "CA_ROTOR_COUNT").unwrap().1;
            assert_eq!(count as usize, rs.len());
            let px0 = p.iter().find(|(k, _)| k == "CA_ROTOR0_PX").unwrap().1;
            assert!((px0 - 0.25 * rs[0].angle.cos()).abs() < 1e-3);
            assert_eq!(Frame::parse(f.name()), Some(f));
        }
        assert_eq!(Frame::QuadX.rotors().len(), 4);
        assert_eq!(Frame::OctoX.rotors().len(), 8);
    }
}
