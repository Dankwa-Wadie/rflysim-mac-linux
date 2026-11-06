//! Vehicle parameters, following the multicopter model used by RflySim's CopterSim
//! (Quan Quan, *Introduction to Multicopter Design and Control*).

/// Rotor layout. Rotor order matches PX4's mixer / control-allocation order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Frame {
    QuadX,
}

/// One rotor: position angle (rad, from body +x towards +y) and spin direction.
#[derive(Clone, Copy, Debug)]
pub struct Rotor {
    pub angle: f64,
    /// +1 for CCW (seen from above), -1 for CW.
    pub dir: f64,
}

impl Frame {
    pub fn rotors(self) -> &'static [Rotor] {
        use std::f64::consts::FRAC_PI_4 as Q;
        match self {
            // PX4 quad_x: 1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right CW.
            Frame::QuadX => &[
                Rotor { angle: Q, dir: 1.0 },
                Rotor { angle: -3.0 * Q, dir: 1.0 },
                Rotor { angle: -Q, dir: -1.0 },
                Rotor { angle: 3.0 * Q, dir: -1.0 },
            ],
        }
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
        let mut s = format!("# RflyTools vehicle parameters: {title}\n# Same 14 values as RflySim's ModelData.db modelInfo.\n");
        for (k, v) in PARAM_KEYS.iter().zip(self.model_info()) {
            s += &format!("{k} = {v:e}\n");
        }
        s
    }

    /// Parses [`Params::to_text`]. Missing keys keep the default F450 value; unknown keys are errors.
    pub fn from_text(text: &str) -> Result<Params, String> {
        let mut v = Params::default().model_info();
        for (n, line) in text.lines().enumerate() {
            let line = line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            let (k, val) = line.split_once('=').ok_or_else(|| format!("line {}: expected name = value", n + 1))?;
            let i = PARAM_KEYS.iter().position(|x| *x == k.trim()).ok_or_else(|| format!("line {}: unknown parameter {}", n + 1, k.trim()))?;
            v[i] = val.trim().parse().map_err(|_| format!("line {}: {} is not a number", n + 1, val.trim()))?;
        }
        let p = Params::from_model_info(&v).ok_or("bad parameters")?;
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
    }
}
