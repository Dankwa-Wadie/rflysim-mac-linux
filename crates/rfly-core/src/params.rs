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

    /// Throttle at which total thrust equals weight.
    pub fn hover_throttle(&self) -> f64 {
        let n = self.frame.rotors().len() as f64;
        let w = (self.mass * self.g / (n * self.ct)).sqrt();
        (w - self.wb) / self.cr
    }
}
