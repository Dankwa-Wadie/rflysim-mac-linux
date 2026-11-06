//! Multicopter design estimates from catalog parts, like CopterSim's configurator (eCalc-style):
//! hover throttle/current/rpm/time, full-throttle thrust and current with limit checks, and the simulator
//! parameters (`rfly_core::Params`) for exactly that build.
//!
//! Model: propeller thrust T = Ct·ω², torque Q = Cm·ω² (Ct/Cm from the propeller size, RflySim's correction
//! factors and air density); brushed-equivalent motor (Kv, winding resistance Rm, no-load current I0);
//! ESC resistance; battery sag (internal resistance). ESC output voltage = throttle × battery voltage.

use rfly_core::{Frame, Params};
use rfly_db::{Battery, Esc, Motor, Propeller};

/// What CopterSim's configurator asks for.
#[derive(Clone, Debug)]
pub struct Design {
    pub name: String,
    pub frame: Frame,
    /// Take-off weight (kg), as typed into CopterSim's "Total Weight".
    pub mass_kg: f64,
    /// Motor-to-motor diagonal (mm).
    pub wheelbase_mm: f64,
    /// Field altitude (m above sea level) and air temperature (°C) for air density.
    pub altitude_m: f64,
    pub temp_c: f64,
    pub motor: Motor,
    pub prop: Propeller,
    pub esc: Esc,
    pub battery: Battery,
}

#[derive(Clone, Debug)]
pub struct Estimate {
    pub air_density: f64,
    pub hover_throttle: f64,
    pub hover_rpm: f64,
    /// Per motor (A).
    pub hover_current: f64,
    pub hover_power_w: f64,
    pub hover_minutes: f64,
    /// Grams of thrust per watt at hover.
    pub hover_efficiency: f64,
    /// All motors at full throttle (kg-force) and thrust-to-weight ratio.
    pub max_thrust_kg: f64,
    pub thrust_to_weight: f64,
    pub max_rpm: f64,
    /// Per motor at full throttle (A).
    pub max_current: f64,
    pub battery_max_current: f64,
    pub warnings: Vec<String>,
    /// Simulator parameters for this build.
    pub params: Params,
}

const G: f64 = 9.8;
/// Shaft torque used for the *electrical* estimate, as a share of the yaw-torque coefficient Cm.
/// RflySim's stored hover estimate for its F450 (3.74 A per motor at 5236 rpm, KV960) implies a shaft
/// torque of ~62 % of the Cm its simulator uses (1.78e-7): its eCalc-style estimate uses propeller power
/// data, not Cm. Calibrated on that reference so the numbers match Windows CopterSim's.
const SHAFT_TORQUE_FACTOR: f64 = 0.62;
const RPM: f64 = 60.0 / std::f64::consts::TAU;

/// ISA pressure at altitude with the given temperature (ideal gas).
pub fn air_density(alt_m: f64, temp_c: f64) -> f64 {
    let p = 101_325.0 * (1.0 - 2.255_77e-5 * alt_m).powf(5.255_88);
    p / (287.05 * (temp_c + 273.15))
}

struct Model {
    n: f64,
    ct: f64,
    cm: f64,
    kv_rad: f64, // rad/s per volt
    kt: f64,     // N·m per amp
    rm: f64,
    i0: f64,
    resc: f64,
    rbat: f64,
    vbat: f64,
}

impl Model {
    fn new(d: &Design) -> (Model, f64) {
        let rho = air_density(d.altitude_m, d.temp_c);
        let dia = d.prop.diameter_in * 0.0254;
        let pd = (d.prop.pitch_in / d.prop.diameter_in.max(0.1)).clamp(0.2, 1.2);
        // Same propeller law as `flight::from_components` (matches RflySim's F450 Ct within 10 %).
        let c_t = 0.09 * (0.7 + 0.6 * pd) * d.prop.ct_correct;
        let ct = c_t * rho * dia.powi(4) / (4.0 * std::f64::consts::PI.powi(2));
        let cm = ct * dia * 0.063 * (pd / 0.45).sqrt() * d.prop.cm_correct;
        let kv_rad = d.motor.kv / RPM;
        let cells = d.battery.cells.max(1.0);
        let m = Model {
            n: d.frame.rotors().len() as f64,
            ct,
            cm,
            kv_rad,
            kt: 1.0 / kv_rad,
            rm: if d.motor.resistance_mohm > 0.0 { d.motor.resistance_mohm / 1000.0 } else { 0.1 },
            // No-load current is quoted at U0; it grows roughly with voltage (speed).
            i0: if d.motor.i0 > 0.0 { d.motor.i0 * (cells * 3.7 / d.motor.u0.max(1.0)).sqrt() } else { 0.5 },
            resc: if d.esc.resistance_mohm > 0.0 { d.esc.resistance_mohm / 1000.0 } else { 0.006 },
            rbat: if d.battery.resistance_mohm > 0.0 { d.battery.resistance_mohm / 1000.0 } else { 0.004 * cells * (5000.0 / d.battery.capacity_mah.max(500.0)).sqrt() },
            vbat: d.battery.volts.max(cells * 3.7),
        };
        (m, rho)
    }

    /// Motor current (A) at speed ω (rad/s), loaded by the propeller.
    fn current(&self, w: f64) -> f64 {
        SHAFT_TORQUE_FACTOR * self.cm * w * w / self.kt + self.i0
    }

    /// Throttle (0..1) needed for speed ω, with battery sag from all motors.
    fn throttle_for(&self, w: f64) -> f64 {
        let i = self.current(w);
        let v_motor = w / self.kv_rad + i * (self.rm + self.resc);
        let v_bat = self.vbat - self.n * i * self.rbat;
        v_motor / v_bat.max(0.1)
    }

    /// Speed (rad/s) reached at throttle σ (bisection on the throttle needed).
    fn speed_at(&self, sigma: f64) -> f64 {
        let (mut lo, mut hi) = (0.0, self.vbat * self.kv_rad * 1.2);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if self.throttle_for(mid) < sigma {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }
}

pub fn estimate(d: &Design) -> Estimate {
    let (m, rho) = Model::new(d);
    let weight = d.mass_kg * G;
    let w_hover = (weight / m.n / m.ct).sqrt();
    let i_hover = m.current(w_hover);
    let thr_hover = m.throttle_for(w_hover);
    let v_bat_h = m.vbat - m.n * i_hover * m.rbat;
    let p_hover = m.n * i_hover * v_bat_h;
    // 85 % usable capacity (LiPo practice, as eCalc's default).
    let minutes = d.battery.capacity_mah / 1000.0 * 0.85 / (m.n * i_hover).max(0.01) * 60.0;
    let w_max = m.speed_at(1.0);
    let i_max = m.current(w_max);
    let thrust_max = m.n * m.ct * w_max * w_max;
    let bat_limit = d.battery.c_cont * d.battery.capacity_mah / 1000.0;

    let mut warnings = vec![];
    if thr_hover > 0.8 {
        warnings.push(format!("hover needs {:.0} % throttle: too heavy for these parts", thr_hover * 100.0));
    }
    if thrust_max / weight < 1.6 {
        warnings.push(format!("thrust/weight {:.1}: below 1.6, hard to control", thrust_max / weight));
    }
    if i_max > d.esc.max_current {
        warnings.push(format!("full-throttle current {:.1} A exceeds the ESC's {:.0} A", i_max, d.esc.max_current));
    }
    if d.motor.imax > 0.0 && i_max > d.motor.imax {
        warnings.push(format!("full-throttle current {:.1} A exceeds the motor's {:.0} A", i_max, d.motor.imax));
    }
    if bat_limit > 0.0 && m.n * i_max > bat_limit {
        warnings.push(format!("total current {:.0} A exceeds the battery's continuous {:.0} A", m.n * i_max, bat_limit));
    }

    // Simulator motor model ω = Cr·σ + ωb: straight-line fit of the real speed curve over 30..100 % throttle.
    let pts: Vec<(f64, f64)> = (6..=20).map(|k| k as f64 * 0.05).map(|s| (s, m.speed_at(s))).collect();
    let (sx, sy) = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0, a.1 + p.1));
    let k = pts.len() as f64;
    let (mx, my) = (sx / k, sy / k);
    let cr = pts.iter().map(|p| (p.0 - mx) * (p.1 - my)).sum::<f64>() / pts.iter().map(|p| (p.0 - mx).powi(2)).sum::<f64>();
    let wb = my - cr * mx;

    let mut params = crate::flight::from_components(&d.name, d.mass_kg, d.wheelbase_mm, d.motor.kv, d.prop.diameter_in, d.prop.pitch_in, d.prop.ct_correct, d.prop.cm_correct, d.battery.cells, d.battery.capacity_mah).params;
    params.frame = d.frame;
    params.mass = d.mass_kg;
    params.ct = m.ct;
    params.cm = m.cm;
    params.cr = cr;
    params.wb = wb;

    Estimate {
        air_density: rho,
        hover_throttle: thr_hover,
        hover_rpm: w_hover * RPM,
        hover_current: i_hover,
        hover_power_w: p_hover,
        hover_minutes: minutes,
        hover_efficiency: d.mass_kg * 1000.0 / p_hover.max(1.0),
        max_thrust_kg: thrust_max / G,
        thrust_to_weight: thrust_max / weight,
        max_rpm: w_max * RPM,
        max_current: i_max,
        battery_max_current: bat_limit,
        warnings,
        params,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// RflySim's DJI F450 (2312 KV960, CFP 10x4.7, E310, 3S 5500 mAh, 1.5 kg). RflySim stores its own hover
    /// estimate in ModelData.db: 18 min, 60.9 % throttle, 3.74 A per motor, 5236 rpm.
    #[test]
    fn f450_matches_rflysims_hover_estimate() {
        let Some(path) = crate::garage::db_path() else {
            eprintln!("skipped: ModelData.db not found");
            return;
        };
        let db = rfly_db::ModelDb::open(path).unwrap();
        let motor = db.motors().unwrap().into_iter().find(|m| m.brand == "DJI" && m.model == "2312 KV960").unwrap();
        let prop = db.propellers().unwrap().into_iter().find(|p| p.brand == "DJI" && p.model == "CFP 10x4.7").unwrap();
        let esc = db.escs().unwrap().into_iter().find(|e| e.brand == "DJI" && e.model == "E310").unwrap();
        let battery = db.batteries().unwrap().into_iter().find(|b| b.model == "LiPo 3S-11.1V-25C-5500mAh").unwrap();
        let d = Design { name: "F450".into(), frame: Frame::QuadX, mass_kg: 1.5, wheelbase_mm: 450.0, altitude_m: 50.0, temp_c: 25.0, motor, prop, esc, battery };
        let e = estimate(&d);
        let h = db.hover_info("default").unwrap().unwrap();
        println!("ours: {:.1} min, {:.1} %, {:.2} A, {:.0} rpm, TWR {:.2}; RflySim: {:?}", e.hover_minutes, e.hover_throttle * 100.0, e.hover_current, e.hover_rpm, e.thrust_to_weight, h);
        assert!((e.hover_throttle * 100.0 - h[1]).abs() < 6.0, "throttle {:.1} vs {}", e.hover_throttle * 100.0, h[1]);
        assert!((e.hover_rpm / h[3] - 1.0).abs() < 0.08, "rpm {:.0} vs {}", e.hover_rpm, h[3]);
        assert!((e.hover_current / h[2] - 1.0).abs() < 0.25, "current {:.2} vs {}", e.hover_current, h[2]);
        assert!((e.hover_minutes / h[0] - 1.0).abs() < 0.25, "minutes {:.1} vs {}", e.hover_minutes, h[0]);
        assert!(e.warnings.is_empty(), "{:?}", e.warnings);
        // The fitted motor line must reproduce the hover speed at the hover throttle.
        let w = e.params.cr * e.hover_throttle + e.params.wb;
        assert!((w * RPM / e.hover_rpm - 1.0).abs() < 0.06, "motor line {:.0} rpm vs {:.0}", w * RPM, e.hover_rpm);
    }

    #[test]
    fn heavier_drones_need_more_throttle_and_fly_less() {
        let d = |kg: f64| Design {
            name: "x".into(),
            frame: Frame::QuadX,
            mass_kg: kg,
            wheelbase_mm: 450.0,
            altitude_m: 0.0,
            temp_c: 15.0,
            motor: Motor { kv: 920.0, resistance_mohm: 120.0, i0: 0.4, u0: 10.0, imax: 20.0, ..Default::default() },
            prop: Propeller { diameter_in: 10.0, pitch_in: 4.5, ct_correct: 1.0, cm_correct: 1.0, ..Default::default() },
            esc: Esc { max_current: 20.0, ..Default::default() },
            battery: Battery { capacity_mah: 5000.0, cells: 3.0, volts: 11.1, c_cont: 25.0, ..Default::default() },
        };
        let (a, b) = (estimate(&d(1.2)), estimate(&d(2.0)));
        assert!(b.hover_throttle > a.hover_throttle && b.hover_minutes < a.hover_minutes);
        assert!(estimate(&d(4.0)).warnings.iter().any(|w| w.contains("too heavy") || w.contains("thrust/weight")));
        assert!(air_density(2000.0, 15.0) < air_density(0.0, 15.0));
    }
}
