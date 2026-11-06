//! rfly-core: dependency-free multicopter simulation core for the RflySim macOS port.

pub mod dynamics;
pub mod math;
pub mod params;
pub mod sensors;

pub use dynamics::{State, Vehicle};
pub use math::{Quat, Vec3};
pub use params::{Frame, Params};
pub use sensors::{Gps, Home, Imu, SensorModel};

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.001;

    #[test]
    fn hover_throttle_is_realistic() {
        let h = Params::default().hover_throttle();
        assert!((0.55..0.65).contains(&h), "hover throttle {h}");
    }

    #[test]
    fn rests_on_ground_with_zero_throttle() {
        let mut v = Vehicle::new(Params::default());
        for _ in 0..1000 {
            v.step(&[0.0; 4], DT);
        }
        assert_eq!(v.s.pos.z, 0.0);
        assert!(v.s.on_ground);
        // Accelerometer reads -g on the body z axis (FRD) at rest.
        assert!((v.specific_force().z + 9.8).abs() < 1e-9);
    }

    #[test]
    fn climbs_above_hover_and_holds_level() {
        let mut v = Vehicle::new(Params::default());
        let u = [v.p.hover_throttle() + 0.05; 4];
        for _ in 0..2000 {
            v.step(&u, DT);
        }
        assert!(v.s.pos.z < -0.5, "should climb, z = {}", v.s.pos.z);
        let (r, p, _) = v.s.att.to_euler();
        assert!(r.abs() < 1e-6 && p.abs() < 1e-6, "symmetric thrust stays level");
    }

    #[test]
    fn differential_thrust_produces_expected_signs() {
        // Raise the right-side rotors (1 front-right, 4 rear-right) → roll left (negative roll).
        let mut v = Vehicle::new(Params::default());
        let h = v.p.hover_throttle() + 0.1;
        for _ in 0..300 {
            v.step(&[h + 0.02, h, h, h + 0.02], DT);
        }
        assert!(v.s.rate.x < 0.0, "roll rate {}", v.s.rate.x);

        // Raise CCW rotors (1, 2) → positive yaw reaction.
        let mut v = Vehicle::new(Params::default());
        for _ in 0..300 {
            v.step(&[h + 0.02, h + 0.02, h, h], DT);
        }
        assert!(v.s.rate.z > 0.0, "yaw rate {}", v.s.rate.z);
    }
}
