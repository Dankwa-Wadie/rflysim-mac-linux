//! Sensor synthesis: IMU, magnetometer, barometer, GPS.

use crate::dynamics::Vehicle;
use crate::math::Vec3;

/// Geodetic origin of the local NED frame.
#[derive(Clone, Copy, Debug)]
pub struct Home {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
    /// Earth magnetic field at home, NED (gauss).
    pub mag_ned: Vec3,
}

impl Default for Home {
    /// PX4 SITL default home (Zürich), with a matching WMM field so EKF2 mag checks pass.
    fn default() -> Self {
        Self {
            lat_deg: 47.397742,
            lon_deg: 8.545594,
            alt_m: 488.0,
            mag_ned: Vec3::new(0.2177, 0.0114, 0.4277),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Imu {
    pub acc: Vec3,
    pub gyro: Vec3,
    pub mag: Vec3,
    /// hPa
    pub abs_pressure: f64,
    pub pressure_alt: f64,
    pub temperature: f64,
    /// Pitot differential pressure (hPa) from the forward airspeed (fixed-wing airspeed sensor).
    pub diff_pressure: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Gps {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
    pub vel_ned: Vec3,
}

/// Small deterministic Gaussian noise source (xorshift64* + Box–Muller).
pub struct Noise(u64);

impl Noise {
    pub fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        ((self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    pub fn gauss(&mut self, sigma: f64) -> f64 {
        let (u1, u2) = (self.uniform(), self.uniform());
        sigma * (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
    pub fn vec(&mut self, sigma: f64) -> Vec3 {
        Vec3::new(self.gauss(sigma), self.gauss(sigma), self.gauss(sigma))
    }
}

const EARTH_R: f64 = 6_378_137.0;

pub struct SensorModel {
    pub home: Home,
    pub noise: Noise,
}

impl SensorModel {
    pub fn new(home: Home) -> Self {
        Self { home, noise: Noise::new(0x5eed) }
    }

    pub fn imu(&mut self, v: &Vehicle) -> Imu {
        let alt = self.home.alt_m - v.s.pos.z;
        // ISA troposphere.
        let p_pa = 101_325.0 * (1.0 - 2.25577e-5 * alt).powf(5.25588);
        let baro_noise = self.noise.gauss(0.01);
        Imu {
            acc: v.specific_force() + self.noise.vec(0.02),
            gyro: v.s.rate + self.noise.vec(0.002),
            mag: v.s.att.rotate_inv(self.home.mag_ned) + self.noise.vec(0.001),
            abs_pressure: p_pa / 100.0 + baro_noise,
            pressure_alt: alt,
            temperature: 15.0 - 0.0065 * alt,
            diff_pressure: {
                let u = v.s.att.rotate_inv(v.s.vel).x.max(0.0);
                0.5 * 1.225 * u * u / 100.0 + self.noise.gauss(0.002).abs()
            },
        }
    }

    pub fn gps(&mut self, v: &Vehicle) -> Gps {
        let lat0 = self.home.lat_deg.to_radians();
        let n = v.s.pos.x + self.noise.gauss(0.05);
        let e = v.s.pos.y + self.noise.gauss(0.05);
        Gps {
            lat_deg: self.home.lat_deg + (n / EARTH_R).to_degrees(),
            lon_deg: self.home.lon_deg + (e / (EARTH_R * lat0.cos())).to_degrees(),
            alt_m: self.home.alt_m - v.s.pos.z + self.noise.gauss(0.1),
            vel_ned: v.s.vel + self.noise.vec(0.02),
        }
    }
}
