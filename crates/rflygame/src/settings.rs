//! Persistent settings (simple key = value file in the per-user data folder, see [`data_dir`]).

use crate::input::{AxisBind, AxisMap};
use std::path::PathBuf;

#[derive(Clone, Debug)]
pub struct Settings {
    pub player: String,
    pub fov_deg: f32,
    pub fpv_uptilt_deg: f32,
    pub acro_rate: f64,
    pub expo: f64,
    pub max_angle_deg: f64,
    pub deadzone: f32,
    pub invert_pitch: bool,
    pub map: AxisMap,
    pub show_sticks: bool,
    pub battery: bool,
    pub last_server: String,
    pub kb_mode2: bool,
    /// 0 = Auto (FPV except Beginner), 1 = always FPV, 2 = always Chase.
    pub default_cam: u8,
    /// 0 = realistic (impacts disarm, drone falls, recover), 1 = arcade (crash screen + reset).
    pub crash_style: u8,
    /// Sound volume 0..10 (0 = muted).
    pub volume: u8,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            player: std::env::var("USER").unwrap_or_else(|_| "Pilot".into()),
            fov_deg: 100.0,
            fpv_uptilt_deg: 20.0,
            acro_rate: 600.0,
            expo: 0.4,
            max_angle_deg: 35.0,
            deadzone: 0.04,
            invert_pitch: false,
            map: AxisMap::default(),
            show_sticks: true,
            battery: true,
            last_server: String::new(),
            kb_mode2: true,
            default_cam: 0,
            crash_style: 0,
            volume: 6,
        }
    }
}

/// Per-user data folder: `$RFLY_DATA_DIR` if set, else the platform's usual place
/// (macOS `~/Library/Application Support/RflyGame`, Linux `$XDG_DATA_HOME/rflygame`
/// or `~/.local/share/rflygame`, Windows `%APPDATA%\RflyGame`).
pub fn data_dir() -> PathBuf {
    let d = platform_data_dir(|k| std::env::var_os(k).filter(|v| !v.is_empty()).map(PathBuf::from));
    let _ = std::fs::create_dir_all(&d);
    d
}

fn platform_data_dir(env: impl Fn(&str) -> Option<PathBuf>) -> PathBuf {
    if let Some(d) = env("RFLY_DATA_DIR") {
        return d;
    }
    let home = env("HOME").unwrap_or_else(|| ".".into());
    if cfg!(target_os = "macos") {
        home.join("Library/Application Support/RflyGame")
    } else if cfg!(windows) {
        env("APPDATA").unwrap_or(home).join("RflyGame")
    } else {
        env("XDG_DATA_HOME").filter(|p| p.is_absolute()).unwrap_or_else(|| home.join(".local/share")).join("rflygame")
    }
}

fn bind_str(b: AxisBind) -> String {
    format!("{}{}", b.index, if b.invert { "i" } else { "" })
}
fn parse_bind(s: &str) -> Option<AxisBind> {
    let inv = s.ends_with('i');
    s.trim_end_matches('i').parse().ok().map(|index| AxisBind { index, invert: inv })
}

impl Settings {
    pub fn load() -> Self {
        let mut s = Settings::default();
        let Ok(text) = std::fs::read_to_string(data_dir().join("settings.txt")) else { return s };
        for line in text.lines() {
            let Some((k, v)) = line.split_once('=') else { continue };
            let (k, v) = (k.trim(), v.trim());
            match k {
                "player" => s.player = v.to_string(),
                "fov" => s.fov_deg = v.parse().unwrap_or(s.fov_deg),
                "uptilt" => s.fpv_uptilt_deg = v.parse().unwrap_or(s.fpv_uptilt_deg),
                "acro_rate" => s.acro_rate = v.parse().unwrap_or(s.acro_rate),
                "expo" => s.expo = v.parse().unwrap_or(s.expo),
                "max_angle" => s.max_angle_deg = v.parse().unwrap_or(s.max_angle_deg),
                "deadzone" => s.deadzone = v.parse().unwrap_or(s.deadzone),
                "invert_pitch" => s.invert_pitch = v == "1",
                "show_sticks" => s.show_sticks = v == "1",
                "battery" => s.battery = v == "1",
                "last_server" => s.last_server = v.to_string(),
                "kb_mode2" => s.kb_mode2 = v == "1",
                "default_cam" => s.default_cam = v.parse().unwrap_or(0),
                "crash_style" => s.crash_style = v.parse().unwrap_or(0),
                "volume" => s.volume = v.parse::<u8>().unwrap_or(6).min(10),
                "axis_roll" => s.map.roll = parse_bind(v).unwrap_or(s.map.roll),
                "axis_pitch" => s.map.pitch = parse_bind(v).unwrap_or(s.map.pitch),
                "axis_yaw" => s.map.yaw = parse_bind(v).unwrap_or(s.map.yaw),
                "axis_throttle" => s.map.throttle = parse_bind(v).unwrap_or(s.map.throttle),
                _ => {}
            }
        }
        s
    }

    pub fn save(&self) {
        let m = &self.map;
        let text = format!(
            "player = {}\nfov = {}\nuptilt = {}\nacro_rate = {}\nexpo = {}\nmax_angle = {}\ndeadzone = {}\ninvert_pitch = {}\nshow_sticks = {}\nbattery = {}\nlast_server = {}\nkb_mode2 = {}\ndefault_cam = {}\ncrash_style = {}\nvolume = {}\naxis_roll = {}\naxis_pitch = {}\naxis_yaw = {}\naxis_throttle = {}\n",
            self.player,
            self.fov_deg,
            self.fpv_uptilt_deg,
            self.acro_rate,
            self.expo,
            self.max_angle_deg,
            self.deadzone,
            self.invert_pitch as u8,
            self.show_sticks as u8,
            self.battery as u8,
            self.last_server,
            self.kb_mode2 as u8,
            self.default_cam,
            self.crash_style,
            self.volume,
            bind_str(m.roll),
            bind_str(m.pitch),
            bind_str(m.yaw),
            bind_str(m.throttle)
        );
        if let Err(e) = std::fs::write(data_dir().join("settings.txt"), text) {
            eprintln!("rflygame: cannot save settings: {e}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::platform_data_dir;
    use std::path::PathBuf;

    fn env<'a>(vars: &'a [(&'a str, &'a str)]) -> impl Fn(&str) -> Option<PathBuf> + 'a {
        move |k| vars.iter().find(|(n, _)| *n == k).map(|(_, v)| PathBuf::from(v))
    }

    #[test]
    fn data_dir_follows_platform_conventions() {
        assert_eq!(platform_data_dir(env(&[("RFLY_DATA_DIR", "/x/y"), ("HOME", "/h")])), PathBuf::from("/x/y"));
        let d = platform_data_dir(env(&[("HOME", "/h"), ("XDG_DATA_HOME", "/xdg"), ("APPDATA", "/ad")]));
        let want = if cfg!(target_os = "macos") {
            "/h/Library/Application Support/RflyGame"
        } else if cfg!(windows) {
            "/ad/RflyGame"
        } else {
            "/xdg/rflygame"
        };
        assert_eq!(d, PathBuf::from(want));
        if cfg!(all(unix, not(target_os = "macos"))) {
            // A relative XDG_DATA_HOME is invalid per the spec and is ignored.
            assert_eq!(platform_data_dir(env(&[("HOME", "/h"), ("XDG_DATA_HOME", "rel")])), PathBuf::from("/h/.local/share/rflygame"));
        }
    }
}
