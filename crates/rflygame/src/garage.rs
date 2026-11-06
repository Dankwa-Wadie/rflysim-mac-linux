//! Garage: custom drones from RflySim's ModelData.db parts catalog.

use crate::flight::{from_components, presets, DronePreset};
use crate::settings::data_dir;
use crate::ui::{Item, Menu};
use rfly_db::{parse_airframe, Battery, ModelDb, Motor, Propeller};
use std::path::PathBuf;

/// Finds ModelData.db: $RFLY_DB, next to the binary, the source checkout, or the copy imported from
/// the user's RflySim ISO (see `rfly_db::find_model_db`).
pub fn db_path() -> Option<PathBuf> {
    let mut c: Vec<PathBuf> = vec![];
    if let Ok(exe) = std::env::current_exe() {
        if let Some(d) = exe.parent() {
            c.push(d.join("../Resources/ModelData.db"));
            c.push(d.join("ModelData.db"));
            c.push(d.join("../../assets/ModelData.db"));
        }
    }
    c.push(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/ModelData.db").into());
    rfly_db::find_model_db(&c)
}

/// Saved build: everything needed to recreate the drone.
#[derive(Clone, Debug, PartialEq)]
pub struct Build {
    pub name: String,
    pub frame_kg: f64,
    pub wheelbase: f64,
    pub motor: Motor,
    pub prop: Propeller,
    pub battery: Battery,
}

impl Build {
    pub fn mass(&self) -> f64 {
        self.frame_kg + 4.0 * (self.motor.weight_g + self.prop.weight_g) / 1000.0 + self.battery.weight_g / 1000.0
    }
    pub fn drone(&self) -> DronePreset {
        let mut d = from_components(
            &self.name,
            self.mass(),
            self.wheelbase,
            self.motor.kv,
            self.prop.diameter_in,
            self.prop.pitch_in,
            self.prop.ct_correct,
            self.prop.cm_correct,
            self.battery.cells.max(1.0),
            self.battery.capacity_mah,
        );
        d.desc = format!("{} | {} {} | {} {} | {} {}", d.desc, self.motor.brand, self.motor.model, self.prop.brand, self.prop.model, self.battery.brand, self.battery.model);
        d
    }
    fn line(&self) -> String {
        let m = &self.motor;
        let p = &self.prop;
        let b = &self.battery;
        format!(
            "{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}",
            self.name, self.frame_kg, self.wheelbase, m.brand, m.model, m.kv, m.weight_g, p.brand, p.model, p.diameter_in, p.pitch_in, p.weight_g, p.ct_correct, p.cm_correct,
            b.brand, b.model, b.capacity_mah, b.weight_g, b.cells, b.volts, ""
        )
    }
    fn parse(l: &str) -> Option<Build> {
        let f: Vec<&str> = l.split('|').collect();
        let n = |i: usize| f.get(i).and_then(|v| v.parse::<f64>().ok());
        Some(Build {
            name: f.first()?.to_string(),
            frame_kg: n(1)?,
            wheelbase: n(2)?,
            motor: Motor { brand: f[3].into(), model: f[4].into(), kv: n(5)?, weight_g: n(6)? },
            prop: Propeller { brand: f[7].into(), model: f[8].into(), diameter_in: n(9)?, pitch_in: n(10)?, weight_g: n(11)?, ct_correct: n(12)?, cm_correct: n(13)? },
            battery: Battery { brand: f[14].into(), model: f[15].into(), capacity_mah: n(16)?, weight_g: n(17)?, cells: n(18)?, volts: n(19)? },
        })
    }
}

fn builds_path() -> PathBuf {
    data_dir().join("drones.txt")
}

pub fn load_builds() -> Vec<Build> {
    std::fs::read_to_string(builds_path()).map(|t| t.lines().filter_map(Build::parse).collect()).unwrap_or_default()
}

pub fn save_build(b: &Build) {
    let mut all: Vec<Build> = load_builds().into_iter().filter(|x| x.name != b.name).collect();
    all.push(b.clone());
    let _ = std::fs::write(builds_path(), all.iter().map(|b| b.line() + "\n").collect::<String>());
}

/// All flyable drones: presets + saved builds.
pub fn all_drones() -> Vec<DronePreset> {
    let mut v = presets();
    v.extend(load_builds().iter().map(|b| b.drone()));
    v
}

pub struct Garage {
    pub motors: Vec<Motor>,
    pub props: Vec<Propeller>,
    pub bats: Vec<Battery>,
    airframes: Vec<rfly_db::Airframe>,
    mbrands: Vec<String>,
    pbrands: Vec<String>,
    bbrands: Vec<String>,
    pub menu: Menu,
    pub build: Build,
    pub ok: bool,
}

fn brands<T>(v: &[T], f: impl Fn(&T) -> &str) -> Vec<String> {
    let mut b: Vec<String> = v.iter().map(|x| f(x).to_string()).collect();
    b.dedup();
    b
}

// Menu rows.
pub const BASE: usize = 0;
pub const WHEEL: usize = 1;
pub const FRAME: usize = 2;
pub const MB: usize = 3;
pub const MM: usize = 4;
pub const PB: usize = 5;
pub const PM: usize = 6;
pub const BB: usize = 7;
pub const BM: usize = 8;
pub const INFO1: usize = 9;
pub const INFO2: usize = 10;
pub const SAVE: usize = 11;
pub const SAVE_FLY: usize = 12;
pub const BACK: usize = 13;

impl Garage {
    pub fn open() -> Self {
        let db = db_path().and_then(|p| ModelDb::open(p).ok());
        let (motors, props, bats, airframes) = match &db {
            Some(d) => (
                d.motors().unwrap_or_default(),
                d.propellers().unwrap_or_default(),
                d.batteries().unwrap_or_default(),
                d.airframes().unwrap_or_default().iter().filter_map(|(n, i)| parse_airframe(n, i)).collect(),
            ),
            None => (vec![], vec![], vec![], vec![]),
        };
        let ok = !motors.is_empty() && !props.is_empty() && !bats.is_empty();
        let mbrands = brands(&motors, |m| &m.brand);
        let pbrands = brands(&props, |p| &p.brand);
        let bbrands = brands(&bats, |b| &b.brand);
        let mut g = Garage {
            build: Build {
                name: format!("Custom {}", load_builds().len() + 1),
                frame_kg: 0.8,
                wheelbase: 450.0,
                motor: motors.first().cloned().unwrap_or(Motor { brand: String::new(), model: String::new(), kv: 920.0, weight_g: 55.0 }),
                prop: props.first().cloned().unwrap_or(Propeller { brand: String::new(), model: String::new(), diameter_in: 10.0, pitch_in: 4.5, weight_g: 15.0, ct_correct: 1.0, cm_correct: 1.0 }),
                battery: bats.first().cloned().unwrap_or(Battery { brand: String::new(), model: String::new(), capacity_mah: 5000.0, weight_g: 420.0, cells: 3.0, volts: 11.1 }),
            },
            menu: Menu::new("Garage", "", vec![]),
            motors,
            props,
            bats,
            airframes,
            mbrands,
            pbrands,
            bbrands,
            ok,
        };
        // Start from the DJI F450 if present.
        let base = g.airframes.iter().position(|a| a.name == "DJI F450").map_or(0, |i| i + 1);
        g.rebuild_menu(base);
        if base > 0 {
            g.apply_airframe(base - 1);
        }
        g
    }

    fn model_names<T>(v: &[T], brand: &str, b: impl Fn(&T) -> &str, m: impl Fn(&T) -> &str) -> Vec<String> {
        v.iter().filter(|x| b(x) == brand).map(|x| m(x).to_string()).collect()
    }

    fn rebuild_menu(&mut self, base: usize) {
        let bd = &self.build;
        let mut bases = vec!["Custom".to_string()];
        bases.extend(self.airframes.iter().map(|a| a.name.clone()));
        let idx = |v: &[String], s: &str| v.iter().position(|x| x == s).unwrap_or(0);
        let mm = Self::model_names(&self.motors, &bd.motor.brand, |x| &x.brand, |x| &x.model);
        let pm = Self::model_names(&self.props, &bd.prop.brand, |x| &x.brand, |x| &x.model);
        let bm = Self::model_names(&self.bats, &bd.battery.brand, |x| &x.brand, |x| &x.model);
        let or1 = |v: Vec<String>| if v.is_empty() { vec!["-".to_string()] } else { v };
        let (mm, pm, bm) = (or1(mm), or1(pm), or1(bm));
        let sel = self.menu.sel;
        self.menu = Menu::new(
            "Garage",
            "Build a drone from RflySim's parts catalog (ModelData.db).",
            vec![
                Item::Choice("Start from airframe".into(), bases, base),
                Item::Number("Wheelbase".into(), bd.wheelbase, 150.0, 1500.0, 10.0, " mm"),
                Item::Number("Frame mass (no motors/battery)".into(), bd.frame_kg, 0.05, 15.0, 0.05, " kg"),
                Item::Choice("Motor brand".into(), self.mbrands.clone(), idx(&self.mbrands, &bd.motor.brand)),
                Item::Choice("Motor".into(), mm.clone(), idx(&mm, &bd.motor.model)),
                Item::Choice("Propeller brand".into(), self.pbrands.clone(), idx(&self.pbrands, &bd.prop.brand)),
                Item::Choice("Propeller".into(), pm.clone(), idx(&pm, &bd.prop.model)),
                Item::Choice("Battery brand".into(), self.bbrands.clone(), idx(&self.bbrands, &bd.battery.brand)),
                Item::Choice("Battery".into(), bm.clone(), idx(&bm, &bd.battery.model)),
                Item::Info(String::new()),
                Item::Info(String::new()),
                Item::Button("Save".into()),
                Item::Button("Save and fly".into()),
                Item::Button("Back".into()),
            ],
        );
        self.menu.sel = sel.min(self.menu.items.len() - 1);
        self.update_info();
    }

    fn apply_airframe(&mut self, i: usize) {
        let a = self.airframes[i].clone();
        if let Some(m) = self.motors.iter().find(|m| m.brand == a.motor.0 && m.model == a.motor.1) {
            self.build.motor = m.clone();
        }
        if let Some(p) = self.props.iter().find(|p| p.brand == a.prop.0 && p.model == a.prop.1) {
            self.build.prop = p.clone();
        }
        if let Some(b) = self.bats.iter().find(|b| b.brand == a.battery.0 && b.model == a.battery.1) {
            self.build.battery = b.clone();
        }
        self.build.wheelbase = a.wheelbase_mm;
        let parts = 4.0 * (self.build.motor.weight_g + self.build.prop.weight_g) / 1000.0 + self.build.battery.weight_g / 1000.0;
        self.build.frame_kg = (a.mass_kg - parts).max(0.1);
        self.build.name = a.name.clone();
        self.rebuild_menu(i + 1);
    }

    fn update_info(&mut self) {
        let d = self.build.drone();
        let p = &d.params;
        let hover = d.params.hover_throttle();
        self.menu.items[INFO1] = Item::Info(format!("Total {:.2} kg   thrust/weight {:.2}   hover throttle {:.0}%", self.build.mass(), d.thrust_to_weight(), hover * 100.0));
        let warn = if d.thrust_to_weight() < 1.3 { "  TOO HEAVY - will not fly well!" } else { "" };
        self.menu.items[INFO2] = Item::Info(format!("Top speed ~{:.0} m/s   flight time ~{:.1} min   motor lag {:.0} ms{}", d.top_speed(), d.hover_minutes(), p.tm * 1000.0, warn));
        self.menu.subtitle = format!("Build: {}", self.build.name);
    }

    /// Applies a menu change. Returns true when the build changed.
    pub fn changed(&mut self, i: usize) {
        let pick = |names: &[String], k: usize| names.get(k).cloned().unwrap_or_default();
        match i {
            BASE => {
                let k = self.menu.choice(BASE);
                if k > 0 {
                    self.apply_airframe(k - 1);
                } else {
                    self.build.name = format!("Custom {}", load_builds().len() + 1);
                    self.update_info();
                }
                return;
            }
            WHEEL => self.build.wheelbase = self.menu.number(WHEEL),
            FRAME => self.build.frame_kg = self.menu.number(FRAME),
            MB => {
                let b = pick(&self.mbrands, self.menu.choice(MB));
                if let Some(m) = self.motors.iter().find(|m| m.brand == b) {
                    self.build.motor = m.clone();
                }
            }
            MM => {
                let names = Self::model_names(&self.motors, &self.build.motor.brand, |x| &x.brand, |x| &x.model);
                let n = pick(&names, self.menu.choice(MM));
                if let Some(m) = self.motors.iter().find(|m| m.brand == self.build.motor.brand && m.model == n) {
                    self.build.motor = m.clone();
                }
            }
            PB => {
                let b = pick(&self.pbrands, self.menu.choice(PB));
                if let Some(p) = self.props.iter().find(|p| p.brand == b) {
                    self.build.prop = p.clone();
                }
            }
            PM => {
                let names = Self::model_names(&self.props, &self.build.prop.brand, |x| &x.brand, |x| &x.model);
                let n = pick(&names, self.menu.choice(PM));
                if let Some(p) = self.props.iter().find(|p| p.brand == self.build.prop.brand && p.model == n) {
                    self.build.prop = p.clone();
                }
            }
            BB => {
                let b = pick(&self.bbrands, self.menu.choice(BB));
                if let Some(x) = self.bats.iter().find(|x| x.brand == b) {
                    self.build.battery = x.clone();
                }
            }
            BM => {
                let names = Self::model_names(&self.bats, &self.build.battery.brand, |x| &x.brand, |x| &x.model);
                let n = pick(&names, self.menu.choice(BM));
                if let Some(x) = self.bats.iter().find(|x| x.brand == self.build.battery.brand && x.model == n) {
                    self.build.battery = x.clone();
                }
            }
            _ => return,
        }
        if self.build.name.starts_with("DJI") || self.airframes.iter().any(|a| a.name == self.build.name) {
            self.build.name = format!("{} (mod)", self.build.name.trim_end_matches(" (mod)"));
        }
        let base = self.menu.choice(BASE);
        if matches!(i, MB | PB | BB) {
            self.rebuild_menu(base);
        } else {
            self.update_info();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn garage_loads_catalog_and_builds_flyable_f450() {
        // RflySim's ModelData.db is not in the repository (users import it); skip without it (fresh clone, CI).
        if db_path().is_none() {
            eprintln!("skipped: ModelData.db not found (scripts/import_rflysim_data.sh)");
            return;
        }
        let g = Garage::open();
        assert!(g.ok, "catalog loaded");
        assert_eq!(g.build.name, "DJI F450");
        let d = g.build.drone();
        assert!((1.4..1.6).contains(&d.params.mass), "mass {}", d.params.mass);
        assert!(d.thrust_to_weight() > 1.8);
        let line = g.build.line();
        assert_eq!(Build::parse(&line), Some(g.build.clone()));
    }
}
