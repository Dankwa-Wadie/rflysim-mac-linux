//! Read vehicle parameter presets from RflySim's `ModelData.db` (SQLite).
//!
//! Schema (observed): `uavparams(name, modelInfo, hoverInfo, frameInfo)`, where `modelInfo` is a
//! comma-separated list of 14 floats; see `rfly_core::Params::from_model_info`.

use rfly_core::Params;
use rusqlite::{Connection, OpenFlags};
use std::path::{Path, PathBuf};

pub use rusqlite::Error;

pub struct ModelDb {
    conn: Connection,
}

impl ModelDb {
    pub fn open(path: impl AsRef<Path>) -> Result<Self, Error> {
        let conn = Connection::open_with_flags(path, OpenFlags::SQLITE_OPEN_READ_ONLY)?;
        Ok(Self { conn })
    }

    /// Names of stored parameter sets (e.g. "Default", "latest").
    pub fn param_set_names(&self) -> Result<Vec<String>, Error> {
        let mut st = self.conn.prepare("SELECT name FROM uavparams")?;
        let rows = st.query_map([], |r| r.get::<_, String>(0))?;
        rows.collect()
    }

    /// Load a named parameter set. Returns `Ok(None)` if absent or malformed.
    pub fn params(&self, name: &str) -> Result<Option<Params>, Error> {
        let mut st = self.conn.prepare("SELECT modelInfo FROM uavparams WHERE name = ?1 COLLATE NOCASE")?;
        let mut rows = st.query([name])?;
        let Some(row) = rows.next()? else { return Ok(None) };
        let info: String = row.get(0)?;
        let vals: Option<Vec<f64>> = info.split(',').map(|s| s.trim().parse().ok()).collect();
        Ok(vals.and_then(|v| Params::from_model_info(&v)))
    }

    /// Motor catalog (brand, type, KV, weight g).
    pub fn motors(&self) -> Result<Vec<Motor>, Error> {
        let mut st = self.conn.prepare("SELECT brand, type, KV, weight, resistance, IO, UO, Immax FROM motor ORDER BY brand, type")?;
        let rows = st.query_map([], |r| {
            Ok(Motor {
                brand: txt(r, 0),
                model: txt(r, 1),
                kv: num(r, 2),
                weight_g: num(r, 3),
                resistance_mohm: num(r, 4),
                i0: num(r, 5),
                u0: num(r, 6),
                imax: num(r, 7),
            })
        })?;
        Ok(rows.filter_map(Result::ok).filter(|m| m.kv > 0.0).collect())
    }

    /// Propeller catalog (diameter and pitch in inches, weight g, thrust/torque corrections).
    pub fn propellers(&self) -> Result<Vec<Propeller>, Error> {
        let mut st = self.conn.prepare("SELECT brand, type, diameter, pitch, weight, CTCorrect, CMCorrect FROM propeller ORDER BY brand, type")?;
        let rows = st.query_map([], |r| {
            Ok(Propeller {
                brand: txt(r, 0),
                model: txt(r, 1),
                diameter_in: num(r, 2),
                pitch_in: num(r, 3),
                weight_g: num(r, 4),
                ct_correct: Some(num(r, 5)).filter(|v| *v > 0.0).unwrap_or(1.0),
                cm_correct: Some(num(r, 6)).filter(|v| *v > 0.0).unwrap_or(1.0),
            })
        })?;
        Ok(rows.filter_map(Result::ok).filter(|p| p.diameter_in > 0.0).collect())
    }

    /// Battery catalog (capacity mAh, weight g, cells, nominal volts).
    pub fn batteries(&self) -> Result<Vec<Battery>, Error> {
        let mut st = self.conn.prepare("SELECT brand, type, capacity, weight, Snum, totalV, resistance, Ccon FROM battery ORDER BY brand, type")?;
        let rows = st.query_map([], |r| {
            let cells = num(r, 4);
            let v = num(r, 5);
            Ok(Battery {
                brand: txt(r, 0),
                model: txt(r, 1),
                capacity_mah: num(r, 2),
                weight_g: num(r, 3),
                cells,
                volts: if v > 0.0 { v } else { cells * 3.7 },
                resistance_mohm: num(r, 6),
                c_cont: num(r, 7),
            })
        })?;
        Ok(rows.filter_map(Result::ok).filter(|b| b.capacity_mah > 0.0 && b.volts > 0.0).collect())
    }

    /// ESC catalog (max continuous current A, resistance mOhm, weight g).
    pub fn escs(&self) -> Result<Vec<Esc>, Error> {
        let mut st = self.conn.prepare("SELECT brand, type, maxI, resistance, weight FROM esc ORDER BY brand, type")?;
        let rows = st.query_map([], |r| {
            Ok(Esc { brand: txt(r, 0), model: txt(r, 1), max_current: num(r, 2), resistance_mohm: num(r, 3), weight_g: num(r, 4) })
        })?;
        Ok(rows.filter_map(Result::ok).filter(|e| e.max_current > 0.0).collect())
    }

    /// RflySim's stored hover estimate for a parameter set (`uavparams.hoverInfo`).
    pub fn hover_info(&self, name: &str) -> Result<Option<Vec<f64>>, Error> {
        let mut st = self.conn.prepare("SELECT hoverInfo FROM uavparams WHERE name = ?1 COLLATE NOCASE")?;
        let mut rows = st.query([name])?;
        let Some(row) = rows.next()? else { return Ok(None) };
        let info: String = row.get(0)?;
        Ok(info.split(',').map(|s| s.trim().parse().ok()).collect())
    }

    /// Named airframe presets (`planeItems`): (name, raw component spec).
    pub fn airframes(&self) -> Result<Vec<(String, String)>, Error> {
        let mut st = self
            .conn
            .prepare("SELECT name, item FROM planeItems WHERE name <> '' ORDER BY name")?;
        let rows = st.query_map([], |r| Ok((r.get(0)?, r.get(1)?)))?;
        rows.collect()
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Motor {
    pub brand: String,
    pub model: String,
    pub kv: f64,
    pub weight_g: f64,
    /// Winding resistance (mOhm), no-load current I0 (A) at U0 (V), max current (A); 0 when unknown.
    pub resistance_mohm: f64,
    pub i0: f64,
    pub u0: f64,
    pub imax: f64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Esc {
    pub brand: String,
    pub model: String,
    /// Max continuous current (A).
    pub max_current: f64,
    /// On-resistance (mOhm), 0 when unknown.
    pub resistance_mohm: f64,
    pub weight_g: f64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Propeller {
    pub brand: String,
    pub model: String,
    pub diameter_in: f64,
    pub pitch_in: f64,
    pub weight_g: f64,
    pub ct_correct: f64,
    pub cm_correct: f64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Battery {
    pub brand: String,
    pub model: String,
    pub capacity_mah: f64,
    pub weight_g: f64,
    pub cells: f64,
    pub volts: f64,
    /// Internal resistance (mOhm) and continuous C rating; 0 when unknown.
    pub resistance_mohm: f64,
    pub c_cont: f64,
}

/// One parsed `planeItems` airframe: frame mass (kg), wheelbase (mm) and component names.
#[derive(Clone, Debug, PartialEq)]
pub struct Airframe {
    pub name: String,
    pub frame: String,
    pub mass_kg: f64,
    pub wheelbase_mm: f64,
    pub motor: (String, String),
    pub prop: (String, String),
    pub esc: (String, String),
    pub battery: (String, String),
}

/// Per-user folder for data imported from the user's RflySim ISO (`scripts/import_rflysim_data.sh`):
/// `$RFLY_TOOLS_DATA`, else macOS `~/Library/Application Support/RflyTools`, Linux
/// `$XDG_DATA_HOME/rflytools` (default `~/.local/share/rflytools`), Windows `%APPDATA%\\RflyTools`.
pub fn tools_data_dir() -> Option<PathBuf> {
    let var = |k: &str| std::env::var_os(k).filter(|v| !v.is_empty()).map(PathBuf::from);
    if let Some(d) = var("RFLY_TOOLS_DATA") {
        return Some(d);
    }
    let home = var("HOME");
    if cfg!(target_os = "macos") {
        home.map(|h| h.join("Library/Application Support/RflyTools"))
    } else if cfg!(windows) {
        var("APPDATA").map(|a| a.join("RflyTools"))
    } else {
        var("XDG_DATA_HOME").filter(|p| p.is_absolute()).or_else(|| home.map(|h| h.join(".local/share"))).map(|d| d.join("rflytools"))
    }
}

/// Finds RflySim's ModelData.db: `$RFLY_DB`, then `extra` (e.g. a source checkout's assets/), then the
/// imported copy in [`tools_data_dir`]. RflyTools does not ship the file; it belongs to RflySim.
pub fn find_model_db(extra: &[PathBuf]) -> Option<PathBuf> {
    let mut c: Vec<PathBuf> = std::env::var_os("RFLY_DB").filter(|v| !v.is_empty()).map(PathBuf::from).into_iter().collect();
    c.extend(extra.iter().cloned());
    c.extend(tools_data_dir().map(|d| d.join("ModelData.db")));
    c.into_iter().find(|p| p.is_file())
}

/// Parses `QUAD,0,1.5,450,...;...;MotorBrand,Type,...;PropBrand,Type,...;EscBrand,Type,...;BatBrand,Type,...`.
pub fn parse_airframe(name: &str, item: &str) -> Option<Airframe> {
    let parts: Vec<Vec<&str>> = item.split(';').map(|p| p.split(',').collect()).collect();
    let f = parts.first()?;
    let pair = |i: usize| parts.get(i).map(|p| (p.first().unwrap_or(&"").to_string(), p.get(1).unwrap_or(&"").to_string()));
    Some(Airframe {
        name: name.to_string(),
        frame: f.first()?.to_string(),
        mass_kg: f.get(2)?.parse().ok()?,
        wheelbase_mm: f.get(3)?.parse().ok()?,
        motor: pair(2)?,
        prop: pair(3)?,
        esc: pair(4)?,
        battery: pair(5)?,
    })
}

fn txt(r: &rusqlite::Row, i: usize) -> String {
    r.get::<_, Option<String>>(i).ok().flatten().unwrap_or_default()
}
fn num(r: &rusqlite::Row, i: usize) -> f64 {
    match r.get_ref(i) {
        Ok(rusqlite::types::ValueRef::Integer(v)) => v as f64,
        Ok(rusqlite::types::ValueRef::Real(v)) => v,
        Ok(rusqlite::types::ValueRef::Text(t)) => std::str::from_utf8(t).ok().and_then(|s| s.trim().parse().ok()).unwrap_or(0.0),
        _ => 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// RflySim's ModelData.db is not in the repository (it belongs to RflySim). These tests run when
    /// it is in assets/ or imported, and skip otherwise (fresh clone, CI).
    fn db() -> Option<ModelDb> {
        let repo = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/ModelData.db"));
        match find_model_db(&[repo]) {
            Some(p) => Some(ModelDb::open(p).unwrap()),
            None => {
                eprintln!("skipped: ModelData.db not found (scripts/import_rflysim_data.sh)");
                None
            }
        }
    }

    #[test]
    fn default_params_match_builtin() {
        let Some(db) = db() else { return };
        let p = db.params("Default").unwrap().expect("Default row");
        let d = Params::default();
        assert_eq!(p.mass, d.mass);
        assert_eq!(p.ct, d.ct);
        assert_eq!(p.cmd, d.cmd);
    }

    #[test]
    fn catalogs_and_airframes_parse() {
        let Some(d) = db() else { return };
        let m = d.motors().unwrap();
        let p = d.propellers().unwrap();
        let b = d.batteries().unwrap();
        assert!(m.len() > 1000 && p.len() > 1000 && b.len() > 1000, "{} {} {}", m.len(), p.len(), b.len());
        assert!(m.iter().any(|x| x.model == "2312 KV960" && x.kv == 960.0));
        let (n, item) = d.airframes().unwrap().into_iter().find(|(n, _)| n == "DJI F450").unwrap();
        let a = parse_airframe(&n, &item).unwrap();
        assert_eq!((a.mass_kg, a.wheelbase_mm), (1.5, 450.0));
        assert_eq!(a.motor, ("DJI".into(), "2312 KV960".into()));
        assert_eq!(a.battery.1, "LiPo 3S-11.1V-25C-5500mAh");
        assert_eq!(a.esc, ("DJI".into(), "E310".into()));
        let m = m.iter().find(|x| x.model == "2312 KV960").unwrap();
        assert!(m.resistance_mohm > 0.0 && m.i0 > 0.0, "{m:?}");
        let e = d.escs().unwrap();
        assert!(e.len() > 500 && e.iter().all(|x| x.max_current > 0.0), "{}", e.len());
        let h = d.hover_info("default").unwrap().unwrap();
        assert_eq!(h.len(), 8);
    }

    #[test]
    fn lists_airframes() {
        let Some(db) = db() else { return };
        let a = db.airframes().unwrap();
        assert!(a.iter().any(|(n, _)| n == "DJI F450"));
    }
}
