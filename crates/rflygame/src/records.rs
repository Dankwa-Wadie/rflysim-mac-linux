//! Leaderboards, replays, ghosts and Flight School progress (plain text files in the data dir).

use crate::game::Sample;
use crate::settings::data_dir;
use macroquad::prelude::*;
use std::path::PathBuf;

/// Local timestamp-ish string (UTC seconds → yyyymmdd-hhmmss without a date crate).
pub fn stamp() -> String {
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let days = secs / 86400;
    let (y, m, d) = civil(days as i64);
    let s = secs % 86400;
    format!("{y:04}{m:02}{d:02}-{:02}{:02}{:02}", s / 3600, s / 60 % 60, s % 60)
}

/// Days since 1970-01-01 → (year, month, day) (Howard Hinnant's algorithm).
fn civil(z: i64) -> (i64, u32, u32) {
    let z = z + 719468;
    let era = z.div_euclid(146097);
    let doe = z - era * 146097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (yoe + era * 400 + (m <= 2) as i64, m, d)
}

// ---------------------------------------------------------------- leaderboard

#[derive(Clone, Debug, PartialEq)]
pub struct Entry {
    pub key: String,
    pub player: String,
    /// Race time (s, lower is better) or mission score (higher is better).
    pub value: f64,
    pub drone: String,
    pub mode: String,
    pub date: String,
}

fn lb_path() -> PathBuf {
    data_dir().join("leaderboard.txt")
}

pub fn load_entries() -> Vec<Entry> {
    let Ok(t) = std::fs::read_to_string(lb_path()) else { return vec![] };
    t.lines()
        .filter_map(|l| {
            let f: Vec<&str> = l.split('|').collect();
            (f.len() >= 6).then(|| Entry {
                key: f[0].into(),
                player: f[1].into(),
                value: f[2].parse().unwrap_or(0.0),
                drone: f[3].into(),
                mode: f[4].into(),
                date: f[5].into(),
            })
        })
        .collect()
}

/// Adds an entry; returns its rank (1-based) within its key.
pub fn submit(mut e: Entry, lower_is_better: bool) -> usize {
    e.value = (e.value * 1000.0).round() / 1000.0; // same precision as the file
    let mut all = load_entries();
    all.push(e.clone());
    let text: String = all.iter().map(|e| format!("{}|{}|{:.3}|{}|{}|{}\n", e.key, e.player.replace('|', "/"), e.value, e.drone, e.mode, e.date)).collect();
    let _ = std::fs::write(lb_path(), text);
    top(&e.key, lower_is_better, 1000).iter().position(|x| x == &e).map_or(0, |i| i + 1)
}

pub fn top(key: &str, lower_is_better: bool, n: usize) -> Vec<Entry> {
    let mut v: Vec<Entry> = load_entries().into_iter().filter(|e| e.key == key).collect();
    v.sort_by(|a, b| if lower_is_better { a.value.total_cmp(&b.value) } else { b.value.total_cmp(&a.value) });
    v.truncate(n);
    v
}

pub fn keys() -> Vec<String> {
    let mut k: Vec<String> = load_entries().into_iter().map(|e| e.key).collect();
    k.sort();
    k.dedup();
    k
}

// ---------------------------------------------------------------- replays and ghosts

pub fn save_samples(path: &PathBuf, s: &[Sample]) -> bool {
    if let Some(d) = path.parent() {
        let _ = std::fs::create_dir_all(d);
    }
    let text: String = s
        .iter()
        .map(|x| format!("{:.3},{:.3},{:.3},{:.3},{:.4},{:.4},{:.4},{:.4},{:.2}\n", x.t, x.pos.x, x.pos.y, x.pos.z, x.q.x, x.q.y, x.q.z, x.q.w, x.spin))
        .collect();
    std::fs::write(path, text).is_ok()
}

pub fn load_samples(path: &PathBuf) -> Vec<Sample> {
    let Ok(t) = std::fs::read_to_string(path) else { return vec![] };
    t.lines()
        .filter_map(|l| {
            let v: Vec<f32> = l.split(',').filter_map(|x| x.parse().ok()).collect();
            (v.len() == 9).then(|| Sample { t: v[0], pos: vec3(v[1], v[2], v[3]), q: Quat::from_xyzw(v[4], v[5], v[6], v[7]).normalize(), spin: v[8] })
        })
        .collect()
}

/// Sample at time t (linear / slerp interpolation).
pub fn sample_at(s: &[Sample], t: f32) -> Option<Sample> {
    if s.is_empty() {
        return None;
    }
    let i = s.partition_point(|x| x.t <= t);
    if i == 0 {
        return Some(s[0]);
    }
    if i >= s.len() {
        return Some(*s.last().unwrap());
    }
    let (a, b) = (s[i - 1], s[i]);
    let k = ((t - a.t) / (b.t - a.t).max(1e-4)).clamp(0.0, 1.0);
    Some(Sample { t, pos: a.pos.lerp(b.pos, k), q: a.q.slerp(b.q, k), spin: b.spin })
}

pub fn ghost_path(key: &str) -> PathBuf {
    data_dir().join("ghosts").join(format!("{}.rpl", key.replace([' ', '/', ':'], "_").replace("__", "_")))
}

/// Replays dir; name encodes key and time.
pub fn replay_dir() -> PathBuf {
    data_dir().join("replays")
}

pub fn save_replay(key: &str, map: &str, s: &[Sample]) -> Option<PathBuf> {
    let p = replay_dir().join(format!("{}__{}__{}.rpl", stamp(), key.replace([' ', '/', ':'], "_").replace("__", "_"), map));
    save_samples(&p, s).then_some(p)
}

/// Recent replays, newest first: (path, label, map).
pub fn list_replays() -> Vec<(PathBuf, String, String)> {
    let Ok(rd) = std::fs::read_dir(replay_dir()) else { return vec![] };
    let mut v: Vec<(PathBuf, String, String)> = rd
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "rpl"))
        .filter_map(|p| {
            let stem = p.file_stem()?.to_string_lossy().to_string();
            let parts: Vec<&str> = stem.split("__").collect();
            (parts.len() == 3).then(|| (p.clone(), format!("{}  {}", parts[0], parts[1].replace('_', " ")), parts[2].to_string()))
        })
        .collect();
    v.sort_by(|a, b| b.0.cmp(&a.0));
    v.truncate(12);
    v
}

// ---------------------------------------------------------------- flight school progress

pub fn school_done() -> Vec<usize> {
    std::fs::read_to_string(data_dir().join("school.txt"))
        .map(|t| t.split_whitespace().filter_map(|x| x.parse().ok()).collect())
        .unwrap_or_default()
}

pub fn mark_school_done(i: usize) {
    let mut d = school_done();
    if !d.contains(&i) {
        d.push(i);
        d.sort();
        let _ = std::fs::write(data_dir().join("school.txt"), d.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(" "));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn civil_dates() {
        assert_eq!(civil(0), (1970, 1, 1));
        assert_eq!(civil(20761), (2026, 11, 4));
    }

    #[test]
    fn interpolation() {
        let s = vec![
            Sample { t: 0.0, pos: Vec3::ZERO, q: Quat::IDENTITY, spin: 0.0 },
            Sample { t: 1.0, pos: vec3(2.0, 0.0, 0.0), q: Quat::IDENTITY, spin: 1.0 },
        ];
        assert_eq!(sample_at(&s, 0.5).unwrap().pos, vec3(1.0, 0.0, 0.0));
        assert_eq!(sample_at(&s, 5.0).unwrap().pos, vec3(2.0, 0.0, 0.0));
    }
}
