//! mechfight/1 message types (see protocol.md and mechfight.schema.json).

use serde::{Deserialize, Deserializer};
use std::collections::HashMap;

pub type Stats = HashMap<String, f64>;

/// Accepts any JSON number (or null) for an integer field: `3`, `3.0`, `3e0`.
fn de_int<'de, D: Deserializer<'de>>(d: D) -> Result<i64, D::Error> {
    let v: Option<f64> = Option::deserialize(d)?;
    Ok(v.map(|f| f as i64).unwrap_or(0))
}

fn de_opt_int<'de, D: Deserializer<'de>>(d: D) -> Result<Option<i64>, D::Error> {
    let v: Option<f64> = Option::deserialize(d)?;
    Ok(v.map(|f| f as i64))
}

/// null-tolerant default for any field.
fn de_null<'de, D, T>(d: D) -> Result<T, D::Error>
where
    D: Deserializer<'de>,
    T: Deserialize<'de> + Default,
{
    Ok(Option::<T>::deserialize(d)?.unwrap_or_default())
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct MapInfo {
    #[serde(deserialize_with = "de_null")]
    pub dir: String,
    #[serde(deserialize_with = "de_null")]
    pub files: Vec<String>,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct Hello {
    #[serde(rename = "type", deserialize_with = "de_null")]
    pub typ: String,
    #[serde(deserialize_with = "de_null")]
    pub protocol: String,
    #[serde(deserialize_with = "de_int")]
    pub slot: i64,
    #[serde(deserialize_with = "de_int")]
    pub seed: i64,
    #[serde(deserialize_with = "de_null")]
    pub map: MapInfo,
    #[serde(deserialize_with = "de_null")]
    pub config: Config,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct ZonePhase {
    pub hold_s: f64,
    pub shrink_s: f64,
    pub radius: f64,
    pub dps: f64,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct Vision {
    pub range: f64,
    pub fov_deg: f64,
    pub near_range: f64,
    pub canopy_blocks: bool,
    pub show_enemy_hp: bool,
    pub memory_s: f64,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct CubesCfg {
    pub pickup_radius: f64,
    pub hp_per_cube: f64,
    pub damage_per_cube: f64,
    pub stat_cap: f64,
    pub death_drop_extra: f64,
    pub size_per_cube: f64,
    #[serde(deserialize_with = "de_null")]
    pub upgrade_thresholds: Vec<f64>,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct LootCfg {
    pub pickup_radius: f64,
    pub equip_s: f64,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct ZoneCfg {
    #[serde(deserialize_with = "de_null")]
    pub start_center: Vec<f64>,
    pub start_radius: f64,
    #[serde(deserialize_with = "de_null")]
    pub phases: Vec<ZonePhase>,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct Config {
    #[serde(rename = "match", deserialize_with = "de_null")]
    pub match_: Stats,
    #[serde(deserialize_with = "de_null")]
    pub mech: Stats,
    #[serde(deserialize_with = "de_null")]
    pub vision: Vision,
    #[serde(deserialize_with = "de_null")]
    pub weapons: HashMap<String, Stats>,
    #[serde(deserialize_with = "de_null")]
    pub cubes: CubesCfg,
    #[serde(deserialize_with = "de_null")]
    pub loot: LootCfg,
    #[serde(deserialize_with = "de_null")]
    pub npc: Stats,
    #[serde(deserialize_with = "de_null")]
    pub crates: Stats,
    #[serde(deserialize_with = "de_null")]
    pub zone: ZoneCfg,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct WeaponObs {
    #[serde(deserialize_with = "de_null")]
    pub kind: String,
    pub cooldown: f64,
    pub ammo: f64,
    pub reload: f64,
    pub charge: f64,
    #[serde(deserialize_with = "de_null")]
    pub stats: Option<Stats>,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct SelfObs {
    #[serde(deserialize_with = "de_int")]
    pub id: i64,
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub aim: [f64; 2],
    pub hp: f64,
    pub max_hp: f64,
    pub energy: f64,
    pub max_energy: f64,
    pub grounded: bool,
    #[serde(deserialize_with = "de_int")]
    pub cubes: i64,
    #[serde(deserialize_with = "de_null")]
    pub upgrades: Vec<String>,
    pub in_zone: bool,
    #[serde(deserialize_with = "de_null")]
    pub weapons: Vec<Option<WeaponObs>>,
    #[serde(deserialize_with = "de_null")]
    pub stats: Option<Stats>,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct ActorObs {
    #[serde(deserialize_with = "de_int")]
    pub id: i64,
    #[serde(deserialize_with = "de_null")]
    pub kind: String,
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub aim: [f64; 2],
    pub hp: f64,
    pub max_hp: f64,
    #[serde(deserialize_with = "de_int")]
    pub cubes: i64,
    #[serde(deserialize_with = "de_null")]
    pub weapons: Vec<String>,
    pub charging: bool,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct ProjectileObs {
    #[serde(deserialize_with = "de_int")]
    pub id: i64,
    #[serde(deserialize_with = "de_null")]
    pub kind: String,
    pub pos: [f64; 3],
    pub vel: [f64; 3],
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct ItemObs {
    #[serde(deserialize_with = "de_int")]
    pub id: i64,
    pub pos: [f64; 3],
    pub hp: f64,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct PickupObs {
    #[serde(deserialize_with = "de_int")]
    pub id: i64,
    #[serde(deserialize_with = "de_null")]
    pub kind: String,
    pub pos: [f64; 3],
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct SoundObs {
    #[serde(deserialize_with = "de_null")]
    pub kind: String,
    pub bearing: f64,
    #[serde(deserialize_with = "de_null")]
    pub dist: String,
    pub age: f64,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct BeamObs {
    #[serde(deserialize_with = "de_int")]
    pub by: i64,
    #[serde(deserialize_with = "de_null")]
    pub kind: String,
    #[serde(deserialize_with = "de_null")]
    pub path: Vec<[f64; 3]>,
    pub age: f64,
}

/// An event id is a number for most events and a string for "upgrade".
#[derive(Default, Clone, Debug)]
pub struct IdValue {
    pub num: i64,
    pub str: String,
}

impl<'de> Deserialize<'de> for IdValue {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let v = serde_json::Value::deserialize(d)?;
        Ok(match v {
            serde_json::Value::String(s) => IdValue { num: 0, str: s },
            serde_json::Value::Number(n) => IdValue { num: n.as_f64().unwrap_or(0.0) as i64, str: String::new() },
            _ => IdValue::default(),
        })
    }
}

/// The union of every event type; unused fields stay zero / None.
#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct Event {
    #[serde(rename = "type", deserialize_with = "de_null")]
    pub typ: String,
    pub amount: f64,
    #[serde(deserialize_with = "de_null")]
    pub weapon: String,
    pub bearing: Option<f64>,
    #[serde(deserialize_with = "de_opt_int")]
    pub by: Option<i64>,
    pub rear: bool,
    #[serde(deserialize_with = "de_int")]
    pub target: i64,
    #[serde(deserialize_with = "de_null")]
    pub effect: String,
    pub id: IdValue,
    #[serde(deserialize_with = "de_int")]
    pub cubes: i64,
    #[serde(deserialize_with = "de_null")]
    pub kind: String,
    #[serde(deserialize_with = "de_int")]
    pub slot: i64,
    #[serde(deserialize_with = "de_null")]
    pub reason: String,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct DropObs {
    #[serde(deserialize_with = "de_int")]
    pub id: i64,
    pub pos: [f64; 3],
    pub radius: f64,
    pub lands_in: f64,
    pub capture_s: f64,
    pub progress: f64,
    #[serde(deserialize_with = "de_opt_int")]
    pub holder: Option<i64>,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct ZoneObs {
    pub center: [f64; 2],
    pub radius: f64,
    pub next_center: [f64; 2],
    pub next_radius: f64,
    pub shrink_start: f64,
    pub shrink_end: f64,
    pub dps: f64,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct OfferChoice {
    #[serde(deserialize_with = "de_null")]
    pub id: String,
    #[serde(deserialize_with = "de_null")]
    pub name: String,
    #[serde(deserialize_with = "de_null")]
    pub desc: String,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct Offer {
    #[serde(deserialize_with = "de_int")]
    pub id: i64,
    pub expires: f64,
    #[serde(deserialize_with = "de_null")]
    pub choices: Vec<OfferChoice>,
}

#[derive(Deserialize, Default, Clone, Debug)]
#[serde(default)]
pub struct Obs {
    #[serde(rename = "type", deserialize_with = "de_null")]
    pub typ: String,
    #[serde(deserialize_with = "de_int")]
    pub tick: i64,
    pub t: f64,
    #[serde(deserialize_with = "de_int")]
    pub alive: i64,
    #[serde(rename = "self", deserialize_with = "de_null")]
    pub self_: SelfObs,
    #[serde(deserialize_with = "de_null")]
    pub mechs: Vec<ActorObs>,
    #[serde(deserialize_with = "de_null")]
    pub projectiles: Vec<ProjectileObs>,
    #[serde(deserialize_with = "de_null")]
    pub crates: Vec<ItemObs>,
    #[serde(deserialize_with = "de_null")]
    pub cubes: Vec<ItemObs>,
    #[serde(deserialize_with = "de_null")]
    pub pickups: Vec<PickupObs>,
    #[serde(deserialize_with = "de_null")]
    pub sounds: Vec<SoundObs>,
    #[serde(deserialize_with = "de_null")]
    pub beams: Vec<BeamObs>,
    #[serde(deserialize_with = "de_null")]
    pub events: Vec<Event>,
    #[serde(deserialize_with = "de_null")]
    pub drops: Vec<DropObs>,
    #[serde(deserialize_with = "de_null")]
    pub zone: ZoneObs,
    pub offer: Option<Offer>,
}

/// One decision. pick/take/drop use -1 for "not set".
#[derive(Clone, Debug, Default)]
pub struct Action {
    pub tick: i64,
    pub mv: [f64; 2],
    pub boost: bool,
    pub jump: bool,
    pub aim: [f64; 2],
    pub has_aim: bool,
    pub fire: [bool; 2],
    pub reload: bool,
    pub pick: i64,
    pub take: i64,
    pub drop: i64,
    pub label: String,
}

impl Action {
    pub fn new(tick: i64) -> Action {
        Action { tick, pick: -1, take: -1, drop: -1, ..Default::default() }
    }

    /// Serialises the action as one JSON line (without the trailing newline).
    pub fn append_json(&self, b: &mut String) {
        use std::fmt::Write;
        fn f(b: &mut String, v: f64, prec: usize) {
            if v.is_nan() || v > 1e15 || v < -1e15 {
                b.push('0');
            } else {
                let _ = write!(b, "{:.*}", prec, v);
            }
        }
        let bo = |v: bool| if v { "true" } else { "false" };
        let _ = write!(b, "{{\"tick\":{},\"move\":[", self.tick);
        f(b, self.mv[0], 4);
        b.push(',');
        f(b, self.mv[1], 4);
        let _ = write!(b, "],\"boost\":{},\"jump\":{}", bo(self.boost), bo(self.jump));
        if self.has_aim {
            b.push_str(",\"aim\":[");
            f(b, self.aim[0], 5);
            b.push(',');
            f(b, self.aim[1], 5);
            b.push(']');
        }
        let _ = write!(b, ",\"fire\":[{},{}]", bo(self.fire[0]), bo(self.fire[1]));
        if self.reload {
            b.push_str(",\"reload\":true");
        }
        if self.pick >= 0 {
            let _ = write!(b, ",\"pick\":{}", self.pick);
        }
        if self.take >= 0 {
            let _ = write!(b, ",\"take\":{}", self.take);
        }
        if self.drop >= 0 {
            let _ = write!(b, ",\"drop\":{}", self.drop);
        }
        if !self.label.is_empty() {
            let lab: String = self.label.bytes().take(70).map(|c| c as char).collect();
            b.push_str(",\"debug\":{\"label\":");
            b.push_str(&serde_json::to_string(&lab).unwrap_or_else(|_| "\"\"".into()));
            b.push('}');
        }
        b.push('}');
    }
}
