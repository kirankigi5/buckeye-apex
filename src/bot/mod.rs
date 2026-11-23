//! Our Mech Royale agent: memory of what it has seen, a nav/dodge movement planner, ballistics for
//! every weapon, and a survival-first strategy layer on top.

mod combat;
mod cover;
pub mod gorand;
mod hide;
mod hidemap;
mod lead;
mod movement;
pub mod params;
mod peek;
mod plan;
mod policy;
mod roof;
mod strategy;
pub mod variants;

use crate::proto::{self, Action, Hello, Obs, Stats};
use crate::world::{Nav, World};
use gorand::GoRand;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::rc::Rc;

pub use params::Params;

use hidemap::{HideMap, Hideout};
use peek::PeekPlan;
use roof::RoofPlan;
use strategy::Stance;

// ------------------------------------------------------------------ small helpers

#[inline]
pub(crate) fn clamp(v: f64, lo: f64, hi: f64) -> f64 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

#[inline]
pub(crate) fn hyp(x: f64, y: f64) -> f64 {
    (x * x + y * y).sqrt()
}

#[inline]
pub(crate) fn dist2(a: [f64; 3], b: [f64; 3]) -> f64 {
    hyp(a[0] - b[0], a[1] - b[1])
}

#[inline]
pub(crate) fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let (dx, dy, dz) = (a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// a-b wrapped to (-pi, pi]. (Go's math.Mod keeps the sign of the dividend, like Rust's %.)
#[inline]
pub(crate) fn ang_diff(a: f64, b: f64) -> f64 {
    use std::f64::consts::PI;
    let mut d = (a - b) % (2.0 * PI);
    if d > PI {
        d -= 2.0 * PI;
    } else if d <= -PI {
        d += 2.0 * PI;
    }
    d
}

#[inline]
pub(crate) fn norm2(x: f64, y: f64) -> (f64, f64) {
    let l = hyp(x, y);
    if l < 1e-9 {
        (0.0, 0.0)
    } else {
        (x / l, y / l)
    }
}

#[inline]
pub(crate) fn stat(m: Option<&Stats>, key: &str, def: f64) -> f64 {
    m.and_then(|m| m.get(key).copied()).unwrap_or(def)
}

/// moves cur toward target by at most max_delta (vector form).
#[inline]
pub(crate) fn approach2(cx: f64, cy: f64, tx: f64, ty: f64, max_delta: f64) -> (f64, f64) {
    let (dx, dy) = (tx - cx, ty - cy);
    let d = hyp(dx, dy);
    if d <= max_delta || d < 1e-12 {
        return (tx, ty);
    }
    (cx + dx / d * max_delta, cy + dy / d * max_delta)
}

/// Distance from point p to segment a-b and the parameter of the closest point.
#[inline]
pub(crate) fn seg_point_dist_2d(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> (f64, f64) {
    let (ex, ey) = (bx - ax, by - ay);
    let l2 = ex * ex + ey * ey;
    let mut t = 0.0;
    if l2 > 1e-12 {
        t = clamp(((px - ax) * ex + (py - ay) * ey) / l2, 0.0, 1.0);
    }
    let (cx, cy) = (ax + ex * t, ay + ey * t);
    (hyp(px - cx, py - cy), t)
}

// ------------------------------------------------------------------ memory

pub(crate) const TRK_HIST: usize = 40; // four seconds at ten observations a second

#[derive(Clone, Copy, Default, Debug)]
pub(crate) struct TrkSample {
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub vx: f64,
    pub vy: f64,
    pub facing: bool,
}

/// What we remember about another mech or drone.
#[derive(Clone, Debug)]
pub struct Track {
    pub id: i64,
    pub npc: bool,
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub aim: [f64; 2],
    pub hp: f64,
    pub max_hp: f64,
    pub cubes: i64,
    pub laser: bool,
    pub mortar: bool,
    pub light: bool,
    pub gun: bool,
    pub charging: bool,
    pub last_seen: f64,
    pub listed: bool,
    pub los: bool,
    pub los_t: f64,
    pub hurt_me_t: f64,
    pub hurt_me: f64,
    pub i_hurt_t: f64,
    pub i_hurt: f64,
    pub aim_at_me_t: f64,
    pub seen_t0: f64,
    pub phantom_t: f64,
    pub was_listed: bool,
    pub view_t: f64,
    pub eng_t: f64,
    pub laser_fired_t: f64,

    pub(crate) hist: [TrkSample; TRK_HIST],
    pub(crate) hn: usize,
    pub(crate) hi: usize,
    pub(crate) lead_n: f64,
    pub(crate) miss_none: f64,
    pub(crate) miss_lead: f64,
}

impl Track {
    fn new(id: i64, npc: bool, t: f64) -> Track {
        Track {
            id,
            npc,
            pos: [0.0; 3],
            vel: [0.0; 3],
            aim: [0.0; 2],
            hp: 0.0,
            max_hp: 0.0,
            cubes: 0,
            laser: false,
            mortar: false,
            light: false,
            gun: false,
            charging: false,
            last_seen: 0.0,
            listed: false,
            los: false,
            los_t: -100.0,
            hurt_me_t: -100.0,
            hurt_me: 0.0,
            i_hurt_t: -100.0,
            i_hurt: 0.0,
            aim_at_me_t: -100.0,
            seen_t0: t,
            phantom_t: -100.0,
            was_listed: false,
            view_t: -100.0,
            eng_t: -100.0,
            laser_fired_t: 0.0,
            hist: [TrkSample::default(); TRK_HIST],
            hn: 0,
            hi: 0,
            lead_n: 0.0,
            miss_none: 0.0,
            miss_lead: 0.0,
        }
    }

    #[inline]
    pub(crate) fn scale(&self, size_per_cube: f64) -> f64 {
        1.0 + size_per_cube * self.cubes as f64
    }
}

#[derive(Clone, Debug, Default)]
pub struct Item {
    pub id: i64,
    pub pos: [f64; 3],
    pub hp: f64,
    pub kind: String,
    pub last_seen: f64,
    pub ground: bool,
    pub cell: usize,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct Slot {
    pub ok: bool,
    pub kind: String,
    pub cooldown: f64,
    pub ammo: f64,
    pub reload: f64,
    pub charge: f64,
    pub stats: Option<Stats>,
}

#[derive(Clone, Copy, Default)]
pub(crate) struct Cand {
    pub dx: f64,
    pub dy: f64,
    pub boost: bool,
    pub score: f64,
}

pub struct Bot {
    pub p: Params,
    pub h: Hello,
    pub w: Rc<World>,
    pub nav: Rc<Nav>,
    rng: GoRand,

    dt: f64,
    decide: f64,
    size_per_cube: f64,

    // current observation
    o: Obs,
    t: f64,
    pos: [f64; 3],
    vel: [f64; 3],
    aim: [f64; 2],
    hp: f64,
    max_hp: f64,
    energy: f64,
    scale: f64,
    radius: f64,
    height: f64,
    eye_h: f64,
    muzzle_h: f64,
    walk: f64,
    boost_v: f64,
    turn: f64,
    pitch_r: f64,
    armor: f64,
    slots: [Slot; 2],

    // memory
    tracks: BTreeMap<i64, Track>,
    crates: BTreeMap<i64, Item>,
    cubes: BTreeMap<i64, Item>,
    pickups: BTreeMap<i64, Item>,

    last_damage_t: f64,
    last_weapon_dt: f64,
    last_enemy_los_t: f64,
    last_dealt_t: f64,
    recent_dmg: f64,
    last_dmg_bear: f64,
    last_dmg_by: i64,
    last_sound_t: f64,
    last_sound_b: f64,

    // movement state
    path: Vec<i32>,
    path_bad: bool,
    path_set: bool,
    path_goal: [f64; 2],
    path_t: f64,
    path_idx: usize,
    strafe_sign: f64,
    strafe_t: f64,
    stuck_t: f64,
    stuck_pos: [f64; 3],
    stuck_for: f64,
    unstick_t: f64,
    unstick_dir: [f64; 2],
    last_move: [f64; 2],

    // strategy state
    mode: String,
    target_id: i64,
    retreating: bool,
    cover_cell: i64,
    cover_t: f64,
    deep_cell: i64,
    deep_t: f64,
    deep_try_t: f64,
    stand_t: f64,
    credit_absorb_t: f64,
    near_cover_t: f64,
    near_cover_ok: bool,
    loot_id: i64,
    hold_cell: i64,
    last_aim_cmd: [f64; 2],
    scan_phase: f64,
    laser_hold: bool,
    take_t: f64,
    ffa_seen_t: f64,
    ffa_beams: i64,
    kills: i64,
    kills_seen: i64,
    dead_seen: i64,
    unexplained: i64,
    ffa: bool,
    st: Stance,
    evade_cell: i64,
    evade_t: f64,
    hide_t: f64,
    last_label: String,
    mode_label: String,
    last_trace_t: f64,
    last_npc_dt: f64,
    late: bool,
    seen_t: Vec<f32>,
    seen_nx: i64,
    seen_ny: i64,
    roam_cell: i64,
    roam_t: f64,
    retreat_t: f64,
    cornered_t: f64,
    zg_key: [f64; 3],
    zg_t: f64,
    zg_comp: i32,
    zg_goal: [f64; 2],
    zg_walk: bool,
    my_comp: i32,
    main_comp_id: i32,
    surf: Vec<i32>,
    surf_goal: [f64; 2],
    surf_t: f64,
    surf_idx: usize,
    surf_ban: HashMap<usize, f64>,
    surf_watch_t: f64,
    surf_watch_pos: [f64; 3],
    surf_pushing: bool,
    ban_until: HashMap<i64, f64>,
    seek_ban: HashMap<i64, f64>,
    roof: RoofPlan,
    roof_cover_c: i64,
    roof_cover_t: f64,
    unquiet_t: f64,
    peek: PeekPlan,
    roof_cover_hold_t: f64,
    hold_climb: bool,
    hold_roof_move: bool,
    jink_sign: f64,
    jink_t: f64,
    edge_on_t: f64,
    slip_cell: i64,
    slip_t: f64,
    slip_on_t: f64,
    hm: Option<&'static HideMap>,
    hide_cell: i64,
    hide_out: Hideout,
    hide_out_ok: bool,
    hide_out_t: f64,
    roof_first_ok: bool,
    roof_first_checked: bool,
    credit_t: f64,
    credit_id: i64,

    scratch: Vec<Cand>,
}

fn env_set(name: &str) -> bool {
    std::env::var(name).map(|v| !v.is_empty()).unwrap_or(false)
}

impl Bot {
    pub fn new(h: Hello, w: Rc<World>, nav: Rc<Nav>, p: Params) -> Bot {
        // (Salt: builds that differ only in it play the same game with different dice.)
        let seed = h.seed.wrapping_mul(7919).wrapping_add(h.slot).wrapping_add(1).wrapping_add(p.salt.wrapping_mul(104729));
        let mut hz = stat(Some(&h.config.match_), "tick_hz", 20.0);
        if hz <= 0.0 {
            hz = 20.0;
        }
        let dt = 1.0 / hz;
        let decide = stat(Some(&h.config.match_), "decide_every", 2.0) * dt;
        let size_per_cube = h.config.cubes.size_per_cube;
        let use_hm = p.use_hide_map || p.hold_hide_map;
        let mut b = Bot {
            p,
            h,
            w,
            nav: nav.clone(),
            rng: GoRand::new(seed),
            dt,
            decide,
            size_per_cube,
            o: Obs::default(),
            t: 0.0,
            pos: [0.0; 3],
            vel: [0.0; 3],
            aim: [0.0; 2],
            hp: 0.0,
            max_hp: 1.0,
            energy: 0.0,
            scale: 1.0,
            radius: 2.5,
            height: 9.0,
            eye_h: 8.0,
            muzzle_h: 7.0,
            walk: 11.7,
            boost_v: 19.5,
            turn: 4.0,
            pitch_r: 3.0,
            armor: 0.0,
            slots: [Slot::default(), Slot::default()],
            tracks: BTreeMap::new(),
            crates: BTreeMap::new(),
            cubes: BTreeMap::new(),
            pickups: BTreeMap::new(),
            last_damage_t: -100.0,
            last_weapon_dt: -100.0,
            last_enemy_los_t: -100.0,
            last_dealt_t: -100.0,
            recent_dmg: 0.0,
            last_dmg_bear: 0.0,
            last_dmg_by: -1,
            last_sound_t: -100.0,
            last_sound_b: 0.0,
            path: Vec::new(),
            path_bad: false,
            path_set: false,
            path_goal: [0.0; 2],
            path_t: 0.0,
            path_idx: 0,
            strafe_sign: 1.0,
            strafe_t: 0.0,
            stuck_t: 0.0,
            stuck_pos: [0.0; 3],
            stuck_for: 0.0,
            unstick_t: 0.0,
            unstick_dir: [0.0; 2],
            last_move: [0.0; 2],
            mode: "init".into(),
            target_id: -1,
            retreating: false,
            cover_cell: -1,
            cover_t: -100.0,
            deep_cell: -1,
            deep_t: -100.0,
            deep_try_t: -100.0,
            stand_t: -100.0,
            credit_absorb_t: -100.0,
            near_cover_t: -100.0,
            near_cover_ok: false,
            loot_id: -1,
            hold_cell: -1,
            last_aim_cmd: [0.0; 2],
            scan_phase: 0.0,
            laser_hold: false,
            take_t: 0.0,
            ffa_seen_t: -100.0,
            ffa_beams: 0,
            kills: 0,
            kills_seen: 0,
            dead_seen: 0,
            unexplained: 0,
            ffa: false,
            st: Stance::default(),
            evade_cell: -1,
            evade_t: -100.0,
            hide_t: 0.0,
            last_label: String::new(),
            mode_label: String::new(),
            last_trace_t: 0.0,
            last_npc_dt: -100.0,
            late: false,
            seen_t: Vec::new(),
            seen_nx: 0,
            seen_ny: 0,
            roam_cell: -1,
            roam_t: 0.0,
            retreat_t: 0.0,
            cornered_t: -100.0,
            zg_key: [0.0; 3],
            zg_t: 0.0,
            zg_comp: 0,
            zg_goal: [0.0; 2],
            zg_walk: false,
            my_comp: 0,
            main_comp_id: 0,
            surf: Vec::new(),
            surf_goal: [0.0; 2],
            surf_t: -100.0,
            surf_idx: 0,
            surf_ban: HashMap::new(),
            surf_watch_t: 0.0,
            surf_watch_pos: [0.0; 3],
            surf_pushing: false,
            ban_until: HashMap::new(),
            seek_ban: HashMap::new(),
            roof: RoofPlan::new(),
            roof_cover_c: -1,
            roof_cover_t: -100.0,
            unquiet_t: -100.0,
            peek: PeekPlan { t: -100.0, tgt: -1, roof: -1, duck: -1, fire: -1, ok: false },
            roof_cover_hold_t: -100.0,
            hold_climb: false,
            hold_roof_move: false,
            jink_sign: 1.0,
            jink_t: -100.0,
            edge_on_t: -100.0,
            slip_cell: -1,
            slip_t: -100.0,
            slip_on_t: -100.0,
            hm: None,
            hide_cell: -1,
            hide_out: Hideout::default(),
            hide_out_ok: false,
            hide_out_t: -100.0,
            roof_first_ok: true,
            roof_first_checked: false,
            credit_t: -100.0,
            credit_id: 0,
            scratch: Vec::new(),
        };
        if use_hm {
            b.hm = hidemap::get_hide_map(&nav);
        }
        // build the shared maps now (inside the hello budget), never during a decision
        nav.exposure();
        nav.components();
        nav.roofs();
        nav.tops();
        b
    }

    // ---- config shortcuts

    #[inline]
    fn mech(&self, key: &str, def: f64) -> f64 {
        stat(Some(&self.h.config.mech), key, def)
    }

    #[inline]
    fn wstat(&self, weapon: &str, key: &str, def: f64) -> f64 {
        stat(self.h.config.weapons.get(weapon), key, def)
    }

    #[inline]
    fn npc_stat(&self, key: &str, def: f64) -> f64 {
        stat(Some(&self.h.config.npc), key, def)
    }

    #[inline]
    fn self_stat(&self, key: &str, def: f64) -> f64 {
        stat(self.o.self_.stats.as_ref(), key, def)
    }

    #[inline]
    fn slot_stat(&self, i: usize, key: &str, def: f64) -> f64 {
        stat(self.slots[i].stats.as_ref(), key, def)
    }

    fn trace(&mut self, act: &Action) {
        if self.mode_label == self.last_label && self.t - self.last_trace_t < 5.0 {
            return;
        }
        self.last_label = self.mode_label.clone();
        self.last_trace_t = self.t;
        let mut tg = String::new();
        if let Some(tr) = self.tracks.get(&self.target_id) {
            tg = format!(" tgt={} d={:.0} los={} hp={:.0}", tr.id, dist3(self.pos, tr.pos), tr.los, tr.hp);
        }
        let mut w = String::new();
        for s in &self.slots {
            if s.ok {
                w += &s.kind.chars().take(3).collect::<String>();
                w.push(' ');
            }
        }
        eprintln!(
            "[{}] t={:6.1} alive={:2} hp={:4.0}/{:4.0} en={:3.0} cubes={:2} pos=({:.0},{:.0},{:.0}) {:<18} w={} proj={} mv=({:+.1},{:+.1}){}{}",
            self.h.slot,
            self.t,
            self.o.alive,
            self.hp,
            self.max_hp,
            self.energy,
            self.o.self_.cubes,
            self.pos[0],
            self.pos[1],
            self.pos[2],
            self.mode_label,
            w,
            self.o.projectiles.len(),
            act.mv[0],
            act.mv[1],
            if act.boost { " BOOST" } else { "" },
            tg
        );
    }

    /// One decision.
    pub fn act(&mut self, o: Obs) -> Action {
        let mut act = Action::new(o.tick);
        self.observe(o);
        self.think(&mut act);
        if env_set("MECH_TRACE") {
            self.trace(&act);
        }
        // never emit garbage
        if act.mv[0].is_nan() || act.mv[1].is_nan() {
            act.mv = [0.0, 0.0];
        }
        if act.aim[0].is_nan() || act.aim[1].is_nan() {
            act.has_aim = false;
        }
        self.last_move = act.mv;
        if act.has_aim {
            self.last_aim_cmd = act.aim;
        }
        act
    }

    /// Folds the observation into memory.
    fn observe(&mut self, o: Obs) {
        self.o = o;
        self.t = self.o.t;
        let s = self.o.self_.clone();
        self.pos = s.pos;
        self.vel = s.vel;
        self.aim = s.aim;
        self.hp = s.hp;
        self.max_hp = s.max_hp;
        self.energy = s.energy;
        if self.max_hp <= 0.0 {
            self.max_hp = 1.0;
        }
        let st = s.stats.as_ref();
        let base_muzzle = self.mech("muzzle_height", 7.0);
        self.scale = 1.0 + self.size_per_cube * s.cubes as f64;
        self.muzzle_h = stat(st, "muzzle_height", base_muzzle * self.scale);
        self.eye_h = stat(st, "eye_height", self.mech("eye_height", 8.0) * self.scale);
        // stats may or may not carry the size-scaled numbers: trust whichever is larger
        if self.muzzle_h < base_muzzle * self.scale - 1e-6 {
            self.muzzle_h = base_muzzle * self.scale;
        }
        let e = self.mech("eye_height", 8.0) * self.scale;
        if self.eye_h < e - 1e-6 {
            self.eye_h = e;
        }
        self.radius = stat(st, "radius", 0.0).max(self.mech("radius", 2.5) * self.scale);
        self.height = stat(st, "height", 0.0).max(self.mech("height", 9.0) * self.scale);
        self.walk = stat(st, "walk_speed", self.mech("walk_speed", 11.7));
        self.boost_v = stat(st, "boost_speed", self.mech("boost_speed", 19.5));
        self.turn = stat(st, "turn_rate", self.mech("turn_rate", 4.0));
        self.pitch_r = stat(st, "pitch_rate", self.mech("pitch_rate", 3.0));
        self.armor = stat(st, "armor", 0.0);
        for i in 0..2 {
            self.slots[i] = Slot::default();
            if let Some(Some(w)) = s.weapons.get(i) {
                let stats = w.stats.clone().or_else(|| self.h.config.weapons.get(&w.kind).cloned());
                self.slots[i] = Slot { ok: true, kind: w.kind.clone(), cooldown: w.cooldown, ammo: w.ammo, reload: w.reload, charge: w.charge, stats };
            }
        }

        // decay
        self.recent_dmg *= (-self.decide / 3.0).exp();
        let dk = (-self.decide / 4.0).exp();
        for tr in self.tracks.values_mut() {
            tr.was_listed = tr.listed;
            tr.listed = false;
            tr.los = false;
            tr.hurt_me *= dk;
            tr.i_hurt *= dk;
        }
        // which walkable region are we in (the stadium floor, for one, is cut off from the rest)
        let c = self.nav.nearest(self.pos[0], self.pos[1], 2.9, 9.0, 4);
        if c >= 0 && self.on_ground_level() {
            self.my_comp = self.nav.components()[c as usize];
        }

        let eye = [self.pos[0], self.pos[1], self.pos[2] + self.eye_h];
        let mut fov_half = self.h.config.vision.fov_deg * std::f64::consts::PI / 360.0;
        if fov_half <= 0.0 {
            fov_half = std::f64::consts::PI / 3.0;
        }
        let vis_range = self.h.config.vision.range + stat(st, "vision_bonus", 0.0);
        let near_range = self.h.config.vision.near_range + stat(st, "near_range_bonus", 0.0);
        let mechs = std::mem::take(&mut self.o.mechs);
        for m in &mechs {
            let t = self.t;
            let mut tr = self.tracks.remove(&m.id).unwrap_or_else(|| Track::new(m.id, m.kind == "npc", t));
            if self.t - tr.last_seen > 6.0 {
                tr.seen_t0 = self.t;
            }
            tr.pos = m.pos;
            tr.vel = m.vel;
            tr.aim = m.aim;
            if m.max_hp > 0.0 {
                tr.hp = m.hp;
                tr.max_hp = m.max_hp;
            } else if tr.max_hp == 0.0 {
                tr.hp = 1400.0;
                tr.max_hp = 1400.0;
            }
            tr.cubes = m.cubes;
            tr.charging = m.charging;
            tr.last_seen = self.t;
            tr.listed = true;
            if !tr.npc {
                tr.gun = false;
                tr.laser = false;
                tr.mortar = false;
                tr.light = false;
                for w in &m.weapons {
                    match w.as_str() {
                        "gun" => tr.gun = true,
                        "laser" => tr.laser = true,
                        "mortar" => tr.mortar = true,
                        "lightning" => tr.light = true,
                        _ => {}
                    }
                }
            }
            let sc = if tr.npc { 1.0 } else { tr.scale(self.size_per_cube) };
            let ch = if tr.npc { self.npc_stat("height", 3.5) * 0.5 } else { 4.5 * sc };
            // the server's rule: we see it when our eye has a line to its centre or to its top
            tr.los = self.w.visible(eye[0], eye[1], eye[2], m.pos[0], m.pos[1], m.pos[2] + ch)
                || self.w.visible(eye[0], eye[1], eye[2], m.pos[0], m.pos[1], m.pos[2] + ch * 1.9);
            if tr.los {
                tr.los_t = self.t;
                if !tr.npc {
                    self.last_enemy_los_t = self.t;
                }
                let d3 = dist3(eye, [m.pos[0], m.pos[1], m.pos[2] + ch]);
                if d3 < near_range - 1.0
                    || (d3 < vis_range - 2.0 && ang_diff((m.pos[1] - self.pos[1]).atan2(m.pos[0] - self.pos[0]), self.aim[0]).abs() < fov_half - 0.05)
                {
                    tr.view_t = self.t;
                }
            }
            if !tr.npc && tr.los {
                // is its torso pointed at us?
                let bear = (self.pos[1] - m.pos[1]).atan2(self.pos[0] - m.pos[0]);
                let d = dist2(self.pos, m.pos);
                let tol = 0.12 + (self.radius * 1.5).atan2(d.max(1.0));
                let off = ang_diff(m.aim[0], bear).abs();
                if off < tol {
                    tr.aim_at_me_t = self.t;
                }
                if self.p.lead_persist || self.p.lead_cum {
                    self.lead_record(&mut tr, off < 0.35);
                }
            }
            self.tracks.insert(m.id, tr);
        }
        self.o.mechs = mechs;
        // forget stale tracks (dead or far gone)
        let mut mem = self.h.config.vision.memory_s;
        if mem <= 0.0 {
            mem = 5.0;
        }
        let ids: Vec<i64> = self.tracks.keys().copied().collect();
        for id in ids {
            let tr = &self.tracks[&id];
            if self.t - tr.last_seen > 45.0 && self.t - tr.hurt_me_t > 45.0 {
                self.tracks.remove(&id);
                continue;
            }
            // one that drops off the list while we were looking straight at it a moment ago is dead
            if tr.was_listed && !tr.listed && self.t - tr.view_t < mem - 1.5 {
                self.tracks.remove(&id);
                if self.target_id == id {
                    self.target_id = -1;
                }
            }
        }

        // events
        let events = std::mem::take(&mut self.o.events);
        for e in &events {
            match e.typ.as_str() {
                "damaged" => {
                    self.last_damage_t = self.t;
                    if e.weapon == "npc" {
                        self.last_npc_dt = self.t;
                        if let Some(b) = e.bearing {
                            self.last_dmg_bear = b;
                        }
                        if let Some(by) = e.by {
                            if let Some(tr) = self.tracks.get_mut(&by) {
                                tr.hurt_me_t = self.t;
                            }
                        }
                    } else if e.weapon != "zone" {
                        self.last_weapon_dt = self.t;
                        self.recent_dmg += e.amount;
                        if let Some(b) = e.bearing {
                            self.last_dmg_bear = b;
                        }
                        if let Some(by) = e.by {
                            if by != s.id {
                                self.last_dmg_by = by;
                                let t = self.t;
                                let tr = self.tracks.entry(by).or_insert_with(|| {
                                    let mut tr = Track::new(by, by >= 100, t);
                                    tr.last_seen = -100.0;
                                    tr.hp = 1400.0;
                                    tr.max_hp = 1400.0;
                                    tr
                                });
                                if !tr.listed && t - tr.last_seen > 2.0 {
                                    if let Some(bear) = e.bearing {
                                        // shot by something we cannot see: place it along the bearing
                                        let guess = match e.weapon.as_str() {
                                            "laser" => 150.0,
                                            "lightning" => 35.0,
                                            _ => 115.0,
                                        };
                                        tr.pos = [self.pos[0] + bear.cos() * guess, self.pos[1] + bear.sin() * guess, self.pos[2]];
                                        tr.vel = [0.0; 3];
                                        tr.phantom_t = t;
                                    }
                                }
                                tr.hurt_me_t = t;
                                tr.hurt_me += e.amount;
                                match e.weapon.as_str() {
                                    "laser" => tr.laser = true,
                                    "mortar" => tr.mortar = true,
                                    "lightning" => tr.light = true,
                                    _ => {}
                                }
                            }
                        }
                    }
                }
                "hit" => {
                    self.last_dealt_t = self.t;
                    if e.target < 100 {
                        self.credit_t = self.t;
                        self.credit_id = e.target;
                    }
                    if let Some(tr) = self.tracks.get_mut(&e.target) {
                        tr.i_hurt_t = self.t;
                        tr.i_hurt += e.amount;
                    }
                }
                "kill" => {
                    self.kills += 1;
                    self.tracks.remove(&e.target);
                    if self.target_id == e.target {
                        self.target_id = -1;
                    }
                }
                "took" => {
                    self.pickups.remove(&e.id.num);
                }
                "take_failed" => {
                    if e.reason == "gone" || e.reason == "have" {
                        self.pickups.remove(&e.id.num);
                    }
                }
                _ => {}
            }
        }
        self.o.events = events;

        // items
        let crates = std::mem::take(&mut self.o.crates);
        let mut m = std::mem::take(&mut self.crates);
        self.fold_items(&crates, &mut m, "crate");
        self.crates = m;
        self.o.crates = crates;
        let cubes = std::mem::take(&mut self.o.cubes);
        let mut m = std::mem::take(&mut self.cubes);
        self.fold_items(&cubes, &mut m, "cube");
        self.cubes = m;
        self.o.cubes = cubes;
        let mut seen = HashSet::new();
        let pickups = std::mem::take(&mut self.o.pickups);
        let mut pm = std::mem::take(&mut self.pickups);
        for p in &pickups {
            seen.insert(p.id);
            let it = pm.entry(p.id).or_insert_with(|| Item { id: p.id, ..Default::default() });
            it.pos = p.pos;
            it.kind = p.kind.clone();
            it.last_seen = self.t;
            self.ground_check(it);
        }
        self.expire_unseen(&mut pm, &seen, eye);
        self.pickups = pm;
        self.o.pickups = pickups;

        // sounds: remember the freshest loud one for looking around
        for sd in &self.o.sounds {
            match sd.kind.as_str() {
                "gun" | "laser" | "lightning" | "mortar" | "laser_charge" | "boost" | "step" | "jet" | "reload" => {
                    if sd.dist != "far" {
                        self.last_sound_t = self.t - sd.age;
                        self.last_sound_b = sd.bearing;
                    }
                }
                _ => {}
            }
        }
        // evidence about the lobby: beams fired by others whose path stays far from us
        let mut saw_ffa_beam = false;
        for bm in &self.o.beams {
            if bm.by == s.id {
                continue;
            }
            if self.p.laser_cd_track && bm.kind == "laser" {
                if let Some(tr) = self.tracks.get_mut(&bm.by) {
                    tr.laser_fired_t = self.t - bm.age;
                }
            }
            if bm.path.len() >= 2 {
                let p0 = bm.path[0];
                let end = bm.path[bm.path.len() - 1];
                let (seg_d, _) = seg_point_dist_2d(self.pos[0], self.pos[1], p0[0], p0[1], end[0], end[1]);
                if dist2(end, self.pos) > 25.0 && seg_d > 22.0 {
                    self.ffa_seen_t = self.t;
                    saw_ffa_beam = true;
                }
            }
        }
        if saw_ffa_beam {
            self.ffa_beams += 1;
        }
    }

    fn fold_items(&self, list: &[proto::ItemObs], m: &mut BTreeMap<i64, Item>, kind: &str) {
        let eye = [self.pos[0], self.pos[1], self.pos[2] + self.eye_h];
        let mut seen = HashSet::new();
        for p in list {
            seen.insert(p.id);
            let it = m.entry(p.id).or_insert_with(|| Item { id: p.id, kind: kind.to_string(), ..Default::default() });
            it.pos = p.pos;
            it.hp = p.hp;
            it.last_seen = self.t;
            self.ground_check(it);
        }
        self.expire_unseen(m, &seen, eye);
    }

    /// Drops remembered items that we should be seeing right now but are not listed.
    fn expire_unseen(&self, m: &mut BTreeMap<i64, Item>, seen: &HashSet<i64>, eye: [f64; 3]) {
        let near = self.h.config.vision.near_range * 0.85;
        m.retain(|id, it| {
            if seen.contains(id) {
                return true;
            }
            if self.t - it.last_seen > 240.0 {
                return false;
            }
            let d = dist3(it.pos, eye);
            !(d < near && self.w.visible(eye[0], eye[1], eye[2], it.pos[0], it.pos[1], it.pos[2] + 0.8))
        });
    }

    /// Marks whether an item can be reached by walking.
    fn ground_check(&self, it: &mut Item) {
        let k = self.nav.index(it.pos[0], it.pos[1]);
        it.cell = k;
        it.ground = false;
        if !self.nav.solid[k] && (self.nav.z[k] as f64 - it.pos[2]).abs() < 3.0 {
            it.ground = true;
            return;
        }
        // just inside a wall cell: look for an adjacent walkable cell at the same height
        let c = self.nav.nearest(it.pos[0], it.pos[1], self.radius, self.height, 2);
        if c >= 0 && (self.nav.z[c as usize] as f64 - it.pos[2]).abs() < 3.0 {
            it.cell = c as usize;
            it.ground = true;
        }
    }

    fn has(&self, kind: &str) -> i64 {
        for i in 0..2 {
            if self.slots[i].ok && self.slots[i].kind == kind {
                return i as i64;
            }
        }
        -1
    }

    fn cube_damage_mul(&self, cubes: i64) -> f64 {
        let capc = self.h.config.cubes.stat_cap;
        let mut c = cubes as f64;
        if c > capc {
            c = capc;
        }
        1.0 + self.h.config.cubes.damage_per_cube * c
    }
}
