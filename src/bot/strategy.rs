use super::*;
use crate::proto::Action;

/// How we behave given what kind of lobby this looks like.
#[derive(Clone, Debug, Default)]
pub(crate) struct Stance {
    pub name: &'static str,
    pub engage_ratio: f64,
    pub disengage: f64,
    pub hunt: bool,
    pub stealth: bool,
    pub avoid: bool,
    pub loot_radius: f64,
    pub want_cubes: i64,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct ZoneInfo {
    pub out_next: f64,
    pub out_cur: f64,
    pub t_pass: f64,
    pub goal: [f64; 2],
    pub urgent: bool,
    pub critical: bool,
    pub final_: bool,
    pub walkable: bool,
}

#[derive(Clone, Debug)]
pub(crate) struct Threat {
    pub tr: Track,
    pub d: f64,
    pub adv: f64,
    pub rmin: f64,
    pub rmax: f64,
    pub engaged: bool,
    pub dps: f64,
    pub phantom: bool,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct Loot {
    pub kind: &'static str,
    pub id: i64,
    pub pos: [f64; 3],
    pub score: f64,
    pub shoot: bool,
}

fn upgrade_rank(v2: bool, id: &str) -> Option<f64> {
    let t: &[(&str, f64)] = if v2 {
        &[
            ("plating", 1.00), ("armor", 0.96), ("gun_mag", 0.95), ("laser_damage", 0.93), ("lightning_efficiency", 0.92),
            ("gun_damage", 0.90), ("lightning_range", 0.87), ("laser_charge", 0.86), ("gun_rate", 0.84), ("gun_velocity", 0.82),
            ("servos", 0.76), ("nanites", 0.74), ("reactor", 0.50), ("capacitor", 0.45),
            ("sensors", 0.35), ("gyros", 0.30), ("jets", 0.30), ("mortar_reload", 0.42), ("mortar_splash", 0.45),
            ("mortar_velocity", 0.40), ("lightning_chain", 0.40), ("laser_pierce", 0.20),
        ]
    } else {
        &[
            ("plating", 1.00), ("armor", 0.96), ("gun_rate", 0.90), ("gun_damage", 0.88), ("gun_velocity", 0.80),
            ("laser_damage", 0.91), ("laser_charge", 0.86), ("lightning_efficiency", 0.92), ("lightning_range", 0.87),
            ("nanites", 0.72), ("servos", 0.76), ("reactor", 0.50), ("capacitor", 0.45), ("gun_mag", 0.60),
            ("sensors", 0.35), ("gyros", 0.30), ("jets", 0.30), ("mortar_reload", 0.42), ("mortar_splash", 0.45),
            ("mortar_velocity", 0.40), ("lightning_chain", 0.40), ("laser_pierce", 0.20),
        ]
    };
    t.iter().find(|(k, _)| *k == id).map(|(_, v)| *v)
}

impl Bot {
    pub(crate) fn zone_info(&mut self) -> ZoneInfo {
        let z = self.o.zone.clone();
        let mut zi = ZoneInfo::default();
        let dn = hyp(self.pos[0] - z.next_center[0], self.pos[1] - z.next_center[1]);
        let dc = hyp(self.pos[0] - z.center[0], self.pos[1] - z.center[1]);
        zi.out_next = dn - z.next_radius;
        zi.out_cur = dc - z.radius;
        zi.t_pass = f64::INFINITY;
        zi.final_ = z.next_radius < 12.0;
        let t0 = self.t.max(z.shrink_start);
        if zi.out_cur > 0.0 {
            zi.t_pass = 0.0;
        } else if zi.out_next > 0.0 && z.shrink_end > t0 {
            for i in 1..=50 {
                let f = i as f64 / 50.0;
                let cx = z.center[0] + (z.next_center[0] - z.center[0]) * f;
                let cy = z.center[1] + (z.next_center[1] - z.center[1]) * f;
                let r = z.radius + (z.next_radius - z.radius) * f;
                if hyp(self.pos[0] - cx, self.pos[1] - cy) > r {
                    zi.t_pass = (t0 - self.t) + f * (z.shrink_end - t0);
                    break;
                }
            }
        } else if zi.out_next > 0.0 && z.shrink_end <= t0 {
            zi.t_pass = 0.0;
        }
        let (dx, dy) = norm2(self.pos[0] - z.next_center[0], self.pos[1] - z.next_center[1]);
        let key = [z.next_center[0], z.next_center[1], z.next_radius];
        if key != self.zg_key || self.t - self.zg_t > 1.0 || self.zg_comp != self.my_comp {
            self.zg_key = key;
            self.zg_t = self.t;
            self.zg_comp = self.my_comp;
            let margin = (5.0f64).min(z.next_radius * 0.3);
            let base = dy.atan2(dx);
            let (mut found, mut found_any) = (false, false);
            let mut any_goal = [0.0; 2];
            'search: for fr in [0.6, 0.45, 0.75, 0.3, 0.15, 0.9, 0.0] {
                let rr = (z.next_radius * fr).min(z.next_radius - 10.0).max(0.0);
                for off in [0.0, 0.4, -0.4, 0.8, -0.8, 1.4, -1.4, 2.2, -2.2, std::f64::consts::PI] {
                    let (x, y) = (z.next_center[0] + (base + off).cos() * rr, z.next_center[1] + (base + off).sin() * rr);
                    let k = self.nav.nearest(x, y, self.radius + 0.3, self.height, 4);
                    if k < 0 {
                        continue;
                    }
                    let (px, py) = self.nav.pos(k as usize);
                    if z.next_radius > 12.0 && hyp(px - z.next_center[0], py - z.next_center[1]) > z.next_radius - margin {
                        continue;
                    }
                    if !self.same_comp(k) {
                        if !found_any {
                            found_any = true;
                            any_goal = [px, py];
                        }
                        continue;
                    }
                    self.zg_goal = [px, py];
                    found = true;
                    break 'search;
                }
            }
            self.zg_walk = found;
            if !found {
                self.zg_goal = if found_any { any_goal } else { [z.next_center[0], z.next_center[1]] };
            }
        }
        zi.goal = self.zg_goal;
        zi.walkable = self.zg_walk;
        let mut need = 0.0;
        if zi.out_next > 0.0 {
            need = (zi.out_next + 8.0) / self.walk * 1.35 + 4.0;
            if !zi.walkable && self.p.overland_early > 0.0 {
                need = need * self.p.overland_early + 25.0;
            }
        }
        zi.critical = zi.out_cur > -2.0 || zi.t_pass < 2.5;
        zi.urgent = zi.critical || zi.t_pass < 9.0 || (zi.out_next > 0.0 && z.shrink_end - self.t < need) || (zi.out_next > 0.0 && zi.t_pass - need < 3.0);
        zi
    }

    pub(crate) fn assess(&mut self) -> Vec<Threat> {
        let mut out = Vec::new();
        let ids: Vec<i64> = self.tracks.keys().copied().collect();
        for id in ids {
            let tr = self.tracks[&id].clone();
            if tr.npc {
                continue;
            }
            let phantom = !tr.listed && self.t - tr.phantom_t < 4.0 && self.t - tr.last_seen > 1.0;
            if self.t - tr.last_seen > 8.0 && !phantom {
                continue;
            }
            let d = dist3(self.pos, tr.pos);
            let (mut adv, rmin, rmax) = self.fight_plan(&tr, d);
            if self.p.exchange_w > 0.0 && (tr.hurt_me > 1.0 || tr.i_hurt > 1.0) {
                let ex = (tr.i_hurt + 150.0) / (tr.hurt_me + 150.0);
                adv *= clamp(ex, 0.4, 1.8).powf(self.p.exchange_w);
            }
            let mut dps = self.their_dps(&tr, d);
            let mut aim_r = 200.0;
            if tr.laser && self.p.laser_aim_range > aim_r && self.real_model() {
                aim_r = self.p.laser_aim_range;
            }
            let mut engaged = self.t - tr.hurt_me_t < 4.0 || (tr.los && self.t - tr.aim_at_me_t < 0.6 && d < aim_r) || (tr.los && d < 40.0);
            if !engaged && tr.los && d < 90.0 {
                let bear = (self.pos[1] - tr.pos[1]).atan2(self.pos[0] - tr.pos[0]);
                let closing = (tr.vel[0] * (self.pos[0] - tr.pos[0]) + tr.vel[1] * (self.pos[1] - tr.pos[1])) / d.max(1.0);
                if ang_diff(tr.aim[0], bear).abs() < 0.5 && closing > 4.0 {
                    engaged = true;
                }
            }
            if phantom {
                engaged = true;
                dps = dps.max(tr.hurt_me / 3.0);
                adv = adv.min(0.5);
            }
            let mut tr = tr;
            if engaged {
                tr.eng_t = self.t;
                self.tracks.get_mut(&id).unwrap().eng_t = self.t;
            } else if self.t - tr.eng_t < 0.8 && tr.listed {
                engaged = true;
            }
            out.push(Threat { tr, d, adv, rmin, rmax, engaged, dps, phantom });
        }
        out
    }

    /// Whether a nav cell is in the walkable region we are standing in.
    pub(crate) fn same_comp(&self, cell: i64) -> bool {
        if self.my_comp == 0 || cell < 0 {
            return true;
        }
        self.nav.components()[cell as usize] == self.my_comp
    }

    /// Whether a ground point can be walked to from here (same walkable region).
    pub(crate) fn reachable(&self, x: f64, y: f64) -> bool {
        if self.my_comp == 0 {
            return true;
        }
        let c = self.nav.nearest(x, y, 2.9, 9.0, 3);
        c >= 0 && self.nav.components()[c as usize] == self.my_comp
    }

    /// The eyes of the threats recent and close enough to matter for cover.
    pub(crate) fn threat_eyes(&self, ths: &[Threat], max_d: f64, max_age: f64) -> Vec<[f64; 3]> {
        let mut eyes = Vec::new();
        for th in ths {
            let tr = &th.tr;
            if (self.t - tr.last_seen > max_age && !th.phantom) || th.d > max_d {
                continue;
            }
            let sc = tr.scale(self.size_per_cube);
            eyes.push([tr.pos[0], tr.pos[1], tr.pos[2] + 8.0 * sc]);
        }
        eyes
    }

    /// Whether a mech standing at (x,y) on ground gz is seen (middle or head) from an eye.
    #[inline]
    pub(crate) fn seen_from(&self, e: [f64; 3], x: f64, y: f64, gz: f64) -> bool {
        self.w.visible(e[0], e[1], e[2], x, y, gz + self.height * 0.5) || self.w.visible(e[0], e[1], e[2], x, y, gz + self.height * 0.95)
    }

    /// Whether none of the threats can see us where we stand right now.
    pub(crate) fn hidden_here(&self, ths: &[Threat]) -> bool {
        !self.threat_eyes(ths, 260.0, 6.0).into_iter().any(|e| self.seen_from(e, self.pos[0], self.pos[1], self.pos[2]))
    }

    /// The nearest walkable cell hidden from every listed threat (-1: none).
    pub(crate) fn find_cover(&self, ths: &[Threat], max_cost: f64) -> i64 {
        let eyes = self.threat_eyes(ths, 260.0, 6.0);
        if eyes.is_empty() {
            return -1;
        }
        let z = &self.o.zone;
        let nav = &self.nav;
        let mut found = -1i64;
        let mut checked = 0;
        nav.flood(self.pos[0], self.pos[1], self.radius + 0.15, self.height, max_cost, None, &mut |k, _cost| {
            if (k % nav.nx + k / nav.nx) % 2 != 0 {
                return true;
            }
            if (nav.clear[k] as f64) < self.radius + 0.6 {
                return true;
            }
            let (x, y) = nav.pos(k);
            if hyp(x - z.center[0], y - z.center[1]) > z.radius - 6.0 {
                return true;
            }
            if z.shrink_end - self.t < 25.0 && hyp(x - z.next_center[0], y - z.next_center[1]) > z.next_radius - 4.0 {
                return true;
            }
            checked += 1;
            if checked > 700 {
                return false;
            }
            let gz = nav.z[k] as f64;
            if eyes.iter().any(|&e| self.seen_from(e, x, y, gz)) {
                return true;
            }
            found = k as i64;
            false
        });
        found
    }

    /// Whether a mech standing in the cell is out of sight of every recent threat's last known position.
    pub(crate) fn cell_hidden(&self, k: i64, ths: &[Threat]) -> bool {
        let (x, y) = self.nav.pos(k as usize);
        let gz = self.nav.z[k as usize] as f64;
        !self.threat_eyes(ths, 260.0, 6.0).into_iter().any(|e| self.seen_from(e, x, y, gz))
    }

    /// Distance to the nearest recent threat's last known position (1e9: none).
    pub(crate) fn nearest_eye(&self, ths: &[Threat]) -> f64 {
        let mut d0 = 1e9;
        for th in ths {
            let tr = &th.tr;
            if (self.t - tr.last_seen > 6.0 && !th.phantom) || th.d > 260.0 {
                continue;
            }
            let d = hyp(tr.pos[0] - self.pos[0], tr.pos[1] - self.pos[1]);
            if d < d0 {
                d0 = d;
            }
        }
        d0
    }

    /// Ranks the weapons for pickup and swap decisions.
    pub(crate) fn weapon_value(&self, kind: &str) -> f64 {
        match kind {
            "laser" => self.p.laser_value,
            "lightning" => self.p.lightning_value,
            "mortar" => 1.2,
            "gun" => 2.0,
            _ => 0.0,
        }
    }

    /// Whether we would take this weapon and which slot it would replace (-1: empty slot).
    pub(crate) fn want_pickup(&self, kind: &str) -> (bool, i64) {
        if self.has(kind) >= 0 {
            return (false, -1);
        }
        let cookoff = self.wstat("laser", "cookoff", 0.0);
        if kind == "mortar" && self.p.no_mortar && (self.real_model() || (self.p.all_no_mortar && cookoff > 0.0)) {
            return (false, -1);
        }
        for i in 0..2 {
            if !self.slots[i].ok {
                return (true, -1);
            }
        }
        let (mut worst, mut wv) = (-1i64, 1e9);
        for i in 0..2 {
            if self.slots[i].kind == "gun" {
                continue;
            }
            let v = self.weapon_value(&self.slots[i].kind);
            if v < wv {
                worst = i as i64;
                wv = v;
            }
        }
        if worst >= 0 && self.weapon_value(kind) > wv + 0.3 && self.slots[worst as usize].charge <= 0.0 {
            return (true, worst);
        }
        (false, -1)
    }

    /// Whether a point will still be inside the zone in `secs` seconds (roughly).
    pub(crate) fn in_future_zone(&self, x: f64, y: f64, secs: f64, margin: f64) -> bool {
        let z = &self.o.zone;
        if hyp(x - z.center[0], y - z.center[1]) > z.radius - margin {
            return false;
        }
        if self.t + secs > z.shrink_start {
            let mut f = 1.0;
            if z.shrink_end > z.shrink_start {
                f = clamp((self.t + secs - z.shrink_start) / (z.shrink_end - z.shrink_start), 0.0, 1.0);
            }
            let cx = z.center[0] + (z.next_center[0] - z.center[0]) * f;
            let cy = z.center[1] + (z.next_center[1] - z.center[1]) * f;
            let r = z.radius + (z.next_radius - z.radius) * f;
            if hyp(x - cx, y - cy) > r - margin {
                return false;
            }
        }
        true
    }

    pub(crate) fn enemy_near(&self, pos: [f64; 3], r: f64) -> bool {
        self.tracks.values().any(|tr| !tr.npc && self.t - tr.last_seen <= 6.0 && dist2(tr.pos, pos) < r)
    }

    pub(crate) fn choose_loot(&self, danger: bool, st: &Stance) -> Option<Loot> {
        let mut best: Option<Loot> = None;
        let mut watch_eyes: Vec<[f64; 3]> = Vec::new();
        let strict = self.shy_phase(st);
        let mem = if strict { self.p.shy_mem } else { 4.0 };
        if (self.p.loot_hidden_bias > 0.0 || strict) && st.avoid {
            for tr in self.tracks.values() {
                if tr.npc || self.t - tr.last_seen > mem {
                    continue;
                }
                let sc = tr.scale(self.size_per_cube);
                watch_eyes.push([tr.pos[0], tr.pos[1], tr.pos[2] + 8.0 * sc]);
                if watch_eyes.len() >= 6 {
                    break;
                }
            }
        }
        let mut loot_radius = st.loot_radius;
        let mut consider = |mut l: Loot, mut value: f64, handling: f64, loot_radius: f64| {
            let d = dist2(self.pos, l.pos);
            if d > loot_radius {
                return;
            }
            if let Some(&until) = self.ban_until.get(&l.id) {
                if self.t < until {
                    return;
                }
            }
            if !self.reachable(l.pos[0], l.pos[1]) {
                return;
            }
            if st.stealth {
                let e = self.nav.exposure().at(self.nav.index(l.pos[0], l.pos[1]));
                if e <= 1.0 {
                    value *= 1.0 - 0.6 * e as f64;
                }
            }
            let tt = d * 1.25 / self.walk + handling;
            if !self.in_future_zone(l.pos[0], l.pos[1], tt + 2.0 + self.p.zone_slack, 6.0) {
                return;
            }
            if self.enemy_near(l.pos, 45.0) && l.kind != "cube" {
                value *= 0.35;
            }
            let mut is_laser = false;
            if l.kind == "pickup" && self.has("laser") < 0 {
                if let Some(it) = self.pickups.get(&l.id) {
                    if it.kind == "laser" {
                        is_laser = true;
                    }
                }
            }
            if strict {
                if self.p.shy_loot_radius > 0.0 && d > self.p.shy_loot_radius && !is_laser {
                    return;
                }
                if self.p.shy_exposure > 0.0 {
                    let e = self.nav.exposure().at(self.nav.index(l.pos[0], l.pos[1]));
                    if e <= 1.0 {
                        value *= 1.0 - self.p.shy_exposure * e as f64;
                    }
                }
            }
            if strict && !watch_eyes.is_empty() {
                let laser_here = is_laser && d < self.p.shy_laser;
                if !laser_here {
                    for e in &watch_eyes {
                        let de = hyp(e[0] - l.pos[0], e[1] - l.pos[1]);
                        if de < 230.0 && self.w.visible(e[0], e[1], e[2], l.pos[0], l.pos[1], l.pos[2] + 4.5) {
                            return;
                        }
                        if de < 150.0 && de < hyp(e[0] - self.pos[0], e[1] - self.pos[1]) - 10.0 {
                            return;
                        }
                    }
                }
            } else if self.p.loot_hidden_bias > 0.0 && st.avoid && !watch_eyes.is_empty() {
                for e in &watch_eyes {
                    if hyp(e[0] - l.pos[0], e[1] - l.pos[1]) < 230.0 && self.w.visible(e[0], e[1], e[2], l.pos[0], l.pos[1], l.pos[2] + 4.5) {
                        value *= 1.0 - self.p.loot_hidden_bias;
                        break;
                    }
                }
            }
            l.score = value / (tt + 1.5);
            if l.id == self.loot_id {
                l.score *= 1.25;
            }
            if best.as_ref().map_or(true, |b| l.score > b.score) {
                best = Some(l);
            }
        };
        let want_cubes = self.o.self_.cubes < st.want_cubes;
        for it in self.pickups.values() {
            if !it.ground {
                continue;
            }
            let (ok, rep) = self.want_pickup(&it.kind);
            if ok {
                let mut v = self.weapon_value(&it.kind) * 4.0;
                if rep >= 0 {
                    v = (self.weapon_value(&it.kind) - self.weapon_value(&self.slots[rep as usize].kind)) * 4.0;
                }
                consider(Loot { kind: "pickup", id: it.id, pos: it.pos, ..Default::default() }, v, 0.5, loot_radius);
            }
        }
        if want_cubes {
            let mut next_thr = 999999i64;
            if self.p.cube_smart {
                for &th in &self.h.config.cubes.upgrade_thresholds {
                    let ti = th.round() as i64;
                    if ti > self.o.self_.cubes {
                        next_thr = ti;
                        break;
                    }
                }
            }
            let thresh_mult = |gain: i64| if self.p.cube_smart && self.o.self_.cubes + gain >= next_thr { 1.75 } else { 1.0 };
            for it in self.cubes.values() {
                if it.ground {
                    let mut v = 11.0;
                    let mut gain = 1;
                    if self.p.cube_smart {
                        for o in self.cubes.values() {
                            if o.id != it.id && o.ground && dist2(o.pos, it.pos) < 7.5 {
                                gain += 1;
                            }
                        }
                        v = 11.0 * (1.0 + 0.65 * (gain - 1) as f64) * thresh_mult(gain);
                    }
                    consider(Loot { kind: "cube", id: it.id, pos: it.pos, ..Default::default() }, v, 0.0, loot_radius);
                }
            }
            for it in self.crates.values() {
                if it.ground {
                    consider(Loot { kind: "crate", id: it.id, pos: it.pos, shoot: true, ..Default::default() }, 7.0 * thresh_mult(1), 2.5, loot_radius);
                }
            }
            if !danger {
                for tr in self.tracks.values() {
                    if tr.npc && self.t - tr.last_seen < 20.0 {
                        let k = self.nav.index(tr.pos[0], tr.pos[1]);
                        if !self.nav.solid[k] && (self.nav.z[k] as f64 - tr.pos[2]).abs() < 4.0 {
                            let npc_cubes = self.npc_stat("cubes", 2.0).max(1.0) as i64;
                            consider(Loot { kind: "npc", id: tr.id, pos: tr.pos, shoot: true, ..Default::default() }, 11.0 * thresh_mult(npc_cubes), 5.0, loot_radius);
                        }
                    }
                }
            }
        }
        // supply drops
        for dr in &self.o.drops {
            let d = dist2(self.pos, dr.pos);
            if d > self.p.drop_radius {
                continue;
            }
            if dr.lands_in > d / self.walk * 1.3 + 6.0 && !(self.p.drop_stick && dr.id == self.loot_id) {
                continue;
            }
            if self.p.drop_stick && st.avoid && self.o.self_.cubes >= st.want_cubes && self.hp > self.max_hp * 0.7 {
                continue;
            }
            if let Some(h) = dr.holder {
                if h != self.o.self_.id && st.stealth {
                    continue;
                }
            }
            if st.stealth && self.enemy_near(dr.pos, 110.0) {
                continue;
            }
            let mut v = 30.0 * self.p.drop_greed * 2.0;
            if self.hp < self.max_hp * 0.6 {
                v *= 1.5;
            }
            let saved = loot_radius;
            loot_radius = self.p.drop_radius;
            consider(Loot { kind: "drop", id: dr.id, pos: dr.pos, ..Default::default() }, v, 4.0 + dr.lands_in * 0.3, loot_radius);
            loot_radius = saved;
        }
        let _ = loot_radius;
        best
    }

    pub(crate) fn pick_upgrade(&self, act: &mut Action) {
        let of = match &self.o.offer {
            Some(of) if !of.choices.is_empty() => of,
            _ => return,
        };
        let (mut best, mut bv) = (0i64, -1.0);
        for (i, c) in of.choices.iter().enumerate() {
            let mut v = upgrade_rank(self.p.up_v2, &c.id).unwrap_or(0.25);
            if self.has("lightning") >= 0 && (c.id == "reactor" || c.id == "capacitor") {
                v += 0.3;
            }
            if c.id == "nanites" && self.p.nano_boost > 0.0 {
                v += self.p.nano_boost;
            }
            if v > bv {
                best = i as i64;
                bv = v;
            }
        }
        act.pick = best;
    }

    /// Takes a wanted weapon when we stand on it.
    pub(crate) fn manage_weapons(&mut self, act: &mut Action) {
        if self.t < self.take_t {
            return;
        }
        let mut reach = self.h.config.loot.pickup_radius;
        if reach <= 0.0 {
            reach = 4.0;
        }
        let (mut best_id, mut best_v, mut best_rep) = (-1i64, 0.0, -1i64);
        for it in self.pickups.values() {
            if self.t - it.last_seen > 0.5 {
                continue;
            }
            if dist2(self.pos, it.pos) > reach - 0.3 || (it.pos[2] - self.pos[2]).abs() > 3.7 {
                continue;
            }
            let (ok, rep) = self.want_pickup(&it.kind);
            if ok {
                let v = self.weapon_value(&it.kind);
                if v > best_v {
                    best_id = it.id;
                    best_v = v;
                    best_rep = rep;
                }
            }
        }
        if best_id >= 0 {
            act.take = best_id;
            if best_rep >= 0 {
                act.drop = best_rep;
            }
            self.take_t = self.t + 0.25;
        }
    }
}
