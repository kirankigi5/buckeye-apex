//! Roof play: a mech standing back from the edge of a roof cannot be seen or hit from the ground
//! around the building, and getting up there costs a third of an energy bar.

use super::hidemap::{Hideout, HM_ROOF};
use super::strategy::{Stance, Threat, ZoneInfo};
use super::*;
use crate::proto::Action;
use crate::world::{Roof, RoofAccess};

pub(crate) const ROOF_NONE: i32 = 0;
pub(crate) const ROOF_APPROACH: i32 = 1;
pub(crate) const ROOF_CLIMB: i32 = 2;
pub(crate) const ROOF_ON: i32 = 3;

#[derive(Clone, Debug, Default)]
pub(crate) struct RoofPlan {
    pub state: i32,
    pub id: i64,
    pub acc: RoofAccess,
    pub t0: f64,
    pub ban: HashMap<i64, f64>,
    pub chosen: f64,
    pub tries: i64,
    pub step_t: f64,
    pub wait_t0: f64,
    pub want: i64,
    pub spot: i64,
}

impl RoofPlan {
    pub fn new() -> RoofPlan {
        RoofPlan { id: -1, want: -1, spot: -1, ..Default::default() }
    }

    fn banned(&self, id: i64, t: f64) -> bool {
        self.ban.get(&id).is_some_and(|&until| t < until)
    }
}

impl Bot {
    /// The id of the roof region we are standing on, or -1.
    pub(crate) fn on_roof(&self) -> i64 {
        let (roofs, roof_of) = self.nav.roofs();
        let k = self.nav.index(self.pos[0], self.pos[1]);
        let id = roof_of[k];
        if id < 0 || id as usize >= roofs.len() {
            return -1;
        }
        if (self.pos[2] - roofs[id as usize].z).abs() < 1.6 {
            return id as i64;
        }
        -1
    }

    pub(crate) fn max_climb(&self) -> f64 {
        let mut drain = self.self_stat("jet_drain", self.mech("jet_drain", 15.0));
        if drain <= 0.0 {
            drain = 15.0;
        }
        (self.o.self_.max_energy - 28.0) / drain * 8.9
    }

    /// Whether a roof's inner spot will stay inside the zone long enough to be worth it.
    fn roof_spot_ok(&self, r: &Roof) -> bool {
        let z = &self.o.zone;
        let (mut x, mut y) = self.nav.pos(r.inner);
        if self.roof.want == r.id as i64 && self.roof.spot >= 0 {
            (x, y) = self.nav.pos(self.roof.spot as usize);
        }
        hyp(x - z.next_center[0], y - z.next_center[1]) < z.next_radius - 8.0
    }

    /// The best roof to hide on, with the access point to climb at.
    pub(crate) fn choose_roof(&self) -> Option<(i64, RoofAccess)> {
        let nav = self.nav.clone();
        let (roofs, _) = nav.roofs();
        let w = self.roof.want;
        if w >= 0 && (w as usize) < roofs.len() {
            return self.roof_access_for(&roofs[w as usize], self.max_climb().min(self.p.roof_max_climb)).map(|(acc, _)| (w, acc));
        }
        let comp = nav.components();
        let mut my_comp = 0;
        let c = nav.nearest(self.pos[0], self.pos[1], 2.9, 9.0, 6);
        if c >= 0 {
            my_comp = comp[c as usize];
        }
        let limit = self.max_climb().min(self.p.roof_max_climb);
        let (mut best_id, mut best_s) = (-1i64, 1e18);
        let mut best_acc = RoofAccess::default();
        for (i, r) in roofs.iter().enumerate() {
            if self.roof.banned(i as i64, self.t) {
                continue;
            }
            if r.height < self.p.roof_min_height || r.inner_d < 6.0 + self.radius || !self.roof_spot_ok(r) {
                continue;
            }
            if hyp(r.cx - self.pos[0], r.cy - self.pos[1]) > 260.0 + r.area.sqrt() {
                continue;
            }
            let mut acc = RoofAccess::default();
            let mut ad = 1e18;
            for a in &r.access {
                if a.climb > limit || a.climb < 6.0 {
                    continue;
                }
                if my_comp != 0 && comp[a.ground] != my_comp {
                    continue;
                }
                if !nav.walkable(a.ground, self.radius + 0.2, self.height) {
                    continue;
                }
                let (gx, gy) = nav.pos(a.ground);
                let d = hyp(gx - self.pos[0], gy - self.pos[1]);
                if d < ad {
                    acc = *a;
                    ad = d;
                }
            }
            if ad > 1e17 {
                continue;
            }
            let mut s = ad * 1.2 - r.inner_d.min(25.0) * 2.5 - r.height.min(30.0) * 1.5;
            let (ix, iy) = nav.pos(r.inner);
            let z = &self.o.zone;
            if self.p.roof_center_w > 0.0 {
                s += hyp(ix - z.next_center[0], iy - z.next_center[1]) * self.p.roof_center_w;
            }
            if self.ffa && self.p.edge_bias > 0.0 && z.next_radius > 60.0 {
                let dn = hyp(ix - z.next_center[0], iy - z.next_center[1]);
                s -= self.p.edge_bias * 25.0 * dn / z.next_radius;
                if self.p.edge_band > 0.0 && dn < z.next_radius * self.p.edge_band {
                    s += 50.0;
                }
            }
            for tr in self.tracks.values() {
                if tr.npc || self.t - tr.last_seen > 20.0 {
                    continue;
                }
                let d = hyp(tr.pos[0] - ix, tr.pos[1] - iy);
                if d < 120.0 {
                    s += (120.0 - d) * 0.6;
                }
            }
            if i as i64 == self.roof.id && self.roof.state != ROOF_NONE {
                s -= 40.0;
            }
            if s < best_s {
                best_id = i as i64;
                best_s = s;
                best_acc = acc;
            }
        }
        if best_id >= 0 {
            Some((best_id, best_acc))
        } else {
            None
        }
    }

    /// Where to stand on roof id: climb the distance-to-edge field from where we are.
    fn roof_spot(&self, id: i64) -> (f64, f64) {
        let nav = &self.nav;
        let (roofs, roof_of) = nav.roofs();
        let dist = nav.roof_dist();
        let r = &roofs[id as usize];
        let s = self.roof.spot;
        if self.roof.want == id && s >= 0 && roof_of[s as usize] as i64 == id {
            return nav.pos(s as usize);
        }
        let k = nav.index(self.pos[0], self.pos[1]);
        if roof_of[k] as i64 != id {
            return nav.pos(r.inner);
        }
        let (ix, iy) = nav.pos(r.inner);
        let d = hyp(ix - self.pos[0], iy - self.pos[1]);
        let steps = (d / 2.0) as i64 + 1;
        let mut clear = true;
        for s in 1..=steps {
            let t = s as f64 / steps as f64;
            let c = nav.index(self.pos[0] + (ix - self.pos[0]) * t, self.pos[1] + (iy - self.pos[1]) * t);
            if roof_of[c] as i64 != id || dist[c] < 2 {
                clear = false;
                break;
            }
        }
        if clear {
            return (ix, iy);
        }
        let (ci, cj) = ((k % nav.nx) as i64, (k / nav.nx) as i64);
        let (mut best, mut bd) = (k, dist[k]);
        for dj in -3..=3 {
            for di in -3..=3 {
                let (i, j) = (ci + di, cj + dj);
                if i < 0 || j < 0 || i >= nav.nx as i64 || j >= nav.ny as i64 {
                    continue;
                }
                let c = (j * nav.nx as i64 + i) as usize;
                if roof_of[c] as i64 == id && dist[c] > bd {
                    best = c;
                    bd = dist[c];
                }
            }
        }
        nav.pos(best)
    }

    /// A spot on our roof that none of the given threats can see (body or head), nearest first (-1: none).
    pub(crate) fn roof_cover(&self, id: i64, ths: &[Threat]) -> i64 {
        let nav = &self.nav;
        let (roofs, roof_of) = nav.roofs();
        let dist = nav.roof_dist();
        let r = &roofs[id as usize];
        let eyes = self.threat_eyes(ths, 280.0, 8.0);
        if eyes.is_empty() {
            return -1;
        }
        let (mut best, mut bs) = (-1i64, 1e18);
        for &c in &r.cells {
            let k = c as usize;
            let (ci, cj) = (k % nav.nx, k / nav.nx);
            if (ci + cj) % 2 != 0 || dist[k] < 3 || roof_of[k] as i64 != id {
                continue;
            }
            let (x, y) = nav.pos(k);
            let d = hyp(x - self.pos[0], y - self.pos[1]);
            if d > 50.0 || d >= bs {
                continue;
            }
            let hidden = !eyes.iter().any(|e| {
                self.w.visible(e[0], e[1], e[2], x, y, r.z + self.height * 0.5) || self.w.visible(e[0], e[1], e[2], x, y, r.z + self.height * 0.97)
            });
            if hidden {
                best = k as i64;
                bs = d;
            }
        }
        best
    }

    /// Whether a mech is up here with us (same level, close).
    pub(crate) fn enemy_on_roof(&self, ths: &[Threat]) -> bool {
        ths.iter().any(|th| th.tr.listed && th.d < 70.0 && (th.tr.pos[2] - self.pos[2]).abs() < 5.0)
    }

    /// Drives the jet climb. It writes the action directly.
    pub(crate) fn roof_climb_step(&mut self, act: &mut Action) {
        let nav = self.nav.clone();
        let (roofs, roof_of) = nav.roofs();
        let r = &roofs[self.roof.id as usize];
        let (ex, ey) = nav.pos(self.roof.acc.edge);
        let (dx, dy) = norm2(ex - self.pos[0], ey - self.pos[1]);
        act.mv = [dx, dy];
        let feet = self.pos[2];
        let cell = nav.index(self.pos[0], self.pos[1]);
        let over = roof_of[cell] as i64 == self.roof.id;
        let grounded = self.o.self_.grounded;
        if grounded && over && (feet - r.z).abs() < 1.6 {
            self.roof.state = ROOF_ON;
            self.roof.t0 = self.t;
            self.roof.tries = 0;
            return;
        } else if grounded && feet < r.z - 1.0 {
            act.jump = true;
        } else if feet < r.z + 0.7 {
            act.boost = true;
        } else if !over {
            act.boost = self.vel[2] < 0.5;
        }
        let budget = self.roof.acc.climb / 8.0 + 5.0;
        if self.t - self.roof.t0 > budget || (self.energy < 2.0 && !grounded && feet < r.z - 0.5) {
            self.roof.tries += 1;
            self.roof.ban.insert(self.roof.id, self.t + 90.0);
            self.roof.state = ROOF_NONE;
        }
    }

    /// Travels to a goal over the top surface of the map: walking, dropping off edges and jetting up
    /// walls. It writes the action directly.
    pub(crate) fn overland(&mut self, act: &mut Action, gx: f64, gy: f64) {
        let nav = self.nav.clone();
        let tops = nav.tops();
        let feet = self.pos[2];
        let grounded = self.o.self_.grounded;
        let here = nav.index(self.pos[0], self.pos[1]);
        if self.t - self.surf_watch_t >= 1.5 {
            if self.surf_pushing && dist3(self.pos, self.surf_watch_pos) < 1.2 && self.surf.len() >= 2 {
                let mut j = self.surf_idx + 1;
                while j < self.surf.len() && j <= self.surf_idx + 3 {
                    self.surf_ban.insert(self.surf[j] as usize, self.t + 25.0);
                    j += 1;
                }
                self.surf.clear();
                self.surf_t = -100.0;
            }
            self.surf_watch_t = self.t;
            self.surf_watch_pos = self.pos;
            self.surf_pushing = false;
        }
        let mut replan = self.surf.len() < 2 || hyp(gx - self.surf_goal[0], gy - self.surf_goal[1]) > 6.0 || self.t - self.surf_t > 1.2;
        if !replan {
            let (px, py) = nav.pos(self.surf[self.surf_idx] as usize);
            if hyp(px - self.pos[0], py - self.pos[1]) > 7.0 {
                replan = true;
            }
        }
        if replan && self.t - self.surf_t >= 0.3 {
            let t = self.t;
            let ban = &self.surf_ban;
            let blocked = |k: usize| ban.get(&k).is_some_and(|&until| t < until);
            self.surf = nav.surface_path(self.pos[0], self.pos[1], feet, gx, gy, self.max_climb(), self.radius, 14000, Some(&blocked));
            self.surf_goal = [gx, gy];
            self.surf_t = self.t;
            self.surf_idx = 0;
        }
        let (mut dx, mut dy) = norm2(gx - self.pos[0], gy - self.pos[1]);
        let mut need = feet;
        if self.surf.len() >= 2 {
            let (mut best, mut bd) = (self.surf_idx, 1e18);
            let mut j = self.surf_idx;
            while j < self.surf.len() && j < self.surf_idx + 8 {
                let (px, py) = nav.pos(self.surf[j] as usize);
                let d = hyp(px - self.pos[0], py - self.pos[1]);
                if d < bd {
                    best = j;
                    bd = d;
                }
                j += 1;
            }
            self.surf_idx = best;
            let n1 = (best + 1).min(self.surf.len() - 1);
            let mut tgt = n1;
            let top = |i: usize| tops[self.surf[i] as usize] as f64;
            while tgt < self.surf.len() - 1 && tgt < best + 3 && (top(tgt + 1) - top(n1)).abs() < 0.6 && top(n1) <= feet + 1.0 {
                tgt += 1;
            }
            let (tx, ty) = nav.pos(self.surf[tgt] as usize);
            if self.surf[tgt] as usize != here {
                (dx, dy) = norm2(tx - self.pos[0], ty - self.pos[1]);
            }
            need = top(n1);
            let h = top(tgt);
            if h > need {
                need = h;
            }
        } else {
            for s in [self.radius + 1.0, self.radius + 3.5] {
                let h = self.w.floor(self.pos[0] + dx * s, self.pos[1] + dy * s, 1e9).0;
                if h > need {
                    need = h;
                }
            }
        }
        act.mv = [dx, dy];
        let rise = need - feet;
        let drain = self.self_stat("jet_drain", self.mech("jet_drain", 15.0));
        if rise < 1.0 {
            act.boost = grounded && !self.o.self_.in_zone && self.energy > 70.0;
            self.surf_pushing = true;
        } else if grounded {
            if self.energy >= (rise / 8.9 * drain + 16.0).min(self.o.self_.max_energy - 3.0) {
                act.jump = true;
                self.surf_pushing = true;
            } else {
                act.mv = [0.0, 0.0];
            }
        } else {
            act.boost = feet < need + 0.8 && self.energy > 1.0;
        }
    }

    /// The id of the largest walkable region (the open map).
    pub(crate) fn main_comp(&mut self) -> i32 {
        if self.main_comp_id != 0 {
            return self.main_comp_id;
        }
        let mut counts: BTreeMap<i32, i64> = BTreeMap::new();
        for &c in self.nav.components() {
            if c != 0 {
                *counts.entry(c).or_insert(0) += 1;
            }
        }
        let (mut best, mut bn) = (0, 0);
        for (&c, &n) in &counts {
            if n > bn {
                best = c;
                bn = n;
            }
        }
        self.main_comp_id = best;
        best
    }

    /// Whether hiding on a roof is what we want right now.
    pub(crate) fn roof_wanted(&self, st: &Stance, zi: &ZoneInfo) -> bool {
        if !self.p.use_roofs || zi.final_ || self.o.zone.next_radius < self.p.roof_min_zone {
            return false;
        }
        if st.name == "duel" {
            return false;
        }
        if !st.stealth && !self.p.roof_in_squad {
            if !(self.p.brawl_roofs && st.avoid && st.name == "ffa") {
                return false;
            }
            if self.has("laser") < 0 && self.o.self_.cubes < self.p.brawl_roof_cubes && self.t < self.p.brawl_roof_after {
                return false;
            }
        }
        true
    }

    /// Runs the roof plan when roofs are wanted; None means "use the ground hide instead".
    pub(crate) fn roof_plan_step(&mut self, wanted: bool) -> Option<(f64, f64, bool)> {
        if !wanted {
            self.roof.state = ROOF_NONE;
            return None;
        }
        self.roof_hold()
    }

    /// The policy's entry to the hideout chooser.
    pub(crate) fn hideout_for(&mut self, st: &Stance, zi: &ZoneInfo, mut allow_roof: bool) -> Option<Hideout> {
        if self.hm.is_none() || !(st.stealth || (self.p.hold_hide_map && st.avoid)) {
            self.roof.want = -1;
            self.roof.spot = -1;
            return None;
        }
        if !st.stealth {
            allow_roof = false;
        }
        match self.hideout_step(zi, allow_roof) {
            None => {
                self.roof.want = -1;
                self.roof.spot = -1;
                None
            }
            Some(ho) => {
                if ho.class == HM_ROOF {
                    self.roof.want = ho.roof;
                    self.roof.spot = ho.cell as i64;
                }
                Some(ho)
            }
        }
    }

    /// The hideout to use right now (re-evaluated every couple of seconds).
    fn hideout_step(&mut self, zi: &ZoneInfo, allow_roof: bool) -> Option<Hideout> {
        self.hm?;
        let mut stale = self.t - self.hide_out_t > 2.0 || !self.hide_out_ok;
        if self.hide_out_ok && !stale {
            let z = &self.o.zone;
            let (x, y) = self.nav.pos(self.hide_out.cell);
            if hyp(x - z.next_center[0], y - z.next_center[1]) > z.next_radius - 4.0 || (self.hide_out.class == HM_ROOF && !allow_roof) {
                stale = true;
            }
        }
        if stale {
            match self.choose_hideout(zi, allow_roof) {
                Some(h) => {
                    self.hide_out = h;
                    self.hide_out_ok = true;
                    self.hide_cell = h.cell as i64;
                }
                None => {
                    self.hide_out = Hideout::default();
                    self.hide_out_ok = false;
                    self.hide_cell = -1;
                }
            }
            self.hide_out_t = self.t;
        }
        if self.hide_out_ok {
            Some(self.hide_out)
        } else {
            None
        }
    }

    /// The "nothing else to do" behaviour when roofs are in play: the goal to walk to and whether we
    /// are settled. None means no roof plan.
    fn roof_hold(&mut self) -> Option<(f64, f64, bool)> {
        let nav = self.nav.clone();
        let (roofs, _) = nav.roofs();
        if self.t - self.roof.step_t > 1.5 {
            self.roof.t0 = self.t;
        }
        self.roof.step_t = self.t;
        let id = self.on_roof();
        if id >= 0 && self.o.self_.grounded {
            let r = &roofs[id as usize];
            let mut stay = self.roof_spot_ok(r) && r.height >= self.p.roof_min_height - 3.0;
            if self.roof.want >= 0 {
                stay = id == self.roof.want && self.roof_spot_ok(r);
            }
            if stay && self.p.roof_stand_check {
                let z = &self.o.zone;
                let (sx, sy) = self.roof_spot(id);
                if hyp(sx - z.next_center[0], sy - z.next_center[1]) > z.next_radius - 4.0 {
                    stay = false;
                    self.roof.ban.insert(id, self.t + 60.0);
                }
            }
            if !stay && self.hold_roof_move {
                stay = true;
            }
            if stay {
                self.roof.state = ROOF_ON;
                self.roof.id = id;
                let (x, y) = self.roof_spot(id);
                return Some((x, y, hyp(x - self.pos[0], y - self.pos[1]) < 2.0));
            }
            self.roof.state = ROOF_NONE;
        }
        if self.roof.state == ROOF_ON {
            self.roof.state = ROOF_NONE;
        }
        if self.roof.state == ROOF_NONE || self.t - self.roof.chosen > 3.0 {
            let found = self.choose_roof();
            self.roof.chosen = self.t;
            match found {
                None => {
                    self.roof.state = ROOF_NONE;
                    return None;
                }
                Some((id, acc)) => {
                    if self.roof.state == ROOF_NONE || id != self.roof.id {
                        self.roof.state = ROOF_APPROACH;
                        self.roof.id = id;
                        self.roof.acc = acc;
                        self.roof.t0 = self.t;
                    }
                }
            }
        }
        let (gx, gy) = nav.pos(self.roof.acc.ground);
        let d = hyp(gx - self.pos[0], gy - self.pos[1]);
        if d < 1.6 && self.on_ground_level() {
            let need = self.roof.acc.climb / 8.9 * self.self_stat("jet_drain", 15.0) + 24.0;
            if self.hold_climb {
                if self.roof.wait_t0 <= 0.0 {
                    self.roof.wait_t0 = self.t;
                }
                if self.t - self.roof.wait_t0 > 4.0 {
                    self.roof.ban.insert(self.roof.id, self.t + 20.0);
                    self.roof.state = ROOF_NONE;
                    self.roof.wait_t0 = 0.0;
                    return None;
                }
                return Some((gx, gy, true));
            }
            self.roof.wait_t0 = 0.0;
            if self.energy >= need {
                self.roof.state = ROOF_CLIMB;
                self.roof.t0 = self.t;
            }
            return Some((gx, gy, true));
        }
        if self.t - self.roof.t0 > 60.0 {
            self.roof.ban.insert(self.roof.id, self.t + 60.0);
            self.roof.state = ROOF_NONE;
            return None;
        }
        Some((gx, gy, false))
    }

    /// The nearest usable place to climb a roof from.
    pub(crate) fn roof_access_for(&self, r: &Roof, limit: f64) -> Option<(RoofAccess, f64)> {
        let comp = self.nav.components();
        let mut acc = RoofAccess::default();
        let mut ad = 1e18;
        for a in &r.access {
            if a.climb > limit || a.climb < 3.0 {
                continue;
            }
            if self.my_comp != 0 && comp[a.ground] != self.my_comp {
                continue;
            }
            if !self.nav.walkable(a.ground, self.radius + 0.2, self.height) {
                continue;
            }
            let (gx, gy) = self.nav.pos(a.ground);
            let d = hyp(gx - self.pos[0], gy - self.pos[1]);
            if d < ad {
                acc = *a;
                ad = d;
            }
        }
        if ad < 1e17 {
            Some((acc, ad))
        } else {
            None
        }
    }
}
