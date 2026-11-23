//! Fighting with a laser from behind cover: charge hidden, step out as the charge completes, let go,
//! and be back behind cover while the other side is still charging its answer.

use super::strategy::Threat;
use super::*;

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct PeekPlan {
    pub t: f64,
    pub tgt: i64,
    pub roof: i64,
    pub duck: i64,
    pub fire: i64,
    pub ok: bool,
}

impl Bot {
    /// Works out (or reuses) the duck and fire cells against a target. roof_id is the roof we stand
    /// on and mean to stay on, or -1.
    pub(crate) fn peek_plan_for(&mut self, tgt: &Threat, ths: &[Threat], roof_id: i64) -> bool {
        if self.t - self.peek.t < 0.4 && self.peek.tgt == tgt.tr.id && self.peek.roof == roof_id {
            return self.peek.ok;
        }
        let mut prev_fire = -1;
        if self.peek.ok && self.peek.tgt == tgt.tr.id && self.peek.roof == roof_id {
            prev_fire = self.peek.fire;
        }
        self.peek = PeekPlan { t: self.t, tgt: tgt.tr.id, roof: roof_id, duck: -1, fire: -1, ok: false };
        let tr = &tgt.tr;
        let sc = tr.scale(self.size_per_cube);
        let (tx, ty, tz) = (tr.pos[0], tr.pos[1], tr.pos[2] + 4.5 * sc);
        let duck = if roof_id >= 0 { self.roof_cover(roof_id, ths) } else { self.find_cover(ths, self.p.peek_cover) };
        if duck < 0 {
            return false;
        }
        let nav = self.nav.clone();
        let (dx, dy) = nav.pos(duck as usize);
        let (mut best, mut bd) = (-1i64, 1e18);
        let mut consider = |k: usize, z: f64| {
            let (x, y) = nav.pos(k);
            let mut d = hyp(x - dx, y - dy);
            if d < 1.5 || d > self.p.peek_reach {
                return;
            }
            if k as i64 == prev_fire {
                d -= 2.5;
            }
            if d >= bd {
                return;
            }
            if !self.w.visible(x, y, z + self.muzzle_h, tx, ty, tz) {
                return;
            }
            best = k as i64;
            bd = d;
        };
        if roof_id >= 0 {
            let (roofs, roof_of) = nav.roofs();
            let dist = nav.roof_dist();
            let r = &roofs[roof_id as usize];
            for &c in &r.cells {
                let k = c as usize;
                if roof_of[k] as i64 != roof_id || dist[k] < 2 {
                    continue;
                }
                consider(k, r.z);
            }
        } else {
            let (ci, cj) = ((duck as usize % nav.nx) as i64, (duck as usize / nav.nx) as i64);
            let rad = (self.p.peek_reach / nav.cell) as i64 + 1;
            for dj in -rad..=rad {
                for di in -rad..=rad {
                    let (i, j) = (ci + di, cj + dj);
                    if i < 0 || j < 0 || i >= nav.nx as i64 || j >= nav.ny as i64 {
                        continue;
                    }
                    let k = (j * nav.nx as i64 + i) as usize;
                    if !nav.walkable(k, self.radius + 0.3, self.height) || !self.same_comp(k as i64) {
                        continue;
                    }
                    let (x, y) = nav.pos(k);
                    if !nav.walk_line(dx, dy, x, y, self.radius + 0.15, self.height) {
                        continue;
                    }
                    consider(k, nav.z[k] as f64);
                }
            }
        }
        if best < 0 {
            return false;
        }
        self.peek.duck = duck;
        self.peek.fire = best;
        self.peek.ok = true;
        true
    }

    /// Whether to stand at the fire cell or the duck cell this decision, and whether to start
    /// charging now (hidden).
    pub(crate) fn peek_step(&self, ths: &[Threat], li: usize, hp_frac: f64) -> (i64, bool) {
        let sl = &self.slots[li];
        let full = self.slot_stat(li, "charge", 0.75);
        let (fx, fy) = self.nav.pos(self.peek.fire as usize);
        let (kx, ky) = self.nav.pos(self.peek.duck as usize);
        let travel = hyp(fx - kx, fy - ky) / (self.walk * 0.5) + 0.15;
        let at_duck = hyp(self.pos[0] - kx, self.pos[1] - ky) < 4.0;
        let at_fire = hyp(self.pos[0] - fx, self.pos[1] - fy) < 4.0;
        let threatened = ths.iter().any(|th| th.tr.laser && th.tr.charging && th.tr.los && self.t - th.tr.aim_at_me_t < 0.3);
        let mut cell = self.peek.duck;
        let mut charge = false;
        if hp_frac < 0.2 || threatened {
            // stay hidden
        } else if sl.charge > 0.0 && sl.charge >= full - travel {
            cell = self.peek.fire;
        } else if sl.charge <= 0.0 && sl.cooldown <= 0.1 && (at_duck || at_fire) {
            charge = true;
        }
        (cell, charge)
    }
}
