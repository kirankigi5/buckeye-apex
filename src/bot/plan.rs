use super::*;

const SEEN_CELL: f64 = 32.0;

impl Bot {
    /// Evaluates fighting a mech at several ranges: the best advantage ratio and the range band to
    /// hold, after paying for the damage we would take (and deal) while moving to that range.
    pub(crate) fn fight_plan(&self, tr: &Track, d: f64) -> (f64, f64, f64) {
        let hp = if tr.hp <= 0.0 { tr.max_hp.max(1.0) } else { tr.hp };
        let cands = [22.0, 45.0, 75.0, 115.0, 160.0, 200.0];
        let (mut best_adv, mut best_r) = (-1.0, d);
        let mut eval = |r: f64, here: bool| {
            let mine = self.my_dps(r);
            let theirs = self.their_dps(tr, r) * (1.0 - self.armor) * self.dodge_factor(r, tr);
            if mine < 1.0 {
                return;
            }
            let mv = (d - r).abs();
            let (mut cost, mut gain) = (0.0, 0.0);
            if !here && mv > 8.0 {
                let mid = (d + r) / 2.0;
                cost = self.their_dps(tr, mid) * (1.0 - self.armor) * self.dodge_factor(mid, tr) * (mv / 9.0) * 0.8;
                if self.p.plan_v2 && self.real_model() {
                    gain = self.my_dps(mid) * (mv / 9.0) * 0.8;
                }
                if r > d && !tr.npc {
                    cost += 60.0;
                }
            }
            let left = self.hp - cost;
            if left <= 0.0 {
                return;
            }
            let mut hp_t = hp - gain;
            if hp_t < 20.0 {
                hp_t = 20.0;
            }
            let mut a = 10.0;
            if theirs >= 1.0 {
                a = (left / theirs) / (hp_t / mine);
            }
            if here {
                a *= 1.05;
            }
            if a > best_adv {
                best_adv = a;
                best_r = r;
            }
        };
        eval(d, true);
        for r in cands {
            eval(r, false);
        }
        if best_adv < 0.0 {
            return (0.0, 30.0, 55.0);
        }
        let w = 10.0 + best_r * 0.15;
        (best_adv, (best_r - w).max(8.0), best_r + w)
    }

    /// Scales an enemy's expected damage on us at a range.
    pub(crate) fn dodge_factor(&self, _r: f64, tr: &Track) -> f64 {
        if tr.npc {
            0.6
        } else {
            1.0
        }
    }

    // --- exploration

    /// Stamps the coarse cells we can currently see.
    pub(crate) fn mark_seen(&mut self) {
        if self.seen_t.is_empty() {
            self.seen_nx = ((self.w.max_x - self.w.min_x) / SEEN_CELL).ceil() as i64 + 1;
            self.seen_ny = ((self.w.max_y - self.w.min_y) / SEEN_CELL).ceil() as i64 + 1;
            self.seen_t = vec![-1e6; (self.seen_nx * self.seen_ny) as usize];
        }
        const R: f64 = 150.0;
        let mut half = self.h.config.vision.fov_deg * std::f64::consts::PI / 360.0;
        if half <= 0.0 {
            half = std::f64::consts::PI / 3.0;
        }
        let (ex, ey, ez) = (self.pos[0], self.pos[1], self.pos[2] + self.eye_h);
        let i0 = ((ex - R - self.w.min_x) / SEEN_CELL) as i64;
        let i1 = ((ex + R - self.w.min_x) / SEEN_CELL) as i64;
        let j0 = ((ey - R - self.w.min_y) / SEEN_CELL) as i64;
        let j1 = ((ey + R - self.w.min_y) / SEEN_CELL) as i64;
        for j in j0..=j1 {
            if j < 0 || j >= self.seen_ny {
                continue;
            }
            for i in i0..=i1 {
                if i < 0 || i >= self.seen_nx {
                    continue;
                }
                let k = (j * self.seen_nx + i) as usize;
                if self.seen_t[k] as f64 > self.t - 2.0 {
                    continue;
                }
                let cx = self.w.min_x + (i as f64 + 0.5) * SEEN_CELL;
                let cy = self.w.min_y + (j as f64 + 0.5) * SEEN_CELL;
                let d = hyp(cx - ex, cy - ey);
                if d > R {
                    continue;
                }
                if d > 40.0 && ang_diff((cy - ey).atan2(cx - ex), self.aim[0]).abs() > half {
                    continue;
                }
                let gz = self.w.terrain_z(cx, cy) + 2.0;
                if d < 20.0 || self.w.visible(ex, ey, ez, cx, cy, gz) {
                    self.seen_t[k] = self.t as f32;
                }
            }
        }
    }

    fn seen_center(&self, k: i64) -> (f64, f64) {
        let (i, j) = (k % self.seen_nx, k / self.seen_nx);
        (self.w.min_x + (i as f64 + 0.5) * SEEN_CELL, self.w.min_y + (j as f64 + 0.5) * SEEN_CELL)
    }

    /// A ground position worth looking at: not seen for a while, inside the circle, close.
    pub(crate) fn roam_goal(&mut self) -> Option<(f64, f64)> {
        if self.seen_t.is_empty() {
            return None;
        }
        if self.roam_cell >= 0 && self.t - self.roam_t < 12.0 && (self.seen_t[self.roam_cell as usize] as f64) < self.t - 20.0 {
            let (x, y) = self.seen_center(self.roam_cell);
            let c = self.nav.nearest(x, y, self.radius + 0.3, self.height, 8);
            if c >= 0 {
                return Some(self.nav.pos(c as usize));
            }
        }
        let z = self.o.zone.clone();
        let (mut best, mut bs) = (-1i64, 1e18);
        for j in 0..self.seen_ny {
            for i in 0..self.seen_nx {
                let k = j * self.seen_nx + i;
                let age = self.t - self.seen_t[k as usize] as f64;
                if age < 45.0 {
                    continue;
                }
                let (x, y) = self.seen_center(k);
                if !self.w.in_bounds(x, y) {
                    continue;
                }
                if hyp(x - z.next_center[0], y - z.next_center[1]) > z.next_radius - 15.0 {
                    continue;
                }
                let nk = self.nav.index(x, y);
                if self.nav.solid[nk] {
                    continue;
                }
                if let Some(&until) = self.ban_until.get(&(-(k + 1))) {
                    if self.t < until {
                        continue;
                    }
                }
                if !self.reachable(x, y) {
                    self.seen_t[k as usize] = self.t as f32;
                    continue;
                }
                let d = hyp(x - self.pos[0], y - self.pos[1]);
                let mut s = d - age.min(300.0) * 0.15;
                if !self.ffa && self.p.practice_rim > 0.0 && z.next_radius >= 180.0 {
                    let dn = hyp(x - z.next_center[0], y - z.next_center[1]);
                    s += (dn - z.next_radius * 0.88).abs() * self.p.practice_rim;
                }
                if s < bs {
                    best = k;
                    bs = s;
                }
            }
        }
        if best < 0 {
            return None;
        }
        self.roam_cell = best;
        self.roam_t = self.t;
        let (x, y) = self.seen_center(best);
        let c = self.nav.nearest(x, y, self.radius + 0.3, self.height, 8);
        if c >= 0 {
            return Some(self.nav.pos(c as usize));
        }
        self.seen_t[best as usize] = self.t as f32;
        None
    }
}
