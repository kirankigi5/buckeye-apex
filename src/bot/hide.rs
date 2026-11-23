use super::strategy::{Stance, Threat, ZoneInfo};
use super::*;

impl Bot {
    /// The free-for-all stance: armed, farming, fighting with an edge and declining coin flips.
    pub(crate) fn brawl_stance(&self, name: &'static str) -> Stance {
        Stance {
            name,
            engage_ratio: self.p.brawl_engage,
            disengage: self.p.brawl_disengage,
            hunt: self.p.brawl_hunt,
            stealth: false,
            avoid: self.p.avoid_adv > 0.0,
            loot_radius: self.p.brawl_loot_radius,
            want_cubes: self.p.brawl_want_cubes,
        }
    }

    /// Whether the measured (team bot) hit rates apply: in a free-for-all, and until we know better.
    pub(crate) fn real_model(&self) -> bool {
        if self.p.hit_model < 1 {
            return false;
        }
        !self.p.practice_aware || self.ffa || self.open_phase()
    }

    /// Whether we still do not know what kind of lobby this is.
    pub(crate) fn open_phase(&self) -> bool {
        self.t < self.p.open_secs || (self.p.credit_open_s > 0.0 && self.t - self.credit_absorb_t < self.p.credit_open_s)
    }

    /// Updates what we think the lobby is and returns the matching stance.
    pub(crate) fn belief(&mut self) -> Stance {
        let total = stat(Some(&self.h.config.match_), "mechs", 16.0) as i64;
        let dead = total - self.o.alive;
        if dead > self.dead_seen {
            let mut n = dead - self.dead_seen;
            self.dead_seen = dead;
            if self.kills > self.kills_seen {
                n -= self.kills - self.kills_seen;
                self.kills_seen = self.kills;
            } else if self.t - self.credit_t < 6.0 {
                let mut mine = true;
                if self.p.credit_fix {
                    if let Some(tr) = self.tracks.get(&self.credit_id) {
                        if tr.listed && tr.hp > 0.0 {
                            mine = false;
                        }
                    }
                }
                if mine {
                    n -= 1;
                    self.credit_absorb_t = self.t;
                }
            }
            if n > 0 {
                self.unexplained += n;
            }
            if (self.unexplained >= 1 && self.t < 35.0) || (self.unexplained >= 2 && self.t < 110.0) || self.unexplained >= 3 {
                self.ffa = true;
            }
        }
        let alive = self.o.alive;
        if self.p.force_ffa {
            self.ffa = true;
        }
        if self.p.beam_ffa && self.ffa_beams >= 2 {
            self.ffa = true;
        }
        if self.p.slot_rule && self.h.slot != 0 {
            self.ffa = true;
        }
        if alive <= 2 {
            return Stance { name: "duel", engage_ratio: 0.0, disengage: 0.45, hunt: true, loot_radius: 150.0, want_cubes: 40, ..Default::default() };
        }
        if self.ffa && self.p.practice_aware {
            if self.p.brawl {
                return self.brawl_stance("ffa");
            }
            return Stance {
                name: "ffa",
                engage_ratio: self.p.ffa_engage,
                disengage: self.p.ffa_disengage,
                hunt: false,
                stealth: true,
                loot_radius: self.p.ffa_loot_radius,
                want_cubes: self.p.ffa_want_cubes,
                ..Default::default()
            };
        }
        if self.open_phase() && self.p.practice_aware {
            if self.p.brawl {
                return self.brawl_stance("open");
            }
            return Stance { name: "open", engage_ratio: 1.5, disengage: 0.9, hunt: false, stealth: true, loot_radius: 110.0, want_cubes: self.p.want_cubes, ..Default::default() };
        }
        if self.p.cautious_t > 0.0
            && self.p.practice_aware
            && (self.t >= self.p.cautious_t || (self.o.zone.next_radius > 0.0 && self.o.zone.next_radius <= 72.0 && self.t >= 180.0))
        {
            return self.brawl_stance("late");
        }
        Stance {
            name: "squad",
            engage_ratio: self.p.engage_ratio,
            disengage: self.p.disengage_rat,
            hunt: self.p.hunt,
            loot_radius: self.p.loot_radius,
            want_cubes: self.p.want_cubes,
            ..Default::default()
        }
    }

    /// A low-exposure place inside the circle we must end up in (-1: none). Lower score is better.
    pub(crate) fn choose_hide(&self, _zi: &ZoneInfo) -> i64 {
        let z = &self.o.zone;
        let exp = self.nav.exposure();
        let comp = self.nav.components();
        let mut my_comp = 0;
        let c = self.nav.nearest(self.pos[0], self.pos[1], 2.9, 9.0, 6);
        if c >= 0 {
            my_comp = comp[c as usize];
        }
        let margin = (14.0f64).min(z.next_radius * 0.25);
        // the region to hide in: the next circle shrunk by a margin, or, in the last circle (it
        // closes to a point), the inner half of the current one
        let (hc, h_lim) = if z.next_radius < 12.0 {
            ([z.center[0], z.center[1]], z.radius * 0.5)
        } else {
            (z.next_center, z.next_radius - margin)
        };
        let mut cands: Vec<(usize, f64)> = Vec::new();
        for j in 0..exp.ny {
            for i in 0..exp.nx {
                let e = exp.v[j * exp.nx + i];
                if e > 1.5 {
                    continue;
                }
                let k = exp.cell(i, j);
                if my_comp != 0 && comp[k] != my_comp {
                    continue;
                }
                let (x, y) = self.nav.pos(k);
                let dn = hyp(x - hc[0], y - hc[1]);
                if dn > h_lim {
                    continue;
                }
                if !self.nav.walkable(k, self.radius + 0.4, self.height) {
                    continue;
                }
                let d = hyp(x - self.pos[0], y - self.pos[1]);
                if d > 320.0 {
                    continue;
                }
                let mut s = e as f64 * 100.0 + d * self.hide_dist_w();
                if self.late {
                    // endgame: the centre is the last place the collapsing circle burns
                    s += dn * 0.6;
                } else if z.next_radius > 60.0 && dn < z.next_radius * 0.35 {
                    s += 10.0;
                }
                if self.ffa && self.p.edge_bias > 0.0 && z.next_radius > 60.0 {
                    s -= self.p.edge_bias * 30.0 * dn / z.next_radius;
                    if self.p.edge_band > 0.0 && dn < z.next_radius * self.p.edge_band {
                        s += 60.0;
                    }
                }
                if !self.ffa && self.p.practice_rim > 0.0 && z.next_radius >= 180.0 {
                    let want_r = z.next_radius * 0.88;
                    s += (dn - want_r).abs() * self.p.practice_rim;
                }
                s += self.nav.clear[k] as f64 * 0.8;
                if k as i64 == self.hold_cell {
                    s -= 9.0;
                }
                cands.push((k, s));
            }
        }
        if cands.is_empty() {
            return -1;
        }
        cands.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        cands.truncate(60);
        let (mut best, mut bs) = (-1i64, 1e18);
        for &(cell, score) in &cands {
            let (x, y) = self.nav.pos(cell);
            let gz = self.nav.z[cell] as f64;
            let mut s = score;
            for tr in self.tracks.values() {
                if tr.npc || self.t - tr.last_seen > 25.0 {
                    continue;
                }
                let d = hyp(tr.pos[0] - x, tr.pos[1] - y);
                if d < 90.0 {
                    s += (90.0 - d) * 0.35;
                }
                if d < 260.0 && self.t - tr.last_seen < 8.0 {
                    let sc = tr.scale(self.size_per_cube);
                    if self.w.visible(tr.pos[0], tr.pos[1], tr.pos[2] + 8.0 * sc, x, y, gz + self.height * 0.6) {
                        s += 30.0;
                    }
                }
            }
            for dr in &self.o.drops {
                if hyp(dr.pos[0] - x, dr.pos[1] - y) < 70.0 {
                    s += 15.0;
                }
            }
            if s < bs {
                best = cell as i64;
                bs = s;
            }
        }
        best
    }

    pub(crate) fn hide_dist_w(&self) -> f64 {
        if self.st.stealth {
            0.16
        } else {
            0.07
        }
    }

    /// Indices of the enemy mechs that currently have a line to us (and are close enough to matter).
    pub(crate) fn seen_by(&self, ths: &[Threat], max_d: f64) -> Vec<usize> {
        (0..ths.len()).filter(|&i| ths[i].tr.los && ths[i].tr.listed && ths[i].d < max_d).collect()
    }
}
