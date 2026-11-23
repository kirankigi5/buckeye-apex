//! Cover that lasts: from the very edge of a shadow (find_cover) on to a place that stays hidden when
//! the attacker shifts sideways, and, hurt with the attacker close, clearly farther from it.

use super::strategy::{Stance, Threat};
use super::*;

impl Bot {
    /// Whether the opening rule applies (first ShySecs of a known free-for-all).
    pub(crate) fn shy_phase(&self, st: &Stance) -> bool {
        self.p.shy_secs > 0.0 && self.ffa && st.avoid && self.t < self.p.shy_secs
    }

    /// Whether a mech seen in the last secs seconds within max_range has a clear line to where we stand.
    pub(crate) fn exposed_to(&self, secs: f64, max_range: f64) -> bool {
        for tr in self.tracks.values() {
            if tr.npc || self.t - tr.last_seen > secs {
                continue;
            }
            if hyp(tr.pos[0] - self.pos[0], tr.pos[1] - self.pos[1]) > max_range {
                continue;
            }
            let sc = tr.scale(self.size_per_cube);
            if self.seen_from([tr.pos[0], tr.pos[1], tr.pos[2] + 8.0 * sc], self.pos[0], self.pos[1], self.pos[2]) {
                return true;
            }
        }
        false
    }

    /// Whether we have seen another mech in the last secs seconds.
    pub(crate) fn mech_seen_within(&self, secs: f64) -> bool {
        self.tracks.values().any(|tr| !tr.npc && self.t - tr.last_seen <= secs)
    }

    /// Whether there is somewhere out of every threat's sight within a few seconds' run (kept 0.5 s).
    pub(crate) fn cover_close(&mut self, ths: &[Threat], tgt: Option<&Threat>) -> bool {
        if self.t - self.near_cover_t < 0.5 {
            return self.near_cover_ok;
        }
        let mut reach = self.p.retreat_run_max;
        if let Some(tg) = tgt {
            if tg.tr.laser && self.has("laser") < 0 && self.p.retreat_run_laser > reach {
                reach = self.p.retreat_run_laser;
            }
        }
        self.near_cover_t = self.t;
        self.near_cover_ok = self.find_cover(ths, reach) >= 0;
        self.near_cover_ok
    }

    /// The nearest cell hidden from every recent threat and from CoverMargin metres to either side of
    /// it, with the straight way there out of sight too (-1: where we stand qualifies, or nothing does).
    pub(crate) fn deeper_cover(&self, ths: &[Threat], max_cost: f64, farther: bool) -> i64 {
        let mut eyes: Vec<[f64; 3]> = Vec::new();
        let mut d0 = 1e9;
        for th in ths {
            let tr = &th.tr;
            if (self.t - tr.last_seen > 6.0 && !th.phantom) || th.d > 260.0 {
                continue;
            }
            let sc = tr.scale(self.size_per_cube);
            eyes.push([tr.pos[0], tr.pos[1], tr.pos[2] + 8.0 * sc]);
            let d = hyp(tr.pos[0] - self.pos[0], tr.pos[1] - self.pos[1]);
            if d < d0 {
                d0 = d;
            }
        }
        if eyes.is_empty() {
            return -1;
        }
        let need_far = farther && self.p.run_on_range > 0.0 && d0 <= self.p.run_on_range;
        let margin = self.p.cover_margin;
        if margin <= 0.0 && !need_far {
            return -1;
        }
        let hidden = |x: f64, y: f64, gz: f64| !eyes.iter().any(|&e| self.seen_from(e, x, y, gz));
        let robust = |x: f64, y: f64, gz: f64| {
            for &e in &eyes {
                if self.seen_from(e, x, y, gz) {
                    return false;
                }
                if margin <= 0.0 {
                    continue;
                }
                let (dx, dy) = (x - e[0], y - e[1]);
                let l = hyp(dx, dy);
                if l < 1.0 {
                    return false;
                }
                let (px, py) = (-dy / l * margin, dx / l * margin);
                if self.seen_from([e[0] + px, e[1] + py, e[2]], x, y, gz) || self.seen_from([e[0] - px, e[1] - py, e[2]], x, y, gz) {
                    return false;
                }
            }
            true
        };
        if !need_far && robust(self.pos[0], self.pos[1], self.pos[2]) {
            return -1;
        }
        let z = &self.o.zone;
        let nav = &self.nav;
        let mut found = -1i64;
        let mut checked = 0;
        nav.flood(self.pos[0], self.pos[1], self.radius + 0.15, self.height, max_cost, None, &mut |k, _| {
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
            if need_far {
                let de = eyes.iter().map(|e| hyp(x - e[0], y - e[1])).fold(1e9, f64::min);
                if de < d0 + self.p.run_on_gain {
                    return true;
                }
            }
            checked += 1;
            if checked > 300 {
                return false;
            }
            if !robust(x, y, nav.z[k] as f64) {
                return true;
            }
            for f in [0.25, 0.5, 0.75] {
                let (mx, my) = (self.pos[0] + (x - self.pos[0]) * f, self.pos[1] + (y - self.pos[1]) * f);
                let mk = nav.index(mx, my);
                if nav.solid[mk] {
                    continue;
                }
                if !hidden(mx, my, nav.z[mk] as f64) {
                    return true;
                }
            }
            found = k as i64;
            false
        });
        found
    }
}
