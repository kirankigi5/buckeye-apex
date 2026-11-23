//! How to lead a mech with the gun: learned per opponent from "virtual shots" (see the original
//! notes: about half the field jitters on the spot under fire, the other half keeps walking).

use super::combat::FIRE_LATENCY;
use super::*;

impl Bot {
    /// Stores the current observation of a listed mech and scores one virtual shot.
    pub(crate) fn lead_record(&self, tr: &mut Track, facing: bool) {
        if tr.hn > 0 {
            let last = tr.hist[(tr.hi + TRK_HIST - 1) % TRK_HIST];
            if self.t - last.t > 0.25 || self.t <= last.t {
                if self.t <= last.t {
                    return;
                }
                tr.hn = 0;
                tr.hi = 0;
            }
        }
        tr.hist[tr.hi] = TrkSample { t: self.t, x: tr.pos[0], y: tr.pos[1], vx: tr.vel[0], vy: tr.vel[1], facing };
        tr.hi = (tr.hi + 1) % TRK_HIST;
        if tr.hn < TRK_HIST {
            tr.hn += 1;
        }
        let d = hyp(tr.pos[0] - self.pos[0], tr.pos[1] - self.pos[1]);
        if !(55.0..=240.0).contains(&d) {
            return;
        }
        let mut gun_spd = self.wstat("gun", "speed", 252.0);
        if self.p.smart_fire_v2 {
            let gi = self.has("gun");
            if gi >= 0 {
                gun_spd = self.slot_stat(gi as usize, "speed", gun_spd);
            }
        }
        let tf = (d - 3.0).max(0.0) / gun_spd.max(1.0);
        let s = match self.lead_sample(tr, self.t - (FIRE_LATENCY + tf)) {
            Some(s) if s.facing => s,
            _ => return,
        };
        let side = |px: f64, py: f64| {
            let (ax, ay) = norm2(px - self.pos[0], py - self.pos[1]);
            (-(tr.pos[0] - self.pos[0]) * ay + (tr.pos[1] - self.pos[1]) * ax).abs()
        };
        let k = self.lead_persist_at(tr, s.t);
        let lead = (FIRE_LATENCY + tf * self.lead_damp(tf)) * k;
        let m_none = side(s.x, s.y).min(6.0);
        let m_lead = side(s.x + s.vx * lead, s.y + s.vy * lead).min(6.0);
        const KEEP: f64 = 0.997;
        tr.lead_n = tr.lead_n * KEEP + 1.0;
        tr.miss_none = tr.miss_none * KEEP + m_none;
        tr.miss_lead = tr.miss_lead * KEEP + m_lead;
    }

    /// The stored sample closest to time t (None if none within 0.07 s).
    fn lead_sample(&self, tr: &Track, t: f64) -> Option<TrkSample> {
        let mut best = None;
        let mut bd = 0.07;
        for i in 0..tr.hn {
            let s = tr.hist[(tr.hi + TRK_HIST - 1 - i) % TRK_HIST];
            let dt = (s.t - t).abs();
            if dt < bd {
                best = Some(s);
                bd = dt;
            }
            if s.t < t - 0.1 {
                break;
            }
        }
        best
    }

    /// How straight the mech had been moving over the 1.2 s before time t: 1 for a steady walk (or
    /// when we cannot tell), 0 for jitter on the spot.
    fn lead_persist_at(&self, tr: &Track, t: f64) -> f64 {
        if !self.p.lead_persist {
            return 1.0;
        }
        let (mut first, mut last): (Option<TrkSample>, Option<TrkSample>) = (None, None);
        let mut n = 0;
        let mut sp = 0.0;
        for i in 0..tr.hn {
            let s = tr.hist[(tr.hi + TRK_HIST - 1 - i) % TRK_HIST];
            if s.t > t + 0.01 {
                continue;
            }
            if s.t < t - 1.25 {
                break;
            }
            if last.is_none() {
                last = Some(s);
            }
            first = Some(s);
            sp += hyp(s.vx, s.vy);
            n += 1;
        }
        let (first, last) = match (first, last) {
            (Some(f), Some(l)) => (f, l),
            _ => return 1.0,
        };
        if n < 8 || last.t - first.t < 0.7 {
            return 1.0;
        }
        let mean = hyp(last.x - first.x, last.y - first.y) / (last.t - first.t);
        let pers = mean / (sp / n as f64).max(0.5);
        clamp((pers - 0.2) / 0.5, 0.0, 1.0)
    }

    /// The multiplier to apply to the velocity lead when shooting the gun at this mech.
    pub(crate) fn gun_lead(&self, tr: &Track) -> f64 {
        if tr.npc {
            return 1.0;
        }
        if self.p.lead_cum && tr.lead_n >= self.p.lead_cum_min {
            let margin = 0.2 * tr.lead_n;
            if tr.miss_none < tr.miss_lead - margin {
                return 0.0;
            }
            if tr.miss_lead < tr.miss_none - margin {
                return 1.0;
            }
        }
        self.lead_persist_at(tr, self.t)
    }
}
