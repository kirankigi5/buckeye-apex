//! The hide map, computed offline from the arena: for every place a mech can stand (4 m lattice),
//! what share of the ground within vision range can see it and what share has a clear line of fire
//! to its centre. Hideouts are picked by these numbers.

use super::strategy::ZoneInfo;
use super::*;
use std::io::Read;
use std::sync::OnceLock;

static HIDE_MAP_GZ: &[u8] = include_bytes!("hidemap.gz");

pub(crate) const HM_GROUND: u8 = 0;
pub(crate) const HM_ENCLOSED: u8 = 1;
pub(crate) const HM_ROOF: u8 = 2;
pub(crate) const HM_NONE: u8 = 255;

#[derive(Default)]
pub(crate) struct HideMap {
    ok: bool,
    nx: usize,
    ny: usize,
    step: usize,
    lw: usize,
    lh: usize,
    class: Vec<u8>,
    seen: Vec<f32>,
    hit: Vec<f32>,
    cells: Vec<i32>,
}

fn parse_hide_map(gz: &[u8]) -> HideMap {
    let mut h = HideMap::default();
    let mut raw = Vec::new();
    if flate2::read::GzDecoder::new(gz).read_to_end(&mut raw).is_err() || raw.len() < 24 || &raw[..4] != b"HM3\n" {
        return h;
    }
    let u = |i: usize| i32::from_le_bytes([raw[4 + 4 * i], raw[5 + 4 * i], raw[6 + 4 * i], raw[7 + 4 * i]]) as i64;
    let (nx, ny, step, lw, lh) = (u(0), u(1), u(2), u(3), u(4));
    if nx < 0 || ny < 0 || step <= 0 || lw <= 0 || lh <= 0 {
        return h;
    }
    h.nx = nx as usize;
    h.ny = ny as usize;
    h.step = step as usize;
    h.lw = lw as usize;
    h.lh = lh as usize;
    let body = &raw[24..];
    let n = h.lw * h.lh;
    if body.len() < n * 3 {
        return h;
    }
    h.class = vec![0; n];
    h.seen = vec![0.0; n];
    h.hit = vec![0.0; n];
    for i in 0..n {
        h.class[i] = body[i * 3];
        let (s, t) = (body[i * 3 + 1] as f32 / 255.0, body[i * 3 + 2] as f32 / 255.0);
        h.seen[i] = s * s;
        h.hit[i] = t * t;
        if h.class[i] != HM_NONE {
            let (li, lj) = (i % h.lw, i / h.lw);
            let (ci, cj) = (li * h.step + h.step / 2, lj * h.step + h.step / 2);
            if ci < h.nx && cj < h.ny {
                h.cells.push((cj * h.nx + ci) as i32);
            }
        }
    }
    h.ok = true;
    h
}

/// The embedded map if it fits this arena (None otherwise).
pub(crate) fn get_hide_map(nav: &Nav) -> Option<&'static HideMap> {
    static DATA: OnceLock<HideMap> = OnceLock::new();
    let h = DATA.get_or_init(|| parse_hide_map(HIDE_MAP_GZ));
    if !h.ok || h.nx != nav.nx || h.ny != nav.ny {
        return None;
    }
    let (mut bad, mut n) = (0, 0);
    let mut i = 0;
    while i < h.cells.len() {
        let k = h.cells[i] as usize;
        let cl = h.class[h.lat(k)];
        n += 1;
        if (cl == HM_ROOF) != nav.solid[k] {
            bad += 1;
        }
        i += 97;
    }
    if n == 0 || bad * 20 > n {
        return None;
    }
    Some(h)
}

impl HideMap {
    fn lat(&self, k: usize) -> usize {
        (k / self.nx / self.step) * self.lw + (k % self.nx) / self.step
    }

    /// The class and the seen-from / hittable-from fractions for the lattice cell holding nav cell k.
    fn at(&self, k: usize) -> (u8, f64, f64) {
        let i = self.lat(k);
        if i >= self.class.len() {
            return (HM_NONE, 1.0, 1.0);
        }
        (self.class[i], self.seen[i] as f64, self.hit[i] as f64)
    }
}

/// A place to wait out the match.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Hideout {
    pub cell: usize,
    pub class: u8,
    pub z: f64,
    pub roof: i64,
    pub score: f64,
}

impl Bot {
    fn hide_danger(&self, seen: f64, hit: f64) -> f64 {
        100.0 * (self.p.hide_seen_w * seen + self.p.hide_hit_w * hit)
    }

    /// The least exposed reachable place inside the circle we must end up in. Lower score is better.
    pub(crate) fn choose_hideout(&mut self, _zi: &ZoneInfo, allow_roof: bool) -> Option<Hideout> {
        let hm = self.hm?;
        let walk_comp = if self.my_comp == 0 { self.main_comp() } else { self.my_comp };
        let nav = self.nav.clone();
        let z = self.o.zone.clone();
        let comp = nav.components();
        let (roofs, roof_of) = nav.roofs();
        let roof_dist = nav.roof_dist();
        let margin = (14.0f64).min(z.next_radius * 0.25);
        let limit = self.max_climb().min(self.p.roof_max_climb);
        let my_roof = if self.o.self_.grounded { self.on_roof() } else { -1 };
        const KEEP: usize = 48;
        let mut short: Vec<Hideout> = Vec::new();
        let mut worst = 1e18;
        let mut roof_acc: Vec<Option<Option<(crate::world::RoofAccess, f64)>>> = if allow_roof { vec![None; roofs.len()] } else { Vec::new() };
        for &c in &hm.cells {
            let k = c as usize;
            let (x, y) = nav.pos(k);
            if hyp(x - z.next_center[0], y - z.next_center[1]) > z.next_radius - margin {
                continue;
            }
            let mut d = hyp(x - self.pos[0], y - self.pos[1]);
            if d > 380.0 {
                continue;
            }
            let (cl, seen, hit) = hm.at(k);
            let mut s = self.hide_danger(seen, hit);
            let mut ho = Hideout { cell: k, class: cl, roof: -1, ..Default::default() };
            match cl {
                HM_GROUND | HM_ENCLOSED => {
                    if comp[k] != walk_comp || !nav.walkable(k, self.radius + 0.4, self.height) {
                        continue;
                    }
                    ho.z = nav.z[k] as f64;
                    if my_roof >= 0 {
                        s += 6.0;
                    }
                }
                HM_ROOF => {
                    if !allow_roof {
                        continue;
                    }
                    let id = roof_of[k];
                    if id < 0 || roof_dist[k] < 2 {
                        continue;
                    }
                    let id = id as i64;
                    if self.roof.ban.get(&id).is_some_and(|&until| self.t < until) {
                        continue;
                    }
                    let r = &roofs[id as usize];
                    ho.z = r.z;
                    ho.roof = id;
                    if id != my_roof {
                        let ra = roof_acc[id as usize].get_or_insert_with(|| self.roof_access_for(r, limit));
                        match ra {
                            None => continue,
                            Some((acc, rd)) => {
                                let (gx, gy) = nav.pos(acc.ground);
                                d = *rd + hyp(x - gx, y - gy);
                                s += acc.climb * self.p.hide_climb_w;
                            }
                        }
                    }
                }
                _ => continue,
            }
            s += d * self.p.hide_dist_w;
            if k as i64 == self.hide_cell {
                s -= self.p.hide_stick;
            }
            ho.score = s;
            if short.len() < KEEP {
                short.push(ho);
                if short.len() == KEEP {
                    worst = short.iter().map(|h| h.score).fold(-1e18, f64::max);
                }
                continue;
            }
            if s >= worst {
                continue;
            }
            let mut wi = 0;
            for i in 0..short.len() {
                if short[i].score > short[wi].score {
                    wi = i;
                }
            }
            short[wi] = ho;
            worst = short.iter().map(|h| h.score).fold(-1e18, f64::max);
        }
        if short.is_empty() {
            return None;
        }
        let mut best = 0usize;
        for i in 0..short.len() {
            let h = short[i];
            let (x, y) = nav.pos(h.cell);
            let mut s = h.score;
            for tr in self.tracks.values() {
                if tr.npc || self.t - tr.last_seen > 25.0 {
                    continue;
                }
                let d = hyp(tr.pos[0] - x, tr.pos[1] - y);
                if d < 90.0 && (tr.pos[2] - h.z).abs() < 12.0 {
                    s += (90.0 - d) * 0.35;
                }
                if d < 260.0 && self.t - tr.last_seen < 8.0 {
                    let sc = tr.scale(self.size_per_cube);
                    if self.w.visible(tr.pos[0], tr.pos[1], tr.pos[2] + 7.0 * sc, x, y, h.z + self.height * 0.5) {
                        s += 40.0;
                    } else if self.w.visible(tr.pos[0], tr.pos[1], tr.pos[2] + 8.0 * sc, x, y, h.z + self.height) {
                        s += 12.0;
                    }
                }
            }
            for dr in &self.o.drops {
                if hyp(dr.pos[0] - x, dr.pos[1] - y) < 70.0 {
                    s += 15.0;
                }
            }
            short[i].score = s;
            if i == 0 || s < short[best].score {
                best = i;
            }
        }
        Some(short[best])
    }
}
