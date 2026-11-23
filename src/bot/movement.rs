use super::*;
use crate::proto::Action;

/// An enemy projectile we may have to dodge.
#[derive(Clone, Copy, Default)]
pub(crate) struct Incoming {
    pub mortar: bool,
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub max_t: f64,
    pub impact: [f64; 3],
    pub dmg: f64,
    pub splash: f64,
}

pub(crate) type PosCost = Rc<dyn Fn(f64, f64) -> f64>;

/// What the strategy layer asks the movement planner for.
#[derive(Default, Clone)]
pub(crate) struct SteerReq {
    pub want_x: f64,
    pub want_y: f64,
    pub want_w: f64,
    pub boost: bool,
    pub slow: bool,
    pub pos_cost: Option<PosCost>,
    pub hold: bool,
    pub threats: Vec<Incoming>,
    pub look_ticks: usize,
    pub dodge_boost: bool,
    pub fence_roof: bool,
    pub zone_free: bool,
}

fn dirs16() -> [[f64; 2]; 16] {
    let mut d = [[0.0; 2]; 16];
    for (i, v) in d.iter_mut().enumerate() {
        let a = i as f64 * std::f64::consts::PI / 8.0;
        *v = [a.cos(), a.sin()];
    }
    d
}

impl Bot {
    /// The visible projectiles that could matter to us in the next couple of seconds.
    pub(crate) fn gather_incoming(&self) -> Vec<Incoming> {
        let mut out = Vec::new();
        let g = self.wstat("mortar", "gravity", 22.0);
        for p in &self.o.projectiles {
            let sp = (p.vel[0] * p.vel[0] + p.vel[1] * p.vel[1] + p.vel[2] * p.vel[2]).sqrt();
            if sp < 1.0 {
                continue;
            }
            if p.kind == "mortar" {
                let (mut x, mut y, mut z) = (p.pos[0], p.pos[1], p.pos[2]);
                let (vx, vy, mut vz) = (p.vel[0], p.vel[1], p.vel[2]);
                let mut tt = 0.0;
                const H: f64 = 0.05;
                let mut hit = false;
                while tt < 7.0 {
                    let (nx, ny, nz) = (x + vx * H, y + vy * H, z + vz * H - 0.5 * g * H * H);
                    if !self.w.visible(x, y, z, nx, ny, nz) {
                        hit = true;
                        break;
                    }
                    x = nx;
                    y = ny;
                    z = nz;
                    vz -= g * H;
                    tt += H;
                }
                if !hit {
                    continue;
                }
                let splash = self.wstat("mortar", "splash_radius", 7.0) * 1.3;
                if dist3([x, y, z], self.pos) > splash + self.radius + 45.0 {
                    continue;
                }
                out.push(Incoming {
                    mortar: true,
                    pos: p.pos,
                    vel: p.vel,
                    max_t: tt + H,
                    impact: [x, y, z],
                    dmg: self.wstat("mortar", "splash", 160.0),
                    splash,
                });
                continue;
            }
            let (rx, ry) = (self.pos[0] - p.pos[0], self.pos[1] - p.pos[1]);
            let along = (rx * p.vel[0] + ry * p.vel[1]) / sp;
            if along < -3.0 {
                continue;
            }
            let d = hyp(rx, ry);
            if d > 330.0 {
                continue;
            }
            let lat = (rx * p.vel[1] - ry * p.vel[0]).abs() / hyp(p.vel[0], p.vel[1]).max(1.0);
            if lat > 14.0 + self.radius {
                continue;
            }
            let reach = self.w.raycast(p.pos[0], p.pos[1], p.pos[2], p.vel[0] / sp, p.vel[1] / sp, p.vel[2] / sp, d + 20.0);
            if reach < d - self.radius - 3.0 {
                continue;
            }
            let dmg = if p.kind == "npc" { self.npc_stat("damage", 25.0) } else { self.wstat("gun", "damage", 45.0) };
            out.push(Incoming { pos: p.pos, vel: p.vel, max_t: reach / sp, dmg, ..Default::default() });
        }
        out
    }

    /// Evaluates candidate moves by simulating our motion against incoming fire and picks the best.
    pub(crate) fn steer(&mut self, act: &mut Action, rq: &SteerReq) {
        let grounded = self.o.self_.grounded;
        let accel = if grounded { self.mech("ground_accel", 58.5) } else { self.mech("air_accel", 12.0) };
        let mut walk = self.walk;
        if rq.slow {
            walk *= self.mech("charge_move_mult", 0.5);
        }
        let mut boost_v = self.boost_v;
        if rq.slow {
            boost_v *= self.mech("charge_move_mult", 0.5);
        }
        let can_boost = self.energy > 12.0 && grounded;
        let mut horizon = 8usize;
        for th in &rq.threats {
            let k = (th.max_t / self.dt) as i64 + 2;
            if k > horizon as i64 {
                horizon = k as usize;
            }
        }
        if horizon > 44 {
            horizon = 44;
        }
        let mut prog_tick = 6usize;
        if rq.look_ticks > prog_tick {
            prog_tick = rq.look_ticks;
            if horizon < prog_tick {
                horizon = prog_tick;
            }
        }

        let mut cands = std::mem::take(&mut self.scratch);
        cands.clear();
        cands.push(Cand::default());
        for d in dirs16() {
            cands.push(Cand { dx: d[0], dy: d[1], boost: false, score: 0.0 });
        }
        if rq.want_x != 0.0 || rq.want_y != 0.0 {
            cands.push(Cand { dx: rq.want_x, dy: rq.want_y, boost: false, score: 0.0 });
        }
        if can_boost && (rq.boost || rq.dodge_boost || !rq.threats.is_empty()) {
            let n = cands.len();
            for i in 1..n {
                let c = cands[i];
                cands.push(Cand { dx: c.dx, dy: c.dy, boost: true, score: 0.0 });
            }
        }
        let (zc, zr) = (self.o.zone.center, self.o.zone.radius);
        let nav = self.nav.clone();
        let mut fence = -1i64;
        let start_cell = nav.index(self.pos[0], self.pos[1]);
        let start_clear = nav.clear[start_cell];
        let ground_level = self.on_ground_level();
        let mut start_rd = 0i16;
        let (roof_of, roof_dist) = (nav.roofs().1, nav.roof_dist());
        if rq.fence_roof {
            fence = self.on_roof();
            start_rd = roof_dist[start_cell];
        }
        let min_clear = (self.radius * 0.8) as f32;
        let mut best: i64 = -1;
        for ci in 0..cands.len() {
            let c = cands[ci];
            let sp = if c.boost { boost_v } else { walk };
            let (tvx, tvy) = (c.dx * sp, c.dy * sp);
            let (mut x, mut y) = (self.pos[0], self.pos[1]);
            let (mut vx, mut vy) = (self.vel[0], self.vel[1]);
            let mut score = 0.0;
            let mut blocked = false;
            let (mut px, mut py) = (0.0, 0.0);
            for k in 1..=horizon {
                let v = approach2(vx, vy, tvx, tvy, accel * self.dt);
                vx = v.0;
                vy = v.1;
                let (mut nx, mut ny) = (x + vx * self.dt, y + vy * self.dt);
                let cell = nav.index(nx, ny);
                if cell != start_cell {
                    if fence >= 0 {
                        if roof_of[cell] as i64 != fence || (roof_dist[cell] < 2 && roof_dist[cell] < start_rd) {
                            blocked = true;
                            vx = 0.0;
                            vy = 0.0;
                            nx = x;
                            ny = y;
                        }
                    } else if ground_level && (nav.solid[cell] || !self.w.in_bounds(nx, ny) || (nav.clear[cell] < min_clear && nav.clear[cell] < start_clear)) {
                        blocked = true;
                        vx = 0.0;
                        vy = 0.0;
                        nx = x;
                        ny = y;
                    }
                }
                x = nx;
                y = ny;
                if k == prog_tick {
                    px = x;
                    py = y;
                }
                let t0 = (k - 1) as f64 * self.dt;
                let t1 = k as f64 * self.dt;
                for th in &rq.threats {
                    if th.mortar {
                        if th.max_t > t0 && th.max_t <= t1 + 1e-9 || (k == horizon && th.max_t > t1) {
                            let dz = self.pos[2] + self.height * 0.5 - th.impact[2];
                            let d = ((x - th.impact[0]) * (x - th.impact[0]) + (y - th.impact[1]) * (y - th.impact[1]) + dz * dz).sqrt();
                            let reach = th.splash + self.radius;
                            if d < reach {
                                score -= th.dmg * (1.0 - 0.7 * d / reach) + 40.0;
                            }
                        }
                        continue;
                    }
                    if t0 >= th.max_t {
                        continue;
                    }
                    let te = t1.min(th.max_t);
                    let (ax, ay) = (th.pos[0] + th.vel[0] * t0, th.pos[1] + th.vel[1] * t0);
                    let (bx, by) = (th.pos[0] + th.vel[0] * te, th.pos[1] + th.vel[1] * te);
                    let (d, s) = seg_point_dist_2d(x, y, ax, ay, bx, by);
                    if d < self.radius + 0.3 + 0.55 {
                        let tz = th.pos[2] + th.vel[2] * (t0 + (te - t0) * s);
                        if tz > self.pos[2] - 0.6 && tz < self.pos[2] + self.height + 0.6 {
                            score -= th.dmg;
                        }
                    }
                }
            }
            if horizon < prog_tick {
                px = x;
                py = y;
            }
            score += ((px - self.pos[0]) * rq.want_x + (py - self.pos[1]) * rq.want_y) * rq.want_w;
            if blocked {
                score -= 3.0;
            }
            if zr > 0.0 && !rq.zone_free {
                let d_out = hyp(px - zc[0], py - zc[1]) - zr;
                let d_now = hyp(self.pos[0] - zc[0], self.pos[1] - zc[1]) - zr;
                if d_now < -0.5 && d_out > -4.0 && d_out > d_now {
                    score -= (d_out - d_now) * 8.0;
                }
            }
            if let Some(pc) = &rq.pos_cost {
                score -= pc(px, py);
            }
            if c.boost && !rq.boost {
                score -= 1.5;
            }
            if c.dx == 0.0 && c.dy == 0.0 {
                if rq.hold {
                    score += 0.6;
                }
            } else {
                score += 0.25 * (c.dx * self.last_move[0] + c.dy * self.last_move[1]);
            }
            cands[ci].score = score;
            if best < 0 || score > cands[best as usize].score {
                best = ci as i64;
            }
        }
        if best >= 0 {
            let c = cands[best as usize];
            act.mv = [c.dx, c.dy];
            act.boost = c.boost;
        }
        self.scratch = cands;
    }

    /// Whether our feet are near the walk-grid surface (not on a roof or in the air).
    pub(crate) fn on_ground_level(&self) -> bool {
        let k = self.nav.index(self.pos[0], self.pos[1]);
        if self.nav.solid[k] {
            return false;
        }
        self.pos[2] - (self.nav.z[k] as f64) < 3.5
    }

    /// The extra per-cell path cost: keep inside the zone, and (in stealth) keep out of the open.
    pub(crate) fn nav_cost(&self) -> impl Fn(usize) -> f32 {
        let z = &self.o.zone;
        let (cx, cy, r) = (z.center[0], z.center[1], z.radius);
        let stealth = self.st.stealth;
        let nav = self.nav.clone();
        move |k: usize| {
            let (x, y) = nav.pos(k);
            let mut c = 0f32;
            if hyp(x - cx, y - cy) > r - 3.0 {
                c += 6.0;
            }
            if stealth {
                let e = nav.exposure().at(k);
                if e <= 1.0 {
                    c += e * 2.5;
                }
            }
            c
        }
    }

    /// The direction to walk now to reach (gx,gy) along the ground, and the remaining straight-line
    /// distance. ok=false when we have no idea how to get there.
    pub(crate) fn nav_dir(&mut self, gx: f64, gy: f64) -> (f64, f64, f64, bool) {
        let d = hyp(gx - self.pos[0], gy - self.pos[1]);
        if d < 1.0 {
            return (0.0, 0.0, d, true);
        }
        if !self.on_ground_level() {
            let (dx, dy) = norm2(gx - self.pos[0], gy - self.pos[1]);
            return (dx, dy, d, true);
        }
        let (r, h) = (self.radius + 0.15, self.height);
        let nav = self.nav.clone();
        if d < 14.0 && nav.walk_line(self.pos[0], self.pos[1], gx, gy, r, h) {
            let (dx, dy) = norm2(gx - self.pos[0], gy - self.pos[1]);
            return (dx, dy, d, true);
        }
        let other = !self.path_set || hyp(gx - self.path_goal[0], gy - self.path_goal[1]) > 5.0;
        let mut replan = self.path.is_empty() || other || self.t - self.path_t > 1.6;
        if !replan && self.path_idx < self.path.len() {
            let (px, py) = nav.pos(self.path[self.path_idx] as usize);
            if hyp(px - self.pos[0], py - self.pos[1]) > 12.0 {
                replan = true;
            }
        }
        let mut gap = if self.path_bad { 1.0 } else { 0.35 };
        if other && self.p.path_fix {
            gap = 0.15;
        }
        if replan && (self.path_set || !self.p.path_fix) && self.t - self.path_t < gap {
            replan = false;
        }
        if other && !replan && self.p.path_fix {
            let (dx, dy) = norm2(gx - self.pos[0], gy - self.pos[1]);
            return (dx, dy, d, true);
        }
        if replan {
            self.path_set = true;
            let cost = self.nav_cost();
            self.path = nav.path(self.pos[0], self.pos[1], gx, gy, r, h, 9000, Some(&cost));
            self.path_goal = [gx, gy];
            self.path_t = self.t;
            self.path_idx = 0;
            self.path_bad = self.path.is_empty();
            if !self.path_bad {
                let (lx, ly) = nav.pos(*self.path.last().unwrap() as usize);
                self.path_bad = hyp(lx - gx, ly - gy) > 7.0;
            }
        }
        if self.path.is_empty() {
            let (dx, dy) = norm2(gx - self.pos[0], gy - self.pos[1]);
            return (dx, dy, d, false);
        }
        if self.path_bad {
            let (lx, ly) = nav.pos(*self.path.last().unwrap() as usize);
            if hyp(lx - self.pos[0], ly - self.pos[1]) < 4.0 {
                let (dx, dy) = norm2(gx - self.pos[0], gy - self.pos[1]);
                return (dx, dy, d, false);
            }
        }
        let n = self.path.len();
        while self.path_idx + 1 < n {
            let (px, py) = nav.pos(self.path[self.path_idx] as usize);
            if hyp(px - self.pos[0], py - self.pos[1]) < 3.2 {
                self.path_idx += 1;
            } else {
                break;
            }
        }
        let mut tgt = self.path_idx;
        let hi = (self.path_idx + 14).min(n - 1);
        let mut j = hi as i64;
        while j > self.path_idx as i64 {
            let (px, py) = nav.pos(self.path[j as usize] as usize);
            if nav.walk_line(self.pos[0], self.pos[1], px, py, r, h) {
                tgt = j as usize;
                break;
            }
            j -= 2;
        }
        if tgt > self.path_idx + 1 {
            self.path_idx = tgt - 1;
        }
        let (mut px, mut py) = nav.pos(self.path[tgt] as usize);
        if tgt == n - 1 && hyp(gx - px, gy - py) < 4.0 {
            px = gx;
            py = gy;
        }
        let (dx, dy) = norm2(px - self.pos[0], py - self.pos[1]);
        (dx, dy, d, true)
    }

    /// Notices when we push against something without moving and hops out of it.
    pub(crate) fn unstick(&mut self, act: &mut Action, navigating: bool) {
        if !navigating {
            self.stuck_for = 0.0;
            self.stuck_t = self.t;
            self.stuck_pos = self.pos;
            self.unstick_t = 0.0;
            return;
        }
        let moving = hyp(act.mv[0], act.mv[1]) > 0.3;
        if self.t - self.stuck_t >= 0.5 {
            let moved = dist2(self.pos, self.stuck_pos);
            if moving && moved < 1.2 {
                self.stuck_for += self.t - self.stuck_t;
            } else {
                self.stuck_for = 0.0;
            }
            self.stuck_t = self.t;
            self.stuck_pos = self.pos;
        }
        if self.stuck_for >= 1.0 && self.t > self.unstick_t {
            let a = self.rng.float64() * 2.0 * std::f64::consts::PI;
            self.unstick_dir = [a.cos(), a.sin()];
            self.unstick_t = self.t + 0.7;
            self.stuck_for = 0.0;
            self.path.clear();
        }
        if self.t < self.unstick_t {
            act.mv = [act.mv[0] * 0.4 + self.unstick_dir[0], act.mv[1] * 0.4 + self.unstick_dir[1]];
            let l = hyp(act.mv[0], act.mv[1]);
            if l > 1.0 {
                act.mv[0] /= l;
                act.mv[1] /= l;
            }
            if self.o.self_.grounded {
                act.jump = true;
            } else if self.energy > 30.0 {
                act.boost = true;
            }
        }
    }
}
