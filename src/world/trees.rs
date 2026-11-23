use super::{clampi, Tree, World, GRID_CELL};

const SEEN_CAP: usize = 96;
const SUB: f64 = 4.0;

impl World {
    fn canopy_limit(&self) -> f64 {
        if self.canopy_density > 0.0 && self.canopy_block_t > 0.0 && self.canopy_block_t < 1.0 {
            -self.canopy_block_t.ln() / self.canopy_density
        } else {
            1e18
        }
    }

    #[inline]
    fn tree_cell(&self, x: f64, y: f64) -> usize {
        let ci = clampi(((x - self.g_min_x) / GRID_CELL).floor() as i64, 0, self.gnx - 1);
        let cj = clampi(((y - self.g_min_y) / GRID_CELL).floor() as i64, 0, self.gny - 1);
        (cj * self.gnx + ci) as usize
    }

    #[inline]
    fn trees_in(&self, c: usize) -> &[i32] {
        &self.tree_idx[self.tree_off[c] as usize..self.tree_off[c + 1] as usize]
    }

    /// Whether trees hide b from a: a trunk on the line, or too much crown along it.
    pub fn canopy_blocked(&self, ax: f64, ay: f64, az: f64, bx: f64, by: f64, bz: f64) -> bool {
        if self.trees.is_empty() {
            return false;
        }
        let (mut dx, mut dy, mut dz) = (bx - ax, by - ay, bz - az);
        let d = (dx * dx + dy * dy + dz * dz).sqrt();
        if d < 1e-6 {
            return false;
        }
        dx /= d;
        dy /= d;
        dz /= d;
        let limit = self.canopy_limit();
        let hl = dx * dx + dy * dy;
        let mut through = 0.0;
        let mut seen: Vec<i32> = Vec::with_capacity(SEEN_CAP);
        let steps = (d / SUB) as i64 + 1;
        let seg = d / steps as f64;
        let mut last_cell = usize::MAX;
        for s in 0..=steps {
            let t = s as f64 * seg;
            let c = self.tree_cell(ax + dx * t, ay + dy * t);
            if c == last_cell {
                continue;
            }
            last_cell = c;
            for &ti in self.trees_in(c) {
                if seen.contains(&ti) {
                    continue;
                }
                if seen.len() < SEEN_CAP {
                    seen.push(ti);
                }
                let tr = &self.trees[ti as usize];
                if tr.trunk_r > 0.0 && hl > 1e-12 {
                    let tt = ((tr.x - ax) * dx + (tr.y - ay) * dy) / hl;
                    if tt > 0.0 && tt < d {
                        let (px, py) = (ax + dx * tt - tr.x, ay + dy * tt - tr.y);
                        if px * px + py * py < tr.trunk_r * tr.trunk_r {
                            let z = az + dz * tt;
                            if z >= tr.z && z <= tr.z + tr.crown_base {
                                return true;
                            }
                        }
                    }
                }
                through += ellipsoid_chord(ax, ay, az, dx, dy, dz, d, tr);
                if through > limit {
                    return true;
                }
            }
        }
        false
    }

    /// How far a sight line gets before trees hide what is behind them, capped at max_t.
    pub fn canopy_reach(&self, ax: f64, ay: f64, az: f64, dx: f64, dy: f64, dz: f64, max_t: f64) -> f64 {
        if self.trees.is_empty() {
            return max_t;
        }
        let limit = self.canopy_limit();
        let hl = dx * dx + dy * dy;
        let mut through = 0.0;
        let mut seen: Vec<i32> = Vec::with_capacity(SEEN_CAP);
        let steps = (max_t / SUB) as i64 + 1;
        let seg = max_t / steps as f64;
        let mut last_cell = usize::MAX;
        let mut best = max_t;
        for s in 0..=steps {
            let t = s as f64 * seg;
            if t > best + GRID_CELL {
                break;
            }
            let c = self.tree_cell(ax + dx * t, ay + dy * t);
            if c == last_cell {
                continue;
            }
            last_cell = c;
            'trees: for &ti in self.trees_in(c) {
                if seen.contains(&ti) {
                    continue;
                }
                if seen.len() < SEEN_CAP {
                    seen.push(ti);
                }
                let tr = &self.trees[ti as usize];
                if tr.trunk_r > 0.0 && hl > 1e-12 {
                    let tt = ((tr.x - ax) * dx + (tr.y - ay) * dy) / hl;
                    if tt > 0.0 && tt < best {
                        let (px, py) = (ax + dx * tt - tr.x, ay + dy * tt - tr.y);
                        if px * px + py * py < tr.trunk_r * tr.trunk_r {
                            let z = az + dz * tt;
                            if z >= tr.z && z <= tr.z + tr.crown_base {
                                best = tt;
                                continue 'trees;
                            }
                        }
                    }
                }
                let ch = ellipsoid_chord(ax, ay, az, dx, dy, dz, max_t, tr);
                if ch > 0.0 {
                    through += ch;
                    if through > limit {
                        let mut tc = (tr.crown_x - ax) * dx + (tr.crown_y - ay) * dy;
                        if hl > 1e-12 {
                            tc /= hl;
                        }
                        if tc < 0.0 {
                            tc = 0.0;
                        }
                        if tc < best {
                            best = tc;
                        }
                    }
                }
            }
        }
        best
    }
}

/// Length of the segment (o + t d, t in [0,max_t]) inside the tree crown.
fn ellipsoid_chord(ox: f64, oy: f64, oz: f64, dx: f64, dy: f64, dz: f64, max_t: f64, tr: &Tree) -> f64 {
    let rh = tr.crown_r;
    let rv = (tr.height - tr.crown_base) * 0.5;
    if rh <= 0.0 || rv <= 0.0 {
        return 0.0;
    }
    let cz = tr.z + tr.crown_base + rv;
    let (px, py, pz) = ((ox - tr.crown_x) / rh, (oy - tr.crown_y) / rh, (oz - cz) / rv);
    let (vx, vy, vz) = (dx / rh, dy / rh, dz / rv);
    let a = vx * vx + vy * vy + vz * vz;
    let b = 2.0 * (px * vx + py * vy + pz * vz);
    let c = px * px + py * py + pz * pz - 1.0;
    let disc = b * b - 4.0 * a * c;
    if disc <= 0.0 || a < 1e-12 {
        return 0.0;
    }
    let sq = disc.sqrt();
    let mut t0 = (-b - sq) / (2.0 * a);
    let mut t1 = (-b + sq) / (2.0 * a);
    if t0 < 0.0 {
        t0 = 0.0;
    }
    if t1 > max_t {
        t1 = max_t;
    }
    if t1 <= t0 {
        return 0.0;
    }
    t1 - t0
}
