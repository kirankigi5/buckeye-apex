use super::nav::{NavItem, NAV_DI, NAV_DJ};
use super::Nav;

impl Nav {
    /// For every nav cell, the height of the top surface there: the terrain, or the top of the
    /// highest prism over the cell centre.
    pub fn tops(&self) -> &[f32] {
        self.tops_cell.get_or_init(|| {
            let mut tops = vec![0f32; self.nx * self.ny];
            for j in 0..self.ny {
                for i in 0..self.nx {
                    let (x, y) = self.center(i, j);
                    tops[j * self.nx + i] = self.w.floor(x, y, 1e9).0 as f32;
                }
            }
            tops
        })
    }

    /// The top-surface height at a nav cell.
    pub fn top_at(&self, k: usize) -> f64 {
        self.tops()[k] as f64
    }

    /// Plans a route over the top surface of everything (ground, roofs, courtyards) for a mech that
    /// can walk small steps, drop any height, and jet up walls no taller than max_climb. sz is our
    /// current feet height. Returns cell indices from start to goal, or empty.
    pub fn surface_path(&self, sx: f64, sy: f64, sz: f64, tx: f64, ty: f64, max_climb: f64, body_r: f64, max_expand: usize, blocked: Option<&dyn Fn(usize) -> bool>) -> Vec<i32> {
        let tops = self.tops();
        let (si, sj) = self.cell_at(sx, sy);
        let (gi, gj) = self.cell_at(tx, ty);
        let (start, goal) = (sj * self.nx + si, gj * self.nx + gi);
        if start == goal {
            return vec![start as i32];
        }
        let mut s = self.s.borrow_mut();
        let gen = s.next_gen();
        let nx = self.nx as i64;
        let (gi, gj) = (gi as i64, gj as i64);
        let cellsz = self.cell;
        let hfn = |k: usize| -> f32 {
            let di = (k as i64 % nx - gi).abs() as f64;
            let dj = (k as i64 / nx - gj).abs() as f64;
            let (mn, mx) = if di > dj { (dj, di) } else { (di, dj) };
            ((mx + 0.4142 * mn) * cellsz * 1.1) as f32
        };
        s.open.clear();
        s.g[start] = 0.0;
        s.parent[start] = -1;
        s.stamp[start] = gen;
        s.open.push(NavItem { k: start as i32, f: hfn(start) });
        let mut expanded = 0;
        let (mut best_k, mut best_h) = (start, hfn(start));
        let mut found = false;
        while s.open.len() > 0 {
            let it = s.open.pop();
            let k = it.k as usize;
            if s.closed[k] == gen {
                continue;
            }
            s.closed[k] = gen;
            if k == goal {
                found = true;
                break;
            }
            expanded += 1;
            if expanded > max_expand {
                break;
            }
            let hk = hfn(k);
            if hk < best_h {
                best_h = hk;
                best_k = k;
            }
            let hcur = if k == start { sz } else { tops[k] as f64 };
            let (ci, cj) = ((k % self.nx) as i64, (k / self.nx) as i64);
            for d in 0..8 {
                let (i, j) = (ci + NAV_DI[d], cj + NAV_DJ[d]);
                if i < 0 || j < 0 || i >= nx || j >= self.ny as i64 {
                    continue;
                }
                let nk = (j * nx + i) as usize;
                if s.closed[nk] == gen {
                    continue;
                }
                let (x, y) = self.center_i(i, j);
                if !self.w.in_bounds(x, y) {
                    continue;
                }
                let mut dist = self.cell;
                if d >= 4 {
                    dist *= 1.41421356;
                }
                if let Some(bl) = blocked {
                    if bl(nk) {
                        continue;
                    }
                }
                let dh = tops[nk] as f64 - hcur;
                let mut cost = dist;
                if dh > max_climb {
                    continue;
                } else if dh > 1.2 {
                    cost += 10.0 + dh * 2.2;
                } else if dh < -1.2 {
                    cost += 1.5;
                }
                if !self.solid[nk] && (self.clear[nk] as f64) < body_r - 0.3 {
                    cost += 14.0;
                }
                let ng = s.g[k] + cost as f32;
                if s.stamp[nk] != gen || ng < s.g[nk] {
                    s.g[nk] = ng;
                    s.parent[nk] = k as i32;
                    s.stamp[nk] = gen;
                    s.open.push(NavItem { k: nk as i32, f: ng + hfn(nk) });
                }
            }
        }
        let mut end = goal;
        if !found {
            end = best_k;
            if end == start {
                return Vec::new();
            }
        }
        s.trace(end as i32)
    }
}
