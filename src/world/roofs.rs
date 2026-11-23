use super::nav::{NAV_DI, NAV_DJ};
use super::Nav;

/// A flat rooftop region a mech can stand on (connected nav cells with the same roof height).
#[derive(Clone, Debug, Default)]
pub struct Roof {
    pub id: usize,
    pub z: f64,
    pub cells: Vec<i32>,
    pub area: f64,
    /// above the lowest adjacent ground (what we have to climb)
    pub height: f64,
    /// the cell deepest inside the region and its distance to the region edge (m)
    pub inner: usize,
    pub inner_d: f64,
    pub access: Vec<RoofAccess>,
    pub cx: f64,
    pub cy: f64,
}

/// A (ground cell, roof cell) pair where the wall can be climbed.
#[derive(Clone, Copy, Debug, Default)]
pub struct RoofAccess {
    pub ground: usize,
    pub edge: usize,
    pub climb: f64,
}

impl Nav {
    /// Rooftop regions usable as standing places, and every nav cell's roof id (-1: none).
    pub fn roofs(&self) -> (&[Roof], &[i32]) {
        let r = self.roofs_cell.get_or_init(|| build_roofs(self));
        (&r.0, &r.1)
    }

    /// For every roof cell, its distance to the region edge in cells (1 = edge cell).
    pub fn roof_dist(&self) -> &[i16] {
        &self.roofs_cell.get_or_init(|| build_roofs(self)).2
    }
}

fn build_roofs(n: &Nav) -> (Vec<Roof>, Vec<i32>, Vec<i16>) {
    let total = n.nx * n.ny;
    let (nx, ny) = (n.nx as i64, n.ny as i64);
    let mut roof_of = vec![-1i32; total];
    let mut roofs: Vec<Roof> = Vec::new();
    let mut stack: Vec<i32> = Vec::new();
    for s in 0..total {
        if roof_of[s] >= 0 || !n.solid[s] || n.roof[s].is_nan() {
            continue;
        }
        let id = roofs.len() as i32;
        let z0 = n.roof[s] as f64;
        roof_of[s] = id;
        stack.clear();
        stack.push(s as i32);
        let mut cells = Vec::new();
        let mut sum_z = 0.0;
        while let Some(k) = stack.pop() {
            let k = k as usize;
            cells.push(k as i32);
            sum_z += n.roof[k] as f64;
            let (ci, cj) = ((k % n.nx) as i64, (k / n.nx) as i64);
            for d in 0..4 {
                let (i, j) = (ci + NAV_DI[d], cj + NAV_DJ[d]);
                if i < 0 || j < 0 || i >= nx || j >= ny {
                    continue;
                }
                let nk = (j * nx + i) as usize;
                if roof_of[nk] >= 0 || !n.solid[nk] || n.roof[nk].is_nan() {
                    continue;
                }
                if (n.roof[nk] as f64 - z0).abs() > 0.8 || ((n.roof[nk] - n.roof[k]) as f64).abs() > 0.6 {
                    continue;
                }
                roof_of[nk] = id;
                stack.push(nk as i32);
            }
        }
        let cnt = cells.len();
        roofs.push(Roof { id: id as usize, z: sum_z / cnt as f64, cells, area: cnt as f64 * n.cell * n.cell, ..Default::default() });
    }
    let mut dist = vec![0i16; total];
    for ri in 0..roofs.len() {
        let r = &mut roofs[ri];
        let mut q: Vec<i32> = Vec::new();
        let mut min_ground = f64::INFINITY;
        let (mut sx, mut sy) = (0.0, 0.0);
        for &c in &r.cells {
            let k = c as usize;
            let (x, y) = n.pos(k);
            sx += x;
            sy += y;
            let (ci, cj) = ((k % n.nx) as i64, (k / n.nx) as i64);
            let mut edge = false;
            for d in 0..8 {
                let (i, j) = (ci + NAV_DI[d], cj + NAV_DJ[d]);
                if i < 0 || j < 0 || i >= nx || j >= ny {
                    edge = true;
                    continue;
                }
                let nk = (j * nx + i) as usize;
                if roof_of[nk] != ri as i32 {
                    edge = true;
                    if !n.solid[nk] {
                        let g = n.z[nk] as f64;
                        if g < min_ground {
                            min_ground = g;
                        }
                    }
                }
            }
            if edge {
                dist[k] = 1;
                q.push(c);
            } else {
                dist[k] = 0;
            }
        }
        r.cx = sx / r.cells.len() as f64;
        r.cy = sy / r.cells.len() as f64;
        let mut h = 0;
        while h < q.len() {
            let k = q[h] as usize;
            let (ci, cj) = ((k % n.nx) as i64, (k / n.nx) as i64);
            for d in 0..4 {
                let (i, j) = (ci + NAV_DI[d], cj + NAV_DJ[d]);
                if i < 0 || j < 0 || i >= nx || j >= ny {
                    continue;
                }
                let nk = (j * nx + i) as usize;
                if roof_of[nk] == ri as i32 && dist[nk] == 0 {
                    dist[nk] = dist[k] + 1;
                    q.push(nk as i32);
                }
            }
            h += 1;
        }
        let (mut best, mut bd) = (r.cells[0] as usize, 0i16);
        for &c in &r.cells {
            if dist[c as usize] > bd {
                best = c as usize;
                bd = dist[c as usize];
            }
        }
        r.inner = best;
        r.inner_d = (bd as f64 - 0.5) * n.cell;
        r.height = if min_ground.is_infinite() { 0.0 } else { r.z - min_ground };
        for ci_ in 0..r.cells.len() {
            let k = r.cells[ci_] as usize;
            if dist[k] != 1 {
                continue;
            }
            let (ci, cj) = ((k % n.nx) as i64, (k / n.nx) as i64);
            for d in 0..4 {
                let (i1, j1) = (ci + NAV_DI[d], cj + NAV_DJ[d]);
                if i1 < 0 || j1 < 0 || i1 >= nx || j1 >= ny || n.solid[(j1 * nx + i1) as usize] {
                    continue;
                }
                for step in 2..=3 {
                    let (i, j) = (ci + NAV_DI[d] * step, cj + NAV_DJ[d] * step);
                    if i < 0 || j < 0 || i >= nx || j >= ny {
                        break;
                    }
                    let g = (j * nx + i) as usize;
                    if n.solid[g] {
                        break;
                    }
                    if n.walkable(g, 2.7, 9.0) {
                        let (i2, j2) = (ci - NAV_DI[d] * 2, cj - NAV_DJ[d] * 2);
                        if i2 < 0 || j2 < 0 || i2 >= nx || j2 >= ny || roof_of[(j2 * nx + i2) as usize] != ri as i32 {
                            break;
                        }
                        let climb = r.z - n.z[g] as f64;
                        r.access.push(super::RoofAccess { ground: g, edge: (j2 * nx + i2) as usize, climb });
                        break;
                    }
                }
            }
        }
    }
    // keep only roofs worth standing on; re-number
    let mut remap = vec![-1i32; roofs.len()];
    let mut keep: Vec<super::Roof> = Vec::new();
    for mut r in roofs {
        if r.area < 250.0 || r.inner_d < 5.0 || r.access.is_empty() {
            continue;
        }
        remap[r.id] = keep.len() as i32;
        r.id = keep.len();
        keep.push(r);
    }
    for v in roof_of.iter_mut() {
        if *v >= 0 {
            *v = remap[*v as usize];
        }
    }
    for i in 0..dist.len() {
        if roof_of[i] < 0 {
            dist[i] = 0;
        }
    }
    (keep, roof_of, dist)
}
