//! The static arena (terrain heightfield, building prisms, trees) and the geometric questions the bot
//! needs: ground height, wall collision, ray casts and line of sight.

pub mod exposure;
pub mod nav;
pub mod roofs;
pub mod surface;
pub mod trees;

pub use exposure::Exposure;
pub use nav::Nav;
pub use roofs::{Roof, RoofAccess};

use serde::Deserialize;
use std::path::Path;

pub const KIND_BUILDING: u8 = 0;
pub const KIND_STADIUM: u8 = 1;
pub const KIND_BRIDGE: u8 = 2;

/// An extruded polygon (with holes) that is solid between base and top.
#[derive(Clone, Debug, Default)]
pub struct Prism {
    pub kind: u8,
    pub base: f64,
    pub top: f64,
    pub ring: Vec<f64>,
    pub holes: Vec<Vec<f64>>,
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}

/// One vertical wall of a prism.
#[derive(Clone, Debug, Default)]
pub struct Edge {
    pub x0: f64,
    pub y0: f64,
    pub x1: f64,
    pub y1: f64,
    pub base: f64,
    pub top: f64,
    pub prism: i32,
}

#[derive(Clone, Debug, Default)]
pub struct Tree {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub height: f64,
    pub crown_r: f64,
    pub crown_base: f64,
    pub trunk_r: f64,
    pub crown_x: f64,
    pub crown_y: f64,
}

pub(crate) const GRID_CELL: f64 = 8.0;

#[derive(Default)]
pub struct World {
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,

    pub t_min_x: f64,
    pub t_min_y: f64,
    pub t_step: f64,
    pub nx: usize,
    pub ny: usize,
    pub h: Vec<f32>,
    pub terrain_max: f64,

    pub prisms: Vec<Prism>,
    pub edges: Vec<Edge>,
    pub trees: Vec<Tree>,

    pub canopy_density: f64,
    pub canopy_block_t: f64,

    pub(crate) g_min_x: f64,
    pub(crate) g_min_y: f64,
    pub(crate) gnx: i64,
    pub(crate) gny: i64,
    edge_off: Vec<i32>,
    edge_idx: Vec<i32>,
    prism_off: Vec<i32>,
    prism_idx: Vec<i32>,
    pub(crate) tree_off: Vec<i32>,
    pub(crate) tree_idx: Vec<i32>,

    tb_n: usize,
    tb_nx: usize,
    tb_ny: usize,
    tb_max: Vec<f32>,
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct BoundsJ {
    #[serde(rename = "minX")]
    min_x: f64,
    #[serde(rename = "maxX")]
    max_x: f64,
    #[serde(rename = "minY")]
    min_y: f64,
    #[serde(rename = "maxY")]
    max_y: f64,
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct RulesJ {
    #[serde(rename = "canopyDensity")]
    canopy_density: f64,
    #[serde(rename = "canopyBlockT")]
    canopy_block_t: f64,
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct TerrainJ {
    #[serde(rename = "minX")]
    min_x: f64,
    #[serde(rename = "minY")]
    min_y: f64,
    step: f64,
    nx: f64,
    ny: f64,
    scale: f64,
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct PrismJ {
    kind: Option<String>,
    base: f64,
    top: f64,
    ring: Option<Vec<f64>>,
    holes: Option<Vec<Vec<f64>>>,
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct TreesJ {
    fields: Option<Vec<String>>,
    data: Option<Vec<f64>>,
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct WorldJ {
    bounds: BoundsJ,
    rules: RulesJ,
    terrain: TerrainJ,
    prisms: Option<Vec<PrismJ>>,
    trees: TreesJ,
}

#[inline]
pub(crate) fn clampi(v: i64, lo: i64, hi: i64) -> i64 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

#[inline]
fn point_in_ring(r: &[f64], x: f64, y: f64) -> bool {
    let mut inside = false;
    let n = r.len();
    let mut j = n.wrapping_sub(2);
    let mut i = 0;
    while i + 1 < n {
        let yi = r[i + 1];
        let yj = r[j + 1];
        if (yi > y) != (yj > y) {
            let xi = r[i];
            let xj = r[j];
            if x < (xj - xi) * (y - yi) / (yj - yi) + xi {
                inside = !inside;
            }
        }
        j = i;
        i += 2;
    }
    inside
}

impl Prism {
    /// Whether (x,y) is inside the prism's footprint.
    #[inline]
    pub fn contains(&self, x: f64, y: f64) -> bool {
        if x < self.min_x || x > self.max_x || y < self.min_y || y > self.max_y {
            return false;
        }
        if !point_in_ring(&self.ring, x, y) {
            return false;
        }
        for h in &self.holes {
            if point_in_ring(h, x, y) {
                return false;
            }
        }
        true
    }
}

pub(crate) fn closest_on_seg(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> (f64, f64) {
    let (ex, ey) = (bx - ax, by - ay);
    let l2 = ex * ex + ey * ey;
    if l2 < 1e-18 {
        return (ax, ay);
    }
    let mut t = ((px - ax) * ex + (py - ay) * ey) / l2;
    if t < 0.0 {
        t = 0.0;
    } else if t > 1.0 {
        t = 1.0;
    }
    (ax + ex * t, ay + ey * t)
}

impl World {
    /// Reads <dir>/v2/world.json and <dir>/v2/terrain.i16.
    pub fn load(dir: &str) -> Result<World, String> {
        let base = Path::new(dir).join("v2");
        let raw = std::fs::read(base.join("world.json")).map_err(|e| e.to_string())?;
        let wj: WorldJ = serde_json::from_slice(&raw).map_err(|e| e.to_string())?;
        let mut w = World {
            min_x: wj.bounds.min_x,
            max_x: wj.bounds.max_x,
            min_y: wj.bounds.min_y,
            max_y: wj.bounds.max_y,
            t_min_x: wj.terrain.min_x,
            t_min_y: wj.terrain.min_y,
            t_step: wj.terrain.step,
            nx: wj.terrain.nx as usize,
            ny: wj.terrain.ny as usize,
            canopy_density: wj.rules.canopy_density,
            canopy_block_t: wj.rules.canopy_block_t,
            ..Default::default()
        };
        if w.t_step <= 0.0 {
            w.t_step = 1.0;
        }
        let scale = if wj.terrain.scale == 0.0 { 0.01 } else { wj.terrain.scale };
        let tr = std::fs::read(base.join("terrain.i16")).map_err(|e| e.to_string())?;
        if tr.len() < w.nx * w.ny * 2 {
            return Err(format!("terrain.i16 too short: {} < {}", tr.len(), w.nx * w.ny * 2));
        }
        w.h = vec![0f32; w.nx * w.ny];
        w.terrain_max = -1e9;
        for i in 0..w.h.len() {
            let v = i16::from_le_bytes([tr[2 * i], tr[2 * i + 1]]) as f64 * scale;
            w.h[i] = v as f32;
            if v > w.terrain_max {
                w.terrain_max = v;
            }
        }
        for p in wj.prisms.unwrap_or_default() {
            let mut pr = Prism {
                base: p.base,
                top: p.top,
                ring: p.ring.unwrap_or_default(),
                holes: p.holes.unwrap_or_default(),
                ..Default::default()
            };
            match p.kind.as_deref() {
                Some("stadium") => pr.kind = KIND_STADIUM,
                Some("bridge") => pr.kind = KIND_BRIDGE,
                _ => {}
            }
            if pr.ring.len() < 6 {
                continue;
            }
            pr.min_x = 1e18;
            pr.min_y = 1e18;
            pr.max_x = -1e18;
            pr.max_y = -1e18;
            let mut i = 0;
            while i + 1 < pr.ring.len() {
                pr.min_x = pr.min_x.min(pr.ring[i]);
                pr.max_x = pr.max_x.max(pr.ring[i]);
                pr.min_y = pr.min_y.min(pr.ring[i + 1]);
                pr.max_y = pr.max_y.max(pr.ring[i + 1]);
                i += 2;
            }
            w.prisms.push(pr);
        }
        let fields = wj.trees.fields.unwrap_or_default();
        let data = wj.trees.data.unwrap_or_default();
        let nf = fields.len();
        if nf > 0 {
            let col = |name: &str| fields.iter().position(|f| f == name);
            let get = |row: usize, name: &str| match col(name) {
                Some(c) => data[row * nf + c],
                None => 0.0,
            };
            let has_cx = col("crownX").is_some();
            let n = data.len() / nf;
            for i in 0..n {
                let mut t = Tree {
                    x: get(i, "x"),
                    y: get(i, "y"),
                    z: get(i, "z"),
                    height: get(i, "height"),
                    crown_r: get(i, "crownR"),
                    crown_base: get(i, "crownBase"),
                    trunk_r: get(i, "trunkR"),
                    crown_x: get(i, "crownX"),
                    crown_y: get(i, "crownY"),
                };
                if !has_cx {
                    t.crown_x = t.x;
                    t.crown_y = t.y;
                }
                w.trees.push(t);
            }
        }
        w.build_index();
        Ok(w)
    }

    fn build_index(&mut self) {
        let mut edges = Vec::new();
        for (pi, p) in self.prisms.iter().enumerate() {
            let mut add = |r: &[f64]| {
                let n = r.len();
                if n < 6 {
                    return;
                }
                let mut j = n - 2;
                let mut i = 0;
                while i + 1 < n {
                    if r[j] != r[i] || r[j + 1] != r[i + 1] {
                        edges.push(Edge { x0: r[j], y0: r[j + 1], x1: r[i], y1: r[i + 1], base: p.base, top: p.top, prism: pi as i32 });
                    }
                    j = i;
                    i += 2;
                }
            };
            add(&p.ring);
            for h in &p.holes {
                add(h);
            }
        }
        self.edges = edges;
        self.g_min_x = (self.min_x / GRID_CELL).floor() * GRID_CELL - GRID_CELL;
        self.g_min_y = (self.min_y / GRID_CELL).floor() * GRID_CELL - GRID_CELL;
        self.gnx = ((self.max_x - self.g_min_x) / GRID_CELL).ceil() as i64 + 2;
        self.gny = ((self.max_y - self.g_min_y) / GRID_CELL).ceil() as i64 + 2;
        let (gnx, gny, gmx, gmy) = (self.gnx, self.gny, self.g_min_x, self.g_min_y);
        let nc = (gnx * gny) as usize;
        let cell_range = |min_x: f64, min_y: f64, max_x: f64, max_y: f64| {
            let i0 = clampi(((min_x - gmx) / GRID_CELL).floor() as i64, 0, gnx - 1);
            let i1 = clampi(((max_x - gmx) / GRID_CELL).floor() as i64, 0, gnx - 1);
            let j0 = clampi(((min_y - gmy) / GRID_CELL).floor() as i64, 0, gny - 1);
            let j1 = clampi(((max_y - gmy) / GRID_CELL).floor() as i64, 0, gny - 1);
            (i0, j0, i1, j1)
        };
        let build = |n: usize, bbox: &dyn Fn(usize) -> (f64, f64, f64, f64)| -> (Vec<i32>, Vec<i32>) {
            let mut cnt = vec![0i32; nc + 1];
            for k in 0..n {
                let b = bbox(k);
                let (i0, j0, i1, j1) = cell_range(b.0, b.1, b.2, b.3);
                for j in j0..=j1 {
                    for i in i0..=i1 {
                        cnt[(j * gnx + i + 1) as usize] += 1;
                    }
                }
            }
            for c in 0..nc {
                cnt[c + 1] += cnt[c];
            }
            let mut idx = vec![0i32; cnt[nc] as usize];
            let mut fill = vec![0i32; nc];
            for k in 0..n {
                let b = bbox(k);
                let (i0, j0, i1, j1) = cell_range(b.0, b.1, b.2, b.3);
                for j in j0..=j1 {
                    for i in i0..=i1 {
                        let c = (j * gnx + i) as usize;
                        idx[(cnt[c] + fill[c]) as usize] = k as i32;
                        fill[c] += 1;
                    }
                }
            }
            (cnt, idx)
        };
        const PAD: f64 = 0.01;
        let edges = &self.edges;
        let (eo, ei) = build(edges.len(), &|k| {
            let e = &edges[k];
            (e.x0.min(e.x1) - PAD, e.y0.min(e.y1) - PAD, e.x0.max(e.x1) + PAD, e.y0.max(e.y1) + PAD)
        });
        let prisms = &self.prisms;
        let (po, pi) = build(prisms.len(), &|k| {
            let p = &prisms[k];
            (p.min_x, p.min_y, p.max_x, p.max_y)
        });
        let trees = &self.trees;
        let (to, ti) = build(trees.len(), &|k| {
            let t = &trees[k];
            let r = t.crown_r.max(t.trunk_r);
            (t.x.min(t.crown_x) - r, t.y.min(t.crown_y) - r, t.x.max(t.crown_x) + r, t.y.max(t.crown_y) + r)
        });
        self.edge_off = eo;
        self.edge_idx = ei;
        self.prism_off = po;
        self.prism_idx = pi;
        self.tree_off = to;
        self.tree_idx = ti;
        // coarse terrain max
        self.tb_n = 16;
        self.tb_nx = (self.nx + self.tb_n - 1) / self.tb_n;
        self.tb_ny = (self.ny + self.tb_n - 1) / self.tb_n;
        self.tb_max = vec![-1e9f32; self.tb_nx * self.tb_ny];
        let tbn = self.tb_n as i64;
        for j in 0..self.ny as i64 {
            for i in 0..self.nx as i64 {
                let h = self.h[(j as usize) * self.nx + i as usize];
                // Go integer division truncates toward zero: (0-1)/16 == 0
                for bj in [j / tbn, (j - 1) / tbn] {
                    for bi in [i / tbn, (i - 1) / tbn] {
                        if bi >= 0 && bj >= 0 && (bi as usize) < self.tb_nx && (bj as usize) < self.tb_ny {
                            let b = bj as usize * self.tb_nx + bi as usize;
                            if h > self.tb_max[b] {
                                self.tb_max[b] = h;
                            }
                        }
                    }
                }
            }
        }
    }

    #[inline]
    fn prisms_in(&self, c: usize) -> &[i32] {
        &self.prism_idx[self.prism_off[c] as usize..self.prism_off[c + 1] as usize]
    }

    #[inline]
    fn edges_in(&self, c: usize) -> &[i32] {
        &self.edge_idx[self.edge_off[c] as usize..self.edge_off[c + 1] as usize]
    }

    #[inline]
    pub(crate) fn prisms_at(&self, x: f64, y: f64) -> &[i32] {
        self.prisms_in(self.cell_of(x, y))
    }

    /// Samples the terrain bilinearly (clamped at the grid edge).
    pub fn terrain_z(&self, x: f64, y: f64) -> f64 {
        let mut fx = (x - self.t_min_x) / self.t_step;
        let mut fy = (y - self.t_min_y) / self.t_step;
        let mx = (self.nx - 1) as f64;
        let my = (self.ny - 1) as f64;
        if fx < 0.0 {
            fx = 0.0;
        } else if fx > mx {
            fx = mx;
        }
        if fy < 0.0 {
            fy = 0.0;
        } else if fy > my {
            fy = my;
        }
        let mut i = fx as usize;
        let mut j = fy as usize;
        if i >= self.nx - 1 {
            i = self.nx - 2;
        }
        if j >= self.ny - 1 {
            j = self.ny - 2;
        }
        let tx = fx - i as f64;
        let ty = fy - j as f64;
        let k = j * self.nx + i;
        let h00 = self.h[k] as f64;
        let h10 = self.h[k + 1] as f64;
        let h01 = self.h[k + self.nx] as f64;
        let h11 = self.h[k + self.nx + 1] as f64;
        (h00 * (1.0 - tx) + h10 * tx) * (1.0 - ty) + (h01 * (1.0 - tx) + h11 * tx) * ty
    }

    #[inline]
    pub(crate) fn cell_of(&self, x: f64, y: f64) -> usize {
        let i = clampi(((x - self.g_min_x) / GRID_CELL) as i64, 0, self.gnx - 1);
        let j = clampi(((y - self.g_min_y) / GRID_CELL) as i64, 0, self.gny - 1);
        (j * self.gnx + i) as usize
    }

    /// Whether a point is inside the playable rectangle.
    #[inline]
    pub fn in_bounds(&self, x: f64, y: f64) -> bool {
        x >= self.min_x && x <= self.max_x && y >= self.min_y && y <= self.max_y
    }

    /// The highest solid surface at (x,y) that is at or below z, and the prism it belongs to (-1: terrain).
    pub fn floor(&self, x: f64, y: f64, z: f64) -> (f64, i32) {
        let mut floor = self.terrain_z(x, y);
        let mut on = -1;
        for &pi in self.prisms_at(x, y) {
            let p = &self.prisms[pi as usize];
            if p.top > floor && p.top <= z && p.contains(x, y) {
                floor = p.top;
                on = pi;
            }
        }
        (floor, on)
    }

    /// The lowest prism underside above z at (x,y), or +Inf.
    pub fn ceiling(&self, x: f64, y: f64, z: f64) -> f64 {
        let mut ceil = f64::INFINITY;
        for &pi in self.prisms_at(x, y) {
            let p = &self.prisms[pi as usize];
            if p.base >= z && p.base < ceil && p.contains(x, y) {
                ceil = p.base;
            }
        }
        ceil
    }

    /// Whether the point is inside terrain or a prism.
    pub fn solid_at(&self, x: f64, y: f64, z: f64) -> bool {
        if z < self.terrain_z(x, y) {
            return true;
        }
        self.prisms_at(x, y).iter().any(|&pi| {
            let p = &self.prisms[pi as usize];
            z > p.base && z < p.top && p.contains(x, y)
        })
    }

    /// Whether a vertical span [z0,z1] at (x,y) intersects any prism solid.
    pub fn column_blocked(&self, x: f64, y: f64, z0: f64, z1: f64) -> bool {
        self.prisms_at(x, y).iter().any(|&pi| {
            let p = &self.prisms[pi as usize];
            z1 > p.base && z0 < p.top && p.contains(x, y)
        })
    }

    #[inline]
    fn cell_box(&self, x: f64, y: f64, r: f64) -> (i64, i64, i64, i64) {
        let i0 = clampi(((x - r - self.g_min_x) / GRID_CELL) as i64, 0, self.gnx - 1);
        let i1 = clampi(((x + r - self.g_min_x) / GRID_CELL) as i64, 0, self.gnx - 1);
        let j0 = clampi(((y - r - self.g_min_y) / GRID_CELL) as i64, 0, self.gny - 1);
        let j1 = clampi(((y + r - self.g_min_y) / GRID_CELL) as i64, 0, self.gny - 1);
        (i0, i1, j0, j1)
    }

    /// Moves a vertical cylinder out of any prism wall it overlaps; returns the corrected centre and
    /// whether a wall was touched.
    pub fn push_out(&self, mut x: f64, mut y: f64, z0: f64, z1: f64, r: f64) -> (f64, f64, bool) {
        let mut hit = false;
        for _pass in 0..3 {
            let mut moved = false;
            let (i0, i1, j0, j1) = self.cell_box(x, y, r);
            for j in j0..=j1 {
                for i in i0..=i1 {
                    let c = (j * self.gnx + i) as usize;
                    for &ei in self.edges_in(c) {
                        let e = &self.edges[ei as usize];
                        if z1 <= e.base || z0 >= e.top {
                            continue;
                        }
                        let (cx, cy) = closest_on_seg(x, y, e.x0, e.y0, e.x1, e.y1);
                        let (dx, dy) = (x - cx, y - cy);
                        let d2 = dx * dx + dy * dy;
                        if d2 >= r * r {
                            continue;
                        }
                        let d = d2.sqrt();
                        if d < 1e-9 {
                            let (mut nx, mut ny) = (e.y1 - e.y0, -(e.x1 - e.x0));
                            let nl = nx.hypot(ny);
                            if nl < 1e-12 {
                                continue;
                            }
                            nx /= nl;
                            ny /= nl;
                            if self.prisms[e.prism as usize].contains(x + nx * 0.05, y + ny * 0.05) {
                                nx = -nx;
                                ny = -ny;
                            }
                            x += nx * r;
                            y += ny * r;
                        } else {
                            let push = r - d;
                            x += dx / d * push;
                            y += dy / d * push;
                        }
                        moved = true;
                        hit = true;
                    }
                }
            }
            if !moved {
                break;
            }
        }
        (x, y, hit)
    }

    /// Horizontal distance from (x,y) to the nearest prism wall overlapping (z0,z1), capped at max_r.
    pub fn wall_dist(&self, x: f64, y: f64, z0: f64, z1: f64, max_r: f64) -> f64 {
        let mut best = max_r * max_r;
        let (i0, i1, j0, j1) = self.cell_box(x, y, max_r);
        for j in j0..=j1 {
            for i in i0..=i1 {
                let c = (j * self.gnx + i) as usize;
                for &ei in self.edges_in(c) {
                    let e = &self.edges[ei as usize];
                    if z1 <= e.base || z0 >= e.top {
                        continue;
                    }
                    let (cx, cy) = closest_on_seg(x, y, e.x0, e.y0, e.x1, e.y1);
                    let d2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
                    if d2 < best {
                        best = d2;
                    }
                }
            }
        }
        best.sqrt()
    }

    /// Distance along the unit direction from the origin to the first solid, or max_t.
    pub fn raycast(&self, ox: f64, oy: f64, oz: f64, dx: f64, dy: f64, dz: f64, max_t: f64) -> f64 {
        if ox.is_nan() || oy.is_nan() || oz.is_nan() || dx.is_nan() || dy.is_nan() || dz.is_nan() {
            return max_t;
        }
        if max_t.is_nan() || max_t <= 0.0 {
            return 0.0;
        }
        let best = self.raycast_prisms(ox, oy, oz, dx, dy, dz, max_t);
        self.raycast_terrain(ox, oy, oz, dx, dy, dz, best)
    }

    fn raycast_prisms(&self, ox: f64, oy: f64, oz: f64, dx: f64, dy: f64, dz: f64, max_t: f64) -> f64 {
        let mut best = max_t;
        let mut ci = ((ox - self.g_min_x) / GRID_CELL).floor() as i64;
        let mut cj = ((oy - self.g_min_y) / GRID_CELL).floor() as i64;
        if ci < 0 || cj < 0 || ci >= self.gnx || cj >= self.gny {
            return best;
        }
        let (mut step_i, mut step_j) = (0i64, 0i64);
        let (mut t_max_x, mut t_max_y) = (f64::INFINITY, f64::INFINITY);
        let (mut t_dx, mut t_dy) = (f64::INFINITY, f64::INFINITY);
        if dx > 1e-12 {
            step_i = 1;
            t_max_x = (self.g_min_x + (ci + 1) as f64 * GRID_CELL - ox) / dx;
            t_dx = GRID_CELL / dx;
        } else if dx < -1e-12 {
            step_i = -1;
            t_max_x = (self.g_min_x + ci as f64 * GRID_CELL - ox) / dx;
            t_dx = -GRID_CELL / dx;
        }
        if dy > 1e-12 {
            step_j = 1;
            t_max_y = (self.g_min_y + (cj + 1) as f64 * GRID_CELL - oy) / dy;
            t_dy = GRID_CELL / dy;
        } else if dy < -1e-12 {
            step_j = -1;
            t_max_y = (self.g_min_y + cj as f64 * GRID_CELL - oy) / dy;
            t_dy = -GRID_CELL / dy;
        }
        let mut t_enter = 0.0;
        let guard_max = 2 * (self.gnx + self.gny) + 8;
        for _ in 0..guard_max {
            let c = (cj * self.gnx + ci) as usize;
            let t_exit = t_max_x.min(t_max_y);
            for &ei in self.edges_in(c) {
                let e = &self.edges[ei as usize];
                let (ex, ey) = (e.x1 - e.x0, e.y1 - e.y0);
                let den = dx * ey - dy * ex;
                if den > -1e-12 && den < 1e-12 {
                    continue;
                }
                let (ax, ay) = (e.x0 - ox, e.y0 - oy);
                let t = (ax * ey - ay * ex) / den;
                if t <= 1e-9 || t >= best {
                    continue;
                }
                let s = (ax * dy - ay * dx) / den;
                if !(0.0..=1.0).contains(&s) {
                    continue;
                }
                let z = oz + t * dz;
                if z >= e.base && z <= e.top {
                    best = t;
                }
            }
            if !(-1e-9..=1e-9).contains(&dz) {
                for &pi in self.prisms_in(c) {
                    let p = &self.prisms[pi as usize];
                    let t;
                    if dz < 0.0 {
                        if oz < p.top {
                            continue;
                        }
                        t = (p.top - oz) / dz;
                    } else {
                        if oz > p.base {
                            continue;
                        }
                        t = (p.base - oz) / dz;
                    }
                    if t < t_enter - 1e-6 || t > t_exit + 1e-6 || t >= best || t < 0.0 {
                        continue;
                    }
                    if p.contains(ox + t * dx, oy + t * dy) {
                        best = t;
                    }
                }
            }
            if t_exit >= best {
                break;
            }
            t_enter = t_exit;
            if t_max_x < t_max_y {
                ci += step_i;
                t_max_x += t_dx;
            } else {
                cj += step_j;
                t_max_y += t_dy;
            }
            if ci < 0 || cj < 0 || ci >= self.gnx || cj >= self.gny {
                break;
            }
        }
        best
    }

    fn raycast_terrain(&self, ox: f64, oy: f64, oz: f64, dx: f64, dy: f64, dz: f64, max_t: f64) -> f64 {
        let z_end = oz + dz * max_t;
        if oz > self.terrain_max && z_end > self.terrain_max {
            return max_t;
        }
        let mut t = 0.0;
        let mut prev_t = 0.0;
        let mut prev_h = oz - self.terrain_z(ox, oy);
        if prev_h < 0.0 {
            return 0.0;
        }
        const COARSE: f64 = 8.0;
        while t < max_t {
            if t + COARSE < max_t {
                let (x0, y0, z0) = (ox + dx * t, oy + dy * t, oz + dz * t);
                let (x1, y1, z1) = (x0 + dx * COARSE, y0 + dy * COARSE, z0 + dz * COARSE);
                let (m0, m1) = (self.block_max(x0, y0), self.block_max(x1, y1));
                let zl = if z1 < z0 { z1 } else { z0 };
                if zl > m0 + 0.3 && zl > m1 + 0.3 {
                    t += COARSE;
                    prev_t = t;
                    prev_h = z1 - m1;
                    continue;
                }
            }
            let mut step = prev_h * 0.5;
            if step < 1.0 {
                step = 1.0;
            } else if step > 12.0 {
                step = 12.0;
            }
            t += step;
            if t > max_t {
                t = max_t;
            }
            let (x, y, z) = (ox + dx * t, oy + dy * t, oz + dz * t);
            let h = z - self.terrain_z(x, y);
            if h < 0.0 {
                let (mut lo, mut hi) = (prev_t, t);
                for _ in 0..12 {
                    let mid = (lo + hi) * 0.5;
                    if oz + dz * mid - self.terrain_z(ox + dx * mid, oy + dy * mid) < 0.0 {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                return lo;
            }
            prev_t = t;
            prev_h = h;
        }
        max_t
    }

    /// The highest terrain in the 16 m block containing (x,y).
    fn block_max(&self, x: f64, y: f64) -> f64 {
        let mut i = ((x - self.t_min_x) / self.t_step) as i64;
        let mut j = ((y - self.t_min_y) / self.t_step) as i64;
        if i < 0 {
            i = 0;
        } else if i >= self.nx as i64 {
            i = self.nx as i64 - 1;
        }
        if j < 0 {
            j = 0;
        } else if j >= self.ny as i64 {
            j = self.ny as i64 - 1;
        }
        self.tb_max[(j as usize / self.tb_n) * self.tb_nx + i as usize / self.tb_n] as f64
    }

    /// Whether the straight segment a→b is free of terrain and prisms.
    pub fn visible(&self, ax: f64, ay: f64, az: f64, bx: f64, by: f64, bz: f64) -> bool {
        let (dx, dy, dz) = (bx - ax, by - ay, bz - az);
        let d = (dx * dx + dy * dy + dz * dz).sqrt();
        if d < 1e-6 {
            return true;
        }
        let inv = 1.0 / d;
        self.raycast(ax, ay, az, dx * inv, dy * inv, dz * inv, d - 0.05) >= d - 0.05
    }
}
