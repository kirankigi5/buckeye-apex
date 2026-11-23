use super::{clampi, Exposure, Roof, World};
use std::cell::{OnceCell, RefCell};
use std::rc::Rc;

/// A 2 m walk grid over the ground level of the arena (terrain, low plinths, under bridges).
/// Roofs are recorded (`roof`) but are not part of the walk graph: reaching them takes jets.
pub struct Nav {
    pub w: Rc<World>,
    pub cell: f64,
    pub nx: usize,
    pub ny: usize,
    pub min_x: f64,
    pub min_y: f64,
    pub z: Vec<f32>,
    pub clear: Vec<f32>,
    pub head: Vec<f32>,
    pub solid: Vec<bool>,
    pub roof: Vec<f32>,

    exp: OnceCell<Exposure>,
    comp: OnceCell<Vec<i32>>,
    pub(crate) roofs_cell: OnceCell<(Vec<Roof>, Vec<i32>, Vec<i16>)>,
    pub(crate) tops_cell: OnceCell<Vec<f32>>,

    pub(crate) s: RefCell<Scratch>,
}

#[derive(Default)]
pub(crate) struct Scratch {
    pub g: Vec<f32>,
    pub parent: Vec<i32>,
    pub stamp: Vec<u32>,
    pub closed: Vec<u32>,
    pub gen: u32,
    pub open: NavHeap,
}

impl Scratch {
    pub fn next_gen(&mut self) -> u32 {
        self.gen = self.gen.wrapping_add(1);
        if self.gen == 0 {
            self.stamp.iter_mut().for_each(|v| *v = 0);
            self.closed.iter_mut().for_each(|v| *v = 0);
            self.gen = 1;
        }
        self.gen
    }

    pub fn trace(&self, end: i32) -> Vec<i32> {
        let mut path = Vec::new();
        let mut k = end;
        while k >= 0 {
            path.push(k);
            if path.len() > 100000 {
                break;
            }
            k = self.parent[k as usize];
        }
        path.reverse();
        path
    }
}

pub const NAV_CLEAR_CAP: f64 = 8.0;
pub const NAV_HEAD_CAP: f64 = 40.0;

pub(crate) const NAV_DI: [i64; 8] = [1, -1, 0, 0, 1, 1, -1, -1];
pub(crate) const NAV_DJ: [i64; 8] = [0, 0, 1, -1, 1, -1, 1, -1];

const TAN_MAX_SLOPE: f64 = 1.19; // tan(50 deg)

#[derive(Clone, Copy)]
pub(crate) struct NavItem {
    pub k: i32,
    pub f: f32,
}

/// A plain binary min-heap on f (same tie behaviour as the original).
#[derive(Default)]
pub(crate) struct NavHeap(pub Vec<NavItem>);

impl NavHeap {
    pub fn push(&mut self, it: NavItem) {
        let a = &mut self.0;
        a.push(it);
        let mut i = a.len() - 1;
        while i > 0 {
            let p = (i - 1) / 2;
            if a[p].f <= a[i].f {
                break;
            }
            a.swap(p, i);
            i = p;
        }
    }

    pub fn pop(&mut self) -> NavItem {
        let a = &mut self.0;
        let top = a[0];
        let n = a.len() - 1;
        a[0] = a[n];
        a.truncate(n);
        let mut i = 0;
        loop {
            let l = 2 * i + 1;
            if l >= n {
                break;
            }
            let mut m = l;
            let r = l + 1;
            if r < n && a[r].f < a[l].f {
                m = r;
            }
            if a[i].f <= a[m].f {
                break;
            }
            a.swap(i, m);
            i = m;
        }
        top
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn clear(&mut self) {
        self.0.clear();
    }
}

#[inline]
fn absi(a: i64) -> i64 {
    a.abs()
}

impl Nav {
    /// Builds the walk grid. step_up and body_height come from the mech config.
    pub fn new(w: Rc<World>, step_up: f64, body_height: f64) -> Nav {
        let cell = 2.0;
        let min_x = w.min_x.floor();
        let min_y = w.min_y.floor();
        let nx = ((w.max_x - min_x) / cell).ceil() as usize;
        let ny = ((w.max_y - min_y) / cell).ceil() as usize;
        let n = nx * ny;
        let mut z = vec![0f32; n];
        let mut clear = vec![0f32; n];
        let mut head = vec![0f32; n];
        let mut solid = vec![false; n];
        let mut roof = vec![f32::NAN; n];
        for j in 0..ny {
            for i in 0..nx {
                let k = j * nx + i;
                let x = min_x + (i as f64 + 0.5) * cell;
                let y = min_y + (j as f64 + 0.5) * cell;
                if !w.in_bounds(x, y) {
                    solid[k] = true;
                    continue;
                }
                let mut g = w.terrain_z(x, y);
                let mut is_solid = false;
                let cand = w.prisms_at(x, y);
                for _ in 0..4 {
                    let mut changed = false;
                    for &pi in cand {
                        let p = &w.prisms[pi as usize];
                        if p.top <= g || p.base > g + 0.3 || !p.contains(x, y) {
                            continue;
                        }
                        if p.top - g <= step_up {
                            g = p.top;
                            changed = true;
                        } else {
                            is_solid = true;
                            if roof[k].is_nan() || (p.top as f32) > roof[k] {
                                roof[k] = p.top as f32;
                            }
                        }
                    }
                    if !changed {
                        break;
                    }
                }
                if is_solid {
                    let mut top = roof[k] as f64;
                    for _ in 0..4 {
                        let mut changed = false;
                        for &pi in cand {
                            let p = &w.prisms[pi as usize];
                            if p.top > top && p.base <= top + 0.3 && p.contains(x, y) {
                                top = p.top;
                                changed = true;
                            }
                        }
                        if !changed {
                            break;
                        }
                    }
                    roof[k] = top as f32;
                    solid[k] = true;
                    continue;
                }
                z[k] = g as f32;
                let mut hd = w.ceiling(x, y, g + 0.3) - g;
                if hd > NAV_HEAD_CAP {
                    hd = NAV_HEAD_CAP;
                }
                head[k] = hd as f32;
                clear[k] = w.wall_dist(x, y, g + step_up + 0.05, g + body_height, NAV_CLEAR_CAP) as f32;
            }
        }
        let s = Scratch { g: vec![0.0; n], parent: vec![0; n], stamp: vec![0; n], closed: vec![0; n], gen: 0, open: NavHeap::default() };
        Nav {
            w,
            cell,
            nx,
            ny,
            min_x,
            min_y,
            z,
            clear,
            head,
            solid,
            roof,
            exp: OnceCell::new(),
            comp: OnceCell::new(),
            roofs_cell: OnceCell::new(),
            tops_cell: OnceCell::new(),
            s: RefCell::new(s),
        }
    }

    /// The shared openness map (built on first use).
    pub fn exposure(&self) -> &Exposure {
        self.exp.get_or_init(|| Exposure::new(self, 3, 24, 8.0, 200.0))
    }

    /// Labels the cells a base-size mech can walk between (0 = not walkable).
    pub fn components(&self) -> &[i32] {
        self.comp.get_or_init(|| {
            let (r, h) = (2.9, 9.0);
            let mut comp = vec![0i32; self.nx * self.ny];
            let mut stack: Vec<i32> = Vec::new();
            let mut next = 0i32;
            for s in 0..comp.len() {
                if comp[s] != 0 || !self.walkable(s, r, h) {
                    continue;
                }
                next += 1;
                comp[s] = next;
                stack.clear();
                stack.push(s as i32);
                while let Some(k) = stack.pop() {
                    let k = k as usize;
                    let (ci, cj) = ((k % self.nx) as i64, (k / self.nx) as i64);
                    for d in 0..4 {
                        let (i, j) = (ci + NAV_DI[d], cj + NAV_DJ[d]);
                        if i < 0 || j < 0 || i >= self.nx as i64 || j >= self.ny as i64 {
                            continue;
                        }
                        let nk = j as usize * self.nx + i as usize;
                        if comp[nk] != 0 || !self.walkable(nk, r, h) {
                            continue;
                        }
                        if !self.can_step(k, nk, self.cell) && !self.can_step(nk, k, self.cell) {
                            continue;
                        }
                        comp[nk] = next;
                        stack.push(nk as i32);
                    }
                }
            }
            comp
        })
    }

    #[inline]
    pub fn center(&self, i: usize, j: usize) -> (f64, f64) {
        (self.min_x + (i as f64 + 0.5) * self.cell, self.min_y + (j as f64 + 0.5) * self.cell)
    }

    #[inline]
    pub fn center_i(&self, i: i64, j: i64) -> (f64, f64) {
        (self.min_x + (i as f64 + 0.5) * self.cell, self.min_y + (j as f64 + 0.5) * self.cell)
    }

    #[inline]
    pub fn cell_at(&self, x: f64, y: f64) -> (usize, usize) {
        let i = clampi(((x - self.min_x) / self.cell) as i64, 0, self.nx as i64 - 1);
        let j = clampi(((y - self.min_y) / self.cell) as i64, 0, self.ny as i64 - 1);
        (i as usize, j as usize)
    }

    #[inline]
    pub fn index(&self, x: f64, y: f64) -> usize {
        let (i, j) = self.cell_at(x, y);
        j * self.nx + i
    }

    #[inline]
    pub fn pos(&self, k: usize) -> (f64, f64) {
        self.center(k % self.nx, k / self.nx)
    }

    /// Whether a body of radius r and height h can stand in cell k.
    #[inline]
    pub fn walkable(&self, k: usize, r: f64, h: f64) -> bool {
        !self.solid[k] && self.clear[k] as f64 >= r && self.head[k] as f64 >= h
    }

    #[inline]
    pub(crate) fn can_step(&self, from: usize, to: usize, dist: f64) -> bool {
        let rise = (self.z[to] - self.z[from]) as f64;
        let mut lim = TAN_MAX_SLOPE * dist * 0.85;
        if lim < 1.2 {
            lim = 1.2;
        }
        rise <= lim
    }

    /// The closest walkable cell to (x,y) within max_r cells (ring search), or -1.
    pub fn nearest(&self, x: f64, y: f64, r: f64, h: f64, max_r: i64) -> i64 {
        let (ci, cj) = self.cell_at(x, y);
        let (ci, cj) = (ci as i64, cj as i64);
        let mut best = -1i64;
        let mut best_d = 1e18;
        for rad in 0..=max_r {
            for dj in -rad..=rad {
                for di in -rad..=rad {
                    if absi(di) != rad && absi(dj) != rad {
                        continue;
                    }
                    let (i, j) = (ci + di, cj + dj);
                    if i < 0 || j < 0 || i >= self.nx as i64 || j >= self.ny as i64 {
                        continue;
                    }
                    let k = (j * self.nx as i64 + i) as usize;
                    if !self.walkable(k, r, h) {
                        continue;
                    }
                    let (px, py) = self.center_i(i, j);
                    let d = (px - x) * (px - x) + (py - y) * (py - y);
                    if d < best_d {
                        best = k as i64;
                        best_d = d;
                    }
                }
            }
            if best >= 0 {
                return best;
            }
        }
        best
    }

    /// A* from (sx,sy) to (tx,ty) for a body of radius r, height h. Returns cell indices from start
    /// to goal (inclusive), or an empty path. max_expand bounds the work. extra may be None.
    pub fn path(&self, sx: f64, sy: f64, tx: f64, ty: f64, r: f64, h: f64, max_expand: usize, extra: Option<&dyn Fn(usize) -> f32>) -> Vec<i32> {
        let start = self.nearest(sx, sy, r, h, 6);
        let goal = self.nearest(tx, ty, r, h, 10);
        if start < 0 || goal < 0 {
            return Vec::new();
        }
        let (start, goal) = (start as usize, goal as usize);
        if start == goal {
            return vec![start as i32];
        }
        let mut s = self.s.borrow_mut();
        let gen = s.next_gen();
        let nx = self.nx as i64;
        let (gi, gj) = ((goal % self.nx) as i64, (goal / self.nx) as i64);
        let cellsz = self.cell;
        let hfn = |k: usize| -> f32 {
            let di = absi(k as i64 % nx - gi) as f64;
            let dj = absi(k as i64 / nx - gj) as f64;
            let (mn, mx) = if di > dj { (dj, di) } else { (di, dj) };
            ((mx + 0.4142 * mn) * cellsz * 1.15) as f32
        };
        s.open.clear();
        s.g[start] = 0.0;
        s.parent[start] = -1;
        s.stamp[start] = gen;
        s.open.push(NavItem { k: start as i32, f: hfn(start) });
        let mut expanded = 0;
        let (mut best_k, mut best_h) = (start, hfn(start));
        let mut found = false;
        let want_clear = (r + 2.0) as f32;
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
            let (ci, cj) = ((k % self.nx) as i64, (k / self.nx) as i64);
            for d in 0..8 {
                let (i, j) = (ci + NAV_DI[d], cj + NAV_DJ[d]);
                if i < 0 || j < 0 || i >= nx || j >= self.ny as i64 {
                    continue;
                }
                let nk = (j * nx + i) as usize;
                if s.closed[nk] == gen || !self.walkable(nk, r, h) {
                    continue;
                }
                let mut dist = self.cell;
                if d >= 4 {
                    if !self.walkable((cj * nx + i) as usize, r, h) || !self.walkable((j * nx + ci) as usize, r, h) {
                        continue;
                    }
                    dist *= 1.41421356;
                }
                if !self.can_step(k, nk, dist) {
                    continue;
                }
                let mut cost = dist as f32;
                if self.clear[nk] < want_clear {
                    cost += (want_clear - self.clear[nk]) * 0.6;
                }
                if let Some(ex) = extra {
                    cost += ex(nk);
                }
                let ng = s.g[k] + cost;
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

    /// Whether a body can walk the straight segment a→b on the ground grid.
    pub fn walk_line(&self, ax: f64, ay: f64, bx: f64, by: f64, r: f64, h: f64) -> bool {
        let d = (bx - ax).hypot(by - ay);
        let steps = (d / (self.cell * 0.5)) as i64 + 1;
        let mut prev = self.index(ax, ay);
        if !self.walkable(prev, r, h) {
            return false;
        }
        for s in 1..=steps {
            let t = s as f64 / steps as f64;
            let k = self.index(ax + (bx - ax) * t, ay + (by - ay) * t);
            if k == prev {
                continue;
            }
            if !self.walkable(k, r, h) {
                return false;
            }
            if !self.can_step(prev, k, self.cell) {
                return false;
            }
            prev = k;
        }
        true
    }

    /// Dijkstra from (sx,sy) up to max_cost metres, calling visit for every reached cell with its
    /// path cost. visit returning false stops the flood. The visitor must not start another search.
    pub fn flood(&self, sx: f64, sy: f64, r: f64, h: f64, max_cost: f64, extra: Option<&dyn Fn(usize) -> f32>, visit: &mut dyn FnMut(usize, f32) -> bool) {
        let start = self.nearest(sx, sy, r, h, 6);
        if start < 0 {
            return;
        }
        let start = start as usize;
        let nx = self.nx as i64;
        let gen = {
            let mut s = self.s.borrow_mut();
            let gen = s.next_gen();
            s.open.clear();
            s.g[start] = 0.0;
            s.parent[start] = -1;
            s.stamp[start] = gen;
            s.open.push(NavItem { k: start as i32, f: 0.0 });
            gen
        };
        loop {
            let (k, gk) = {
                let mut s = self.s.borrow_mut();
                if s.open.len() == 0 {
                    return;
                }
                let it = s.open.pop();
                let k = it.k as usize;
                if s.closed[k] == gen {
                    continue;
                }
                s.closed[k] = gen;
                (k, s.g[k])
            };
            if !visit(k, gk) {
                return;
            }
            let mut s = self.s.borrow_mut();
            let (ci, cj) = ((k % self.nx) as i64, (k / self.nx) as i64);
            for d in 0..8 {
                let (i, j) = (ci + NAV_DI[d], cj + NAV_DJ[d]);
                if i < 0 || j < 0 || i >= nx || j >= self.ny as i64 {
                    continue;
                }
                let nk = (j * nx + i) as usize;
                if s.closed[nk] == gen || !self.walkable(nk, r, h) {
                    continue;
                }
                let mut dist = self.cell;
                if d >= 4 {
                    if !self.walkable((cj * nx + i) as usize, r, h) || !self.walkable((j * nx + ci) as usize, r, h) {
                        continue;
                    }
                    dist *= 1.41421356;
                }
                if !self.can_step(k, nk, dist) {
                    continue;
                }
                let mut cost = dist as f32;
                if let Some(ex) = extra {
                    cost += ex(nk);
                }
                let ng = s.g[k] + cost;
                if ng as f64 > max_cost {
                    continue;
                }
                if s.stamp[nk] != gen || ng < s.g[nk] {
                    s.g[nk] = ng;
                    s.parent[nk] = k as i32;
                    s.stamp[nk] = gen;
                    s.open.push(NavItem { k: nk as i32, f: ng });
                }
            }
        }
    }

    /// Reconstructs the path to cell k after a flood (valid until the next search).
    pub fn path_to(&self, k: usize) -> Vec<i32> {
        self.s.borrow().trace(k as i32)
    }
}
