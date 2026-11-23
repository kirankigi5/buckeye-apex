use super::Nav;

/// A coarse map of how open each ground position is: the mean free sight distance over a ring of
/// horizontal rays at eye height, normalised to 0..1. Low values are enclosed pockets.
pub struct Exposure {
    pub step: usize,
    pub nx: usize,
    pub ny: usize,
    pub v: Vec<f32>, // 2 = not standable
    nav_nx: usize,
    nav_ny: usize,
}

impl Exposure {
    /// Samples every step-th nav cell with `rays` rays out to max_r metres.
    pub fn new(n: &Nav, step: usize, rays: usize, eye_h: f64, max_r: f64) -> Exposure {
        let nx = (n.nx + step - 1) / step;
        let ny = (n.ny + step - 1) / step;
        let mut v = vec![0f32; nx * ny];
        let dirs: Vec<(f64, f64)> = (0..rays)
            .map(|i| {
                let a = (i as f64 + 0.5) * 2.0 * std::f64::consts::PI / rays as f64;
                (a.cos(), a.sin())
            })
            .collect();
        for j in 0..ny {
            for i in 0..nx {
                let ci = (i * step + step / 2).min(n.nx - 1);
                let cj = (j * step + step / 2).min(n.ny - 1);
                let k = cj * n.nx + ci;
                if n.solid[k] || n.clear[k] < 2.6 {
                    v[j * nx + i] = 2.0;
                    continue;
                }
                let (x, y) = n.center(ci, cj);
                let z = n.z[k] as f64 + eye_h;
                let mut sum = 0.0;
                for &(dx, dy) in &dirs {
                    let mut free = n.w.raycast(x, y, z, dx, dy, 0.0, max_r);
                    if free > 45.0 {
                        let c = n.w.canopy_reach(x, y, z, dx, dy, 0.0, free);
                        if c < free {
                            free = c.max(45.0);
                        }
                    }
                    sum += free;
                }
                v[j * nx + i] = (sum / (rays as f64 * max_r)) as f32;
            }
        }
        Exposure { step, nx, ny, v, nav_nx: n.nx, nav_ny: n.ny }
    }

    /// The exposure at a nav cell (2 when the lattice point is not standable).
    pub fn at(&self, k: usize) -> f32 {
        let (ci, cj) = (k % self.nav_nx, k / self.nav_nx);
        let i = (ci / self.step).min(self.nx - 1);
        let j = (cj / self.step).min(self.ny - 1);
        self.v[j * self.nx + i]
    }

    /// The nav cell index of lattice point (i,j).
    pub fn cell(&self, i: usize, j: usize) -> usize {
        let ci = (i * self.step + self.step / 2).min(self.nav_nx - 1);
        let cj = (j * self.step + self.step / 2).min(self.nav_ny - 1);
        cj * self.nav_nx + ci
    }
}
