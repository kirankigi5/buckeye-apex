use super::*;
use crate::proto::Action;

/// An aiming solution.
#[derive(Clone, Copy, Default, Debug)]
pub(crate) struct Shot {
    pub ok: bool,
    pub yaw: f64,
    pub pitch: f64,
    pub tf: f64,
    pub point: [f64; 3],
    pub dist: f64,
    pub clear: bool,
}

pub(crate) const FIRE_LATENCY: f64 = 0.075; // our action lands 1-2 ticks after the observation

/// Something to shoot at.
#[derive(Clone, Copy, Default, Debug)]
pub(crate) struct AimTarget {
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub radius: f64,
    pub center_z: f64,
    pub mech: bool,
    pub lead: f64,
    pub lead_set: bool,
    pub gun_lead: f64,
    pub gun_lead_set: bool,
    pub rear: bool,
    pub has_mortar: bool,
}

fn gun_hit_prob(d: f64) -> f64 {
    if d < 60.0 {
        0.95
    } else if d < 120.0 {
        0.95 - (d - 60.0) / 60.0 * 0.12
    } else if d < 180.0 {
        0.83 - (d - 120.0) / 60.0 * 0.10
    } else if d < 230.0 {
        0.73 - (d - 180.0) / 50.0 * 0.15
    } else {
        0.05
    }
}

fn their_gun_hit_prob(d: f64) -> f64 {
    if d < 40.0 {
        0.97
    } else if d < 80.0 {
        0.97 - (d - 40.0) / 40.0 * 0.12
    } else if d < 120.0 {
        0.85 - (d - 80.0) / 40.0 * 0.40
    } else if d < 180.0 {
        0.45 - (d - 120.0) / 60.0 * 0.20
    } else {
        0.12
    }
}

/// Interpolates a hit rate over range from (distance, rate) knots.
fn hit_table(d: f64, knots: &[[f64; 2]]) -> f64 {
    if d <= knots[0][0] {
        return knots[0][1];
    }
    for i in 1..knots.len() {
        if d <= knots[i][0] {
            let f = (d - knots[i - 1][0]) / (knots[i][0] - knots[i - 1][0]);
            return knots[i - 1][1] + (knots[i][1] - knots[i - 1][1]) * f;
        }
    }
    knots[knots.len() - 1][1]
}

const REAL_MY_GUN: [[f64; 2]; 6] = [[30.0, 0.95], [60.0, 0.86], [100.0, 0.55], [150.0, 0.28], [200.0, 0.18], [230.0, 0.12]];
const REAL_THEIR_GUN: [[f64; 2]; 6] = [[30.0, 0.92], [60.0, 0.78], [100.0, 0.57], [150.0, 0.32], [200.0, 0.15], [230.0, 0.10]];

impl Bot {
    pub(crate) fn is_rear(&self, tr: &Track) -> bool {
        if tr.npc {
            return false;
        }
        let half_front = self.mech("front_arc_deg", 180.0) * 0.5 * std::f64::consts::PI / 180.0;
        let bear = (self.pos[1] - tr.pos[1]).atan2(self.pos[0] - tr.pos[0]);
        ang_diff(tr.aim[0], bear).abs() > half_front
    }

    pub(crate) fn target_of(&self, tr: &Track) -> AimTarget {
        if tr.npc {
            return AimTarget { pos: tr.pos, vel: tr.vel, radius: self.npc_stat("radius", 1.8), center_z: self.npc_stat("height", 3.5) * 0.5, ..Default::default() };
        }
        let sc = tr.scale(self.size_per_cube);
        let mut at = AimTarget {
            pos: tr.pos,
            vel: tr.vel,
            radius: self.mech("radius", 2.5) * sc,
            center_z: 4.5 * sc,
            mech: true,
            rear: self.is_rear(tr),
            has_mortar: tr.mortar,
            ..Default::default()
        };
        if self.p.lead_persist || self.p.lead_cum {
            at.gun_lead = self.gun_lead(tr);
            at.gun_lead_set = true;
        }
        at
    }

    pub(crate) fn lead_damp(&self, tf: f64) -> f64 {
        if tf <= 0.3 {
            return 1.0;
        }
        let d = 1.0 - (tf - 0.3) / 0.5 * (1.0 - self.p.lead_damp);
        d.max(self.p.lead_damp)
    }

    pub(crate) fn solve_direct_lat(&self, tg: &AimTarget, speed: f64, origin_h: f64, lat: f64) -> Shot {
        let ox = self.pos[0] + self.vel[0] * lat;
        let oy = self.pos[1] + self.vel[1] * lat;
        let mut oz = self.pos[2] + origin_h;
        if !self.o.self_.grounded {
            oz += self.vel[2] * lat;
        }
        let mut tf = 0.0;
        let (mut px, mut py, mut pz) = (0.0, 0.0, 0.0);
        let k = if tg.lead_set { tg.lead } else { 1.0 };
        for _ in 0..4 {
            let damp = self.lead_damp(tf);
            px = tg.pos[0] + tg.vel[0] * (lat + tf * damp) * k;
            py = tg.pos[1] + tg.vel[1] * (lat + tf * damp) * k;
            let vz = clamp(tg.vel[2], -15.0, 12.0);
            pz = tg.pos[2] + tg.center_z + vz * (lat + tf) * 0.6;
            if speed <= 0.0 {
                break;
            }
            let d = ((px - ox) * (px - ox) + (py - oy) * (py - oy) + (pz - oz) * (pz - oz)).sqrt();
            tf = (d - 3.0).max(0.0) / speed;
        }
        let hd = hyp(px - ox, py - oy);
        let mut s = Shot { ok: true, tf, point: [px, py, pz], ..Default::default() };
        s.yaw = (py - oy).atan2(px - ox);
        s.pitch = (pz - oz).atan2(hd);
        s.dist = (hd * hd + (pz - oz) * (pz - oz)).sqrt();
        if s.dist > 1e-3 {
            let (dx, dy, dz) = ((px - ox) / s.dist, (py - oy) / s.dist, (pz - oz) / s.dist);
            let reach = s.dist - tg.radius * 0.9;
            s.clear = if reach < 0.5 { true } else { self.w.raycast(ox, oy, oz, dx, dy, dz, reach) >= reach - 1e-6 };
        } else {
            s.clear = true;
        }
        s
    }

    /// A ballistic solution (low arc first, high arc when the low one is blocked).
    pub(crate) fn solve_mortar(&self, tg: &AimTarget, st: Option<&Stats>) -> Shot {
        let v = stat(st, "speed", 72.0);
        let g = stat(st, "gravity", 22.0);
        let lat = FIRE_LATENCY;
        let ox = self.pos[0] + self.vel[0] * lat;
        let oy = self.pos[1] + self.vel[1] * lat;
        let oz = self.pos[2] + self.height + 0.5 * self.scale;
        let min_p = self.mech("min_pitch", -0.7);
        let max_p = self.mech("max_pitch", 1.3);
        for arc in 0..2 {
            let mut tf = 0.6;
            let (mut th, mut px, mut py, mut pz) = (0.0, 0.0, 0.0, 0.0);
            let mut ok = true;
            for _ in 0..5 {
                let damp = self.lead_damp(tf);
                px = tg.pos[0] + tg.vel[0] * (lat + tf * damp);
                py = tg.pos[1] + tg.vel[1] * (lat + tf * damp);
                pz = self.w.floor(px, py, tg.pos[2] + 3.0).0;
                pz += 0.3;
                let d = hyp(px - ox, py - oy);
                if d < 1.0 {
                    ok = false;
                    break;
                }
                let h = pz - oz;
                let disc = v * v * v * v - g * (g * d * d + 2.0 * h * v * v);
                if disc < 0.0 {
                    ok = false;
                    break;
                }
                let sq = disc.sqrt();
                th = if arc == 0 { (v * v - sq).atan2(g * d) } else { (v * v + sq).atan2(g * d) };
                let c = th.cos();
                if c < 0.05 {
                    ok = false;
                    break;
                }
                tf = d / (v * c);
            }
            if !ok || th < min_p || th > max_p {
                continue;
            }
            if tf > stat(st, "ttl", 7.0) - 0.3 {
                continue;
            }
            let mut s = Shot { ok: true, tf, point: [px, py, pz], pitch: th, ..Default::default() };
            s.yaw = (py - oy).atan2(px - ox);
            s.dist = hyp(px - ox, py - oy);
            s.clear = true;
            let (cy, sy) = (s.yaw.cos(), s.yaw.sin());
            let (vh, vz) = (v * th.cos(), v * th.sin());
            let (mut lx, mut ly, mut lz) = (ox + cy * 3.0 * th.cos(), oy + sy * 3.0 * th.cos(), oz + 3.0 * th.sin());
            let n = (tf / 0.12) as i64 + 2;
            for i in 1..=n {
                let mut tt = tf * i as f64 / n as f64;
                if i == n {
                    tt = tf - 0.04;
                }
                let (nx, ny, nz) = (ox + cy * vh * tt, oy + sy * vh * tt, oz + vz * tt - 0.5 * g * tt * tt);
                if !self.w.visible(lx, ly, lz, nx, ny, nz) {
                    if hyp(nx - px, ny - py) > 3.5 {
                        s.clear = false;
                    }
                    break;
                }
                lx = nx;
                ly = ny;
                lz = nz;
            }
            if s.clear {
                return s;
            }
        }
        Shot::default()
    }

    /// How far our torso is from a solution, after the slew we get before firing.
    pub(crate) fn aim_err(&self, s: &Shot) -> (f64, f64) {
        let ey = ang_diff(s.yaw, self.aim[0]).abs() - self.turn * self.dt * 0.9;
        let ep = (s.pitch - self.aim[1]).abs() - self.pitch_r * self.dt * 0.9;
        (ey.max(0.0), ep.max(0.0))
    }

    pub(crate) fn laser_damage_at(&self, st: Option<&Stats>, charge: f64) -> f64 {
        let full = stat(st, "charge", 0.75);
        let mf = stat(st, "min_frac", 0.35);
        let f = if full > 0.0 { clamp(charge / full, 0.0, 1.0) } else { 1.0 };
        stat(st, "damage", 315.0) * (mf + (1.0 - mf) * f) * self.cube_damage_mul(self.o.self_.cubes)
    }

    /// Aims at tg and works the triggers.
    pub(crate) fn engage(&mut self, act: &mut Action, tg: AimTarget, has_los: bool, los_recent: bool, hp_left: f64, allow_fire: bool) {
        let (gun_i, las_i, lig_i, mor_i) = (self.has("gun"), self.has("laser"), self.has("lightning"), self.has("mortar"));
        let (mut gun_s, mut las_s, mut mor_s) = (Shot::default(), Shot::default(), Shot::default());
        let mut head_tg = tg;
        let mut head_ok = false;
        if tg.mech {
            head_tg.center_z = tg.center_z * 1.82;
            head_tg.radius = tg.radius * 0.8;
            head_ok = true;
        }
        let mut pitch_tol = tg.center_z * 0.8;
        if gun_i >= 0 {
            let gi = gun_i as usize;
            let (mut gun_tg, mut gun_head) = (tg, head_tg);
            if tg.gun_lead_set {
                gun_tg.lead = tg.gun_lead;
                gun_tg.lead_set = true;
                gun_head.lead = tg.gun_lead;
                gun_head.lead_set = true;
            }
            let mut gun_lat = FIRE_LATENCY;
            if self.p.smart_fire_v2 {
                gun_lat = if self.slots[gi].cooldown <= self.dt + 1e-6 { self.dt } else { self.decide };
            }
            let speed = self.slot_stat(gi, "speed", 252.0);
            gun_s = self.solve_direct_lat(&gun_tg, speed, self.muzzle_h, gun_lat);
            if !gun_s.clear && head_ok {
                let alt = self.solve_direct_lat(&gun_head, speed, self.muzzle_h, gun_lat);
                if alt.clear {
                    gun_s = alt;
                    pitch_tol = 0.7;
                }
            }
        }
        if las_i >= 0 || lig_i >= 0 {
            let las_lat = if self.p.smart_fire_v2 { self.dt } else { FIRE_LATENCY };
            las_s = self.solve_direct_lat(&tg, 0.0, self.muzzle_h, las_lat);
            if !las_s.clear && head_ok {
                let alt = self.solve_direct_lat(&head_tg, 0.0, self.muzzle_h, las_lat);
                if alt.clear {
                    las_s = alt;
                    pitch_tol = 0.7;
                }
            }
        }
        if mor_i >= 0 {
            mor_s = self.solve_mortar(&tg, self.slots[mor_i as usize].stats.as_ref());
        }
        let d = dist3([self.pos[0], self.pos[1], self.pos[2] + self.muzzle_h], [tg.pos[0], tg.pos[1], tg.pos[2] + tg.center_z]);

        // --- decide what the torso tracks
        let mut primary = las_s;
        let mut using_mortar_aim = false;
        let mut gun_primary = false;
        if gun_i >= 0 {
            primary = gun_s;
            gun_primary = true;
        }
        let mut laser_ready = false;
        if las_i >= 0 {
            let li = las_i as usize;
            let full = self.slot_stat(li, "charge", 0.75);
            let charge = self.slots[li].charge;
            if charge > 0.0 && (charge >= full - 0.16 || self.laser_damage_at(self.slots[li].stats.as_ref(), charge + 0.1) >= hp_left) {
                laser_ready = true;
                primary = las_s;
                gun_primary = false;
            }
            if gun_i < 0 {
                primary = las_s;
            }
        }
        if mor_i >= 0 && mor_s.ok && mor_s.clear && self.slots[mor_i as usize].cooldown <= 0.15 && !laser_ready {
            let gun_idle = gun_i < 0
                || self.slots[gun_i as usize].reload > 0.0
                || self.slots[gun_i as usize].ammo == 0.0
                || !gun_s.clear
                || d > self.p.gun_max_range;
            if allow_fire && (self.mortar_worth(&tg, &mor_s, d) || (gun_idle && mor_s.dist > 25.0)) {
                primary = mor_s;
                using_mortar_aim = true;
                gun_primary = false;
            }
        }
        if !primary.ok {
            return;
        }
        act.aim = [primary.yaw, clamp(primary.pitch, self.mech("min_pitch", -0.7), self.mech("max_pitch", 1.3))];
        act.has_aim = true;
        if !allow_fire && !(las_i >= 0 && self.slots[las_i as usize].charge > 0.0) {
            return;
        }

        // --- gun
        if gun_i >= 0 && allow_fire {
            let gi = gun_i as usize;
            let sl = &self.slots[gi];
            if sl.ammo <= 0.0 && sl.reload <= 0.0 {
                act.reload = true;
            }
            let rng = self.slot_stat(gi, "range", 230.0);
            let mut max_r = (rng - 5.0).min(self.p.gun_max_range);
            if !tg.mech {
                max_r = (rng - 5.0).min(120.0);
            }
            if has_los && gun_s.clear && sl.ammo > 0.0 && sl.reload <= 0.0 && gun_s.dist < max_r {
                let (mut ey, mut ep) = self.aim_err(&gun_s);
                if !gun_primary {
                    ey = ang_diff(gun_s.yaw, self.aim[0]).abs();
                    ep = (gun_s.pitch - self.aim[1]).abs();
                }
                let tol_y = (tg.radius * 0.8).atan2(gun_s.dist.max(1.0));
                let tol_p = pitch_tol.atan2(gun_s.dist.max(1.0));
                if ey < tol_y && ep < tol_p {
                    act.fire[gi] = true;
                }
            }
        }
        // --- laser
        if las_i >= 0 {
            let li = las_i as usize;
            let rng = self.slot_stat(li, "range", 280.0) - 4.0;
            let full = self.slot_stat(li, "charge", 0.75);
            let in_range = las_s.dist < rng;
            let charge = self.slots[li].charge;
            if charge > 0.0 {
                let mut hold = true;
                let mut las_mult = 1.0;
                if self.p.smart_fire_v2 && tg.rear {
                    las_mult *= self.mech("rear_mult", 1.5);
                }
                if (self.p.smart_fire_v2 || self.p.cookoff_aware) && tg.has_mortar {
                    las_mult *= 1.0 + self.wstat("laser", "cookoff", 0.0);
                }
                let st = self.slots[li].stats.as_ref();
                let lethal = self.laser_damage_at(st, charge) * las_mult >= hp_left && charge >= stat(st, "min_charge", 0.15);
                let mut rel_thr = full - 1e-3;
                if self.p.laser_early_rel {
                    rel_thr = full - self.dt - 1e-3;
                }
                if (charge >= rel_thr || lethal) && has_los && las_s.clear && in_range {
                    let (ey, ep) = self.aim_err(&las_s);
                    let tol_y = (tg.radius * 0.75).atan2(las_s.dist.max(1.0));
                    let tol_p = (pitch_tol * 0.9).atan2(las_s.dist.max(1.0));
                    if ey < tol_y && ep < tol_p {
                        hold = false;
                    }
                }
                act.fire[li] = hold;
                self.laser_hold = hold;
            } else if allow_fire && self.slots[li].cooldown <= 0.12 && in_range && (has_los || los_recent) && (tg.mech || !self.p.laser_mech_only) {
                act.fire[li] = true;
                self.laser_hold = true;
            }
        }
        if !allow_fire {
            return;
        }
        // --- lightning
        if lig_i >= 0 {
            let li = lig_i as usize;
            let rng = self.slot_stat(li, "range", 45.0);
            let cone = self.slot_stat(li, "cone_deg", 30.0) * std::f64::consts::PI / 180.0;
            let cost = self.slot_stat(li, "energy", 3.0);
            if has_los && las_s.clear && d < rng - 1.0 - 0.5 * hyp(tg.vel[0] - self.vel[0], tg.vel[1] - self.vel[1]) * 0.1 && self.energy >= cost {
                let ey = ang_diff(las_s.yaw, self.aim[0]).abs() - self.turn * self.dt * 0.9;
                if ey < cone * 0.4 {
                    act.fire[li] = true;
                }
            }
        }
        // --- mortar
        if mor_i >= 0 && mor_s.ok && mor_s.clear {
            let mi = mor_i as usize;
            if self.slots[mi].cooldown <= 0.05 && using_mortar_aim {
                let (ey, ep) = self.aim_err(&mor_s);
                if ey < 0.03 && ep < 0.02 {
                    act.fire[mi] = true;
                }
            }
        }
    }

    /// Whether a shell at this solution is likely to connect.
    pub(crate) fn mortar_worth(&self, tg: &AimTarget, s: &Shot, _d: f64) -> bool {
        let i = self.has("mortar");
        let splash = if i >= 0 { self.slot_stat(i as usize, "splash_radius", 7.0) } else { 7.0 };
        if s.dist < splash + self.radius + 3.0 {
            return false;
        }
        let sp = hyp(tg.vel[0], tg.vel[1]);
        let miss = if tg.mech { sp * s.tf * 0.5 } else { sp * s.tf * 0.2 };
        miss < splash * 0.75
    }

    /// Looks along our heading with a slow sweep, or toward the last threat cue.
    pub(crate) fn idle_aim(&mut self, act: &mut Action, heading: f64, have_heading: bool) {
        let yaw;
        if self.t < 1.75 {
            yaw = self.aim[0] + 0.39;
        } else if self.t - self.last_weapon_dt < 2.5 || self.t - self.last_npc_dt < 2.5 {
            yaw = self.last_dmg_bear;
        } else if self.t - self.last_sound_t < 1.5 {
            yaw = self.last_sound_b;
        } else {
            self.scan_phase += self.decide;
            yaw = if have_heading { heading + 1.9 * (self.scan_phase * 1.3).sin() } else { self.aim[0] + 0.35 };
        }
        act.aim = [yaw, 0.0];
        act.has_aim = true;
    }

    pub(crate) fn my_gun_hit(&self, d: f64) -> f64 {
        if self.real_model() {
            hit_table(d, &REAL_MY_GUN)
        } else {
            gun_hit_prob(d)
        }
    }

    pub(crate) fn their_gun_hit(&self, d: f64) -> f64 {
        if self.real_model() {
            hit_table(d, &REAL_THEIR_GUN)
        } else {
            their_gun_hit_prob(d)
        }
    }

    fn laser_hit_rate(&self) -> f64 {
        if self.real_model() {
            0.75
        } else {
            0.85
        }
    }

    /// Sustained damage per second of a loadout at range d.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn dps_of(&self, gun: bool, laser: bool, light: bool, mortar: bool, cubes: i64, d: f64, energy_ok: bool, mine: bool) -> f64 {
        let mul = self.cube_damage_mul(cubes);
        let mut total = 0.0;
        if gun {
            let g = self.h.config.weapons.get("gun");
            let cd = stat(g, "cooldown", 0.24);
            let mag = stat(g, "mag", 12.0);
            let rl = stat(g, "reload", 2.4);
            if d < stat(g, "range", 230.0) {
                let hit = if mine { self.my_gun_hit(d) } else { self.their_gun_hit(d) };
                total += stat(g, "damage", 45.0) * mag / (mag * cd + rl) * hit;
            }
        }
        if laser {
            let l = self.h.config.weapons.get("laser");
            if d < stat(l, "range", 280.0) {
                total += stat(l, "damage", 315.0) / (stat(l, "charge", 0.75) + stat(l, "cooldown", 1.65)) * self.laser_hit_rate();
            }
        }
        if light && energy_ok {
            let l = self.h.config.weapons.get("lightning");
            let mut lr = stat(l, "range", 45.0);
            if self.p.anti_lightning_w > 0.0 {
                lr += 6.0;
            }
            if d < lr {
                total += stat(l, "damage", 60.0) / stat(l, "cooldown", 0.25);
            }
        }
        if mortar {
            let m = self.h.config.weapons.get("mortar");
            if d > 12.0 && d < 200.0 {
                total += stat(m, "damage", 200.0) / stat(m, "cooldown", 2.6) * 0.25;
            }
        }
        total * mul
    }

    pub(crate) fn my_dps(&self, d: f64) -> f64 {
        let gun = self.has("gun") >= 0 && d < self.p.gun_max_range;
        self.dps_of(gun, self.has("laser") >= 0, self.has("lightning") >= 0, self.has("mortar") >= 0, self.o.self_.cubes, d, self.energy > 15.0, true)
    }

    pub(crate) fn their_dps(&self, tr: &Track, d: f64) -> f64 {
        if tr.npc {
            if d > 95.0 {
                return 0.0;
            }
            return self.npc_stat("damage", 25.0) / self.npc_stat("cooldown", 1.0) * 0.5;
        }
        let gun = tr.gun || (!tr.laser && !tr.mortar && !tr.light);
        let mut laser = tr.laser;
        if laser {
            let li = self.has("lightning");
            if li >= 0 && self.energy > 20.0 && d < self.slot_stat(li as usize, "range", 45.0) - 3.0 {
                laser = false;
            }
            if laser && self.p.laser_cd_track && !self.ffa && !tr.charging && tr.laser_fired_t > 0.0 {
                let cd_win = self.wstat("laser", "cooldown", 1.65) + self.wstat("laser", "charge", 0.75) - 0.40;
                if self.t - tr.laser_fired_t < cd_win {
                    laser = false;
                }
            }
        }
        let mut dps = self.dps_of(gun, laser, tr.light, tr.mortar, tr.cubes, d, true, false);
        if self.p.cookoff_aware && laser && self.has("mortar") >= 0 {
            let cook = self.wstat("laser", "cookoff", 0.0);
            if cook > 0.0 && d < self.wstat("laser", "range", 280.0) {
                let l = self.h.config.weapons.get("laser");
                dps += stat(l, "damage", 315.0) / (stat(l, "charge", 0.75) + stat(l, "cooldown", 1.65)) * self.laser_hit_rate() * cook * self.cube_damage_mul(tr.cubes);
            }
        }
        dps
    }

    /// (time they need to kill us) / (time we need to kill them); > 1 favours us.
    #[allow(dead_code)]
    pub(crate) fn advantage(&self, tr: &Track, d: f64) -> f64 {
        let mine = self.my_dps(d);
        let theirs = self.their_dps(tr, d) * (1.0 - self.armor);
        if mine < 1.0 {
            return 0.0;
        }
        if theirs < 1.0 {
            return 10.0;
        }
        let hp = if tr.hp <= 0.0 { tr.max_hp } else { tr.hp };
        (self.hp / theirs) / (hp / mine)
    }
}
