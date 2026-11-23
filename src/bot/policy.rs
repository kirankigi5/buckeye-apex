//! The per-decision policy: who to shoot, fight or flight, where to go, and the triggers.

use super::combat::AimTarget;
use super::hidemap::HM_ROOF;
use super::movement::{PosCost, SteerReq};
use super::roof::{ROOF_APPROACH, ROOF_CLIMB, ROOF_NONE, ROOF_ON};
use super::strategy::{Loot, Stance, Threat};
use super::*;
use crate::proto::{Action, DropObs};

/// Mutable movement bookkeeping shared by the goal setter.
struct Goal {
    heading: f64,
    have_heading: bool,
    goal_ok: bool,
    navigating: bool,
    last_goal: [f64; 2],
}

impl Bot {
    fn set_goal(&mut self, g: &mut Goal, rq: &mut SteerReq, gx: f64, gy: f64, w: f64, boost: bool) -> f64 {
        let (dx, dy, d, ok) = self.nav_dir(gx, gy);
        g.goal_ok = ok;
        g.navigating = true;
        g.last_goal = [gx, gy];
        rq.want_x = dx;
        rq.want_y = dy;
        rq.want_w = w;
        rq.boost = boost;
        if dx != 0.0 || dy != 0.0 {
            g.heading = dy.atan2(dx);
            g.have_heading = true;
        }
        d
    }

    fn chain_cost(rq: &mut SteerReq, f: impl Fn(f64, f64) -> f64 + 'static) {
        let prev = rq.pos_cost.take();
        let c: PosCost = Rc::new(move |x, y| {
            let base = prev.as_ref().map_or(0.0, |p| p(x, y));
            base + f(x, y)
        });
        rq.pos_cost = Some(c);
    }

    /// The per-decision policy.
    pub(crate) fn think(&mut self, act: &mut Action) {
        self.pick_upgrade(act);
        self.manage_weapons(act);
        let mut st = self.belief();
        // the end is when the circle closes to nothing (or the clock, if that comes first): the
        // mechs left in the collapse burn in HP order, so arrive there at full health
        let zone_end: f64 = self.h.config.zone.phases.iter().map(|p| p.hold_s + p.shrink_s).sum();
        let duration = stat(Some(&self.h.config.match_), "duration_s", 480.0);
        let end_t = if zone_end > 0.0 { zone_end.min(duration) } else { duration };
        let late = self.p.endgame_s > 0.0 && self.t > end_t - self.p.endgame_s;
        if late {
            // endgame: no even fights (the winner arrives at the collapse hurt); exact thresholds below
            st.engage_ratio = 1e9;
        }
        self.late = late;
        if self.p.k_engage && !late && st.avoid {
            // a*(k): advantage at which the win chance p = Phi(ln a / 0.5) beats k/(2+h(k-1)),
            // h = 1.1 (cubes from a kill), k = mechs alive besides us
            const A_STAR: [f64; 16] = [1.0, 1.0, 1.2, 1.33, 1.41, 1.47, 1.52, 1.56, 1.59, 1.62, 1.65, 1.67, 1.68, 1.7, 1.71, 1.72];
            let k = (self.o.alive - 1).clamp(0, 15) as usize;
            st.engage_ratio = st.engage_ratio.max(A_STAR[k]);
        }
        self.st = st.clone();
        if self.o.tick % 6 == 0 {
            self.mark_seen();
        }
        let zi = self.zone_info();
        let ths = self.assess();
        let inc = self.gather_incoming();
        let hp_frac = self.hp / self.max_hp;

        // --- who would we shoot
        let mut tgt_i: Option<usize> = None;
        let mut best_s = -1.0;
        let mut engaged_dps = 0.0;
        let mut n_engaged = 0;
        for (i, th) in ths.iter().enumerate() {
            if th.engaged {
                engaged_dps += th.dps;
                n_engaged += 1;
            }
            if !th.tr.listed {
                continue;
            }
            let mut s = 1.0 / (1.0 + th.d / 60.0);
            if th.tr.los {
                s *= 3.0;
            }
            if th.engaged {
                s *= 1.6;
            }
            if th.tr.max_hp > 0.0 {
                let kf = if self.p.kill_focus > 0.0 { self.p.kill_focus } else { 1.0 };
                s *= 1.0 + kf * (1.0 - th.tr.hp / th.tr.max_hp);
            }
            if th.tr.id == self.target_id {
                s *= 1.2;
            }
            if self.p.rear_bonus > 0.0 && self.is_rear(&th.tr) {
                s *= 1.0 + self.p.rear_bonus;
            }
            if self.p.cookoff_bonus > 0.0 && self.has("laser") >= 0 && th.tr.mortar && self.wstat("laser", "cookoff", 0.0) > 0.0 {
                s *= 1.0 + self.p.cookoff_bonus;
            }
            if self.p.threat_focus {
                if th.tr.light && th.d < self.wstat("lightning", "range", 45.0) + 35.0 {
                    s *= 1.25;
                } else if th.tr.laser {
                    s *= 1.18;
                }
                if th.tr.max_hp > 0.0 && th.tr.hp > 0.0 && th.tr.hp / th.tr.max_hp < 0.35 {
                    s *= 1.25;
                }
            }
            if s > best_s {
                best_s = s;
                tgt_i = Some(i);
            }
        }
        let tgt: Option<&Threat> = tgt_i.map(|i| &ths[i]);
        self.target_id = tgt.map_or(-1, |t| t.tr.id);
        let watchers = self.seen_by(&ths, 230.0);

        // Endgame thresholds from the placement model: start a fight only when the win chance beats
        // k/(2+h(k-1)), k = mechs left besides us, h ~ 0.6 with no time to heal before the collapse.
        // That is advantage 1.45 with two others and practically never with more. With one other the
        // collapse decides: it burns us both and the lower health dies first, so fight exactly when we
        // would arrive there behind on health (projecting regen at regen_per_s after regen_delay).
        let mut burn_fight = false;
        if late {
            let k = self.o.alive - 1;
            if k <= 1 {
                st.engage_ratio = 1e9; // ahead or level at the collapse: waiting wins (Theorem 2)
                if let Some(t) = tgt {
                    let left = (end_t - self.t).max(0.0);
                    let rate = self.mech("regen_per_s", 45.0);
                    let delay = self.mech("regen_delay", 5.0);
                    // health on arrival at the collapse if nobody touches either of us
                    let proj = |hp: f64, max_hp: f64| (hp + rate * (left - delay).max(0.0)).min(max_hp.max(hp));
                    let us = proj(self.hp, self.max_hp);
                    let them = proj(t.tr.hp, t.tr.max_hp.max(1.0));
                    burn_fight = !t.tr.npc && t.tr.listed && them > us + 20.0;
                }
            } else if k == 2 {
                st.engage_ratio = 1.45; // a*(2) with h = 0.6
            } else if k == 3 {
                st.engage_ratio = 2.16; // a*(3) with h = 0.6
            } else {
                st.engage_ratio = 1e9; // p*(k) >= 1 for k >= 4 with h = 0.6: never start one
            }
        }

        // --- fight or flight
        let under_fire = self.t - self.last_weapon_dt < 3.0;
        let danger = n_engaged > 0 || under_fire;
        let mut overall = 10.0;
        if engaged_dps > 1.0 {
            let mine = tgt.map_or(1.0, |t| self.my_dps(t.d));
            let mut need = 0.0;
            for th in &ths {
                if th.engaged {
                    need += th.tr.hp.max(20.0) / mine.max(1.0);
                }
            }
            overall = (self.hp / (engaged_dps * (1.0 - self.armor))) / need.max(0.1);
            if n_engaged == 1 {
                if let Some(t) = tgt {
                    if t.engaged {
                        overall = t.adv;
                    }
                }
            }
        }
        if st.stealth {
            if let Some(t) = tgt {
                if t.engaged {
                    let f = clamp((t.d - 60.0) / 60.0, 0.0, 1.0);
                    st.disengage = self.p.close_disengage + (st.disengage - self.p.close_disengage) * f;
                }
            }
        }
        let visible_foe = tgt.is_some_and(|t| t.engaged && t.tr.los && t.tr.listed);
        let known_foe = tgt.is_some_and(|t| t.engaged && t.tr.listed);
        if self.retreating {
            let healed = hp_frac >= self.p.return_hp;
            let winnable = overall > (st.disengage * 1.25).max(0.95);
            if healed && (!danger || winnable) {
                self.retreating = false;
            } else if !danger && hp_frac >= self.p.return_hp - 0.1 {
                self.retreating = false;
            } else if winnable && hp_frac > 0.3 && n_engaged <= 1 && known_foe {
                self.retreating = false;
            } else if visible_foe && tgt.unwrap().d < 45.0 && self.t - self.retreat_t > 2.0 && n_engaged <= 1 && overall > 0.35 {
                self.retreating = false;
                self.cornered_t = self.t;
            } else if self.p.retreat_run_max > 0.0 && st.avoid && visible_foe && self.t - self.retreat_t > 0.5 && !self.cover_close(&ths, tgt) {
                self.retreating = false;
                self.cornered_t = self.t;
                self.stand_t = self.t;
            }
        } else if danger && self.t - self.cornered_t > 4.0 {
            let phantoms = ths.iter().filter(|t| t.phantom).count();
            let dmg_rate = self.recent_dmg / 3.0;
            let ttd = self.hp / dmg_rate.max(1.0);
            let mut leave = if n_engaged <= 1 && known_foe {
                overall < self.p.duel_disengage && tgt.unwrap().d > 30.0 && ttd < 25.0
            } else if n_engaged >= 2 {
                (ttd < self.p.leave_ttd && hp_frac < 0.8) || (hp_frac < self.p.retreat_hp && ttd < 2.0 * self.p.leave_ttd)
            } else {
                under_fire && phantoms > 0 && (ttd < 1.5 * self.p.leave_ttd || hp_frac < 0.6)
            };
            if leave && self.p.retreat_run_max > 0.0 && st.avoid && visible_foe && !self.cover_close(&ths, tgt) {
                leave = false;
                self.stand_t = self.t;
            }
            if leave {
                self.retreating = true;
                self.retreat_t = self.t;
                self.cover_cell = -1;
            }
        }
        if let Some(t) = tgt {
            if t.tr.los && t.tr.listed && t.tr.hp > 0.0 && t.tr.hp < self.my_dps(t.d) * 1.2 + 40.0 && hp_frac > 0.12 {
                self.retreating = false;
            }
        }

        let patient = st.name == "squad" && self.p.patient_secs > 0.0 && self.t < self.p.patient_secs && self.o.self_.cubes < self.p.patient_cubes;
        let mut want_fight = false;
        let mut slip = false;
        let mut plink = false;
        let mut unprovoked = false;
        let mut roof_duck = false;
        let mut fight_why = "";
        let on_roof = self.on_roof() >= 0 && self.o.self_.grounded;
        let roof_wanted = self.roof_wanted(&st, &zi);
        let mut hunt_r = self.p.hunt_range;
        if self.has("laser") >= 0 && self.p.hunt_range_laser > hunt_r {
            hunt_r = self.p.hunt_range_laser;
        }
        if let (Some(t), false) = (tgt, self.retreating) {
            if patient && n_engaged <= 1 && t.d > self.p.patient_range && hp_frac > 0.55 {
                // leave it alone for now
            } else if t.engaged {
                if st.stealth {
                    if t.d < self.p.stealth_fight_range || t.adv >= st.engage_ratio {
                        want_fight = true;
                    }
                } else if st.avoid {
                    let provoked = !self.p.provoke_rule
                        || self.t - t.tr.hurt_me_t < 4.0
                        || t.d < self.p.avoid_range
                        || (t.tr.laser && t.tr.charging && self.t - t.tr.aim_at_me_t < 0.4)
                        || (self.p.laser_preempt && t.tr.laser && t.tr.los && self.t - t.tr.aim_at_me_t < 0.6);
                    let far = t.d >= self.p.avoid_range && t.adv < self.p.avoid_adv;
                    let heavy = t.tr.laser || self.recent_dmg / 3.0 > self.p.plink_max_dps || hp_frac < 0.5 || n_engaged >= 2;
                    let (mut edge_thr, mut edge_big) = (self.p.avoid_adv, self.p.cover_edge_adv);
                    if self.p.edge_hold > 0.0 {
                        if self.t - self.edge_on_t < self.p.edge_hold {
                            edge_thr *= 0.75;
                            edge_big *= 0.75;
                        } else if self.t - self.slip_on_t < self.p.edge_hold {
                            edge_thr *= 1.3;
                            edge_big *= 1.3;
                        }
                    }
                    let edge = (self.laser_edge(&t.tr) && t.adv >= edge_thr) || t.adv >= edge_big;
                    if edge && provoked {
                        self.edge_on_t = self.t;
                    }
                    let cover_first = self.p.cover_first && provoked && !edge && t.d >= self.p.cover_min_range;
                    if !provoked {
                        unprovoked = true;
                    } else if cover_first && !(on_roof && roof_wanted) && self.slip_from_x(t, &ths, true) {
                        slip = true;
                    } else if on_roof && roof_wanted && !self.enemy_on_roof(&ths) {
                        roof_duck = true;
                    } else if far && heavy && self.slip_from(t, &ths) {
                        slip = true;
                    } else if far && !heavy && self.p.plink_max_dps > 0.0 {
                        plink = true;
                    } else if far && self.slip_from(t, &ths) {
                        slip = true;
                    } else {
                        want_fight = true;
                        fight_why = if t.adv >= self.p.avoid_adv {
                            ".edge"
                        } else if t.d < self.p.avoid_range && !(self.p.cover_first && t.d >= self.p.cover_min_range) {
                            ".close"
                        } else {
                            ".nocover"
                        };
                        if self.t - self.stand_t < 1.0 {
                            fight_why = ".stand";
                        }
                    }
                } else {
                    want_fight = true;
                }
            } else if burn_fight && t.d < hunt_r.max(self.p.hunt_range) + 60.0 {
                want_fight = true;
                fight_why = ".burn";
            } else if st.hunt
                && t.adv >= st.engage_ratio
                && t.d < hunt_r
                && t.tr.los
                && hp_frac > 0.6
                && (!self.p.hunt_laser_edge || !st.avoid || self.laser_edge(&t.tr) || t.adv >= self.p.cover_edge_adv)
            {
                want_fight = true;
                fight_why = ".hunt";
            } else if st.name == "duel" && hp_frac > 0.5 && !late {
                want_fight = true;
            } else if st.stealth && t.tr.los && t.d < 85.0 && t.tr.hp < 0.25 * t.tr.max_hp.max(1.0) && t.adv > 1.5 {
                want_fight = true;
            }
        }
        // With a laser and cover at hand, fight from behind it.
        let mut peek = false;
        let peek_li = self.has("laser");
        if let Some(t) = tgt {
            if self.p.peek
                && (st.avoid || self.p.squad_peek)
                && peek_li >= 0
                && !self.retreating
                && t.tr.listed
                && !t.tr.npc
                && (want_fight || slip || roof_duck)
                && t.d > self.p.peek_min_range
                && t.d < 272.0
            {
                let roof_id = if on_roof && roof_wanted { self.on_roof() } else { -1 };
                if self.peek_plan_for(t, &ths, roof_id) {
                    peek = true;
                    want_fight = false;
                    slip = false;
                    roof_duck = false;
                }
            }
        }
        if plink && self.choose_loot(true, &st).is_none() {
            plink = false;
            if self.slip_from(tgt.unwrap(), &ths) {
                slip = true;
            } else {
                want_fight = true;
            }
        }
        let mut quiet = watchers.is_empty() && !danger;
        if quiet && ths.iter().any(|th| self.t - th.tr.last_seen < 6.0 && th.d < 180.0) {
            quiet = false;
        }
        if !quiet {
            self.unquiet_t = self.t;
        } else if self.t - self.unquiet_t < self.p.quiet_hold_s {
            quiet = false;
        }

        // --- movement
        let mut rq = SteerReq { threats: inc, ..Default::default() };
        let li = self.has("laser");
        if li >= 0 && self.slots[li as usize].charge > 0.0 {
            rq.slow = true;
        }
        let mut laser_threat_now = false;
        for th in &ths {
            let tr = &th.tr;
            if !tr.laser || !tr.charging || !tr.los {
                continue;
            }
            let mut aim_match = self.t - tr.aim_at_me_t < 0.25;
            if self.p.laser_dodge_v2 && !aim_match {
                let bear = (self.pos[1] - tr.pos[1]).atan2(self.pos[0] - tr.pos[0]);
                let off = ang_diff(tr.aim[0], bear).abs();
                aim_match = self.t - tr.aim_at_me_t < 0.55 || off < 0.42;
            }
            if aim_match {
                laser_threat_now = true;
                let sc = tr.scale(self.size_per_cube);
                let (ex, ey, ez) = (tr.pos[0], tr.pos[1], tr.pos[2] + 7.0 * sc);
                let mut dmg = self.wstat("laser", "damage", 315.0) * self.cube_damage_mul(tr.cubes) * 0.85;
                if self.has("mortar") >= 0 {
                    dmg *= 1.0 + self.wstat("laser", "cookoff", 0.0);
                }
                let (aim_cos, aim_sin) = (tr.aim[0].cos(), tr.aim[0].sin());
                let beam_w = self.p.laser_beam_w;
                let beam_half = self.radius + 3.0;
                let las_range = self.wstat("laser", "range", 280.0);
                let w = self.w.clone();
                let (pz, h) = (self.pos[2], self.height);
                let dodge_v2 = self.p.laser_dodge_v2;
                Self::chain_cost(&mut rq, move |x, y| {
                    let mut c = 0.0;
                    if w.visible(ex, ey, ez, x, y, pz + h * 0.5) || w.visible(ex, ey, ez, x, y, pz + h * 0.9) {
                        c += dmg;
                        if dodge_v2 && beam_w > 0.0 {
                            let (bx, by) = (ex + aim_cos * las_range, ey + aim_sin * las_range);
                            let (perp, _) = seg_point_dist_2d(x, y, ex, ey, bx, by);
                            if perp < beam_half * 2.5 {
                                c += beam_w * (1.0 - perp / (beam_half * 2.5));
                            }
                        }
                    }
                    c
                });
                rq.look_ticks = 12;
                rq.dodge_boost = true;
            }
        }
        if self.p.squad_laser_cover && laser_threat_now && !st.avoid && !self.retreating && !peek && !slip && !(on_roof && roof_wanted) {
            let finish_soon = tgt.is_some_and(|t| t.tr.los && t.tr.hp > 0.0 && t.tr.hp < self.my_dps(t.d) * 1.1 + 40.0);
            if !finish_soon {
                let c = self.find_cover(&ths, 24.0);
                if c >= 0 {
                    self.slip_cell = c;
                    self.slip_t = self.t;
                    self.slip_on_t = self.t;
                    slip = true;
                    want_fight = false;
                }
            }
        }
        if self.p.anti_lightning_w > 0.0 {
            let have_light = self.has("lightning") >= 0 && self.energy > 18.0;
            if !have_light || hp_frac < 0.55 {
                let light_base = self.wstat("lightning", "range", 45.0);
                let pad = if self.p.anti_lightning_pad <= 0.0 { 10.0 } else { self.p.anti_lightning_pad };
                for th in &ths {
                    let tr = &th.tr;
                    if !tr.light || (!tr.listed && self.t - tr.last_seen > 2.5) {
                        continue;
                    }
                    let mut safe_r = light_base + pad;
                    if tr.cubes >= 6 {
                        safe_r += 6.0;
                    }
                    if th.d > safe_r + 22.0 {
                        continue;
                    }
                    let (lx, ly) = (tr.pos[0], tr.pos[1]);
                    let w = self.p.anti_lightning_w;
                    Self::chain_cost(&mut rq, move |x, y| {
                        let d = hyp(x - lx, y - ly);
                        if d < safe_r {
                            let f = (safe_r - d) / safe_r;
                            w * f * (1.0 + (safe_r - d) / 20.0)
                        } else {
                            0.0
                        }
                    });
                    if th.d < safe_r + 4.0 && tr.los {
                        rq.dodge_boost = true;
                        if rq.look_ticks < 10 {
                            rq.look_ticks = 10;
                        }
                    }
                }
            }
        }
        if let Some(t) = tgt {
            if self.p.rear_avoid_w > 0.0 && t.tr.los && n_engaged >= 2 {
                let half_front = self.mech("front_arc_deg", 180.0) * 0.5 * std::f64::consts::PI / 180.0;
                let (tx, ty) = (t.tr.pos[0], t.tr.pos[1]);
                let extras: Vec<[f64; 2]> = ths
                    .iter()
                    .filter(|th| th.engaged && th.tr.id != t.tr.id && th.tr.los && th.d < 210.0)
                    .map(|th| [th.tr.pos[0], th.tr.pos[1]])
                    .collect();
                if !extras.is_empty() {
                    let w = self.p.rear_avoid_w;
                    Self::chain_cost(&mut rq, move |x, y| {
                        let face = (ty - y).atan2(tx - x);
                        let mut c = 0.0;
                        for e in &extras {
                            let bear = (e[1] - y).atan2(e[0] - x);
                            if ang_diff(bear, face).abs() > half_front {
                                c += w;
                            }
                        }
                        c
                    });
                }
            }
        }
        let mut g = Goal { heading: 0.0, have_heading: false, goal_ok: true, navigating: false, last_goal: [0.0; 2] };
        let mut shoot_item: Option<AimTarget> = None;
        let mut shoot_hp = 0.0;
        let mut hold_fire = false;
        let mut peek_charge = false;
        let mut direct = false;
        self.hold_climb = st.avoid && (!watchers.is_empty() || under_fire);
        if st.avoid && !self.hold_climb && ths.iter().any(|th| th.tr.listed && th.tr.los && th.d < self.p.climb_clear_range) {
            self.hold_climb = true;
        }
        self.hold_roof_move = self.p.roof_leave_quiet > 0.0
            && st.avoid
            && on_roof
            && (!watchers.is_empty() || under_fire)
            && zi.t_pass > self.p.roof_leave_quiet
            && !zi.urgent;
        let mut roof_cover_cell = -1i64;
        let mut want_roof_cover = st.stealth && (self.retreating || want_fight || !watchers.is_empty() || under_fire);
        if st.avoid && (roof_duck || self.retreating || !watchers.is_empty() || under_fire) {
            want_roof_cover = true;
            self.roof_cover_hold_t = self.t;
        } else if st.avoid && self.t - self.roof_cover_hold_t < self.p.roof_cover_hold {
            want_roof_cover = true;
        }
        if on_roof && roof_wanted && want_roof_cover && !self.enemy_on_roof(&ths) {
            if self.t - self.roof_cover_t > 0.4 {
                self.roof_cover_c = self.roof_cover(self.on_roof(), &ths);
                self.roof_cover_t = self.t;
            }
            roof_cover_cell = self.roof_cover_c;
        } else {
            self.roof_cover_c = -1;
            self.roof_cover_t = -100.0;
        }
        if roof_duck && roof_cover_cell < 0 && !self.retreating {
            want_fight = true;
        }
        if self.roof_first_ok && !self.roof_first_checked && self.p.roof_first_secs > 0.0 && roof_wanted {
            self.roof_first_checked = true;
            let mut ok = false;
            if let Some((_, acc)) = self.choose_roof() {
                let (gx, gy) = self.nav.pos(acc.ground);
                ok = hyp(gx - self.pos[0], gy - self.pos[1]) <= self.p.roof_first_dist;
            }
            self.roof_first_ok = ok;
        }
        let roof_first = roof_wanted && !on_roof && self.t < self.p.roof_first_secs && self.roof_first_ok && self.roof_first_checked;
        let mut shy = false;
        let mut shy_loot: Option<Loot> = None;
        if self.shy_phase(&st) && !want_fight && !slip && !peek && !roof_duck && !self.retreating && !on_roof {
            shy = self.exposed_to(self.p.shy_mem, self.p.shy_range);
            if shy {
                shy_loot = self.choose_loot(false, &st);
                if shy_loot.as_ref().is_some_and(|l| dist2(self.pos, l.pos) <= 60.0) {
                    shy = false;
                }
            }
        }
        if self.roof.state == ROOF_APPROACH && self.t - self.roof.step_t > 2.5 {
            self.roof.state = ROOF_NONE;
        }

        let pest = self.pest();
        if zi.critical || (zi.urgent && (!want_fight || tgt.is_none_or(|t| t.d > 60.0))) || (zi.urgent && zi.t_pass < 5.0) {
            self.mode = "zone".into();
            self.roof.state = ROOF_NONE;
            if self.p.zone_slack > 0.0 && self.loot_id >= 0 {
                self.ban_until.insert(self.loot_id, self.t + 20.0);
                self.loot_id = -1;
            }
            let boost = zi.critical && self.energy > 15.0;
            self.set_goal(&mut g, &mut rq, zi.goal[0], zi.goal[1], 3.0, boost);
            rq.zone_free = true;
            if !zi.walkable {
                self.mode = "zone-over".into();
                self.overland(act, zi.goal[0], zi.goal[1]);
                direct = true;
            }
        } else if self.roof.state == ROOF_CLIMB {
            self.mode = "climb".into();
            self.roof_climb_step(act);
            direct = true;
        } else if peek {
            self.mode = "peek".into();
            let (cell, charge) = self.peek_step(&ths, peek_li as usize, hp_frac);
            peek_charge = charge;
            let (gx, gy) = self.nav.pos(cell as usize);
            if self.set_goal(&mut g, &mut rq, gx, gy, 2.6, false) < 1.0 {
                rq.want_w = 0.0;
                rq.hold = true;
            }
            rq.fence_roof = on_roof && roof_wanted;
        } else if roof_cover_cell >= 0 {
            self.mode = "roof-cover".into();
            let (x, y) = self.nav.pos(roof_cover_cell as usize);
            if self.set_goal(&mut g, &mut rq, x, y, 2.4, false) < 1.8 {
                rq.want_w = 0.0;
                rq.hold = true;
            }
            rq.fence_roof = true;
            hold_fire = tgt.is_none_or(|t| !t.engaged || t.d > self.p.ffa_return_fire);
        } else if slip {
            self.mode = "slip".into();
            let deep_slip = self.p.cover_margin > 0.0
                && self.deep_cell >= 0
                && self.t - self.deep_t < 8.0
                && self.t - self.last_weapon_dt > 0.45
                && self.cell_hidden(self.deep_cell, &ths);
            if !deep_slip {
                self.deep_cell = -1;
            }
            if deep_slip {
                let (cx, cy) = self.nav.pos(self.deep_cell as usize);
                let d = self.set_goal(&mut g, &mut rq, cx, cy, 2.5, false);
                if d < 1.5 || !g.goal_ok {
                    rq.want_w = 0.0;
                    rq.hold = true;
                    self.mode = "ambush".into();
                }
            } else {
                let (cx, cy) = self.nav.pos(self.slip_cell as usize);
                let boost = self.energy > 25.0 && under_fire;
                let d = self.set_goal(&mut g, &mut rq, cx, cy, 2.5, boost);
                if d < 0.9 || (d < 3.0 && self.hidden_here(&ths)) {
                    if self.p.cover_margin > 0.0 && self.t - self.deep_try_t > 0.6 {
                        self.deep_try_t = self.t;
                        let c = self.deeper_cover(&ths, 45.0, false);
                        if c >= 0 {
                            self.deep_cell = c;
                            self.deep_t = self.t;
                        }
                    }
                    if self.deep_cell >= 0 {
                        let (cx, cy) = self.nav.pos(self.deep_cell as usize);
                        self.set_goal(&mut g, &mut rq, cx, cy, 2.5, false);
                    } else {
                        rq.want_w = 0.0;
                        rq.hold = true;
                        self.mode = "ambush".into();
                    }
                }
            }
        } else if self.retreating {
            self.mode = "retreat".into();
            if self.p.retreat_pest && n_engaged == 0 {
                if let Some(tr) = &pest {
                    if tr.los && dist2(self.pos, tr.pos) < 120.0 {
                        shoot_item = Some(self.target_of(tr));
                        shoot_hp = tr.hp;
                    }
                }
            }
            let deeper = (self.p.run_on_range > 0.0 || self.p.cover_margin > 0.0) && st.avoid;
            let mut deep = deeper && self.deep_cell >= 0 && self.t - self.deep_t < 8.0;
            if deep && (self.t - self.last_weapon_dt < 0.45 || !self.cell_hidden(self.deep_cell, &ths)) {
                deep = false;
            }
            if !deep {
                self.deep_cell = -1;
                if self.t - self.cover_t > 0.5 {
                    self.cover_cell = self.find_cover(&ths, 90.0);
                    self.cover_t = self.t;
                }
            }
            if deep {
                self.mode = "retreat-on".into();
                let (cx, cy) = self.nav.pos(self.deep_cell as usize);
                let boost = self.energy > 35.0 && self.nearest_eye(&ths) < 60.0;
                let d = self.set_goal(&mut g, &mut rq, cx, cy, 2.5, boost);
                if d < 1.5 || !g.goal_ok {
                    self.deep_cell = -1;
                    self.cover_t = -100.0;
                }
            } else if self.cover_cell >= 0 {
                let (cx, cy) = self.nav.pos(self.cover_cell as usize);
                let boost = self.energy > 20.0 && under_fire;
                let d = self.set_goal(&mut g, &mut rq, cx, cy, 2.5, boost);
                if d < 0.9 || (d < 3.0 && self.hidden_here(&ths)) {
                    if deeper && self.t - self.deep_try_t > 0.6 {
                        self.deep_try_t = self.t;
                        let c = self.deeper_cover(&ths, 75.0, true);
                        if c >= 0 {
                            self.deep_cell = c;
                            self.deep_t = self.t;
                        }
                    }
                    if self.deep_cell >= 0 {
                        self.mode = "retreat-on".into();
                        let (cx, cy) = self.nav.pos(self.deep_cell as usize);
                        let boost = self.energy > 35.0 && self.nearest_eye(&ths) < 60.0;
                        self.set_goal(&mut g, &mut rq, cx, cy, 2.5, boost);
                    } else {
                        rq.want_w = 0.0;
                        rq.hold = true;
                        self.mode = "heal".into();
                    }
                }
            } else if let Some(from) = self.flee_from(tgt, &ths) {
                let (dx, dy) = norm2(self.pos[0] - from.tr.pos[0], self.pos[1] - from.tr.pos[1]);
                let (gx, gy) = (self.pos[0] + dx * 30.0, self.pos[1] + dy * 30.0);
                let boost = self.energy > 30.0;
                self.set_goal(&mut g, &mut rq, gx, gy, 2.0, boost);
            } else if self.p.cover_margin > 0.0 && st.avoid && zi.out_next < -5.0 && self.t - self.last_npc_dt > 4.0 && self.t - self.last_weapon_dt < 30.0 {
                rq.want_w = 0.0;
                rq.hold = true;
                self.mode = "heal".into();
            } else {
                self.set_goal(&mut g, &mut rq, zi.goal[0], zi.goal[1], 1.5, false);
            }
            if st.stealth && tgt.is_some_and(|t| t.d > self.p.ffa_return_fire) {
                hold_fire = true;
            }
        } else if want_fight {
            self.mode = format!("fight{}", fight_why);
            let t = tgt.unwrap();
            let dr = self.drop_run(&st, hp_frac);
            let wr = if dr.is_none() || on_roof { self.weapon_run(32.0) } else { None };
            if let (Some(dr), false) = (&dr, on_roof) {
                self.mode = "fight+drop".into();
                let boost = self.energy > 40.0 && dr.lands_in < 6.0;
                let d = self.set_goal(&mut g, &mut rq, dr.pos[0], dr.pos[1], 2.4, boost);
                if d < 3.0 {
                    rq.want_w = 0.0;
                    rq.hold = true;
                }
            } else if let (Some(w), false) = (&wr, on_roof) {
                self.mode = "fight+arm".into();
                let (dx, dy, _, _) = self.nav_dir(w.pos[0], w.pos[1]);
                rq.want_x = dx;
                rq.want_y = dy;
                rq.want_w = 2.4;
            } else if let Some(c) = self.cube_run(Some(t), &st, hp_frac) {
                self.mode = "fight+cube".into();
                let (dx, dy, _, _) = self.nav_dir(c.pos[0], c.pos[1]);
                rq.want_x = dx;
                rq.want_y = dy;
                rq.want_w = 2.2;
            } else if let Some(near) = self.kite_from(&ths, n_engaged) {
                self.mode = "kite".into();
                let (np, nd) = (ths[near].tr.pos, ths[near].d);
                self.strafe(&mut rq, np, self.p.kite_min, self.p.kite_max, 2.0);
                if nd < 70.0 && self.energy > 45.0 {
                    rq.boost = true;
                }
            } else {
                self.fight_move(&mut rq, t);
            }
            rq.fence_roof = on_roof && roof_wanted;
        } else if shy {
            self.mode = "shy".into();
            if self.t - self.evade_t > 0.6 {
                self.evade_cell = self.find_cover(&ths, 110.0);
                self.evade_t = self.t;
            }
            if self.evade_cell >= 0 {
                let (cx, cy) = self.nav.pos(self.evade_cell as usize);
                let boost = self.p.shy_boost > 0.0 && self.energy > self.p.shy_boost;
                let d = self.set_goal(&mut g, &mut rq, cx, cy, 2.2, boost);
                if d < 0.9 || (d < 3.0 && self.hidden_here(&ths)) {
                    rq.want_w = 0.0;
                    rq.hold = true;
                }
            } else if let Some(l) = &shy_loot {
                self.set_goal(&mut g, &mut rq, l.pos[0], l.pos[1], 2.0, false);
            } else {
                let mut nearest: Option<&Threat> = None;
                for th in &ths {
                    if self.t - th.tr.last_seen <= self.p.shy_mem && nearest.is_none_or(|n| th.d < n.d) {
                        nearest = Some(th);
                    }
                }
                if let Some(n) = nearest {
                    let (dx, dy) = norm2(self.pos[0] - n.tr.pos[0], self.pos[1] - n.tr.pos[1]);
                    let (gx, gy) = (self.pos[0] + dx * 40.0, self.pos[1] + dy * 40.0);
                    self.set_goal(&mut g, &mut rq, gx, gy, 2.0, false);
                }
            }
            hold_fire = true;
        } else if pest.is_some() && (!st.stealth || watchers.is_empty()) {
            let tr = pest.clone().unwrap();
            self.mode = "pest".into();
            let at = self.target_of(&tr);
            let d = dist2(self.pos, tr.pos);
            if tr.los && d < 120.0 {
                shoot_item = Some(at);
                shoot_hp = tr.hp;
                self.strafe(&mut rq, tr.pos, 35.0, 60.0, 1.0);
            } else {
                self.set_goal(&mut g, &mut rq, tr.pos[0], tr.pos[1], 1.5, false);
            }
        } else if st.stealth && n_engaged == 0 && !on_roof && !roof_first && self.weapon_run(self.p.arm_radius).is_some() {
            let w = self.weapon_run(self.p.arm_radius).unwrap();
            self.mode = "arm".into();
            self.set_goal(&mut g, &mut rq, w.pos[0], w.pos[1], 2.2, false);
            if !g.goal_ok {
                self.ban_until.insert(w.id, self.t + 40.0);
            }
            hold_fire = true;
        } else if st.stealth && !watchers.is_empty() && !(on_roof && roof_wanted) && self.roof.state != ROOF_APPROACH {
            self.mode = "evade".into();
            if self.t - self.evade_t > 0.6 {
                self.evade_cell = self.find_cover(&ths, 110.0);
                self.evade_t = self.t;
            }
            if self.evade_cell >= 0 {
                let (cx, cy) = self.nav.pos(self.evade_cell as usize);
                let d = self.set_goal(&mut g, &mut rq, cx, cy, 2.2, false);
                if d < 0.9 || (d < 3.0 && self.hidden_here(&ths)) {
                    rq.want_w = 0.0;
                    rq.hold = true;
                }
            } else {
                let w = &ths[watchers[0]];
                let (dx, dy) = norm2(self.pos[0] - w.tr.pos[0], self.pos[1] - w.tr.pos[1]);
                let (gx, gy) = (self.pos[0] + dx * 40.0, self.pos[1] + dy * 40.0);
                self.set_goal(&mut g, &mut rq, gx, gy, 1.8, false);
            }
            hold_fire = tgt.is_none_or(|t| !t.engaged || t.d > self.p.ffa_return_fire);
        } else if self.p.regen_hold_hp > 0.0
            && !danger
            && hp_frac < self.p.regen_hold_hp
            && self.t - self.last_weapon_dt < 16.0
            && self.t - self.last_npc_dt > 3.0
            && zi.out_next < -5.0
        {
            self.mode = "regen-hold".into();
            rq.want_w = 0.0;
            rq.hold = true;
        } else {
            if patient {
                hold_fire = true;
            }
            let mut l: Option<Loot> = None;
            if (!st.stealth || quiet) && !roof_first && !late {
                l = self.choose_loot(danger && !patient, &st);
            } else if late && !roof_first && self.o.self_.cubes < self.h.config.cubes.stat_cap as i64 {
                // endgame: a cube a few steps away is +4% max HP for the final burn
                l = self.choose_loot(true, &st).filter(|l| l.kind == "cube" && dist2(self.pos, l.pos) < 22.0);
            }
            if let Some(lt) = &l {
                if on_roof && roof_wanted && lt.kind != "drop" && !(lt.kind == "pickup" && self.has("laser") < 0 && dist2(self.pos, lt.pos) < 90.0) {
                    l = None;
                }
            }
            if let Some(l) = l {
                self.mode = format!("loot:{}", l.kind);
                self.loot_id = l.id;
                let d = dist2(self.pos, l.pos);
                match l.kind {
                    "crate" => {
                        let it_hp = self.crates.get(&l.id).map(|it| it.hp);
                        let at = AimTarget {
                            pos: l.pos,
                            radius: stat(Some(&self.h.config.crates), "radius", 1.4),
                            center_z: stat(Some(&self.h.config.crates), "height", 2.4) * 0.5,
                            ..Default::default()
                        };
                        let vis = d < 70.0 && self.w.visible(self.pos[0], self.pos[1], self.pos[2] + self.muzzle_h, at.pos[0], at.pos[1], at.pos[2] + at.center_z);
                        if vis {
                            shoot_item = Some(at);
                            if let Some(hp) = it_hp {
                                shoot_hp = hp;
                            }
                        }
                        if d > 9.0 || !vis {
                            self.set_goal(&mut g, &mut rq, l.pos[0], l.pos[1], 1.5, false);
                        } else {
                            rq.hold = true;
                        }
                    }
                    "npc" => {
                        if let Some(tr) = self.tracks.get(&l.id).cloned() {
                            let at = self.target_of(&tr);
                            if tr.los && d < 110.0 {
                                shoot_item = Some(at);
                                shoot_hp = tr.hp;
                            }
                            if d > 60.0 || !tr.los {
                                self.set_goal(&mut g, &mut rq, l.pos[0], l.pos[1], 1.5, false);
                            } else {
                                self.strafe(&mut rq, tr.pos, 50.0, 70.0, 1.0);
                            }
                        }
                    }
                    "drop" => {
                        if d > 3.0 {
                            self.set_goal(&mut g, &mut rq, l.pos[0], l.pos[1], 2.0, false);
                        } else {
                            rq.hold = true;
                        }
                    }
                    _ => {
                        let mut sprint = false;
                        if l.kind == "pickup" && self.p.loot_boost > 0.0 && self.energy > self.p.loot_boost && d > 25.0 {
                            if let Some(it) = self.pickups.get(&l.id) {
                                if it.kind == "laser" && self.has("laser") < 0 {
                                    sprint = true;
                                }
                            }
                        }
                        self.set_goal(&mut g, &mut rq, l.pos[0], l.pos[1], 2.0, sprint);
                    }
                }
                if g.navigating && !g.goal_ok {
                    self.ban_until.insert(l.id, self.t + 40.0);
                }
            } else {
                self.loot_id = -1;
                self.mode = "hold".into();
                if self.shy_phase(&st) && !on_roof && self.mech_seen_within(self.p.shy_mem) {
                    self.mode = "lie-low".into();
                    rq.want_w = 0.0;
                    rq.hold = true;
                } else if zi.final_ && late && self.o.zone.radius > 14.0 && self.endgame_hold_cell() >= 0 {
                    // last circle, endgame: everyone else walks to the exact centre and brawls
                    // there; wait under cover inside half the radius until the circle is small
                    self.mode = "endgame-hold".into();
                    let (gx, gy) = self.nav.pos(self.hold_cell as usize);
                    if self.set_goal(&mut g, &mut rq, gx, gy, 1.6, false) < 2.5 {
                        rq.want_w = 0.0;
                        rq.hold = true;
                    }
                } else if zi.final_ {
                    let mut tol = 2.0;
                    let mut boost_final = false;
                    if self.p.final_hold {
                        tol = clamp(self.o.zone.radius * 0.06, 0.35, 2.0);
                        boost_final = self.o.zone.radius < 18.0 && self.energy > 15.0;
                    }
                    let nc = self.o.zone.next_center;
                    if self.set_goal(&mut g, &mut rq, nc[0], nc[1], 1.8, boost_final) < tol {
                        rq.want_w = 0.0;
                        rq.hold = true;
                    }
                } else if let Some((sx, sy, sid)) = self.seek_target(&st, hp_frac).filter(|_| !roof_first && !late) {
                    self.mode = "seek".into();
                    self.set_goal(&mut g, &mut rq, sx, sy, 1.8, false);
                    if !g.goal_ok && sid >= 0 {
                        self.seek_ban.insert(sid, self.t + 30.0);
                    }
                } else if let Some((rx, ry)) = self.roam_check(&st, hp_frac, roof_first, roof_wanted, quiet).filter(|_| !late) {
                    self.mode = "roam".into();
                    self.set_goal(&mut g, &mut rq, rx, ry, 1.5, false);
                    if !g.goal_ok && self.roam_cell >= 0 {
                        self.ban_until.insert(-(self.roam_cell + 1), self.t + 120.0);
                        self.roam_cell = -1;
                    }
                } else if let Some(ho) = self.hideout_for(&st, &zi, roof_wanted).filter(|h| h.class != HM_ROOF) {
                    self.mode = "hide".into();
                    self.roof.state = ROOF_NONE;
                    self.roof.want = -1;
                    self.roof.spot = -1;
                    let (gx, gy) = self.nav.pos(ho.cell);
                    if self.set_goal(&mut g, &mut rq, gx, gy, 1.5, false) < 2.5 {
                        rq.want_w = 0.0;
                        rq.hold = true;
                    }
                } else if let Some((gx, gy, settled)) = self.roof_plan_step(roof_wanted) {
                    self.mode = "roof".into();
                    if settled {
                        rq.hold = true;
                    } else {
                        self.set_goal(&mut g, &mut rq, gx, gy, 1.6, false);
                    }
                    rq.fence_roof = on_roof && self.roof.state == ROOF_ON;
                } else {
                    if self.hold_cell < 0 || self.t - self.hide_t > 2.0 {
                        self.hold_cell = self.choose_hide(&zi);
                        self.hide_t = self.t;
                    }
                    let (mut gx, mut gy) = (zi.goal[0], zi.goal[1]);
                    if self.hold_cell >= 0 {
                        (gx, gy) = self.nav.pos(self.hold_cell as usize);
                    }
                    if self.set_goal(&mut g, &mut rq, gx, gy, 1.4, false) < 2.5 {
                        rq.want_w = 0.0;
                        rq.hold = true;
                    }
                }
            }
        }
        let collapse = self.o.zone.radius < 8.0;
        if late && !collapse && !want_fight && tgt.is_none_or(|t| !t.engaged) {
            hold_fire = true; // dealing damage resets our own regen: only answer fire now
        }
        // Off the walk grid or heading for a place the grid cannot reach: go straight and jet over walls.
        if !direct && g.navigating && !rq.hold && !rq.fence_roof && (!self.on_ground_level() || !g.goal_ok) {
            let gd = hyp(g.last_goal[0] - self.pos[0], g.last_goal[1] - self.pos[1]);
            if gd > 3.0 {
                self.overland(act, g.last_goal[0], g.last_goal[1]);
                direct = true;
            }
        }
        // Walking somewhere under gun fire from range: zig-zag.
        if self.p.jink > 0.0 && !direct && g.navigating && !rq.hold && !rq.fence_roof && under_fire && (rq.want_x != 0.0 || rq.want_y != 0.0) {
            let far = ths.iter().any(|th| th.engaged && !th.phantom && th.d > 55.0 && self.t - th.tr.hurt_me_t < 3.0);
            let gd = hyp(g.last_goal[0] - self.pos[0], g.last_goal[1] - self.pos[1]);
            if far && gd > 12.0 {
                if self.t >= self.jink_t {
                    if self.rng.float64() < 0.85 {
                        self.jink_sign = -self.jink_sign;
                    }
                    self.jink_t = self.t + 0.3 + self.rng.float64() * 0.4;
                }
                let (px, py) = (-rq.want_y, rq.want_x);
                let (wx, wy) = norm2(rq.want_x + px * self.p.jink * self.jink_sign, rq.want_y + py * self.p.jink * self.jink_sign);
                rq.want_x = wx;
                rq.want_y = wy;
            }
        }
        if !direct {
            self.steer(act, &rq);
            self.unstick(act, g.navigating && !rq.hold && !rq.fence_roof);
        }

        // --- aim and triggers
        let unseen = ths.iter().any(|th| th.phantom && self.t - th.tr.hurt_me_t < 1.5);
        if unseen && tgt.is_none_or(|t| !t.engaged || !t.tr.los) {
            act.aim = [self.last_dmg_bear, 0.0];
            act.has_aim = true;
            let li = self.has("laser");
            if li >= 0 && self.slots[li as usize].charge > 0.0 {
                act.fire[li as usize] = true;
            }
        } else if plink && shoot_item.is_some() {
            self.engage(act, shoot_item.unwrap(), true, true, shoot_hp, true);
        } else if tgt.is_some_and(|_| !hold_fire && !unprovoked && (want_fight || tgt.unwrap().engaged || self.retreating || peek)) {
            let t = tgt.unwrap();
            let at = self.target_of(&t.tr);
            self.engage(act, at, t.tr.los, self.t - t.tr.los_t < 0.8, t.tr.hp, true);
            if peek_charge && peek_li >= 0 {
                act.fire[peek_li as usize] = true;
            }
        } else if let Some(si) = shoot_item {
            self.engage(act, si, true, true, shoot_hp, true);
        } else if tgt.is_some_and(|t| t.tr.los || self.t - t.tr.last_seen < 3.0) {
            let t = tgt.unwrap();
            let at = self.target_of(&t.tr);
            self.engage(act, at, t.tr.los, false, t.tr.hp, false);
        } else {
            self.idle_aim(act, g.heading, g.have_heading);
            let gi = self.has("gun");
            if gi >= 0 {
                let sl = &self.slots[gi as usize];
                if sl.reload <= 0.0 && sl.ammo < self.slot_stat(gi as usize, "mag", 12.0) * 0.67 {
                    act.reload = true;
                }
            }
            let li = self.has("laser");
            if li >= 0 && self.slots[li as usize].charge > 0.0 && self.tracks.values().any(|tr| !tr.npc && self.t - tr.last_seen < 1.5) {
                act.fire[li as usize] = true;
            }
        }
        if self.p.cover_reload && !act.reload && shoot_item.is_none() {
            let gi = self.has("gun");
            if gi >= 0 && !act.fire[gi as usize] {
                let sl = &self.slots[gi as usize];
                let mag = self.slot_stat(gi as usize, "mag", 12.0);
                let frac = if self.ffa { 0.25 } else { 0.60 };
                if sl.reload <= 0.0 && sl.ammo <= mag * frac && sl.ammo < mag && self.t - self.last_enemy_los_t >= 1.1 {
                    act.reload = true;
                }
            }
        }
        self.mode_label = format!("{}:{}", st.name, self.mode);
        let mut wl = String::new();
        for s in &self.slots {
            if s.ok {
                wl += &s.kind.chars().take(2).collect::<String>();
            }
        }
        let td = tgt.map_or(-1.0, |t| t.d);
        act.label = format!("{} h{:.0} a{} e{} k{} c{} {} d{:.0}", self.mode_label, self.hp, self.o.alive, n_engaged, self.kills, self.o.self_.cubes, wl, td);
    }

    /// The cover cell to wait at in the last circle (-1: none). Re-chosen every second, and at once
    /// when the shrinking circle leaves the current one outside half its radius.
    fn endgame_hold_cell(&mut self) -> i64 {
        let z = &self.o.zone;
        let stale = self.hold_cell < 0 || self.t - self.hide_t > 1.0 || {
            let (x, y) = self.nav.pos(self.hold_cell as usize);
            hyp(x - z.center[0], y - z.center[1]) > z.radius * 0.55
        };
        if stale {
            let zi = super::strategy::ZoneInfo::default();
            self.hold_cell = self.choose_hide(&zi);
            self.hide_t = self.t;
        }
        self.hold_cell
    }

    /// The roam branch of the hold logic: a goal only when exploring is what this stance does now.
    fn roam_check(&mut self, st: &Stance, hp_frac: f64, roof_first: bool, roof_wanted: bool, quiet: bool) -> Option<(f64, f64)> {
        let r = self.roam_target(st, hp_frac)?;
        if !roof_first && (!roof_wanted || (!self.armed() && quiet)) {
            Some(r)
        } else {
            None
        }
    }

    /// A weapon on the ground that is worth going for right now even with enemies about.
    pub(crate) fn weapon_run(&self, max_d: f64) -> Option<Item> {
        let mut best: Option<&Item> = None;
        let mut bv = 0.0;
        for it in self.pickups.values() {
            if !it.ground || !self.same_comp(it.cell as i64) {
                continue;
            }
            if self.ban_until.get(&it.id).is_some_and(|&u| self.t < u) {
                continue;
            }
            let (ok, rep) = self.want_pickup(&it.kind);
            if !ok {
                continue;
            }
            let mut v = self.weapon_value(&it.kind);
            if rep >= 0 {
                v -= self.weapon_value(&self.slots[rep as usize].kind);
                if v < self.p.arm_swap_min {
                    continue;
                }
            }
            let d = dist2(self.pos, it.pos);
            if d > max_d {
                continue;
            }
            if !self.in_future_zone(it.pos[0], it.pos[1], d / self.walk + 2.0, 5.0) {
                continue;
            }
            let contested = self.tracks.values().any(|tr| {
                if tr.npc || self.t - tr.last_seen > 5.0 {
                    return false;
                }
                let de = dist2(tr.pos, it.pos);
                de < 30.0 || de < d * 0.8
            });
            if contested {
                continue;
            }
            let s = v / (d + 15.0);
            if s > bv {
                best = Some(it);
                bv = s;
            }
        }
        best.cloned()
    }

    fn slip_from(&mut self, tgt: &Threat, ths: &[Threat]) -> bool {
        self.slip_from_x(tgt, ths, false)
    }

    /// Whether we hold a laser and this mech does not.
    pub(crate) fn laser_edge(&self, tr: &Track) -> bool {
        self.has("laser") >= 0 && !tr.laser
    }

    /// Whether to decline an exchange at range and go for cover (left in slip_cell).
    fn slip_from_x(&mut self, tgt: &Threat, ths: &[Threat], any_edge: bool) -> bool {
        let (mut min_r, mut max_adv) = (self.p.avoid_range, self.p.avoid_adv);
        if self.p.cover_first && self.p.cover_min_range < min_r {
            min_r = self.p.cover_min_range;
        }
        if self.t - self.slip_on_t < 1.0 {
            min_r *= 0.85;
            max_adv *= 1.15;
        }
        if any_edge {
            max_adv = 1e9;
        }
        if self.p.avoid_adv <= 0.0 || tgt.d < min_r || tgt.adv >= max_adv {
            return false;
        }
        if tgt.tr.los && tgt.tr.hp > 0.0 && tgt.tr.hp < self.my_dps(tgt.d) * 1.5 + 60.0 {
            return false;
        }
        if self.t - self.slip_t > 0.5 {
            let mut reach = self.p.avoid_cover;
            if tgt.tr.laser && self.p.avoid_cover_laser > reach {
                reach = self.p.avoid_cover_laser;
            }
            let c = self.find_cover(ths, reach);
            if c >= 0 || self.p.edge_hold <= 0.0 || self.t - self.slip_on_t > 1.5 {
                self.slip_cell = c;
            }
            self.slip_t = self.t;
        }
        if self.slip_cell < 0 {
            return false;
        }
        self.slip_on_t = self.t;
        true
    }

    /// The nearest attacker to back away from when two or more are on us (None: do not kite).
    fn kite_from(&self, ths: &[Threat], n_engaged: i64) -> Option<usize> {
        if n_engaged < 2 || self.p.kite_min <= 0.0 {
            return None;
        }
        if self.p.kite_needs_laser && self.real_model() && self.has("laser") < 0 {
            return None;
        }
        if self.has("lightning") >= 0 && self.energy > 40.0 && self.has("laser") < 0 {
            return None;
        }
        let mut near: Option<usize> = None;
        for (i, th) in ths.iter().enumerate() {
            if th.engaged && !th.phantom && th.tr.listed && near.is_none_or(|n| th.d < ths[n].d) {
                near = Some(i);
            }
        }
        let n = near?;
        if ths[n].d > self.p.kite_max + 20.0 {
            return None;
        }
        Some(n)
    }

    /// Whether we hold anything besides the starting gun.
    pub(crate) fn armed(&self) -> bool {
        self.slots.iter().filter(|s| s.ok).count() >= 2
    }

    /// The threat to move away from: our target, else whoever is hurting us (seen or not).
    fn flee_from<'a>(&self, tgt: Option<&'a Threat>, ths: &'a [Threat]) -> Option<&'a Threat> {
        if tgt.is_some() {
            return tgt;
        }
        let mut best: Option<&Threat> = None;
        for th in ths {
            if th.engaged && best.is_none_or(|b| th.tr.hurt_me_t > b.tr.hurt_me_t) {
                best = Some(th);
            }
        }
        best
    }

    /// A cube (or pile of cubes) worth walking to in the middle of a fight.
    fn cube_run(&self, tgt: Option<&Threat>, st: &Stance, hp_frac: f64) -> Option<Item> {
        if self.o.self_.cubes >= st.want_cubes + 6 {
            return None;
        }
        let mut best: Option<&Item> = None;
        let mut bv = 0.0;
        for c in self.cubes.values() {
            if !c.ground || self.t - c.last_seen > 20.0 || !self.same_comp(c.cell as i64) {
                continue;
            }
            let d = dist2(self.pos, c.pos);
            let n = self.cubes.values().filter(|o| dist2(o.pos, c.pos) < 7.0).count();
            let mut max_d = if n >= 2 { 110.0 } else { 40.0 };
            if st.stealth {
                max_d *= 0.5;
            }
            if d > max_d || (d > 30.0 && hp_frac < 0.3) {
                continue;
            }
            if self.has("lightning") < 0 {
                if self.p.anti_lightning_w > 0.0 {
                    let light_r = self.wstat("lightning", "range", 45.0) + 8.0;
                    let danger_light = self.tracks.values().any(|tr| !tr.npc && tr.light && self.t - tr.last_seen < 4.0 && dist2(c.pos, tr.pos) < light_r);
                    if danger_light {
                        continue;
                    }
                } else if tgt.is_some_and(|t| t.tr.light && dist2(c.pos, t.tr.pos) < 50.0) {
                    continue;
                }
            }
            if !self.in_future_zone(c.pos[0], c.pos[1], d / self.walk + 1.0, 3.0) {
                continue;
            }
            let v = n as f64 / (d + 12.0);
            if v > bv {
                best = Some(c);
                bv = v;
            }
        }
        best.cloned()
    }

    /// Where to go exploring, when exploring is what this stance does.
    fn roam_target(&mut self, st: &Stance, hp_frac: f64) -> Option<(f64, f64)> {
        if hp_frac < 0.7 {
            return None;
        }
        if st.stealth {
            if self.armed() || self.t < 12.0 {
                return None;
            }
            return self.roam_goal();
        }
        if self.o.self_.cubes >= st.want_cubes {
            return None;
        }
        self.roam_goal()
    }

    /// The drone that is shooting at us (the closest one that hurt us in the last few seconds).
    fn pest(&self) -> Option<Track> {
        if self.t - self.last_npc_dt > 4.0 {
            return None;
        }
        let mut best: Option<&Track> = None;
        let mut bd = 1e18;
        for tr in self.tracks.values() {
            if !tr.npc || self.t - tr.hurt_me_t > 4.0 || self.t - tr.last_seen > 3.0 {
                continue;
            }
            let d = dist2(self.pos, tr.pos);
            if d < bd {
                best = Some(tr);
                bd = d;
            }
        }
        best.cloned()
    }

    /// Circles a point while keeping the distance inside [rmin, rmax].
    fn strafe(&mut self, rq: &mut SteerReq, at: [f64; 3], rmin: f64, rmax: f64, w: f64) {
        if self.t >= self.strafe_t {
            self.strafe_sign = -self.strafe_sign;
            if self.rng.float64() < 0.25 {
                self.strafe_sign = -self.strafe_sign;
            }
            self.strafe_t = self.t + self.p.strafe_min + self.rng.float64() * (self.p.strafe_max - self.p.strafe_min);
        }
        let (rx, ry) = norm2(at[0] - self.pos[0], at[1] - self.pos[1]);
        let d = dist2(self.pos, at);
        let mut radial = 0.0;
        if d > rmax {
            radial = clamp((d - rmax) / 15.0, 0.0, 1.0);
        } else if d < rmin {
            radial = -clamp((rmin - d) / 15.0, 0.0, 1.0);
        }
        let (tx, ty) = (-ry * self.strafe_sign, rx * self.strafe_sign);
        let wx = tx * (1.0 - 0.6 * radial.abs()) + rx * radial;
        let wy = ty * (1.0 - 0.6 * radial.abs()) + ry * radial;
        let (nx, ny) = norm2(wx, wy);
        rq.want_x = nx;
        rq.want_y = ny;
        rq.want_w = w;
    }

    /// Picks the range we want against a target and strafes there.
    fn fight_move(&mut self, rq: &mut SteerReq, th: &Threat) {
        let tr = &th.tr;
        let have_light = self.has("lightning") >= 0 && self.energy > 18.0;
        let (mut rmin, mut rmax) = (th.rmin, th.rmax);
        if rmax <= 0.0 {
            rmin = 30.0;
            rmax = 55.0;
        }
        if !tr.los {
            let (dx, dy, _, _) = self.nav_dir(tr.pos[0], tr.pos[1]);
            rq.want_x = dx;
            rq.want_y = dy;
            rq.want_w = 1.2;
            return;
        }
        self.strafe(rq, tr.pos, rmin, rmax, 1.6);
        if have_light && th.d > rmax + 10.0 && self.energy > 55.0 {
            rq.boost = true;
        }
        if self.p.light_kite_boost && !have_light && tr.light && self.energy > 28.0 {
            let lr = self.wstat("lightning", "range", 45.0);
            if th.d < lr + 12.0 {
                rq.boost = true;
            }
        }
    }

    /// (DropRun > 0, squad stance) a supply drop in reach worth breaking off for.
    fn drop_run(&self, st: &Stance, hp_frac: f64) -> Option<DropObs> {
        if self.p.drop_run <= 0.0 || st.name != "squad" {
            return None;
        }
        if hp_frac > 0.85 && self.o.self_.cubes >= 10 {
            return None;
        }
        let mut best: Option<&DropObs> = None;
        let mut bd = 1e18;
        for dr in &self.o.drops {
            if self.ban_until.get(&dr.id).is_some_and(|&u| self.t < u) {
                continue;
            }
            let d = dist2(self.pos, dr.pos);
            if d > self.p.drop_run {
                continue;
            }
            if dr.lands_in > d / self.walk * 1.25 + 4.0 {
                continue;
            }
            if !self.in_future_zone(dr.pos[0], dr.pos[1], d / self.walk + 6.0, 4.0) {
                continue;
            }
            if d < bd {
                best = Some(dr);
                bd = d;
            }
        }
        best.cloned()
    }

    /// (Seek > 0, squad stance) the nearest favourable mech to pursue when healthy.
    fn seek_target(&mut self, st: &Stance, hp_frac: f64) -> Option<(f64, f64, i64)> {
        let seek_hp = if self.p.seek_hp <= 0.0 { 0.65 } else { self.p.seek_hp };
        let seek_r = if self.p.seek_range <= 0.0 { 320.0 } else { self.p.seek_range };
        if self.p.seek <= 0.0 || st.name != "squad" || !st.hunt || hp_frac < seek_hp {
            return None;
        }
        let mut best: Option<Track> = None;
        let mut bd = 1e18;
        let ids: Vec<i64> = self.tracks.keys().copied().collect();
        for id in ids {
            let tr = self.tracks[&id].clone();
            if tr.npc || self.t - tr.last_seen > self.p.seek {
                continue;
            }
            if self.seek_ban.get(&tr.id).is_some_and(|&u| self.t < u) {
                continue;
            }
            let d = dist2(self.pos, tr.pos);
            if d > seek_r || !self.in_future_zone(tr.pos[0], tr.pos[1], 8.0, 6.0) {
                continue;
            }
            if d < 22.0 && !tr.listed {
                self.seek_ban.insert(tr.id, self.t + 20.0);
                continue;
            }
            if self.fight_plan(&tr, d).0 < st.engage_ratio {
                continue;
            }
            if d < bd || (d == bd && best.as_ref().is_some_and(|b| tr.id < b.id)) {
                bd = d;
                best = Some(tr);
            }
        }
        if let Some(b) = best {
            return Some((b.pos[0], b.pos[1], b.id));
        }
        self.roam_goal().map(|(x, y)| (x, y, -1))
    }
}
