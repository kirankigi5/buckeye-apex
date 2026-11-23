//! Named parameter tweaks for A/B runs in the local arena (roster names). Select one with the
//! BOT_VARIANT environment variable.

use super::Params;

fn v05(p: &mut Params) {
    p.brawl = false;
    p.hit_model = 0;
    p.plan_v2 = false;
    p.kite_needs_laser = false;
    p.laser_value = 3.0;
    p.avoid_adv = 0.0;
    p.use_hide_map = false;
    p.quiet_hold_s = 0.0;
    p.path_fix = false;
    p.credit_fix = false;
    p.loot_boost = 0.0;
    p.provoke_rule = false;
    p.plink_max_dps = 0.0;
    p.zone_slack = 0.0;
    p.drop_stick = false;
    p.roof_leave_quiet = 0.0;
    p.roof_stand_check = false;
    p.run_on_range = 0.0;
    p.cover_margin = 0.0;
    p.retreat_run_max = 0.0;
    p.credit_open_s = 0.0;
    old_lead(p);
}

fn v074(p: &mut Params) {
    p.retreat_run_max = 0.0;
    p.credit_open_s = 0.0;
    p.shy_secs = 0.0;
    p.slot_rule = false;
}

fn old_lead(p: &mut Params) {
    p.lead_damp = 0.75;
    p.lead_persist = false;
    p.lead_cum = false;
}

fn v061(p: &mut Params) {
    brawl2(p);
    p.loot_boost = 0.0;
    p.provoke_rule = false;
    p.plink_max_dps = 0.0;
    p.credit_fix = false;
    p.zone_slack = 0.0;
    p.brawl_want_cubes = 15;
    p.brawl_roofs = false;
    p.laser_preempt = false;
    p.laser_aim_range = 0.0;
    p.avoid_cover_laser = 0.0;
    p.roof_leave_quiet = 0.0;
    p.drop_stick = false;
    p.cover_first = false;
    p.hunt_laser_edge = false;
    p.loot_hidden_bias = 0.0;
    p.edge_hold = 0.0;
    p.no_mortar = false;
    p.run_on_range = 0.0;
    p.cover_margin = 0.0;
    p.retreat_run_max = 0.0;
    p.credit_open_s = 0.0;
    old_lead(p);
}

fn v07(p: &mut Params) {
    p.cover_first = false;
    p.hunt_laser_edge = false;
    p.loot_hidden_bias = 0.0;
    p.edge_hold = 0.0;
    p.no_mortar = false;
    p.run_on_range = 0.0;
    p.cover_margin = 0.0;
    p.retreat_run_max = 0.0;
    p.credit_open_s = 0.0;
}

fn brawl2(p: &mut Params) {
    p.brawl = true;
    p.hit_model = 1;
    p.plan_v2 = true;
    p.kite_needs_laser = true;
    p.laser_value = 6.0;
    p.avoid_adv = 1.25;
}

fn brawl(p: &mut Params) {
    p.practice_aware = false;
    p.hit_model = 1;
    p.plan_v2 = true;
    p.kite_needs_laser = true;
    p.laser_value = 6.0;
}

/// Applies the named variant; false when the name is unknown.
pub fn apply(name: &str, p: &mut Params) -> bool {
    match name {
        "v05" => v05(p),
        "v05-hm" => { v05(p); p.use_hide_map = true; p.quiet_hold_s = 3.0; }
        "fighter" => { v05(p); p.practice_aware = false; }
        "ghost" => p.force_ffa = true,
        "ghost-noloot" => { p.force_ffa = true; p.ffa_loot_radius = 0.0; }
        "nohm" => p.use_hide_map = false,
        "hm-near" => p.hide_dist_w = 0.30,
        "hm-far" => p.hide_dist_w = 0.08,
        "hm-hit" => { p.hide_seen_w = 0.2; p.hide_hit_w = 1.3; }
        "hm-seen" => { p.hide_seen_w = 1.2; p.hide_hit_w = 0.4; }
        "hm-noroof" | "noroof" => p.use_roofs = false,
        "sfr70" => p.stealth_fight_range = 70.0,
        "sfr100" => p.stealth_fight_range = 100.0,
        "sfr200" => p.stealth_fight_range = 200.0,
        "patient60" => p.patient_secs = 60.0,
        "patient120" => p.patient_secs = 120.0,
        "patient200" => { p.patient_secs = 200.0; p.patient_cubes = 15; }
        "nokite" => p.kite_min = 0.0,
        "kite80" => { p.kite_min = 80.0; p.kite_max = 130.0; }
        "kite140" => { p.kite_min = 140.0; p.kite_max = 200.0; }
        "rooffirst" => { p.roof_first_secs = 40.0; p.roof_first_dist = 90.0; }
        "rooffirst60" => { p.roof_first_secs = 60.0; p.roof_first_dist = 150.0; }
        "ttd6" => p.leave_ttd = 6.0,
        "ttd16" => p.leave_ttd = 16.0,
        "duel0" => { p.use_roofs = false; p.duel_disengage = 0.0; }
        "duel25" => { p.use_roofs = false; p.duel_disengage = 0.25; }
        "duel75" => { p.use_roofs = false; p.duel_disengage = 0.75; }
        "roofsquad" => p.roof_in_squad = true,
        "fighter-noroof" => { p.practice_aware = false; p.use_roofs = false; }
        "ghost-greedy" => { p.force_ffa = true; p.ffa_loot_radius = 260.0; p.ffa_want_cubes = 20; }
        "f-hit" => { p.practice_aware = false; p.hit_model = 1; }
        "f-plan" => { p.practice_aware = false; p.hit_model = 1; p.plan_v2 = true; }
        "f-nk" => { p.practice_aware = false; p.hit_model = 1; p.plan_v2 = true; p.kite_needs_laser = true; }
        "f-las" => brawl(p),
        "f-las4" => { brawl(p); p.laser_value = 4.0; }
        "f-las8" => { brawl(p); p.laser_value = 8.0; p.arm_swap_min = 0.5; }
        "f-ex" => { brawl(p); p.exchange_w = 0.6; }
        "f-ex1" => { brawl(p); p.exchange_w = 1.0; }
        "f-d35" => { brawl(p); p.duel_disengage = 0.35; }
        "f-d70" => { brawl(p); p.duel_disengage = 0.70; }
        "f-d90" => { brawl(p); p.duel_disengage = 0.90; }
        "f-e10" => { brawl(p); p.engage_ratio = 1.0; }
        "f-e16" => { brawl(p); p.engage_ratio = 1.6; }
        "f-nohunt" => { brawl(p); p.hunt = false; }
        "hit1" => { p.hit_model = 1; p.plan_v2 = true; }
        "brawl" => brawl2(p),
        "b-stand" => { brawl2(p); p.avoid_adv = 0.0; }
        "b-av10" => { brawl2(p); p.avoid_adv = 1.0; }
        "b-av16" => { brawl2(p); p.avoid_adv = 1.6; }
        "b-avr60" => { brawl2(p); p.avoid_range = 60.0; }
        "b-avr120" => { brawl2(p); p.avoid_range = 120.0; }
        "b-cov90" => { brawl2(p); p.avoid_cover = 90.0; }
        "b-las3" => { brawl2(p); p.laser_value = 3.0; }
        "b-las10" => { brawl2(p); p.laser_value = 10.0; p.arm_swap_min = 0.5; }
        "b-e10" => { brawl2(p); p.brawl_engage = 1.0; }
        "b-e18" => { brawl2(p); p.brawl_engage = 1.8; }
        "b-nohunt" => { brawl2(p); p.brawl_hunt = false; }
        "b-hl200" => { brawl2(p); p.hunt_range_laser = 200.0; }
        "b-ex" => { brawl2(p); p.exchange_w = 0.7; }
        "b-c10" => { brawl2(p); p.brawl_want_cubes = 10; }
        "b-c25" => { brawl2(p); p.brawl_want_cubes = 25; }
        "b-lr120" => { brawl2(p); p.brawl_loot_radius = 120.0; }
        "b-d35" => { brawl2(p); p.duel_disengage = 0.35; }
        "b-d75" => { brawl2(p); p.duel_disengage = 0.75; }
        "b-oldhit" => { brawl2(p); p.hit_model = 0; }
        "b-open45" => p.open_secs = 45.0,
        "b-open90" => p.open_secs = 90.0,
        "b-open999" => p.open_secs = 9999.0,
        "b-oldlead" => old_lead(p),
        "b-damp5" => { old_lead(p); p.lead_damp = 0.5; }
        "b-pers" => { old_lead(p); p.lead_damp = 0.5; p.lead_persist = true; }
        "b-cum" => { old_lead(p); p.lead_damp = 0.5; p.lead_cum = true; }
        "b-perscum" => { old_lead(p); p.lead_damp = 0.5; p.lead_persist = true; p.lead_cum = true; }
        "b-pc75" => p.lead_damp = 0.75,
        "b-pc100" => p.lead_damp = 1.0,
        "b-jit" => { p.strafe_min = 0.25; p.strafe_max = 0.55; }
        "b-jit2" => { p.strafe_min = 0.2; p.strafe_max = 0.42; }
        "b-jitj" => { p.strafe_min = 0.25; p.strafe_max = 0.55; p.jink = 0.6; }
        "v07" => v07(p),
        "v071" => { v074(p); p.edge_hold = 0.0; p.no_mortar = false; p.run_on_range = 0.0; p.cover_margin = 0.0; }
        "v072" => { v074(p); p.no_mortar = false; p.run_on_range = 0.0; p.cover_margin = 0.0; }
        "v073" => { v074(p); p.run_on_range = 0.0; p.cover_margin = 0.0; }
        "v074" => v074(p),
        "b-standrun" => p.retreat_run_max = 50.0,
        "s1" => p.salt = 1,
        "s2" => p.salt = 2,
        "s3" => p.salt = 3,
        "s4" => p.salt = 4,
        "s5" => p.salt = 5,
        "s6" => p.salt = 6,
        "b-shy" => { p.slot_rule = true; p.shy_secs = 25.0; }
        "b-slotonly" => p.slot_rule = true,
        "b-ffa0" => p.force_ffa = true,
        "b-shy40" => { p.slot_rule = true; p.shy_secs = 40.0; }
        "b-shyb" => { p.slot_rule = true; p.shy_secs = 25.0; p.shy_boost = 40.0; }
        "b-shyr90" => { p.slot_rule = true; p.shy_secs = 25.0; p.shy_loot_radius = 90.0; }
        "b-shyall" => { p.slot_rule = true; p.shy_secs = 25.0; p.shy_boost = 40.0; p.shy_loot_radius = 90.0; p.shy_exposure = 0.6; }
        "b-nocredit2" => p.credit_open_s = 0.0,
        "b-rr35" => p.retreat_run_max = 35.0,
        "b-rr70" => p.retreat_run_max = 70.0,
        "b-run0" => p.run_on_range = 0.0,
        "b-cm0" => p.cover_margin = 0.0,
        "b-cm14" => p.cover_margin = 14.0,
        "b-run120" => p.run_on_range = 120.0,
        "b-real" => { p.cover_first = true; p.hunt_laser_edge = true; }
        "b-realh" => { p.cover_first = true; p.hunt_laser_edge = true; p.hunt_range_laser = 200.0; }
        "b-realc" | "b-cf" => p.cover_first = true,
        "b-realx" => p.hunt_laser_edge = true,
        "b-realb5" => { p.cover_first = true; p.hunt_laser_edge = true; p.loot_hidden_bias = 0.5; }
        "b-realb8" => { p.cover_first = true; p.hunt_laser_edge = true; p.loot_hidden_bias = 0.8; }
        "b-cf60" => { p.cover_first = true; p.cover_min_range = 60.0; }
        "b-cf90c" => { p.cover_first = true; p.avoid_cover = 90.0; }
        "b-cf15" => { p.cover_first = true; p.avoid_adv = 1.5; }
        "b-cf20" => { p.cover_first = true; p.avoid_adv = 2.0; }
        "b-cf30" => { p.cover_first = true; p.avoid_adv = 3.0; }
        "b-cf99" => { p.cover_first = true; p.avoid_adv = 99.0; }
        "b-cf20h" => { p.cover_first = true; p.avoid_adv = 2.0; p.brawl_engage = 2.0; }
        "b-cf99h" => { p.cover_first = true; p.avoid_adv = 99.0; p.brawl_hunt = false; }
        "b-cfpeek" => { p.cover_first = true; p.peek = true; }
        "b-peek" => p.peek = true,
        "b-peek14" => { p.peek = true; p.peek_reach = 14.0; p.peek_cover = 40.0; }
        "b-peekh" => { p.peek = true; p.hunt_range_laser = 200.0; }
        "b-noslack" => p.zone_slack = 0.0,
        "b-oe2" => p.overland_early = 2.5,
        "b-oe4" => p.overland_early = 4.0,
        "b-rc3" => p.roof_center_w = 0.3,
        "b-rc6" => p.roof_center_w = 0.6,
        "b-rlq" => p.roof_leave_quiet = 25.0,
        "b-rc3lq" => { p.roof_center_w = 0.3; p.roof_leave_quiet = 25.0; }
        "b-noroof" => p.brawl_roofs = false,
        "b-nopre" => { p.laser_preempt = false; p.laser_aim_range = 0.0; p.avoid_cover_laser = 0.0; }
        "b-c15" => p.brawl_want_cubes = 15,
        "b-roof" => p.brawl_roofs = true,
        "b-roofpre" => { p.brawl_roofs = true; p.laser_preempt = true; p.laser_aim_range = 285.0; }
        "b-roof3" => { p.brawl_roofs = true; p.brawl_roof_cubes = 3; p.brawl_roof_after = 40.0; }
        "b-roof10" => { p.brawl_roofs = true; p.brawl_roof_cubes = 10; p.brawl_roof_after = 100.0; }
        "b-jink5" => p.jink = 0.5,
        "b-jink8" => p.jink = 0.8,
        "b-hm" => p.hold_hide_map = true,
        "b-las285" => p.laser_aim_range = 285.0,
        "b-hm285" => { p.hold_hide_map = true; p.laser_aim_range = 285.0; }
        "b-pre" => { p.laser_preempt = true; p.laser_aim_range = 285.0; }
        "b-pre90" => { p.laser_preempt = true; p.laser_aim_range = 285.0; p.avoid_cover_laser = 90.0; }
        "b-cov90l" => p.avoid_cover_laser = 90.0,
        "b-noprov2" => p.provoke_rule = false,
        "b-noplink2" => p.plink_max_dps = 0.0,
        "b-nosprint" => p.loot_boost = 0.0,
        "b-nocredit" => p.credit_fix = false,
        "v06" => { v061(p); p.path_fix = false; }
        "v061" => v061(p),
        "b-sprint" => { brawl2(p); p.loot_boost = 30.0; }
        "b-sprint10" => { brawl2(p); p.loot_boost = 30.0; p.laser_value = 10.0; p.arm_swap_min = 0.5; }
        "b-nomortar" => { brawl2(p); p.no_mortar = true; }
        "b-prov" => { brawl2(p); p.provoke_rule = true; }
        "b-plink" => { brawl2(p); p.plink_max_dps = 60.0; }
        "b-provplink" => { brawl2(p); p.provoke_rule = true; p.plink_max_dps = 60.0; }
        "b-pps" => { brawl2(p); p.provoke_rule = true; p.plink_max_dps = 60.0; p.loot_boost = 30.0; }
        "b-pp40" => { brawl2(p); p.provoke_rule = true; p.plink_max_dps = 40.0; }
        "b-pp90" => { brawl2(p); p.provoke_rule = true; p.plink_max_dps = 90.0; }
        _ => return false,
    }
    true
}
