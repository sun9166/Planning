#pragma once

#include "gflags/gflags.h"

DECLARE_bool(specific_pull_over_enable);
DECLARE_bool(pull_over_boundary_enable);
DECLARE_bool(congestion_enable);
DECLARE_bool(pedal_enable);
DECLARE_bool(gap_enable);
DECLARE_double(dis_to_switch_junction);
DECLARE_double(turnlight_dis);
DECLARE_double(pull_over_dis);
DECLARE_double(base_lc_dis);
DECLARE_double(expected_lc_dis);
DECLARE_int32(decision_frame);
DECLARE_int32(start_yield_frame);
DECLARE_double(stable_time);
DECLARE_double(stable_l);
DECLARE_double(stable_angle);
DECLARE_bool(mission_lc_stop_enable);
DECLARE_bool(junction_conflict_enable);
DECLARE_double(crosswalk_speed_limit);
DECLARE_double(min_yield_dis);
DECLARE_double(max_dec);
DECLARE_double(max_acc);
DECLARE_bool(decision_stmap);
DECLARE_double(yellow_t);
DECLARE_double(blink_t);
DECLARE_double(delta_t);
DECLARE_double(ready_ego_angle);
DECLARE_double(ready_steer_angle);
DECLARE_int32(turnlight_frame);
DECLARE_bool(enable_dynamic_sl_generation);
DECLARE_bool(enable_dynamic_sl_debug_plot);
DECLARE_int32(mission_lc_blocked_frame);
DECLARE_int32(cruise_blocked_frame);
DECLARE_bool(green_wave_enable);
DECLARE_double(green_wave_acc);
DECLARE_double(green_wave_dec);
DECLARE_double(green_wave_lowest_spd);
DECLARE_double(green_wave_highest_spd);

