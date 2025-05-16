#pragma once

#include "gflags/gflags.h"

DECLARE_bool(log_enable);
DECLARE_bool(task_enable);
DECLARE_bool(stmap_enable);
DECLARE_bool(vstatus_enable);
DECLARE_bool(use_openmp);
DECLARE_bool(waiting_in_junction);
DECLARE_double(lane_width);
DECLARE_double(collision_buff);
DECLARE_double(front_buff);
DECLARE_double(back_buff);
DECLARE_double(junction_range);
DECLARE_int32(history_size);
DECLARE_bool(avoid_junction_object); 
DECLARE_bool(jam_enable);
DECLARE_int32(debug_id);
// for object history
DECLARE_double(acc_level);
DECLARE_double(people_static_speed);
DECLARE_double(car_static_speed);
DECLARE_int32(static_times);
DECLARE_int32(junction_static_times);
// DECLARE_double(junction_search_range);
// DECLARE_double(junction_search_angle);
// for semantic information of reference line
DECLARE_int32(focus_num);
DECLARE_double(front_perception_range);
DECLARE_double(frenet_perception_range);
DECLARE_double(free_speed);
DECLARE_double(speed_threshold);
DECLARE_double(boundary_buff);
DECLARE_int32(road_block_times);
DECLARE_int32(queue_times);
DECLARE_int32(block_times);
DECLARE_int32(slow_times);
DECLARE_int32(stable_times);
DECLARE_int32(safety_times);
DECLARE_int32(meeting_times);
DECLARE_int32(offset_block_times);
DECLARE_int32(offset_recover_times);
DECLARE_double(offset_recover_buff);
DECLARE_bool(offset_disturb_enable);
DECLARE_double(recommended_planning_acc);
DECLARE_double(recommended_planning_dec);
DECLARE_double(max_planning_acc);
DECLARE_double(max_planning_dec);
DECLARE_double(risk_ttc);
DECLARE_double(side_risk_range);
DECLARE_double(min_lc_dis);
DECLARE_double(search_range);
DECLARE_double(boundary_width);
DECLARE_double(filter_buff);

// for st map
DECLARE_double(s_range);
DECLARE_double(t_range);
DECLARE_double(scale_s);
DECLARE_double(scale_t);
DECLARE_double(break_t);
DECLARE_double(p_o);
DECLARE_double(p_e);
DECLARE_double(p_threshold);
DECLARE_double(p_thd_delete);
DECLARE_double(p_thd_cost);
DECLARE_double(max_pd_velocity);
DECLARE_double(max_pd_acc);
DECLARE_double(upper_acc);
DECLARE_double(under_dec);
DECLARE_double(max_lon_speedtime);
DECLARE_double(min_lon_speedtime);

// for gap
DECLARE_bool(gaplog_enable);
DECLARE_double(pre_lc_t);
DECLARE_double(min_ttc);
DECLARE_double(min_thw);
DECLARE_double(self_acc);
DECLARE_double(self_dec);
DECLARE_double(lc_dis);
DECLARE_double(system_delay);
DECLARE_double(min_lc_speedtime);
DECLARE_int32(lane_id_match_num);
DECLARE_double(low_spd_thr);
//for v2x
DECLARE_bool(obu_events_enable);
DECLARE_double(dis2speed_limit);
DECLARE_double(dis2sharp_turn);
DECLARE_double(dis2slow_down);
DECLARE_double(dis2speed_limit_back);
DECLARE_double(dis2sharp_turn_back);
DECLARE_double(dis2slow_down_back);
DECLARE_double(sharp_turn_reduce_spd);
DECLARE_double(slow_down_reduce_spd);
DECLARE_double(dis2pedestrian_crosswalk);
DECLARE_double(run_red_light_dec);

DECLARE_bool(v2v_enable);
DECLARE_double(v2v_collision_warning_ttc);
DECLARE_double(v2v_collision_warning_thw);
DECLARE_double(v2v_emergency_brake_warning_dec);
DECLARE_bool(use_calmcar_tfl);