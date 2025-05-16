#include "cognition_gflags.h"

DEFINE_bool(log_enable, true, "log enable");
DEFINE_bool(task_enable, true, "task enable");
DEFINE_bool(stmap_enable, false, "stmap enable");
DEFINE_bool(vstatus_enable, false, "perception is_static status enable");
DEFINE_bool(use_openmp, true, "whether use openmp.");
DEFINE_bool(waiting_in_junction, true, "response trafficlight in junction.");
DEFINE_double(lane_width, 3.5, "value for judgement in IsInLine");
DEFINE_double(collision_buff, 0.4, "collision buffer");
DEFINE_double(front_buff, 3.0, "limit space front buffer");
DEFINE_double(back_buff, 2.0, "limit space back buffer");
DEFINE_double(junction_range, 10.0, "junction range");
DEFINE_int32(history_size, 5, "history information size");
DEFINE_bool(avoid_junction_object, false, "avoid static objects in junction");
DEFINE_bool(jam_enable, false, "shut down oa and speed lc under overpass");
DEFINE_int32(debug_id, 0, "debug object id");

// for object history 
DEFINE_double(acc_level, 0.4, "value for judging acc type");
DEFINE_double(people_static_speed, 0.5, "value to decide movement type of people");
DEFINE_double(car_static_speed, 1.0, "value to decide movement type of car");
DEFINE_int32(static_times, 1, "7 value for changing speed state");
DEFINE_int32(junction_static_times, 1, "30 value for changing speed state in junction");
// DEFINE_double(junction_search_range, 5.0, "max search range in junction");
// DEFINE_double(junction_search_angle, 60.0, "max search angle in junction");
// for semantic information of reference line
DEFINE_int32(focus_num, 2, " focus obstacle size in special lane");
DEFINE_double(front_perception_range, 60.0, " front perception range");
DEFINE_double(frenet_perception_range, 20.0, " static frenet perception l range");
DEFINE_double(free_speed, 15.0, "lane speed without obstacle");
DEFINE_double(speed_threshold, 5.0, "value to decide the lane is slow");
DEFINE_double(boundary_buff, 0.2, "safe distance to boundary");
DEFINE_int32(road_block_times, 20, "value for changing road block state");
DEFINE_int32(queue_times, 60, "value for changing queue state");
DEFINE_int32(block_times, 5, "value for changing block state");
DEFINE_int32(slow_times, 5, "value for changing speed state");
DEFINE_int32(stable_times, 20, "value for changing stable state");
DEFINE_int32(safety_times, 3, "value for changing safety state");
DEFINE_int32(meeting_times, 3, "value for changing meeting flag");
DEFINE_int32(offset_block_times, 5, "value for changing offset block state");
DEFINE_int32(offset_recover_times, 20, "value for recover offset block state");
DEFINE_double(offset_recover_buff, 0.6, "buff for recover offset free state");
DEFINE_bool(offset_disturb_enable, false, "enable to check dynamic objects");
DEFINE_double(recommended_planning_acc, 1.0, "recommended acceleration of ego");
DEFINE_double(recommended_planning_dec, -1.5, "recommended deceleration of ego");
DEFINE_double(max_planning_acc, 2.0, "max acceleration of ego");
DEFINE_double(max_planning_dec, -5.0, "max deceleration of ego");
DEFINE_double(risk_ttc, 8.0, " ttc to filt rear cars");
DEFINE_double(side_risk_range, 5.0, " risk range of neighbor line");
DEFINE_double(min_lc_dis, 11.0, "min distance to excute lane changing");
DEFINE_double(search_range, 15.0, "local path updates distance, search range");
DEFINE_double(boundary_width, 0.2, "boundary width");
DEFINE_double(filter_buff, 0.4, "driver filters object l value.");

// for st map 
DEFINE_double(s_range, 120.0, " st_map s range");
DEFINE_double(t_range, 8.0, " st_map t range");
DEFINE_double(scale_s, 0.5, "scale value of st_map s");
DEFINE_double(scale_t, 0.1, "scale value of st_map t");
DEFINE_double(break_t, 4.0, "break point between vt model and at model");
DEFINE_double(p_o, 0.75, "probability of exist and occupied");// ln3
DEFINE_double(p_e, 0.8, "probability of not exist or occupied");// ln4
DEFINE_double(p_threshold, 0.9, "threshold of probability");// 1 - 1/(1 + 9) 连续出现2帧
DEFINE_double(p_thd_delete, 0.2, "threshold of deleting obj");
DEFINE_double(p_thd_cost, 0.6, "threshold of st point cost");
DEFINE_double(max_pd_velocity, 16.7, "acceleration upper buff for lon discretization velocity.m/s");
DEFINE_double(max_pd_acc, 3.0, "max allowable acceleration/deceleration of obstacle prediction");// 百公里加速10s
DEFINE_double(upper_acc, 1.0, "acceleration upper buff for lon discretization acceleration");
DEFINE_double(under_dec, 1.0, "deceleration under buff for lon discretization acceleration");
DEFINE_double(max_lon_speedtime, 1.4, "max speedtime for lon discretization of obstacles");
DEFINE_double(min_lon_speedtime, 0.6, "min speedtime for lon discretization of obstacles");

// for gap
DEFINE_bool(gaplog_enable, true, "gap log enable");
DEFINE_double(pre_lc_t, 5.0, "max prepare lc time");
DEFINE_double(min_ttc, 4.0, "min ttc to front car");
DEFINE_double(min_thw, 1.2, "min thw to front car");
DEFINE_double(self_acc, 1.5, "acceleration upper buff for selfcar sim gap");
DEFINE_double(self_dec, 1.5, "deceleration under buff for selfcar sim gap");
DEFINE_double(lc_dis, 15.0, "min lon lc dis");
DEFINE_double(system_delay, 0.6, "min delay time of acu");
DEFINE_double(min_lc_speedtime, 0.7, "speed time of back obstacle speed");
DEFINE_int32(lane_id_match_num, 2, "lane_id_match_num");
DEFINE_double(low_spd_thr, 5.0 , "speed threshold of perception blind lidar Cells To Objects, km/h");
//for v2x
DEFINE_bool(obu_events_enable,true, "obu events response enable");
DEFINE_double(dis2speed_limit, 50.0, "vehicle distance near to speed limit point");
DEFINE_double(dis2sharp_turn, 50.0, "vehicle distance near to sharp turn point");
DEFINE_double(dis2slow_down, 50.0, "vehicle distance near to slow down point");
DEFINE_double(dis2speed_limit_back, 50.0, "vehicle distance far away speed limit point");
DEFINE_double(dis2sharp_turn_back, 50.0, "vehicle distance far away sharp turn point");
DEFINE_double(dis2slow_down_back, 50.0, "vehicle distance far away slow down point");
DEFINE_double(sharp_turn_reduce_spd, 5.0, "vehicle reduce speed");
DEFINE_double(slow_down_reduce_spd, 5.0, "vehicle reduce speed");
DEFINE_double(dis2pedestrian_crosswalk, 5.0, "vehicle stop distance to pedestrian");
DEFINE_double(run_red_light_dec,-1.0, "vehicle dec when through traffic lights");

DEFINE_bool(v2v_enable,true, "v2v vehicle enbale");
DEFINE_double(v2v_collision_warning_ttc,10, "v2v_collision_warning_ttc");
DEFINE_double(v2v_collision_warning_thw,5, "v2v_collision_warning_thw");
DEFINE_double(v2v_emergency_brake_warning_dec,-3.0, "v2v_emergency_brake_warning_dec");

DEFINE_bool(use_calmcar_tfl, true, "use calmcar perception traffic light enable");