
#pragma once

#include "gflags/gflags.h"
#include "src/execution/cognition/struct_cognition/conf/cognition_gflags.h"
//ego info
DECLARE_bool(using_chassis_velocity);

//vehicle_state_predictor
DECLARE_string(vehicle_model_type);
DECLARE_bool(using_new_vehicle_state_in_trajectory_stitcher);

//reference_line_provider
DECLARE_bool(enable_smooth_reference_line);
DECLARE_double(look_forward_extend_distance);
DECLARE_double(smoothed_reference_line_max_diff);
DECLARE_double(smoothed_reference_line_max_kappa_diff);

//trajectory_stitcher
DECLARE_double(trajectory_stitcher_forward_rel_time);
DECLARE_double(drive_system_lag_time);
DECLARE_bool(enable_two_stitching_trajectory);
DECLARE_double(start_velocity);
DECLARE_double(start_acceleration);
DECLARE_int32(generate_DRpath_type);
DECLARE_double(replan_lateral_distance_threshold);
DECLARE_double(replan_longitudinal_distance_threshold);
DECLARE_double(replan_longitudinal_speed_threshold);
DECLARE_double(stitching_trajectory_time_buffer);
DECLARE_bool(enable_replan_lateral_error);
DECLARE_bool(enable_replan_longitudinal_error);
DECLARE_bool(enable_const_length_stitching_trajectory);

//parameter for reference line and frame
DECLARE_bool(using_all_obstacle);
DECLARE_double(default_reference_line_width);
DECLARE_bool(enable_collision_detection);
DECLARE_int32(max_history_frame_num);
DECLARE_bool(enable_check_collision_on_stitching_trajectory);
DECLARE_bool(use_map_speed_limit);
DECLARE_bool(use_multi_ref_line);
DECLARE_bool(enable_change_lane_safety_check);
DECLARE_bool(enable_curvature_calculation);

//behavior_parser
DECLARE_bool(enable_realtime_avoid_obstacle_in_lane);
DECLARE_bool(enable_remote_drive_brake);
DECLARE_double(remote_stop_obstacle_length);
DECLARE_double(remote_stop_obstacle_width);
DECLARE_string(destination_obstacle_id);
DECLARE_double(virtual_stop_wall_length);
DECLARE_double(virtual_stop_wall_height);
DECLARE_bool(use_road_boundary_in_pull_over);
DECLARE_double(pull_over_extend_s);

//path_bound_decider && piecewise_jerk_path_optimizer
DECLARE_bool(enable_piecewise_path_debug);
DECLARE_bool(enable_smarter_lane_change);
DECLARE_double(obstacle_lat_buffer);
DECLARE_double(obstacle_lat_buffer_max);
DECLARE_double(obstacle_lon_start_buffer);
DECLARE_double(obstacle_lon_end_buffer);
DECLARE_bool(considerate_adc_bound);
DECLARE_bool(enable_force_end_state_constraint);
DECLARE_double(path_bounds_decider_resolution);
DECLARE_double(boundary_error_max);
DECLARE_bool(enable_solid_line_constraint_in_path_decider);
DECLARE_double(lane_change_time_buffer);
DECLARE_double(lane_change_length_minimum);
DECLARE_bool(enable_obstacle_weight);
DECLARE_bool(enable_dynamic_obstacle_weight);
DECLARE_bool(enable_bound_type_weight);
DECLARE_bool(enable_ideal_lane_change_length_weight);
DECLARE_double(lane_change_time_buffer_min);
DECLARE_bool(use_predict_info_in_piecewise_jerk_path);
DECLARE_double(dynamic_obstacle_lateral_ignore_buffer);
DECLARE_double(dynamic_obstacle_lateral_buffer);
DECLARE_int32(max_iter);
DECLARE_bool(enable_road_bound_constraint_in_piecewise_jerk_path);
DECLARE_bool(use_piecewise_jerk_path_optimizer_mainly);
DECLARE_bool(enable_skip_path_tasks);
DECLARE_bool(enable_apply_piecewise_jerk_plan_in_pull_over);
DECLARE_double(lateral_derivative_bound_default);
DECLARE_bool(use_const_lateral_jerk_bound);
DECLARE_double(lateral_jerk_bound);
DECLARE_bool(use_const_lateral_acc_bound);
DECLARE_double(lateral_acc_bound);
DECLARE_double(trajectory_space_resolution);
DECLARE_double(dl_bound);
DECLARE_double(kappa_bound);
DECLARE_double(dkappa_bound);
DECLARE_double(extreme_nudge_obstacle_s_max);
DECLARE_bool(enable_box_checking);
DECLARE_double(distance_to_freespace);
DECLARE_double(over_distance_to_solid_line);

//dp_path_plan
DECLARE_bool(using_cubic_spline_curve);
DECLARE_double(prediction_total_time);
DECLARE_int32(trajectory_point_num_for_debug);
DECLARE_bool(enable_road_boundary_constraint);
DECLARE_bool(enable_decision_path_bound_constraint);

//Qp_path_optimizer
DECLARE_double(look_forward_time_sec);

//path fallback
DECLARE_bool(enable_generate_one_pathpoint_fallback_path);
DECLARE_bool(use_last_publishable_trajectory_for_fallback_path);

// path_decision 
DECLARE_bool(enable_nudge_decision);
DECLARE_bool(enable_nudge_slowdown);
DECLARE_bool(enable_movable_nudge_slowdown);
DECLARE_double(static_decision_nudge_l_buffer);
DECLARE_double(moveable_obstacle_l_buffer);
DECLARE_double(static_decision_nudge_l_buffer_max);
DECLARE_double(static_decision_nudge_l_buffer_min);
DECLARE_double(lateral_ignore_buffer);
DECLARE_double(min_stop_distance_obstacle);
DECLARE_double(max_stop_distance_obstacle);
DECLARE_double(congestion_max_stop_distance_obstacle);
DECLARE_double(nudge_distance_obstacle);

//stmap
DECLARE_bool(enable_plan_based_on_stmap);
DECLARE_double(yield_weight_hp);
DECLARE_double(yield_weight_mp);
DECLARE_double(yield_weight_lp);
//st_boundary_mapper
DECLARE_bool(ignore_obstacle_acceleration_intention);
DECLARE_double(ignore_obstacle_acceleration_max);
DECLARE_bool(ignore_obstacle_deceleration_intention);
DECLARE_double(ignore_obstacle_deceleration_max);
DECLARE_double(max_trajectory_len);
DECLARE_bool(enable_expand_t_in_yield_boundary);  
// STBoundary
DECLARE_double(st_max_s);
DECLARE_double(st_max_t);
//speed_limit_decider
DECLARE_bool(use_average_kappa_to_calculate_speed_limit);
DECLARE_double(max_centric_acc_limit);
DECLARE_double(overtake_max_centric_acc_limit);
DECLARE_bool(enable_pedestrian_bicycle_slowdown);
//speed decider
DECLARE_bool(enable_follow_obj_status_estimation);
DECLARE_double(follow_min_distance);
DECLARE_double(follow_min_obs_lateral_distance);
DECLARE_double(yield_distance);
DECLARE_double(yield_distance_pedestrian_bycicle);
DECLARE_double(follow_time_buffer);
DECLARE_double(follow_min_time_sec);
DECLARE_double(max_stop_speed);
DECLARE_bool(enable_decision_replace_dp_speed_optimizer);
DECLARE_bool(enable_decision_ignore_obstacle);
//speedplan
DECLARE_bool(enable_u_turn_stop_yield);
DECLARE_bool(enable_yield_kernel);
DECLARE_double(overtake_max_acceleration);
DECLARE_bool(enable_multitime_speedplan);
DECLARE_double(change_lane_speed_relax_percentage);
DECLARE_bool(use_dynamic_weight);
DECLARE_bool(use_speed_limit_as_kernel);
// QpSt optimizer
DECLARE_bool(enable_follow_accel_constraint);
DECLARE_bool(enable_sqp_solver);
DECLARE_bool(use_osqp_optimizer_for_qp_st);
DECLARE_double(st_spline_solver_time_limit);
DECLARE_double(piecewise_jerk_path_osqp_solver_time_limit);
//speed fallback
DECLARE_double(default_front_clear_distance);
DECLARE_double(slowdown_profile_deceleration);
DECLARE_double(fallback_total_time);
DECLARE_double(fallback_time_unit);
DECLARE_double(polynomial_speed_fallback_velocity);
DECLARE_bool(enable_limit_decel_when_fallback);

// parameters for trajectory planning
DECLARE_double(trajectory_time_length);
DECLARE_double(trajectory_time_min_interval);
DECLARE_double(trajectory_time_max_interval);
DECLARE_double(trajectory_time_high_density_period);
DECLARE_double(extend_path_resolution);

//obstacle related
DECLARE_double(exceed_path_length_buffer);
DECLARE_bool(use_predict_info);
DECLARE_bool(using_cognition_sl_boundary);
DECLARE_bool(using_cognition_st_boundary);
DECLARE_bool(is_simulation_mode);

//others
DECLARE_double(parking_control_accuracy);
DECLARE_double(common_control_accuracy);
DECLARE_double(numerical_epsilon);
DECLARE_bool(enable_record_debug);
DECLARE_bool(enable_generate_trajectory_manual_drive_mode);
DECLARE_bool(enable_debug_motion);
DECLARE_bool(enable_debug_speedplan);
DECLARE_double(obstacle_extreme_nudge_speed);
DECLARE_bool(enable_pull_over_plan);
DECLARE_bool(enable_compensate_stmap);
DECLARE_bool(enable_lite_coupling_without_decision_meeting_ids);
DECLARE_double(increased_dynamic_weight);
DECLARE_double(overall_system_delay);
