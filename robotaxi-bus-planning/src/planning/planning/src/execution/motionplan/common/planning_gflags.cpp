#include "planning_gflags.h"
//ego info
DEFINE_bool(using_chassis_velocity, false , "determin whether using chassis_velocity");

//vehicle_state_predictor
DEFINE_string(vehicle_model_type, "REAR_CENTERED_KINEMATIC_BICYCLE_MODEL", "vehicle model type");
DEFINE_bool(using_new_vehicle_state_in_trajectory_stitcher,false,"using_new_vehicle_state_in_trajectory_stitcher");

//reference_line_provider
DEFINE_bool(enable_smooth_reference_line, false, "whether enbale smooth reference line.");
DEFINE_double(look_forward_extend_distance, 50, "The step size when extending reference line.");
DEFINE_double(smoothed_reference_line_max_diff, 5.0, "Maximum position difference between the smoothed and the raw "
              "reference lines.");
DEFINE_double(smoothed_reference_line_max_kappa_diff, 0.01, "Maximum kappa between the smoothed and the raw "
              "reference lines.");

//trajectory_stitcher
DEFINE_double(trajectory_stitcher_forward_rel_time, 0.4, "trajectory_stitcher_forward_rel_time.");
DEFINE_double(drive_system_lag_time, 0.1, "drive_system_lag_time used in trajectory_stitcher.");
DEFINE_bool(enable_two_stitching_trajectory,false,"whether enable_two_stitching_trajectory.");
DEFINE_double(start_velocity, 0.5, "stop-start init speed setting ,m/s");
DEFINE_double(start_acceleration, 1.0, "stop-start init acc setting ,m/ss");
DEFINE_int32(generate_DRpath_type, 1, "generate DR_path flag, 0: not generate, 1: generate in speed planner, 2: generate in path planner.");
DEFINE_double(replan_lateral_distance_threshold, 0.5, "The lateral distance threshold of replan");
DEFINE_double(replan_longitudinal_distance_threshold, 2.0, "The longitudinal distance threshold of replan");
DEFINE_double(replan_longitudinal_speed_threshold, 1.5, "The longitudinal speed threshold (default 5.4km/h) of replan");
DEFINE_double(stitching_trajectory_time_buffer, 0.7, "The time buffer of stitching_trajectory when has lateral behavior.");
DEFINE_bool(enable_replan_lateral_error, false, "enable lateral_error to calculate stitching trajectory");
DEFINE_bool(enable_replan_longitudinal_error, false,  "enable longitudinal_error to calculate stitching trajectory");
DEFINE_bool(enable_const_length_stitching_trajectory, true, "determin whether use const length pre-trajectory as stitching trajectory");

//parameter for reference line and frame
DEFINE_bool(using_all_obstacle,false,"whether use all obstacle provided.");
DEFINE_double(default_reference_line_width, 4.0, "Default reference line width");
DEFINE_bool(enable_collision_detection, false, "enable collision detection in planning");
DEFINE_int32(max_history_frame_num, 1, "The maximum history frame number");
DEFINE_bool(enable_check_collision_on_stitching_trajectory, false,
           "determin whether enable_check_collision_on_stitching_trajectory in frame init function. ");
DEFINE_bool(use_map_speed_limit, false,
           "determin whether use map_speed_limit in GetSpeedLimitFromS function of class ReferenceLine. ");
DEFINE_bool(use_multi_ref_line, false, "determin whether use multi_ref_line to plan.");
DEFINE_bool(enable_change_lane_safety_check, false, "determin whether enable change_lane_safety_check.");
DEFINE_bool(enable_curvature_calculation, false, "determin whether enable_curvature_calculation");

//behavior_parser
DEFINE_bool(enable_realtime_avoid_obstacle_in_lane, true, "whether enable_realtime_avoid_obstacle_in_lane");
DEFINE_bool(enable_remote_drive_brake,false, "whether enable_remote_drive_brake.");
DEFINE_double(remote_stop_obstacle_length, 0.001, "(unit: meter) remote_stop_obstacle_length.");
DEFINE_double(remote_stop_obstacle_width, 5, "(unit: meter) remote_stop_obstacle_width.");
DEFINE_string(destination_obstacle_id, "DEST", "obstacle id for converting destination to an obstacle");
DEFINE_double(virtual_stop_wall_length, 0.1, "virtual stop wall length (meters)");
DEFINE_double(virtual_stop_wall_height, 2.0, "virtual stop wall height (meters)");
DEFINE_bool(use_road_boundary_in_pull_over, false, "determin whether use road boundary in pullover pathplan .");
DEFINE_double(pull_over_extend_s, 0.5, "(unit: meter) pull_over_extend_s.");

//path_bound_decider && piecewise_jerk_path_optimizer
DEFINE_bool(enable_piecewise_path_debug, false, "whether enable print piecewise_path debug info.");
DEFINE_bool(enable_smarter_lane_change, false, "enable smarter lane change with longer preparation distance.");
DEFINE_double(obstacle_lat_buffer, 0.4, "obstacle lateral buffer (meters) for deciding path boundaries");
DEFINE_double(obstacle_lat_buffer_max, 0.8, "obstacle lateral buffer (meters) maximumfor deciding path boundaries");
DEFINE_double(obstacle_lon_start_buffer, 3.0, "obstacle longitudinal start buffer (meters) for deciding path boundaries");
DEFINE_double(obstacle_lon_end_buffer, 2.0, "obstacle longitudinal end buffer (meters) for deciding path boundaries"); 
DEFINE_bool(considerate_adc_bound, false, "whether considerate adc position as path bound. ");
DEFINE_bool(enable_force_end_state_constraint, true, "whether enable force end_state constraint when exceed certain length. ");
DEFINE_double(path_bounds_decider_resolution, 0.5, "path_bounds_decider_resolution. ");
DEFINE_double(boundary_error_max, 0.35, "road boundary error maximum value. ");
DEFINE_bool(enable_solid_line_constraint_in_path_decider, false, "enable_solid_line_constraint_in_path_decider.");
DEFINE_double(lane_change_time_buffer, 5.0, "lane_change_time_buffer.");
DEFINE_double(lane_change_length_minimum, 25.0, "lane_change_length_minimum.");
DEFINE_bool(enable_obstacle_weight,false,"whether enable obstacle_weight in PiecewiseJerkPathOptimizer.");
DEFINE_bool(enable_dynamic_obstacle_weight,false,"whether enable dynamic_obstacle_weight in PiecewiseJerkPathOptimizer.");
DEFINE_bool(enable_bound_type_weight,false,"bound type includes dotted_line , solid line, curb;"
                                            "whether enable bound_type_weight in PiecewiseJerkPathOptimizer.");
DEFINE_bool(enable_ideal_lane_change_length_weight,false, "whether enable ideal_lane_change_length_weight in PiecewiseJerkPathOptimizer.");
DEFINE_double(lane_change_time_buffer_min,2.5, "lane_change_time_buffer minimum value.");
DEFINE_bool(use_predict_info_in_piecewise_jerk_path, false, "whether use predict_info in_piecewise_jerk_path.");
DEFINE_double(dynamic_obstacle_lateral_ignore_buffer,1.5,"If an dynamic obstacle's lateral distance is further away than this "
              "distance, ignore it");
DEFINE_double(dynamic_obstacle_lateral_buffer,0.4,"Avoiding low-speed dynamic obstacles lateral distance ");
DEFINE_int32(max_iter, 4000, "max_iter in PiecewiseJerkPathOptimizer.");
DEFINE_bool(enable_road_bound_constraint_in_piecewise_jerk_path,false, "determin whether use road_bound as a constraint.");
DEFINE_bool(use_piecewise_jerk_path_optimizer_mainly, false, "whether use piecewise_jerk_path_optimizer to generate path mainly.");
DEFINE_bool(enable_skip_path_tasks, true, "skip all path tasks and use trimmed previous path");
DEFINE_bool(enable_apply_piecewise_jerk_plan_in_pull_over, true, "enable apply piecewise_jerk_plan algarithm in pull over. ");
DEFINE_double(lateral_derivative_bound_default, 1.5, "the default value for lateral derivative bound.");
DEFINE_bool(use_const_lateral_jerk_bound, true, "whether enable use const_lateral_jerk_bound.");
DEFINE_double(lateral_jerk_bound, 3.0, "Bound of lateral jerk; symmetric for left and right.");
DEFINE_bool(use_const_lateral_acc_bound, true, "whether enable use const_lateral_acc_bound.");
DEFINE_double(lateral_acc_bound, 0.2, "the default value for lateral acc constraint.");
DEFINE_double(trajectory_space_resolution, 1.0, "Trajectory space resolution in planning");
DEFINE_double(dl_bound, 0.10, "The bound for derivative l in s-l coordinate system.");
DEFINE_double(kappa_bound, 0.20, "The bound for trajectory curvature");
DEFINE_double(dkappa_bound, 0.02, "The bound for trajectory curvature change rate");
DEFINE_double(extreme_nudge_obstacle_s_max, 20.0, "obstacle_distance maximum when triggering extreme_nudge.");
DEFINE_bool(enable_box_checking, false, "whether execute box_checking and optimizition after pathplan.");
DEFINE_double(distance_to_freespace, 0.3, "used in path bound decider, the minimum distance to freespace.");
DEFINE_double(over_distance_to_solid_line, 0.2, "used in path bound decider, the maximum distance over to solid line.");

//dp_path_plan
DEFINE_bool(using_cubic_spline_curve, false, "wheter using cubic spline curve to fit dp path result.");
DEFINE_double(prediction_total_time, 5.0, "Total prediction time");
DEFINE_int32(trajectory_point_num_for_debug, 10, "number of output trajectory points for debugging");
DEFINE_bool(enable_road_boundary_constraint,true, "determin whether use road boundary as a constraint.");
DEFINE_bool(enable_decision_path_bound_constraint,true, "determin whether use decision_path_bound as a constraint.");

//Qp_path_optimizer
DEFINE_double(look_forward_time_sec, 8.0,"look forward time times adc speed to calculate this distance "
              "when creating reference line from routing");

//path fallback
DEFINE_bool(enable_generate_one_pathpoint_fallback_path,true,
            "enable generate stop_trajectory(just one pathpoint and v = 0 m/s) when_pathplan_failed.");
DEFINE_bool(use_last_publishable_trajectory_for_fallback_path, false, "wheter use last_publishable_trajectory for fallback_path.");

// path_decision 
DEFINE_bool(enable_nudge_decision, true, "enable nudge decision");
DEFINE_bool(enable_nudge_slowdown, true, "True to slow down when nudge obstacles.");
DEFINE_bool(enable_movable_nudge_slowdown, true, "True to slow down when movable nudge obstacles.");
DEFINE_double(static_decision_nudge_l_buffer, 0.3, "l buffer for nudge");
DEFINE_double(moveable_obstacle_l_buffer, 0.5, "l buffer for moveble osatcle");
DEFINE_double(static_decision_nudge_l_buffer_max, 0.4, "l buffer max for nudge");
DEFINE_double(static_decision_nudge_l_buffer_min, 0.2, "l buffer min for nudge");
DEFINE_double(lateral_ignore_buffer, 3.0, "If an obstacle's lateral distance is further away than this "
              "distance, ignore it");
DEFINE_double(min_stop_distance_obstacle, 5.0, "min stop distance from in-lane obstacle (meters)");
DEFINE_double(max_stop_distance_obstacle, 10.0, "max stop distance from in-lane obstacle (meters)");
DEFINE_double(congestion_max_stop_distance_obstacle, 3.0, "max stop distance from in-lane obstacle (meters)");
DEFINE_double(nudge_distance_obstacle, 0.3, "minimum distance to nudge a obstacle (meters)");

//stmap
DEFINE_bool(enable_plan_based_on_stmap, false, "enable plan based on stmap.");
DEFINE_double(yield_weight_hp, -1, "yield weight in high probability area.");
DEFINE_double(yield_weight_mp, -0.1, "yield weight in medium probability area.");
DEFINE_double(yield_weight_lp, -0.01, "yield weight in low probability area.");
//st_boundary_mapper
DEFINE_bool(ignore_obstacle_acceleration_intention, true, "whether ignore_obstacle_acceleration_intention.");
DEFINE_double(ignore_obstacle_acceleration_max, 0.7, "ignore_obstacle_acceleration_max.");
DEFINE_bool(ignore_obstacle_deceleration_intention, true, "whether ignore_obstacle_acceleration_intention.");
DEFINE_double(ignore_obstacle_deceleration_max, -1.0, "ignore_obstacle_acceleration_max.");
DEFINE_double(max_trajectory_len, 1000.0, "(unit: meter) max possible trajectory length.");
DEFINE_bool(enable_expand_t_in_yield_boundary, false, "determin whether enable expand t in yield_boundary .");
// STBoundary
DEFINE_double(st_max_s, 100, "the maximum s of st boundary");
DEFINE_double(st_max_t, 8, "the maximum t of st boundary");
//speed_limit_decider
DEFINE_bool(use_average_kappa_to_calculate_speed_limit, false, 
            "whether use average_kappa to calculate centri_acc_speed_limit in speed_limit_decidr.");
DEFINE_double(max_centric_acc_limit, 1.2, "max_centric_acc_limit.");
DEFINE_double(overtake_max_centric_acc_limit, 1.5, "max_centric_acc_limit when overtake.");
DEFINE_bool(enable_pedestrian_bicycle_slowdown,false, "determin whether slowdown when pedestrian/bicycle along the road.");
//speed decider
DEFINE_bool(enable_follow_obj_status_estimation, false, "whether enable follow_obj_status_estimation. ");
DEFINE_double(follow_min_distance, 3.0, "min follow distance for vehicles/bicycles/moving objects");
DEFINE_double(follow_min_obs_lateral_distance, 2.5, "obstacle min lateral distance to follow");
DEFINE_double(yield_distance, 3.0, "min yield distance for vehicles/moving objects "
              "other than pedestrians/bicycles");
DEFINE_double(yield_distance_pedestrian_bycicle, 5.0, "min yield distance for pedestrians/bicycles");
DEFINE_double(follow_time_buffer, 2.5, "time buffer in second to calculate the following distance.");
DEFINE_double(follow_min_time_sec, 0.1, "min follow time in st region before considering a valid follow");
DEFINE_double(max_stop_speed, 0.2, "max speed(m/s) to be considered as a stop");
DEFINE_bool(enable_decision_replace_dp_speed_optimizer,true, "determin whether use decision result replace dp_speed_optimizer.");
DEFINE_bool(enable_decision_ignore_obstacle,false,  
           "determin whether use decision ignore result in speed decider.");
//speedplan
DEFINE_bool(enable_u_turn_stop_yield, true, "enable stop_yield in u_turn .");
DEFINE_bool(enable_yield_kernel,false, "whether enable yield_kernel.");
DEFINE_double(overtake_max_acceleration, 1.5, "overtake max_acceleration.");
DEFINE_bool(enable_multitime_speedplan, false, "whether enable multitime_speedplan");
DEFINE_double(change_lane_speed_relax_percentage, 0.05, "The percentage of change lane speed relaxation.");
DEFINE_bool(use_dynamic_weight, false, "yield , follow , cruise weight can been adapted dynamically.");
DEFINE_bool(use_speed_limit_as_kernel, false, "whether use speed limit as kernel.");
// QpSt optimizer
DEFINE_bool(enable_follow_accel_constraint, true, "Enable follow acceleration constraint.");
DEFINE_bool(enable_sqp_solver, false, "True to enable SQP solver.");
DEFINE_bool(use_osqp_optimizer_for_qp_st, false,"Use OSQP optimizer for QpSt speed optimization.");
DEFINE_double(st_spline_solver_time_limit, 0.1, " 0.03 < maximum seconds allowed to solve osqp spline problem."); 
DEFINE_double(piecewise_jerk_path_osqp_solver_time_limit, 0.03, " < maximum seconds allowed to solve piecewise_jerk_path problem.");  

//speed fallback
DEFINE_double(default_front_clear_distance, 300.0, "default front clear distance value in case there is no obstacle around.");
DEFINE_double(slowdown_profile_deceleration, -1.0, "The deceleration to generate slowdown profile. unit: m/s^2.");
DEFINE_double(fallback_total_time, 3.0, "total fallback trajectory time");
DEFINE_double(fallback_time_unit, 0.02, "fallback trajectory unit time in seconds");
DEFINE_double(polynomial_speed_fallback_velocity, 3.5, "velocity to use polynomial speed fallback.");
DEFINE_bool(enable_limit_decel_when_fallback, false, "determin whether enable limit decel when objects caused fallback. ");

// parameters for trajectory planning
DEFINE_double(trajectory_time_length, 8.0, "Trajectory time length");
DEFINE_double(trajectory_time_min_interval, 0.02, "(seconds) Trajectory time interval when publish. The is the min value.");
DEFINE_double(trajectory_time_max_interval, 0.1,
             "(seconds) Trajectory time interval when publish. The is the max value.");
DEFINE_double(trajectory_time_high_density_period, 1.0,
             "(seconds) Keep high density in the next this amount of seconds. ");
DEFINE_double(extend_path_resolution, 0.1, "extend_path_resolution.");

//obstacle related
DEFINE_double(exceed_path_length_buffer, 3.0, "if obstacle sl_boundary start_s > path_length + exceed_path_length_buffer, ignore.");
DEFINE_bool(use_predict_info, true, "determin whether use obstacle predict information to construct obstacle  ");
DEFINE_bool(using_cognition_sl_boundary, true, "determin whether using cognition sl_boundary");
DEFINE_bool(using_cognition_st_boundary, true, "determin whether using cognition st_boundary");
DEFINE_bool(is_simulation_mode, true , "simulation_mode is or not.");

//others
DEFINE_double(parking_control_accuracy, 0.3, "control_accuracy under parking senario (m).");
DEFINE_double(common_control_accuracy, 1.0, "control_accuracy under common senario (m).");
DEFINE_double(numerical_epsilon, 1e-6, "Epsilon in planner.");
DEFINE_bool(enable_record_debug, false, "True to enable record debug info in chart format");
DEFINE_bool(enable_generate_trajectory_manual_drive_mode,false,  
           "determin whether generate trajectory in manual_drive_mode.");
DEFINE_bool(enable_debug_motion, true, "determin whether enable print of motion");
DEFINE_bool(enable_debug_speedplan,false,"determin whether enable print of speedplan");
DEFINE_double(obstacle_extreme_nudge_speed, 1.0, "when nudge distance is less than normal value, limit passby speed.");        
DEFINE_bool(enable_pull_over_plan, true, "whether execute pullover pathplan.");
DEFINE_bool(enable_compensate_stmap, true, "whether enable compensation of st_map.");
DEFINE_bool(enable_lite_coupling_without_decision_meeting_ids, true, "whether enable lite coupling without decision meeting ids.");
DEFINE_double(increased_dynamic_weight, 100.0, "increased weight value after lite-coupling boundary failed of piecewise_jerk_path.");
DEFINE_double(overall_system_delay, 0.5, "Output to execution, overall system delay compensation.");          
