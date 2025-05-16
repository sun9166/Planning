/**
 * @file piecewise_jerk_path_optimizer.cpp
 **/
#include "datapool/include/data_pool.h"
#include "piecewise_jerk_path_optimizer.h"
#include <string>
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/common/speed/speed_info.h"
#include "src/algorithm/curve/piecewise_jerk_trajectory1d.h"
#include "piecewise_jerk_path_problem.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/scenario_context.h"
#include "src/execution/motionplan/common/new_path_manager.h"
#include "common/math/angle.h"


namespace acu {
namespace planning {

using acu::common::ErrorCode;
using acu::common::Status;

PiecewiseJerkPathOptimizer::PiecewiseJerkPathOptimizer()
    : PathOptimizer("PiecewiseJerkPathOptimizer") {
}

bool PiecewiseJerkPathOptimizer::Init(const PlanningConfig &config) {
  CHECK(config.planner_config().has_piecewise_jerk_path_config());
  config_ = config.planner_config().piecewise_jerk_path_config();
  is_init_ = true;
  return true;
}

common::Status PiecewiseJerkPathOptimizer::Process(
    const SpeedInfo& speed_data, const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point, 
    PathInfo* const path_data) {
  auto DP_ptr = acu::planning::DataPool::Instance()->GetMainDataPtr();
  // skip piecewise_jerk_path_optimizer if reused path
  is_parking_ = reference_line.GetMapPath().is_parking_path();
  bool path_reusable =
       reference_line_info_->LaterAction().first == eActionEnum::DEFAULT_VALUE? true:false;
  DP_ptr->debug_planning_msg.motionplan_debug.set_dis_to_virtual_wall(EgoInfo::instance()->dis_to_virtual_wall());
  DP_ptr->debug_planning_msg.motionplan_debug.set_cog_linear_velocity(EgoInfo::instance()->vehicle_state().linear_velocity);
  if(EgoInfo::instance()->dis_to_virtual_wall()<10&&EgoInfo::instance()->vehicle_state().linear_velocity<0.25){
    path_reusable=true;
    DP_ptr->debug_planning_msg.motionplan_debug.set_real_time_ob_avoid_state(9);
  }
 
  bool is_start_from_road_side = false;  
  is_set_meeting_x_ref_ = false;//还原
  auto init_sl_point = reference_line_info_->init_sl_point();
  AINFO_IF(FLAGS_enable_debug_motion)<<"plan start from s = " <<init_sl_point.s()
                      <<", l = "<<init_sl_point.l();
  if (FLAGS_enable_skip_path_tasks 
      && path_reusable && EgoInfo::instance()->vehicle_state().driving_mode == 0) {
    if ((fabs(init_sl_point.l()) < FLAGS_replan_lateral_distance_threshold && NewPathManager::instance()->stitch_state())
      || reference_line_info_->IsReverse()
      || NewPathManager::instance()->is_pull_over_path()) {
      if (FLAGS_enable_two_stitching_trajectory
        && frame_->StitchingTrajectoryForPath().size() > frame_->last_stitching_trajectory().size()) {
        auto start_s  = reference_line_info_->speedplan_init_sl().s();
        GenerateReferencePathData(start_s, path_data);
      } else {
        GenerateReferencePathData(init_sl_point.s(), path_data);
      }
      
      DP_ptr->debug_planning_msg.motionplan_debug.set_init_point_s(init_sl_point.s());
      DP_ptr->debug_planning_msg.motionplan_debug.set_init_point_l(init_sl_point.l());
      
      return Status::OK();
    } else {
      if (EgoInfo::instance()->vehicle_state().linear_velocity < 0.2) {
        is_start_from_road_side = true;
      }
      AINFO_IF(FLAGS_enable_debug_motion)<< "car is far away from reference line, need a new path. ";
    }
  }
  AINFO_IF(EgoInfo::instance()->vehicle_state().driving_mode == 2)<<"standby realtime pathplan.";
  common::TrajectoryPoint planning_start_point = init_point;
  if (FLAGS_enable_two_stitching_trajectory) {
    planning_start_point = reference_line_info_->pathplan_init_point();
  }
  auto init_frenet_state =
      reference_line.ToFrenetFrame(planning_start_point);
  AINFO_IF(FLAGS_enable_debug_motion)<<"init s = ("<<init_frenet_state.first[0]
         <<", "<<init_frenet_state.first[1]<<", "<<init_frenet_state.first[2]
         <<", l = ("<<init_frenet_state.second[0]
         <<", "<<init_frenet_state.second[1]<<", "<<init_frenet_state.second[2];
  common::SLPoint start_point_sl;
  reference_line.XYToSL({planning_start_point.path_point().x(), planning_start_point.path_point().y()}, &start_point_sl);
  if(std::fabs(start_point_sl.l()-init_frenet_state.second[0])>0.5){
    init_frenet_state.second[0]=start_point_sl.l();
    AINFO_IF(FLAGS_enable_debug_motion)<<"cal_start_l_error, using function XLToSL to cal start_l";
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<"1.start_point_sl.s: "<<start_point_sl.s()<<", start_point_sl.l: "<<start_point_sl.l();
  if (frame_->StartPointType() == 0 
      || EgoInfo::instance()->vehicle_state().driving_mode > 0) {
    std::array<double, 3> l_condition;
    l_condition.at(0) = init_frenet_state.second[0];
    l_condition.at(1) = 0.0;
    l_condition.at(2) = 0.0;
    init_frenet_state = std::make_pair(init_frenet_state.first,l_condition);
  }
  DP_ptr->debug_planning_msg.motionplan_debug.set_init_point_s(init_frenet_state.first[0]);
  DP_ptr->debug_planning_msg.motionplan_debug.set_init_point_l(init_frenet_state.second[0]);
  DP_ptr->debug_planning_msg.motionplan_debug.set_init_point_dl(init_frenet_state.second[1]);
  DP_ptr->debug_planning_msg.motionplan_debug.set_init_point_ddl(init_frenet_state.second[2]);       
  // Choose lane_change_path_config for lane-change cases
  // Otherwise, choose default_path_config for normal path planning
  auto piecewise_jerk_path_config =
      reference_line_info_->IsChangeLanePath()
          ? config_.lane_change_path_config()
          : config_.default_path_config();
  if (is_start_from_road_side) {
    piecewise_jerk_path_config = config_.start_from_road_side_config();
  }        

  std::array<double, 9> w = {
      piecewise_jerk_path_config.l_weight(),
      piecewise_jerk_path_config.dl_weight() *
          std::fmax(init_frenet_state.first[1] * init_frenet_state.first[1],
                    5.0),
      piecewise_jerk_path_config.ddl_weight(),
      piecewise_jerk_path_config.dddl_weight(), 
      piecewise_jerk_path_config.obstacle_weight(),
      piecewise_jerk_path_config.dynamic_obstacle_weight(),
      piecewise_jerk_path_config.dotted_line_weight(),
      piecewise_jerk_path_config.solid_line_weight(),
      piecewise_jerk_path_config.curb_weight()};
  
  if (reference_line_info_->IsChangeLanePath()) {
    w[5]=0.0;//when lane change path plan ,disable dynamic obstacle offset.
  }
  
  const auto& path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();//获取备选的path_boundaries,该结果由path_bound_decider完成
  AINFO_IF(FLAGS_enable_debug_motion) << "There are " << path_boundaries.size() << " path boundaries.";

  std::vector<PathInfo> candidate_path_data;
  bool res_opt = false;
  for (const auto& path_boundary : path_boundaries) {
    // if the path_boundary is normal, it is possible to have less than 2 points
    // skip path boundary of this kind
    AINFO_IF(FLAGS_enable_debug_motion)<<"label: "<<path_boundary.label();
    if (path_boundary.label().find("regular") != std::string::npos &&
        path_boundary.boundary().size() < 2) {
      continue;
    }

    if (path_boundary.label().find("fallback") != std::string::npos && 
      path_boundaries.size() >= 2) { // 先不规划fallback路径 @pqg
      AINFO_IF(FLAGS_enable_debug_motion)<<"fallback boundary , continue ......";
      continue;
    }

    std::array<double, 3> end_state = {0.0, 0.0, 0.0};

    //有has_litecoupling标签说明当前边界是在处理过lite-coupling边界失败后的普通边界
    //此状态下：1.增大动态障碍物权重，2.将ref_x设置为决策的平移结果
    if (path_boundary.label().find("has_litecoupling") != std::string::npos || 
        path_boundary.label().find("lite_coupling") != std::string::npos) {
      auto behavior_expand_l = BehaviorParser::instance()->decision_expand_l();
      AERROR<<"behavior_expand_l: "<<behavior_expand_l;
      if (behavior_expand_l > 0.0 && behavior_expand_l < 10.0) {
        is_set_meeting_x_ref_ = true;
        // end_state[0] = (-1) * behavior_expand_l;//只考虑向右侧
      }
      //减小x趋近于0的权重
      w[0] = 0.1;
      //增大远离动态障碍物的权重
      w[5] = FLAGS_increased_dynamic_weight;
      AINFO_IF(true)<<"has_lite_coupling_path_label boundary, increase weight: "<<w[0]<<", "<<w[5];
    }

    int max_iter = FLAGS_max_iter;
    // lower max_iter for regular/self/
    if (path_boundary.label().find("self") != std::string::npos) {
      max_iter = 1000;//4000;
    }

    CHECK_GT(path_boundary.boundary().size(), 1);

    std::vector<double> opt_l;
    std::vector<double> opt_dl;
    std::vector<double> opt_ddl;


    if (BehaviorParser::instance()->pull_over_modified_l()<50 &&
        path_boundary.label().find("pullover") != std::string::npos) {
      //pullover下也修改了end_state，此时还原is_set_meeting_x_ref_标志，避免影响OptimizePath中的权重
      is_set_meeting_x_ref_ = false;
      //end_state[0] = BehaviorParser::instance()->pull_over_modified_l();
      end_state[0] = BehaviorParser::instance()->pull_over_sl().front().l();
    }

    const auto& veh_param =
        EgoInfo::instance()->vehicle_param();
    const double front_wheel_angle_max = 
         (fabs(veh_param.right_max_steerangle) / fabs(veh_param.eps_transmission_ratio));//degree not radian    
    CHECK_LT(front_wheel_angle_max, 90);
    double lat_acc_bound = FLAGS_use_const_lateral_acc_bound ? FLAGS_lateral_acc_bound : 
              std::tan(front_wheel_angle_max /180 * M_PI) / veh_param.length_wheelbase;
    // AINFO_IF(FLAGS_enable_debug_motion)<<"1) lat_acc_bound = "<<lat_acc_bound;  
    std::vector<std::pair<double, double>> ddl_bounds;

    std::vector<std::pair<double, double>> dl_bounds;
    double shrink_dis = 0.10;
    double curr_road_left_width = 2;
    double curr_road_right_width = 2;
    double half_wheel = veh_param.half_wheel;
    bool is_pull_over_in_current_lane_ = false;
    if (reference_line_info_->LaterAction().first == eActionEnum::PULL_OVER){
      double shrink_dis = 0.0;
      if (!BehaviorParser::instance()->has_pull_over_request()){
          is_pull_over_in_current_lane_ = true;
      }
    }

    for (size_t i = 0; i < path_boundary.boundary().size(); ++i) {
      double s = static_cast<double>(i) * path_boundary.delta_s() +
                 path_boundary.start_s();

      AINFO_IF(FLAGS_enable_debug_motion) <<"pathbdy s= "<< s 
                                          <<" ,r= " << path_boundary.boundary()[i].first
                                          <<" ,l= " << path_boundary.boundary()[i].second;

      double kappa = reference_line.GetNearestReferencePoint(s).kappa();
      ddl_bounds.emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
            if(!reference_line_info_->reference_line().GetMapPath().GetRoadBoundary(std::fmax(0.0,s), &curr_road_left_width,
                                       &curr_road_right_width)){
      }
      // 左边界存在借道场景，左边界以决策边界约束范围
      double left_dl = std::max(curr_road_left_width*2.0 - shrink_dis - half_wheel, path_boundary.boundary()[i].second);
      double right_dl = -1*(curr_road_right_width - shrink_dis - half_wheel);
      if(is_pull_over_in_current_lane_){
        right_dl = std::max(right_dl,path_boundary.boundary()[i].first);
      }
      // for (auto& dis_to_junction : dis_to_junctions) {
      //   if (s >=dis_to_junction.first -
      //               EgoInfo::instance()->vehicle_front_edge_to_center() * 0.5 &&
      //       s <= dis_to_junction.second) {
      //     left_dl += 0.30;
      //     right_dl -=
      //         0.30;  // 制图转弯绘图不会考虑车头，路口拓宽车头约束，减少规划失败。
      //   }
      // }
      if(left_dl < right_dl){
            string error_str =" dl bound error , vehicle can't pass";
            AERROR<<error_str;
            return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR,
                  error_str);
      }
      AINFO_IF(FLAGS_enable_debug_motion) <<"right_dl= " << right_dl <<" , left_dl= "<< left_dl;
      dl_bounds.emplace_back(right_dl,left_dl);

    }

    res_opt =
        OptimizePath(init_frenet_state.second, end_state,
                     path_boundary.delta_s(), path_boundary.boundary(),path_boundary.dynamic_obstacle_boundary(),
                     dl_bounds, ddl_bounds, path_boundary.boundary_type(),w, &opt_l, &opt_dl, &opt_ddl, max_iter);


    if (res_opt) {
      if (reference_line_info_->LaterAction().first == eActionEnum::CHANGE_OFFSET) {
        if (reference_line_info_->LaterAction().second == eDirectionEnum::DEFAULT_VALUE) {
          ScenarioContext::instance()->UpdateLatScenario(ScenarioContext::eLatScenarioEnum::DEFAULT_VALUE);  
        } else {
          ScenarioContext::instance()->UpdateLatScenario(ScenarioContext::eLatScenarioEnum::NUDGE_OFFSET);
        }
      } else if (reference_line_info_->LaterAction().first == eActionEnum::PULL_OVER) {
        ScenarioContext::instance()->UpdateLatScenario(ScenarioContext::eLatScenarioEnum::PULL_OVER);
      } else if (reference_line_info_->LaterAction().first == eActionEnum::OBSTACLE_AVOID) {
        ScenarioContext::instance()->UpdateLatScenario(ScenarioContext::eLatScenarioEnum::OBSTACLE_AVOID);
      } else if (reference_line_info_->LaterAction().first == eActionEnum::LANE_CHANGE) {
        ScenarioContext::instance()->UpdateLatScenario(ScenarioContext::eLatScenarioEnum::LANE_CHANGE);
      }

      for (size_t i = 0; i < path_boundary.boundary().size(); i += 4) {
        AINFO_IF(FLAGS_enable_debug_motion) << "for s[" << static_cast<double>(i) * path_boundary.delta_s()
               << "], l = " << opt_l[i] << ", dl = " << opt_dl[i]<< ", ddl = " << opt_ddl[i];
      }
      
      double property_s = -1;
      auto frenet_frame_path =
          ToPiecewiseJerkPath(opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
                              path_boundary.start_s(),property_s);
      AINFO_IF(FLAGS_enable_debug_motion)<<"property_s = "<<property_s; 
      if (FLAGS_enable_two_stitching_trajectory
        && frame_->StitchingTrajectoryForPath().size() > frame_->last_stitching_trajectory().size()) {
        AddPathPlanStithingTrajectory(frenet_frame_path); 
      } 
       
      PathInfo path_candidate = *path_data;
      path_candidate.SetReferenceLine(&reference_line);
      path_data->SetReferenceLine(&reference_line);
      path_candidate.SetFrenetPath(std::move(frenet_frame_path));
      AINFO_IF(FLAGS_enable_debug_motion)<<"1) size = "<<frenet_frame_path.size();
      path_data->SetFrenetPath(std::move(frenet_frame_path));
      AINFO_IF(FLAGS_enable_debug_motion)<<"2) size = "<<frenet_frame_path.size();

      if (FLAGS_enable_two_stitching_trajectory 
            && frame_->StitchingTrajectoryForPath().size() > frame_->last_stitching_trajectory().size()) {
        size_t index = std::max((size_t)0,frame_->PathStitchingStartIndex() - 1);//-1是为了包括黏合线的起点。
        std::vector<common::TrajectoryPoint> update_path(frame_->StitchingTrajectoryForPath().begin() + index, 
                                    frame_->StitchingTrajectoryForPath().end());
        path_data->UpdateDiscretizedPath(update_path);
      }

      path_candidate.set_path_label(path_boundary.label());
      path_data->set_path_label(path_boundary.label());
      path_candidate.SetPropertyLength(property_s);
      path_data->SetPropertyLength(property_s);
      path_candidate.SetIsNew(true);
      path_data->SetIsNew(true);
      candidate_path_data.push_back(std::move(path_candidate));
      break;
    }
  }
  if (candidate_path_data.empty()) {
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR,
                  "Path Optimizer failed to generate path");
  }
  reference_line_info_->SetCandidatePathData(std::move(candidate_path_data));
  return Status::OK();
}

void PiecewiseJerkPathOptimizer::AddPathPlanStithingTrajectory(FrenetFramePath& frenet_frame_path) {
  FrenetFramePath stitching_path;  
  size_t index = frame_->PathStitchingStartIndex();
  if (index - 1 < 0) return ;
  for (size_t i = index - 1; i + 1 < frame_->StitchingTrajectoryForPath().size();++i) {
    common::FrenetFramePoint frenet_frame_point;
    auto frenet_point = reference_line_info_->reference_line().ToFrenetFrame(frame_->StitchingTrajectoryForPath().at(i));
    frenet_frame_point.set_s(frenet_point.first[0]);
    frenet_frame_point.set_l(frenet_point.second[0]);
    frenet_frame_point.set_dl(frenet_point.second[1]);
    frenet_frame_point.set_ddl(frenet_point.second[2]);
    stitching_path.push_back(frenet_frame_point);
  }
  frenet_frame_path.insert(frenet_frame_path.begin(),stitching_path.begin(),stitching_path.end());
}

bool PiecewiseJerkPathOptimizer::OptimizePath(
    const std::array<double, 3>& init_state,
    const std::array<double, 3>& end_state, const double delta_s,
    const std::vector<std::pair<double, double>>& lat_boundaries,
    const std::vector<std::pair<double, double>>& dynamic_obstacle_boundaries,
    const std::vector<std::pair<double, double>>& dl_bounds,
    const std::vector<std::pair<double, double>>& ddl_bounds,
    const std::vector<std::pair<int, int>>& boundary_types,
    const std::array<double, 9>& w, std::vector<double>* x,
    std::vector<double>* dx, std::vector<double>* ddx, const int max_iter) {
  PiecewiseJerkPathProblem piecewise_jerk_problem(lat_boundaries.size(),
                                                  delta_s, init_state);
  piecewise_jerk_problem.set_x_bound_types(boundary_types);
  piecewise_jerk_problem.set_dynamic_obstacle_bounds(dynamic_obstacle_boundaries);
  if (BehaviorParser::instance()->lanechange_prepare_direction() > 0) {//处于准备换道阶段，不要求终点回到目标参考线上
    piecewise_jerk_problem.set_end_state_ref({0.0, 0.0, 0.0}, end_state);
  } else {
    piecewise_jerk_problem.set_end_state_ref({1000.0, 0.0, 0.0}, end_state);//1000,0,0分别为终点l.dl.ddl的惩罚权重
  }
  
  if (end_state[0] != 0) {
    std::vector<double> x_ref(lat_boundaries.size(), end_state[0]);
    const double weight_x_ref = (is_set_meeting_x_ref_) ? 10.0 : 10.0;    
    piecewise_jerk_problem.set_x_ref(weight_x_ref, x_ref);
    AINFO_IF(FLAGS_enable_debug_motion && is_set_meeting_x_ref_)<<"set end_state[0]: "<<end_state[0]<<", w: "<<weight_x_ref;
  }
  if (reference_line_info_->IsChangeLanePath() && FLAGS_enable_ideal_lane_change_length_weight) {
    double lane_change_length = GetPreferredLaneChangeDistance();
    lane_change_length = std::fmin(lane_change_length + reference_line_info_->init_sl_point().s(), 
                                   reference_line_info_->reference_line().Length());
    if ((reference_line_info_->SDistanceToNearestStaticObstacle() - 5.0 <= lane_change_length
      || reference_line_info_->SDistanceToNearestStaticObstacle() > 100) 
      && !IsIdeaLaneChangeDistanceInSolidLine(lane_change_length)) {//IdeaLaneChangeDistance should not be In SolidLine.
      int index =  ceil(lane_change_length / delta_s);
      AINFO_IF(FLAGS_enable_debug_motion)<<"lane_change_length = "<<lane_change_length<<", index = "<<index;
      piecewise_jerk_problem.set_lane_change_index(index);
    } 
  }

  piecewise_jerk_problem.set_weight_x(w[0]);
  piecewise_jerk_problem.set_weight_dx(w[1]);
  piecewise_jerk_problem.set_weight_ddx(w[2]);
  piecewise_jerk_problem.set_weight_dddx(w[3]);
  if (reference_line_info_->LaterAction().first == eActionEnum::PULL_OVER) {
    piecewise_jerk_problem.set_weight_obs(30.0);//@pqg
  } else if (FLAGS_enable_obstacle_weight ) {
    if (!reference_line_info_->IsChangeLanePath() 
      && BehaviorParser::instance()->CurrentLateralAction().first == eActionEnum::LANE_CHANGE
      || !BehaviorParser::instance()->plan_in_lane()) {//borrow lane,车道内绕行和借对向车道绕行
      piecewise_jerk_problem.set_weight_obs(w[4]);//@pqg
    } else {
      piecewise_jerk_problem.set_weight_obs(w[4]);//@pqg
    }
    
  } else {
    piecewise_jerk_problem.set_weight_obs(0.0);//@pqg
  }

  if (FLAGS_enable_dynamic_obstacle_weight) {
    piecewise_jerk_problem.set_weight_dynamic_obs(w[5]);
  } else {
    piecewise_jerk_problem.set_weight_dynamic_obs(0.0);
  }
  if (FLAGS_enable_bound_type_weight) {
    piecewise_jerk_problem.set_weight_bound_type(std::make_tuple(w[6],w[7],w[8]));
  }

  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});//这个factor? 因为dl ddl本来就比较小,所以在求解时进行缩放

  auto start_time = std::chrono::system_clock::now();

  piecewise_jerk_problem.set_x_bounds(lat_boundaries);
  //piecewise_jerk_problem.set_dx_bounds(-FLAGS_lateral_derivative_bound_default,
  //                                     FLAGS_lateral_derivative_bound_default);
  piecewise_jerk_problem.set_dx_bounds(dl_bounds);
  piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);
  piecewise_jerk_problem.set_dddx_bound(FLAGS_lateral_jerk_bound);

  double vehicle_constraint = EgoInfo::instance()->vehicle_front_edge_to_center()
                /std::max(1.4, EgoInfo::instance()->vehicle_state().linear_velocity);
  piecewise_jerk_problem.set_vehicle_constraint(vehicle_constraint);

  
  if (!FLAGS_use_const_lateral_jerk_bound) {
    // Estimate lat_acc and jerk boundary from vehicle_params
    const auto& veh_param =
        EgoInfo::instance()->vehicle_param();
    const double axis_distance = veh_param.length_wheelbase;
    const double max_yaw_rate =
        (veh_param.right_max_steerangle / 180.0 * M_PI) / veh_param.eps_transmission_ratio / 2.0;
    const double jerk_bound = EstimateJerkBoundary(std::fmax(init_state[1], 1.0),
                                                   axis_distance, max_yaw_rate);
    AINFO_IF(FLAGS_enable_debug_motion)<<"jerk_bound = "<<jerk_bound;
    piecewise_jerk_problem.set_dddx_bound(jerk_bound);
  }
  
  piecewise_jerk_problem.set_time_limit(FLAGS_piecewise_jerk_path_osqp_solver_time_limit);
  bool success = piecewise_jerk_problem.Optimize(max_iter);

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  AINFO_IF(FLAGS_enable_debug_motion) << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

  if (!success) {
    AERROR_IF(FLAGS_enable_debug_motion) << "piecewise jerk path optimizer failed";
    return false;
  }

  *x = piecewise_jerk_problem.opt_x();
  *dx = piecewise_jerk_problem.opt_dx();
  *ddx = piecewise_jerk_problem.opt_ddx();

  return true;
}

FrenetFramePath PiecewiseJerkPathOptimizer::SLToFrenet(
    const std::vector<double>& x, const std::vector<double>& dx,
    const std::vector<double>& ddx, const double delta_s,
    const double start_s) const {
  CHECK(!x.empty());
  CHECK(!dx.empty());
  CHECK(!ddx.empty());
  std::vector<common::FrenetFramePoint> frenet_frame_path;
  for (int i = 0; i < x.size(); ++i) {
    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(i * delta_s + start_s);
    frenet_frame_point.set_l(x[i]);
    frenet_frame_point.set_dl(dx[i]);
    frenet_frame_point.set_ddl(ddx[i]);
    frenet_frame_path.push_back(frenet_frame_point);
  }
  return FrenetFramePath(std::move(frenet_frame_path));

}

FrenetFramePath PiecewiseJerkPathOptimizer::ToPiecewiseJerkPath(
    const std::vector<double>& x, const std::vector<double>& dx,
    const std::vector<double>& ddx, const double delta_s,
    const double start_s,double &property_length) const {
  CHECK(!x.empty());
  CHECK(!dx.empty());
  CHECK(!ddx.empty());

  PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(), dx.front(),
                                                ddx.front());

  for (std::size_t i = 1; i < x.size(); ++i) {
    const auto dddl = (ddx[i] - ddx[i - 1]) / delta_s;
    piecewise_jerk_traj.AppendSegment(dddl, delta_s);
  }

  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;
  double min_l = 3.5; 
  double min_dl = 3.5;
  while (accumulated_s < piecewise_jerk_traj.ParamLength()) {
    double l = piecewise_jerk_traj.Evaluate(0, accumulated_s);
    double dl = piecewise_jerk_traj.Evaluate(1, accumulated_s);
    double ddl = piecewise_jerk_traj.Evaluate(2, accumulated_s);

    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s + start_s);
    frenet_frame_point.set_l(l);
    frenet_frame_point.set_dl(dl);
    frenet_frame_point.set_ddl(ddl);
    constexpr double kLateralOffset = 0.0001;//2e-2;
    if (fabs(l) > kLateralOffset) {
      property_length = frenet_frame_point.s();
    }
    frenet_frame_path.push_back(std::move(frenet_frame_point));
    accumulated_s += FLAGS_trajectory_space_resolution;
  }
  if (property_length < 0 
     && !(reference_line_info_->LaterAction().first == eActionEnum::DEFAULT_VALUE)) {
    property_length = piecewise_jerk_traj.ParamLength();
  }
  if (!frenet_frame_path.empty()) {
    if (fabs(frenet_frame_path.back().l()) < 0.05 
         && fabs(frenet_frame_path.back().dl()) < 0.0001) {
       property_length = piecewise_jerk_traj.ParamLength();
    }
  }
  
  return FrenetFramePath(std::move(frenet_frame_path));
}

double PiecewiseJerkPathOptimizer::EstimateJerkBoundary(
    const double vehicle_speed, const double axis_distance,
    const double max_yaw_rate) const {
  return max_yaw_rate / axis_distance / vehicle_speed;
}

bool PiecewiseJerkPathOptimizer::GenerateReferencePathData(double start_s, PathInfo *const path_data) {
  AINFO_IF(FLAGS_enable_debug_motion)<< "Generate path based on reference_line. start from s: "<< start_s;
  double s = start_s;
  double total_length = reference_line_info_->reference_line().Length();
  double dis_to_virtual_wall = EgoInfo::instance()->dis_to_virtual_wall() - EgoInfo::instance()->vehicle_front_edge_to_center()+0.45;
  AINFO<<"**********************dis_to_virtual_wall :"<< EgoInfo::instance()->dis_to_virtual_wall();
  total_length =std::min(dis_to_virtual_wall,  reference_line_info_->path_decision()->stop_reference_line_s());
  AINFO<<"**********************total_length :"<<total_length;
  double s_end = s;


  std::vector<common::FrenetFramePoint> origin_path;
  while (s + FLAGS_trajectory_space_resolution / 2.0 < total_length) {//以path_resolution加密路点
    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(s);
    frenet_frame_point.set_l(0.0);
    frenet_frame_point.set_dl(0.0);
    frenet_frame_point.set_ddl(0.0);
    origin_path.push_back(std::move(frenet_frame_point));
    s_end = s;
    s += FLAGS_trajectory_space_resolution;
  }
  if (s_end < total_length ) {
    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(total_length);
    frenet_frame_point.set_l(0.0);
    frenet_frame_point.set_dl(0.0);
    frenet_frame_point.set_ddl(0.0);
    origin_path.push_back(std::move(frenet_frame_point));
  }
  FrenetFramePath path(origin_path);
  path_data->SetReferenceLine(&reference_line_info_->reference_line());//参考线不变,将其存入path_data中的成员变量reference_line_
  path_data->SetFrenetPath(path);//将tunnel转化成xy坐标，更新path_data中的discretized_path_
  path_data->SetPropertyLength(reference_line_info_->reference_line().GetMapPath().property_length());
  path_data->SetOffsetPropertyLength(reference_line_info_->reference_line().GetMapPath().offset_property_length());
  path_data->SetIsNew(false);
  AINFO_IF(FLAGS_enable_debug_motion)<<"path_data->property_length :"<<path_data->property_length();
  return true;
}

bool PiecewiseJerkPathOptimizer::IsIdeaLaneChangeDistanceInSolidLine(const double& distance) const {
  if (reference_line_info_->DecisionPathBound().empty()) {
    return false;
  } 
  for (const auto& bound : reference_line_info_->DecisionPathBound()) {
    if (distance <= get<0>(bound)) {
      if (get<1>(bound) == get<2>(bound) && get<1>(bound) < 20) {
        AINFO_IF(FLAGS_enable_debug_motion)<<"IdeaLaneChangeDistance In SolidLine !";
        return true;
      } else {
        return false;
      }
    }

  }
  return false;
}

double PiecewiseJerkPathOptimizer::GetPreferredLaneChangeDistance() const {
  // double lane_change_length = 0.0;
  // lane_change_length = frame_->PlanningStartPoint().v() * FLAGS_lane_change_time_buffer;
  double preferred_distance = 0.0;
  //最小的换道距离,避免车速过快的情况变道距离过小
  double lane_change_length_min = std::fmax(frame_->PlanningStartPoint().v() * FLAGS_lane_change_time_buffer_min
          ,FLAGS_lane_change_length_minimum);
  preferred_distance = std::fmax(preferred_distance, lane_change_length_min);
  
  //目标车道没有障碍物
  if (BehaviorParser::instance()->TargetGap().first < 0 
     && BehaviorParser::instance()->TargetGap().second < 0) {
    return preferred_distance;
  }
  //目标车道前面有个障碍物
  if (BehaviorParser::instance()->TargetGap().first < 0 && 
    BehaviorParser::instance()->TargetGap().second > 0) {//lanechange position behind gap.second
    std::string obstacle_id = std::to_string(BehaviorParser::instance()->TargetGap().second);
    auto obstacle_ptr = reference_line_info_->path_decision()->Find(obstacle_id);
    if (obstacle_ptr != nullptr) {
      preferred_distance = std::fmin(preferred_distance, 
         obstacle_ptr->speed() * FLAGS_lane_change_time_buffer + obstacle_ptr->PerceptionSLBoundary().start_s());
      AINFO_IF(FLAGS_enable_debug_motion)<<"1) preferred_distance = "<<preferred_distance;
      preferred_distance = std::fmax(preferred_distance, FLAGS_lane_change_length_minimum);
    }
    return preferred_distance;
  }
  //目标车道后方有个障碍物
  if (BehaviorParser::instance()->TargetGap().first > 0 && 
    BehaviorParser::instance()->TargetGap().second < 0) {//lanechange position in front gap.first
    std::string obstacle_id = std::to_string(BehaviorParser::instance()->TargetGap().first);
    auto obstacle_ptr = reference_line_info_->path_decision()->Find(obstacle_id);
    if (obstacle_ptr != nullptr) {
      preferred_distance = std::fmax(preferred_distance, 
         obstacle_ptr->speed() * FLAGS_lane_change_time_buffer + obstacle_ptr->PerceptionSLBoundary().start_s());
      AINFO_IF(FLAGS_enable_debug_motion)<<"2)preferred_distance = "<<preferred_distance;
      preferred_distance = std::fmax(preferred_distance, FLAGS_lane_change_length_minimum);
    }
    return preferred_distance;
  }
  //目标车道前、后方都有障碍物
  if (BehaviorParser::instance()->TargetGap().first > 0 && 
    BehaviorParser::instance()->TargetGap().second > 0) {//lanechange position in gap
    std::string obstacle_id = std::to_string(BehaviorParser::instance()->TargetGap().first);
    auto start_obstacle_ptr = reference_line_info_->path_decision()->Find(obstacle_id);
    obstacle_id = std::to_string(BehaviorParser::instance()->TargetGap().second);
    auto end_obstacle_ptr = reference_line_info_->path_decision()->Find(obstacle_id);
    if (start_obstacle_ptr != nullptr && end_obstacle_ptr != nullptr) {
      //half_delta_s = (s2-s1)/2;
      double half_delta_s = 0.5 * (std::fmax(0.0, (end_obstacle_ptr->speed() - start_obstacle_ptr->speed())* FLAGS_lane_change_time_buffer) 
             + end_obstacle_ptr->PerceptionSLBoundary().start_s()
             - start_obstacle_ptr->PerceptionSLBoundary().end_s());
      AINFO_IF(FLAGS_enable_debug_motion)<<"half_delta_s = "<<half_delta_s;
      double gap_preferred_distance = 0.5 * (end_obstacle_ptr->PerceptionSLBoundary().start_s()* FLAGS_lane_change_time_buffer - half_delta_s);
      preferred_distance = std::fmin(preferred_distance, gap_preferred_distance);
      AINFO_IF(FLAGS_enable_debug_motion)<<"3)preferred_distance = "<<preferred_distance;
      preferred_distance = std::fmax(preferred_distance, FLAGS_lane_change_length_minimum);
    }
    return preferred_distance;
  }

  return preferred_distance;
}

}  // namespace planning
}  // namespace acu
