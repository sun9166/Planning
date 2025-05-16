/**
 * @file piecewise_jerk_path_optimizer.cpp
 **/
#include <string>
#include "datapool/include/data_pool.h"
#include "quasi_potential_field_path_optimizer.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/common/speed/speed_info.h"
#include "src/algorithm/curve/piecewise_jerk_trajectory1d.h"
#include "quasi_potential_field_path_problem.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/scenario_context.h"
#include "src/execution/motionplan/common/new_path_manager.h"


namespace acu {
namespace planning {

using acu::common::ErrorCode;
using acu::common::Status;

QuasiPotentialFieldPathOptimizer::QuasiPotentialFieldPathOptimizer()
    : PathOptimizer("QuasiPotentialFieldPathOptimizer") {
}

bool QuasiPotentialFieldPathOptimizer::Init(const PlanningConfig &config) {
  CHECK(config.planner_config().has_quasi_potential_field_path_config());
  config_ = config.planner_config().quasi_potential_field_path_config();
  is_init_ = true;
  return true;
}

common::Status QuasiPotentialFieldPathOptimizer::Process(
    const SpeedInfo& speed_data, const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point, 
    PathInfo* const path_data) {
  // skip piecewise_jerk_path_optimizer if reused path
  bool path_reusable = 
       reference_line_info_->LaterAction().first == eActionEnum::DEFAULT_VALUE? true:false;
  auto DP_ptr = acu::planning::DataPool::Instance()->GetMainDataPtr();
  bool is_start_from_road_side = false;  
  auto init_sl_point = reference_line_info_->init_sl_point();
  AINFO_IF(FLAGS_enable_debug_motion)<<"QUASI plan start from s = " <<init_sl_point.s()
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
  AINFO_IF(EgoInfo::instance()->vehicle_state().driving_mode == 2)<<"QUASI standby realtime pathplan.";
  common::TrajectoryPoint planning_start_point = init_point;
  if (FLAGS_enable_two_stitching_trajectory) {
    planning_start_point = reference_line_info_->pathplan_init_point();
  }
  auto init_frenet_state =
      reference_line.ToFrenetFrame(planning_start_point);
  AINFO_IF(FLAGS_enable_debug_motion)<<"QUASI init s = ("<<init_frenet_state.first[0]
         <<", "<<init_frenet_state.first[1]<<", "<<init_frenet_state.first[2]
         <<", l = ("<<init_frenet_state.second[0]
         <<", "<<init_frenet_state.second[1]<<", "<<init_frenet_state.second[2];
  
  if (frame_->StartPointType() == 0 
      || EgoInfo::instance()->vehicle_state().driving_mode == 2) {
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
  auto quasi_potential_field_path_config = config_.default_path_config();
  if (is_start_from_road_side) {
    quasi_potential_field_path_config = config_.start_from_road_side_config();
  }        

  std::array<double, 8> w = {
      quasi_potential_field_path_config.l_weight(),
      quasi_potential_field_path_config.dl_weight() *
          std::fmax(init_frenet_state.first[1] * init_frenet_state.first[1],
                    5.0),
      quasi_potential_field_path_config.ddl_weight(),
      quasi_potential_field_path_config.dddl_weight(),
      quasi_potential_field_path_config.dotted_line_weight(),
      quasi_potential_field_path_config.solid_line_weight(),
      quasi_potential_field_path_config.curb_weight(),
      quasi_potential_field_path_config.obstacle_weight()};
  const auto& path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();//获取备选的path_boundaries,该结果由path_bound_decider完成
  AINFO_IF(FLAGS_enable_debug_motion) << "QUASI There are " << path_boundaries.size() << " path boundaries.";

  std::vector<PathInfo> candidate_path_data;
  bool res_opt = false;
  for (const auto& path_boundary : path_boundaries) {
    // if the path_boundary is normal, it is possible to have less than 2 points
    // skip path boundary of this kind
    if (path_boundary.label().find("regular") != std::string::npos &&
        path_boundary.boundary().size() < 2) {
      continue;
    }

    if (path_boundary.label().find("fallback") != std::string::npos && 
      path_boundaries.size() >= 2) { // 先不规划fallback路径 @pqg
      AINFO_IF(FLAGS_enable_debug_motion)<<"QUASI fallback boundary , continue ......";
      continue;
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
    std::vector<double> left_soft_delt_l;
    std::vector<double> right_soft_delt_l;

    std::array<double, 5> end_state = {0.0, 0.0, 0.0, 0.0, 0.0};

    if (BehaviorParser::instance()->pull_over_modified_l()<50 &&
        path_boundary.label().find("pullover") != std::string::npos) {
      end_state[0] = BehaviorParser::instance()->pull_over_modified_l();
    }

    const auto& veh_param =
        EgoInfo::instance()->vehicle_param();
    const double front_wheel_angle_max = 
         (fabs(veh_param.right_max_steerangle) / fabs(veh_param.eps_transmission_ratio));//degree not radian    
    CHECK_LT(front_wheel_angle_max, 90);
    //FLAGS_lateral_acc_bound：0.333应该改成0.2
    double lat_acc_bound = FLAGS_use_const_lateral_acc_bound ? FLAGS_lateral_acc_bound : 
              std::tan(front_wheel_angle_max /180 * M_PI) / veh_param.length_wheelbase;
    AINFO_IF(FLAGS_enable_debug_motion)<<"QUASI 1) lat_acc_bound = "<<lat_acc_bound;  
    std::vector<std::pair<double, double>> ddl_bounds;
    for (size_t i = 0; i < path_boundary.boundary().size(); ++i) {
      double s = static_cast<double>(i) * path_boundary.delta_s() +
                 path_boundary.start_s();
      double kappa = reference_line.GetNearestReferencePoint(s).kappa();
      ddl_bounds.emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
    }
    std::array<double, 5> init_state = {0.0, 0.0, 0.0, 0.0, 0.0};
    init_state[0] = init_frenet_state.second[0];
    init_state[1] = init_frenet_state.second[1];
    init_state[2] = init_frenet_state.second[2];
    res_opt =
        OptimizePath(init_state, end_state,
                     path_boundary.delta_s(), path_boundary.boundary(),path_boundary.soft_boundary(),
                     ddl_bounds, path_boundary.boundary_type(),w, &opt_l, &opt_dl, &opt_ddl,
                      &left_soft_delt_l, &right_soft_delt_l, max_iter);
    AERROR << "QUASI res_opt success ";

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
      AINFO_IF(FLAGS_enable_debug_motion) << "QUASI path_boundary.boundary().size =  " << path_boundary.boundary().size();
      AINFO_IF(FLAGS_enable_debug_motion) << "QUASI path_boundary.boundary().back s =  " 
            << static_cast<double>((int)path_boundary.boundary().size() -1) * path_boundary.delta_s();
      for (size_t i = 0; i < path_boundary.boundary().size(); i += 4) {
       AINFO_IF(FLAGS_enable_debug_motion) << "QUASI for s1[" << static_cast<double>(i) * path_boundary.delta_s()
                   << "], l = " << opt_l[i] << ", dl = " << opt_dl[i] << ", ddl = " << opt_ddl[i] << ", left_soft_delt_l[i] = " << left_soft_delt_l[i] << 
                   ", right_soft_delt_l[i] = " << right_soft_delt_l[i];
      }
      double property_s = -1;
      auto frenet_frame_path =
          ToPiecewiseJerkPath(opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
                              path_boundary.start_s(),property_s);
      AINFO_IF(FLAGS_enable_debug_motion)<<"QUASI property_s = "<<property_s; 
      if (FLAGS_enable_two_stitching_trajectory
        && frame_->StitchingTrajectoryForPath().size() > frame_->last_stitching_trajectory().size()) {
        AddPathPlanStithingTrajectory(frenet_frame_path); 
      } 
       
      PathInfo path_candidate = *path_data;
      path_candidate.SetReferenceLine(&reference_line);
      path_data->SetReferenceLine(&reference_line);
      path_candidate.SetFrenetPath(std::move(frenet_frame_path));
      path_data->SetFrenetPath(std::move(frenet_frame_path));

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
  if (!res_opt && path_boundaries.size() >= 2) {
    for (const auto& path_boundary : path_boundaries) {
      if (path_boundary.label().find("fallback") != std::string::npos) { // 规划fallback路径 @pqg
        AWARN_IF(FLAGS_enable_debug_motion)<<"fallback pathplan !!!!!!!!";
        int max_iter = FLAGS_max_iter;
        CHECK_GT(path_boundary.boundary().size(), 1);
    
        std::vector<double> opt_l;
        std::vector<double> opt_dl;
        std::vector<double> opt_ddl;
        std::vector<double> left_soft_delt_l;
        std::vector<double> right_soft_delt_l;
    
        std::array<double, 5> end_state = {0.0, 0.0, 0.0, 0.0, 0.0};
    
        const auto& veh_param =
            EgoInfo::instance()->vehicle_param();
        const double front_wheel_angle_max = 
             (fabs(veh_param.right_max_steerangle) / fabs(veh_param.eps_transmission_ratio));//degree not radian    
        CHECK_LT(front_wheel_angle_max, 90);
        const double lat_acc_bound =
              std::tan(front_wheel_angle_max /180 * M_PI) / veh_param.length_wheelbase;    
        std::vector<std::pair<double, double>> ddl_bounds;
        for (size_t i = 0; i < path_boundary.boundary().size(); ++i) {
          double s = static_cast<double>(i) * path_boundary.delta_s() +
                     path_boundary.start_s();
          double kappa = reference_line.GetNearestReferencePoint(s).kappa();
          ddl_bounds.emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
        }
        std::array<double, 5> init_state = {0.0, 0.0, 0.0, 0.0, 0.0};
        init_state[0] = init_frenet_state.second[0];
        init_state[1] = init_frenet_state.second[1];
        init_state[2] = init_frenet_state.second[2];
        res_opt =
            OptimizePath(init_state, end_state,
                         path_boundary.delta_s(), path_boundary.boundary(),path_boundary.soft_boundary(),
                         ddl_bounds, path_boundary.boundary_type(),w, &opt_l, &opt_dl, &opt_ddl, 
                         &left_soft_delt_l, &right_soft_delt_l, max_iter);
    
        if (res_opt) {
          for (size_t i = 0; i < path_boundary.boundary().size(); i += 4) {
            AINFO_IF(FLAGS_enable_debug_motion) << "QUASI for s2[" << static_cast<double>(i) * path_boundary.delta_s()
                   << "], l = " << opt_l[i] << ", dl = " << opt_dl[i] << ", ddl = " << opt_ddl[i] << ", left_soft_delt_l[i] = " << left_soft_delt_l[i] << 
                   ", right_soft_delt_l[i] = " << right_soft_delt_l[i];
          }
          double property_s = -1;
          auto frenet_frame_path =
              ToPiecewiseJerkPath(opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
                                  path_boundary.start_s(),property_s);
          if (FLAGS_enable_two_stitching_trajectory 
            && frame_->StitchingTrajectoryForPath().size() > frame_->last_stitching_trajectory().size()) {
            AddPathPlanStithingTrajectory(frenet_frame_path); 
          }     
          PathInfo path_candidate = *path_data;
          path_candidate.SetReferenceLine(&reference_line);
          path_data->SetReferenceLine(&reference_line);
          path_candidate.SetFrenetPath(std::move(frenet_frame_path));
          path_data->SetFrenetPath(std::move(frenet_frame_path));

          if (FLAGS_enable_two_stitching_trajectory 
            && frame_->StitchingTrajectoryForPath().size() > frame_->last_stitching_trajectory().size()) {
            size_t index = frame_->PathStitchingStartIndex();
            std::vector<common::TrajectoryPoint> update_path(frame_->StitchingTrajectoryForPath().begin() + index, 
                                        frame_->StitchingTrajectoryForPath().end());
            path_data->UpdateDiscretizedPath(update_path);
          }
          

          path_candidate.set_path_label(path_boundary.label());
          path_data->set_path_label(path_boundary.label());
          candidate_path_data.push_back(std::move(path_candidate));
          path_data->SetPropertyLength(property_s);
          break;
        }
      }
    }
  }
  if (candidate_path_data.empty()) {
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR,
                  "Path Optimizer failed to generate path");
  }
  reference_line_info_->SetCandidatePathData(std::move(candidate_path_data));
  return Status::OK();
}

void QuasiPotentialFieldPathOptimizer::AddPathPlanStithingTrajectory(FrenetFramePath& frenet_frame_path) {
  FrenetFramePath stitching_path;  
  size_t index = frame_->PathStitchingStartIndex();
  if (index - 1 < 0) return ;
  // AINFO_IF(FLAGS_enable_debug_motion && frame_->StitchingTrajectoryForPath().size())
  //        <<fixed<<"speed plan stitching_trajectory xgyg = ("
  //        <<frame_->StitchingTrajectoryForPath().at(index).path_point().x()<<", "
  //        <<frame_->StitchingTrajectoryForPath().at(index).path_point().y()<<") . ";
  for (size_t i = index - 1; i + 1 < frame_->StitchingTrajectoryForPath().size();++i) {
    common::FrenetFramePoint frenet_frame_point;
    auto frenet_point = reference_line_info_->reference_line().ToFrenetFrame(frame_->StitchingTrajectoryForPath().at(i));
    frenet_frame_point.set_s(frenet_point.first[0]);
    frenet_frame_point.set_l(frenet_point.second[0]);
    frenet_frame_point.set_dl(frenet_point.second[1]);
    frenet_frame_point.set_ddl(frenet_point.second[2]);
    stitching_path.push_back(frenet_frame_point);
    // AINFO_IF(FLAGS_enable_debug_motion)<<"s= "<<frenet_frame_point.s()
    //        <<", l = ("<<frenet_frame_point.l()<<", "<<frenet_frame_point.dl()
    //        <<", "<<frenet_frame_point.ddl()<<").";
  }
  frenet_frame_path.insert(frenet_frame_path.begin(),stitching_path.begin(),stitching_path.end());

  // for (const auto& ff_point:frenet_frame_path) {
  //   AINFO_IF(FLAGS_enable_debug_motion)<<"final s= "<<ff_point.s()
  //          <<", l = ("<<ff_point.l()<<", "<<ff_point.dl()
  //          <<", "<<ff_point.ddl()<<").";
  // }

}

bool QuasiPotentialFieldPathOptimizer::OptimizePath(
    const std::array<double, 5>& init_state,
    const std::array<double, 5>& end_state, const double delta_s,
    const std::vector<std::pair<double, double>>& lat_boundaries,
    const std::vector<std::tuple<double, double, double, double, std::string>>& soft_boundaries,
    const std::vector<std::pair<double, double>>& ddl_bounds,
    const std::vector<std::pair<int, int>>& boundary_types,
    const std::array<double, 8>& w, std::vector<double>* x,
    std::vector<double>* dx, std::vector<double>* ddx, 
    std::vector<double>* left_soft_delt_l, std::vector<double>* right_soft_delt_l,
    const int max_iter) {
  QuasiPotentialFieldPathProblem piecewise_jerk_problem(lat_boundaries.size(),
                                                  delta_s, init_state);
  piecewise_jerk_problem.set_end_state_ref({1000.0, 0.0, 0.0, 0.0 ,0.0}, end_state);//1000,0,0分别为终点l.dl.ddl的惩罚权重
  if (end_state[0] != 0) {
    std::vector<double> x_ref(lat_boundaries.size(), end_state[0]);
    const double weight_x_ref = 10.0;    
    piecewise_jerk_problem.set_x_ref(weight_x_ref, x_ref);
  }
  piecewise_jerk_problem.set_x_bound_types(boundary_types);
  piecewise_jerk_problem.set_weight_x(w[0]);
  piecewise_jerk_problem.set_weight_dx(w[1]);
  piecewise_jerk_problem.set_weight_ddx(w[2]);
  piecewise_jerk_problem.set_weight_dddx(w[3]);
  piecewise_jerk_problem.set_weight_dotted_line(w[4]);
  piecewise_jerk_problem.set_weight_solid_line(w[5]);
  piecewise_jerk_problem.set_weight_curb(w[6]);
  piecewise_jerk_problem.set_weight_obstacle(w[7]);
  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

  auto start_time = std::chrono::system_clock::now();
  int j = 0;
  for(const auto& lat_boundary : lat_boundaries){
    j++;
    if(std::get<1>(lat_boundary) - std::get<0>(lat_boundary) <= 0){
      AINFO_IF(FLAGS_enable_debug_motion)<< "j = " << j 
            << " std::get<1>(lat_boundary) - std::get<0>(lat_boundary) = " 
            << std::get<1>(lat_boundary) - std::get<0>(lat_boundary);
    }
  }
  piecewise_jerk_problem.set_x_bounds(lat_boundaries);
  piecewise_jerk_problem.set_dx_bounds(-FLAGS_lateral_derivative_bound_default,
                                       FLAGS_lateral_derivative_bound_default);
  piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);
  std::vector<std::pair<double, double>> soft_x_bounds;
  j = 0;
  for(const auto& soft_boundary : soft_boundaries){
    j++;
    std::pair<double, double> soft_x_bound;
    soft_x_bound.first = std::get<0>(soft_boundary);
    soft_x_bound.second = std::get<1>(soft_boundary);
    if(soft_x_bound.second - soft_x_bound.first <= 0){
      AINFO_IF(FLAGS_enable_debug_motion)<< "j = " << j 
          << " soft_x_bound.second - soft_x_bound.first = " 
          << soft_x_bound.second - soft_x_bound.first;
    }
    soft_x_bounds.push_back(soft_x_bound);
  }
  piecewise_jerk_problem.set_soft_x_bounds(soft_x_bounds);
  piecewise_jerk_problem.set_dddx_bound(FLAGS_lateral_jerk_bound);

  
  if (!FLAGS_use_const_lateral_jerk_bound) {
    // Estimate lat_acc and jerk boundary from vehicle_params
    const auto& veh_param =
        EgoInfo::instance()->vehicle_param();
    const double axis_distance = veh_param.length_wheelbase;
    const double max_yaw_rate =
        // veh_param.max_steer_angle_rate() / veh_param.steer_ratio() / 2.0;
        // veh_param.max_steer_angle_rate() / veh_param.eps_transmission_ratio / 2.0;
        8.55 / 16 / 2.0; //TODO
    const double jerk_bound = EstimateJerkBoundary(std::fmax(init_state[1], 1.0),
                                                   axis_distance, max_yaw_rate);
    AINFO_IF(FLAGS_enable_debug_motion)<<"jerk_bound = "<<jerk_bound;
    piecewise_jerk_problem.set_dddx_bound(jerk_bound);
  }
  
  AINFO_IF(FLAGS_enable_debug_motion)<<"START Optimize!!!!!!!!!!!!!";
  bool success = piecewise_jerk_problem.Optimize(max_iter);

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  AINFO_IF(1) << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

  if (!success) {
    AERROR_IF(FLAGS_enable_debug_motion) << "piecewise jerk path optimizer failed";
    return false;
  }

  *x = piecewise_jerk_problem.opt_x();
  *dx = piecewise_jerk_problem.opt_dx();
  *ddx = piecewise_jerk_problem.opt_ddx();
  *left_soft_delt_l = piecewise_jerk_problem.opt_left_soft_delt_x();
  *right_soft_delt_l = piecewise_jerk_problem.opt_right_soft_delt_x();

  return true;
}

FrenetFramePath QuasiPotentialFieldPathOptimizer::ToPiecewiseJerkPath(
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

double QuasiPotentialFieldPathOptimizer::EstimateJerkBoundary(
    const double vehicle_speed, const double axis_distance,
    const double max_yaw_rate) const {
  return max_yaw_rate / axis_distance / vehicle_speed;
}

bool QuasiPotentialFieldPathOptimizer::GenerateReferencePathData(double start_s, PathInfo *const path_data) {
  AINFO_IF(FLAGS_enable_debug_motion)<< "Generate path based on reference_line. start from s: "<< start_s;
  double s = start_s;
  double total_length = reference_line_info_->reference_line().Length();
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

bool QuasiPotentialFieldPathOptimizer::IsIdeaLaneChangeDistanceInSolidLine(const double& distance) const {
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

double QuasiPotentialFieldPathOptimizer::GetPreferredLaneChangeDistance() const {
  double preferred_distance = 0.0;
  preferred_distance = frame_->PlanningStartPoint().v() * FLAGS_lane_change_time_buffer;
  preferred_distance = std::fmax(preferred_distance, FLAGS_lane_change_length_minimum);
  AINFO_IF(FLAGS_enable_debug_motion)<<"0) preferred_distance = "<<preferred_distance; 
  if (BehaviorParser::instance()->TargetGap().first < 0 
     && BehaviorParser::instance()->TargetGap().second < 0) {
    return preferred_distance;
  }
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
