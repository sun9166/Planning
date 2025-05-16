/**
 * @file qp_spline_st_speed_optimizer.cpp
 **/

#include "qp_spline_st_speed_optimizer.h"
#include <utility>
#include <vector>
//#include "ros/ros.h"
#include "pnc_point.pb.h"

#include "common/util/file.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "qp_spline_st_graph.h"
#include "qp_piecewise_st_graph.h"

namespace acu {
namespace planning {

using acu::common::ErrorCode;
using acu::common::Status;
using acu::common::TrajectoryPoint;
using acu::planning_internal::STGraphDebug;

QpSplineStSpeedOptimizer::QpSplineStSpeedOptimizer()
  : SpeedOptimizer("QpSplineStSpeedOptimizer") {}

bool QpSplineStSpeedOptimizer::Init(const PlanningConfig& config) {
  qp_st_speed_config_ = config.planner_config().qp_st_speed_config();
  st_boundary_config_ = qp_st_speed_config_.st_boundary_config();
  std::vector<double> init_knots;
  is_init_ = true;
  if (FLAGS_use_osqp_optimizer_for_qp_st) {
    spline_solver_.reset(new math::OsqpSpline1dSolver(init_knots, 5));
  } else {
    spline_solver_.reset(new math::ActiveSetSpline1dSolver(init_knots, 5));
  }
  return true;
}


Status QpSplineStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,
    const PathInfo& path_data,
    const TrajectoryPoint& init_point,
    const ReferenceLine& reference_line,
    const SpeedInfo& reference_speed_data,
    PathDecision* const path_decision,
    SpeedInfo* const speed_data) {
  if (reference_line_info_->ReachedDestination()) { 
    return Status::OK();
  }

  // const double kDistanceBuffer = 1.0;//0.5;

  // if (reference_line_info_->DistanceToStopPoint() <= kDistanceBuffer) {
  //   std::string msg("DistanceToStopPoint "+std::to_string(reference_line_info_->DistanceToStopPoint())
  //      +"<" +std::to_string(kDistanceBuffer)+", use fallback speedplan.");
  //   AWARN_IF(FLAGS_enable_debug_motion)<< msg;
  //   const double init_v = reference_line_info_->speed_init_point().v();
  //   const double init_a = reference_line_info_->speed_init_point().a();
  //   GenerateStopSpeedProfile(reference_line_info_->DistanceToStopPoint(),init_v,init_a,speed_data); 
  //   return Status::OK();
  // }

  if (path_data.discretized_path().empty()) {
    std::string msg("Empty path data");
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  StBoundaryMapper boundary_mapper(
    adc_sl_boundary, st_boundary_config_, reference_line, path_data,
    qp_st_speed_config_.total_path_length(), qp_st_speed_config_.total_time(),
    reference_line_info_->IsChangeLanePath());

  for (const auto* obstacle : path_decision->obstacles().Items()) {
    DCHECK(obstacle->HasLongitudinalDecision());
  }
  double start_timestamp = acu::common::NodeTime::Now().ToSecond();
  // step 1 get boundaries
  path_decision->EraseStBoundaries();
  if (boundary_mapper.CreateStBoundary(path_decision).code() ==
      ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "Mapping obstacle for qp st speed optimizer failed!";
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR,
                  "Mapping obstacle for qp st speed optimizer failed!");
  }
  //log time 
  double end_timestamp_1 = acu::common::NodeTime::Now().ToSecond();
  double time_diff_ms = (end_timestamp_1 - start_timestamp) * 1000;
  auto latency_stats = reference_line_info_->mutable_latency_stats();
  auto time_latency = latency_stats->add_task_stats();
  time_latency->set_name("CreateStBoundary"); 
  time_latency->set_time_ms(time_diff_ms);

  std::vector<const StBoundary*> boundaries;
  for (auto* obstacle : path_decision->obstacles().Items()) {
    auto id = obstacle->Id();
    if (!obstacle->st_boundary().IsEmpty()) {
      path_decision->Find(id)->SetBlockingObstacle(true);
      boundaries.push_back(&obstacle->st_boundary());
    } 
  }
  // //添加动态障碍物通过的限速
  // reference_line_info_->AddPassByObstacleSpeedlimit();
  // //log time 
  double end_timestamp_2 = acu::common::NodeTime::Now().ToSecond();
  // time_diff_ms = (end_timestamp_2 - end_timestamp_1) * 1000;
  // auto time_latency_1 = reference_line_info_->mutable_latency_stats()->add_task_stats();
  // time_latency_1->set_name("AddPassByObstacleSpeedlimit"); 
  // time_latency_1->set_time_ms(time_diff_ms);

  bool has_overtake_decision = path_decision->has_overtake_obstacle(); 
  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, st_boundary_config_,
                                        reference_line, path_data,has_overtake_decision);
  SpeedLimit speed_limits;
  if (speed_limit_decider.GetSpeedLimits(path_decision->whole_obstacles(),
                                         &speed_limits) != Status::OK()) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "GetSpeedLimits for qp st speed optimizer failed!";
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR,
                  "GetSpeedLimits for qp st speed optimizer failed!");
  }
  //commit add liuya
  //speed_limits.TrimSpeedLimit(reference_line.cruise_speed() , init_point.v());
  //log time
  double end_timestamp_3 = acu::common::NodeTime::Now().ToSecond();
  time_diff_ms = (end_timestamp_3 - end_timestamp_2) * 1000;
  auto time_latency_2 = reference_line_info_->mutable_latency_stats()->add_task_stats();
  time_latency_2->set_name("GetSpeedLimits"); 
  time_latency_2->set_time_ms(time_diff_ms);

  AINFO_IF(FLAGS_enable_debug_motion)<< "speedlimit size:" << speed_limits.speed_limit_points().size();

  // step 2 perform graph search
  const CarModel veh_param = EgoInfo::instance()->vehicle_param();;

  
  double cruise_speed = reference_line.cruise_speed();
  StGraphData st_graph_data(boundaries, init_point, speed_limits,
                            path_data.discretized_path().Length(),cruise_speed);
  QpSplineStGraph st_graph(spline_solver_.get(), qp_st_speed_config_, veh_param,
                           reference_line_info_->IsChangeLanePath(),has_overtake_decision,
                           st_graph_data,reference_line_info_->path_decision());

  std::pair<double, double> accel_bound = {
    qp_st_speed_config_.preferred_min_deceleration(),
    qp_st_speed_config_.preferred_max_acceleration()
  };

  if (has_overtake_decision) {
    accel_bound.second = FLAGS_overtake_max_acceleration;//if has overtake ,set max_acc as 1.5m/ss.
  }

  AWARN_IF(FLAGS_enable_debug_motion)<<"max_acceleration = "<<accel_bound.second;
  STGraphDebug* st_graph_debug = reference_line_info_->mutable_debug()
                                     ->mutable_planning_data()
                                     ->add_st_graph();
  st_graph.SetDebugLogger(st_graph_debug, reference_line_info_->mutable_latency_stats());
  auto ret = st_graph.Search(accel_bound, reference_speed_data,
                             speed_data);

  if (ret != Status::OK()) {
    // 添加备选方案
    AWARN_IF(FLAGS_enable_debug_motion)<< "**********Use Fallback Speed Plan!!!!***************";
    bool fallback_speedplan_ok = GenerateFallbackSpeedData(speed_limits, veh_param, 
    st_graph.GetYeildStMap(),st_graph.GetYeildVtMap(),st_graph.GetTEvaluated(), path_decision, speed_data,path_data);
    if (fallback_speedplan_ok) {
      AWARN_IF(FLAGS_enable_debug_motion)<< "Failed to solve with ideal acceleration conditions.";
      AWARN_IF(FLAGS_enable_debug_motion)<< "**********Use Fallback Speed Plan!!!!***************";
      return Status::OK();
    }
    std::string msg = common::util::StrCat(
                          Name(), ": Failed to search graph with quadratic programming!");
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }
  return Status::OK();
}


bool QpSplineStSpeedOptimizer::GenerateFallbackSpeedData(
          const SpeedLimit speed_limit, const CarModel car_model, 
          const  std::vector<std::vector<std::pair<double, double>> > yield_st_map, 
          const  std::vector<std::vector<std::pair<double, double>> > yield_vt_map, 
          std::vector<double> t_evaluated,
          const PathDecision* path_decision, 
          SpeedInfo* const speed_data,
          const PathInfo& path_data)  {
  // std::vector<SpeedPoint> speed_profile;
  auto DP_ptr = acu::planning::DataPool::Instance()->GetMainDataPtr();
  speed_data->clear();
  if (reference_line_info_ == nullptr) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "reference_line_info_.";
    return false;
  }
  if (path_data.discretized_path().empty()) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "path_data.discretized_path().empty().";
    return false;
  };

  const double init_v = reference_line_info_->speed_init_point().v();
  // const auto& obstacles = path_decision->obstacles();
  double dest_s = qp_st_speed_config_.total_path_length();
  double dest_v = reference_line_info_->reference_line_ptr()->cruise_speed();

  //跟随目标或跟停目标
  for (auto obstacle : path_decision->obstacles().Items()) {
    const auto& decision = obstacle->LongitudinalDecision();
    if (decision.has_stop()) {
      double main_obstacle_stop_dis = obstacle->PerceptionSLBoundary().start_s() 
              - obstacle->GetStopDistance(car_model) 
              - EgoInfo::instance()->vehicle_front_edge_to_center();
      common::PathPoint stop_point;
      if (path_data.GetPathPointWithRefS(main_obstacle_stop_dis, &stop_point) && !obstacle->IsVirtual()) {
        dest_s = std::fmin(main_obstacle_stop_dis , stop_point.s());
        dest_v = 0.0;
        AINFO_IF(FLAGS_enable_debug_motion)<<"stop_point.s(): "<<stop_point.s();
      }
    }

    if (decision.has_follow()) {
      const double obstacle_v = obstacle->st_boundary().STspeed();
      double follow_v =  std::fmin(obstacle_v, init_v);
      double follow_dis = std::fmax(2.5*follow_v, FLAGS_follow_min_distance);
      dest_s = std::min(dest_s, obstacle->st_boundary().min_s() - follow_dis);
      dest_v = std::min(dest_v, obstacle_v);
      if (obstacle_v > 1.2*init_v) {
        dest_s = std::min(dest_s, obstacle->st_boundary().min_s() - FLAGS_follow_min_distance);
        dest_v = std::min(init_v, obstacle_v);
      }
    }
  }

  // const auto main_focus_obstacle = path_decision->MainFocusObstacle();
  //让行目标基于st_map计算，因为基本不在同一车道上
   bool need_yield = false;
  for (const double curr_t : t_evaluated) {
    if (!yield_vt_map.empty() &&!yield_st_map.empty() && yield_st_map.size() == t_evaluated.size()) {
      int t = floor(curr_t / 0.2);
      for (const auto& st_point:yield_st_map.at(t)) {
        if (st_point.second >= 0.8) {
          need_yield = true;
          dest_s = std::fmin(dest_s, st_point.first - FLAGS_yield_distance
                        - std::fmax(FLAGS_overall_system_delay*init_v, 2.0));
        }
      }
      if (need_yield) {
        for (const auto& vt_point:yield_vt_map.at(t)) {
          if (vt_point.first < 0) {
            dest_v = 0;
          } else {
            dest_v = std::fmin(dest_v, 0.8*vt_point.first);//让行时速度比障碍物速度再低一点
          }
        }
      }
    }
  }
  //
  AINFO_IF(FLAGS_enable_debug_motion)<<"dest_s = "<< dest_s <<", dest_v = "<< dest_v;
  double s_speed_limit = speed_limit.GetSpeedLimitByS(dest_s);
  if (s_speed_limit < dest_v)  dest_v = s_speed_limit; 

  for (auto speed_limit_point : speed_limit.speed_limit_points()) {
    if (speed_limit_point.second < dest_v) {
      dest_s = std::min(dest_s, speed_limit_point.first);
      dest_v = speed_limit_point.second;
    }
  }

  const double max_acc = 2.0;
  const double min_acc = -3.0;
  double acceleration = 0.0;
  if (dest_s < 0.2) dest_s = 0;//s不能为负值

  //说明有需要处理的目标
  acceleration = 0.5*(dest_v*dest_v - init_v*init_v)/(dest_s + FLAGS_numerical_epsilon);
  if (acceleration > max_acc) acceleration = max_acc;
  if (acceleration < min_acc) acceleration = min_acc;

  if (/*dest_v< 0.2 && */dest_s < 0.2){ 
    acceleration = -1.2;
  }
  if (acceleration > 0) {
    ConstantAccelerationTrajectory1d const_acc_traj(0.0, init_v, dest_v, acceleration);
    for (double t = 0.0; t <= FLAGS_trajectory_time_length; t += FLAGS_trajectory_time_min_interval) {
      double s = const_acc_traj.Evaluate(0, t);
      double v = const_acc_traj.Evaluate(1, t);
      double a = const_acc_traj.Evaluate(2, t);
      double da = const_acc_traj.Evaluate(3, t);
      AWARN_IF(FLAGS_enable_debug_speedplan)<<"t = "<<time<<", s = "<<s<<", v = "<<v<<", a = "<<a<<", da = "<<da;
      speed_data->AppendSpeedPoint(s, t, v, a, da);
    }
  } else if (acceleration < 0){
    ConstantDecelerationTrajectory1d const_acc_traj(0.0, init_v, acceleration);
    for (double t = 0.0; t <= FLAGS_trajectory_time_length; t += FLAGS_trajectory_time_min_interval) {
      double s = const_acc_traj.Evaluate(0, t);
      double v = const_acc_traj.Evaluate(1, t);
      double a = const_acc_traj.Evaluate(2, t);
      double da = const_acc_traj.Evaluate(3, t);
      AWARN_IF(FLAGS_enable_debug_speedplan)<<"t = "<<time<<", s = "<<s<<", v = "<<v<<", a = "<<a<<", da = "<<da;
      speed_data->AppendSpeedPoint(s, t, v, a, da);
    } 
  } else {
    for (double t = 0.0; t <= FLAGS_trajectory_time_length; t += FLAGS_trajectory_time_min_interval) {
      double s = init_v*t;
      speed_data->AppendSpeedPoint(s, t, init_v, 0.0, 0.0);
    } 
  }
  return true;
}

}  // namespace planning
}  // namespace acu
