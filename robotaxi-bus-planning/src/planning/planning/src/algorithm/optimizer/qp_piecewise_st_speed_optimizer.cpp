/**
 * @file qp_spline_st_speed_optimizer.cpp
 **/

#include "qp_piecewise_st_speed_optimizer.h"
#include <utility>
#include <vector>
////#include "ros/ros.h"
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

QpPiecewiseStSpeedOptimizer::QpPiecewiseStSpeedOptimizer()
  : SpeedOptimizer("QpPiecewiseStSpeedOptimizer") {}

bool QpPiecewiseStSpeedOptimizer::Init(const PlanningConfig& config) {
  qp_st_speed_config_ = config.planner_config().qp_st_speed_config();
  st_boundary_config_ = qp_st_speed_config_.st_boundary_config();
  std::vector<double> init_knots;
  is_init_ = true;
  return true;
}


Status QpPiecewiseStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,
    const PathInfo& path_data,
    const TrajectoryPoint& init_point,
    const ReferenceLine& reference_line,
    const SpeedInfo& reference_speed_data,
    PathDecision* const path_decision,
    SpeedInfo* const speed_data) {
  if (reference_line_info_->ReachedDestination()) { 
    return Status::OK();
  }

  const double kDistanceBuffer = 1.0;//0.5;

  if (reference_line_info_->DistanceToStopPoint() <= kDistanceBuffer) {
    std::string msg("DistanceToStopPoint "+std::to_string(reference_line_info_->DistanceToStopPoint())
       +"<" +std::to_string(kDistanceBuffer)+", use fallback speedplan.");
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    const double init_v = reference_line_info_->speed_init_point().v();
    const double init_a = reference_line_info_->speed_init_point().a();
    GenerateStopSpeedProfile(reference_line_info_->DistanceToStopPoint(),init_v,init_a,speed_data); 
    return Status::OK();
  }

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
  double start_timestamp = acu::common::NodeTime::Now().ToSecond();//acu::common::NodeTime::Now().ToSecond();
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

  reference_line_info_->AddPassByObstacleSpeedlimit();
  //log time 
  double end_timestamp_2 = acu::common::NodeTime::Now().ToSecond();;
  time_diff_ms = (end_timestamp_2 - end_timestamp_1) * 1000;
  auto time_latency_1 = reference_line_info_->mutable_latency_stats()->add_task_stats();
  time_latency_1->set_name("AddPassByObstacleSpeedlimit"); 
  time_latency_1->set_time_ms(time_diff_ms);

  bool has_overtake_decision = path_decision->has_overtake_obstacle(); 
  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, st_boundary_config_,
                                        reference_line, path_data,has_overtake_decision);
  SpeedLimit speed_limits;
  if (speed_limit_decider.GetSpeedLimits(path_decision->obstacles(),
                                         &speed_limits) != Status::OK()) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "GetSpeedLimits for qp st speed optimizer failed!";
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR,
                  "GetSpeedLimits for qp st speed optimizer failed!");
  }

  //log time
  double end_timestamp_3 = acu::common::NodeTime::Now().ToSecond();//acu::common::NodeTime::Now().ToSecond();
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
  QpPiecewiseStGraph piecewise_st_graph(qp_st_speed_config_,st_graph_data);

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
  piecewise_st_graph.SetDebugLogger(st_graph_debug);
  auto ret = piecewise_st_graph.Search(speed_data, accel_bound);
  
  if (ret != Status::OK()) {
    std::string msg = common::util::StrCat(
                        Name(), ": Failed to search graph with quadratic programming!");
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }
  return Status::OK();
}


void QpPiecewiseStSpeedOptimizer::GenerateStopSpeedProfile(
          const double stop_point_s,const double init_speed,
          const double init_acc,SpeedInfo* const speed_data) {
  speed_data->clear();
  double max_t = FLAGS_fallback_total_time;
  const double unit_t = FLAGS_fallback_time_unit;
  AINFO_IF(FLAGS_enable_debug_motion)<<"fallback stop_distance: "<<stop_point_s
           <<", v = "<<init_speed<<", init_acc = "<<init_acc;         
  if (stop_point_s <= 0.1 || init_speed <= 0.05) {
    for (double t = 0.0; t < max_t; t += unit_t) {
      double s = 0.0;
      double v = 0.0;
      speed_data->AppendSpeedPoint(s, t, v, -5, 0.0);
    }
    return ;
  } 

  double acc = -1 * init_speed * init_speed / (2 * stop_point_s);
  AINFO_IF(FLAGS_enable_debug_motion)<<"fallback acc: "<<acc;  
  if (fabs(acc - 0.0) < 0 || acc < -5) {
    for (double t = 0.0; t < max_t; t += unit_t) {
      double s = 0.0;
      double v = 0.0;
      speed_data->AppendSpeedPoint(s, t, v, -5, 0.0);
    }
    return ;
  }    
  max_t = init_speed / fabs(acc); 
  double pre_s = 0.0;
  double pre_v = init_speed;
  if (max_t <= unit_t) {
    for (double t = 0.0; t < max_t; t += unit_t) {
      double s = 0.0;
      double v = 0.0;
      speed_data->AppendSpeedPoint(s, t, v, -5, 0.0);
    }
    return ;
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<"max_t = "<<max_t;
  for (double t = 0.0; t < max_t; t += unit_t) {
    double s = 0.0;
    double v = 0.0;
    if (t > 0) {
      v = std::fmax(0.0, pre_v + acc * unit_t);
      s = (v *v - pre_v * pre_v ) / ( 2 * acc) + pre_s;
      speed_data->AppendSpeedPoint(s, t, v, acc, 0.0);
    } else {
      s = 0.0;
      v = std::fmax(0.0,pre_v);
      t = 0.0;
      speed_data->AppendSpeedPoint(s, t, v, init_acc, 0.0);
    }
    
    pre_s = s;
    pre_v = v;
  }
  double s = 0.5 * fabs(acc) * max_t * max_t;
  speed_data->AppendSpeedPoint(s, max_t, 0.0000, acc, 0.0);
  AINFO_IF(FLAGS_enable_debug_motion)<<"s = "<<speed_data->back().s()
    <<", v = "<<speed_data->back().v()<<", t ="<<speed_data->back().t();

}

}  // namespace planning
}  // namespace acu
