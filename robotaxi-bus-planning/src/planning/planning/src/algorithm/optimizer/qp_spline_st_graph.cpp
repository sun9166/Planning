/** 
 * @file qp_spline_st_graph.cpp
 **/

////#include "ros/ros.h"
#include <algorithm>
#include <limits>
#include <string>

////#include "ros/ros.h"
#include "qp_spline_st_graph.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/algorithm/optimizer/piecewise_jerk/piecewise_jerk_speedlimit_smooth_problem.h"

namespace acu {
namespace planning {

using acu::common::ErrorCode;
using acu::common::SpeedPoint;
using acu::common::Status;

QpSplineStGraph::QpSplineStGraph(math::Spline1dSolver* spline_solver,
                                 const QpStSpeedConfig& qp_st_speed_config,
                                 const CarModel& veh_param,//@pqg
                                 const bool is_change_lane,
                                 const bool has_overtake,
                                 const StGraphData& st_graph_data,
                                 PathDecision* const path_decision)
  : spline_solver_(spline_solver),
    qp_st_speed_config_(qp_st_speed_config),
    is_change_lane_(is_change_lane),
    has_overtake_(has_overtake),
    vehicle_param_(veh_param),
    init_point_(st_graph_data.init_point()),
	path_object_decision_(path_decision),
    t_knots_resolution_(
      qp_st_speed_config_.total_time() /
      (qp_st_speed_config_.qp_spline_config().number_of_discrete_graph_t() - 1)),
      st_graph_data_(st_graph_data) {
  Init(path_decision);
}

void QpSplineStGraph::Init(PathDecision* const path_decision) {
  // init knots
  double curr_t = 0.0;
  uint32_t num_spline =
    qp_st_speed_config_.qp_spline_config().number_of_discrete_graph_t() - 1;//5-1
  for (uint32_t i = 0; i <= num_spline; ++i) {
    t_knots_.push_back(curr_t);//0,2,4,6.8
    curr_t += t_knots_resolution_;//t_knots_resolution_ 为 2
  }

  uint32_t num_evaluated_t = 10 * num_spline + 1;

  // init evaluated t positions
  curr_t = 0;
  t_evaluated_resolution_ = qp_st_speed_config_.total_time() / (num_evaluated_t - 1);//0.02s
  for (uint32_t i = 0; i < num_evaluated_t; ++i) {
    t_evaluated_.push_back(curr_t);//8s/40:0.02s
    curr_t += t_evaluated_resolution_;
  }
  // sidepass_obstacle_v_upper_bound_.clear();
  // sidepass_obstacle_v_upper_bound_.assign(num_evaluated_t, std::numeric_limits<double>::max());
  if (FLAGS_enable_plan_based_on_stmap  && 
      (!is_change_lane_ || !BehaviorParser::instance()->has_lateral_behavior())) {
    GetStMap(path_decision);
  }
}

void QpSplineStGraph::SetDebugLogger(
  planning_internal::STGraphDebug* st_graph_debug, LatencyStats* latency_ptr) {
  if (st_graph_debug) {
    st_graph_debug->Clear();
    st_graph_debug_ = st_graph_debug;
  }
  if (latency_ptr) {
    latency_infos_ = latency_ptr;
  }
}

Status QpSplineStGraph::Search(const std::pair<double, double>& accel_bound,
                               const SpeedInfo& reference_speed_data,
                               SpeedInfo* const speed_data) {
  constexpr double kBounadryEpsilon = 1e-2;
  // for (auto boundary : st_graph_data_.st_boundaries()) {
  //   if (boundary->IsPointInBoundary({0.0, 0.0}) ||
  //       (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
  //        std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
  //     const std::string msg = "there are some obstacles closed to ego car" 
  //           " which boundary.min_t and min_s less than  1e-2 ,STOP! , id = "+ boundary->id();
  //     if (boundary->IsVirtualObstacleBoundary()) continue;
  //     AWARN_IF(FLAGS_enable_debug_motion)<< msg;
  //     return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  //   }
  // }

  cruise_.clear();
  reference_dp_speed_points_ = *speed_data;

  // reset spline generator
  spline_solver_->Reset(t_knots_,
                        qp_st_speed_config_.qp_spline_config().spline_order());
  
  double start_timestamp = acu::common::NodeTime::Now().ToSecond();
  if (!AddConstraint(st_graph_data_.init_point(), st_graph_data_.speed_limit(),
                     st_graph_data_.st_boundaries(), accel_bound)
      .ok()) {
    const std::string msg = "Add constraint failed!";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }
  //time log
  double end_timestamp_1 = acu::common::NodeTime::Now().ToSecond();
  double time_diff_ms = (end_timestamp_1 - start_timestamp) * 1000;
  auto time_latency = latency_infos_->add_task_stats();
  time_latency->set_name("QpSplineStGraph-AddConstraint"); 
  time_latency->set_time_ms(time_diff_ms);

  if (!AddKernel(st_graph_data_.st_boundaries(), st_graph_data_.speed_limit())
      .ok()) {
    const std::string msg = "Add kernel failed!";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  double end_timestamp_2 = acu::common::NodeTime::Now().ToSecond();
  time_diff_ms = (end_timestamp_2 - end_timestamp_1) * 1000;
  auto time_latency_1 = latency_infos_->add_task_stats();
  time_latency_1->set_name("QpSplineStGraph-AddKernel"); 
  time_latency_1->set_time_ms(time_diff_ms);

  if (!Solve().ok()) {
    auto end = acu::common::NodeTime::Now().ToSecond();
    time_diff_ms = (end - end_timestamp_2) * 1000;
    auto time_latency = latency_infos_->add_task_stats();
    time_latency->set_name("QpSplineStGraph-Solve"); 
    time_latency->set_time_ms(time_diff_ms);
    const std::string msg = "Solve qp problem failed!";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }
  auto end = acu::common::NodeTime::Now().ToSecond();
  time_diff_ms = (end - end_timestamp_2) * 1000;
  auto time_latency_2 = latency_infos_->add_task_stats();
  time_latency_2->set_name("QpSplineStGraph-Solve"); 
  time_latency_2->set_time_ms(time_diff_ms);

  // extract output
  speed_data->clear();
  const math::Spline1d& spline = spline_solver_->spline();

  const double t_output_resolution = FLAGS_trajectory_time_min_interval;//0.02
  double time = 0.0;
  const double kEpsilon = 1e-6;
  while (time <= qp_st_speed_config_.total_time() + kEpsilon ) {
    double s = spline(time);
    double v = std::max(0.0, spline.Derivative(time));
    double a = spline.SecondOrderDerivative(time);
    double da = spline.ThirdOrderDerivative(time);
    AWARN_IF(FLAGS_enable_debug_speedplan)<<"t = "<<time<<", s = "<<s<<", v = "<<v<<", a = "<<a<<", da = "<<da;
    speed_data->AppendSpeedPoint(s, time, v, a, da);
    time += t_output_resolution;
  }
  return Status::OK();
}

void QpSplineStGraph::GetYieldFollowAndCruiseWeightFactor(double& yield_weight_factor, 
      double& follow_weight_factor, double& cruise_weight_factor) const {
  if (yield_st_map_.empty() && follow_st_map_.empty() && follow_vt_map_.empty()) {
    return ;
  }
  if (!yield_st_map_.empty()) {
    if (yield_st_map_.size() != t_evaluated_.size()) {
      AWARN_IF(FLAGS_enable_debug_motion)<<"yield_st_map_ size is not equal to t_evaluated_ size";
      return ;
    }
    double yield_stop_distance = 2.0;//让行距离保持在2m
    bool is_distance_not_enough = false;
    std::multiset<std::pair<double, double>, compare_func> ego_t_a_pairs;
    for (const double curr_t : t_evaluated_) {
      double s_upper = 150.0;
      int t = floor(curr_t / 0.2);
      for (const auto& st_point:yield_st_map_.at(t)) {
        if (st_point.second >= 0.8 && st_point.first < s_upper) {
          double s_upper_yield = st_point.first - yield_stop_distance;
                    
          s_upper = st_point.first;
          if (s_upper_yield <= 2) {
            AWARN_IF(FLAGS_enable_debug_motion)<<"s_upper_yield <= 2 , yield distance is not enough ";
            is_distance_not_enough = true ;
            break;
          } else {
            AINFO_IF(FLAGS_enable_debug_motion)<<"yield t = "<<curr_t<<",  s = "<<s_upper_yield;
            double time = curr_t + FLAGS_numerical_epsilon; 
            double accel = 2 * (s_upper_yield - init_point_.v() * time) / time / time;
            ego_t_a_pairs.insert(std::make_pair(curr_t, accel));
          }
        }
      }
    }
    if (is_distance_not_enough) {
		AWARN_IF(FLAGS_enable_debug_motion)<<"need emergency decel control!!";
        yield_weight_factor = 10;
        cruise_weight_factor = 0.5;
    } else if (!ego_t_a_pairs.empty()) {
      auto front_pair = *(ego_t_a_pairs.begin());
      AWARN_IF(FLAGS_enable_debug_motion)<<"yield accel = "<<front_pair.second;
      if (front_pair.second < 0) {
        double acel_abs = fabs(front_pair.second);
        cruise_weight_factor = 0.7;
        yield_weight_factor = std::fmin(25, 2*acel_abs);
      } else {
        //TODO keep factor = 1.0;
        yield_weight_factor = 0.0;
        cruise_weight_factor = 1.0;
        
      }
      AWARN_IF(FLAGS_enable_debug_motion)<<"yield_weight_factor = "
        <<yield_weight_factor<<",cruise_weight_factor = "<<cruise_weight_factor;
    }
  }

  //calculate follow weight
  bool is_follow_dis_not_enough = false;
  std::multiset<double> follow_ttcs;
  double drag_decel = 0.25;//松油门、刹车后自然阻力的减速度
  double slam_decel = 1.0;
  for (const auto* const_obstacle : path_object_decision_->obstacles().Items()) {
    if (!const_obstacle->HasLongitudinalDecision()) {
      continue;
    }
    
    const auto& decision = const_obstacle->LongitudinalDecision();
    if (decision.has_follow()) {
      double follow_dis = std::fmax(
          const_obstacle->PerceptionSLBoundary().start_s() - EgoInfo::instance()->vehicle_front_edge_to_center() , 0.0);
      double ttc =  FLAGS_numerical_epsilon + follow_dis / 
                       fabs(init_point_.v() - const_obstacle->speed() + FLAGS_numerical_epsilon);                  
      follow_ttcs.insert(ttc);
  }

  if (is_follow_dis_not_enough) {
    follow_weight_factor = 3;
    cruise_weight_factor = std::fmin(0.5, cruise_weight_factor);
  } else if (!follow_ttcs.empty()) {
    double min_ttc = *(follow_ttcs.begin());
    follow_weight_factor = 2 / (1 + std::exp(-1 / min_ttc));
    // cruise_weight_factor = std::fmin(0.5, cruise_weight_factor);
    cruise_weight_factor = std::fmin(1.0, cruise_weight_factor);
    AWARN_IF(FLAGS_enable_debug_motion)<<"follow_weight_factor = "
      <<follow_weight_factor<<",cruise_weight_factor = "<<cruise_weight_factor
      <<", min_ttc = "<<min_ttc;
  }
}
}

Status QpSplineStGraph::AddConstraint(
  const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
  const std::vector<const StBoundary*>& boundaries,
  const std::pair<double, double>& accel_bound) {
  math::Spline1dConstraint* constraint = spline_solver_->mutable_spline_constraint();

  if (!constraint->AddPointConstraint(0.0, 0.0)) {//规划起点限制
    const std::string msg = "add st start point constraint failed";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  if (!constraint->AddPointDerivativeConstraint(0.0, init_point_.v())) {//起点速度限制，即一阶导限制
    const std::string msg = "add st start point velocity constraint failed!";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  // monotone constraint
  if (!constraint->AddMonotoneInequalityConstraint(t_evaluated_)) {//单调
    const std::string msg = "add monotone inequality constraint failed!";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  // smoothness constraint
  if (!constraint->AddThirdDerivativeSmoothConstraint()) {//三阶导即da限制
    const std::string msg = "add smoothness joint constraint failed!";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  // boundary constraint
  std::vector<double> s_upper_bound;
  std::vector<double> s_lower_bound;

  for (const double curr_t : t_evaluated_) {
    double lower_s = 0.0;
    double upper_s = st_graph_data_.path_data_length();
    if (FLAGS_enable_plan_based_on_stmap && 
      (!is_change_lane_ || is_change_lane_ && !BehaviorParser::instance()->has_decision_lateral_behavior())) {
      GetSConstraintByTime(curr_t,st_graph_data_.path_data_length(), 
                           &upper_s,&lower_s,boundaries);
    } else {
      GetSConstraintByTime(boundaries, curr_t,
                         st_graph_data_.path_data_length(), &upper_s,
                         &lower_s);
    }
    s_upper_bound.push_back(upper_s);
    s_lower_bound.push_back(lower_s);
  }

  if (st_graph_debug_) {
    auto s_constraint = st_graph_debug_->mutable_s_constraint();
    std::cout<<"set s_constraint==========================="<<std::endl;
    for (size_t i = 0; i < t_evaluated_.size(); ++i) {
      s_constraint->add_t(t_evaluated_[i]);
      s_constraint->add_lower_bound(s_lower_bound[i]);
      s_constraint->add_upper_bound(s_upper_bound[i]);
    }
  }

  DCHECK_EQ(t_evaluated_.size(), s_lower_bound.size());
  DCHECK_EQ(t_evaluated_.size(), s_upper_bound.size());
  if (!constraint->AddBoundary(t_evaluated_, s_lower_bound, s_upper_bound)) {
    const std::string msg = "Fail to apply distance constraints.";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  // speed constraint
  std::vector<double> speed_upper_bound;
  std::vector<std::pair<double, double>> speed_limit_points= speed_limit.speed_limit_points();

  if (!EstimateSpeedUpperBound(init_point, speed_limit_points, &speed_upper_bound).ok()) {
    std::string msg = "Fail to estimate speed upper constraints.";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  reference_speed_points_ = speed_upper_bound;
  std::vector<double> speed_lower_bound(t_evaluated_.size(), 0.0);

  DCHECK_EQ(t_evaluated_.size(), speed_upper_bound.size());
  DCHECK_EQ(t_evaluated_.size(), speed_lower_bound.size());

  for (size_t i = 0; i < t_evaluated_.size(); ++i) {
    AWARN_IF(FLAGS_enable_debug_speedplan)<<"v limit = ("<<speed_lower_bound.at(i)<<", "<<speed_upper_bound.at(i)<<").";
  }
  size_t t_evaluated_size = t_evaluated_.size();
  if (st_graph_debug_) {
    auto speed_constraint = st_graph_debug_->mutable_speed_constraint();

    for (size_t i = 0; i < t_evaluated_size; ++i) {
      speed_constraint->add_t(t_evaluated_[i]);
      speed_constraint->add_lower_bound(speed_lower_bound[i]);
      speed_constraint->add_upper_bound(speed_upper_bound[i]);
    }
  }

  if (!constraint->AddDerivativeBoundary(t_evaluated_, speed_lower_bound,
                                         speed_upper_bound)) {
    const std::string msg = "Fail to apply speed constraints.";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  // acceleration constraint
  std::vector<double> accel_lower_bound(t_evaluated_.size(), accel_bound.first);
  std::vector<double> accel_upper_bound(t_evaluated_.size(),
                                        accel_bound.second);

  bool has_follow = false;
  double delta_s = 1.0;
  for (const auto* boundary : boundaries) {
    if (boundary->boundary_type() == StBoundary::BoundaryType::FOLLOW) {
      has_follow = true;
      delta_s = std::fmin(
                  delta_s, boundary->min_s() - fabs(boundary->characteristic_length()));
    }
  }
  if (FLAGS_enable_follow_accel_constraint && has_follow && delta_s < 0.0) {
    accel_upper_bound.front() = 0.0;
  }

  if (st_graph_debug_) {
    auto a_constraint = st_graph_debug_->mutable_a_constraint();

    for (size_t i = 0; i < t_evaluated_size; ++i) {
      a_constraint->add_t(t_evaluated_[i]);
      a_constraint->add_lower_bound(accel_lower_bound[i]);
      a_constraint->add_upper_bound(accel_upper_bound[i]);
    }
  }

  DCHECK_EQ(t_evaluated_.size(), accel_lower_bound.size());
  DCHECK_EQ(t_evaluated_.size(), accel_upper_bound.size());
  if (!constraint->AddSecondDerivativeBoundary(t_evaluated_, accel_lower_bound,
      accel_upper_bound)) {
    const std::string msg = "Fail to apply acceleration constraints.";
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  return Status::OK();
}

Status QpSplineStGraph::AddKernel(
  const std::vector<const StBoundary*>& boundaries,
  const SpeedLimit& speed_limit) {
  math::Spline1dKernel* spline_kernel = spline_solver_->mutable_spline_kernel();


  double yield_weight_factor = 1.0;
  double follow_weight_factor = 1.0;
  double cruise_weight_factor = 1.0;

  if (FLAGS_use_dynamic_weight) {
    // GetYieldFollowAndCruiseWeightFactor(yield_weight_factor, 
    //                    follow_weight_factor, cruise_weight_factor);
    //add debug infos
    auto DP_ptr = acu::planning::DataPool::Instance()->GetMainDataPtr();
    DP_ptr->debug_planning_msg.motionplan_debug.set_yield_weight_factor(yield_weight_factor);
    DP_ptr->debug_planning_msg.motionplan_debug.set_cruise_weight_factor(cruise_weight_factor);
    DP_ptr->debug_planning_msg.motionplan_debug.set_follow_weight_factor(follow_weight_factor);
  }

  if (qp_st_speed_config_.qp_spline_config().accel_kernel_weight() > 0) {
    spline_kernel->AddSecondOrderDerivativeMatrix(
      qp_st_speed_config_.qp_spline_config().accel_kernel_weight());
  }

  if (qp_st_speed_config_.qp_spline_config().jerk_kernel_weight() > 0) {
    spline_kernel->AddThirdOrderDerivativeMatrix(
      qp_st_speed_config_.qp_spline_config().jerk_kernel_weight());
  }

  if (!AddCruiseReferenceLineKernel(
        qp_st_speed_config_.qp_spline_config().cruise_weight()*cruise_weight_factor)
      .ok()) {
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, "QpSplineStGraph::AddKernel");
  }

  if (FLAGS_enable_plan_based_on_stmap && 
      (!is_change_lane_ || is_change_lane_ && !BehaviorParser::instance()->has_decision_lateral_behavior())) {
    if (!AddFollowAndYieldStMapKernel(qp_st_speed_config_.qp_spline_config().follow_weight() * follow_weight_factor,
          qp_st_speed_config_.qp_spline_config().follow_v_weight()).ok()){
      return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, "QpSplineStGraph::AddKernel");
    }
  } else {
    if (!AddFollowAndYieldReferenceLineKernel(boundaries, qp_st_speed_config_.qp_spline_config().follow_weight() * follow_weight_factor,
              qp_st_speed_config_.qp_spline_config().follow_v_weight()).ok()){
      return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, "QpSplineStGraph::AddKernel");
    }
  }

  if (!FLAGS_enable_decision_replace_dp_speed_optimizer) {
    if (!AddDpStReferenceKernel(
          qp_st_speed_config_.qp_spline_config().dp_st_reference_weight())) {
      return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, "QpSplineStGraph::AddKernel");
    }
  }
  
  if (FLAGS_use_speed_limit_as_kernel && !AddPiecewiseJerkVtReferenceKernel(
        qp_st_speed_config_.qp_spline_config().speed_kernel_weight()).ok()) { // TBD
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, "QpSplineStGraph::AddKernel");
  }

  // init point jerk continuous kernel
  (*spline_kernel->mutable_kernel_matrix())(2, 2) +=
    2.0 * 4.0 *
    qp_st_speed_config_.qp_spline_config().init_jerk_kernel_weight();
  (*spline_kernel->mutable_offset())(2, 0) +=
    -4.0 * init_point_.a() *
    qp_st_speed_config_.qp_spline_config().init_jerk_kernel_weight();

  spline_kernel->AddRegularization(
    qp_st_speed_config_.qp_spline_config().regularization_weight());

  return Status::OK();
}

Status QpSplineStGraph::Solve() {
  return spline_solver_->Solve()
         ? Status::OK()
         : Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, "QpSplineStGraph::solve");
}

Status QpSplineStGraph::AddCruiseReferenceLineKernel(const double weight) {
  auto* spline_kernel = spline_solver_->mutable_spline_kernel();
  
  uint32_t t_evaluated_size = t_evaluated_.size();
  const double init_v = st_graph_data_.init_point().v();
  // 10km/h以下车速提速加快点,增大cruise_weight权重(0.05, 0.5)
  double cruise_wight =  weight;
  const double theta = init_point_.path_point().theta();
  const double vehicle_theta= EgoInfo::instance()->vehicle_state_new().heading;
  AWARN_IF(FLAGS_enable_debug_motion)<<"init point s = " << init_point_.path_point().s()<< 
            ", init point theta = " << theta << "; vehicle_theta = " << vehicle_theta;
  // if (/*无横向行为场景，同时path_in_current*/) {
  if (BehaviorParser::instance()->is_local_path_in_current()){
    cruise_wight = std::max(0.05, (10.0-init_v*3.6)*weight);
    cruise_wight = std::min(cruise_wight, 0.5);
  }
  double dist_ref = qp_st_speed_config_.total_path_length();
  for (uint32_t i = 0; i < t_evaluated_size; ++i) {
    dist_ref += reference_speed_points_[i]*t_evaluated_resolution_;
    cruise_.push_back(dist_ref);
  }
  // if (st_graph_debug_) {
  //   auto kernel_cruise_ref = st_graph_debug_->mutable_kernel_cruise_ref();
  //   kernel_cruise_ref->mutable_t()->Add(t_evaluated_[0]);
  //   kernel_cruise_ref->mutable_cruise_line_s()->Add(dist_ref);
  //   for (uint32_t i = 1; i < t_evaluated_.size(); ++i) {
  //     kernel_cruise_ref->mutable_t()->Add(t_evaluated_[i]);
  //     kernel_cruise_ref->mutable_cruise_line_s()->Add(cruise_[i]);
  //   }
  // }
  DCHECK_EQ(t_evaluated_.size(), cruise_.size());

  if (t_evaluated_.size() > 0) {
    spline_kernel->AddReferenceLineKernelMatrix(
      t_evaluated_, cruise_, cruise_wight * qp_st_speed_config_.total_time() /
      static_cast<double>(t_evaluated_.size()));//[0,0.02~8.0], 150, weight*40
  }

  return Status::OK();
}

Status QpSplineStGraph::AddFollowAndYieldStMapKernel(const double s_weight,const double v_weight){
  AWARN_IF(FLAGS_enable_debug_motion)<<"AddFollowAndYieldStMapKernel.";
  if ((follow_st_map_.empty() || follow_st_map_.size() != t_evaluated_.size()) 
        && (yield_st_map_.empty() || yield_st_map_.size()!= t_evaluated_.size())
        &&(speed_limit_st_map_.empty()||speed_limit_st_map_.size()!=t_evaluated_.size())) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"No follow and yield st_map.";
    return Status::OK();
  }

  if ((follow_vt_map_.empty() || follow_vt_map_.size() != t_evaluated_.size()) 
        && (yield_vt_map_.empty() || yield_vt_map_.size()!= t_evaluated_.size())
        &&(speed_limit_vt_map_.empty()||speed_limit_vt_map_.size()!=t_evaluated_.size())) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"No follow and yield vt_map";
    return Status::OK();
  }

  auto* spline_kernel = spline_solver_->mutable_spline_kernel();
  std::vector<double> ref_s;
  std::vector<double> ref_v;
  std::vector<double> filtered_evaluate_t;
  std::vector<double> dynamic_s_weight;
  std::vector<double> dynamic_v_weight;
  size_t t_evaluated_size = t_evaluated_.size();

  const double follow_probability = 0.75;
  const double yeild_high_probability = 0.8;
  const double init_v = st_graph_data_.init_point().v();
  size_t last_index_in_stmap = t_evaluated_size;
  double last_s_in_stmap = std::numeric_limits<double>::infinity();
  double last_v_in_stmap = std::numeric_limits<double>::infinity();
  double v_slope = std::numeric_limits<double>::infinity();


  for (size_t i = 0; i < t_evaluated_size; ++i) {
    const double curr_t = t_evaluated_[i];
    double follow_distance_s = FLAGS_follow_min_distance;
    double s_min = std::numeric_limits<double>::infinity();
    double v_min = std::numeric_limits<double>::infinity();

    bool find_mins_success = false;
    bool find_minv_success = false;
    double yeild_distance_s = FLAGS_yield_distance;
    // 跟车的参考速度
    if (!follow_vt_map_.empty() && !follow_st_map_.empty()) {
      for (const auto& vt : follow_vt_map_.at(i)) {
        if (vt.second >= follow_probability) {
          find_minv_success = true;
          v_min = std::fmin(v_min, vt.first);
        }
      }
      //跟车对应参考距离
      if (find_minv_success) {
        // double v_min_follow = std::fmin (v_min, init_v);//障碍物、自车速度取小的值
        // init_v > v_min 自车速度比跟车速度大时，0 < ratio < 0.8 
        // init_v < v_min ratio = 0
        double ratio =std::max(0.0, 0.8*(init_v - v_min) / (init_v +FLAGS_numerical_epsilon)); 
        double v_min_follow = ratio*v_min + (1-ratio)*init_v;// 
        if (v_min < -0.5 ) {
          AWARN_IF(FLAGS_enable_debug_motion)<<"Reverse scenario!!!";
          v_min_follow = std::fmax(std::fabs(v_min), init_v);//目标车对向
        } 
        follow_distance_s = std::fmax(v_min_follow * FLAGS_follow_time_buffer, follow_distance_s);
      }
      
      for (const auto& st : follow_st_map_.at(i)) {
        if (st.second >= follow_probability) {//概率比较
          find_mins_success = true;
          // 基于障碍物的跟随距离
          double reference_follow_dis = st.first - follow_distance_s - std::fmax(FLAGS_overall_system_delay*init_v, 2.0);
          if (v_min > 1.2*init_v) {// 目标车速自车高，至少保持让行参考距离就行
            s_min = std::fmin(s_min, std::fmax(std::min(init_v*curr_t, st.first - FLAGS_follow_min_distance), reference_follow_dis));
          } else if (v_min > -0.5){
            // double a = -1.5;// 低速车切入时，以-1.5减速度算参考s
            double a = -0.6*(init_v*init_v - v_min*v_min)/st.first;
            a = std::min(-1.0, a);
            s_min = std::fmin(s_min, std::fmax(std::min(init_v*curr_t + 0.5*a*curr_t*curr_t, st.first - FLAGS_follow_min_distance), 
                  reference_follow_dis));
          } else {
            s_min = std::fmin(s_min, reference_follow_dis);
          }
        }
      }
    }
   
    if (!yield_vt_map_.empty() && !yield_st_map_.empty()) {
      //让行的参考速度
      //让行的参考距离
      for (const auto& vt : yield_vt_map_.at(i)) {
        if (vt.second >= yeild_high_probability) {
          find_minv_success = true;
          v_min = std::fmin(v_min, vt.first);
        }
      }

      for (const auto& st : yield_st_map_.at(i)) {
        if (st.second >= yeild_high_probability) {//概率比较
          find_mins_success = true;
          if (v_min < 0) {
            yeild_distance_s = yeild_distance_s - v_min * 0.3;//让行对向车逆向的场景，让行距离加大
          }
          double s_yield_min = st.first - yeild_distance_s 
                  - std::fmax(FLAGS_overall_system_delay*init_v, 2.0);
          double v_yeild_min = std::fmax(0.0, 2*s_yield_min/(curr_t+FLAGS_numerical_epsilon) - init_v);
          if (s_yield_min < s_min) {
            s_min = std::max(0.0, s_yield_min);
            v_min = std::fmin(v_min, v_yeild_min);
          }
        }
      }
    }

    //决策限速场景
    if (!speed_limit_vt_map_.empty()&&!speed_limit_st_map_.empty()) {
      for (const auto& st : speed_limit_st_map_.at(i)) {
        if (st.second >= 0.7) {
          s_min = std::min(s_min, st.first - FLAGS_yield_distance - std::fmax(FLAGS_overall_system_delay*init_v, 2.0));
          find_mins_success = true;
        }
      }
      for (const auto& vt : speed_limit_vt_map_.at(i)) {
        if (vt.second >= 0.7) {
          find_minv_success = true;
          v_min = std::fmin(v_min, 0.9*vt.first);//限速决策比前车速度还低一点
        }
      }
    }
    if (s_min < 0) s_min = 0;
    if (v_min < 0) v_min = 0;

    //v_min:TBD move to conf
    if (find_mins_success && find_minv_success && s_min < cruise_[i] && v_min < 30) {
      if (v_min > st_graph_data_.cruise_speed()) continue;
      if (i == 0) {
        filtered_evaluate_t.push_back(curr_t);
        ref_s.push_back(s_min);
        ref_v.push_back(v_min);
        dynamic_s_weight.push_back(s_weight);
        dynamic_v_weight.push_back(v_weight);
      }

      // st图中s_min/curr_t对应的v的斜率，取最小即对应最小需求速度
      if (i > 0 && s_min/curr_t <=v_slope) {
        last_index_in_stmap = i;
        v_slope = s_min/curr_t;
        last_s_in_stmap = s_min;
        last_v_in_stmap = v_min;
        filtered_evaluate_t.push_back(curr_t);
        ref_s.push_back(s_min);
        ref_v.push_back(v_min);
        dynamic_s_weight.push_back(s_weight);
        dynamic_v_weight.push_back(v_weight);
      }
    }
  }

  for (size_t i = last_index_in_stmap+1; i < t_evaluated_size; ++i) {
    const double curr_t = t_evaluated_[i];
    last_s_in_stmap += last_v_in_stmap*t_evaluated_resolution_;
    filtered_evaluate_t.push_back(curr_t);
    ref_s.push_back(last_s_in_stmap);
    ref_v.push_back(last_v_in_stmap);
    dynamic_s_weight.push_back(s_weight);
    dynamic_v_weight.push_back(v_weight);
  }
  if (!ref_s.empty()) {
    for (size_t i = 0; i < t_evaluated_size; ++i) {
      AWARN_IF(FLAGS_enable_debug_speedplan)<<"follow and yield t = "<<filtered_evaluate_t[i]<<", s_min = "
          <<ref_s[i] << ", v_min = " << ref_v[i];
    }
  }

  DCHECK_EQ(filtered_evaluate_t.size(), ref_s.size());

  if (!ref_s.empty()) {
    spline_kernel->AddReferenceLineKernelMatrix(
      filtered_evaluate_t, ref_s,dynamic_s_weight);
  }
  if (!ref_v.empty()) {
    spline_kernel->AddReferenceDxKernelMatrix(
      filtered_evaluate_t, ref_v,dynamic_v_weight);
  }

  return Status::OK();
}


Status QpSplineStGraph::AddFollowAndYieldReferenceLineKernel(
  const std::vector<const StBoundary*>& boundaries, 
  const double s_weight,const double v_weight) {
  auto* spline_kernel = spline_solver_->mutable_spline_kernel();
  std::vector<double> ref_s;
  std::vector<double> ref_v;
  std::vector<double> filtered_evaluate_t;
  size_t t_evaluated_size = t_evaluated_.size();
  for (size_t i = 0; i < t_evaluated_size; ++i) {
    const double curr_t = t_evaluated_[i];
    double s_min = std::numeric_limits<double>::infinity();
    double v_min = std::numeric_limits<double>::infinity();
    bool success = false;
    for (const auto* boundary : boundaries) {
      if (boundary->boundary_type() != StBoundary::BoundaryType::FOLLOW) {
        continue;
      }
      if (curr_t < boundary->min_t() || curr_t > boundary->max_t()) {
        continue;
      }
      double s_upper = 0.0;
      double s_lower = 0.0;
      if (boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
        success = true;
        s_min = std::min(
                  s_min,
                  s_upper - boundary->characteristic_length() -
                  std::fmax(FLAGS_overall_system_delay*st_graph_data_.init_point().v(), 2.0));
        v_min = boundary->speed();
      }
    }

    for (const auto* boundary : boundaries) {
      if (boundary->boundary_type() != StBoundary::BoundaryType::YIELD) {
        continue;
      }
      if (curr_t < boundary->min_t() || curr_t > boundary->max_t()) {
        continue;
      }
      double s_upper = 0.0;
      double s_lower = 0.0;
      if (boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
        success = true;
        s_min = std::fmin(s_min, s_upper - FLAGS_yield_distance);
        v_min = std::fmin(boundary->STspeed(), v_min);
        if (v_min < 0) v_min = 0;
      }
    }

    //v_min:TBD move to conf
    if (success && s_min < cruise_[i] && v_min < 30) {
      //目标车速度比自车快，自车以最大加速度1s追都赶不上时
      double lagloop_dis  = (v_min - st_graph_data_.init_point().v()) * 1.0  
        + 0.5 * qp_st_speed_config_.preferred_max_acceleration() * 4;
      if(s_min > lagloop_dis && v_min > st_graph_data_.init_point().v() + 1.0){
        continue;
      }
      double very_comfort_decel = -0.25;
      double very_comfort_decel_dis = 0.5 * (st_graph_data_.init_point().v() *
             st_graph_data_.init_point().v() - v_min * v_min) / fabs(very_comfort_decel);
      if(s_min > very_comfort_decel_dis + 10 && v_min < st_graph_data_.init_point().v()){
        continue;
      }
      // AERROR << "curr_t = " << curr_t << " ,s_min = " << s_min << ", v_min = " << v_min;
      filtered_evaluate_t.push_back(curr_t);
      ref_s.push_back(s_min);
      ref_v.push_back(v_min);
      AWARN_IF(FLAGS_enable_debug_speedplan)<<"follow and yield t = "<<curr_t<<", s_min = "<<s_min << ", v_min = " << v_min;
      // if (st_graph_debug_) {
      //   auto kernel_follow_ref = st_graph_debug_->mutable_kernel_follow_ref();
      //   kernel_follow_ref->mutable_t()->Add(curr_t);
      //   kernel_follow_ref->mutable_follow_line_s()->Add(s_min);
      // }
    }
  }
  DCHECK_EQ(filtered_evaluate_t.size(), ref_s.size());

  if (!ref_s.empty()) {
    spline_kernel->AddReferenceLineKernelMatrix(
      filtered_evaluate_t, ref_s,
      s_weight * qp_st_speed_config_.total_time() /
      static_cast<double>(t_evaluated_.size()));
  }

  if (!ref_v.empty()) {
    spline_kernel->AddReferenceDxKernelMatrix(
      filtered_evaluate_t, ref_v,v_weight);
      // weight * qp_st_speed_config_.total_time() /
      // static_cast<double>(t_evaluated_.size()));
  }

  return Status::OK();
}

Status QpSplineStGraph::AddOvertakeReferenceLineKernel(
  const std::vector<const StBoundary*>& boundaries, const double weight) {
  auto* spline_kernel = spline_solver_->mutable_spline_kernel();
  std::vector<double> ref_s;
  std::vector<double> filtered_evaluate_t;
  size_t t_evaluated_size = t_evaluated_.size();
  for (size_t i = 0; i < t_evaluated_size; ++i) {
    const double curr_t = t_evaluated_[i];
    double s_max = 0.0;
    bool success = false;
    for (const auto* boundary : boundaries) {
      if (boundary->boundary_type() != StBoundary::BoundaryType::OVERTAKE) {
        continue;
      }
      if (curr_t < boundary->min_t() || curr_t > boundary->max_t()) {
        continue;
      }
      double s_upper = 0.0;
      double s_lower = 0.0;
      if (boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
        success = true;
        s_max = std::max(s_max,s_lower);
      }
    }
    if (success && s_max < cruise_[i]) {
      filtered_evaluate_t.push_back(curr_t);
      ref_s.push_back(s_max);
    }
  }
  DCHECK_EQ(filtered_evaluate_t.size(), ref_s.size());

  if (!ref_s.empty()) {
    spline_kernel->AddReferenceLineKernelMatrix(
      filtered_evaluate_t, ref_s,
      weight * qp_st_speed_config_.total_time() /
      static_cast<double>(t_evaluated_.size()));
  }
  
  return Status::OK();
}

bool QpSplineStGraph::AddDpStReferenceKernel(const double weight) const {
  std::vector<double> t_pos;
  std::vector<double> s_pos;
  for (auto point : reference_dp_speed_points_) {
    t_pos.push_back(point.t());
    s_pos.push_back(point.s());
  }
  auto* spline_kernel = spline_solver_->mutable_spline_kernel();
  if (!t_pos.empty()) {
    spline_kernel->AddReferenceLineKernelMatrix(
      t_pos, s_pos, weight * qp_st_speed_config_.total_time() /
      static_cast<double>(t_pos.size()));
  }
  return true;
}

Status QpSplineStGraph::AddPiecewiseJerkVtReferenceKernel(const double weight) {
  auto* spline_kernel = spline_solver_->mutable_spline_kernel();
  if ( !reference_speed_points_.empty() ) {
    if(spline_kernel->AddReferenceDxKernelMatrix(
      t_evaluated_, reference_speed_points_, weight)){
      return Status::OK();
    }else{
      return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, "AddReferenceDxKernelMatrix fail");
    }
  }
  return Status::OK();
}

Status QpSplineStGraph::GetSConstraintByTime(
  const std::vector<const StBoundary*>& boundaries, const double time,
  const double total_path_s, double* const s_upper_bound,
  double* const s_lower_bound) const {
  const double kEpsilon = 1e-6;
  for (const StBoundary* boundary : boundaries) {
    double s_upper = total_path_s;
    double s_lower = 0.0;

    if (!boundary->GetUnblockSRange(time, &s_upper, &s_lower)) {
      continue;
    }

    if (boundary->boundary_type() == StBoundary::BoundaryType::FOLLOW) {
      *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
      if (boundary->min_t() > 0.5) {
        double s_buffer = 3.0;
        if (boundary->min_s() - 
          std::fmax(init_point_.v(), st_graph_data_.cruise_speed()) * boundary->min_t() >= 
          s_buffer) {
          s_buffer = 0;
        }
        *s_upper_bound = std::fmin(*s_upper_bound, s_upper - s_buffer);
      }
    } else if (boundary->boundary_type() == StBoundary::BoundaryType::STOP) {
      *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
    } else if (boundary->boundary_type() ==
               StBoundary::BoundaryType::OVERTAKE) {
      if (time + kEpsilon < boundary->min_t() || time > boundary->max_t() + kEpsilon) {
        *s_lower_bound = std::fmax(*s_lower_bound, s_lower);
      } else {
        *s_lower_bound = std::fmax(*s_lower_bound, s_lower + fabs(boundary->characteristic_length()));
      }
      
    } else if (boundary->boundary_type() == StBoundary::BoundaryType::YIELD) {//@pqg add 
      if (!FLAGS_enable_yield_kernel) {
        *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
      }
      
    } else {
      // AWARN << "Unhandled boundary type: "
      //       << StBoundary::TypeName(boundary->StBoundary_type());
    }
  }

  return Status::OK();
}


Status QpSplineStGraph::GetSConstraintByTime(const double time,
  const double total_path_s, double* const s_upper_bound,
  double* const s_lower_bound, const std::vector<const StBoundary*>& boundaries) const {
  const double kEpsilon = 1e-6;
  int t = floor(time / 0.2);
  bool has_stop_s = false;
  for (const StBoundary* boundary : boundaries) {//1. stop 
    double s_upper = total_path_s;
    double s_lower = 0.0;

    if (!boundary->GetUnblockSRange(time, &s_upper, &s_lower)) {
      continue;
    }

    if (boundary->boundary_type() == StBoundary::BoundaryType::STOP) {
      *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
      has_stop_s = true;
    }
  }

  //让行，保证冲突点不碰撞。并不是速度一定要降为0
  if (yield_st_map_.size() && yield_vt_map_.size() && time <= 8.00) {
    double yield_obstacle_s = std::numeric_limits<double>::infinity();
    double yield_obstacle_v = std::numeric_limits<double>::infinity();
    bool need_yield = false;
    for (const auto& st_point:yield_st_map_.at(t)) {
      if (st_point.second >= 0.8) {
        yield_obstacle_s = std::fmin(yield_obstacle_s, st_point.first);//time时刻，对应的让行障碍物的距离 
        need_yield = true;
        //如果有停车点<让行位置点，则不考虑让行了
        if (has_stop_s && *s_upper_bound < yield_obstacle_s -0.5){
          // AWARN_IF(FLAGS_enable_debug_motion)<<"yield_obstacle_s is over stop_s!!!";
          need_yield = false;
        } 
      }
    }
    if (need_yield) {
      double yield_distance = 2.5 + 0.5*st_graph_data_.init_point().v();//让行缩边界距离改为2m，@sly 20230630
      yield_distance = std::fmin(yield_distance, 5.0);//让行距离为2.5~5m
      double s_upper_yield = yield_obstacle_s - yield_distance;      
      if (s_upper_yield <= 0 ) s_upper_yield = 0.0 ;
      *s_upper_bound = std::fmin(*s_upper_bound, s_upper_yield);
      AWARN_IF(FLAGS_enable_debug_motion)<<"t = "<<time<<", yield upper s = "<<*s_upper_bound;
    }
  }

  const double follow_probability = 0.75;
  if (follow_st_map_.size() && time <= 8.00 ) {//3.follow
    for (const auto& st_point:follow_st_map_.at(t)) {
      if (st_point.second >= follow_probability) {
        *s_upper_bound = std::fmin(*s_upper_bound, st_point.first);
      }
    }
    // AWARN_IF(FLAGS_enable_debug_motion)<<"t = "<<time<<", follow upper s = "<<*s_upper_bound;
  }
  if (speed_limit_st_map_.size() && time <= 8.00 ) {
    for (const auto& st_point:speed_limit_st_map_.at(t)) {
    // AINFO<<"*******t:  "<<st_point.first<<" ,first: "<<st_point.first<<" ,second: "<<st_point.second;
      if (st_point.second >= 0.7) {
        *s_upper_bound = std::fmin(*s_upper_bound, st_point.first);
        // AWARN_IF(FLAGS_enable_debug_motion)<<"t = "<<time<<", follow upper s = "<<*s_upper_bound;
      }
    }
  }

  if (overtake_st_map_.size() && time <=8.00 ) {
    for (const auto& st_point:overtake_st_map_.at(t)) {
      if (st_point.second >= FLAGS_p_thd_cost) {
        *s_lower_bound = std::fmax(*s_lower_bound, st_point.first);
      }
    }
    //AWARN_IF(FLAGS_enable_debug_motion)<<"t = "<<time<<", overtake lower s = "<<*s_lower_bound;
  }

  return Status::OK();
}

const SpeedInfo QpSplineStGraph::GetHistorySpeed() const {
  const auto* last_frame = FrameHistory::instance()->Latest();
  if (!last_frame) {
    // AWARN << "last frame is empty";
    return SpeedInfo();
  }
  const ReferenceLineInfo* last_reference_line_info =
    last_frame->DriveReferenceLineInfo();
  if (!last_reference_line_info) {
    ADEBUG << "last reference line info is empty";
    return SpeedInfo();
  }
  return last_reference_line_info->speed_data();
}

Status QpSplineStGraph::EstimateSpeedUpperBound(
  const common::TrajectoryPoint& init_point, 
  const std::vector<std::pair<double, double>>& speed_limit_points,
  std::vector<double>* speed_upper_bound) const {
  DCHECK_NOTNULL(speed_upper_bound);

  if (speed_limit_points.empty()) {
    const std::string msg = "speed_limit_points size is zero!";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  // speed_upper_bound->clear();
  if (speed_limit_points.empty()) {
    std::string msg = "speed_limit_points size is zero !";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  // use v to estimate position: not accurate, but feasible in cyclic
  // processing. We can do the following process multiple times and use
  // previous cycle's results for better estimation.
  const double v = init_point.v();
  auto last_speed_data = GetHistorySpeed();
  size_t t_evaluated_size = t_evaluated_.size();
  AINFO_IF(FLAGS_enable_debug_motion)<<"~~~~~2) Update speedlimit ";
  auto cmp = [](const std::pair<double, double>& p1, const double s) {
    return p1.first < s;
  };

  for (const double t : t_evaluated_) {
    double s = std::max(v, 3.0) * t;//至少24m的限速结果

    const auto& it = std::lower_bound(speed_limit_points.begin(),
                                      speed_limit_points.end(), s, cmp);
    if (it != speed_limit_points.end()) {
      speed_upper_bound->push_back(it->second);
    } else {
      if (!speed_limit_points.empty()) {
        speed_upper_bound->push_back(speed_limit_points.back().second);
      } else {
        speed_upper_bound->push_back(st_graph_data_.cruise_speed());
      }
    }
  }

  return Status::OK();
}

void QpSplineStGraph::GetStMap(PathDecision* const path_decision) {
  follow_st_map_.clear();
  follow_vt_map_.clear();
  overtake_st_map_.clear();
  yield_st_map_.clear();
  yield_vt_map_.clear();
  speed_limit_st_map_.clear();
  speed_limit_vt_map_.clear();
  constexpr double kOvertakeTimeBuffer = 0.5;//3.0;    // in seconds
  constexpr double kMinOvertakeDistance = 2.0;  // in meters
  constexpr double kMaxOvertakeDistance = 10.0;  // in meters
  constexpr double kSidePastTimeThreshold = 0.0;  // 0.5; // in seconds
  const auto& bp_ptr = BehaviorParser::instance();
  std::pair<double, double> u_turn_interval=std::make_pair(0.0,0.0);
  bool is_in_u_turn = bp_ptr->target_reference_line().IsUturn(u_turn_interval);
  std::map<int, double> follow_obstacles;//first:id , second : st_speed
  std::vector<int> speed_limit_obstacles;//id 
  const double stop_s_upper_limit = path_decision->stop_reference_line_s();//停止点上限
  for (const auto& obstacle : bp_ptr->object_decision()) {
    if (obstacle.second == eObjectDecisionEnum::FOLLOW) {
      // follow_obstacles.push_back(obstacle.first);
      const auto* obj = path_decision->Find(std::to_string(obstacle.first));
      double st_speed = 0.0;
      if (obj!= nullptr) {
        if (!obj->LongitudinalDecision().has_follow()) continue;
        if (obj->st_boundary().min_s() > stop_s_upper_limit) continue;
        st_speed = obj->speed();
        follow_obstacles.insert(pair<int, int>(obstacle.first, st_speed));
      }  
    } else if(obstacle.second == eObjectDecisionEnum::SPEED_LIMIT){
        const auto* obj = path_decision->Find(std::to_string(obstacle.first));
        if (obj!= nullptr) {
          if (obj->LongitudinalDecision().has_ignore()) continue;
          if (obj->st_boundary().min_s() > stop_s_upper_limit) continue;
          speed_limit_obstacles.push_back(obstacle.first);
        }
    }
  }
  // pair is <id, overlap_time>
  std::vector<std::pair<int, double>> sidepass_obstacles;
  for (const auto& obstacle : bp_ptr->object_decision()) {
    if (obstacle.second == eObjectDecisionEnum::SIDEPASS) {
      const auto* obj = path_decision->Find(std::to_string(obstacle.first));
      double overlap_time = std::numeric_limits<double>::max();
      if(obj) {
        if (obj->st_boundary().min_s() > stop_s_upper_limit) continue;
        double delta_v = init_point_.v() - obj->speed();
        double ttc = delta_v > 1e-3 ? (obj->PerceptionSLBoundary().start_s() 
          - vehicle_param_.front_over_hang) / delta_v : std::numeric_limits<double>::max();
        double ttc_by_last_speed_data = obj->STUpperLowerPoints().HasPoint() ?
              obj->STUpperLowerPoints().lower_points.front().t() : ttc;
        overlap_time = ttc_by_last_speed_data > kSidePastTimeThreshold ?
          ttc_by_last_speed_data : std::numeric_limits<double>::max();
        AWARN_IF(FLAGS_enable_debug_motion)<<"ttc "<<ttc<<" init_point_v "<<init_point_.v()<<" obj_v "<<obj->speed()
          <<" has_point? "<<(int)obj->STUpperLowerPoints().HasPoint();
      }
      AWARN_IF(FLAGS_enable_debug_motion)<<"sidepass id = "<<obstacle.first<<" overlap_time "<<overlap_time;
      sidepass_obstacles.push_back(std::make_pair(obstacle.first, overlap_time));
    }
  }
  std::vector<int> ovetake_obstacles;
  for (const auto& obstacle : bp_ptr->object_decision()) {
    if (obstacle.second == eObjectDecisionEnum::TAKEWAY) {
      const auto* obj = path_decision->Find(std::to_string(obstacle.first));
      if (obj!= nullptr) {
        if (obj->st_boundary().min_s() > stop_s_upper_limit) continue;
        ovetake_obstacles.push_back(obstacle.first);
      }
    }
  }
  //std::vector<int> yield_obstacles;
  std::vector<std::pair<int, double>> yield_obstacles;
  for (const auto& obstacle : bp_ptr->object_decision()) {
    if (obstacle.second == eObjectDecisionEnum::GIVEWAY) {
      const auto* obj = path_decision->Find(std::to_string(obstacle.first));
      if (obj!= nullptr && !obj->LongitudinalDecision().has_yield()) continue;
      double st_speed = 0.0;
      if (obj!= nullptr) {
        if (obj->st_boundary().min_s() > stop_s_upper_limit) continue;
        if (!obj->st_boundary().IsEmpty()) {
          st_speed = obj->st_boundary().STspeed();
          AWARN_IF(FLAGS_enable_debug_motion)<<"obstacle.iD = " << obstacle.first << ", st_speed = " << st_speed;
        } else {
          st_speed = obj->speed();;
        }
        yield_obstacles.push_back(std::make_pair(obstacle.first, st_speed));
      }
    }
  }

  if (follow_obstacles.empty() && sidepass_obstacles.empty()
     && ovetake_obstacles.empty() && yield_obstacles.empty()&&speed_limit_obstacles.empty()) return;

  const STmap* st_map = &(BehaviorParser::instance()->target_reference_line().st_map); 

  if (!BehaviorParser::instance()->is_local_path_in_current()) {
    st_map = &(BehaviorParser::instance()->reference_lines_data().front().st_map);
    AWARN_IF(FLAGS_enable_debug_motion)<<"use local st_map";
  }

  for (const auto& t : t_evaluated_) {
    std::vector<std::pair<double, double>> follow_st_points;
    std::vector<std::pair<double, double>> follow_vt_points;
    std::vector<std::pair<double, double>> overtake_st_points;
    std::vector<std::pair<double, double>> yield_st_points;
    std::vector<std::pair<double, double>> yield_vt_points;
    std::vector<std::pair<double, double>> speed_limit_st_points;
    std::vector<std::pair<double, double>> speed_limit_vt_points;
    vector<STMapPointStruct> st_point;
    int time_index = floor(t / FLAGS_scale_t);
    std::vector<int> found_overtake_obstacles;

    if (st_map->GetTPoints(time_index,st_point)) {
      int count_s = 0;
      for (const auto& st : st_point) { 
        if (follow_obstacles.size()) {
          for (const auto& object : follow_obstacles) {
            bool is_find = false;
            auto iterater = st.objs.find(object.first);
            if (iterater == st.objs.end()) {
              continue;
            } else {
              is_find = true;
              follow_st_points.push_back(std::make_pair(st.index.s * FLAGS_scale_s,iterater->second.p));
              follow_vt_points.push_back(std::make_pair(object.second,iterater->second.p));
            }
            if (is_find) {
              break;
            }
          }
        }
        if(speed_limit_obstacles.size()){
            for(const auto& id:speed_limit_obstacles){
                bool is_find = false;
                auto iterater = st.objs.find(id);
                if(iterater == st.objs.end()){
                    continue;
                }else{
                    is_find = true;
                    speed_limit_st_points.push_back(std::make_pair(st.index.s * FLAGS_scale_s,iterater->second.p));
                    speed_limit_vt_points.push_back(std::make_pair(iterater->second.speed,iterater->second.p));
                }
                if(is_find){
                    break;
                }
            }
        }

        if (ovetake_obstacles.size()) {
          for (const auto& id : ovetake_obstacles) {
            bool is_find = false;
            auto iterater = st.objs.find(id);
            if (iterater == st.objs.end()) {
              continue;
            } else {
              is_find = true;
              if (found_overtake_obstacles.empty() || found_overtake_obstacles.back() != id) {
                found_overtake_obstacles.push_back(id);
              }
              double obstacle_speed = iterater->second.speed;
              double overtake_distance_s = std::fmin(
              std::fabs(st_graph_data_.init_point().v() - obstacle_speed) * kOvertakeTimeBuffer,
                  kMaxOvertakeDistance);
              overtake_distance_s = std::fmax(overtake_distance_s, kMinOvertakeDistance);
              overtake_st_points.push_back(std::make_pair(st.index.s * FLAGS_scale_s + overtake_distance_s,iterater->second.p));
            }
            if (is_find) {
              break;
            }
          }
        }
        if (yield_obstacles.size()) {
          for (const auto& obj : yield_obstacles) {
            bool is_find = false;
            auto iterater = st.objs.find(obj.first);
            if (iterater == st.objs.end()) {
              continue;
            } else {
              is_find = true;
              yield_st_points.push_back(std::make_pair(st.index.s * FLAGS_scale_s,iterater->second.p));
              yield_vt_points.push_back(std::make_pair(obj.second,iterater->second.p));
            }
            if (is_find) {
              break;
            }
          }
        }
        count_s++;
      }
    }
    if (follow_obstacles.size()) {
      follow_st_map_.push_back(follow_st_points);
      follow_vt_map_.push_back(follow_vt_points);
    }
    if(speed_limit_obstacles.size()){
      speed_limit_st_map_.push_back(speed_limit_st_points);
      speed_limit_vt_map_.push_back(speed_limit_vt_points);     
    }
    if (ovetake_obstacles.size() || overtake_st_map_.size()) {
      overtake_st_map_.push_back(overtake_st_points);
    }
    if (yield_obstacles.size()) {
      yield_st_map_.push_back(yield_st_points);
      yield_vt_map_.push_back(yield_vt_points);
    }
  }
}

void QpSplineStGraph::ModifySpeedLimit(std::vector<double>& speed_upper_bound) {// avoid init_point_ > cruise_speed;
  if (init_point_.v() > st_graph_data_.cruise_speed()) {
    const double deceleration = -0.8;
    const double time_to_cruise_speed = (st_graph_data_.cruise_speed() - init_point_.v()) / deceleration;
    for (size_t i = 0 ; i < speed_upper_bound.size(); ++i) {
      if (i * t_evaluated_resolution_ < time_to_cruise_speed) {
        speed_upper_bound.at(i) = init_point_.v();
      }
    }
  }
}

}  // namespace planning
}  // namespace acu
