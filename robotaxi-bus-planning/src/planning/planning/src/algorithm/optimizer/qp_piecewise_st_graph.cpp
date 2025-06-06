/**
 * @file qp_piecewise_st_graph.cpp
 **/
////#include "ros/ros.h"
#include <algorithm>
#include <limits>
#include <string>
#include "qp_piecewise_st_graph.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/planning_gflags.h"

namespace acu {
namespace planning {

using acu::common::ErrorCode;
using acu::common::Status;

QpPiecewiseStGraph::QpPiecewiseStGraph(
  const QpStSpeedConfig& qp_st_speed_config,const StGraphData& st_graph_data)
  : qp_st_speed_config_(qp_st_speed_config),
    t_evaluated_resolution_(qp_st_speed_config_.total_time() /
                            qp_st_speed_config_.qp_piecewise_config()
                            .number_of_evaluated_graph_t()),
                            st_graph_data_(st_graph_data) {
  Init();
}

void QpPiecewiseStGraph::Init() {
  // init evaluated t positions
  double curr_t = t_evaluated_resolution_;
  t_evaluated_.resize(
    qp_st_speed_config_.qp_piecewise_config().number_of_evaluated_graph_t());
  for (auto& t : t_evaluated_) {
    t = curr_t;
    curr_t += t_evaluated_resolution_;
  }
}

void QpPiecewiseStGraph::SetDebugLogger(
  planning_internal::STGraphDebug* st_graph_debug) {
  if (st_graph_debug) {
    st_graph_debug->Clear();
    st_graph_debug_ = st_graph_debug;
  }
}

Status QpPiecewiseStGraph::Search(
  // const StGraphData& st_graph_data, 
  SpeedInfo* const speed_data,
  const std::pair<double, double>& accel_bound) {
  cruise_.clear();

  init_point_ = st_graph_data_.init_point();

  // reset piecewise linear generator
  generator_.reset(new math::PiecewiseLinearGenerator(
                     qp_st_speed_config_.qp_piecewise_config().number_of_evaluated_graph_t(),
                     t_evaluated_resolution_));

  if (!AddConstraint(st_graph_data_.init_point(), st_graph_data_.speed_limit(),
                     st_graph_data_.st_boundaries(), accel_bound)
      .ok()) {
    const std::string msg = "Add constraint failed!";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  if (!AddKernel(st_graph_data_.st_boundaries(), st_graph_data_.speed_limit())
      .ok()) {
    const std::string msg = "Add kernel failed!";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  if (!Solve().ok()) {
    const std::string msg = "Solve qp problem failed!";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  // extract output
  speed_data->clear();
  const auto& res = generator_->params();
  speed_data->AppendSpeedPoint(0.0, 0.0, init_point_.v(), init_point_.a(), 0.0);

  double v = 0.0;
  double a = 0.0;

  double time = t_evaluated_resolution_;
  double dt = t_evaluated_resolution_;

  for (int i = 0; i < res.rows(); ++i, time += t_evaluated_resolution_) {
    double s = res(i, 0);
    if (i == 0) {
      v = s / dt;
      a = (v - init_point_.v()) / dt;
    } else {
      const double curr_v = (s - res(i - 1, 0)) / dt;
      a = (curr_v - v) / dt;
      v = curr_v;
    }
    speed_data->AppendSpeedPoint(s, time, v, a, 0.0);
  }
  return Status::OK();
}

Status QpPiecewiseStGraph::AddConstraint(
  const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
  const std::vector<const StBoundary*>& boundaries,
  const std::pair<double, double>& accel_bound) {
  auto* constraint = generator_->mutable_constraint();
  // position, velocity, acceleration

  // monotone constraint
  if (!constraint->AddMonotoneInequalityConstraint()) {
    const std::string msg = "add monotone inequality constraint failed!";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  std::vector<uint32_t> index_list(t_evaluated_.size());

  // boundary constraint
  std::vector<double> s_upper_bound(t_evaluated_.size());
  std::vector<double> s_lower_bound(t_evaluated_.size());
  uint32_t t_evaluated_size = t_evaluated_.size();
  for (uint32_t i = 0; i < t_evaluated_size; ++i) {
    index_list[i] = i;
    const double curr_t = t_evaluated_[i];
    double lower_s = 0.0;
    double upper_s = 0.0;
    GetSConstraintByTime(boundaries, curr_t,
                         qp_st_speed_config_.total_path_length(), &upper_s,
                         &lower_s);
    s_upper_bound[i] = upper_s;
    s_lower_bound[i] = lower_s;
  }
  if (!constraint->AddBoundary(index_list, s_lower_bound, s_upper_bound)) {
    const std::string msg = "Fail to apply distance constraints.";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  // speed constraint
  std::vector<double> speed_upper_bound;
  if (!EstimateSpeedUpperBound(init_point, speed_limit, &speed_upper_bound)
      .ok()) {
    std::string msg = "Fail to estimate speed upper constraints.";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  std::vector<double> speed_lower_bound(t_evaluated_.size(), 0.0);
  if (st_graph_debug_) {
    auto speed_constraint = st_graph_debug_->mutable_speed_constraint();
    
    for (size_t i = 0; i < t_evaluated_size; ++i) {
      speed_constraint->add_t(t_evaluated_[i]);
      speed_constraint->add_lower_bound(speed_lower_bound[i]);
      speed_constraint->add_upper_bound(speed_upper_bound[i]);
    }
  }

  if (!constraint->AddDerivativeBoundary(index_list, speed_lower_bound,
                                         speed_upper_bound)) {
    const std::string msg = "Fail to apply speed constraints.";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  // acceleration constraint
  std::vector<double> accel_lower_bound(t_evaluated_.size(), accel_bound.first);
  std::vector<double> accel_upper_bound(t_evaluated_.size(),
                                        accel_bound.second);

  if (!constraint->AddSecondDerivativeBoundary(
        init_point.v(), index_list, accel_lower_bound, accel_upper_bound)) {
    const std::string msg = "Fail to apply acceleration constraints.";
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  return Status::OK();
}

Status QpPiecewiseStGraph::AddKernel(
  const std::vector<const StBoundary*>& boundaries,
  const SpeedLimit& speed_limit) {
  auto* kernel = generator_->mutable_kernel();
  DCHECK_NOTNULL(kernel);

  if (qp_st_speed_config_.qp_piecewise_config().accel_kernel_weight() > 0) {
    kernel->AddSecondOrderDerivativeMatrix(
      init_point_.v(),
      qp_st_speed_config_.qp_piecewise_config().accel_kernel_weight());
  }

  if (qp_st_speed_config_.qp_piecewise_config().jerk_kernel_weight() > 0) {
    kernel->AddThirdOrderDerivativeMatrix(
      init_point_.v(), init_point_.a(),
      qp_st_speed_config_.qp_piecewise_config().jerk_kernel_weight());
  }

  if (!AddCruiseReferenceLineKernel(
        speed_limit,
        qp_st_speed_config_.qp_piecewise_config().cruise_weight())
      .ok()) {
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, "QpSplineStGraph::AddKernel");
  }

  if (!AddFollowReferenceLineKernel(
        boundaries,
        qp_st_speed_config_.qp_piecewise_config().follow_weight())
      .ok()) {
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, "QpSplineStGraph::AddKernel");
  }
  kernel->AddRegularization(
    qp_st_speed_config_.qp_piecewise_config().regularization_weight());
  return Status::OK();
}

Status QpPiecewiseStGraph::AddCruiseReferenceLineKernel(
  const SpeedLimit& speed_limit, const double weight) {
  auto* ref_kernel = generator_->mutable_kernel();
  if (speed_limit.speed_limit_points().size() == 0) {
    std::string msg = "Fail to apply_kernel due to empty speed limits.";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }
  std::vector<uint32_t> index_list(t_evaluated_.size());
  cruise_.resize(t_evaluated_.size());
  uint32_t t_evaluated_size = t_evaluated_.size();
  for (uint32_t i = 0; i < t_evaluated_.size(); ++i) {
    index_list[i] = i;
    cruise_[i] = qp_st_speed_config_.total_path_length();
  }
  if (st_graph_debug_) {
    auto kernel_cruise_ref = st_graph_debug_->mutable_kernel_cruise_ref();
    for (uint32_t i = 0; i < t_evaluated_size; ++i) {
      kernel_cruise_ref->mutable_t()->Add(t_evaluated_[i]);
      kernel_cruise_ref->mutable_cruise_line_s()->Add(cruise_[i]);
    }
  }

  if (t_evaluated_.size() > 0) {
    ref_kernel->AddReferenceLineKernelMatrix(
      index_list, cruise_, weight * static_cast<double>(t_evaluated_.size()) /
      qp_st_speed_config_.total_time());
  }

  return Status::OK();
}

Status QpPiecewiseStGraph::AddFollowReferenceLineKernel(
  const std::vector<const StBoundary*>& boundaries, const double weight) {
  auto* follow_kernel = generator_->mutable_kernel();
  std::vector<double> ref_s;
  std::vector<double> filtered_evaluate_t;
  std::vector<uint32_t> index_list;
  size_t t_evaluated_size = t_evaluated_.size();
  for (size_t i = 0; i < t_evaluated_size; ++i) {
    const double curr_t = t_evaluated_[i];
    double s_min = std::numeric_limits<double>::infinity();
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
        s_min = std::min(s_min, s_upper - boundary->characteristic_length() -
                         qp_st_speed_config_.qp_piecewise_config()
                         .follow_drag_distance());
      }
    }
    if (success && s_min < cruise_[i]) {
      filtered_evaluate_t.push_back(curr_t);
      ref_s.push_back(s_min);
      index_list.push_back(static_cast<uint32_t>(i));
      if (st_graph_debug_) {
        auto kernel_follow_ref = st_graph_debug_->mutable_kernel_follow_ref();
        kernel_follow_ref->mutable_t()->Add(curr_t);
        kernel_follow_ref->mutable_follow_line_s()->Add(s_min);
      }
    }
  }
  DCHECK_EQ(filtered_evaluate_t.size(), ref_s.size());

  if (!ref_s.empty()) {
    follow_kernel->AddReferenceLineKernelMatrix(
      index_list, ref_s, weight * static_cast<double>(t_evaluated_.size()) /
      qp_st_speed_config_.total_time());
  }

  size_t filtered_evaluate_t_size = filtered_evaluate_t.size();

  return Status::OK();
}

Status QpPiecewiseStGraph::GetSConstraintByTime(
  const std::vector<const StBoundary*>& boundaries, const double time,
  const double total_path_s, double* const s_upper_bound,
  double* const s_lower_bound) const {
  *s_upper_bound = total_path_s;

  for (const StBoundary* boundary : boundaries) {
    double s_upper = 0.0;
    double s_lower = 0.0;

    if (!boundary->GetUnblockSRange(time, &s_upper, &s_lower)) {
      continue;
    }

    if (boundary->boundary_type() == StBoundary::BoundaryType::STOP ||
        boundary->boundary_type() == StBoundary::BoundaryType::FOLLOW) {
      *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
    } else if (boundary->boundary_type() == StBoundary::BoundaryType::YIELD) {
      *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
    } else if (boundary->boundary_type() ==
               StBoundary::BoundaryType::OVERTAKE) {
      DCHECK(boundary->boundary_type() == StBoundary::BoundaryType::OVERTAKE);
      *s_lower_bound = std::fmax(*s_lower_bound, s_lower);
    }
  }

  return Status::OK();
}

Status QpPiecewiseStGraph::EstimateSpeedUpperBound(
  const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
  std::vector<double>* speed_upper_bound) const {
  DCHECK_NOTNULL(speed_upper_bound);

  speed_upper_bound->clear();
  if (speed_limit.speed_limit_points().empty()) {
    std::string msg = "speed_limit_points size is zero !";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }
  // use v to estimate position: not accurate, but feasible in cyclic
  // processing. We can do the following process multiple times and use
  // previous cycle's results for better estimation.
  const double v = init_point.v();

  if (static_cast<double>(t_evaluated_.size() +
                          speed_limit.speed_limit_points().size()) <
      static_cast<double>(t_evaluated_.size()) *
      std::log(
        static_cast<double>(speed_limit.speed_limit_points().size()))) {
    uint32_t i = 0;
    uint32_t j = 0;
    const double kDistanceEpsilon = 1e-6;
    while (i < t_evaluated_.size() &&
           j + 1 < speed_limit.speed_limit_points().size()) {
      const double distance = v * t_evaluated_[i];
      if (fabs(distance - speed_limit.speed_limit_points()[j].first) <
          kDistanceEpsilon) {
        speed_upper_bound->push_back(
          speed_limit.speed_limit_points()[j].second);
        ++i;
      } else if (distance < speed_limit.speed_limit_points()[j].first) {
        ++i;
      } else if (distance <= speed_limit.speed_limit_points()[j + 1].first) {
        speed_upper_bound->push_back(speed_limit.GetSpeedLimitByS(distance));
        ++i;
      } else {
        ++j;
      }
    }
    size_t t_evaluated_size = t_evaluated_.size(); 
    for (size_t k = speed_upper_bound->size(); k < t_evaluated_size; ++k) {
      speed_upper_bound->push_back(st_graph_data_.cruise_speed());
    }
  } else {
    auto cmp = [](const std::pair<double, double>& p1, const double s) {
      return p1.first < s;
    };

    const auto& speed_limit_points = speed_limit.speed_limit_points();
    for (const double t : t_evaluated_) {
      const double s = v * t;

      // NOTICE: we are using binary search here based on two assumptions:
      // (1) The s in speed_limit_points increase monotonically.
      // (2) The evaluated_t_.size() << number of speed_limit_points.size()
      //
      // If either of the two assumption is failed, a new algorithm must be
      // used
      // to replace the binary search.

      const auto& it = std::lower_bound(speed_limit_points.begin(),
                                        speed_limit_points.end(), s, cmp);
      if (it != speed_limit_points.end()) {
        speed_upper_bound->push_back(it->second);
      } else {
        if (!speed_limit_points.empty()) {
          speed_upper_bound->push_back(speed_limit_points.back().second);
        } else {
          //TODO
        }
      }
    }
  }

  const double kTimeBuffer = 2.0;
  const double kSpeedBuffer = 0.1;
  uint32_t t_evaluated_size = t_evaluated_.size();
  for (uint32_t k = 0; k < t_evaluated_size && t_evaluated_[k] < kTimeBuffer;
       ++k) {
    speed_upper_bound->at(k) =
      std::fmax(init_point_.v() + kSpeedBuffer, speed_upper_bound->at(k));
  }

  return Status::OK();
}

Status QpPiecewiseStGraph::Solve() {
  return generator_->Solve()
         ? Status::OK()
         : Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, "QpPiecewiseStGraph::solve");
}

}  // namespace planning
}  // namespace acu
