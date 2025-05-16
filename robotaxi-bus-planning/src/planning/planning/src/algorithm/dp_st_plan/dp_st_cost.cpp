/**
 * @file dp_st_cost.cpp
 **/

#include <algorithm>
#include <limits>

#include "dp_st_cost.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/common/speed/st_point.h"

namespace acu {
namespace planning {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}

DpStCost::DpStCost(const DpStSpeedConfig& config,
                   const std::vector<const Obstacle*>& obstacles,
                   const common::TrajectoryPoint& init_point)
    : config_(config), obstacles_(obstacles), init_point_(init_point) {
  int index = 0;
  for (auto& obstacle : obstacles) {
    boundary_map_[obstacle->st_boundary().id()] = index++;
  }
  unit_t_ = config_.total_time() / config_.matrix_dimension_t();

  AddToKeepClearRange(obstacles);

  boundary_cost_.resize(obstacles_.size());
  for (auto& vec : boundary_cost_) {
    vec.resize(config_.matrix_dimension_t(), std::make_pair(-1.0, -1.0));
  }
  accel_cost_.fill(-1.0);
  jerk_cost_.fill(-1.0);
}

void DpStCost::AddToKeepClearRange(
    const std::vector<const Obstacle*>& obstacles) {
  for (const auto& obstacle : obstacles) {
    if (obstacle->st_boundary().IsEmpty()) {
      continue;
    }
    if (obstacle->st_boundary().boundary_type() !=
        StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    double start_s = obstacle->st_boundary().min_s();
    double end_s = obstacle->st_boundary().max_s();
    keep_clear_range_.emplace_back(start_s, end_s);
  }
  SortAndMergeRange(&keep_clear_range_);
}

void DpStCost::SortAndMergeRange(
    std::vector<std::pair<double, double>>* keep_clear_range) {
  if (!keep_clear_range || keep_clear_range->empty()) {
    return;
  }
  std::sort(keep_clear_range->begin(), keep_clear_range->end());
  size_t i = 0;
  size_t j = i + 1;
  while (j < keep_clear_range->size()) {
    if (keep_clear_range->at(i).second < keep_clear_range->at(j).first) {
      ++i;
      ++j;
    } else {
      keep_clear_range->at(i).second = std::max(keep_clear_range->at(i).second,
                                                keep_clear_range->at(j).second);
      ++j;
    }
  }
  keep_clear_range->resize(i + 1);
}

bool DpStCost::InKeepClearRange(double s) const {
  if (keep_clear_range_.empty()) {
    return false;
  }
  for (const auto& p : keep_clear_range_) {
    if (p.first <= s && p.second >= s) {
      return true;
    }
  }
  return false;
}

double DpStCost::GetObstacleCost(const StGraphPoint& st_graph_point) {
  const double s = st_graph_point.point().s();
  const double t = st_graph_point.point().t();

  double cost = 0.0;

  for (const auto* obstacle : obstacles_) { 
    if (!obstacle->IsBlockingObstacle()) {
      continue;
    }

    auto boundary = obstacle->st_boundary();
    const double kIgnoreDistance = 200.0;

    if (boundary.min_s() > kIgnoreDistance) {
      // AWARN_IF(FLAGS_enable_debug_motion)<<"Obstacle is more than 200 m away from the car,ignore ";
      continue;
    }
    if (t < boundary.min_t() || t > boundary.max_t()) {
      // AWARN_IF(FLAGS_enable_debug_motion)<<"Obstacle is out time scope,ignore ";
      continue;
    }

    if (obstacle->IsBlockingObstacle() &&
        boundary.IsPointInBoundary(STPoint(s, t))) {
      // AINFO_IF(FLAGS_enable_debug_motion)<<"this is BlockingObstacle() ,return kInf";
      return kInf;
    }
    double s_upper = 0.0;
    double s_lower = 0.0;

    int boundary_index = boundary_map_[boundary.id()];
    if (boundary_cost_[boundary_index][st_graph_point.index_t()].first < 0.0) {
      boundary.GetBoundarySRange(t, &s_upper, &s_lower);
      boundary_cost_[boundary_index][st_graph_point.index_t()] =
          std::make_pair(s_upper, s_lower);
    } else {
      s_upper = boundary_cost_[boundary_index][st_graph_point.index_t()].first;
      s_lower = boundary_cost_[boundary_index][st_graph_point.index_t()].second;
    }
    if (s < s_lower) {
      constexpr double kSafeTimeBuffer = 3.0;
      const double len = obstacle->speed() * kSafeTimeBuffer;
      if (s + len < s_lower) {
        continue;
      } else {
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                std::pow((len - s_lower + s), 2);
      }
    } else if (s > s_upper) {
      const double kSafeDistance = 20.0;  // or calculated from velocity
      if (s > s_upper + kSafeDistance) {
        continue;
      } else {
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                std::pow((kSafeDistance + s_upper - s), 2);
      }
    }
  }
  return cost * unit_t_;
}

double DpStCost::GetReferenceCost(const STPoint& point,
                                 const STPoint& reference_point) const {
  return config_.reference_weight() * (point.s() - reference_point.s()) *
         (point.s() - reference_point.s()) * unit_t_;
}

double DpStCost::GetSpeedCost(const STPoint& first, const STPoint& second,
                             const double speed_limit) const {
  double cost = 0.0;
  const double speed = (second.s() - first.s()) / unit_t_;
  if (speed < 0) {
    return kInf;
  }

  if (speed < FLAGS_max_stop_speed && InKeepClearRange(second.s())) {
    // first.s in range //keep_clear_low_speed_penalty 10.0 ;default_speed_cost :1.0e3 即1
    cost += config_.keep_clear_low_speed_penalty() * unit_t_ *   
            config_.default_speed_cost();
  }

  double det_speed = (speed - speed_limit) / speed_limit;
  if (det_speed > 0) {//说明超过速度限制  //exceed_speed_penalty = low_speed_penalty=10
    cost += config_.exceed_speed_penalty() * config_.default_speed_cost() *
            fabs(speed * speed) * unit_t_;
  } else if (det_speed < 0) { //说明低于速度限制
    cost += config_.low_speed_penalty() * config_.default_speed_cost() *
            -det_speed * unit_t_;
  }
  return cost;//当速度等于速度限制值时，cost为零，即到达了理想速度值
}

double DpStCost::GetAccelCost(const double accel) {
  double cost = 0.0;
  constexpr double kEpsilon = 0.1;
  constexpr size_t kShift = 100;
  const size_t accel_key = static_cast<size_t>(accel / kEpsilon + 0.5 + kShift);//accel_key = accel/0.1+0.5+100
  DCHECK_LT(accel_key, accel_cost_.size());
  if (accel_key >= accel_cost_.size()) {
    return kInf;
  }

  if (accel_cost_.at(accel_key) < 0.0) {
    const double accel_sq = accel * accel;
    double max_acc = config_.max_acceleration();
    double max_dec = config_.max_deceleration();
    double accel_penalty = config_.accel_penalty();//1.0
    double decel_penalty = config_.decel_penalty();//1.0

    if (accel > 0.0) {
      cost = accel_penalty * accel_sq;
    } else {
      cost = decel_penalty * accel_sq;
    }
    cost += accel_sq * decel_penalty * decel_penalty /
                (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty /
                (1 + std::exp(-1.0 * (accel - max_acc)));
    accel_cost_.at(accel_key) = cost;
  } else {
    cost = accel_cost_.at(accel_key);
  }
  return cost * unit_t_;
}

double DpStCost::GetAccelCostByThreePoints(const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third) {
  double accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);
  return GetAccelCost(accel);
}

double DpStCost::GetAccelCostByTwoPoints(const double pre_speed,
                                        const STPoint& pre_point,
                                        const STPoint& curr_point) {
  double current_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  double accel = (current_speed - pre_speed) / unit_t_;
  return GetAccelCost(accel);
}

double DpStCost::JerkCost(const double jerk) {
  double cost = 0.0;
  constexpr double kEpsilon = 0.1;
  constexpr size_t kShift = 200;
  const size_t jerk_key = static_cast<size_t>(jerk / kEpsilon + 0.5 + kShift);
  if (jerk_key >= jerk_cost_.size()) {
    return kInf;
  }

  if (jerk_cost_.at(jerk_key) < 0.0) {
    double jerk_sq = jerk * jerk;
    if (jerk > 0) {
      cost = config_.positive_jerk_coeff() * jerk_sq * unit_t_;
    } else {
      cost = config_.negative_jerk_coeff() * jerk_sq * unit_t_;
    }
    jerk_cost_.at(jerk_key) = cost;//填充进去，方便下次遇到该dda时，不用再次计算即可得到cost，减少运算量，加速度的cost也同理
  } else {
    cost = jerk_cost_.at(jerk_key);
  }

  // TODO(All): normalize to unit_t_
  return cost;
}

double DpStCost::GetJerkCostByFourPoints(const STPoint& first,
                                        const STPoint& second,
                                        const STPoint& third,
                                        const STPoint& fourth) {
  double jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
               (unit_t_ * unit_t_ * unit_t_);
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByTwoPoints(const double pre_speed,
                                       const double pre_acc,
                                       const STPoint& pre_point,
                                       const STPoint& curr_point) {
  const double curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  const double curr_accel = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_accel - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByThreePoints(const double first_speed,//规划起始点的速度
                                         const STPoint& first,//规划起点的ST点
                                         const STPoint& second,//第一列的STpoint
                                         const STPoint& third) {//当前STpoint
  const double pre_speed = (second.s() - first.s()) / unit_t_;//中间点的速度
  const double pre_acc = (pre_speed - first_speed) / unit_t_;//中间点的加速度
  const double curr_speed = (third.s() - second.s()) / unit_t_;//当前点的速度
  const double curr_acc = (curr_speed - pre_speed) / unit_t_;//当前点的加速度
  const double jerk = (curr_acc - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

}  // namespace planning
}  // namespace acu
