/**
 * @file st_boundary.cpp
 **/

//#include "ros/ros.h"
#include "st_boundary.h"
#include "common/math/math_utils.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/datatype.h"

namespace acu {
namespace planning {

using common::math::LineSegment2d;
using common::math::Vec2d;

StBoundary::StBoundary(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs) {
  CHECK(IsValid(point_pairs)) << "The input point_pairs are NOT valid";

  std::vector<std::pair<STPoint, STPoint>> reduced_pairs(point_pairs);
  RemoveRedundantPoints(&reduced_pairs);

  for (const auto& item : reduced_pairs) {
    // use same t for both points
    const double t = item.first.t();
    lower_points_.emplace_back(item.first.s(), t);
    upper_points_.emplace_back(item.second.s(), t);
  }

  for (auto it = lower_points_.begin(); it != lower_points_.end(); ++it) {
    points_.emplace_back(it->x(), it->y());
  }
  for (auto rit = upper_points_.rbegin(); rit != upper_points_.rend(); ++rit) {
    points_.emplace_back(rit->x(), rit->y());
  }

  BuildFromPoints();

  for (const auto& point : lower_points_) {
    min_s_ = std::fmin(min_s_, point.s());
  }
  for (const auto& point : upper_points_) {
    max_s_ = std::fmax(max_s_, point.s());
  }
  min_t_ = lower_points_.front().t();
  max_t_ = lower_points_.back().t();
  st_speed_ = (lower_points_.back().s() - lower_points_.front().s())/
              (lower_points_.back().t() - lower_points_.front().t() + FLAGS_numerical_epsilon);
}

bool StBoundary::IsPointNear(const common::math::LineSegment2d& seg,
                             const Vec2d& point, const double max_dist) {
  return seg.DistanceSquareTo(point) < max_dist * max_dist;
}

std::string StBoundary::TypeName(BoundaryType type) {
  if (type == BoundaryType::FOLLOW) {
    return "FOLLOW";
  } else if (type == BoundaryType::KEEP_CLEAR) {
    return "KEEP_CLEAR";
  } else if (type == BoundaryType::OVERTAKE) {
    return "OVERTAKE";
  } else if (type == BoundaryType::STOP) {
    return "STOP";
  } else if (type == BoundaryType::YIELD) {
    return "YIELD";
  } else if (type == BoundaryType::UNKNOWN) {
    return "UNKNOWN";
  }
  return "UNKNOWN";
}

void StBoundary::RemoveRedundantPoints(
    std::vector<std::pair<STPoint, STPoint>>* point_pairs) {
  if (!point_pairs || point_pairs->size() <= 2) {
    return;
  }

  const double kMaxDist = 0.1;
  size_t i = 0;
  size_t j = 1;

  while (i < point_pairs->size() && j + 1 < point_pairs->size()) {
    LineSegment2d lower_seg(point_pairs->at(i).first,
                            point_pairs->at(j + 1).first);
    LineSegment2d upper_seg(point_pairs->at(i).second,
                            point_pairs->at(j + 1).second);
    if (!IsPointNear(lower_seg, point_pairs->at(j).first, kMaxDist) ||
        !IsPointNear(upper_seg, point_pairs->at(j).second, kMaxDist)) {
      ++i;
      if (i != j) {
        point_pairs->at(i) = point_pairs->at(j);
      }
    }
    ++j;
  }
  point_pairs->at(++i) = point_pairs->back();
  point_pairs->resize(i + 1);
}

bool StBoundary::IsValid(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const {
  if (point_pairs.size() < 2) {
    AWARN_IF(FLAGS_enable_debug_motion) << "point_pairs.size() must > 2. current point_pairs.size() = "
           << point_pairs.size();
    return false;
  }

  constexpr double kStBoundaryEpsilon = 1e-9;
  constexpr double kMinDeltaT = 1e-6;
  size_t point_pairs_size = point_pairs.size();
  for (size_t i = 0; i < point_pairs_size; ++i) {
    const auto& curr_lower = point_pairs[i].first;
    const auto& curr_upper = point_pairs[i].second;
    if (curr_upper.s() < curr_lower.s()) {
      AWARN_IF(FLAGS_enable_debug_motion) << "s is not increasing";
      return false;
    }

    if (std::fabs(curr_lower.t() - curr_upper.t()) > kStBoundaryEpsilon) {
      AWARN_IF(FLAGS_enable_debug_motion) << "t diff is larger in each STPoint pair";
      return false;
    }

    if (i + 1 != point_pairs.size()) {
      const auto& next_lower = point_pairs[i + 1].first;
      const auto& next_upper = point_pairs[i + 1].second;
      if (std::fmax(curr_lower.t(), curr_upper.t()) + kMinDeltaT >=
          std::fmin(next_lower.t(), next_upper.t())) {
        AWARN_IF(FLAGS_enable_debug_motion) << "t is not increasing. curr.t = ("
             <<curr_lower.t()<<", "<<curr_upper.t()<<") , next t = ("<<next_lower.t()<<", "<<next_upper.t()<<").";
        return false;
      }
    }
  }
  return true;
}

bool StBoundary::IsPointInBoundary(const STPoint& st_point) const {
  if (st_point.t() <= min_t_ || st_point.t() >= max_t_) {
    return false;
  }
  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, st_point.t(), &left, &right)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "fait to get index range.";
    return false;
  }
  const double check_upper = common::math::CrossProd(
      st_point, upper_points_[left], upper_points_[right]);
  const double check_lower = common::math::CrossProd(
      st_point, lower_points_[left], lower_points_[right]);

  return (check_upper * check_lower < 0);
}

STPoint StBoundary::BottomLeftPoint() const {
  DCHECK(!lower_points_.empty()) << "StBoundary has zero points.";
  return lower_points_.front();
}

STPoint StBoundary::BottomRightPoint() const {
  DCHECK(!lower_points_.empty()) << "StBoundary has zero points.";
  return lower_points_.back();
}

StBoundary StBoundary::ExpandByS(const double s) const {
  if (lower_points_.empty()) {
    return StBoundary();
  }
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  size_t lower_points_size = lower_points_.size();
  for (size_t i = 0; i < lower_points_size; ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points_[i].y() - s, lower_points_[i].x()),
        STPoint(upper_points_[i].y() + s, upper_points_[i].x()));
  }
  return StBoundary(std::move(point_pairs));
}

StBoundary StBoundary::ExpandByT(const double t) const {
  if (lower_points_.empty()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "The current st_boundary has NO points.";
    return StBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  const double left_delta_t = lower_points_[1].t() - lower_points_[0].t();
  const double lower_left_delta_s = lower_points_[1].s() - lower_points_[0].s();
  const double upper_left_delta_s = upper_points_[1].s() - upper_points_[0].s();

  point_pairs.emplace_back(
      STPoint(lower_points_[0].y() - t * lower_left_delta_s / left_delta_t,
              lower_points_[0].x() - t),
      STPoint(upper_points_[0].y() - t * upper_left_delta_s / left_delta_t,
              upper_points_.front().x() - t));

  const double kMinSEpsilon = 1e-3;
  point_pairs.front().first.set_s(
      std::fmin(point_pairs.front().second.s() - kMinSEpsilon,
                point_pairs.front().first.s()));
  size_t lower_points_size = lower_points_.size();
  for (size_t i = 0; i < lower_points_size; ++i) {
    point_pairs.emplace_back(lower_points_[i], upper_points_[i]);
  }

  size_t length = lower_points_.size();
  DCHECK_GE(length, 2);

  const double right_delta_t =
      lower_points_[length - 1].t() - lower_points_[length - 2].t();
  const double lower_right_delta_s =
      lower_points_[length - 1].s() - lower_points_[length - 2].s();
  const double upper_right_delta_s =
      upper_points_[length - 1].s() - upper_points_[length - 2].s();

  point_pairs.emplace_back(STPoint(lower_points_.back().y() +
                                       t * lower_right_delta_s / right_delta_t,
                                   lower_points_.back().x() + t),
                           STPoint(upper_points_.back().y() +
                                       t * upper_right_delta_s / right_delta_t,
                                   upper_points_.back().x() + t));
  point_pairs.back().second.set_s(
      std::fmax(point_pairs.back().second.s(),
                point_pairs.back().first.s() + kMinSEpsilon));

  return StBoundary(std::move(point_pairs));
}


StBoundary StBoundary::ModifyByST(const double t,const double s) const {
  if (lower_points_.empty() || upper_points_.empty()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "The current st_boundary has NO points.";
    return StBoundary();
  }

  if (t >= lower_points_.back().t() && s < lower_points_.front().s()) {
    AERROR_IF(FLAGS_enable_debug_motion)<<" init t: "<<t<<"> obs max t: "<<lower_points_.back().t() 
                                        <<" and s < obs min s: "<<lower_points_.front().s()<<"ignore it .";
    return StBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  //修正st的左边
  size_t larger_than_t_index = 0;
  for (size_t i = 0 ; i <  lower_points_.size(); ++i) {
    if (lower_points_[i].t() >= t) {
      larger_than_t_index = i;
      break;
    }
  }
  const double kMinSEpsilon = 1e-3;
  constexpr double kMinDeltaT = 1e-6;
  AWARN_IF(FLAGS_enable_debug_motion)<<"larger_than_t_index = "<<larger_than_t_index<<", t = "<<t<<", s = "<<s;
  if (larger_than_t_index > 0) {
    
    const double left_delta_t = lower_points_[larger_than_t_index].t() - lower_points_[0].t();
    const double lower_left_delta_s = lower_points_[larger_than_t_index].s() - lower_points_[0].s();
    const double upper_left_delta_s = upper_points_[larger_than_t_index].s() - upper_points_[0].s();
  
    double distance_to_front_t = t - lower_points_[0].t(); 
  
    point_pairs.emplace_back(
        STPoint((lower_points_[0].y() + distance_to_front_t * lower_left_delta_s / left_delta_t) - s,
                (lower_points_[0].x() + distance_to_front_t) - t - kMinDeltaT),
        STPoint((upper_points_[0].y() + distance_to_front_t * upper_left_delta_s / left_delta_t) - s,
                (upper_points_[0].x() + distance_to_front_t) - t - kMinDeltaT));
  }
  
  //修正右边
  for (size_t i = 0 ; i <  lower_points_.size(); ++i) {
    if (lower_points_[i].x() - t > 0) {
      point_pairs.emplace_back(STPoint(lower_points_[i].y() - s, lower_points_[i].x() - t + kMinDeltaT),
                               STPoint(upper_points_[i].y() - s, upper_points_[i].x() - t + kMinDeltaT));
    }
  }

  size_t length = point_pairs.size();
  if (length < 2) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"after modify st point pair size < 2";
    return StBoundary();
  }

  return StBoundary(std::move(point_pairs));
}

StBoundary StBoundary::RightBoundaryExpandByT(const double t) const {
  if (lower_points_.empty()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "The current st_boundary has NO points.";
    return StBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  const double kMinSEpsilon = 1e-3;
    size_t lower_points_size = lower_points_.size();
  for (size_t i = 0; i < lower_points_size; ++i) {
    point_pairs.emplace_back(lower_points_[i], upper_points_[i]);
  }

  size_t length = lower_points_.size();
  DCHECK_GE(length, 2);

  const double right_delta_t =
      lower_points_[length - 1].t() - lower_points_[length - 2].t();
  const double lower_right_delta_s =
      lower_points_[length - 1].s() - lower_points_[length - 2].s();
  const double upper_right_delta_s =
      upper_points_[length - 1].s() - upper_points_[length - 2].s();  
                                                                              
  if (lower_right_delta_s < 0 || upper_right_delta_s < 0 ) {
    point_pairs.emplace_back(STPoint(lower_points_.back().y(),
                                   lower_points_.back().x() + t),
                             STPoint(upper_points_.back().y(),
                                   upper_points_.back().x() + t));
  } else {
    point_pairs.emplace_back(STPoint(lower_points_.back().y() +
                                       t * lower_right_delta_s / right_delta_t,
                                   lower_points_.back().x() + t),
                             STPoint(upper_points_.back().y() +
                                       t * upper_right_delta_s / right_delta_t,
                                   upper_points_.back().x() + t));
  }
  
  
  point_pairs.back().second.set_s(
      std::fmax(point_pairs.back().second.s(),
                point_pairs.back().first.s() + kMinSEpsilon));

  return StBoundary(std::move(point_pairs));

}

StBoundary StBoundary::LeftBoundaryExpandByT(const double t) const {
  if (lower_points_.empty()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "The current st_boundary has NO points.";
    return StBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  const double left_delta_t = lower_points_[1].t() - lower_points_[0].t();
  const double lower_left_delta_s = lower_points_[1].s() - lower_points_[0].s();
  const double upper_left_delta_s = upper_points_[1].s() - upper_points_[0].s();

  point_pairs.emplace_back(
      STPoint(lower_points_[0].y() - t * lower_left_delta_s / left_delta_t,
              lower_points_[0].x() - t),
      STPoint(upper_points_[0].y() - t * upper_left_delta_s / left_delta_t,
              upper_points_.front().x() - t));

  const double kMinSEpsilon = 1e-3;
  point_pairs.front().first.set_s(
      std::fmin(point_pairs.front().second.s() - kMinSEpsilon,
                point_pairs.front().first.s()));
  size_t lower_points_size = lower_points_.size();
  for (size_t i = 0; i < lower_points_size; ++i) {
    point_pairs.emplace_back(lower_points_[i], upper_points_[i]);
  }

  return StBoundary(std::move(point_pairs));

}

StBoundary::BoundaryType StBoundary::boundary_type() const {
  return boundary_type_;
}
void StBoundary::SetBoundaryType(const BoundaryType& boundary_type) {
  boundary_type_ = boundary_type;
}

const std::string& StBoundary::id() const { return id_; }

const bool StBoundary::IsVirtualObstacleBoundary() const { return is_virtual_obstacle_; }

const double& StBoundary::speed() const { return speed_; }
const double& StBoundary::STspeed() const { 
    return st_speed_ ; 
  }

void StBoundary::SetObstacleId(const std::string& id, const bool is_virtual) {
  id_ = id;
  is_virtual_obstacle_ = is_virtual;
}

void StBoundary::SetSpeed(const double& speed) { speed_ = speed;}

double StBoundary::characteristic_length() const {
  return characteristic_length_;
}

void StBoundary::SetCharacteristicLength(const double characteristic_length) {
  characteristic_length_ = characteristic_length;
}

bool StBoundary::GetUnblockSRange(const double curr_time, double* s_upper,
                                  double* s_lower) const {
  CHECK_NOTNULL(s_upper);
  CHECK_NOTNULL(s_lower);

  *s_upper = s_high_limit_;
  *s_lower = 0.0;
  if (curr_time < min_t_ - FLAGS_numerical_epsilon || curr_time > max_t_ + FLAGS_numerical_epsilon) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"curr_time = "<<curr_time<<", t = ("<<min_t_<<", "<<max_t_<<").";
    return true;
  }

  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, curr_time, &left, &right)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Fail to get index range.";
    return false;
  }
  double r = (curr_time - upper_points_[left].t()) /
                   (upper_points_.at(right).t() - upper_points_.at(left).t());

  // AWARN_IF(curr_time > 4 || curr_time < 0.2)<<"r = "<<r<<", left = "<<left<<", right = "<<right;

  if (left - right <= FLAGS_numerical_epsilon) {
    if (left <= FLAGS_numerical_epsilon) {
      r = 0;
      AWARN_IF(FLAGS_enable_debug_motion)<<"set r = 0, curr_time = "<<curr_time<<", left = "<<left<<", right = "<<right;
    } else {
      r = 1;
      AWARN_IF(FLAGS_enable_debug_motion)<<"set r = 1, curr_time = "<<curr_time<<", left = "<<left<<", right = "<<right;
    }
  }

  double upper_cross_s =
      upper_points_[left].s() +
      r * (upper_points_[right].s() - upper_points_[left].s());
  double lower_cross_s =
      lower_points_[left].s() +
      r * (lower_points_[right].s() - lower_points_[left].s());

  if (boundary_type_ == BoundaryType::STOP ||
      boundary_type_ == BoundaryType::YIELD ||
      boundary_type_ == BoundaryType::FOLLOW) {
    *s_upper = lower_cross_s;
  } else if (boundary_type_ == BoundaryType::OVERTAKE) {
    *s_lower = std::fmax(*s_lower, upper_cross_s);
  } else {
    AWARN_IF(FLAGS_enable_debug_motion) << "boundary_type is not supported. boundary_type: "
           << static_cast<int>(boundary_type_);
    return false;
  }
  return true;
}

bool StBoundary::GetBoundarySRange(const double curr_time, double* s_upper,
                                   double* s_lower) const {
  CHECK_NOTNULL(s_upper);
  CHECK_NOTNULL(s_lower);
  if (curr_time < min_t_ || curr_time > max_t_) {
    return false;
  }

  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, curr_time, &left, &right)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Fail to get index range.";
    return false;
  }
  const double r = (curr_time - upper_points_[left].t()) /
                   (upper_points_[right].t() - upper_points_[left].t());

  *s_upper = upper_points_[left].s() +
             r * (upper_points_[right].s() - upper_points_[left].s());
  *s_lower = lower_points_[left].s() +
             r * (lower_points_[right].s() - lower_points_[left].s());

  *s_upper = std::fmin(*s_upper, s_high_limit_);
  *s_lower = std::fmax(*s_lower, 0.0);
  return true;
}

double StBoundary::min_s() const { return min_s_; }
double StBoundary::min_t() const { return min_t_; }
double StBoundary::max_s() const { return max_s_; }
double StBoundary::max_t() const { return max_t_; }

bool StBoundary::GetIndexRange(const std::vector<STPoint>& points,
                               const double t, size_t* left,
                               size_t* right) const {
  CHECK_NOTNULL(left);
  CHECK_NOTNULL(right);
  if (t < points.front().t() - FLAGS_numerical_epsilon || t > points.back().t() + FLAGS_numerical_epsilon) {
    AWARN_IF(FLAGS_enable_debug_motion) << "t is out of range. t = " << t;
    return false;
  }
  auto comp = [](const STPoint& p, const double t) { return p.t() < t; };
  auto first_ge = std::lower_bound(points.begin(), points.end(), t, comp);
  size_t index = std::distance(points.begin(), first_ge);
  if (index == 0) {
    *left = *right = 0;
  } else if (first_ge == points.end()) {
    *left = *right = points.size() - 1;
  } else {
    *left = index - 1;
    *right = index;
  }
  return true;
}

StBoundary StBoundary::GenerateStBoundary(
    const std::vector<STPoint>& lower_points,
    const std::vector<STPoint>& upper_points) {
  if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
    return StBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  size_t lower_points_size = lower_points.size();
  size_t upper_points_size = upper_points.size();
  for (size_t i = 0; i < lower_points_size && i < upper_points_size; ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
        STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  }
  return StBoundary(point_pairs);
}

StBoundary StBoundary::CutOffByT(const double t) const {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  size_t lower_points_size = lower_points_.size();
  size_t upper_points_size = upper_points_.size();
  for (size_t i = 0; i < lower_points_size && i < upper_points_size;
       ++i) {
    if (lower_points_[i].t() < t) {
      continue;
    }
    lower_points.push_back(lower_points_[i]);
    upper_points.push_back(upper_points_[i]);
  }
  return GenerateStBoundary(lower_points, upper_points);
}

}  // namespace planning
}  // namespace acu
