/**
 * @file discretized_trajectory.cpp
 **/

#include "discretized_trajectory.h"
#include "src/algorithm/math_util/interpolation/linear_interpolation.h"
//#include "ros/ros.h"
#include <limits>

namespace acu {
namespace planning {

DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<common::TrajectoryPoint>& trajectory_points)
    : std::vector<common::TrajectoryPoint>(trajectory_points) {
  CHECK(!trajectory_points.empty())
      << "trajectory_points should NOT be empty()";
}

DiscretizedTrajectory::DiscretizedTrajectory(const ADCTrajectory& trajectory) {
  assign(trajectory.trajectory_point().begin(),
         trajectory.trajectory_point().end());
}

common::PathPoint DiscretizedTrajectory::Evaluate(const double path_s) const {
  CHECK(!empty());
  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == begin()) {
    return front().path_point();
  }
  if (it_lower == end()) {
    return back().path_point();
  }
  return math::InterpolateUsingLinearApproximation((it_lower - 1)->path_point(),it_lower->path_point(), path_s);
}

std::vector<common::TrajectoryPoint>::const_iterator DiscretizedTrajectory::QueryLowerBound(
    const double path_s) const {
  auto func = [](const common::TrajectoryPoint &tp, const double path_s) {
    return tp.path_point().s() < path_s;
  };
  return std::lower_bound(begin(), end(), path_s, func);
}

size_t DiscretizedTrajectory::QueryLowerBoundPoint(
    const double relative_time) const {
  CHECK(!empty());

  if (relative_time >= back().relative_time()) {
    return size() - 1;
  }
  auto func = [](const common::TrajectoryPoint& tp, const double relative_time) {
    return tp.relative_time() < relative_time;
  };
  auto it_lower = std::lower_bound(begin(), end(), relative_time, func);
  return std::distance(begin(), it_lower);
}

size_t DiscretizedTrajectory::QueryLowerBoundPointByS(
    const double s) const {
  CHECK(!empty());

  if (s >= back().path_point().s()) {
    return size() - 1;
  }
  auto func = [](const common::TrajectoryPoint& tp, const double s) {
    return tp.path_point().s() < s;
  };
  auto it_lower = std::lower_bound(begin(), end(), s, func);
  return std::distance(begin(), it_lower);
}

size_t DiscretizedTrajectory::QueryNearestPointWithBuffer(
    const common::math::Vec2d& position, const double buffer) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {
    const common::math::Vec2d curr_point(data()[i].path_point().x(),
                                         data()[i].path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min + buffer) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

void DiscretizedTrajectory::AppendTrajectoryPoint(
    const common::TrajectoryPoint& trajectory_point) {
  if (!empty()) {//@pqg 将限制改为未来的点的时间>=当前点的时间
    CHECK_GE(trajectory_point.relative_time(), back().relative_time());
  }
  push_back(trajectory_point);
}

const common::TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(
    const size_t index) const {
  CHECK_LT(index, NumOfPoints());
  return data()[index];
}

common::TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  CHECK(!empty());
  return front();
}

void DiscretizedTrajectory::UpdateTrajectoryPoint(const common::math::Vec2d& position, const double buffer) {
  size_t matched_index = QueryNearestPointWithBuffer(position,buffer);
  double s_zero = data()[matched_index].path_point().s();
  double t_zero = data()[matched_index].relative_time();
  erase(begin(),begin() + matched_index);
  for (size_t i = 0 ; i < size(); ++i) {
    data()[i].mutable_path_point()->set_s(data()[i].path_point().s() - s_zero);
    data()[i].set_relative_time(data()[i].relative_time() - t_zero);
  }
  length_ = length_ - s_zero;
}

}  // namespace planning
}  // namespace acu
