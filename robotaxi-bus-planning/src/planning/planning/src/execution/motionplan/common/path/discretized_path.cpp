/**
 * @file path.cpp
 **/

//#include "ros/ros.h"
#include "discretized_path.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/datatype.h"
#include "src/algorithm/math_util/interpolation/linear_interpolation.h"


namespace acu {
namespace planning {

DiscretizedPath::DiscretizedPath(const std::vector<common::PathPoint> &path_points)
    : std::vector<common::PathPoint>(path_points) {}

double DiscretizedPath::Length() const {
  if (empty()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "[DiscretizedPath::Length]DiscretizedPath is empty.";
    return 0.0;
  }
  return back().s() - front().s();
}

common::PathPoint DiscretizedPath::Evaluate(const double path_s) const {
  CHECK(!empty());
  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == begin()) {
    return front();
  }
  if (it_lower == end()) {
    return back();
  }
  return math::InterpolateUsingLinearApproximation(*(it_lower - 1),*it_lower, path_s);
}

std::vector<common::PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
    const double path_s) const {
  auto func = [](const common::PathPoint &tp, const double path_s) {
    return tp.s() < path_s;
  };
  return std::lower_bound(begin(), end(), path_s, func);
}

}  // namespace planning
}  // namespace acu
