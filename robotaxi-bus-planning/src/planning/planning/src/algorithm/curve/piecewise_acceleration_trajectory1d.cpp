/**
 * @file piecewise_acceleration_trajectory1d.cpp
 **/

#include "piecewise_acceleration_trajectory1d.h"

#include <algorithm>
#include "common/base/log/include/log.h"
#include "common/math/linear_interpolation.h"
#include "src/execution/motionplan/common/planning_gflags.h"

namespace acu {
namespace planning {

PiecewiseAccelerationTrajectory1d::PiecewiseAccelerationTrajectory1d(
    const double start_s, const double start_v) {
  s_.push_back(start_s);
  v_.push_back(start_v);
  a_.push_back(0.0);
  t_.push_back(0.0);
}

void PiecewiseAccelerationTrajectory1d::AppendSegment(const double a,
                                                      const double t_duration) {
  double s0 = s_.back();
  double v0 = v_.back();
  double t0 = t_.back();

  double v1 = v0 + a * t_duration;
  CHECK(v1 >= -FLAGS_numerical_epsilon);

  double delta_s = (v0 + v1) * t_duration * 0.5;
  double s1 = s0 + delta_s;
  double t1 = t0 + t_duration;

  CHECK(s1 >= s0 - FLAGS_numerical_epsilon);
  s1 = std::max(s1, s0);
  s_.push_back(s1);
  v_.push_back(v1);
  a_.push_back(a);
  t_.push_back(t1);
}

void PiecewiseAccelerationTrajectory1d::PopSegment() {
  if (a_.size() > 0) {
    s_.pop_back();
    v_.pop_back();
    a_.pop_back();
    t_.pop_back();
  }
}

double PiecewiseAccelerationTrajectory1d::ParamLength() const {
  CHECK_GT(t_.size(), 1);
  return t_.back() - t_.front();
}

double PiecewiseAccelerationTrajectory1d::Evaluate(const std::uint32_t order,
                                                   const double param) const {
  CHECK_GT(t_.size(), 1);
  CHECK(t_.front() <= param && param <= t_.back());

  switch (order) {
    case 0:
      return Evaluate_s(param);
    case 1:
      return Evaluate_v(param);
    case 2:
      return Evaluate_a(param);
    case 3:
      return Evaluate_j(param);
  }
  return 0.0;
}

double PiecewiseAccelerationTrajectory1d::Evaluate_s(const double t) const {
  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  auto index = std::distance(t_.begin(), it_lower);

  double s0 = s_[index - 1];
  double v0 = v_[index - 1];
  double t0 = t_[index - 1];

  double v1 = v_[index];
  double t1 = t_[index];

  double v = common::math::lerp(v0, t0, v1, t1, t);
  double s = (v0 + v) * (t - t0) * 0.5 + s0;
  return s;
}

double PiecewiseAccelerationTrajectory1d::Evaluate_v(const double t) const {
  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  auto index = std::distance(t_.begin(), it_lower);

  double v0 = v_[index - 1];
  double t0 = t_[index - 1];

  double v1 = v_[index];
  double t1 = t_[index];

  double v = acu::common::math::lerp(v0, t0, v1, t1, t);
  return v;
}

double PiecewiseAccelerationTrajectory1d::Evaluate_a(const double t) const {
  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  auto index = std::distance(t_.begin(), it_lower);
  return a_[index - 1];
}

double PiecewiseAccelerationTrajectory1d::Evaluate_j(const double t) const {
  return 0.0;
}

std::array<double, 4> PiecewiseAccelerationTrajectory1d::Evaluate(
    const double t) const {
  CHECK_GT(t_.size(), 1);
  CHECK(t_.front() <= t && t <= t_.back());

  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  auto index = std::distance(t_.begin(), it_lower);

  double s0 = s_[index - 1];
  double v0 = v_[index - 1];
  double t0 = t_[index - 1];

  double v1 = v_[index];
  double t1 = t_[index];

  double v = common::math::lerp(v0, t0, v1, t1, t);
  double s = (v0 + v) * (t - t0) * 0.5 + s0;

  double a = a_[index - 1];
  double j = 0.0;

  return {{s, v, a, j}};
}

}  // namespace planning
}  // namespace acu
