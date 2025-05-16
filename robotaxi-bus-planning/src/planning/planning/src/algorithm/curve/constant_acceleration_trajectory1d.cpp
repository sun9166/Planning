/**
 * @file constant_acceleration_trajectory1d.cpp
 **/

#include "constant_acceleration_trajectory1d.h"
#include <cmath>
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/planning_gflags.h"

namespace acu {
namespace planning {

ConstantAccelerationTrajectory1d::ConstantAccelerationTrajectory1d(
    const double init_s, const double init_v, const double dest_v, const double a)
    : init_s_(init_s), init_v_(init_v), dest_v_(dest_v), acceleration_(a) {
  if (init_v_ < -FLAGS_numerical_epsilon) {
    AERROR_IF(FLAGS_enable_debug_motion) << "negative init v = " << init_v_;
  }
  init_v_ = std::fabs(init_v_);
  CHECK(acceleration_ > 0.0);
  end_t_ = (dest_v_ - init_v_) / acceleration_;
  end_s_ = (dest_v_*dest_v_ - init_v_ * init_v_) / (2.0 * acceleration_) + init_s_;
}

double ConstantAccelerationTrajectory1d::Evaluate_s(const double t) const {
  if (t < end_t_) {
    double curr_v = init_v_ - acceleration_ * t;
    double delta_s = (curr_v + init_v_) * t * 0.5;
    return init_s_ + delta_s;
  } else {
    return end_s_;
  }
}

double ConstantAccelerationTrajectory1d::Evaluate_v(const double t) const {
  if (t < end_t_) {
    return init_v_ + acceleration_ * t;
  } else {
    return dest_v_;
  }
}

double ConstantAccelerationTrajectory1d::Evaluate_a(const double t) const {
  if (t < end_t_) {
    return acceleration_;
  } else {
    return 0.0;
  }
}

double ConstantAccelerationTrajectory1d::Evaluate_j(const double t) const {
  return 0.0;
}

double ConstantAccelerationTrajectory1d::ParamLength() const { return end_t_; }

std::string ConstantAccelerationTrajectory1d::ToString() const { return ""; }

double ConstantAccelerationTrajectory1d::Evaluate(const std::uint32_t order,
                                                  const double param) const {
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

}  // namespace planning
}  // namespace acu
