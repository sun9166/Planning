/**
 * @file piecewise_jerk_trajectory1d.h
 **/

#pragma once

#include <string>
#include <vector>

#include "constant_jerk_trajectory1d.h"
#include "curve1d.h"

namespace acu {
namespace planning {

class PiecewiseJerkTrajectory1d : public math::Curve1d {
 public:
  PiecewiseJerkTrajectory1d(const double p, const double v, const double a);

  virtual ~PiecewiseJerkTrajectory1d() = default;

  double Evaluate(const std::uint32_t order, const double param) const;

  double ParamLength() const;

  std::string ToString() const;

  void AppendSegment(const double jerk, const double param);

 private:
  std::vector<ConstantJerkTrajectory1d> segments_;

  double last_p_;

  double last_v_;

  double last_a_;

  std::vector<double> param_;
};

}  // namespace planning
}  // namespace acu
