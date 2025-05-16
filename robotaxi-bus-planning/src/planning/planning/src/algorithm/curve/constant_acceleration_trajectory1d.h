/**
 * @file constant_acceleration_trajectory1d.h
 **/

#pragma once

#include <string>

#include "curve1d.h"

namespace acu {
namespace planning {

class ConstantAccelerationTrajectory1d : public math::Curve1d {
 public:
  ConstantAccelerationTrajectory1d(const double init_s, const double init_v,
                                  const double dest_v, const double a);

  virtual ~ConstantAccelerationTrajectory1d() = default;

  double ParamLength() const override;

  std::string ToString() const override;

  // handles extrapolation internally
  double Evaluate(const std::uint32_t order, const double param) const override;

 private:
  double Evaluate_s(const double t) const;

  double Evaluate_v(const double t) const;

  double Evaluate_a(const double t) const;

  double Evaluate_j(const double t) const;

  double init_s_;

  double init_v_;

  double dest_v_;

  double acceleration_;

  double end_t_;

  double end_s_;
};

}  // namespace planning
}  // namespace acu
