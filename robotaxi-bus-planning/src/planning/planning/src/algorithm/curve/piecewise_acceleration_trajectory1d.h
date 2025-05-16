/**
 * @file piecewise_acceleration_trajectory1d.h
 **/

#pragma once

#include <array>
#include <string>
#include <vector>
#include "curve1d.h"

namespace acu {
namespace planning {

class PiecewiseAccelerationTrajectory1d : public math::Curve1d {
 public:
  PiecewiseAccelerationTrajectory1d(const double start_s, const double start_v);

  virtual ~PiecewiseAccelerationTrajectory1d() = default;

  void AppendSegment(const double a, const double t_duration);

  void PopSegment();

  double ParamLength() const override;

  double Evaluate(const std::uint32_t order, const double param) const override;

  std::array<double, 4> Evaluate(const double t) const;

 private:
  double Evaluate_s(const double t) const;

  double Evaluate_v(const double t) const;

  double Evaluate_a(const double t) const;

  double Evaluate_j(const double t) const;

 private:
  // accumulated s
  std::vector<double> s_;

  std::vector<double> v_;

  // accumulated t
  std::vector<double> t_;

  std::vector<double> a_;
};

}  // namespace planning
}  // namespace acu
