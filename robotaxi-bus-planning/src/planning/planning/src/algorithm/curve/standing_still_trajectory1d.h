/**
 * @file standing_still_trajectory1d.h
 **/

#pragma once

#include <string>

#include "curve1d.h"

namespace acu {
namespace planning {

class StandingStillTrajectory1d : public math::Curve1d {
 public:
  StandingStillTrajectory1d(const double p, const double duration);

  virtual ~StandingStillTrajectory1d() = default;

  double ParamLength() const override;

  std::string ToString() const override;

  double Evaluate(const std::uint32_t order, const double param) const override;

 private:
  double Evaluate_s(const double t) const;

  double Evaluate_v(const double t) const;

  double Evaluate_a(const double t) const;

  double Evaluate_j(const double t) const;

 private:
  double fixed_position_;

  double duration_;
};

}  // namespace planning
}  // namespace acu
