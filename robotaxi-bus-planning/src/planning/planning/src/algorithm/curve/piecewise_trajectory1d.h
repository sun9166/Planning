/**
 * @file piecewise_trajectory1d.h
 **/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "curve1d.h"

namespace acu {
namespace planning {

class PiecewiseTrajectory1d : public math::Curve1d {
 public:
  PiecewiseTrajectory1d() = default;

  virtual ~PiecewiseTrajectory1d() = default;

  double Evaluate(const std::uint32_t order, const double param) const;

  double ParamLength() const;

  std::string ToString() const;

  void AppendSegment(const std::shared_ptr<Curve1d> trajectory);

  void PopSegment();

  size_t NumOfSegments() const;

 private:
  std::vector<std::shared_ptr<Curve1d>> trajectory_segments_;

  std::vector<double> accumulated_param_lengths_;
};

}  // namespace planning
}  // namespace acu
