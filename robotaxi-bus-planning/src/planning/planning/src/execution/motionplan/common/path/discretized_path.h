/**
 * @file discretized_path.h
 **/

#pragma once

#include <vector>

#include "motionplanning.pb.h"
#include "pnc_point.pb.h"
#include "src/execution/motionplan/common/planning_gflags.h"
namespace acu {
namespace planning {

class DiscretizedPath : public std::vector<common::PathPoint> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(const std::vector<common::PathPoint>& path_points);

  double Length() const;

  common::PathPoint Evaluate(const double path_s) const;

  std::vector<common::PathPoint>::const_iterator QueryLowerBound(
      const double path_s) const;
};

}  // namespace planning
}  // namespace acu
