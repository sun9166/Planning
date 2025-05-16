
/**
 * @file
 **/

#pragma once

#include <vector>

#include "reference_line_smoother_config.pb.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line.h"

namespace acu {
namespace planning {

struct AnchorPoint {
  common::PathPoint path_point;
  double lateral_bound = 0.0;
  double longitudinal_bound = 0.0;
  // enforce smoother to strictly follow this reference point
  bool enforced = false;
};

class ReferenceLineSmoother {
 public:
  explicit ReferenceLineSmoother(const ReferenceLineSmootherConfig& config)
      : config_(config) {}

  /**
   * Smoothing constraints
   */
  virtual void SetAnchorPoints(
      const std::vector<AnchorPoint>& achor_points) = 0;

  /**
   * Smooth a given reference line
   */
  virtual bool Smooth(const ReferenceLine&, ReferenceLine* const) = 0;

  virtual ~ReferenceLineSmoother() = default;

 protected:
  ReferenceLineSmootherConfig config_;
};

}  // namespace planning
}  // namespace acu
