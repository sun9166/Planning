#pragma once

#include <utility>
#include <vector>

#include "fem_pos_deviation_smoother_config.pb.h"
#include "reference_line_smoother.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line.h"
#include "src/execution/motionplan/common/reference_line_info/reference_point.h"

namespace acu {
namespace planning {

class DiscretePointsReferenceLineSmoother : public ReferenceLineSmoother {
 public:
  explicit DiscretePointsReferenceLineSmoother(
      const ReferenceLineSmootherConfig& config);

  virtual ~DiscretePointsReferenceLineSmoother() = default;

  bool Smooth(const ReferenceLine& raw_reference_line,
              ReferenceLine* const smoothed_reference_line) override;

  void SetAnchorPoints(const std::vector<AnchorPoint>&) override;
 private:

  bool FemPosSmooth(
      const std::vector<std::pair<double, double>>& raw_point2d,
      const std::vector<double>& bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  bool GenerateRefPointProfile(
      const ReferenceLine& raw_reference_line,
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<ReferencePoint>* reference_points);

  std::vector<AnchorPoint> anchor_points_;

  double zero_x_ = 0.0;

  double zero_y_ = 0.0;
};

}  // namespace planning
}  // namespace acu
