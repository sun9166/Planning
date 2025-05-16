/**
 * @file path_optimizer.h
 **/

#pragma once

#include "pnc_point.pb.h"
#include "common/common_header/status.h"
#include "src/execution/motionplan/planner/procedure.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line.h"


namespace acu {
namespace planning {

class PathOptimizer : public Procedure {
 public:
  explicit PathOptimizer(const std::string &name);
  virtual ~PathOptimizer() = default;
  acu::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 protected:
  virtual acu::common::Status Process(
      const SpeedInfo &speed_data, const ReferenceLine &reference_line,
      const common::TrajectoryPoint &init_point, PathInfo *const path_data) = 0;

  void RecordDebugInfo(const PathInfo &path_data);
};

}  // namespace planning
}  // namespace acu
