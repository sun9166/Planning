/**
 * @file speed_optimizer.h
 **/

#pragma once

#include "common/common_header/status.h"
#include "src/execution/motionplan/planner/procedure.h"
#include "src/execution/motionplan/common/speed/speed_info.h"
#include "src/execution/motionplan/common/st_graph/st_graph_data.h"

namespace acu {
namespace planning {

class SpeedOptimizer : public Procedure {
 public:
  explicit SpeedOptimizer(const std::string& name);
  virtual ~SpeedOptimizer() = default;
  acu::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

 protected:
  virtual acu::common::Status Process(
      const SLBoundary& adc_sl_boundary, const PathInfo& path_data,
      const common::TrajectoryPoint& init_point,
      const ReferenceLine& reference_line,
      const SpeedInfo& reference_speed_data, PathDecision* const path_decision,
      SpeedInfo* const speed_data) = 0;

  void RecordSTGraphDebug(const StGraphData& st_graph_data,
                          planning_internal::STGraphDebug* stGraphDebug) const;

  void RecordDebugInfo(const SpeedInfo& speed_data);
};

}  // namespace planning
}  // namespace acu
