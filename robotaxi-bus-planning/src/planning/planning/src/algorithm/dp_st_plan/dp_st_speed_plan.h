/**
 * @file dp_st_speed_optimizer.h
 **/

#pragma once

#include <string>

#include "dp_st_speed_config.pb.h"
#include "st_boundary_config.pb.h"
#include "src/algorithm/optimizer/speed_optimizer.h"
#include "src/algorithm/speed_limit_decider/speed_limit_decider.h"
#include "src/algorithm/st_boundary_mapper/st_boundary_mapper.h"

namespace acu {
namespace planning {

/**
 * @class DpStSpeedPlan
 * @brief DpStSpeedPlan does ST graph speed planning with dynamic
 * programming algorithm.
 */
class DpStSpeedPlan : public SpeedOptimizer {
 public:
  DpStSpeedPlan();
  bool Init(const PlanningConfig& config) override;

 private:
  acu::common::Status Process(const SLBoundary& adc_sl_boundary,
                                 const PathInfo& path_data,
                                 const common::TrajectoryPoint& init_point,
                                 const ReferenceLine& reference_line,
                                 const SpeedInfo& reference_speed_data,
                                 PathDecision* const path_decision,
                                 SpeedInfo* const speed_data) override;

  bool SearchStGraph(const StBoundaryMapper& boundary_mapper,
                     const SpeedLimitDecider& speed_limit_decider,
                     const PathInfo& path_data, SpeedInfo* speed_data,
                     PathDecision* path_decision,
                     planning_internal::STGraphDebug* debug) const;

 private:
  common::TrajectoryPoint init_point_;
  const ReferenceLine* reference_line_ = nullptr;
  SLBoundary adc_sl_boundary_;
  DpStSpeedConfig dp_st_speed_config_;
  StBoundaryConfig st_boundary_config_;
};

}  // namespace planning
}  // namespace acu
