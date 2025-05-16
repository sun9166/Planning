/**
 * @file path_decider.h
 **/

#pragma once

#include <limits>
#include <string>

#include "src/execution/motionplan/planner/procedure.h"

namespace acu {
namespace planning { 

class PathDecider : public Procedure {
 public:
  PathDecider();
  ~PathDecider() = default;

  acu::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:

  void AddTerminalVirtualObstacle(const PathInfo &path_data);

  acu::common::Status Process(const PathInfo &path_data,
                                 PathDecision *const path_decision);

  bool MakeObjectDecision(const PathInfo &path_data,
                          PathDecision *const path_decision);

  bool MakeStaticObstacleDecision(const PathInfo &path_data,
                                  PathDecision *const path_decision);

  ObjectStop GenerateObjectStopDecision(
      const Obstacle &path_obstacle) const;

  bool FindLongitudinalDecisionHistory(ObjectDecisionType& lon_desicion_last, const std::string& id) const;
  bool FindLateralDecisionHistory(ObjectDecisionType& lat_desicion_last, const std::string& id) const;

  /**
   * @brief Calculate stop distance with the obstacle using the ADC's minimum
   * turning radius
   */
  

  bool has_stop_decision_history_;
};

}  // namespace planning
}  // namespace acu
