/**
 * @file path_decider.h
 **/

#pragma once

#include <limits>
#include <string>

#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line_info.h"

namespace acu {
namespace planning { 

class FallBackPathDecider {
 public:
  FallBackPathDecider();
  ~FallBackPathDecider() = default;

  static void MakeStaticObstacleDecision(const PathInfo &path_data, 
     PathDecision *const path_decision, ReferenceLineInfo *reference_line_info) ;

 private:
  
  static ObjectStop GenerateObjectStopDecision(const Obstacle &path_obstacle,const PathInfo &path_data);
  static bool HasOverlap(const PathInfo &path_data, const Obstacle &path_obstacle);
  static bool FindLateralDecisionHistory(ObjectDecisionType& lat_desicion_last, const std::string& id);
  /**
   * @brief Calculate stop distance with the obstacle using the ADC's minimum
   * turning radius
   */
  

  bool has_stop_decision_history_;
};

}  // namespace planning
}  // namespace acu
