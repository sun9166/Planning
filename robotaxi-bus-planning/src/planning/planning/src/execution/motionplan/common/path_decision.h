/**
 * @file path_boundary.h
 **/

#pragma once

#include <limits>
#include <string>
#include <mutex> 
#include "decision.pb.h"
#include "indexed_list.h"
#include "planning_gflags.h"
#include "src/execution/motionplan/common/obstacle/obstacle.h"

namespace acu {
namespace planning {

/**
 * @class PathDecision
 *
 * @brief PathDecision represents all obstacle decisions on one path.
 */
class PathDecision {
 public:
  PathDecision() = default;

  Obstacle *AddObstacle(const Obstacle &obstacle);
  Obstacle *AddWholeObstacle(const Obstacle &obstacle);

  const IndexedList<std::string, Obstacle> &obstacles() const;
  const IndexedList<std::string, Obstacle> &whole_obstacles() const;

  bool AddLateralDecision(const std::string &tag, const std::string &object_id,
                          const ObjectDecisionType &decision);
  bool AddLongitudinalDecision(const std::string &tag,
                               const std::string &object_id,
                               const ObjectDecisionType &decision);

  const Obstacle *Find(const std::string &object_id) const;

  Obstacle *Find(const std::string &object_id);

  void SetStBoundary(const std::string &id, const StBoundary &boundary);
  void SetNudgeBuffer(const std::string &id, const double &buffer);
  void EraseStBoundaries();
  MainStop main_stop() const { return main_stop_; }
  double stop_reference_line_s() const { return stop_reference_line_s_; }
  bool MergeWithMainStop(const ObjectStop &obj_stop, const std::string &obj_id,
                         const ReferenceLine &ref_line,
                         const SLBoundary &adc_sl_boundary);
  const Obstacle *MainFocusObstacle() const {
    return main_focus_obstacle_;
  }

  const Obstacle *MainYieldObstacle() const {
    return main_yield_obstacle_;
  }

  bool GetMainFocusObstacle();
  bool GetMainYieldObstacle();  

  bool has_overtake_obstacle() const;

 private:
  std::mutex obstacle_mutex_;
  IndexedList<std::string, Obstacle> obstacles_, whole_obstacles_;
  MainStop main_stop_;
  double stop_reference_line_s_ = std::numeric_limits<double>::max();
  const Obstacle* main_focus_obstacle_ = nullptr;
  const Obstacle* main_yield_obstacle_ = nullptr;
};

}  // namespace planning
}  // namespace acu
