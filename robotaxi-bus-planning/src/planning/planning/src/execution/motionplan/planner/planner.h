/**
 * @file planner.h
 **/
#pragma once

#include "pnc_point.pb.h"
#include "planning_config.pb.h"
#include "common/common_header/status.h"
#include "src/execution/motionplan/common/frame/frame.h"

/**
 * @namespace acu::planning
 * @brief acu::planning
 */
namespace acu {
namespace planning {

/**
 * @class Planner
 * @brief Planner is a base class for specific planners.
 *        It contains a pure virtual function Plan which must be implemented in
 * derived class.
 */
class Planner {
 public:
  /**
   * @brief Constructor
   */
  Planner() = default;

  /**
   * @brief Destructor
   */
  virtual ~Planner() = default;

  virtual acu::common::Status Init(const PlanningConfig& config) = 0;
//   virtual acu::common::Status Init() = 0;

  /**
   * @brief Compute trajectories for execution.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  virtual acu::common::Status Plan(
      const common::TrajectoryPoint& planning_init_point, Frame* frame) = 0;

  /**
   * @brief Compute a trajectory for execution.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  virtual acu::common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) = 0;
};

}  // namespace planning
}  // namespace acu
