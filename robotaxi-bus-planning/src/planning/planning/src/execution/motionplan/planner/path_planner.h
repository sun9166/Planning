/**
 * @file path_planner.h
 **/
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "pnc_point.pb.h"
#include "motionplanning.pb.h"

#include "planner.h"
#include "common/util/factory.h"
#include "common/common_header/status.h"
#include "src/execution/motionplan/planner/procedure.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line.h"
#include "src/execution/motionplan/common/reference_line_info/reference_point.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line_info.h"
#include "src/execution/motionplan/common/trajectory/publishable_trajectory.h"

/**
 * @namespace acu::planning
 * @brief acu::planning
 */
namespace acu {
namespace planning {

/**
 * @class PathPlanner
 * @brief PathPlanner is an expectation maximization planner.
 */

class PathPlanner : public Planner {
 public:
  /**
   * @brief Constructor
   */
  PathPlanner() = default;

  /**
   * @brief Destructor
   */
  virtual ~PathPlanner() = default;

  common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  acu::common::Status Plan(
      const common::TrajectoryPoint& planning_init_point,
      Frame* frame) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) override;

 private:
  void RegisterTasks();
  void GetDRTrajectory(PathInfo* pathdata,
      const VehicleState* vehicle_state_ptr);
  acu::common::util::Factory<TaskType, Procedure> task_factory_;
  std::vector<std::unique_ptr<Procedure>> tasks_;
};

}  // namespace planning
}  // namespace acu
