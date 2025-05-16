/**
 * @file motionplan.h
 **/
#pragma once

#include <functional>
#include <thread>

#include "planning_config.pb.h"

#include "common/datatype.h"
#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/planner/path_planner.h"
#include "src/execution/motionplan/planner/speed_planner.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line.h"
#include "src/execution/motionplan/trajectory_stitcher/trajectory_stitcher.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line_info.h"
#include "src/execution/motionplan/common/trajectory/publishable_trajectory.h"
#include "src/execution/motionplan/reference_line_provider/reference_line_provider.h"

namespace acu {
namespace planning {
class MotionPlan {
public:
  MotionPlan();
  ~MotionPlan();
  int Init();
  void Process();
  void Reset();

protected:

  acu::common::Status InitMotionPlanFrame(const uint32_t sequence_num,
                           const std::vector<common::TrajectoryPoint>& stitching_trajectory,
                           const std::vector<common::TrajectoryPoint>& stitching_trajectory_speedplan,
                           const double start_time,
                           const VehicleState& vehicle_state);
  void GenerateStopTrajectory();
  void GenerateTrajectory();
  void DebugTrajectory(const PathData& output_trajectory);
  void GenerateTrajectoryBasedCurrentReferenceLine();
  void AddDebugInfo(const int replan_reason) const;
  double CalculateSteeringAngleRateMax(const std::vector<std::pair<double ,double>>& kappa_t_pairs);
  void ManualDriveReset();
 protected:
  bool reverse_flag_ = false;
  size_t seq_num_;
  
  std::unique_ptr<PathPlanner> path_planner_;
  std::unique_ptr<SpeedPlanner> speed_planner_;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
  PlanningConfig pathplan_config_;
  PlanningConfig speedplan_config_;
  std::unique_ptr<Frame> frame_;
  std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;
  int last_publishable_trajectory_type_ = 0 ;//0:origin path 1:new path
  ReferenceLineFrame local_reference_line_;

};

}  // namespace planning
}  // namespace acu
