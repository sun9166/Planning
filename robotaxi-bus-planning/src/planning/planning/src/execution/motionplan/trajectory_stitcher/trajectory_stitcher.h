/**
 * @file trajectory_stitcher.h
 **/

#pragma once

#include <string>
#include <utility>
#include <vector>
//#include "ros/ros.h"
#include "pnc_point.pb.h"
#include "../common/trajectory/publishable_trajectory.h"
#include "src/execution/motionplan/common/datatype.h"
#include "referenceline_frame/sl_boundary/sl_boundary.h"

namespace acu {
namespace planning {
namespace trajectory_stitcher {

std::vector<common::TrajectoryPoint> ComputeStitchingTrajectory(
      const VehicleState& vehicle_state, const VehicleState& vehicle_state_latest,
      const double current_timestamp, const double planning_cycle_time,
      const PublishableTrajectory* prev_trajectory,
      int* replan_reason);

std::vector<common::TrajectoryPoint> ComputeStitchingTrajectoryForSpeedPlan(
      const VehicleState& vehicle_state, const VehicleState& vehicle_state_latest,
      const double current_timestamp, const double planning_cycle_time,
      const PublishableTrajectory* prev_trajectory,
      int* replan_reason);

std::vector<common::TrajectoryPoint> ComputeStitchingTrajectoryForPathPlan(
      const VehicleState& vehicle_state, const VehicleState& vehicle_state_latest,
      const double current_timestamp, const double planning_cycle_time,
      const PublishableTrajectory* prev_trajectory, const ReferenceLineFrame& local_reference_line,
      int* replan_reason);

void FixStitchingTrajectoryByDR(const VehicleState& vehicle_state,
    const VehicleState& vehicle_state_latest,
    std::vector<common::TrajectoryPoint>& stitching_trajectory);

//private
std::vector<common::TrajectoryPoint> ComputeReinitStitchingTrajectory(
      const VehicleState& vehicle_state);

std::vector<common::TrajectoryPoint> ComputeStitchingTrajectoryReInit(
     const VehicleState& vehicle_state, const common::TrajectoryPoint& reinit_point); 

std::vector<common::TrajectoryPoint> ConstLengthStitchingTrajectory(const VehicleState& vehicle_state,
         const PublishableTrajectory* prev_trajectory, const size_t time_matched_index, const size_t position_matched_index,
         const double length, const double veh_rel_time);

std::vector<common::TrajectoryPoint> ComputeReplanStartPoint(
            const VehicleState& vehicle_state,const common::TrajectoryPoint& matched_point);
std::vector<common::TrajectoryPoint> ComputeReplanStartPoint(
            const PublishableTrajectory* prev_trajectory, const VehicleState& vehicle_state, 
            const VehicleState& vehicle_state_new);
double GetOptionalKeepLength(
            const PublishableTrajectory* prev_trajectory, const std::map<int, LineObject>& obstacles,
            const double& car_speed, const double car_s_in_pre_traj);

}
}  // namespace planning
}  // namespace acu
