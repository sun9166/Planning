/**
 * @file publishable_trajectory.h
 **/

#pragma once
#include "motionplanning.pb.h"
#include "discretized_trajectory.h"
#include "src/execution/motionplan/common/datatype.h"

namespace acu {
namespace planning {

class PublishableTrajectory : public DiscretizedTrajectory {
 public:
  PublishableTrajectory() = default;

  PublishableTrajectory(const double header_time,
                        const DiscretizedTrajectory& discretized_trajectory);
  /**
   * Create a publishable trajectory based on a trajectory protobuf
   */
  explicit PublishableTrajectory(const ADCTrajectory& trajectory_pb);

  double header_time() const;

  void PopulateTrajectoryProtobuf(ADCTrajectory* trajectory_pb) const;

  void set_trajectory_type(
      const ADCTrajectory::TrajectoryType trajectory_type) {
    trajectory_type_ = trajectory_type;
  }

  ADCTrajectory::TrajectoryType trajectory_type() const {
    return trajectory_type_;
  }

 private:
  double header_time_ = 0.0;
  ADCTrajectory::TrajectoryType trajectory_type_ = ADCTrajectory::UNKNOWN;
};

}  // namespace planning
}  // namespace acu
