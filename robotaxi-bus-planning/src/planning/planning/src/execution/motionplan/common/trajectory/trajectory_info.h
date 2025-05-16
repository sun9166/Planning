/**
 * @file
 **/

#pragma once

#include "discretized_trajectory.h"
#include "src/execution/motionplan/common/path/path_info.h"
#include "src/execution/motionplan/common/speed/speed_info.h"

namespace acu {
namespace planning {

class TrajectoryInfo {
 public:
  TrajectoryInfo() = default;
  ~TrajectoryInfo() = default;

  const PathInfo& path_data() const { return path_data_; }
  PathInfo* mutable_path_data() { return &path_data_; }

  const SpeedInfo& speed_data() const { return speed_data_; }
  SpeedInfo* mutable_speed_data() { return &speed_data_; }

  const DiscretizedTrajectory& discretized_trajectory() const {
    return discretized_trajectory_;
  }
  DiscretizedTrajectory* mutable_discretized_trajectory() {
    return &discretized_trajectory_;
  }

 private:
  PathInfo path_data_;
  SpeedInfo speed_data_;
  DiscretizedTrajectory discretized_trajectory_;
};

}  // namespace planning
}  // namespace acu
