/**
 * @file: st_graph_data.h
 * @brief: data with map info and obstacle info
 **/

#pragma once

#include <vector>

#include "pnc_point.pb.h"
#include "src/execution/motionplan/common/speed/speed_limit.h"
#include "src/execution/motionplan/common/speed/st_boundary.h"


namespace acu {
namespace planning {

class StGraphData {
 public:
  StGraphData(const std::vector<const StBoundary*>& st_boundaries,
              const acu::common::TrajectoryPoint& init_point,
              const SpeedLimit& speed_limit, const double path_data_length,
              const double cruise_speed);
  StGraphData() = default;

  const std::vector<const StBoundary*>& st_boundaries() const;

  const acu::common::TrajectoryPoint& init_point() const;

  const SpeedLimit& speed_limit() const;

  double path_data_length() const;

  double cruise_speed() const;

 private:
  std::vector<const StBoundary*> st_boundaries_;
  acu::common::TrajectoryPoint init_point_;

  SpeedLimit speed_limit_;
  double path_data_length_ = 0.0;
  double cruise_speed_;
};

}  // namespace planning
}  // namespace acu
