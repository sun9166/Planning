/**
 * @file: st_graph_data.cpp
 **/

#include "st_graph_data.h"

namespace acu {
namespace planning {

using acu::common::TrajectoryPoint;

StGraphData::StGraphData(const std::vector<const StBoundary*>& st_boundaries,
                         const TrajectoryPoint& init_point,
                         const SpeedLimit& speed_limit,
                         const double path_data_length,
                         const double cruise_speed)
    : st_boundaries_(st_boundaries),
      init_point_(init_point),
      speed_limit_(speed_limit),
      path_data_length_(path_data_length),
      cruise_speed_(cruise_speed) {}

const std::vector<const StBoundary*>& StGraphData::st_boundaries() const {
  return st_boundaries_;
}

const TrajectoryPoint& StGraphData::init_point() const { return init_point_; }

const SpeedLimit& StGraphData::speed_limit() const { return speed_limit_; }

double StGraphData::path_data_length() const { return path_data_length_; }

double StGraphData::cruise_speed() const { return cruise_speed_; }

}  // namespace planning
}  // namespace acu
