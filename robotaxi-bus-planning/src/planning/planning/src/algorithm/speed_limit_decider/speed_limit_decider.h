/**
 *   @file speed_limit_decider.h
 **/

#pragma once

#include <string>
#include <vector>

#include "st_boundary_config.pb.h"
#include "common/common_header/status.h"
#include "src/execution/motionplan/common/obstacle/obstacle.h"
#include "src/execution/motionplan/common/path/path_info.h"
#include "src/execution/motionplan/common/speed/speed_limit.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line.h"

namespace acu {
namespace planning {

class SpeedLimitDecider {
 public:
  SpeedLimitDecider(const SLBoundary& adc_sl_boundary,
                    const StBoundaryConfig& config,
                    const ReferenceLine& reference_line,
                    const PathInfo& path_data,
                    const bool& has_overtake_decision);

  virtual ~SpeedLimitDecider() = default;

  virtual acu::common::Status GetSpeedLimits(
      const IndexedList<std::string, Obstacle>& obstacles,
      SpeedLimit* const speed_limit_data) const;

 private:

  double GetCentricAccLimit(const double kappa) const;
  double GetUncertainObstacleNudgeSpeedLimit(
       const Obstacle* obstacle_ptr, const double car_s, const double car_l) const;
  double GetMovebleObstacleSpeedLimit(
       const Obstacle* obstacle_ptr, const double car_s, const double car_l) const;
  double GetMissionEndSpeedLimit(const IndexedList<std::string, Obstacle>& obstacles,
               const double& mission_end_s, const double point_s, const double point_l) const;

  void GetAvgKappa(const std::vector<common::PathPoint>& path_points,
                   std::vector<double>* kappa) const;

 private:
  const SLBoundary& adc_sl_boundary_;
  const StBoundaryConfig& st_boundary_config_;
  const ReferenceLine& reference_line_;
  const PathInfo& path_data_;
  CarModel vehicle_param_;
  bool has_overtake_decision_ = false;
};

}  // namespace planning
}  // namespace acu
