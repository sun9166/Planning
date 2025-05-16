/**
 * @file speed_profile_generator.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "pnc_point.pb.h"

#include "src/execution/motionplan/common/speed/speed_info.h"
#include "src/algorithm/curve/quintic_polynomial_curve1d.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line_info.h"

namespace acu {
namespace planning {
namespace speed_profile_generator {

std::vector<common::SpeedPoint> GenerateInitSpeedProfile(
      const common::TrajectoryPoint& planning_init_point,
      const ReferenceLineInfo* reference_line_info);

std::vector<common::SpeedPoint> GenerateSpeedHotStart(
      const common::TrajectoryPoint& planning_init_point,const double cruise_speed);

SpeedInfo GenerateFallbackSpeedProfile(ReferenceLineInfo* reference_line_info);

SpeedInfo GenerateStopProfile(const double init_speed,
                                       const double init_acc);

bool IsValidProfile(const math::QuinticPolynomialCurve1d& curve);

double CalculateStopDistance(
    ReferenceLineInfo* reference_line_info, const double init_s);
SpeedInfo GenerateStopSpeedProfile(
          const double stop_point_s,const double init_speed,const double init_acc,const bool& is_must_decel);

}
}  // namespace planning
}  // namespace acu
