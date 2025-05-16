/**
 * @file linear_interpolation.h
 * @brief Linear interpolation functions.
 */

#pragma once

#include <cmath>

#include "pnc_point.pb.h"

using acu::common::SLPoint;

namespace acu {
namespace planning {
namespace math {

SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
                                            const SLPoint &p1, const double w);

common::PathPoint InterpolateUsingLinearApproximation(const common::PathPoint &p0,
                                              const common::PathPoint &p1,
                                              const double s);
common::TrajectoryPoint InterpolateUsingLinearApproximation(const common::TrajectoryPoint &tp0,
                                                    const common::TrajectoryPoint &tp1,
                                                    const double t);

}  // namespace math
}  // namespace planning
}  // namespace acu

