/**
 * @file discrete_points_math.h
 **/

#pragma once

#include <utility>
#include <vector>

namespace acu {
namespace planning {
namespace discrete_points_math {
bool ComputeDiscretePathProfile(
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<double>* headings, std::vector<double>* accumulated_s,
      std::vector<double>* kappas, std::vector<double>* dkappas);

}
}  // namespace planning
}  // namespace acu
