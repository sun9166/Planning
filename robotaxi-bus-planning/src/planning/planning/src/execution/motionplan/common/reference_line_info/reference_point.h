/**
 * @file reference_point.h
 **/

#pragma once

#include <string>
#include <vector>

#include "pnc_point.pb.h"
#include "src/execution/motionplan/common/path/path.h"

namespace acu {
namespace planning {

class ReferencePoint : public hdmap::MapPathPoint {
 public:
  ReferencePoint() = default;

  ReferencePoint(const MapPathPoint& map_path_point, const double kappa,const double dkappa);

  common::PathPoint ToPathPoint(double s) const;

  double kappa() const;
  double dkappa() const;

  static void RemoveDuplicates(std::vector<ReferencePoint>* points);

 private:
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
};

}  // namespace planning
}  // namespace acu
