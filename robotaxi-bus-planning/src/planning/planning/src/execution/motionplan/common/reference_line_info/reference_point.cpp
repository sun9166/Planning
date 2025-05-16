/**
 * @file reference_point.cpp
 **/

#include "reference_point.h"
#include "common/util/util.h"
#include "common/util/string_util.h"

namespace acu {
namespace planning {

using acu::common::util::StrCat;

namespace {
// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;
}  // namespace

ReferencePoint::ReferencePoint(const MapPathPoint& map_path_point,const double kappa, const double dkappa)
    : hdmap::MapPathPoint(map_path_point), kappa_(kappa), dkappa_(dkappa){}

common::PathPoint ReferencePoint::ToPathPoint(double s) const {
  common::PathPoint path_point = common::util::MakePathPoint(
      x(), y(), 0.0, heading(), kappa_, dkappa_, 0.0);
  path_point.set_s(s);
  return path_point;
}

double ReferencePoint::kappa() const { return kappa_; }

double ReferencePoint::dkappa() const { return dkappa_; }

void ReferencePoint::RemoveDuplicates(std::vector<ReferencePoint>* points) {
  CHECK_NOTNULL(points);
  int count = 0;
  const double limit = kDuplicatedPointsEpsilon; 
  size_t points_num = points->size();
  for (size_t i = 0; i < points_num; ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } else {
      (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    }
  }
  points->resize(count);
}

}  // namespace planning
}  // namespace acu
