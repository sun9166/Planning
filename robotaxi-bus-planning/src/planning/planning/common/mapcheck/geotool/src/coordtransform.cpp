#include "common/mapcheck/geotool/include/coordtransform.h"

namespace acu {
namespace planning {

CoordTransform::CoordTransform() {}
CoordTransform::~CoordTransform() {}

bool CoordTransform::GCCS2VCS(const Site &car_pos, const Site &target_pos, Site &vcs) {
  PointGCCS p1, p2; 
  PointVCS p3;
  p1.xg = car_pos.xg;
  p1.yg = car_pos.yg;
  p1.angle = car_pos.globalangle;
  p2.xg = target_pos.xg;
  p2.yg = target_pos.yg;
  p2.angle = target_pos.globalangle;
  int ret = geotool_.GCCS2VCS(p1, p2, p3);
  vcs.x = p3.x;
  vcs.y = p3.y;
  vcs.angle = p3.angle;
  vcs.xg = target_pos.xg;
  vcs.yg = target_pos.yg;
  if (ret == -1) return false;
  return true;
}

bool CoordTransform::GCCS2VCS(const LocalizationData &car_pos, const Site &target_pos, Site &vcs) {
  PointGCCS p1, p2; 
  PointVCS p3;
  p1.xg = car_pos.xg;
  p1.yg = car_pos.yg;
  p1.angle = car_pos.yaw;
  p2.xg = target_pos.xg;
  p2.yg = target_pos.yg;
  p2.angle = target_pos.globalangle;
  int ret = geotool_.GCCS2VCS(p1, p2, p3);
  vcs.x = p3.x;
  vcs.y = p3.y;
  vcs.angle = p3.angle;
  vcs.xg = target_pos.xg;
  vcs.yg = target_pos.yg;
  if (ret == -1) return false;
  return true;
}

bool CoordTransform::GCCS2VCS(const PointGCCS &car_pos, const PointGCCS &target_pos, PointVCS &vcs) {
  int ret = geotool_.GCCS2VCS(car_pos, target_pos, vcs);
  if (ret == -1) return false;
  return true;
}

bool CoordTransform::VCS2GCCS(const Site &car_pos, const Site &vcs, Site &target_pos) {
  PointGCCS p1, p2; 
  PointVCS p3;
  p1.xg = car_pos.xg;
  p1.yg = car_pos.yg;
  p1.angle = car_pos.globalangle;
  p3.x = vcs.x;
  p3.y = vcs.y;
  int ret = geotool_.VCS2GCCS(p1, p3, p2);
  target_pos.xg = p2.xg;
  target_pos.yg = p2.yg;
  target_pos.globalangle = p2.angle;
  target_pos.x = p3.x;
  target_pos.y = p3.y;
  if (ret == -1) return false;
  return true;
}

bool CoordTransform::VCS2GCCS(const LocalizationData &car_pos, const Site &vcs, Site &target_pos) {
  PointGCCS p1, p2; 
  PointVCS p3;
  p1.xg = car_pos.xg;
  p1.yg = car_pos.yg;
  p1.angle = car_pos.yaw;
  p3.x = vcs.x;
  p3.y = vcs.y;
  int ret = geotool_.VCS2GCCS(p1, p3, p2);
  target_pos.xg = p2.xg;
  target_pos.yg = p2.yg;
  target_pos.globalangle = p2.angle;
  target_pos.x = p3.x;
  target_pos.y = p3.y;
  if (ret == -1) return false;
  return true;
}

bool CoordTransform::VCS2GCCS(const PointGCCS &car_pos, const PointVCS &vcs, PointGCCS &target_pos) {
  int ret = geotool_.VCS2GCCS(car_pos, vcs, target_pos);
  if (ret == -1) return false;
  return true;
}

} // namespace planning
} // namespace acu