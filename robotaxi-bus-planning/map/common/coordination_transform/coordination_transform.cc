/******************************************************************************
 * Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
 *
 * NodeName: ivplanner
 * FileName: coordination_transform.cc
 *
 * Description: coordination transform function
 *
 * param:       GCCS: global cartesian coordinate system
 * param:       VCS : vehicle coordinate system
 *
 * History:
 * Feng younan          2018/12/06    1.5.1        build this model.
 ******************************************************************************/
#include "common/coordination_transform/coordination_transform.h"

CoordinationTransform::CoordinationTransform() {}

bool CoordinationTransform::Init(const double &x_offset, const double &y_offset) {
  instance() -> map_x_offset_ = x_offset;
  instance() -> map_y_offset_ = y_offset;
}

bool CoordinationTransform::GCCS2VCS (const Site &ego_pos, const Site &gccs, Site &vcs) {
  PointVCS  relative_pos;
  PointGCCS glob_pos;
  PointGCCS car_pos;

  car_pos.xg    = ego_pos.xg + map_x_offset_;
  car_pos.yg    = ego_pos.yg + map_y_offset_;
  car_pos.angle = ego_pos.globalangle;

  glob_pos.xg    = gccs.xg + map_x_offset_;
  glob_pos.yg    = gccs.yg + map_y_offset_;
  glob_pos.angle = gccs.globalangle;

  geotool_.GCCS2VCS(car_pos, glob_pos, relative_pos);

  vcs.x  = relative_pos.x;
  vcs.y  = relative_pos.y;
  vcs.angle  = relative_pos.angle;
  vcs.xg = glob_pos.xg;
  vcs.yg = glob_pos.yg;

  return true;
}

//reloaded by DZ for PointGCCS PointVCS form
int CoordinationTransform::GCCS2VCS (const PointGCCS &ego_pos, const PointGCCS &gccs, PointVCS &vcs) {
  PointVCS  relative_pos;
  PointGCCS glob_pos;
  PointGCCS car_pos;

  car_pos.xg    = ego_pos.xg + map_x_offset_;
  car_pos.yg    = ego_pos.yg + map_y_offset_;
  car_pos.angle = ego_pos.angle;

  glob_pos.xg    = gccs.xg + map_x_offset_;
  glob_pos.yg    = gccs.yg + map_y_offset_;
  glob_pos.angle = gccs.angle;

  int ret = geotool_.GCCS2VCS(car_pos, glob_pos, relative_pos);

  vcs.x  = relative_pos.x;
  vcs.y  = relative_pos.y;
  vcs.angle  = relative_pos.angle;
  return true;
}

bool CoordinationTransform::VCS2GCCS (const Site &ego_pos, const Site &vcs, Site &gccs) {
  PointVCS  relative_pos;
  PointGCCS glob_pos;
  PointGCCS car_pos;

  car_pos.xg    = ego_pos.xg + map_x_offset_;
  car_pos.yg    = ego_pos.yg + map_y_offset_;
  car_pos.angle = ego_pos.globalangle;

  relative_pos.x     = vcs.x;
  relative_pos.y     = vcs.y;
  relative_pos.angle = vcs.angle;

  geotool_.VCS2GCCS(car_pos, relative_pos, glob_pos);

  gccs.x     = relative_pos.x;
  gccs.y     = relative_pos.y;
  gccs.xg    = glob_pos.xg - map_x_offset_;
  gccs.yg    = glob_pos.yg - map_y_offset_;
  gccs.globalangle = glob_pos.angle;
  return true;
}

//reloaded by DZ for PointGCCS PointVCS form
int CoordinationTransform::VCS2GCCS (const PointGCCS &ego_pos, const PointVCS &vcs, PointGCCS &gccs) {
  PointVCS  relative_pos;
  PointGCCS glob_pos;
  PointGCCS car_pos;

  car_pos.xg    = ego_pos.xg + map_x_offset_;
  car_pos.yg    = ego_pos.yg + map_y_offset_;
  car_pos.angle = ego_pos.angle;

  relative_pos.x     = vcs.x;
  relative_pos.y     = vcs.y;
  relative_pos.angle = vcs.angle;

  int ret = geotool_.VCS2GCCS(car_pos, relative_pos, glob_pos);

  // gccs.x     = relative_pos.x;
  // gccs.y     = relative_pos.y;
  gccs.xg    = glob_pos.xg - map_x_offset_;
  gccs.yg    = glob_pos.yg - map_y_offset_;
  gccs.angle = glob_pos.angle;
  return ret;
}

bool CoordinationTransform::GPS2GCCS (const Site &ego_gps, Site &gccs) {
  PointGPS gps_pos;
  PointGCCS glob_pos;
  gps_pos.lon = ego_gps.yg;
  gps_pos.lat = ego_gps.xg;
  int result = geotool_.GPS2GCCS(gps_pos, glob_pos);
  if (-1 == result) {
    return false;
  }
  gccs.xg = glob_pos.xg - map_x_offset_;
  gccs.yg = glob_pos.yg - map_y_offset_;
  return true;
}

double CoordinationTransform::NormalizeAngle(double angle) {
  while (angle < -180) angle += 360;
  while (angle > 180) angle -= 360;
  return angle;
}

bool CoordinationTransform::Frenet2Cartesian2(const SiteVec & base_path,
  const SiteVec & vcs_path, SiteVec & reference_path) {
    reference_path.clear();
    for (int index = 0; index < base_path.size(); ++index) {
      PointVCS vcs;
      PointGCCS gccs;
      Site globalPoint;
      PointGCCS base_point_gccs{base_path[index].xg,
                                base_path[index].yg,
                                base_path[index].globalangle};

      vcs.x = vcs_path[index].x;
      vcs.y = vcs_path[index].y;
      vcs.angle = vcs_path[index].angle;
      geotool_.VCS2GCCS(base_point_gccs, vcs, gccs);
      globalPoint.xg = gccs.xg;
      globalPoint.yg = gccs.yg;
      globalPoint.globalangle = gccs.angle;
      // VCS2GCCS(base_point_gccs, vcs, gccs);

        // std::cout << "base_path 123 (" << base_path[index].xg << ", " << base_path[index].yg << ")" << std::endl;
        // std::cout << "base_point_gccs 123 (" << base_point_gccs.xg << ", " << base_point_gccs.yg << ")" << std::endl;
        // std::cout << "vcs 123 (" << vcs.x << ", " << vcs.y << ")" << std::endl;
        // std::cout << "gccs 123 (" << gccs.xg << ", " << gccs.yg << ")" << std::endl;
        // std::cout << "globalPoint 123 (" << globalPoint.xg << ", " << globalPoint.yg << ")" << std::endl;

      reference_path.push_back(globalPoint);
    }

  return true;
}

bool CoordinationTransform::Cartesian2Frenet2(const SiteVec & base_path,
  const SiteVec & reference_path, SiteVec & vcs_path) {
    vcs_path.clear();
    for (int index = 0; index < base_path.size(); ++index) {
      PointVCS vcs;
      Site vcsPoint;
      PointGCCS base_point_gccs{base_path[index].xg + map_x_offset_,
                                base_path[index].yg + map_y_offset_,
                                base_path[index].globalangle};
      PointGCCS gccs{
        reference_path[index].xg + map_x_offset_,
        reference_path[index].yg + map_y_offset_,
        reference_path[index].globalangle
      };
      geotool_.GCCS2VCS(base_point_gccs, gccs, vcs);
      vcsPoint.x  = 0;
      vcsPoint.y  = vcs.y;
      vcsPoint.xg = gccs.xg;
      vcsPoint.yg = gccs.yg;
      vcsPoint.angle = vcs.angle;
      vcs_path.push_back(vcsPoint);
    }
  return true;
}
