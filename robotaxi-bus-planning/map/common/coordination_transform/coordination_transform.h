/******************************************************************************
 * Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
 *
 * NodeName: ivplanner
 * FileName:  coordination_transform.h
 *
 * Description: coordination transform function
 *
 * param:       GCCS: global cartesian coordinate system
 * param:       VCS : vehicle coordinate system
 *
 * History:
 * Feng younan          2018/12/06    1.5.1        build this model.
 ******************************************************************************/
#ifndef __IVPATHPLANNER_COORDINATION_TRANSFORM_H__
#define __IVPATHPLANNER_COORDINATION_TRANSFORM_H__

#include "map/vectormap/include/geotool.h"
#include "map/common/macro.h"
#include "map/common/geometry/path.h"
#include "map/common/geometry/geoheader.h"

using geometry::Path;
using geometry::Site;
using geometry::SiteVec;
using acu::vectormap::GeoTool;
using acu::vectormap::PointVCS;
using acu::vectormap::PointGCCS;
using acu::vectormap::PointGPS;

class CoordinationTransform {
  DECLARE_SINGLETON(CoordinationTransform);
public:
  bool Init(const double &x_offset, const double &y_offset);
  /**
   * @brief  check if this apdapter working
   * @param  none
   * @return true if ok, false otherwise
  **/
  // bool Check();
  /**
   * @brief  coordination transform function
   * @param  gccs: glob cartesian coordinate system
   * @param  vcs : vehicle coordinate system
   * @return true if ok, false otherwise
  **/
  bool GCCS2VCS (const Site &ego_pos, const Site &gccs, Site &vcs);
  int  GCCS2VCS (const PointGCCS &ego_pos, const PointGCCS &gccs, PointVCS &vcs);
  bool VCS2GCCS (const Site &ego_pos, const Site &vcs, Site &gccs);
  int  VCS2GCCS (const PointGCCS &ego_pos, const PointVCS &vcs, PointGCCS &gccs);
  bool GPS2GCCS (const Site &ego_gps, Site &gccs);

  // @add by ljm
  bool Frenet2Cartesian2(const SiteVec& base_path, const SiteVec& vcs_path, SiteVec& reference_path);
  bool Cartesian2Frenet2(const SiteVec& base_path, const SiteVec& reference_path, SiteVec &vcs_path);


private:
  double NormalizeAngle(double angle);
private:
  GeoTool  geotool_;
  double map_x_offset_;
  double map_y_offset_;
};

#endif // __IVPATHPLANNER_COORDINATION_TRANSFORM_H__

