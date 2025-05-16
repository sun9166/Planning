/******************************************************************************
* Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: common
* FileName: path.h
*
* Description: a construct of path with Site
*
* History:
* Feng younan          2018/08/20          v1.0.0        Build this model.
******************************************************************************/
#ifndef __COMMON_GEOMETRY_PATH_H__
#define __COMMON_GEOMETRY_PATH_H__

#include "geoheader.h"
#include "site.h"

namespace geometry {

class Path {
 public:
  Path() {
    points.clear();
    forward_flag = true;
  }
  Path(SiteVec &vec) {
    points = vec;
    forward_flag = true;
  }
  ~Path(){};

 public:
  void add(double x, double y) { points.push_back(Site(x, y)); }

 public:
  SiteVec points;
  bool forward_flag;
};

}  // geometry
#endif  // __COMMON_GEOMETRY_PATH_H__