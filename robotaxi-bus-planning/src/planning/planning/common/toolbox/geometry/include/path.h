
#ifndef COMMON_TOOLBOX_GEOMETRY_INCLUDE_PATH_H__
#define COMMON_TOOLBOX_GEOMETRY_INCLUDE_PATH_H__

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