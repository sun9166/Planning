#ifndef COMMON_MAPCHECK_GEOTOOL_INCLUDE_COORTRANSFORM_H_
#define COMMON_MAPCHECK_GEOTOOL_INCLUDE_COORTRANSFORM_H_

#include "map/vectormap/include/geotool.h"
#include "common/base/macros.h"
#include "common/toolbox/geometry/include/geoheader.h"
#include "datapool/include/locperception_input.h"

using acu::vectormap::GeoTool;
using acu::vectormap::PointVCS;
using acu::vectormap::PointGCCS;
using geometry::Site;

namespace acu {
namespace planning {

class CoordTransform {
  BASE_DECLARE_SINGLETON(CoordTransform);
 public:
  CoordTransform();
  ~CoordTransform();
 public:
  bool GCCS2VCS(const Site &car_pos, const Site &target_pos, Site &vcs);
  bool GCCS2VCS(const LocalizationData &car_pos, const Site &target_pos, Site &vcs);
  bool GCCS2VCS(const PointGCCS &car_pos, const PointGCCS &target_pos, PointVCS &vcs);
  bool VCS2GCCS(const Site &car_pos, const Site &vcs, Site &target_pos);
  bool VCS2GCCS(const LocalizationData &car_pos, const Site &vcs, Site &target_pos);
  bool VCS2GCCS(const PointGCCS &car_pos, const PointVCS &vcs, PointGCCS &target_pos);

  // no test
  template < typename T, typename P>
  bool Frenet2Cartesian(const P& loc_data, const T& source,
                        const double l, T& vcs_site) {
    PointVCS vcs;
    PointGCCS gccs;
    PointGCCS base_point_gccs = {source.xg,
                                 source.yg,
                                 source.angle + loc_data.globalangle
                                };

    PointGCCS ego_pos = {loc_data.xg,
                         loc_data.yg,
                         loc_data.globalangle
                        };
    vcs.x = 0.0;
    vcs.y = l;
    geotool_.VCS2GCCS(base_point_gccs, vcs, gccs);
    geotool_.GCCS2VCS(ego_pos, gccs, vcs);
    vcs_site.x = vcs.x;
    vcs_site.y = vcs.y;
    vcs_site.angle = source.angle;
    return true;
  }
  
  // no test
  template <typename T, typename P>
  bool Cartesian2Frenet(
    const P& loc_data, const T& source,
    const T & vcs_site, double & l) {
    PointVCS vcs = {vcs_site.x, vcs_site.y, vcs_site.angle};
    PointGCCS gccs;
    PointGCCS base_point_gccs{source.xg,
                              source.yg,
                              source.angle + loc_data.globalangle};
    PointGCCS ego_pos = {loc_data.xg,
                         loc_data.yg,
                         loc_data.globalangle
                        };
    geotool_.VCS2GCCS(ego_pos, vcs, gccs);
    geotool_.GCCS2VCS(base_point_gccs, gccs, vcs);
    l = vcs.y;
    return true;
  }
 private:
  GeoTool geotool_;
};

}
}


#endif