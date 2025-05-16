#ifndef COMMON_MAPCHECK_FREESPACE_INCLUDE_FREESPACE_H_
#define COMMON_MAPCHECK_FREESPACE_INCLUDE_FREESPACE_H_

#include "map/vectormap/include/geotool.h"
#include "common/base/macros.h"
#include "common/toolbox/geometry/include/geoheader.h"
#include "map/map_loader/include/map_loader.h"
#include "datapool/include/cognition_typedef.h"
#include "datapool/include/common_config.h"

using acu::vectormap::PointVCS;
using acu::vectormap::PointGCCS;
using acu::map::BaseMap;
using geometry::Site;

namespace acu {
namespace planning {

class Freespace {
  BASE_DECLARE_SINGLETON(Freespace);
 public:
  Freespace();
  ~Freespace();
 public:
  void Init();
  bool Check();
  bool IsInFreespace(const Site &car_pos);
  bool IsInFreespace(const Site &car_pos, const Site &vcs_pos);
  bool IsInFreespace(const PointGCCS &car_pos);
  bool IsInFreespace(const PointGCCS &car_pos, const PointVCS &vcs_pos);

  bool MODLFSCheck(const Site &gccs, const CarModel &car_model);
  bool MODLPathFSCheck(const std::list<Site> &gccs_list,
                       const CarModel &car_model,
                       const double &check_dis,
                       CollisionInfo &collision_info);
 private:
  BaseMap *basemap_;
};

}
}


#endif