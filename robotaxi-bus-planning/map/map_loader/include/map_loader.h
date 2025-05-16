#ifndef MAP_LOADER_H_
#define MAP_LOADER_H_

#include <memory>
#include <mutex>
#include <string>

#include "map/vectormap/include/vectormap.h"
#include "map/vectormap/src/hdmap/hdmap_common.h"
#include "map/xmlmap/include/xmlmap.h"
#include "map/basemap/include/basemap.h"
#include "map/map_loader/include/app_map.h"
#include "map/map_loader/include/link_map.h"
#include "map/map_loader/include/map_info.h"

using namespace acu::xmlmap;
using namespace acu::vectormap;
using namespace acu::hdmap;

namespace acu {
namespace map {

std::vector<std::string> VectorMapFile();

class MapLoader {
 public:
  static VectorMap* GetVectorMapPtr();
  static HDMapImpl* GetHDMapImplPtr();
  static LinkMap* GetLinkmapPtr();
  static BaseMap *GetBasemapPtr();
  static AppMap *GetAppmapPtr();
  static Mapinfo* GetMapinfoPtr();

private:
  MapLoader() = delete;
  static std::shared_ptr<VectorMap> CreateVectorMap();
  static std::shared_ptr<LinkMap> CreateLinkMap();
  static std::shared_ptr<AppMap> CreateAppMap();

  static std::shared_ptr<LinkMap> link_map_;
  static std::mutex link_map_mutex_;
  static std::shared_ptr<VectorMap> vector_map_;
  static std::mutex vector_map_mutex_;
  static std::shared_ptr<BaseMap> base_map_;
  static std::mutex base_map_mutex_;
  static std::shared_ptr<AppMap> app_map_;
  static std::mutex app_map_mutex_;
  static std::shared_ptr<Mapinfo> mapinfo_;
  static std::mutex mapinfo_mutex_;
};

}  // namespace map
}  // namespace acu

#endif
