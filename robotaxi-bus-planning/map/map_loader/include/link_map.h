#ifndef LINK_MAP_H_
#define LINK_MAP_H_

#include <memory>
#include <mutex>
#include <string>

#include "map/vectormap/include/geotool.h"
#include "map/xmlmap/include/xmlmap.h"
#include "map/map_loader/include/map_info.h"

using namespace acu::xmlmap;
using namespace acu::vectormap;

namespace acu {
namespace map {

struct LinkRoad {
  std::string road_id;
  double length;
  std::string type;
  std::vector<std::string> predecessor;
  std::vector<std::string> successor;
  std::vector<PointGCCS> pgccs_set;
};

struct LinkArea {
  std::string file_name;
  std::string area_id;
  std::vector<LinkRoad> roads;
  XmlVectormap* map;
  PointGCCS left_down;
  PointGCCS right_top;
};

class LinkMap {
public:
  int LoadMapFromFile(const std::string &map_filename, const std::string &wxb_xml_dir);
  static std::string GetMapAreaIdByPoint(PointGCCS pgccs);
  static std::string GetMapAreaCenterRoad(std::string area_id);
  static std::string GetMapAreaIdByPath(std::string line_id);
  static bool GetKeypointByRoad(std::string road, PointGCCS pgccs);
  static int GetMapType(std::string map_id);
  static bool GetVirtualMapLinkPoint(std::string road_id, PointGCCS& point);
  static bool GetVirtualMapRoadType(std::string road_id, std::string& type);
  static bool IsRoadInLinkmap(std::string road_id);
  static bool GetMapBoundary(std::string area_id, 
        PointGCCS& left_down, PointGCCS& right_top);
  static XmlVectormap* GetWxbmapByNamePtr(std::string file_name);
  static XmlVectormap* GetWxbmapPtr(std::string area_id);
  static std::vector<LinkArea> GetLinkAreas();

private:
  static std::vector<LinkArea> areas_;
};

}  // namespace map
}  // namespace acu

#endif
