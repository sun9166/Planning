
#include "map/map_loader/include/link_map.h"
#include "tinyxml2.h"
#include "common/base/log/include/log.h"
#include "map/map_loader/include/map_loader.h"

namespace acu {
namespace map {

std::vector<LinkArea> LinkMap::areas_;

bool isFileExist(const std::string file_name);

int LinkMap::LoadMapFromFile(const std::string &map_filename, 
        const std::string &wxb_xml_dir)
{
	auto mapinfo = acu::map::MapLoader::GetMapinfoPtr();
  if (!mapinfo->IsWxbMapEnabled()) {
    return true;
  }

  if (!isFileExist(map_filename)) {
    AERROR << "file (" << map_filename << ") is included in link_map.xml, but it isn't exist!";
    return false;
  }

  tinyxml2::XMLDocument document;
  if (document.LoadFile(map_filename.c_str()) != tinyxml2::XML_SUCCESS) {
      AERROR << "Failed to load " << map_filename;
      return false;
  }

  this->areas_.clear();

  tinyxml2::XMLElement *root_node = document.RootElement();
  if (root_node == nullptr) {
      return true;
  }

  tinyxml2::XMLElement *node;

  node = root_node->FirstChildElement();
  if (node == nullptr) {
      return true;
  }

  node = node->FirstChildElement();

  while (node != nullptr) {
      LinkArea link_area;

      std::string name = node->Attribute("name");

      link_area.file_name = name;
      link_area.area_id = node->Attribute("id");

      tinyxml2::XMLElement *road_node = node->FirstChildElement();
      while (road_node != nullptr) {
          LinkRoad road;
          road.road_id = road_node->Attribute("id");
          road.length = strtod(road_node->Attribute("length"), nullptr);
          road.type = road_node->Attribute("type");

          tinyxml2::XMLElement *link_node_tag = road_node->FirstChildElement();
          if (link_node_tag == nullptr) {
            road_node = road_node->NextSiblingElement();
            continue;
          }
          tinyxml2::XMLElement *link_node = link_node_tag->FirstChildElement();
          while (link_node != nullptr) {
              std::string value = link_node->Value();
              if (value == "predecessor") {
                road.predecessor.push_back(link_node->Attribute("elementId"));
              }
              if (value == "successor")
              {
                road.successor.push_back(link_node->Attribute("elementId"));
              }

              link_node = link_node->NextSiblingElement();
          }

          road.pgccs_set.clear();
          tinyxml2::XMLElement *points_node_tag = road_node->FirstChildElement();
          if (points_node_tag == nullptr) {
            road_node = road_node->NextSiblingElement();
            continue;
          }
          tinyxml2::XMLElement *points_node = points_node_tag->NextSiblingElement();
          if (points_node != nullptr) {
              tinyxml2::XMLElement *point_node = points_node->FirstChildElement();
              while (point_node != nullptr) {
              PointGCCS pgccs;
              pgccs.xg = strtod(point_node->Attribute("x"), nullptr);
              pgccs.yg = strtod(point_node->Attribute("y"), nullptr);
              pgccs.angle = strtod(point_node->Attribute("heading"), nullptr);
              road.pgccs_set.push_back(pgccs);

              point_node = point_node->NextSiblingElement();
              }
          }

          link_area.roads.push_back(road);

          road_node = road_node->NextSiblingElement();
      }

      XmlVectormap *xmlmap = new XmlVectormap();
      std::string wxb_file = wxb_xml_dir + name;
      if (xmlmap->LoadMapFromFile(wxb_file) != 0) {
          AERROR << "link_map failed to load " << wxb_file;
          return false;
      }

      link_area.map = xmlmap;

      Boundary boundary = xmlmap->GetAreaBoundary();

      link_area.left_down.xg = boundary.xg;
      link_area.left_down.yg = boundary.yg;
      link_area.right_top.xg = boundary.xg + boundary.length;
      link_area.right_top.yg = boundary.yg + boundary.width;

      this->areas_.push_back(link_area);

      node = node->NextSiblingElement();
  }

  return true;
}

bool LinkMap::GetVirtualMapLinkPoint(std::string road_id, PointGCCS& point)
{
  if (areas_.size() == 0) {
    return false;
  }

  for (auto link_area : areas_) {
    for (auto road : link_area.roads) {
      if (road.road_id == road_id) {
        if (road.pgccs_set.size() <= 0) {
          return false;
        }

        point = road.pgccs_set.at(0);
        return true;
      }
    }
  }

  return false;
}

bool LinkMap::GetVirtualMapRoadType(std::string road_id, std::string& type)
{
  if (areas_.size() == 0) {
    return false;
  }

  for (auto link_area : areas_) {
    for (auto road : link_area.roads) {
      if (road.road_id == road_id) {
        type = road.type;
        return true;
      }
    }
  }

  return false;
}

bool LinkMap::IsRoadInLinkmap(std::string road_id)
{
  if (areas_.size() == 0) {
    return false;
  }

  for (auto link_area : areas_) {
    for (auto road : link_area.roads) {
      if (road.road_id == road_id) {
        return true;
      }
    }
  }

  return false;
}

// return empty string if in structed map
std::string LinkMap::GetMapAreaIdByPoint(PointGCCS pgccs)
{
  if (areas_.size() == 0) {
    return "";
  }

  for (auto link_area : areas_) {
// ROS_INFO_STREAM("boundary left down (" << link_area.left_down.xg << ", " << link_area.left_down.yg << ")");
// ROS_INFO_STREAM("boundary right top (" << link_area.left_down.xg << ", " << link_area.left_down.yg << ")");
    if ((pgccs.xg > link_area.left_down.xg) && (pgccs.xg < link_area.right_top.xg) &&
        (pgccs.yg > link_area.left_down.yg) && (pgccs.yg < link_area.right_top.yg) ) {
          
      return link_area.area_id;
    }
  }

  return "";
}

// return empty string if in structed map
std::string LinkMap::GetMapAreaIdByPath(std::string line_id)
{
  if (areas_.size() == 0) {
    return "";
  }

	for (auto link_area : areas_) {
    if (nullptr != link_area.map) {
      if (nullptr != link_area.map->GetSegmentById(line_id)) {
        return link_area.area_id;
      }
    }
  }

  return "";
}

std::string LinkMap::GetMapAreaCenterRoad(std::string area_id)
{
  if (areas_.size() == 0) {
    return "";
  }

  for (auto link_area : areas_) {
    if (link_area.area_id == area_id) {
      for (auto road : link_area.roads) {
        if (road.type == "center") {
          return road.road_id;
        }
      }
    }
  }

  return "";
}

bool LinkMap::GetKeypointByRoad(std::string road, PointGCCS pgccs)
{
  return true;
}

// 1-structed map
// 2-unstructed map
int LinkMap::GetMapType(std::string map_id)
{
  if (areas_.size() == 0) {
    return 1;
  }

  for (auto link_area : areas_) {
    if (link_area.area_id == map_id) {
      return 2;
    }
  }

  return 1;
}

bool LinkMap::GetMapBoundary(std::string area_id, 
    PointGCCS& left_down, PointGCCS& right_top)
{
  if (areas_.size() == 0) {
    return false;
  }

  for (auto link_area : areas_) {
    if (link_area.area_id == area_id) {
      left_down = link_area.left_down;
      right_top = link_area.right_top;
      return true;
    }
  }

  return false;
}

XmlVectormap* LinkMap::GetWxbmapPtr(std::string area_id)
{
  if (areas_.size() == 0) {
    return nullptr;
  }

  for (auto link_area : areas_) {
    if (link_area.area_id == area_id) {
      return link_area.map;
    }
  }

  return nullptr;
}

XmlVectormap* LinkMap::GetWxbmapByNamePtr(std::string file_name) {
  if (areas_.size() == 0) {
    return nullptr;
  }

  for (auto link_area : areas_) {
    if (link_area.file_name == file_name) {
      return link_area.map;
    }
  }

  return nullptr;
}

std::vector<LinkArea> LinkMap::GetLinkAreas()
{
  return areas_;
}

}  // namespace map
}  // namespace acu
