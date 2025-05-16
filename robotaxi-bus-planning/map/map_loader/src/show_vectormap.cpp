#include "map/map_loader/include/map_loader.h"
#include "map/map_loader/include/show_vectormap.h"

void PointGps::convert() {
  GeoTool geo_tool;
  PointGPS gps;
  PointGCCS gccs;
  gps.lat = lat;
  gps.lon = lon;
  geo_tool.GPS2GCCS(gps, gccs);

  x = gccs.xg;
  y = gccs.yg;
}

void PointGps::pointOff(double xg, double yg)
{
  x -= xg;
  y -= yg;
}

// void PointGps::GetXgYg(double _x, double _y) {
//   x = _x;
//   y = _y;
// }

vector<std::map<std::string , std::vector<std::string>>*> groad_info_map_vec;
vector<std::map<std::string, MissionLaneInfo>*> glane_info_map_vec;
std::map<std::string , std::vector<std::string>> *groad_info_map = nullptr;
std::map<std::string, MissionLaneInfo> *glane_info_map = nullptr;

string map_version = "";
bool is_path_point = false;
std::string current_road, current_lane;

PointGps pgps_init;

std::vector<std::string > select_road;
void ParseSubNode(tinyxml2::XMLElement *xml_node) {
  auto node = xml_node->FirstChildElement();

  if (node) {
    std::string node_value = node->Value();
    if (node_value == "header") {
      map_version = node->Attribute("version");
      AINFO << "header: " << map_version;

      if (map_version == "1.4") {
        double north = strtod(node->Attribute("north"), nullptr);
        double south = strtod(node->Attribute("south"), nullptr);
        double east = strtod(node->Attribute("east"), nullptr);
        double west = strtod(node->Attribute("west"), nullptr);
        pgps_init.lon = (east + west)/2;
        pgps_init.lat = (north + south)/2;
        pgps_init.convert();
        AINFO << "center point gps: (" << fixed<<setprecision(5) << pgps_init.lon << ", " << 
            fixed<<setprecision(5) << pgps_init.lat << ") ";
        AINFO << "center point gccs: (" << pgps_init.x << ", " << pgps_init.y << ") ";
      } else {
        auto head_node = node->FirstChildElement()->FirstChildElement();
        pgps_init.x = strtod(head_node->Attribute("x"), nullptr);
        pgps_init.y = strtod(head_node->Attribute("y"), nullptr);
        AINFO << "center point gccs: (" << pgps_init.x << ", " << pgps_init.y << ") ";
      }
    }
    if (map_version == "1.4") {
      if (node_value == "road") {
        // AINFO << "road:" << node->Attribute("id");
        is_path_point = false;
        current_road = node->Attribute("id");
      }
      if (node_value == "lane") {
        current_lane = node->Attribute("uid") ;
        // AINFO << "lane:" << node->Attribute("uid");
        is_path_point = true;
        (*groad_info_map)[current_road].push_back(current_lane);
      }
    } else {
      if (node_value == "segment") {
        AINFO << "segment: " << node->Attribute("id");
        is_path_point = true;
        current_road = node->Attribute("id");
      }
    }

    if (node_value == "geometry" && is_path_point) {
      (*glane_info_map)[current_lane].length = atoi(node->Attribute("length"));
      (*glane_info_map)[current_lane].road_id = current_road;
      (*glane_info_map)[current_lane].lane_id = current_lane;
    }

    if (node_value == "point" && is_path_point) {
      if (map_version == "1.4") {
        PointGps point_gps;
        point_gps.lon = strtod(node->Attribute("x"), nullptr);
        point_gps.lat = strtod(node->Attribute("y"), nullptr);
        point_gps.convert();
        PointGCCS point;
        point.xg = point_gps.x;
        point.yg = point_gps.y;
        point.angle = 0.0;
        (*glane_info_map)[current_lane].gccs_point.push_back(point);

        static bool first_point = true;
        if (first_point) {
          pgps_init = point_gps;
          first_point = false;
        }
      } else {
        PointGCCS point;
        point.xg = strtod(node->Attribute("x"), nullptr) + pgps_init.x;
        point.yg = strtod(node->Attribute("y"), nullptr) + pgps_init.y;
        point.angle = 0.0;
        (*glane_info_map)[current_road].gccs_point.push_back(point);
      }
    }

    ParseSubNode(node);
  }

  node = xml_node->NextSiblingElement();
  if (node) {
    std::string node_value = node->Value();
    if (map_version == "1.4") {
      if (node_value == "road") {
        // AINFO << "road:" << node->Attribute("id");
        is_path_point = false;
        current_road = node->Attribute("id");
      }
      if (node_value == "lane") {
        current_lane = node->Attribute("uid") ;
        // AINFO << "lane:" << node->Attribute("uid");
        is_path_point = true;
        (*groad_info_map)[current_road].push_back(current_lane);

      }
    } else {
      if (node_value == "segment") {
        // AINFO << "segment: " << node->Attribute("id");
        is_path_point = true;
        current_road = node->Attribute("id");
      }
    }
    if (node_value == "geometry" && is_path_point) {
      (*glane_info_map)[current_lane].length = atoi(node->Attribute("length"));
    }

    if (node_value == "point" && is_path_point) {
      if (map_version == "1.4") {
        PointGps point_gps;
        point_gps.lon = strtod(node->Attribute("x"), nullptr);
        point_gps.lat = strtod(node->Attribute("y"), nullptr);
        point_gps.convert();
        PointGCCS point;
        point.xg = point_gps.x;
        point.yg = point_gps.y;
        point.angle = 0.0;
        (*glane_info_map)[current_lane].gccs_point.push_back(point);
        (*glane_info_map)[current_lane].x = node->Attribute("x");
        (*glane_info_map)[current_lane].y = node->Attribute("y");
      } else {
        PointGCCS point;
        point.xg = strtod(node->Attribute("x"), nullptr) + pgps_init.x;
        point.yg = strtod(node->Attribute("y"), nullptr) + pgps_init.y;
        point.angle = 0.0;
        (*glane_info_map)[current_road].gccs_point.push_back(point);
        (*glane_info_map)[current_lane].x = node->Attribute("x");
        (*glane_info_map)[current_lane].y = node->Attribute("y");
      }
    }
    ParseSubNode(node);
  }
}


bool isFileExist(const std::string file_name) {
  if (file_name == "")
    return false;
  if (access(file_name.c_str(), F_OK) == 0)
    return true;
  return false;
}

int LoadVectorFile(PointGCCS &car_gccs) {

	auto mapinfo = acu::map::MapLoader::GetMapinfoPtr();
  auto mpheader = mapinfo->GetMapParamHeader();

  std::string path;
  std::string file;
  XMLDocument doc;
  XMLElement* root;

  if (mpheader.VectormapEnabled) {
    path = mapinfo->GetMapPath() + mpheader.vector_map_dir;
    file = path + mpheader.vector_map_file + ".xml";

    if (isFileExist(file)) {
      if (doc.LoadFile(file.c_str()) != 0)
      {
        AERROR << "load apollo file failed " << file;
        return -1;
      }

      glane_info_map = new std::map<std::string, MissionLaneInfo>;
      glane_info_map_vec.push_back(glane_info_map);
      groad_info_map = new std::map<std::string , std::vector<std::string>>;
      groad_info_map_vec.push_back(groad_info_map);

      root = doc.RootElement();
      ParseSubNode(root);

      glane_info_map = NULL;
      groad_info_map = NULL;
    }
  }

  if (mpheader.WxbmapEnabled) {
    acu::map::LinkMap* link_map = acu::map::MapLoader::GetLinkmapPtr();
    for (auto area : link_map->GetLinkAreas()) {
      path = mapinfo->GetMapPath() + mpheader.wxb_map_dir;
      file = path + area.file_name;

      if (isFileExist(file)) {
        if (doc.LoadFile(file.c_str()) != 0)
        {
          AERROR << "load wxb file failed " << file;
          return -1;
        }

        glane_info_map = new std::map<std::string, MissionLaneInfo>;
        glane_info_map_vec.push_back(glane_info_map);
        groad_info_map = new std::map<std::string, std::vector<std::string>>;
        groad_info_map_vec.push_back(groad_info_map);

        root = doc.RootElement();
        ParseSubNode(root);

        glane_info_map = NULL;
        groad_info_map = NULL;
      }
    }

    file = mapinfo->GetMapPath() + mpheader.link_map_dir + mpheader.link_map_file;

    if (doc.LoadFile(file.c_str()) != 0)
    {
      AERROR << "load link file failed " << file;
      return -1;
    }

    glane_info_map = new std::map<std::string, MissionLaneInfo>;
    glane_info_map_vec.push_back(glane_info_map);
    groad_info_map = new std::map<std::string , std::vector<std::string>>;
    groad_info_map_vec.push_back(groad_info_map);

    root = doc.RootElement();

    auto node = root->FirstChildElement()->FirstChildElement();

    while (node != nullptr) { // area
      tinyxml2::XMLElement *road_node = node->FirstChildElement();
      while (road_node != nullptr) { // road
        current_lane = road_node->Attribute("id");
        tinyxml2::XMLElement *pointset_node = road_node->FirstChildElement()->NextSiblingElement();
        if (pointset_node != nullptr) {
          tinyxml2::XMLElement *point_node = pointset_node->FirstChildElement();
          while (point_node != nullptr) { // point
            PointGCCS point;
            point.xg = strtod(point_node->Attribute("x"), nullptr);
            point.yg = strtod(point_node->Attribute("y"), nullptr);
            point.angle = 0.0;
            (*glane_info_map)[current_road].gccs_point.push_back(point);

            point_node = point_node->NextSiblingElement();
          }
        }

        road_node = road_node->NextSiblingElement();
      }

      node = node->NextSiblingElement();
    }

    glane_info_map = NULL;
    groad_info_map = NULL;
  }
  car_gccs.xg = pgps_init.x;
  car_gccs.yg = pgps_init.y;
  car_gccs.angle = 0.0;

  return 0;
}
