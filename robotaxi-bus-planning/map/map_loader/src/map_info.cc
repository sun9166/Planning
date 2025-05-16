#include "map_info.h"
#include "common/base/log/include/acu_node.h"
#include "common/base/log/include/log.h"
#include "common/util/file.h"
#include "tinyxml2.h"
#include "config_schema.pb.h"

namespace acu {
namespace map {

using namespace acu::vectormap;

int MyExec(const char *cmd, std::vector<std::string> &resvec) {
  resvec.clear();
  FILE *pp = popen(cmd, "r");
  if (!pp) {
    return -1;
  }
  char tmp[1024];
  while (fgets(tmp, sizeof(tmp), pp) != NULL) {
    if (tmp[strlen(tmp) - 1] == '\n') {
      tmp[strlen(tmp) - 1] = '\0';
    }
    resvec.push_back(tmp);
  }
  pclose(pp);
  return resvec.size();
}

std::string GetProjectWS() {
  std::string res = "";
  std::vector<std::string> resvec;
  std::string cmd = "rospack find map_loader";
  MyExec(cmd.c_str(), resvec);
  if (resvec.size() < 1){
    AERROR << cmd << "execute failure!!!";
    return res;
  }
  res = resvec[0];
  if (res.back() != '/') res += '/';
  return res + "../../";
}

bool Mapinfo::isFileExist(const std::string file_name) {
  if (file_name == "")
    return false;
  if (access(file_name.c_str(), F_OK) == 0)
    return true;
  return false;
}

std::string Mapinfo::GetWorkspace() {
  return mheader_.name;
}

std::string Mapinfo::GetMapsBasePasth() {
  //std::string path = GetProjectWS() + "map_file/";
  std::string path = "/map/map/";
  path += "map_file/";     //huangchuo
  return path;
}

std::string Mapinfo::GetMapPath() {
  std::string path = GetMapsBasePasth() + mheader_.name;
  if (path.back() != '/') path += '/';
  return path;
}

Mapinfo::Mapinfo() {
  mpheader_.image_matrix_size = 0;
  mpheader_.basemap_image_size = 0;
  mpheader_.basemap_resolution_ratio = 0;
  mpheader_.image_pixel_chars = 0;
  mpheader_.basemap_filename_level = 0;
  mpheader_.basemap_dir = "";
  mpheader_.link_map_dir = "";
  mpheader_.link_map_file = "";
  mpheader_.vector_map_dir = "";
  mpheader_.vector_map_file = "";
  mpheader_.app_map_dir = "";
  mpheader_.app_map_file = "";
  mpheader_.wxb_map_dir = "";
  mpheader_.VectormapEnabled = false;
  mpheader_.WxbmapEnabled = false;
}

Mapinfo::~Mapinfo() { }

/******************************************************
 * 地图有效性校验
 * 
 * 返回值：
 *  0 - 校验通过
 *  1 - 地图不存在
 *  2 - 地图文件内容校验失败
 * ****************************************************/
int Mapinfo::MapVerify(std::string& err_msg) {

  if (mheader_.name != miheader_.name) {
    err_msg = "mheader_.name != miheader_.name";
    return 2;
  }


  // if (mheader_.date != miheader_.date) {
  //   err_msg = "mheader_.date != miheader_.date";
  //   return 2;
  // }

  err_msg = "OK";
  return 0;
}

std::vector<std::string> Mapinfo::GetMapDirList(const std::string& dir_name)
{
    std::vector<std::string> list;
    auto dir = opendir(dir_name.c_str());
    struct dirent* ent;
    if (dir) {
        while ((ent = readdir(dir)) != NULL) {
            if (0 == strcmp(ent->d_name, "..") || 0 == strcmp(ent->d_name, ".")) {
                continue;
            }
            
            auto path = std::string(dir_name) + "/" + std::string(ent->d_name) + "/";
            if (opendir(path.c_str()) != NULL) {
              list.push_back(path);
            }
        }
        closedir(dir);
    }
    return list;
}

bool Mapinfo::GetCurrentMap(MapItem& map_item)
{
  tinyxml2::XMLDocument doc;
  std::string path = GetMapsBasePasth() + "map.xml";

  if (doc.LoadFile(path.c_str()) != 0) {
    AERROR << "load map.xml failed " << path;
    return false;
  }

  auto node = doc.RootElement()->FirstChildElement();
  std::string node_value = node->Value();
  if (std::string(node_value) != "header") {
    AERROR << "GetCurrentMap, Could not find header node";
    return false;
  }

  map_item.name = node->Attribute("default_name");
  if (map_item.name == "") {
    AERROR << "GetCurrentMap, get default_name failed" << path;
    return false;
  }
  map_item.date = node->Attribute("date");
  if (map_item.date == "") {
    AERROR << "GetCurrentMap, get date failed" << path;
    return false;
  }

  return true;
}

bool Mapinfo::SetCurrentMap(MapItem& map_item)
{
  // AINFO << "SetCurrentMap map_item.name=" << map_item.name << ", map_item.date=" << map_item.date;

  tinyxml2::XMLDocument doc;
  std::string path = GetMapsBasePasth() + "map.xml";

  if (doc.LoadFile(path.c_str()) != 0) {
    AERROR << "load map.xml failed " << path;
    return false;
  }

  auto node = doc.RootElement()->FirstChildElement();
  std::string node_value = node->Value();
  if (std::string(node_value) != "header") {
    AERROR << "SetCurrentMap, Could not find header node";
    return false;
  }

  node->SetAttribute("default_name", map_item.name.c_str());
  node->SetAttribute("date", map_item.date.c_str());

  doc.SaveFile(path.c_str());

  return true;
}

bool Mapinfo::GetAvailableMaps(std::vector<MapItem>& maps)
{
  std::vector<std::string> dir_list = GetMapDirList(GetMapsBasePasth());
  tinyxml2::XMLDocument doc;
  maps.clear();
  MapItem map_item;

  for (auto dir_name : dir_list) {
    auto path = dir_name + "mapinfo.xml";
    if (doc.LoadFile(path.c_str()) != 0) {
      AWARN << "load mapinfo.xml failed, " << path;
      // this is not as a valid map package
      continue;
    }

    auto root_node = doc.RootElement(); // header
    if (nullptr == root_node) {
      AWARN << "map_item, get root_node failed, " << path;
      continue;
    }

    auto node = root_node->FirstChildElement(); // header
    if (nullptr == node) {
      AWARN << "map_item, get header failed, " << path;
      continue;
    }

    auto node_value = node->Value();
    if (std::string(node_value) != "header") {
      AWARN << "map_item, Could not find header node, " << path << "node value is " << node_value << ".";
      continue;
    }

    map_item.name = node->Attribute("name");
    if (map_item.name == "") {
      AWARN << "map_item, get name failed, " << path;
      continue;
    }

    map_item.date = node->Attribute("date");
    if (map_item.date == "") {
      AWARN << "map_item, get date failed, " << path;
      continue;
    }

    // AINFO << "map_item.name=" << map_item.name << ", map_item.date=" << map_item.date;
    maps.push_back(map_item);
  }

  return true;
}

int Mapinfo::LoadMapInfo() {
  tinyxml2::XMLDocument doc;
  std::string path = GetMapsBasePasth() + "map.xml";

  if (doc.LoadFile(path.c_str()) != 0) {
    AERROR << "load map.xml failed " << path;
    return -1;
  }

  auto node = doc.RootElement()->FirstChildElement();
  std::string node_value = node->Value();
  if (std::string(node_value) != "header") {
    AERROR << "mheader, Could not find header node, node value is " << node_value;
    return -1;
  }

  mheader_.name = node->Attribute("default_name");
  if (mheader_.name == "") {
    AERROR << "mheader, get default_name failed" << path;
    return -1;
  }
  mheader_.date = node->Attribute("date");
  if (mheader_.date == "") {
    AERROR << "mheader, get date failed" << path;
    return -1;
  }
  
  if (-1 == GetParams()) {
    AERROR << "mpheader, GetParams failed" << path;
    return -1;
  }

  path = GetMapPath() + "mapinfo.xml";
  if (doc.LoadFile(path.c_str()) != 0) {
    AERROR << "load mapinfo.xml failed " << path;
    return -1;
  }

  node = doc.RootElement()->FirstChildElement(); // header
  if (nullptr == node) {
    AERROR << "miheader, get header failed" << path;
    return -1;
  }

  node_value = node->Value();
  if (node_value != "header") {
    AERROR << "miheader, Could not find header node";
    return -1;
  }

  miheader_.name = node->Attribute("name");
  if (miheader_.name == "") {
    AERROR << "miheader, get name failed" << path;
    return -1;
  }
  miheader_.version = node->Attribute("version");
  if (miheader_.version == "") {
    AERROR << "miheader, get version failed" << path;
    return -1;
  }

  miheader_.type = "standard";
  auto result = node->Attribute("type");
  if (nullptr != result) {
    miheader_.type = result;
  }

  miheader_.describe = node->Attribute("describe");

  miheader_.date = node->Attribute("date");
  if (miheader_.date == "") {
    AERROR << "miheader, get date failed" << path;
    return -1;
  }

  node = node->FirstChildElement(); // move_origin
  if ((nullptr == node) && (node->Value() != "move_origin")) {
    AERROR << "miheader, get move_origin failed" << path;
    return -1;
  }

    miheader_.zone = atoi(node->Attribute("zone"));
    miheader_.multpcd_enable = atoi(node->Attribute("multpcd_enable"));

  auto centerpoint_node = node->FirstChildElement();
  if (nullptr == centerpoint_node) {
    AERROR << "miheader, get centerPoint failed" << path;
    return -1;
  }

  miheader_.center_point.xg = strtod(centerpoint_node->Attribute("x"), nullptr);
  miheader_.center_point.yg = strtod(centerpoint_node->Attribute("y"), nullptr);
  miheader_.center_point.zg = strtod(centerpoint_node->Attribute("z"), nullptr);

  if (miheader_.center_point.xg < 10000) {
    // this is GPS point, transfer it to GCCS
    GeoTool geotool;
    PointGPS gps1(miheader_.center_point.yg, miheader_.center_point.xg, 0.0);
    PointGCCS gccs1;
    geotool.GPS2GCCS(gps1, gccs1);

    miheader_.center_point.xg = gccs1.xg;
    miheader_.center_point.yg = gccs1.yg;
    miheader_.center_point.zg = 0.0;
    AINFO << std::fixed <<"2 miheader_.center_point: gps (" << gps1.lat << ", " << gps1.lon << ")";
  }

  AINFO << std::fixed << "2 miheader_.center_point: gccs (" 
                      << miheader_.center_point.xg << ", " 
                      << miheader_.center_point.yg << ", " 
                      << miheader_.center_point.zg << ")";
  miheader_.center_point.angle = 0.0;

  node = node->NextSiblingElement(); // boundary
  if ((nullptr == node) && (node->Value() != "boundary")) {
    AERROR << "miheader, get boundary failed" << path;
    return -1;
  }

  GeoTool geo_tool;
  PointGPS gps;

  gps.lat = strtod(node->Attribute("ldx"), nullptr);
  gps.lon = strtod(node->Attribute("ldy"), nullptr);

  if (gps.lat < 10000) {
    // this is GPS point, transfer it to GCCS
    geo_tool.GPS2GCCS(gps, miheader_.left_down);
    AINFO << std::fixed <<"3 miheader_.left_down: gps (" << gps.lat << ", " << gps.lon << ")";
  } else {
    miheader_.left_down.xg = gps.lat;
    miheader_.left_down.yg = gps.lon;
  }
  AINFO << std::fixed <<"3 miheader_.left_down: gccs (" << miheader_.left_down.xg << ", " << miheader_.left_down.yg << ")";

  double length = strtod(node->Attribute("length"), nullptr);
  double width = strtod(node->Attribute("width"), nullptr);

  miheader_.right_top.xg = miheader_.left_down.xg + length;
  miheader_.right_top.yg = miheader_.left_down.yg + width;

  std::string err_msg;
  int ret = MapVerify(err_msg);
  if (0 != ret) {
    AERROR << "MapVerify (" << ret << "): " << err_msg;
    return -1;
  }
  
  return 0;
}

bool Mapinfo::GetMapBoundary(PointGCCS& left_down, PointGCCS& right_top)
{
    left_down = miheader_.left_down;
    right_top = miheader_.right_top;
    return true;
}
int Mapinfo::GetParams() {
  acu::common::config::MultiModelConfigProto configs;
  std::string map_param = GetMapPath() + "map_param.conf";
  std::cout << "map config file: " << map_param << std::endl;
  if (!acu::common::util::GetProtoFromFile(map_param, &configs)) {
    AERROR << "################# failed to load map config file: " << map_param;
    return -1;
  }

  for (const auto &config : configs.model_configs()) {
    if (config.name() == "basemap") {
      auto &params = config.string_params();
      for (const auto &param : params) {
        mpheader_.basemap_dir =
            param.name() == "basemap_dir" ? param.value() : mpheader_.basemap_dir;
      }
      auto &parami = config.integer_params();
      for (const auto &param : parami) {
        mpheader_.basemap_image_size = param.name() == "basemap_image_size"
                                       ? param.value()
                                       : mpheader_.basemap_image_size;
        mpheader_.image_matrix_size = param.name() == "image_matrix_size"
                                      ? param.value()
                                      : mpheader_.image_matrix_size;
        mpheader_.basemap_filename_level = param.name() == "basemap_filename_level"
                                           ? param.value()
                                           : mpheader_.basemap_filename_level;
      }
      auto &paramd = config.double_params();
      for (const auto &param : paramd) {
        mpheader_.basemap_resolution_ratio =
            param.name() == "basemap_resolution_ratio"
                ? param.value()
                : mpheader_.basemap_resolution_ratio;
        mpheader_.image_pixel_chars =
            param.name() == "image_pixel_chars"
                ? param.value()
                : mpheader_.image_pixel_chars;
      }
    } else if (config.name() == "vectormap") {
      mpheader_.VectormapEnabled = true;
      auto &params = config.string_params();
      for (const auto &param : params) {
        mpheader_.vector_map_dir =
            param.name() == "vector_map_dir" ? param.value() : mpheader_.vector_map_dir;
        mpheader_.vector_map_file =
            param.name() == "vector_map_file" ? param.value() : mpheader_.vector_map_file;
      }

      // check if the map file exist
      std::string file_name = GetMapPath() + mpheader_.vector_map_dir + 
            mpheader_.vector_map_file  + ".xml";
      if (!isFileExist(file_name)) {
        mpheader_.VectormapEnabled = false;
      }
    } else if (config.name() == "wxbmap") {
      auto &params = config.string_params();
      for (const auto &param : params) {
        mpheader_.wxb_map_dir =
            param.name() == "wxb_map_dir" ? param.value() : mpheader_.wxb_map_dir;
      }
    } else if (config.name() == "linkmap") {
      mpheader_.WxbmapEnabled = true;
      auto &params = config.string_params();
      for (const auto &param : params) {
        mpheader_.link_map_file =
            param.name() == "link_map_file" ? param.value() : mpheader_.link_map_file;
        mpheader_.link_map_dir =
            param.name() == "link_map_dir" ? param.value() : mpheader_.link_map_dir;
      }
    } else if (config.name() == "appmap") {
      auto &params = config.string_params();
      for (const auto &param : params) {
        mpheader_.app_map_file =
            param.name() == "app_map_file" ? param.value() : mpheader_.app_map_file;
        mpheader_.app_map_dir =
            param.name() == "app_map_dir" ? param.value() : mpheader_.app_map_dir;
      }
    }
  }
  return 0;
}

// vectorS Mapinfo::GetWorkspaceByGps(double lon, double lat) {
//   vectorS vs;
//   PointGPS pgps = {lon, lat, 0};
//   PointGCCS pgccs;
//   int re = GeoTransform(pgps, pgccs);
//   if (re < 0) {
//     std::cout << "error lonlat!!!!" << std::endl;
//     vs;
//   }
//   auto dirs = ListSubDirectories(GetProjectWS() + default_map_dir);
//   vs = FindGpsWorkspace(pgccs.xg, pgccs.yg, dirs);
//   return vs;
// }
// vectorS Mapinfo::GetWorkspaces() {
//   return ListSubDirectories(GetProjectWS() + default_map_dir);
// }
// std::string Mapinfo::GetDefaultWorkspace() { 
//     return mheader_.name; 
// }


// vectorS Mapinfo::ListSubDirectories(const std::string directory_path) {
//   vectorS result;
//   DIR* directory = opendir(directory_path.c_str());
//   if (directory == nullptr) {
//     std::cout << "Cannot open directory " << directory_path << std::endl;
//     return result;
//   }
//   struct dirent* entry;
//   while ((entry = readdir(directory)) != nullptr) {
//     // skip directory_path/. and directory_path/..
//     if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) {
//       continue;
//     }

//     if (entry->d_type == DT_DIR) {
//       result.emplace_back(entry->d_name);
//     }
//   }
//   closedir(directory);
//   return result;
// }

// miHeader Mapinfo::GetCurrentMapxmlinfoHeader(const std::string& map_filename) {
//   std::string name =
//     GetProjectWS() + default_map_dir + map_filename + default_mapinfo_file;
//   int re = LoadMapinfoxmlFromFile(name);
//   if (re >= 0)  return miheader_;
//   else {
//     std::cout << "Load " << name << " is error!!!" << std::endl;
//     return miheader_;
//   }
// }

// vectorS Mapinfo::FindGpsWorkspace(double x, double y, vectorS& dirs) {
//   vectorS results;
//   for (auto& dir : dirs) {
//     std::string name =
//       GetProjectWS() + default_map_dir + dir + default_mapinfo_file;
//     int re = LoadMapinfoxmlFromFile(name);
//     if (re >= 0) {
//       PointGPS pgps = {miheader_.boundary.ldy, miheader_.boundary.ldx, 0};
//       PointGCCS pgccs;
//       int re = GeoTransform(pgps, pgccs);
//       if (re < 0) continue;
//       auto dx = x - pgccs.xg;
//       auto dy = y - pgccs.yg;
//       if (dx > 0 && dx < miheader_.boundary.length && dy > 0 &&
//           dy < miheader_.boundary.width)
//         results.push_back(dir);
//     }
//   }
//   return results;
// }

}
}
