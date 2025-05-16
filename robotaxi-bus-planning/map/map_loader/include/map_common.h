
#ifndef MAP_COMMON_H_
#define MAP_COMMON_H_

#include <dirent.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "map/vectormap/include/geotool.h"

namespace acu {
namespace map {

using namespace acu::vectormap;
using vectorS = std::vector<std::string>;

struct MapItem {
  std::string name;
  std::string date;
};

// mapinfo.xml
struct miHeader {
  std::string name;
  std::string version;
  std::string type;
  std::string describe;
  std::string date;
  int zone;
  int multpcd_enable;
  PointGCCS center_point;
  PointGCCS left_down;
  PointGCCS right_top;
};

// map.xml
typedef std::vector<std::string> Nameset;
struct mHeader {
  std::string name;
  std::string date;
  Nameset nameset;
};

// map_para.conf
typedef std::vector<std::string> Nameset;
struct mpHeader {
  std::string app_map_dir;
  std::string app_map_file;
  std::string vector_map_dir;
  std::string vector_map_file;
  std::string link_map_dir;
  std::string link_map_file;
  std::string wxb_map_dir;
  int image_matrix_size;
  int basemap_image_size;
  double basemap_resolution_ratio;
  double image_pixel_chars;
  int basemap_filename_level;
  std::string basemap_dir;
  bool VectormapEnabled;
  bool WxbmapEnabled;

  Nameset nameset;
};

}
}
#endif
