/*
idriverplus
create 20181122 lbh
=========================================================================*/

#ifndef ACU1_XMLSTRUCT_MAPINFO_H_
#define ACU1_XMLSTRUCT_MAPINFO_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "xmlstruct_common.h"
namespace acu {
namespace xmlmap {

// mapinfo.xml
struct miHeader {
  std::string name;
  std::string version;
  std::string describe;
  std::string date;
  Boundary boundary;
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
struct mConf {
  int image_matrix_size;
  int basemap_image_size;
  double basemap_resolution_ratio;
  double image_pixel_chars;
  int basemap_filename_level;
  std::string basemap_dir;
  Nameset nameset;
};

// mapinfoxml
struct MapinfoXml {
  miHeader header;
};

// mapxml
struct MapXml {
  mHeader header;
};
}
}

#endif
