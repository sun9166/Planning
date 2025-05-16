
#ifndef MAP_INFO_H_
#define MAP_INFO_H_

#include <dirent.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "map/map_loader/include/map_common.h"

namespace acu {
namespace map {

using namespace acu::vectormap;

class Mapinfo {
 public:
  Mapinfo();
  ~Mapinfo();
  int LoadMapInfo();
  std::string GetWorkspace();
  std::string GetMapsBasePasth();
  std::string GetMapPath();
  miHeader GetMapinfoHeader() { return miheader_; }
  mpHeader GetMapParamHeader() { return mpheader_; }
  bool IsVectorMapEnabled() { return mpheader_.VectormapEnabled; }
  bool IsWxbMapEnabled() { return mpheader_.WxbmapEnabled; }
  mHeader GetMapHeader() { return mheader_; }
  PointGCCS GetCenterPoint() const { return miheader_.center_point; }
  bool GetMapBoundary(PointGCCS& left_down, PointGCCS& right_top);
  bool GetCurrentMap(MapItem& map_item);
  bool SetCurrentMap(MapItem& map_item);
  bool GetAvailableMaps(std::vector<MapItem>& maps);

 private:
  miHeader miheader_; // mapinfo.xml
  mpHeader mpheader_; // map_param.conf
  mHeader mheader_; // map.xml

  int GetParams();
  bool isFileExist(const std::string file_name);
  int MapVerify(std::string& err_msg);
  std::vector<std::string> GetMapDirList(const std::string& dir_name);
};

}
}

#endif
