#ifndef APP_MAP_H_
#define APP_MAP_H_

#include <memory>
#include <mutex>
#include <string>

#include "map/map_loader/include/map_info.h"
#include "Point32.h"
#include "Polygon.h"


namespace acu {
namespace map {

struct FunctionPoint {
  geometry_msgs::Point32 point;
  double heading;
  std::string type;
  std::string name;
};

struct FunctionArea {
  geometry_msgs::Polygon polygon;
  std::string type;
  std::string id;
  std::string name;
  std::string signal;
  std::string canid;
  std::string bit;
};

class AppMap {
public:
  int LoadMapFromFile(const std::string &map_filename);
  static std::vector<FunctionArea> GetFunctionAreaByPoint(geometry_msgs::Point32 point);
  static std::vector<FunctionPoint>& GetFunctionPoints();
  static std::vector<FunctionArea>& GetFunctionAreas();

private:
  static bool IsInPolygon(FunctionArea area, geometry_msgs::Point32 point);
  bool isFileExist(const std::string file_name);

private:
  static std::vector<FunctionPoint> points_;
  static std::vector<FunctionArea> areas_;
};

}  // namespace map
}  // namespace acu

#endif
